/*
 * One Arduino ino file of freePEMF firmware.
 *
 * Chris Czoba (c) krzysiek@biotronika.pl
 * See: biotronics.eu or biotronika.pl (for Polish)
 *
 * New software version running bioZAP 2018-10-21
 * See: https://biotronika.pl/sites/default/files/2018-10/bioZAP%202018-10-21%20EN.pdf
 */

//#define SERIAL_DEBUG     //Uncomment this line for debug purpose
//#define NO_CHECK_BATTERY //Uncomment this line for debug purpose

#define HRDW_VER "NANO 4.2"
#define SOFT_VER "2018-11-02"

#include <EEPROM.h>

//Pin definition
#define coilPin 5      // Coil driver IRF540
#define powerPin 4     // Power relay
#define relayPin 9     // Direction relay
#define buzzPin 10     // Buzzer
#define btnPin 3       // Power On-Off / Pause / Change program button 
#define redPin 12      // Red LED
#define greenPin 11    // Green LED
#define hrmPin 2       // Biofeedback HR meter on 3th plug pin.

//Battery staff
#define batPin PIN_A7                 // Analog-in battery level
#define BATTERY_VOLTAGE_RATIO 0.153   // include 10k/4,7k resistor voltage divider. 5V*(10k+4,7k)/4,7k = 0,0153 (x10)
#define MIN_BATTERY_LEVEL 90          // 90 means 9.0 V  (x10), less then that turn off
#define USB_POWER_SUPPLY_LEVEL 65     // Maximum USB voltage level means 6.5V


//bioZAP
#define WELCOME_SCR "Free BIOzap interpreter welcome! See http://biotronics.eu"
#define PROGRAM_SIZE 1000   // Maximum program size
#define PROGRAM_BUFFER 500  // SRAM buffer size, used for script loading
#define MAX_CMD_PARAMS 4    // Count of command parameters
#define LCD_SCREEN_LINE -1  // LCD user line number, -1 = no LCD
#define MIN_FREQ_OUT 1      //  0.01 Hz
#define MAX_FREQ_OUT 6100   // 61.00 Hz for software function
#define MAX_FREQ_TIMER 1600000	// 16kHz dla timera
#define SCAN_STEPS 20       // For scan function purpose - default steps
#define MAX_LABELS 9        // jump labels maximum

//TODO delete
#define XON 17  //0x11
#define XOFF 19 //0x13


//bioZAP
String inputString = "";                // a string to hold incoming serial data
String line;
String param[MAX_CMD_PARAMS];           // param[0] = cmd name
boolean stringComplete = false;         // whether the string is complete
boolean memComplete = false;
unsigned long lastFreq = MIN_FREQ_OUT;  // Uses scan function
byte pwm = 50;							// Duty cycle of pulse width modulation: 1-99 %
int minBatteryLevel = 0; 
boolean Xoff = false;

//use global variables locally, for saving RAM memory
byte b;
int i;
long l;
unsigned long ul;


//bioZAP jump & labels
int labelPointer[MAX_LABELS+1];  		// Next line beginning address of label
unsigned int labelLoops[MAX_LABELS+1];  // Number of left jump loops
int adr=0;								// Script interpreter pointer


const unsigned long checkDeltaBatteryIncreasingVoltageTime = 600000UL;  // During charging battery minimum increasing voltage after x milliseconds.
                                                                        //  If after the x period the same voltage, green led starts lights. 
const unsigned long pauseTimeOut = 600000UL;                            // 600000 Time of waiting in pause state as turn power off. (60000 = 1 min.)
const unsigned int btnTimeOut = 5000UL;                                 // Choose therapy program time out. Counted form released button.
boolean outputDir = false;
byte coilState = LOW;
byte pin3State = LOW;
byte relayState =LOW;
unsigned long pauseTime =0; 
 
volatile boolean pause = false; // true = pause on
unsigned long pressTime = 0;    // Time of pressing the button
unsigned long startInterval;    // For unused timeout off.
byte programNo = 1;             // 0 = PC connection, 1= first program etc.
byte hr = 0;                    // User pulse from hrmPin

char memBuffer[PROGRAM_BUFFER];

//function prototypes
 int readEepromLine(int fromAddress, String &lineString);
void getParams(String &inputString);
 int executeCmd(String cmdLine, boolean directMode = false);
void eepromUpload(int adr = 0);
boolean readSerial2Buffer(int &endBuffer);
unsigned long inline checkPause();

//bioZAP functions
void scan(unsigned long freq, unsigned long period, int steps=SCAN_STEPS);
 int jump(int labelNumber, int &adr);
void off();
void beep( unsigned int period);
void freq(unsigned long _freq, long period, byte pwm = 50);
 int bat();
void wait( unsigned long period);
void exe(int &adr, int prog=0);
 int mem(String param);
void ls();
void rm();

 
void setup() {  
              
	// Initialize the digital pin as an in/output
	pinMode(coilPin,  OUTPUT);  // Coil driver
	pinMode(powerPin, OUTPUT);  // Power relay
	pinMode(greenPin, OUTPUT);  // LED on board
	pinMode(redPin,   OUTPUT);  // LED on board
	pinMode(relayPin, OUTPUT);  // Direction signal relay
	pinMode(buzzPin,  OUTPUT);  // Buzzer relay (5V or 12V which is no so loud)
	pinMode(btnPin,    INPUT);  // Main button
	pinMode(hrmPin,    INPUT_PULLUP); //Devices connection

	//bioZAP
	// Initialize serial communication to PC communication
	Serial.begin(9600);

	// Reserve the memory for inputString:
	inputString.reserve(65); //NANO serial buffer has 63 bytes


  
	if (bat() < USB_POWER_SUPPLY_LEVEL) {
		//Detected USB PC connection

		programNo = 0; //PC
    
	} else if (digitalRead(btnPin)==HIGH) {
		//Power button pressed
    
		//Turn power on
		digitalWrite(powerPin, HIGH);
    
	} else {
		//Power supplier id plugged

    
		//Work as a power charger
		rechargeBattery();
	}


	//Turn on green LED
	digitalWrite(greenPin, HIGH);
	beep(200);
  
	//Wait until turn-on button release
	startInterval = millis();
	while (digitalRead(btnPin)==HIGH){
		if ( millis() > ( startInterval + btnTimeOut ) ) {
			programNo = 4; //Coil measurement test
			digitalWrite(redPin, HIGH);
      
		}
	}
  
	delay(10);

  
	//Define minimum battery level uses in working for performance purpose.
	minBatteryLevel /*0-1023*/= (MIN_BATTERY_LEVEL / BATTERY_VOLTAGE_RATIO) ;

                                
 if (programNo) { 
    // not PC option (programNo>0)
    //Program select  
    unsigned long startInterval = millis();
    while(((millis()-startInterval) < btnTimeOut) && programNo!=4){
      if (digitalRead(btnPin)) {  
          //Reset start moment after btn pressed
          startInterval = millis();
                
          programNo++;
          if (programNo>3) programNo=1;
          for (int p=programNo; p>0; p--){
            //Signals count
            beep(80); delay(150);
          }

          //Wait until button is pressed
          while(digitalRead(btnPin));
         
          //Turn off if pressed more then 1 sec.
          if ((millis()-startInterval) > 1000 ) { 
            beep(500); 
            off();
          } 
      }
    }
    
    // Configure button interrupt
    attachInterrupt(digitalPinToInterrupt(btnPin), btnEvent, CHANGE);
    
  } else {
    //PC option
    
    //Power on
    digitalWrite(powerPin, HIGH);
     
    Serial.println(WELCOME_SCR);
    Serial.print("Device freePEMF ");
    Serial.print(HRDW_VER);
    Serial.print(" ");
    Serial.println(SOFT_VER);
    
    Serial.print('>');  
    
    // Configure button interrupt for using off option
    attachInterrupt(digitalPinToInterrupt(btnPin), btnEvent, CHANGE); 
    
  }

  startInterval=millis();
}


void loop() {

  switch (programNo) {
    	case 1:
    	//Check if user program is load in EEPROM memory

    		if ((byte)EEPROM.read(0)!=255 && (byte)EEPROM.read(0)!=0 && (byte)EEPROM.read(0)!='@') {
    			//User program execute
    			executeCmd("exe\n",true);
    			off();

    		} else {

			  	//Standard program execute
			  	executeCmd("exe 1\n",true);

    		} break;
      
		case 2:

				//Earth regeneration
		  		executeCmd("exe 2\n",true);

		  	break;

		case 3:

				// Antistress & meditation
				executeCmd("exe 3\n",true);

			break;

		case 4:
		//ON-OFF coil by pressing the button

			digitalWrite(redPin, LOW);
			while(1) {
				checkBattLevel(); //If too low then off

				if (digitalRead(btnPin)) {

					startInterval = millis();
					beep(100);

					while(digitalRead(btnPin));

						if ((millis()-startInterval) > btnTimeOut ) {
							beep(500);
							off();
						}

						if (coilState == LOW) {
							coilState = HIGH;
						} else {
							coilState = LOW;
						}

						digitalWrite(coilPin, coilState);   // turn coil on/off
						digitalWrite(redPin, coilState);   // turn LED on/off

				}
			} break;

		default:
    	// PC controlled program

			if (stringComplete) {

				//Restart timeout interval to turn off.
				startInterval=millis();

				executeCmd(inputString, true);
				Serial.print('>'); //Cursor for new command

				// clear the command string
				inputString = "";
				stringComplete = false;

			} break;
    
  	  }

  	  //Turn of if pause time is too long (longer then timeout)
  	  if (millis()-startInterval > pauseTimeOut) off();

} 


///////////////  BIOzap BEGINING  ///////////////////

String formatLine(int adr, String line){
  String printLine;
  printLine.reserve(22);
  printLine = "000"+String(adr,DEC);
  printLine = printLine.substring(printLine.length()-3,  printLine.length());
  printLine+=": "+line; //end marker for appending program
  return printLine;
}

int executeCmd(String cmdLine, boolean directMode){
// Main interpreter function

	getParams(cmdLine);


    if ( param[0]=="mem" ) { 
// Upload terapy to EEPROM
    	if ( !mem(param[1]) ){
    		Serial.println("OK");
    	}

        
    } else if ( param[0]=="ls" ) {
//List therapy
    	ls();

    
    } else if (param[0].charAt(0)=='#') {
// Comment
      ;


    } else if (param[0]==""){
// Emptyline
      
      ;


    } else if (param[0].charAt(0)==':') {
// Label - setup for new label jump counter
    	b = param[0].substring(1).toInt();
    	if (b>0 && b<MAX_LABELS){
    		if(param[1].length()>=1) {
    			if (param[1].toInt()) {
    				labelLoops[b] = param[1].toInt()-1;
    			}

#ifdef SERIAL_DEBUG
        Serial.print("label->labelno: ");
        Serial.print(b);
        Serial.print(" ");
        Serial.println(labelLoops[b]);
#endif
    		} else {
    			labelLoops[b] = -1; //Infinity loop
    		}
    	}


    } else if (param[0]=="jump"){
// Jump [label number]

    	if (  jump(param[1].toInt(), adr)  )  {

			#ifdef SERIAL_DEBUG
				Serial.print("jump->label: ");
				Serial.println(param[1].toInt());
				Serial.print("jump->address: ");
				Serial.println(adr);
			#endif

    	   	return adr;
    	}


    } else if (param[0]=="rm"){
      // Remove, clear therapy - high speed

    	EEPROM.put(0, '@');

//TODO permanent clear memory
/*
        for(int i=0; i<PROGRAM_SIZE; i++){
        	EEPROM.put(i, 255);
        	if (!(i % 128)) Serial.print(".");
      	}

*/
      Serial.println("OK");


    } else if (param[0]=="print"){
// Print command
      
      if (cmdLine.length()>6) {
        Serial.println(cmdLine.substring(6,cmdLine.length()-1));
      } else {
        Serial.println();
      }
      

    } else if (param[0]=="bat"){
// Print baterry voltage
    	Serial.println( int(analogRead(batPin)*BATTERY_VOLTAGE_RATIO));
        //Serial.println( int(analogRead(batPin)*BATTERY_VOLTAGE_RATIO));
        Serial.println(bat());


    } else if (param[0]=="cbat"){
// Calibrate battery voltage - deprecated

        Serial.println("OK");
   

    } else if (param[0]=="hr"){
// Print heart rate
      
        Serial.println(hr);


    } else if (param[0]=="beep"){
// Beep [time_ms]
        
        beep(param[1].toInt());
        Serial.println("OK");


    } else if (param[0]=="off"){
// Turn off 

    	off();


    } else if (param[0]=="chp"){
// Change output signal polarity
    
    	if (param[1]=="~") {

			  if (relayState) {
				  chp(LOW);
			  } else {
				  chp(HIGH);
			  }

    	} else 	chp(byte(param[1].toInt()));

    	Serial.println("OK");


    } else if (param[0]=="wait"){
// Wait millis or micros (negative value)

    	wait(param[1].toInt());
    	Serial.println("OK");


    } else if (param[0]=="rec"){
// Generate rectangle signal - rec [freq] [time_sec]
      
    	if (param[1]!="" ) {
    		//Deprecated way of using this command
    		freq(param[1].toInt(), param[2].toInt());
    	}
    	Serial.println("OK");


    } else if (param[0]=="sin"){
// Generate sinusoidal signal - not supported

    	Serial.println("OK");


    } else if (param[0]=="pwm"){
// Change pwm duty cycle

    	pwm = constrain( param[1].toInt(), 1, 99) ;

    	Serial.println("OK");


    } else if (param[0]=="out"){
// On-off coil
    	switch (param[1].charAt(0)) {
    		case '1':
    	    	coilState = HIGH;
    	    	Serial.println("OK");
    	    break;

    		case '0':
    	    	coilState = LOW;
    	    	Serial.println("OK");
    	    break;

    		case '~':
    			if (coilState == HIGH){
    				coilState=LOW;
    			} else {
    				coilState=HIGH;
    			}

	    	break;

    		default:

				Serial.print("Error: wrong out parameter: ");
	    		Serial.println(param[1]);
	    	break;
    	}

    	digitalWrite(coilPin, coilState);


    } else if (param[0]=="pin3"){
// On-off pin3

    	pinMode(hrmPin, OUTPUT); //pin3 output mode

    	switch (param[1].charAt(0)) {
    		case '1':
    			pin3State = HIGH;
    	    	Serial.println("OK");
    	    break;

    		case '0':
    			pin3State = LOW;
    	    	Serial.println("OK");
    	    break;

    		case '~':
    			if (pin3State == HIGH){
    				pin3State=LOW;
    			} else {
    				pin3State=HIGH;
    			}

	    	break;

    		default:

				Serial.print("Error: wrong pin3 parameter: ");
	    		Serial.println(param[1]);
	    	break;

    	}

    	digitalWrite(hrmPin, pin3State);
    	digitalWrite(redPin, pin3State);

    } else if (param[0]=="freq"){
// Generate square signal - freq [freq] [time_sec]

      freq(param[1].toInt(), param[2].toInt(), pwm);
      Serial.println("OK");


    } else if (param[0]=="scan"){
// Scan from lastFreq  - scan [freq to] [time_ms] <steps>
      
    	if (param[3]==""){
    		scan(param[1].toInt(), param[2].toInt());
    	} else {
    		scan(param[1].toInt(), param[2].toInt(), param[3].toInt());
    	}
      Serial.println("OK");


    } else if (param[0]=="restart"){
// User program restart

    	adr=0;
    	Serial.println("OK");
    	exe(adr,0);


    } else if (param[0]=="exe"){
// Execute EEPROM program only in direct mode
      //if ( directMode) {
    	b = param[1].toInt();

      if (b<4){

    	exe(adr, b);

      } else {

        Serial.print("Error: can't execute program: ");
        Serial.println(b);

      }
      //param[0]="";


    }  else {
//Unknown command
      Serial.println("Unknown command: "+param[0]);         
    }
    
return 0;
}



///////////////////////// bioZAP ////////////////////////////////////////

void rm(){
// Remove, clear script therapy from memory
	EEPROM.put(0, '@');

// Full version
//	for(int i=0; i<PROGRAM_SIZE; i++){
//		EEPROM.put(i, 255);
		//if (!(i % 128)) Serial.print(".");
//	}

}

int mem(String param){
// Upload therapy to EEPROM
    if (param=="\0") {
      eepromUpload();


    } else if (param=="@") {
      //Find script end
      int endAdr=0;
      for (int i=0; i<PROGRAM_SIZE; i++){
        if ((byte)EEPROM.read(i)==255 || (char)EEPROM.read(i)=='@'){
          endAdr=i;

          break;
        }
      }
      Serial.println(formatLine(endAdr,"appending from..."));
      eepromUpload(endAdr);


    } else if (param.toInt()>0 && param.toInt()<PROGRAM_SIZE) {
      eepromUpload(param.toInt());

    } else {
      Serial.print("Error: incorrect parameter ");
      Serial.println(param);
      return -1;
    }
    return 0;
}


int readLabelPointers(int prog){
	/* Initialize Labels pointers and jump loops
	 * prog:
	 * 0 - user program, jumps have counters,
	 * 1-9 Internal programs,
	 */
	int i;
	int adr=0;

	for(i=1; i<MAX_LABELS+1; i++ )
		labelLoops[i] = 0;

	i=0;

	do {
		if (prog>0) {
			//Internal program addresses
			adr = readFlashLine(i,line);
			getParams(line);
		} else {
			//EEPROM program labels
			adr = readEepromLine(i,line);
			getParams(line);
		}

		if (line.length()>1)
		if (line[0]==':'){
			byte lblNo = line[1]-48;
			if(lblNo>0 && lblNo<10){
				labelPointer[lblNo] = i+line.length();  // Next line of label
				//labelPointer[lblNo] = adr;  // Next line of label
				if (param[1].length()){

					if (param[1].toInt()>0) {
						labelLoops[lblNo] = param[1].toInt()-1;
					}

				} else {
					labelLoops[lblNo] = -1;
				}

				if (lblNo==prog && prog>0) return labelPointer[lblNo];

			}
		}

		i+=line.length();
		//i=adr;

	} while(adr);

#ifdef SERIAL_DEBUG
	for (i=1; i<MAX_LABELS+1;i++){
		Serial.print("readLabelPointers->label: ");
		Serial.print(i);
		Serial.print(" loops: ");
		Serial.print(labelLoops[i]);
		Serial.print(" ptr: ");
		Serial.println(labelPointer[i]);
	}
#endif


	return 0;
}
void exe(int &adr, int prog){
//Execute program

	String line;
	int endLine;


	//First time of internal and user program init.
	if (!adr && (prog>0) ){

		//Internal flash programs table
		adr = readLabelPointers(prog);

	} else if (!adr) {

		//User program label table initialization
		readLabelPointers(0);
	}


	do {

		// Read program line
		if (prog>0){

			//EEPROM memory
			endLine = readFlashLine(adr,line);

		} else {

			//Flash memory
			endLine = readEepromLine(adr,line);

		}


		adr = adr + endLine;


  		//while (endLine = readEepromLine(adr,line))
		if (endLine){

  			//Serial.print("$");
  			Serial.print(line);

  			executeCmd(line);

		}

  	} while (endLine);
       
  		Serial.println("Script done.");
  		Serial.println("OK");
  		adr=0;
}


int jump(int labelNumber, int &adr){

	if (labelNumber>0 && labelNumber<MAX_LABELS){
#ifdef SERIAL_DEBUG
			Serial.print("jump->ptr: ");
			Serial.println(labelPointer[labelNumber]);
#endif

		if (labelPointer[labelNumber]) {

#ifdef SERIAL_DEBUG
			Serial.print("jump->loops: ");
			Serial.println(labelLoops[labelNumber]);
#endif

			if (labelLoops[labelNumber] > 0) {

				adr = labelPointer[labelNumber];	//Jump to new position
				labelLoops[labelNumber]--;			//Decrees jump counter

				return adr;

			} else if(labelLoops[labelNumber]==-1) { //Unlimited loop  (-1 means maximum value of unsigned variable)

				adr = labelPointer[labelNumber];
				return adr;

			}
		}
	}
	return 0;
}


void scan(unsigned long _freq, unsigned long period, int steps){
// Scan from lastFreq to freq used SCAN_STEPS by period

	//SCAN_STEPS default;
	long scanSteps=steps;

  
	long stepPeriod = period /scanSteps;
	if (stepPeriod < 1) {
		scanSteps = period;
		stepPeriod=1;
	}
	long startFreq = lastFreq;
	long stepFreq = long( constrain(_freq, MIN_FREQ_OUT, MAX_FREQ_TIMER) - lastFreq ) / scanSteps;


	#ifdef SERIAL_DEBUG
		Serial.println("scan");
		Serial.print("freq: ");
		Serial.println(_freq);
		Serial.print("lastFreq: ");
		Serial.println(lastFreq);
		Serial.print("freq-lastFreq: ");
		Serial.println(_freq-lastFreq);
		Serial.print("startFreq: ");
		Serial.println(startFreq);
		Serial.print("stepPeriod: ");
		Serial.println(stepPeriod);
		Serial.print("scanSteps: ");
		Serial.println(scanSteps);
		Serial.print("stepFreq: ");
		Serial.println(stepFreq);
	#endif

	for (int i=1; i<=scanSteps; i++) {
		freq(startFreq+(i*stepFreq), stepPeriod);

		#ifdef SERIAL_DEBUG
			Serial.print("freq: ");
			Serial.println(startFreq+(i*stepFreq));
		#endif
	}
}

void ls(){
//List script therapy
	int adr=0;
	int endLine;
	String line;

	if (param[1]=="-n") {
		Serial.println("Adr  Command");

		while ((endLine = readEepromLine(adr,line)) && (adr<PROGRAM_SIZE) ){
		  Serial.print(formatLine(adr,line));
		  adr = adr + endLine;
		}

		//End marker (@) informs an user where to start appending of program
		if (adr<PROGRAM_SIZE) {
			Serial.println(formatLine(adr,"@"));
		}

	} else {

		for(int i=0; i<PROGRAM_SIZE; i++){
			char eeChar=(char)EEPROM.read(i);

			if ((eeChar=='@') || (eeChar==char(255))) {
				break;
			}

			Serial.print(eeChar);
		}
	}

}




void xfreq(unsigned long _freq, /*long period,*/ byte pwm){
/* Experimental version of freq function, generating more then 60Hz signal on pin PD5 (OC0B)
 * xfreq supports 61Hz - 16kHz with accuracy less then 1.6% + quartz error
 * See:
 * https://biotronika.pl/sites/default/files/2018-10/freePEMF_supported_freq_61Hz-16kHz.pdf
 */


	uint16_t prescaler = 1;
	//unsigned long l;
	//boolean flashLED = 1;


	lastFreq =constrain( _freq, MAX_FREQ_OUT, MAX_FREQ_TIMER) ;

	//OC0B / PD5 / PIN5 / CoilPin as output
    //DDRD |= (1 << DDD5); //Already is set

	//Set mode 7 (Fast PWM - TOP OCRA, non-inverting mode)
    TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00);


    // Choose the best prescaler for the frequency
    if (lastFreq<24500) {

    	prescaler = 1024;
    	TCCR0B = (1 << WGM02)  | (1 << CS02)   | (0 << CS01)   | (1 << CS00);

    } else if(lastFreq<98000){

    	prescaler = 256;
    	TCCR0B = (1 << WGM02)  | (1 << CS02)   | (0 << CS01)   | (0 << CS00);

    } else if (lastFreq<784300) {

    	prescaler = 64;
    	TCCR0B = (1 << WGM02)  | (0 << CS02)   | (1 << CS01)   | (1 << CS00);

    } else {

    	prescaler = 8;
    	TCCR0B = (1 << WGM02)  | (0 << CS02)   | (1 << CS01)   | (0 << CS00);

    }


    //Set the nearest applicable frequency
    OCR0A = F_CPU /( prescaler * (lastFreq / 100));

    //Set PWM duty cycle
    OCR0B = pwm * OCR0A / 100;

	#ifdef SERIAL_DEBUG
    /*
			Serial.println("xfreq->registers: ");
			Serial.print("TCCR0A: ");
			Serial.println(TCCR0A, BIN);
			Serial.print("TCCR0B: ");
			Serial.println(TCCR0B, BIN);
			Serial.print("OCR0A : ");
			Serial.println(OCR0A , BIN);
			Serial.print("OCR0B : ");
			Serial.println(OCR0B , BIN);
			*/
	#endif

}


void freq(unsigned long _freq, long period, byte pwm) {
  //Square signal generate, freq=783 for 7.83Hz, period in seconds or milliseconds (negative)

	//Remember last frequency for scan function
	//lastFreq =constrain( _freq, MIN_FREQ_OUT, MAX_FREQ_OUT) ;
	lastFreq =_freq;
	boolean flashLED = 1;

	if (lastFreq==0) lastFreq=1;

	if (lastFreq>MAX_FREQ_OUT) {

		//Use experimental time-counter generator
		xfreq(lastFreq, /*period,*/ pwm);

		//Wait time
		if (period >= 0) {

			//time in seconds;

			for( l = 0 ; l < period*10; l++ ) {
				_delay_ms(100);
				if (l % 10 == 0) Serial.print('.');

				// Battery level check
				checkBattLevel(); //If too low then off

				//Green led flashing every one second
				digitalWrite(greenPin, flashLED);
				flashLED = !flashLED;


				if (checkPause()) {

					//Set timer registers again after back from pause
					xfreq(lastFreq,  pwm);
				}

			}

		} else {

			//Turn green LED off for a while (milliseconds)
			digitalWrite(greenPin, LOW);

			//negative time in milliseconds
			for( l=0 ; l < -period; l++ ) _delay_ms(1);

			checkBattLevel(); //If too low then off

			checkPause();

		}

	    //Reset registers
	    TCCR0A = 3;
	    TCCR0B = 3;
	    OCR0A=0;
	    OCR0B=0;
	    digitalWrite(greenPin, HIGH);

	} else {
  
		//Use software generator
		unsigned long upInterval = pwm*1000UL/lastFreq;
		unsigned long downInterval = (100UL-pwm)*1000UL/lastFreq;


		unsigned long uptime;

		if (period>0) {
			//Seconds
			uptime = millis() + (period*1000);
		} else {
			//Milliseconds
			uptime = millis() + (-period);
		}
	
#ifdef SERIAL_DEBUG
		Serial.print("freq->upInterval: ");
		Serial.println(upInterval);
		Serial.print("freq->downInterval: ");
		Serial.println(downInterval);
#endif

		unsigned long serialStartMillis = millis();
		unsigned long startIntervalMillis = millis();
		//unsigned long pausePressedMillis;

		coilState=HIGH;
		digitalWrite(coilPin, coilState);   // turn coil off
		digitalWrite(greenPin, coilState);   // turn LED off

		while(millis()< uptime) {
		  //time loop

			if (((millis() - startIntervalMillis) >= upInterval) && (coilState==HIGH)) {

				//Save start time interval
				startIntervalMillis = millis();

				coilState=LOW;
				digitalWrite(coilPin, coilState);   // turn coil off
				digitalWrite(greenPin, coilState);   // turn LED off

			}

			if (((millis() - startIntervalMillis) >= downInterval) && (coilState==LOW)) {

				//Save start time interval
				startIntervalMillis = millis();

				coilState=HIGH;
				digitalWrite(coilPin, coilState);   // turn coil on
				digitalWrite(greenPin, coilState);   // turn LED on

			}


			checkBattLevel(); //If too low then off

			//TODO serial break command

			//Check if pause button pressed and correct uptime
			uptime += checkPause();

			//Count each second and print dot
			if (millis()-serialStartMillis >= 1000) { //one second
				Serial.print('.');
				serialStartMillis = millis();
			}
		}
		digitalWrite(coilPin, LOW);     // turn coil off
		digitalWrite(greenPin, HIGH);   // turn LED on
	}

}


unsigned long inline checkPause(){
	if (pause) {
	//Pause - button pressed

		unsigned long pausePressedMillis;

		pausePressedMillis = millis();
		beep(200);
		digitalWrite(coilPin, LOW);     // turn coil off
		digitalWrite(greenPin, HIGH);   // turn LED on

		while (pause){
		//wait pauseTimeOut or button pressed

			if (millis()> pausePressedMillis + pauseTimeOut) {
				beep(500);
				off();
			}

		}
		beep(200);

		//Correct working time
		//timeUp += millis()-pausePressedMillis;
		//startIntervalMillis += millis()-pausePressedMillis;


		//Continue
		digitalWrite(coilPin, coilState);    // turn coil on
		digitalWrite(greenPin, coilState);   // turn LED on/

		//Return delta to work time correction
		return millis()-pausePressedMillis;
	} else	return 0;
}

void off() {
  // Power off function
  
  digitalWrite(coilPin, LOW);     // Turn coil off by making the voltage LOW
  delay(20);
  digitalWrite(relayPin, LOW);    // Relay off
  digitalWrite(greenPin, LOW);    // Green LED off

  digitalWrite(powerPin, LOW);  // Turn power off //If not USB power

  while(digitalRead(btnPin)==HIGH); // Wait because still power on

  //If USB PC connection is plugged to arduino pcb cannot turn power off
  //detachInterrupt(digitalPinToInterrupt(btnPin));
  
  while(1); //forever loop
  
}

void chp(byte outputDir){
  //Change output polarity

  digitalWrite(coilPin, LOW);  // turning coil off
  
  relayState=outputDir;
  digitalWrite(relayPin, relayState);



}

int bat() {
// Get battery voltage function

  return ( analogRead(batPin) * BATTERY_VOLTAGE_RATIO );

}

void wait( unsigned long period) {
// wait [period_ms]

//TODO : change to _delay_ms(1000)

  unsigned long serialStartPeriod = millis();
  unsigned long startInterval = millis();    
  
  while(millis()-startInterval <= period){
    //time loop
      
    //TODO serial break command
          
    //count each second
    if (millis()-serialStartPeriod >= 1000) {
      Serial.print('.');
      serialStartPeriod = millis();
    }
  }

}


void beep( unsigned int period) {
  // beep [period_ms]

  unsigned long serialStartPeriod = millis();
  unsigned long startInterval = millis();

  digitalWrite(buzzPin, HIGH);
  while(millis()-startInterval <= period){
    //time loop

    //TODO serial break command

    //count each second
    if (millis()-serialStartPeriod >= 1000) { //one second
      Serial.print('.');
      serialStartPeriod = millis();
    }
  }
    
  digitalWrite(buzzPin, LOW);
}

///////////////  BIOzap END  ///////////////////

void checkBattLevel() {
  //Check battery level
#ifndef NO_CHECK_BATTERY

  if ( analogRead(batPin) < minBatteryLevel) { 
    //Emergency turn off 
    Serial.println();  
    Serial.print("Error: battery too low: ");
    Serial.println(bat());
    
      
    // red LED on
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);   
      
    // Turn all off
    digitalWrite(coilPin, LOW);    // Turn coil off by making the voltage LOW
    digitalWrite(relayPin, LOW);    // Relay off
        
    for (int x=0; x<10; x++){
      digitalWrite(buzzPin, HIGH);   // Turn buzzer on 
      delay(100); 
      digitalWrite(buzzPin, LOW);    // Turn buzzer off
      delay(200); 
    }
        
    beep(500);
    off();
  }
#endif

}

void rechargeBattery() {
  //Recharges is plugged
  
  digitalWrite(powerPin, LOW); // turn power relay off
  digitalWrite(redPin, HIGH);
  beep(200);
  digitalWrite(greenPin, LOW);
      
  unsigned long startInterval = millis();
  int startBatLevel = analogRead(batPin);

  do {
    if ( millis() - startInterval > checkDeltaBatteryIncreasingVoltageTime) {          
      if (analogRead(batPin)-startBatLevel <= 0) { //no increasing voltage
        //Battery recharged

        digitalWrite(greenPin, HIGH);
        beep(200);
        // ... and charge further.
        while (1);
      }
 
      //Start new charging period with new values
      startInterval = millis();
      startBatLevel = analogRead(batPin);
    }
  }  while (1); //forever loop
}

void btnEvent() {
   //Change button state interruption 
   //unsigned long pressTime =0;
   
  if (digitalRead(btnPin)==HIGH){ 
    pressTime = millis(); //Specific use of millis(). No increment in interruption function.
  } else { 
    if (pressTime && (millis() - pressTime > 50)) pause=!pause;
    if (pressTime && (millis() - pressTime > 1000)) { 
      for(unsigned int i=0; i<50000; i++) digitalWrite(buzzPin, HIGH); //Cannot use delay() therefore beep() function in innteruption
      digitalWrite(buzzPin, LOW);
      off(); 
    } 
    pressTime = 0;
  }
}


int readEepromLine(int fromAddress, String &lineString){
  //Read one line from EEPROM memory
  int i = 0;
  lineString="";
  do {
    char eeChar=(char)EEPROM.read(fromAddress+i);
    if ((eeChar==char(255)) ||(eeChar==char('@'))) {
      if (i>0) {
        eeChar='\n';
      } else {
        i=0;
        break;
      }
    }
    lineString+=eeChar; 
    i++;
    if (eeChar=='\n') break;
  } while (1);
  
  return i;
}

void getParams(String &inputString){
  for (int i=0; i<MAX_CMD_PARAMS; i++) param[i]="";
  
  int from =0;
  int to =0;
  for (int i=0; i<MAX_CMD_PARAMS; i++){
    to = inputString.indexOf(' ',from); //Find SPACE
   
    if (to==-1) {
      to = inputString.indexOf('\n',from); //Find NL #10
      if (to>0) param[i] = inputString.substring(from,to);
      param[i].trim();
      break;
    }

    if (to>0) param[i] = inputString.substring(from,to);
    param[i].trim();
    from = to+1;
  }
}




/////////////////////////////////////////////////////////////////////////////
void serialEvent() {

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    if (inChar!='\r'){
      inputString += inChar;
    }

    Serial.print(inChar); //echo

    // if the incoming character is a newline, set a flag
    if (inChar == '\n') {
      stringComplete = true;
    }

    if (inChar == '@') {
      memComplete = true;
    }
  }

}


void eepromUpload(int adr) {
  unsigned int i = 0;
  boolean flagCompleted;
  boolean Xoff = false;
  int endBuffer;
  //eepromLoad = true;

  do { 
    //Serial.print(char(XON));
    Xoff = readSerial2Buffer(endBuffer);
    int b =0; // buffer pointer
    flagCompleted = false;
    while (!flagCompleted){
      
      flagCompleted = !(i+adr<PROGRAM_SIZE) || (memBuffer[b]=='@') || !(b < endBuffer);
      if (memBuffer[b]==';') memBuffer[b]='\n';   //Semicolon as end line LF (#10) for windows script
      if (memBuffer[b]=='\r') memBuffer[b] = ' '; //#13 -> space, No continue because of changing script length 
      EEPROM.write(i+adr, memBuffer[b]); 
      //Serial.print(memBuffer[b]);
      i++; b++;
    }
    //End of shorter program then PROGRAM_SIZE size
    
  } while (Xoff); 
  if (i+adr<PROGRAM_SIZE) EEPROM.write(i+adr, 255); 
  //eepromLoad=false;
}

boolean readSerial2Buffer(int &endBuffer) {  
    int i = 0; //buffer indicator
    char c;
   
    boolean Xoff = false;
    int highBufferLevel = 0.7 * PROGRAM_BUFFER;
    
    Serial.write(XON);
    //Serial.print("\nXON\n");

    while(true) {
      if (Xoff) {
        //after send Xoff
        
          if (Serial.available()){
            c = Serial.read();
            memBuffer[i] = c;
            //Serial.print(c);
            endBuffer = i;
            i++;

          } else {
            break;
          };
           //if (i>= PROGRAM_BUFFER) break;
         

      } else {
        //before send Xoff
          
          Xoff = (i > highBufferLevel);

          while(!Serial.available());

          c = Serial.read();
          
          memBuffer[i] = c;
          //Serial.print(c);
          endBuffer = i;

          if (c == '@' ) {
            break;    
          }
          
          i++;
          if (Xoff) {
            for (int j=0; j<64; j++)
            Serial.write(XOFF);  
            //Serial._rx_complete_irq();
            //Serial._tx_udr_empty_irq();
            //Serial.print("\nXOFF\n");
          }         
      }
      
      
      
    }
  return Xoff;  

}


#include <avr/pgmspace.h>
const char internalProgram[] PROGMEM   = {

		":1\n"
		"#Standard program 13 m\n"
		"rec 1179 120\n"
		"chp 1\n"
		"rec 783 120\n"
		"chp 0\n"
		"rec 2000 60\n"
		"chp 1\n"
		"rec 1500 60\n"
		"chp 0\n"
		"rec 1000 90\n"
		"chp 1\n"
		"rec 700 90\n"
		"chp 0\n"
		"rec 200 120\n"
		"beep 500\n"
		"off\n"

		":2\n"
		"#Earth regeneration - 8 m\n"
		"rec 1179 120\n"
		"chp 1\n"
		"rec 1179 120\n"
		"chp 0\n"
		"rec 783 120\n"
		"chp 1\n"
		"rec 783 120\n"
		"beep 500\n"
		"off \n"

		":3\n"
		"#Antisterss & meditation 16 m\n"
		"freq 1200 20\n"
		"freq 1179 150\n"
		"chp 1\n"
		"freq 1166 20\n"
		"freq 1133 20\n"
		"freq 1100 20\n"
		"freq 1066 20\n"
		"freq 1033 20\n"
		"freq 1000 20\n"
		"freq 966 20\n"
		"freq 933 20\n"
		"freq 900 20\n"
		"freq 866 20\n"
		"freq 833 20\n"
		"freq 800 20\n"
		"chp 0\n"
		"freq 800 20\n"
		"freq 783 120\n"
		"chp 1\n"
		"freq 766 20\n"
		"freq 733 20\n"
		"freq 700 20\n"
		"freq 666 20\n"
		"freq 633 20\n"
		"freq 600 20\n"
		"freq 566 20\n"
		"freq 533 20\n"
		"freq 500 20\n"
		"freq 466 20\n"
		"freq 433 20\n"
		"freq 400 20\n"
		"chp 0\n"
		"freq 366 20\n"
		"freq 333 20\n"
		"freq 300 20\n"
		"freq 266 20\n"
		"freq 233 20\n"
		"freq 200 20\n"
		"freq 166 20\n"
		"freq 130 20\n"
		"freq 100 20\n"
		"off \n"

		"@"

};

int readFlashLine(int fromAddress, String &lineString){
	  //Read one line from EEPROM memory
	  int i = 0;
	  lineString="";

#ifdef SERIAL_DEBUG
	  	//Serial.print("readFlashLine->fromAddress: ");
		//Serial.println(fromAddress);
#endif

	  do {

	    char eeChar = char( pgm_read_byte(&internalProgram[fromAddress+i])  )  ;

#ifdef SERIAL_DEBUG
	  	//Serial.print("readFlashLine->eeChar: ");
		//Serial.println(eeChar);
#endif
	    if ( eeChar==char('@') ) {
	      if (i>0) {
	        eeChar='\n';
	      } else {
	        i=0;
	        break;
	      }
	    }
	    lineString+=eeChar;
	    i++;
	    if (eeChar=='\n') break;
	  } while (1);
#ifdef SERIAL_DEBUG
	  	//Serial.print("readFlashLine->i: ");
		//Serial.println(i);
#endif
	  return i;
}








