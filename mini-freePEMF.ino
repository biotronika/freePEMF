// mini-freePEMF See: biotronika.pl  biotronics.eu
// Chris Czoba (c) krzysiek@biotronika.pl
// Renew 2017-07-28 sof_ver with running  bioZAP 2018-04-09

#define HRDW_VER "NANO 4.2"
#define SOFT_VER "2018-10-19"

#include <EEPROM.h>
#include "freePEMF_prog.h"
//#include <stdio.h>

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
#define batPin A7                               // Analog-in battery level 
#define BATTERY_VOLTAGE_RATIO 0.153             // include 10k/4,7k resistor voltage divider. 5V*(10k+4,7k)/4,7k = 0,0153 (x10)
#define EEPROM_BATTERY_CALIBRATION_ADDRESS 1023 // Memory address of battery correction factor - 100 means RATIO x 1,00
#define MIN_BATTERY_LEVEL 90                    // 90 means 9.0 V  (x10), less then that turn off
#define USB_POWER_SUPPLY_LEVEL 65               // Maximum USB voltage level means 6.5V


//BIOzap
#define WELCOME_SCR "Free BIOzap interpreter welcome! See http://biotronics.eu"
#define PROGRAM_SIZE 1000   // Maximum program size
#define PROGRAM_BUFFER 500  // SRAM buffer size, used for script loading
#define MAX_CMD_PARAMS 4    // Count of command parameters
#define LCD_SCREEN_LINE -1  // LCD user line number, -1 = no LCD
#define MIN_FREQ_OUT 1      //  0.01 Hz
#define MAX_FREQ_OUT 5000   // 50.00 Hz
#define SCAN_STEPS 20       // For scan function purpose - default steps
#define MAX_LABELS 9        // jump labels maximum

#define XON 17  //0x11
#define XOFF 19 //0x13

//BIOzap
String inputString = "";                // a string to hold incoming serial data
String param[MAX_CMD_PARAMS];           // param[0] = cmd name
boolean stringComplete = false;         // whether the string is complete
boolean memComplete = false;
unsigned long lastFreq = MIN_FREQ_OUT;  // Uses scan function
int minBatteryLevel = 0; 
boolean Xoff = false;
byte b;
int i;
long l;

int labelPointer[MAX_LABELS+1];  				// Next line of label
int labelLoops[MAX_LABELS+1];    				// Number of left jump loops



const unsigned long checkDeltaBatteryIncreasingVoltageTime = 600000UL;  // During charging battery minimum inreasing voltage after x millisecounds. 
                                                                        //  If after the x period the same voltage, green led starts lights. 
const unsigned long pauseTimeOut = 600000UL;                            // 600000 Time of waiting in pause state as turn power off. (60000 = 1 min.)
const unsigned int btnTimeOut = 5000UL;                                 // Choose therapy program time out. Counted form released button.
boolean outputDir = false;
byte coilState = LOW;
unsigned long pauseTime =0; 
 
volatile boolean pause = false; // true = pause on
unsigned long pressTime = 0;    // Time of pressing the button
unsigned long startInterval;    // For unused timeout off.
byte programNo = 1;             // 0 = PC connection, 1= first program etc.
byte hr = 0;                    // User pulse from hrmPin

//Serial buffer
//char *memBuffer; 
char memBuffer[PROGRAM_BUFFER];

//function prototypes
int readEepromLine(int fromAddress, String &lineString);
void getParams(String &inputString);
void executeCmd(String cmdLine, boolean directMode = false);
void eepromUpload(int adr = 0);
boolean readSerial2Buffer(int &endBuffer);
void scan(unsigned long freq, unsigned long period, int steps=SCAN_STEPS);

 
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

  //BIOzap
  // Initialize serial communication to PC communication
  Serial.begin(9600);
  // reserve the bytes for the inputString:
  inputString.reserve(65); //NANO serial buffer has 63 bytes

  
  if (bat() < USB_POWER_SUPPLY_LEVEL) { 
    //Detected USB PC connection

    programNo = 0; //PC
    
  } else if (digitalRead(btnPin)==HIGH) {
    //Power button pressed
    
    //Turn power on
    digitalWrite(powerPin, HIGH);
    
  } else {
    //Power supplyer id pluged

    if (digitalRead(hrmPin)==LOW) {
      //Work as the water magnetizer

      //Special signal
      digitalWrite(greenPin, HIGH);
      digitalWrite(redPin, HIGH);
      beep(250);
      delay(1000);
      digitalWrite(redPin, LOW);
      
      digitalWrite(greenPin, HIGH);
      
      beep(250);
      delay(1000);

      //Forever  loop
      while (1) {
        digitalWrite(greenPin, HIGH);
        digitalWrite(powerPin, HIGH);
        delay(50);
        executeCmd("exe\n",true); //default programNo;
        digitalWrite(powerPin, LOW);
        delay(10000);
      }
      
      //digitalWrite(greenPin, LOW);
    }
    
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
  };
  
  delay(10);

  //Auto-correction voltage - for new device
  /*
  if ( (byte)EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS) > 130 ||
       (byte)EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS) < 70 ) {
    EEPROM.put(EEPROM_BATTERY_CALIBRATION_ADDRESS,100); // 100 =  x 1.00
  }
  */
  
  //Define minimum battery level uses in working for perfomance puropose.
  minBatteryLevel /*0-1023*/= 100 * 
                              MIN_BATTERY_LEVEL / 
                              BATTERY_VOLTAGE_RATIO ; /* /
                              (byte)EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS);*/

                                
 if (programNo) { 
    // not PC option (programNo>0)
    //Program select  
    unsigned long startInterval = millis();
    while(((millis()-startInterval) < btnTimeOut) && programNo!=4){
      if (digitalRead(btnPin)) {  
          //Reset start moment after btn preesed
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
    Serial.print("Device free-PEMF ");
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
      //eeFirst=(byte)EEPROM.read(0);
      //Serial.println(eeFirst);
      if ((byte)EEPROM.read(0)!=255 && (byte)EEPROM.read(0)!=0) {
        //User program execute
   
        executeCmd("exe\n",true);
        off();
        
      } else {
        //Standard program execute
          
        freq(1179, 120); //2 min   11,79Hz Earth geomagnetic field
        chp(1);
        freq( 783, 120); //2 min    7,83 Schuman    
        chp(0);      
        freq(2000,  60); //2min    20,0  Capillary (ukł. krwinośny) 
        chp(1);
        freq(1500,  60); //2min    15,0  Capillary   
        chp(0);
        freq(1000,  90); //1:30min 10,0  Ligament (wiązadła)
        chp(1);
        freq( 700,  90); //1:30min  7,0  Bone (kości)   
        chp(0);
        freq( 200, 120); //2min     2,0  Nerve 
        beep(500);
        off();
      }
      break;
      
    case 2:
    //Earth regeneration - 8 minutes
      
      freq(1179,120); //4 min 11,79Hz Earth geomagnetic field
      chp(1);
      freq(1179,120); 
      chp(0);
      freq(783, 120); //4 min 7,83 Schumanns resonance    
      chp(1);
      freq(783, 120); 
      beep(500);
      off();
      break;

    case 3:
    // Antistress & meditation (without feedback)
    
      // 16 min
      freq( 1179, 120); //2 min 11,79Hz Earth geomagnetic field
      chp(1);
      freq( 1200,  10); //4 min 12->8Hz  Alpha
      scan( 800, 230); 
      chp(0);
      freq(  783, 120); //2 min  7,83 Schumanns resonance
      chp (1);
      freq(  800,  10);
      scan( 400, 230);  //4 min 8->4 Theta
      chp (0);
      scan( 100, 240);  //4 min 4->0,5  Delta
      beep(500);
      off();
      break;

    case 4:
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
      }
      break;
     
    default: 
    
    // PC controled program   

      if (stringComplete) {

        //Restart timeout interval to turn off. 
        startInterval=millis();       
        
        executeCmd(inputString, true);
        Serial.print('>'); //Currsor for new command

        // clear the command string
        inputString = "";
        stringComplete = false;  
      } 
    
    break; 
    
  } 
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

void executeCmd(String cmdLine, boolean directMode){
  // Main interpreter function
  //digitalWrite(powerPin, HIGH);
  getParams(cmdLine);


    if ( param[0]=="mem" ) { 
// Upload terapy to EEPROM
      
      if (param[1]=="\0") {
        eepromUpload();
     
      } else if (param[1]=="@") {
        //Find script end  
        int endAdr=0;
        for (int i=0; i<PROGRAM_SIZE; i++){
          if ((byte)EEPROM.read(i)==255 || (char)EEPROM.read(i)=='@'){
            endAdr=i;
            
            break;
          }    
        }
        Serial.println(formatLine(endAdr,"appendin from..."));
        eepromUpload(endAdr);   
        
      } else if (param[1].toInt()>0 && param[1].toInt()<PROGRAM_SIZE) {
        eepromUpload(param[1].toInt());         
      } else {
        Serial.print("Error: unknow parametr "); 
        Serial.println(param[1]);
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
        Serial.print("label: ");
        Serial.print(b);
        Serial.print(" ");
        Serial.println(labelLoops[b]);
#endif
    		} else {
    			labelLoops[b] = -1; //Infinity loop
    		}
    	}

    } else if (param[0]=="rm"){
      // Remove, clear therapy - hispeed

    	EEPROM.put(0, '@');
      //for(int i=0; i<PROGRAM_SIZE; i++){
        //EEPROM.put(i, 255);
        //if (!(i % 128)) Serial.print(".");
      //}
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
// Calibrate battery voltage
    	//do nothing
        //Correction factor
        //byte i = 100 * param[1].toInt()/(int(analogRead(batPin)*BATTERY_VOLTAGE_RATIO));
       
        //EEPROM.put(EEPROM_BATTERY_CALIBRATION_ADDRESS, i);
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
    
      //chp(byte(param[1].toInt()));
      chp(byte(param[1].toInt()));
      Serial.println("OK");

    } else if (param[0]=="wait"){
// Wait millis or micros (negative value)
    	int w = param[1].toInt();
    	if (w>=0) {
    		wait(w);
    	} else {
    		delayMicroseconds(-w);
    	}
      Serial.println("OK");


    } else if (param[0]=="rec"){
// Generate rectangle signal - rec [freq] [time_sec]
      
    	if (param[1]!="" ) {
    		freq(param[1].toInt(), param[2].toInt());
    	}
    	Serial.println("OK");


    } else if (param[0]=="freq"){
// Generate rectangle signal - freq [freq] [time_sec]

      freq(param[1].toInt(), param[2].toInt());
      Serial.println("OK");

    } else if (param[0]=="scan"){
      // Scan from lastFreq  - scan [freq to] [time_ms] <steps>
      
    	if (param[3]==""){
    		scan(param[1].toInt(), param[2].toInt());
    	} else {
    		scan(param[1].toInt(), param[2].toInt(), param[3].toInt());
    	}
      Serial.println("OK");

      //void scan(unsigned int freq, unsigned long period){

    } else if (param[0]=="exe"){
      // Execute EEPROM program only in direct mode
      if ( directMode) { 
        exe();
      } else {
        Serial.println("Error: can't execute program from EEPROM program!");
      }
      //param[0]="";

    }  else {
//Unknown command
      Serial.println("Unknown command: "+param[0]);         
    }
    

}

void rm(){
// Remove, clear script therapy from memory
	EEPROM.put(0, '@');

//	for(int i=0; i<PROGRAM_SIZE; i++){
//		EEPROM.put(i, 255);
		//if (!(i % 128)) Serial.print(".");
//	}
	//Serial.println("OK");
}


void exe(){
  //Execute program 
  
  int adr=0;
  String line;
  while (int endLine = readEepromLine(adr,line)){

    Serial.print("executing: ");
    Serial.print(line);  

    executeCmd(line);
    adr = adr + endLine;
  }
       
  Serial.println("Script done."); 
  Serial.println("OK"); 
}

void scan(unsigned long freq_, unsigned long period, int steps){
  // Scan from lastFreq to freq used SCAN_STEPS by period

  
  long scanSteps=SCAN_STEPS;

  
  long stepPeriod = period /scanSteps;
  if (stepPeriod < 1) {
    scanSteps = period;
    stepPeriod=1;
  }
  long startFreq = lastFreq;
  long stepFreq = long( constrain(freq_, MIN_FREQ_OUT, MAX_FREQ_OUT) - lastFreq ) / scanSteps;
/*
  Serial.println(freq);
  Serial.println(lastFreq);
  Serial.println(long(freq-lastFreq));
  Serial.println(startFreq);  
  Serial.println(stepPeriod);
  Serial.println(scanSteps);
  Serial.println(stepFreq);
*/  

  for (int i=0; i<scanSteps; i++) {
    freq(startFreq+(i*stepFreq), stepPeriod);
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


void freq(unsigned int freq, unsigned long period) {
  //Rectangle signal generate, freq=783 for 7.83Hz, period in secounds

  lastFreq =constrain( freq, MIN_FREQ_OUT, MAX_FREQ_OUT) ; //For scan() function puropose
  
  unsigned long interval = 50000/constrain(freq, MIN_FREQ_OUT, MAX_FREQ_OUT);   
  unsigned long timeUp = millis() + (period*1000);

  unsigned long serialStartPeriod = millis();
  unsigned long startInterval = millis();
  unsigned long pausePressed;

  while(millis()< timeUp) {
      //time loop
      
      if ((millis() - startInterval) >= interval) {

        //Save start time interval 
        startInterval = millis();
        
        if (coilState == LOW) {
          coilState = HIGH;
        } else {
          coilState = LOW;
        }
        
        digitalWrite(coilPin, coilState);   // turn coil on/off 
        digitalWrite(greenPin, coilState);   // turn LED on/off
      }

      checkBattLevel(); //If too low then off

      //TODO serial break command - mark @

      if (pause) {
        //Pause - button pressed
          
          pausePressed = millis();
          beep(200);
          digitalWrite(coilPin, LOW);     // turn coil ooff 
          digitalWrite(greenPin, HIGH);   // turn LED on

          while (pause){
            //wait pauseTimeOut or button pressed
            if (millis()> pausePressed + pauseTimeOut) { 
               beep(500); 
               off();
            } 
          }
          beep(200);
          
          //Correct working time
          timeUp += millis()-pausePressed;
          startInterval += millis()-pausePressed;


          //Continue
          digitalWrite(coilPin, coilState);    // turn coil on
          digitalWrite(greenPin, coilState);   // turn LED on/      
      }     
      

      //count each second
      if (millis()-serialStartPeriod >= 1000) { //one second
        Serial.print('.');
        serialStartPeriod = millis();
      }
  }
  digitalWrite(coilPin, LOW);     // turn coil off 
  digitalWrite(greenPin, HIGH);   // turn LED on

}

void off() {
  // Power off function
  
  digitalWrite(coilPin, LOW);     // Turn coil off by making the voltage LOW
  delay(20);
  digitalWrite(relayPin, LOW);    // Relay off
  digitalWrite(greenPin, LOW);    // Green LED off

  digitalWrite(powerPin, LOW);  // Turn power off //If not USB power

  while(digitalRead(btnPin)==HIGH); // Wait because still power on

  //If USB PC connection is pluged microcontroller cannot turn power off
  //detachInterrupt(digitalPinToInterrupt(btnPin));
  
  while(1); //forever loop
  
}

void chp(byte outputDir){
  //Change output polarity

  digitalWrite(coilPin, LOW);  // turning coil off
  if (outputDir) {
    digitalWrite(relayPin, HIGH); // turn relay on 
  } else {
    digitalWrite(relayPin, LOW); // turn relay off 
  }
  
}

int bat() {
  // Get battery voltage function
  
  return (  analogRead(batPin) *
            BATTERY_VOLTAGE_RATIO *
            (byte)EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS) /
            100
          );  
}

void wait( unsigned long period) {
  // wait [period_ms]

  unsigned long serialStartPeriod = millis();
  unsigned long startInterval = millis();    
  
  while(millis()-startInterval <= period){
    //time loop
      
    //TODO serial break command - mark @
          
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

    //TODO serial break command - mark @

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

}

void rechargeBattery() {
  //Recharger is pluged 
  
  digitalWrite(powerPin, LOW); // turn power relay off
  digitalWrite(redPin, HIGH);
  beep(200);
  digitalWrite(greenPin, LOW);
      
  unsigned long startInterval = millis();
  int startBatLevel = analogRead(batPin);

  do {
    if ( millis() - startInterval > checkDeltaBatteryIncreasingVoltageTime) {          
      if (analogRead(batPin)-startBatLevel <= 0) { //no inreasing voltage
        //Battery rechareged

        digitalWrite(greenPin, HIGH);
        beep(200);
        // ... and charege further.
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
    pressTime = millis(); //Specific use of millis(). No increment in innteruption function.
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
    to = inputString.indexOf(' ',from); //Wykryj spacje
   
    if (to==-1) {
      to = inputString.indexOf('\n',from); //Wykryj NL #10
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

/*
 void eepromUpload(int adr) {
  unsigned int i = 0;
  boolean flagCompleted = false;

  while (!flagCompleted){
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      flagCompleted =( !(i<PROGRAM_SIZE) ) || (inChar=='@');
      if (inChar==';') inChar='\n';   //Semicollon as end line LF (#10)
      EEPROM.put(i, (byte)inChar);
      i++;
    }
  }
  if (i<PROGRAM_SIZE) EEPROM.put(i, 255); //End of shorter program then PROGRAM_SIZE size
}
 */

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
      if (memBuffer[b]==';') memBuffer[b]='\n';   //Semicollon as end line LF (#10) for windows script
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




