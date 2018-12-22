# freePEMF
One Arduino ino file for beginners. Full functional software for freePEMF device. 
See: [http://biotronics.eu](https://biotronika.pl/en/) or for Polish: https://biotronika.pl 

Support frequency up to 16kHz (0.01-60.99Hz standard, 61Hz-16kHz experimental)

Related document: [bioZAP 2018-10-21 EN.pdf](https://biotronika.pl/sites/default/files/2018-10/bioZAP%202018-10-21%20EN.pdf)
or a litle out of date for Polish: [bioZAP 2018-04-09.pdf](https://biotronika.pl/sites/default/files/2018-04/bioZAP%202018-04-09.pdf) :(


### To compile code and upload using Arduino IDE:
1. Download freePEMF.ino file and put it into freePEMF folder (it must has exactly that name). 
2. Open freePEMF.ino file in Arduino IDE.
3. Check if you have EEPROM libraries already installed (Sketch->Include Library-> see on list: EEPROM and Wire).
4. Install LiquidCrystal_I2C  (Sketch->Include Library->Manage Libraries..., Filter your search: LiquidCrystal_I2C, Find by Frank de Brabander, Choose version 1.1.2 and Install)
7. Configure board (Tools->Board->Arduino Nano)  (Tools->Processor->ATmega328).
8. Install Arduino Nano driver - **biotronics.eu** website: [CH341SER.ZIP]( https://biotronika.pl/sites/default/files/2016-12/CH341SER.ZIP).
9. Configure serial port. Plug USB cable to PC and freePEMF or Adruino Nano board. Then Tools->Port->select proper COM port.
10. Compile and upload. Sketch->Upload. Wait until on down side of Arduino IDE window see **Done uploading**.


#### TODO in freePEMF user manual
1. support for bluetooth interface and communication with PC & mobile phones
2. freePREMF duo functions description

#### TODO:
* hrm
* chrm
* jump ... min max
* njump ... min max
* jump3
* support for bluetooth interface

#### DONE
* during loading a therapy add voice signal after committing each part of script

