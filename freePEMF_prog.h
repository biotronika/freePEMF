/*
 * freePEMF_prog.h
 *
 *  Created on: 22 lut 2018
 *      Author: blos
 */

#ifndef FREEPEMF_PROG_H_
#define FREEPEMF_PROG_H_
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
		"#Antistress & meditation 16 m\n"

		"off \n"
			
		"#Antisterss & meditation 16 m\n"
		"freq 1200 20\n"
		"freq 1179 150\n"
		"chp 1\n"
		"freq 1166 20\n"
		"freq 1133 20\n"
		"freq 1100 20\n"
		"freq 1066 20\n"
freq 1033 20
freq 1000 20
freq 966 20
freq 933 20
freq 900 20
freq 866 20
freq 833 20
freq 800 20
chp 0
freq 800 20
freq 783 120
chp 1
freq 766 20
freq 733 20
freq 700 20
freq 666 20
freq 633 20
freq 600 20
freq 566 20
freq 533 20
freq 500 20
freq 466 20
freq 433 20
freq 400 20
chp 0
freq 366 20
freq 333 20
freq 300 20
freq 266 20
freq 233 20
freq 200 20
freq 166 20
freq 130 20
freq 100 20
		"off \n"
			
/*
        ":4\n"
		"#Pineal gland 15m\n"
		"beep 200\n"
		"freq 783 120\n"
		"chp 1\n"
		"freq 2000 180\n"
		"chp 0\n"
		"freq 53700 180\n"
		"chp 1\n"
		"freq 66200 180\n"
		"chp 0\n"
		"freq 93600 180\n"
		"beep 200\n"
		"off\n"
*/
		"@"


};









#endif /* FREEPEMF_PROG_H_ */
