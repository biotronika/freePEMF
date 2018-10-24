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

#endif /* FREEPEMF_PROG_H_ */
