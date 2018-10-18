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
		"rec 1179 120\n"
		"chp 1\n"
		"rec 1200 10\n"
		"scan 800 230\n"
		"chp 0\n"
		"rec 783 120\n"
		"chp 1\n"
		"rec 800 10\n"
		"scan 400 230\n"
		"chp 0\n"
		"scan 100 240\n"
		"beep 500\n"
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
