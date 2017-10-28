/*
 * ui.h
 *
 *  Created on: Oct 4, 2017
 *      Author: wrieden
 */

#ifndef UI_H_
#define UI_H_

#define STM8S103
#define __SDCC
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm8s.h"
#include "stm8s_itc.h"

#define MIDDLEPRESS 25
#define LONGPRESS 50
#define SHORTPRESS 2
#define VERYLONGPRESS 150
#define SHOW_SET_TEMP 75

enum {
	UI_BACK, UI_MAIN, UI_AIRQUICK, UI_IRONQUICK, UI_MENU
};

enum {
	UI_MENU_BACK, UI_MENU_IRON, UI_MENU_AIR,UI_MENU_ROOMTEMP, UI_MENU_BRIGHTNESS, UI_MENU_SHOWALWAYSTEMP, UI_MENU_SHOWRAW
};

enum {
	UI_PID_BACK, UI_PID_KP, UI_PID_KI, UI_PID_KD, UI_PID_ITHRESHOLD
};

enum {
	UI_MENU_IRON_BACK, UI_MENU_IRON_PID, UI_MENU_IRON_CALIBRATE
};

enum {
	UI_MENU_AIR_BACK, UI_MENU_AIR_FANSHUTDOWNTEMP, UI_MENU_AIR_FANDELAY, UI_MENU_AIR_PID, UI_MENU_AIR_CALIBRATE
};


uint8_t TM1628_buffer[6];
uint8_t TM1628_charbuffer[6];


const uint8_t TM1628_ASCII[] = {
		0x00,	// (32)  <space>
		0x50,	// (33)	!
		0x60,	// (34)	"
		0x7B,	// (35)	#
		0x23,	// (36)	$
		0xB8,	// (37)	%
		0x51,	// (38)	&
		0x40,	// (39)	'
		0xD8,	// (40)	(
		0xAA,	// (41)	)
		0xE1,	// (42)	*
		0x51,	// (43)	+
		0x10,	// (44)	,
		0x01,	// (45)	-
		0x04,	// (46)	.
		0x31,	// (47)	/
		0xFA,	// (48) 0
		0x22,	// (49) 1
		0xB9,	// (50) 2
		0xAB,	// (51) 3
		0x63,	// (52) 4
		0xCB,	// (53) 5
		0xDB,	// (54) 6
		0xA2,	// (55) 7
		0xFB,	// (56) 8
		0xEB,	// (57) 9
		0x88,	// (58)	:
		0x72,	// (59)	;
		0x19,	// (60)	<
		0x09,	// (61)	=
		0x0B,	// (62)	>
		0xB1,	// (63)	?
		0xBB,	// (64)	@
		0xF3,	// (65)	A
		0x5B,	// (66)	B
		0x19,	// (67)	C
		0x3B,	// (68)	D
		0xD9,	// (69)	E
		0xD1,	// (70)	F
		0xDA,	// (71)	G
		0x73,	// (72)	H
		0x50,	// (73)	I
		0x3A,	// (74)	J
		0xD3,	// (75)	K
		0x58,	// (76)	L
		0x92,	// (77)	M
		0x13,	// (78)	N
		0x1B,	// (79)	O
		0xF1,	// (80)	P
		0xE3,	// (81)	Q
		0x11,	// (82)	R
		0xCB,	// (83)	S
		0x59,	// (84)	T
		0x7A,	// (85)	U
		0x1A,	// (86)	V
		0x68,	// (87)	W
		0x89,	// (88)	X
		0x6B,	// (89)	Y
		0xB9,	// (90)	Z
		0x68,	// (91)	[
		0x43,	// (92)	\ multi line...
		0x2A,	// (93)	]
		0xE0,	// (94)	^
		0x08,	// (95)	_
		0x40	// (96)	`
		};

#endif /* UI_H_ */
