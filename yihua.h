//	SW2				-| D4 ----- D3 |-	IRON SENSE
//	TX				-| D5		D2 |-	AIR SENSE
//	RX				-| D6		D1 |-	NC
//	NC				-| RST		C7 |-	STB
//	IRONH on=5V 	-| A1		C6 |-	SCLK
//	AIRH on=0V		-| A2		C5 |-	DIO
//	GND				-| GND		C4 |-	AIR REED
//	5V				-| 5V		C3 |-	AIR FAN		on=0V
//	5V				-| 3V3		B4 |-	SCL
//	SW1				-| A3 ----- B5 |-	SDA

#ifndef YIHUA_H_
#define YIHUA_H_

#define STM8S103
#define __SDCC
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm8s.h"
#include "stm8s_itc.h"

#define IRON_HEAT_GPIO GPIOA
#define IRON_HEAT (1<<1)
#define IRON_SW_GPIO GPIOD
#define IRON_SW (1<<4)
#define AIR_HEAT_GPIO GPIOA
#define AIR_HEAT (1<<2)
#define AIR_FAN_GPIO GPIOC
#define AIR_FAN (1<<3)
#define AIR_SW_GPIO GPIOA
#define AIR_SW (1<<3)
#define TM1628_GPIO GPIOC
#define TM1628_STB (1<<7)
#define TM1628_SCLK (1<<6)
#define TM1628_DIO (1<<5)

#define CALSIZE 15
#define CALSTEP 50

typedef struct Fan {
	volatile uint16_t shutdown_raw;
	volatile uint16_t shutdown_deg;
	volatile uint16_t offdelay_sec;
	volatile int16_t offdelay_cnt;

	const void (*enable)(void);
	const void (*disable)(void);

} Fan;


typedef struct SolderingTool {
	volatile uint8_t enable;
	volatile uint8_t connected;

	volatile uint16_t temp_sense_raw;
	volatile uint16_t temp_set_raw;
	volatile uint16_t temp_set_deg;
	volatile uint16_t temp_max_raw;
	volatile uint16_t temp_max_deg;
	volatile int16_t temp_na_thres_raw;
	volatile int16_t temp_diff_last_raw;

	volatile uint16_t pid_kp;
	volatile uint16_t pid_ki;
	volatile uint16_t pid_kd;
	volatile uint16_t pid_ithreshold;
	volatile int32_t pid_i;
	volatile uint8_t sw;

	volatile uint8_t* pwmlow;
	volatile uint8_t* pwmhigh;

	volatile uint16_t adcstart;
	volatile uint16_t calibration[CALSIZE];

	volatile uint8_t show_set;
	volatile int32_t ui_temp;

	volatile uint8_t quicklen;
	volatile uint16_t* quickset_deg;

	Fan* fan;

	const void (*enableheater)(void);
	const void (*disableheater)(void);
	const void (*fireheater)(void);
} SolderingTool;

void showMain(SolderingTool* tool, uint8_t offset);
void ui_setpid(SolderingTool* tool, uint8_t depth) ;
void ui_set8(char* label, uint8_t size, int8_t* val, int8_t min, int8_t max, uint8_t depth) ;
void ui_menu_btn(uint8_t items, uint8_t depth) ;
void ui_set16(char* label, uint8_t size, int16_t* val, int16_t min, int16_t max, uint8_t depth);
void ui_tempset(SolderingTool* tool, uint8_t offset);
void ui_reset(uint8_t from);
void init();
void showui();
void selectValue16(int16_t* val, int16_t min, int16_t max);
void selectValue8(int8_t* val, int8_t min, int8_t max);
void command(char*);
void checkSerial();
void serialWrite(char*);
void serialPut(char);
char serialGet();
void toString(char* buf, int16_t num, uint8_t len);
uint8_t calibrate(SolderingTool*, uint8_t);
uint16_t rawToDeg(SolderingTool*, uint16_t);
uint16_t degToRaw(SolderingTool*, uint16_t);
void quickset(SolderingTool* tool);

void TM1628_init();
void TM1628_sendByte(uint8_t data);
void TM1628_sendCommand(uint8_t data);
void TM1628_clear();
void TM1628_writeDigit(uint8_t digit, uint8_t data);
void TM1628_setBrightness(uint8_t level);
void TM1628_print(uint8_t* data, int8_t start, uint8_t len);
void TM1628_printDot(uint8_t* data, int8_t start, uint8_t len, uint8_t dots);
void TM1628_update();
void TM1628_readkeys();
uint8_t TM1628_readByte();
void TM1628_addDot(int8_t start, uint8_t len);

void IRON_init(void);
void IRON_fireheater(void);
void IRON_enableheater(void);
void IRON_disableheater(void);

void AIR_init(void);
void AIR_fireheater(void);
void AIR_enableheater(void);
void AIR_disableheater(void);
void AIR_enablefan(void);
void AIR_disablefan(void);

#endif /* YIHUA_H_ */
