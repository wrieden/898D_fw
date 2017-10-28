#include "yihua.h"
#include "ui.h"

volatile uint16_t mscounter = 0;
volatile uint16_t keyup;
volatile uint16_t keydown;
volatile uint16_t keyenter;
volatile uint16_t lastkeyup;
volatile uint16_t lastkeydown;
volatile uint16_t lastkeyenter;

volatile uint8_t update_display;
volatile uint8_t update_ui_temp;
volatile uint8_t read_keys;

uint8_t ui_menu_item[5];
bool ui_selected[5];
uint8_t brightness = 1;
uint8_t ui_show_always_temp = 1;
uint8_t ui_show_raw = 0;
uint16_t ui_calibrate_var = 0;
int8_t room_temp = 20;
uint8_t ui_adc_divider = 0;

uint8_t serialcnt = 0;
char *serialbuffer;
uint8_t adccycle;

SolderingTool iron = {
		.enable = 0,
		.connected = 1,
		.temp_sense_raw = 0,
		.temp_set_raw = 0,
		.temp_set_deg = 0,
		.temp_max_raw = 500,
		.temp_max_deg = 350,
		.temp_na_thres_raw = 720,
		.temp_diff_last_raw = 0,
		.pid_kp = 500,
		.pid_ki = 1,
		.pid_kd = 0,
		.pid_ithreshold = 45,
		.pid_i = 0,
		.sw = 0,

		.pwmlow = &TIM2->CCR1L,
		.pwmhigh = &TIM2->CCR1H,

		.adcstart = 128,
		.calibration = { 0, 42, 84, 128, 170, 212, 260, 307, 360, 414, 466, 520, 670, 750, 800 },

		.show_set = 0,
		.ui_temp = 0,
		.quicklen = 5,
		.enableheater = &IRON_enableheater,
		.disableheater = &IRON_disableheater,
		.fireheater = &IRON_fireheater,
		.fan = 0
};

Fan air_fan = {
		.shutdown_raw = 0,
		.shutdown_deg = 80,
		.offdelay_sec = 10,
		.offdelay_cnt = -1,
		.enable = &AIR_enablefan,
		.disable = &AIR_disablefan
};

SolderingTool air = {
		.enable = 0,
		.connected = 1,
		.temp_sense_raw = 0,
		.temp_set_raw = 0,
		.temp_set_deg = 0,
		.temp_max_raw = 500,
		.temp_max_deg = 350,
		.temp_na_thres_raw = 720,
		.temp_diff_last_raw = 0,
		.pid_kp = 500,
		.pid_ki = 1,
		.pid_kd = 0,
		.pid_ithreshold = 45,
		.pid_i = 0,
		.sw = 0,

		.pwmlow = &TIM2->CCR2L,
		.pwmhigh = &TIM2->CCR2H,

		.adcstart = 128,
		.calibration = { 0, 42, 84, 128, 170, 212, 260, 307, 360, 414, 466, 520, 670, 750, 800 },

		.show_set = 0,
		.ui_temp = 0,
		.quicklen = 5,
		.enableheater = &AIR_enableheater,
		.disableheater = &AIR_disableheater,
		.fireheater = &AIR_fireheater,
		.fan = &air_fan
};



void main(void) {
	serialbuffer = malloc(100);

	air.fan->shutdown_raw = degToRaw(&air, air.fan->shutdown_deg);
	iron.quickset_deg = malloc(sizeof(*iron.quickset_deg) * iron.quicklen);
	air.quickset_deg = malloc(sizeof(*air.quickset_deg) * air.quicklen);

	iron.quickset_deg[0] = 50;
	iron.quickset_deg[1] = 100;
	iron.quickset_deg[2] = 150;
	iron.quickset_deg[3] = 200;
	iron.quickset_deg[4] = 250;

	air.quickset_deg[0] = 50;
	air.quickset_deg[1] = 100;
	air.quickset_deg[2] = 150;
	air.quickset_deg[3] = 200;
	air.quickset_deg[4] = 250;

	ui_reset(0);

	init();
	TM1628_init();
	IRON_init();
	AIR_init();

	enableInterrupts()
	;

	serialWrite("YIHUA Solder Station\n\r");

	while (1) {
		checkSerial();

		if (read_keys) {
			TM1628_readkeys();

			if (iron.show_set) {
				iron.show_set--;
			}

			if (air.show_set) {
				air.show_set--;
			}

			read_keys = 0;
		}

		if (update_ui_temp) {
			if (ui_show_raw) {
				iron.ui_temp = iron.temp_sense_raw;
				air.ui_temp = air.temp_sense_raw;
			} else {
				iron.ui_temp = rawToDeg(&iron, iron.temp_sense_raw);
				air.ui_temp = rawToDeg(&air, air.temp_sense_raw);
			}

			update_ui_temp = 0;
		}

		if (update_display) {
			showui();
			update_display = 0;
		}

	}
}

void showui() {
	switch (ui_menu_item[0]) {
	case UI_MAIN:
		showMain(&air, 0);
		showMain(&iron, 3);

		if (keydown >= MIDDLEPRESS) {
			ui_menu_item[0] = UI_AIRQUICK;
			air.show_set = SHOW_SET_TEMP;
		} else if (lastkeydown >= SHORTPRESS) {
			if (air.show_set) {
				quickset(&air);
			}
			air.show_set = SHOW_SET_TEMP;
		}

		if (keyup >= MIDDLEPRESS) {
			ui_menu_item[0] = UI_IRONQUICK;
			iron.show_set = SHOW_SET_TEMP;
		} else if (lastkeyup >= SHORTPRESS) {
			if (iron.show_set) {
				quickset(&iron);
			}
			iron.show_set = SHOW_SET_TEMP;
		}

		if (lastkeyenter >= SHORTPRESS) {
			ui_menu_item[0] = UI_MENU;
		}

		break;

	case UI_AIRQUICK:
		ui_tempset(&air, 0);
		break;

	case UI_IRONQUICK:
		ui_tempset(&iron, 3);
		break;

	case UI_MENU:
		switch (ui_menu_item[1]) {
		case UI_MENU_BACK:
			TM1628_print("$$    ", 0, 6);
			if (ui_selected[1]) {
				ui_reset(0);
			}
			break;

		case UI_MENU_AIR:
			TM1628_print("AIR ++", 0, 6);
			if (ui_selected[1]) {
				switch (ui_menu_item[2]) {
				case UI_MENU_AIR_BACK:
					TM1628_print("$$    ", 0, 6);
					if (ui_selected[2]) {
						ui_reset(2);
					}
					break;

				case UI_MENU_AIR_FANSHUTDOWNTEMP:
					ui_set16("FSD", 3, &air.fan->shutdown_deg, 0, 255, 2);
					air.fan->shutdown_raw = degToRaw(&air, air.fan->shutdown_deg);
					break;

				case UI_MENU_AIR_FANDELAY:
					ui_set16("FD ", 3, &air.fan->offdelay_sec, 0, 255, 2);
					break;

				case UI_MENU_AIR_PID:
					TM1628_print("PID ++", 0, 6);

					if (ui_selected[2]) {
						ui_setpid(&air, 3);
						ui_menu_btn(4, 3);
					}
					break;

				case UI_MENU_AIR_CALIBRATE:
					TM1628_print("CAL ++", 0, 6);

					if (ui_selected[2]) {
						calibrate(&air, 3);
					}
					break;

				}

				ui_menu_btn(4, 2);

				break;
			}
			break;

		case UI_MENU_IRON:
			TM1628_print("IRO ++", 0, 6);
			if (ui_selected[1]) {
				switch (ui_menu_item[2]) {
				case UI_MENU_IRON_BACK:
					TM1628_print("$$    ", 0, 6);
					if (ui_selected[2]) {
						ui_reset(2);
					}
					break;
				case UI_MENU_IRON_PID:
					TM1628_print("PID ++", 0, 6);

					if (ui_selected[2]) {
						ui_setpid(&iron, 3);
						ui_menu_btn(4, 3);
					}
					break;

				case UI_MENU_IRON_CALIBRATE:
					TM1628_print("CAL ++", 0, 6);

					if (ui_selected[2]) {
						calibrate(&iron, 3);
					}
					break;
				}

				ui_menu_btn(2, 2);

				break;
			}
			break;

		case UI_MENU_ROOMTEMP:
			ui_set8("RT ", 3, &room_temp, -99, 99, 1);
			break;

		case UI_MENU_SHOWRAW:
			ui_set8("RAW", 3, &ui_show_raw, 0, 1, 1);
			break;

		case UI_MENU_BRIGHTNESS:
			ui_set8("LED", 3, &brightness, 1, 8, 1);
			TM1628_setBrightness(brightness);
			break;

		case UI_MENU_SHOWALWAYSTEMP:
			ui_set8("SHT", 3, &ui_show_always_temp, 0, 1, 1);
			break;
		}
		ui_menu_btn(6, 1);

		break;
	}

	lastkeyup = 0;
	lastkeydown = 0;
	lastkeyenter = 0;

	TM1628_update();
}

void showMain(SolderingTool* tool, uint8_t offset) {
	if (tool->show_set) {
		toString(TM1628_charbuffer, tool->temp_set_deg, 3);
		TM1628_printDot(TM1628_charbuffer, offset, 3, 0x7);
	} else if ((ui_show_always_temp || tool->enable || (tool->fan && tool->fan->offdelay_cnt != -1))) {
		toString(TM1628_charbuffer, tool->ui_temp, 3);
		TM1628_printDot(TM1628_charbuffer, offset, 3, 0);
	} else if (!tool->connected) {
		TM1628_print("ERR", offset, 3);
	} else {
		TM1628_print("---", offset, 3);
	}
}

void ui_tempset(SolderingTool* tool, uint8_t offset) {
	toString(TM1628_charbuffer, tool->temp_set_deg, 3);
	TM1628_printDot(TM1628_charbuffer, offset, 3, 0x7);
	TM1628_print("SET", offset ? 0 : 3, 3);

	selectValue16(&tool->temp_set_deg, 0, 999);

	tool->temp_set_raw = degToRaw(tool, tool->temp_set_deg);

	if (keyup || keydown || lastkeyup || lastkeydown) {
		tool->show_set = SHOW_SET_TEMP;
	}

	if (!tool->show_set || lastkeyenter >= SHORTPRESS) {
		ui_reset(0);
		tool->show_set = 0;
	}
}

void ui_reset(uint8_t from) {
	memset(&ui_menu_item + from, 1, 5 - from);
	if (from) {
		from--;
	}
	memset(&ui_selected + from, 0, 5 - from);
}

void ui_setpid(SolderingTool* tool, uint8_t depth) {
	switch (ui_menu_item[depth]) {
	case UI_PID_BACK:
		TM1628_print("$$    ", 0, 6);
		if (ui_selected[depth]) {
			ui_reset(depth);
		}
		break;
	case UI_PID_KP:
		ui_set16("P", 1, &tool->pid_kp, 0, 0x7FFF, depth);
		break;

	case UI_PID_KI:
		ui_set16("I", 1, &tool->pid_ki, 0, 0x7FFF, depth);
		break;

	case UI_PID_KD:
		ui_set16("D", 1, &tool->pid_kd, 0, 0x7FFF, depth);
		break;

	case UI_PID_ITHRESHOLD:
		ui_set16("ITH", 3, &tool->pid_ithreshold, 0, 999, depth);
		break;
	}

}

void ui_set8(char* label, uint8_t size, int8_t* val, int8_t min, int8_t max, uint8_t depth) {
	TM1628_print(label, 0, size);
	toString(TM1628_charbuffer, *val, 6 - size);

	if (ui_selected[depth]) {
		selectValue8(val, min, max);

		if (lastkeyenter >= SHORTPRESS) {
			ui_selected[depth] = 0;
			lastkeyenter = 0;
		}
		TM1628_printDot(TM1628_charbuffer, size, 6 - size, 0x07 << (size - 3));
	} else {
		TM1628_print(TM1628_charbuffer, size, 6 - size);
	}

}

void ui_menu_btn(uint8_t items, uint8_t depth) {
	if (!ui_selected[depth]) {
		selectValue8(&ui_menu_item[depth], 0, items);

		if (lastkeyenter >= MIDDLEPRESS) {
			ui_reset(0);
		} else if (lastkeyenter >= SHORTPRESS) {
			ui_selected[depth] = 1;
		}
	}

}

void ui_set16(char* label, uint8_t size, int16_t* val, int16_t min, int16_t max, uint8_t depth) {
	TM1628_print(label, 0, size);
	toString(TM1628_charbuffer, *val, 6 - size);

	if (ui_selected[depth]) {
		selectValue16(val, min, max);
		if (lastkeyenter >= SHORTPRESS) {
			ui_selected[depth] = 0;
			lastkeyenter = 0;
		}
		TM1628_printDot(TM1628_charbuffer, size, 6 - size, 0x07 << (size - 3));
	} else {
		TM1628_print(TM1628_charbuffer, size, 6 - size);
	}
}

uint8_t calibrate(SolderingTool* tool, uint8_t depth) {
	if (ui_menu_item[depth] == 1) {
		tool->temp_set_raw = 0;
		ui_show_raw = 1;

		TM1628_print("HW ", 0, 3);
		toString(TM1628_charbuffer, tool->ui_temp, 3);
		TM1628_printDot(TM1628_charbuffer, 3, 3, 0);
	} else if (ui_menu_item[depth] == 2) {
		TM1628_print("RT ", 0, 3);
		toString(TM1628_charbuffer, room_temp, 3);
		TM1628_printDot(TM1628_charbuffer, 3, 3, 0x7);
		selectValue8(&room_temp, -99, 99);
		ui_calibrate_var = room_temp;
	} else {
		tool->temp_set_raw = tool->adcstart + ((ui_menu_item[depth] - 2) * 50);
		toString(TM1628_charbuffer, tool->ui_temp, 3);
		toString(TM1628_charbuffer + 3, ui_calibrate_var, 3);
		TM1628_printDot(TM1628_charbuffer, 0, 6, 0x38);
		selectValue16(&ui_calibrate_var, 0, 999);
	}

	if (lastkeyenter >= SHORTPRESS) {
		if (ui_menu_item[depth]) {
			tool->calibration[ui_menu_item[depth] - 2] = ui_calibrate_var - room_temp;
		} else {
			tool->adcstart = tool->ui_temp;
		}

		if ((ui_menu_item[depth] > 11) || ((ui_menu_item[depth] > 4) &&
				(((tool->calibration[ui_menu_item[depth] - 2] << 1) - tool->calibration[ui_menu_item[depth] - 3] + room_temp) > tool->temp_max_raw))) {
			ui_show_raw = 0;
			return 1;
		} else {
			ui_menu_item[depth]++;
		}
		lastkeyenter = 0;
	}
	return 0;
}

uint16_t rawToDeg(SolderingTool* tool, uint16_t raw) {
	uint8_t rawpos, rawrange, degrange;
	uint16_t rawlow, rawhigh, deglow, deghigh;

	if (raw > tool->adcstart) {
		raw = raw - tool->adcstart;
	} else {
		raw = 0;
	}

	rawpos = raw / CALSTEP;

	deglow = tool->calibration[rawpos];
	deghigh = tool->calibration[rawpos + 1];
	degrange = deghigh - deglow;

	rawlow = rawpos * CALSTEP;
	rawhigh = rawlow + CALSTEP;
	rawrange = rawhigh - rawlow;

	raw -= rawlow;

	return (((raw << 8) / rawrange * degrange) >> 8) + deglow + room_temp;
}

uint16_t degToRaw(SolderingTool* tool, uint16_t deg) {
	uint8_t i, rawrange, degrange;
	uint16_t rawlow, rawhigh, deglow, deghigh;

	if (deg > room_temp) {
		deg = deg - room_temp;
	} else {
		deg = 0;
	}

	for (i = 1; (i < CALSIZE) && (deg >= tool->calibration[i]); i++) {

	}

	deglow = tool->calibration[i - 1];
	deghigh = tool->calibration[i];
	degrange = deghigh - deglow;

	rawlow = (CALSTEP * (i - 1));
	rawhigh = rawlow + CALSTEP;
	rawrange = rawhigh - rawlow;

	deg = deg - deglow;

	return (((deg << 8) / degrange * rawrange) >> 8) + rawlow + tool->adcstart;

}

void selectValue8(int8_t* val, int8_t min, int8_t max) {
	int16_t tmp = *val;
	selectValue16(&tmp, min, max);
	*val = tmp;
}

void selectValue16(int16_t* val, int16_t min, int16_t max) {
	if (lastkeyup >= SHORTPRESS && *val < max) {
		*val = *val + 1;
	}
	if (keyup >= MIDDLEPRESS) {
		int16_t next = *val + (1 << (keyup >> 6));

		if (next < max && next > *val) {
			*val = next;
		}
		else {
			*val = max;
		}

	}

	if (lastkeydown >= SHORTPRESS && *val > min) {
		*val = *val - 1;
	}
	if (keydown >= MIDDLEPRESS) {
		int16_t next = *val - (1 << (keydown >> 6));

		if (next > min && next < *val) {
			*val = next;
		}
		else {
			*val = min;
		}

	}
}

void quickset(SolderingTool* tool) {
	uint8_t i = 0;
	while (i < tool->quicklen) {
		if (tool->quickset_deg[i] > tool->temp_set_deg) {
			tool->temp_set_deg = tool->quickset_deg[i];
			break;
		}
		i++;
	}
	if (i == tool->quicklen) {
		tool->temp_set_deg = tool->quickset_deg[0];
	}
	tool->temp_set_raw = degToRaw(tool, tool->temp_set_deg);

}

void controller(SolderingTool* tool) {
	int16_t temp_diff_raw;
	int32_t pid_out;

	if (tool->enable) {
		temp_diff_raw = tool->temp_set_raw - tool->temp_sense_raw;

		if (abs(temp_diff_raw) < tool->pid_ithreshold) {
			tool->pid_i += temp_diff_raw;
		} else {
			tool->pid_i = 0;
		}

		pid_out = (int32_t) tool->pid_kp * temp_diff_raw;
		pid_out += (int32_t) tool->pid_ki * tool->pid_i;
		pid_out += (int32_t) tool->pid_kd * (tool->temp_diff_last_raw - temp_diff_raw);

		if (pid_out < 0) {
			pid_out = 0;
		} else if (pid_out > 0xFFFF) {
			pid_out = 0xFFFF;
		}
		*tool->pwmhigh = (uint8_t) (pid_out >> 8);
		*tool->pwmlow = (uint8_t) pid_out;

		tool->temp_diff_last_raw = temp_diff_raw;
	}
	else {
		*tool->pwmhigh = 0;
		*tool->pwmlow = 0;
		tool->temp_diff_last_raw = 0;
		tool->pid_i = 0;

		if (tool->fan) {
			if (tool->temp_sense_raw <= tool->fan->shutdown_raw) {
				if (tool->fan->offdelay_cnt != -1) {
					if (!tool->fan->offdelay_cnt) {
						tool->fan->disable();
					}
					tool->fan->offdelay_cnt--;
				} else {
					tool->fan->offdelay_cnt = tool->fan->offdelay_sec * 100;
				}
			} else {
				tool->fan->offdelay_cnt = -1;
			}
		}

	}

}

void checkSerial() {
	if (UART1->SR & UART1_SR_RXNE) {
		char recvbyte = UART1->DR;

		switch (recvbyte) {
		case 127: // del
		case '\b':
			if (serialcnt) {
				serialcnt--;
				serialbuffer[serialcnt] = 0;
				serialWrite("\b \b");
			}
			break;
		case '\n':
			case '\r':
			if (serialcnt) {
				serialWrite("\n\r");
				command(serialbuffer);
				memset(serialbuffer, 0, serialcnt);
				serialcnt = 0;
			}
			break;
		default:
			serialPut(recvbyte);
			serialbuffer[serialcnt] = recvbyte;
			serialcnt++;
		}
	}

}

void command(char* cmd) {
	char out[10] = { 0 };
	int8_t argc = 0;
	char* argv[10];
	argv[0] = strtok(cmd, " ");

	do {
		argc++;
		argv[argc] = strtok(0, " ");
	} while (argv[argc] != 0 && argc < 10);

	if (strcmp(cmd, "brightness") == 0 && argc == 2) {
		TM1628_setBrightness(atoi(argv[1]));
		serialWrite("done\n\r");
	} else if (strcmp(cmd, "irontemp") == 0 && argc == 2) {
		iron.temp_set_raw = atoi(argv[1]);
		serialWrite("done\n\r");
	} else if (strcmp(cmd, "rawToDeg") == 0 && argc == 2) {
		toString(out, rawToDeg(&air, atoi(argv[1])), 3);
		serialWrite(out);
		serialWrite("\n\r");
	} else if (strcmp(cmd, "degToRaw") == 0 && argc == 2) {
		toString(out, degToRaw(&air, atoi(argv[1])), 3);
		serialWrite(out);
		serialWrite("\n\r");
	} else if (strcmp(cmd, "eeread") == 0 && argc == 2) {
//		I2C->CR2 |= I2C_CR2_START;
//		while (!(I2C->SR1 & I2C_SR1_SB));
//			  I2C->DR = 0xA1;                    // zu schickende I2C-Bus Adresse (LM75)
//			  I2C->OARL = 0;
//			  I2C->OARH = 0;
//			  while (!(I2C->SR1 & I2C_SR1_ADDR));
//
//
//			  I2C->CR2 |= I2C_CR2_STOP;
	} else {

		serialWrite("Unknown Command!\n\r");
	}
}

void serialWrite(char* data) {
	for (uint8_t i = 0; data[i] != 0; i++) {
		serialPut(data[i]);
	}
}

void serialPut(char c) {
	while (!(UART1->SR & UART1_SR_TXE))
		;
	UART1->DR = c;
	//while (!(UART1->SR & UART1_SR_TXE));
}

void init() {
	//CLK
	CLK->CKDIVR = 0x00; // 16MHz
	CLK->PCKENR1 = 0XFF;
	CLK->PCKENR2 = 0xFF;

//	//GPIO
//	GPIOA->CR1 = GPIO_CR1_RESET_VALUE;
//	GPIOA->CR2 = GPIO_CR2_RESET_VALUE;
//	GPIOA->ODR = GPIO_ODR_RESET_VALUE;
//	GPIOA->DDR = GPIO_DDR_RESET_VALUE;
//
//	GPIOB->CR1 = GPIO_CR1_RESET_VALUE;
//	GPIOB->CR2 = GPIO_CR2_RESET_VALUE;
//	GPIOB->ODR = GPIO_ODR_RESET_VALUE;
//	GPIOB->DDR = GPIO_DDR_RESET_VALUE;
//
//	GPIOC->CR1 = GPIO_CR1_RESET_VALUE;
//	GPIOC->CR2 = GPIO_CR2_RESET_VALUE;
//	GPIOC->ODR = GPIO_ODR_RESET_VALUE;
//	GPIOC->DDR = GPIO_DDR_RESET_VALUE;
//
//	GPIOD->CR1 = GPIO_CR1_RESET_VALUE;
//	GPIOD->CR2 = GPIO_CR2_RESET_VALUE;
//	GPIOD->ODR = GPIO_ODR_RESET_VALUE;
//	GPIOD->DDR = GPIO_DDR_RESET_VALUE;

	//UART 115200 8b1
	UART1->BRR2 = 0x0B;
	UART1->BRR1 = 0x08;
	UART1->CR2 = UART1_CR2_RESET_VALUE | UART1_CR2_TEN | UART1_CR2_REN;

	//TIM1 adc1 200Hz (100Hz per channel)
	TIM1->CR1 = TIM1_CR1_RESET_VALUE | TIM1_CR1_URS | TIM1_CR1_ARPE;
	TIM1->CR2 = TIM1_CR2_RESET_VALUE | (TIM1_CR2_MMS & 0x20);
	TIM1->PSCRH = 1000 >> 8;
	TIM1->PSCRL = 1000 & 0xFF;
	//TIM1->EGR = TIM1_EGR_RESET_VALUE | TIM1_EGR_UG;
	TIM1->ARRH = 0; //80 >> 8;
	TIM1->ARRL = 80 & 0xFF;
	TIM1->CR1 |= TIM1_CR1_CEN;

	//TIM2 iron pwm
	TIM2->CR1 = TIM2_CR1_RESET_VALUE | TIM2_CR1_URS | TIM2_CR1_ARPE;
	//TIM1->CR2 = TIM1_CR2_RESET_VALUE | (TIM1_CR2_MMS & 0x20);
	TIM2->IER = TIM2_IER_RESET_VALUE | TIM2_IER_UIE | TIM2_IER_CC1IE | TIM2_IER_CC2IE;
	TIM2->PSCR = TIM2_PSCR_RESET_VALUE | 4;
	TIM2->CCMR1 = TIM2_CCMR1_RESET_VALUE | TIM2_CCMR_OCxPE;
	TIM2->CCMR2 = TIM2_CCMR2_RESET_VALUE | TIM2_CCMR_OCxPE;
	//TIM2->EGR = TIM2_EGR_RESET_VALUE | TIM2_EGR_UG;
	TIM2->ARRH = 0xFF; //80 >> 8;
	TIM2->ARRL = 0xFF;
	TIM2->CCR1H = 0x00;
	TIM2->CCR1L = 0x00;
	TIM2->CCR2H = 0x00;
	TIM2->CCR2L = 0x00;
	TIM2->CR1 |= TIM2_CR1_CEN;

	//TIM 4 mscounter
	TIM4->CR1 = TIM4_CR1_RESET_VALUE | TIM4_CR1_ARPE;
	TIM4->ARR = 125;
	TIM4->PSCR = 0x07;
	TIM4->IER = TIM4_IER_RESET_VALUE | TIM4_IER_UIE;
	//TIM4->EGR = TIM4_EGR_RESET_VALUE | TIM4_EGR_UG;
	TIM4->CR1 = TIM4_CR1_CEN;

	//ADC1
	ADC1->CR1 = ADC1_CR1_RESET_VALUE | ADC1_CR1_SPSEL;
	ADC1->CR2 = ADC1_CR2_RESET_VALUE | ADC1_CR2_ALIGN | ADC1_CR2_EXTTRIG;
	ADC1->CSR = ADC1_CSR_RESET_VALUE | ADC1_CSR_EOCIE | 4;
	//ADC1->CR3 = ADC1_CR3_DBUF;
	ADC1->CR1 |= ADC1_CR1_ADON;

	//I2C

//	I2C->FREQR = 16;
//	I2C->CCRL = 16;
//	I2C->CCRH = 0x0b;
//	I2C->TRISER = 2;
//I2C->CR1 = I2C_CR1_RESET_VALUE | I2C_CR1_PE;
//I2C->CR2 = I2C_CR2_RESET_VALUE| I2C_CR2_ACK;      // ACK nach Transmit abfragen

	//ITC
	ITC->ISPR1 = ITC_SPRX_RESET_VALUE | (ITC_PRIORITYLEVEL_1 << 6);		//PORTA
	ITC->ISPR2 = ITC_SPRX_RESET_VALUE | (ITC_PRIORITYLEVEL_1 << 4);		//PORTD
	ITC->ISPR6 = ITC_SPRX_RESET_VALUE | (ITC_PRIORITYLEVEL_2 << 6) | (ITC_PRIORITYLEVEL_2 << 4);		//TIM4, ADC1

}

void TM1628_init() {
	TM1628_GPIO->CR1 &= ~(TM1628_STB | TM1628_SCLK | TM1628_DIO);
	TM1628_GPIO->CR2 &= ~(TM1628_STB | TM1628_SCLK | TM1628_DIO);
	TM1628_GPIO->ODR |= (TM1628_STB | TM1628_SCLK | TM1628_DIO);
	TM1628_GPIO->DDR |= (TM1628_STB | TM1628_SCLK | TM1628_DIO);

	TM1628_sendCommand(0x03); // 7grid 11seg
	TM1628_sendCommand(0x40); // normal, inc, write
	// sendCommand(0x8F); // Turn on full bright

	TM1628_clear();

	TM1628_setBrightness(brightness);

}

void TM1628_sendByte(uint8_t data) {
	for (uint8_t i = 0; i < 8; i++) {
		TM1628_GPIO->ODR &= ~TM1628_SCLK;
		if (data & 1) {
			TM1628_GPIO->ODR |= TM1628_DIO;
		} else {
			TM1628_GPIO->ODR &= ~ TM1628_DIO;
		}
		data >>= 1;
		TM1628_GPIO->ODR |= TM1628_SCLK;
	}
	TM1628_GPIO->ODR |= TM1628_DIO;
}

uint8_t TM1628_readByte() {
	uint8_t buf = 0;
	for (uint8_t i = 0; i < 8; i++) {
		TM1628_GPIO->ODR &= ~TM1628_SCLK;
		buf >>= 1;
		if (TM1628_GPIO->IDR & TM1628_DIO) {
			buf |= 0x80;
		}
		TM1628_GPIO->ODR |= TM1628_SCLK;
	}
	return buf;
}

void TM1628_sendCommand(uint8_t data) {
	TM1628_GPIO->ODR &= ~ TM1628_STB;
	TM1628_sendByte(data);
	TM1628_GPIO->ODR |= TM1628_STB;
}

void TM1628_writeDigit(uint8_t digit, uint8_t data) {
	uint8_t tmp = data;
	TM1628_sendCommand(0x44);
	TM1628_GPIO->ODR &= ~ TM1628_STB;
	TM1628_sendByte(0xC0 | digit);
	TM1628_sendByte(tmp);
	TM1628_GPIO->ODR |= TM1628_STB;
}

void TM1628_print(uint8_t* data, int8_t start, uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		TM1628_buffer[i + start] = TM1628_ASCII[data[i] - ' '];
		if (data[i + 1] == '.') {
			TM1628_buffer[i + start] |= 0x04;
			i++;
			start--;
		}
	}
}

void TM1628_printDot(uint8_t* data, int8_t start, uint8_t len, uint8_t dots) {
	for (uint8_t i = 0; i < len; i++) {
		TM1628_buffer[i + start] = TM1628_ASCII[data[i] - ' '];
		if (dots & 1) {
			TM1628_buffer[i + start] |= 0x04;
		}
		dots >>= 1;
	}
}

void TM1628_addDot(int8_t start, uint8_t len) {
	for (uint8_t i = start; i < len + start; i++) {
		TM1628_buffer[i] |= 0x04;
	}
}

void TM1628_update() {
	TM1628_sendCommand(0x40);
	TM1628_GPIO->ODR &= ~ TM1628_STB;
	TM1628_sendByte(0xC2);
	for (uint8_t i = 5; i != 255; i--) {
		TM1628_sendByte(TM1628_buffer[i]);
		TM1628_sendByte(0x00);
	}
	TM1628_GPIO->ODR |= TM1628_STB;
}

void TM1628_readkeys() {
	TM1628_GPIO->ODR &= ~ TM1628_STB;
	TM1628_sendByte(0x42);
	TM1628_GPIO->DDR &= ~ TM1628_DIO;
	TM1628_GPIO->ODR |= TM1628_DIO;

	for (uint8_t i = 0; i < 3; i++) {
		uint8_t buf = TM1628_readByte();
		if (i == 1) {
			if (buf & 0x08) {
				if (keyenter < 0xFFFF) {
					keyenter++;
				}
			} else if (keyenter) {
				lastkeyenter = keyenter;
				keyenter = 0;
			}
		}
		if (i == 2) {
			if (buf & 0x01) {
				if (keydown < 0xFFFF) {
					keydown++;
				}
			} else if (keydown) {
				lastkeydown = keydown;
				keydown = 0;
			}

			if (buf & 0x08) {
				if (keyup < 0xFFFF) {
					keyup++;
				}
			} else if (keyup) {
				lastkeyup = keyup;
				keyup = 0;
			}
		}
	}

	TM1628_GPIO->ODR |= TM1628_STB;
	TM1628_GPIO->DDR |= TM1628_DIO;
}

void TM1628_clear() {
	TM1628_GPIO->ODR &= ~ TM1628_STB;
	TM1628_sendByte(0xC0);
	for (uint8_t i = 0; i < 14; i++) {
		TM1628_sendByte(0x00);
	}
	TM1628_GPIO->ODR |= TM1628_STB;
}

void TM1628_setBrightness(uint8_t level) {
	uint8_t cmd = level ? (0x88 | (level - 1)) : 0x80;
	TM1628_sendCommand(cmd & 0x8F);
}

void IRON_init(void) {
	IRON_HEAT_GPIO->CR1 |= IRON_HEAT;
	IRON_HEAT_GPIO->CR2 &= ~ IRON_HEAT;
	IRON_HEAT_GPIO->ODR &= ~ IRON_HEAT;
	IRON_HEAT_GPIO->DDR |= IRON_HEAT;

	IRON_SW_GPIO->CR1 |= IRON_SW;
	IRON_SW_GPIO->CR2 |= IRON_SW;
	IRON_SW_GPIO->ODR &= ~ IRON_SW;
	IRON_SW_GPIO->DDR &= ~ IRON_SW;
	EXTI->CR1 |= EXTI_CR1_PDIS;

}

void IRON_fireheater(void) {
	IRON_HEAT_GPIO->ODR |= IRON_HEAT;
	IRON_HEAT_GPIO->ODR &= ~ IRON_HEAT;
}

void IRON_enableheater(void) {
	IRON_HEAT_GPIO->ODR |= IRON_HEAT;
}

void IRON_disableheater(void) {
	IRON_HEAT_GPIO->ODR &= ~ IRON_HEAT;
}

void AIR_init(void) {
	AIR_HEAT_GPIO->CR1 |= AIR_HEAT;
	AIR_HEAT_GPIO->CR2 &= ~ AIR_HEAT;
	AIR_HEAT_GPIO->ODR |= AIR_HEAT;
	AIR_HEAT_GPIO->DDR |= AIR_HEAT;

	AIR_FAN_GPIO->CR1 |= AIR_FAN;
	AIR_FAN_GPIO->CR2 &= ~ AIR_FAN;
	AIR_FAN_GPIO->ODR |= AIR_FAN;
	AIR_FAN_GPIO->DDR |= AIR_FAN;

	AIR_SW_GPIO->CR1 |= AIR_SW;
	AIR_SW_GPIO->CR2 |= AIR_SW;
	AIR_SW_GPIO->ODR &= ~ AIR_SW;
	AIR_SW_GPIO->DDR &= ~ AIR_SW;
	EXTI->CR1 |= EXTI_CR1_PAIS;
}

void AIR_fireheater(void) {
	AIR_HEAT_GPIO->ODR &= ~ AIR_HEAT;
	AIR_HEAT_GPIO->ODR |= AIR_HEAT;
}

void AIR_enableheater(void) {
	AIR_HEAT_GPIO->ODR &= ~ AIR_HEAT;
}

void AIR_disableheater(void) {
	AIR_HEAT_GPIO->ODR |= AIR_HEAT;
}

void AIR_enablefan(void) {
	AIR_FAN_GPIO->ODR &= ~ AIR_FAN;
}

void AIR_disablefan(void) {
	AIR_FAN_GPIO->ODR |= AIR_FAN;
}

void PORTA_IRQHandler()
__interrupt( ITC_IRQ_PORTA) {
	if (AIR_SW_GPIO->IDR & AIR_SW) {
		air.enable = 0;
	} else {
		air.enable = 1;
		AIR_enablefan();
	}
}

void PORTD_IRQHandler()
__interrupt( ITC_IRQ_PORTD) {
	if (IRON_SW_GPIO->IDR & IRON_SW) {
		iron.enable = 0;
	} else {
		iron.enable = 1;
	}
}

void TIM2_OVF_IRQHandler()
__interrupt( ITC_IRQ_TIM2_OVF) {
	if (iron.enable && (*iron.pwmhigh || *iron.pwmlow)) {
		iron.enableheater();
	}
	if (air.enable && (*air.pwmhigh || *air.pwmlow)) {
		air.fan->enable();
		air.enableheater();
	}

	TIM2->SR1 &= ~ TIM2_SR1_UIF;
}

void TIM2_CAPCOM_IRQHandler()
__interrupt( ITC_IRQ_TIM2_CAPCOM) {
	if (TIM2->SR1 & TIM2_SR1_CC1IF) {
		IRON_disableheater();
		TIM2->SR1 &= ~ TIM2_SR1_CC1IF;
	}

	if (TIM2->SR1 & TIM2_SR1_CC2IF) {
		AIR_disableheater();
		TIM2->SR1 &= ~ TIM2_SR1_CC2IF;
	}
}

void TIM4_OVF_IRQHandler()
__interrupt( ITC_IRQ_TIM4_OVF) {
	mscounter++;

	if ((mscounter & 0x1F) == 0x1F) { 		// 32.25 Hz
		read_keys = 1;
		update_display = 1;
		if ((mscounter & 0xFF) == 0xFF) { 	// 3.9 Hz
			update_ui_temp = 1;

		}
	}

	TIM4->SR1 &= ~ TIM4_SR1_UIF;
}

void ADC_IRQHandler()
__interrupt( ITC_IRQ_ADC1) {
	uint16_t sensor = ADC1->DRL | (ADC1->DRH << 8);
	SolderingTool* tool;

	if (!adccycle) {
		tool = &iron;
		ADC1->CSR &= ~ ADC1_CSR_CH;
		ADC1->CSR |= ADC1_CSR_CH & 3;
	} else {
		tool = &air;
		ADC1->CSR &= ~ ADC1_CSR_CH;
		ADC1->CSR |= ADC1_CSR_CH & 4;
	}

	tool->temp_sense_raw = sensor;
	if (sensor > tool->temp_na_thres_raw) {
		tool->connected = 0;
		tool->enable = 0;
	} else if (sensor > tool->temp_max_raw) {
		tool->enable = 0;
	} else {
		tool->connected = 1;
	}

	controller(tool);

	adccycle = !adccycle;
	ADC1->CSR &= ~ ADC1_CSR_EOC;
}

void toString(char* buf, int16_t num, uint8_t len) {
	//buf[len]= '\0';

	if (num < 0) {
		buf[0] = '-';
		num = -num;
		buf += len - 1;
		len--;
	} else {
		buf += len - 1;
	}

	for (uint8_t i = 0; i < len; i++) {
		if (num || !i) {
			*buf = '0' + num % 10;
		} else {
			*buf = ' ';
		}
		num /= 10;
		buf--;
	}

}
