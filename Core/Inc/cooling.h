/*
 * cooling.h
 *
 *  Created on: Apr 18, 2026
 *      Author: ishanchitale
 */

#ifndef INC_COOLING_H_
#define INC_COOLING_H_

#include "fdcan.h"
#include "vcu_state.h"

typedef union COOLING_CMD_DF {
	struct __attribute__((packed)) {
		uint8_t cooling_en;
		uint8_t tractive_fan_pwm;
		uint8_t tractive_pump_pwm;
		uint8_t accy_fan_pwm;
		uint8_t reserved4;
		uint8_t reserved5;
		uint8_t reserved6;
		uint8_t reserved7;
	} data;
	uint8_t array[8];
} COOLING_CMD_DF;

#define COOLING_CMD_RATE_LIMIT_MS 500

#define IDLE_TRACTIVE_FAN_PWM 10
#define IDLE_TRACTIVE_PUMP_PWM 50
#define IDLE_ACCY_FAN_PWM 10

#define INVERTER_TEMP_THRESHOLD_L 20
#define INVERTER_TEMP_THRESHOLD_H 50
#define BMS_TEMP_THRESHOLD_L 20
#define BMS_TEMP_THRESHOLD_H 55

void configureCoolingCmdMsg();
void processBMS_Temp();
void processInverter_Temp();
void sendCoolingCmd();
void calculateTractiveFanPWM(float inverter_temp);
void calculateTractivePumpPWM(float inverter_temp);
void calculateAccyFanPWM(float bms_temp);

#endif /* INC_COOLING_H_ */
