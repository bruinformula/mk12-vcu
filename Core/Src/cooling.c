/*
 * cooling.c
 *
 *  Created on: Apr 18, 2026
 *      Author: ishanchitale
 */

#include "cooling.h"

static COOLING_CMD_DF cooling_cmd_df;
static FDCAN_TxHeaderTypeDef CoolingCmd_TxHeader;

static uint8_t last_tractive_fan_pwm = IDLE_TRACTIVE_FAN_PWM;
static uint8_t last_tractive_pump_pwm = IDLE_TRACTIVE_PUMP_PWM;
static uint8_t last_accy_fan_pwm = IDLE_ACCY_FAN_PWM;

static float bms_avg_temp;
static float phase_a_temp;
static float phase_b_temp;
static float phase_c_temp;
static float inv_avg_temp;

void configureCoolingCmdMsg() {
	CoolingCmd_TxHeader.Identifier = VCU_COOLING_CMD_TX_ID;
	CoolingCmd_TxHeader.IdType = FDCAN_STANDARD_ID;
	CoolingCmd_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	CoolingCmd_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	CoolingCmd_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CoolingCmd_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	CoolingCmd_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	CoolingCmd_TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	CoolingCmd_TxHeader.MessageMarker = 0;
}

void calculateTractiveFanPWM(float inverter_temp) {
	if (vcu_state != VCU_DRIVE) {
		last_tractive_fan_pwm = IDLE_TRACTIVE_FAN_PWM;
		return;
	}

	if (inverter_temp > INVERTER_TEMP_THRESHOLD_H) {
		last_tractive_fan_pwm = 100;
	} else if (inverter_temp < INVERTER_TEMP_THRESHOLD_L) {
		last_tractive_fan_pwm = 0;
	} else {
		last_tractive_fan_pwm = (int) (( (inverter_temp - INVERTER_TEMP_THRESHOLD_L) /
				(INVERTER_TEMP_THRESHOLD_H - INVERTER_TEMP_THRESHOLD_L))*100);
	}
}

void calculateTractivePumpPWM(float inverter_temp) {
	if (vcu_state != VCU_DRIVE) {
		last_tractive_pump_pwm = IDLE_TRACTIVE_PUMP_PWM;
		return;
	}

	if (inverter_temp > INVERTER_TEMP_THRESHOLD_H) {
		last_tractive_pump_pwm = 100;
	} else if (inverter_temp < INVERTER_TEMP_THRESHOLD_L){
		last_tractive_pump_pwm = 0;
	} else {
		last_tractive_pump_pwm = (int) (( (inverter_temp - INVERTER_TEMP_THRESHOLD_L) /
				(INVERTER_TEMP_THRESHOLD_H - INVERTER_TEMP_THRESHOLD_L))*100);
	}
}

void calculateAccyFanPWM(float bms_temp) {
    if (vcu_state != VCU_DRIVE) {
        last_accy_fan_pwm = IDLE_ACCY_FAN_PWM;
        return;
    }

    if (bms_temp >= BMS_TEMP_THRESHOLD_H) {
        last_accy_fan_pwm = 75;
    } else if (bms_temp <= BMS_TEMP_THRESHOLD_L) {
        last_accy_fan_pwm = 0;
    } else {
        last_accy_fan_pwm = (int)( (  (bms_temp - BMS_TEMP_THRESHOLD_L) /
                       (BMS_TEMP_THRESHOLD_H - BMS_TEMP_THRESHOLD_L))*75);
    }
}

void processBMS_Temp() {
	uint16_t avg_temp_raw = (uint16_t)(RxData1[3] << 8) | RxData1[2];
	bms_avg_temp = (float)avg_temp_raw/100.0f;
	calculateAccyFanPWM(bms_avg_temp);
}

void processInverter_Temp() {
	phase_a_temp = (float)((RxData1[1] << 8) | RxData1[0]) / 10.0f;
	phase_b_temp = (float)((RxData1[3] << 8) | RxData1[2]) / 10.0f;
	phase_c_temp = (float)((RxData1[5] << 8) | RxData1[4]) / 10.0f;
	inv_avg_temp = (phase_a_temp + phase_b_temp + phase_c_temp)/3.0f;
	calculateTractiveFanPWM(inv_avg_temp);
	calculateTractivePumpPWM(inv_avg_temp);
}

static uint32_t last_cooling_cmd_send = 0;
void sendCoolingCmd() {
	if (HAL_GetTick() - last_cooling_cmd_send < COOLING_CMD_RATE_LIMIT_MS) return; // 2 Hz Rate Limit

	cooling_cmd_df.data.cooling_en = 1;
    cooling_cmd_df.data.tractive_fan_pwm = last_tractive_fan_pwm;
    cooling_cmd_df.data.tractive_pump_pwm = last_tractive_pump_pwm;
    cooling_cmd_df.data.accy_fan_pwm = last_accy_fan_pwm;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
    	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CoolingCmd_TxHeader, cooling_cmd_df.array);
    	last_cooling_cmd_send = HAL_GetTick();
    }
}
