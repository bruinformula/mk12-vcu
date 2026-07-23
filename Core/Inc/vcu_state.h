/*
 * vcu_state.h
 *
 *  Created on: Mar 31, 2026
 *      Author: ishanchitale
 */

#ifndef INC_VCU_STATE_H_
#define INC_VCU_STATE_H_

#include "stdbool.h"
#include "motor_control.h"
#include "prchg.h"
#include "gpio.h"
#include "adc.h"

typedef enum {
	VCU_IDLE = 0,
	VCU_PRECHARGED,
	VCU_DRIVE,
	VCU_CAN_FAULT
} VCU_STATE;
extern volatile VCU_STATE vcu_state;
extern volatile bool rtd_button_pressed;
extern volatile bool prchg_button_pressed;

#define VCU_DIAGNOSTICS_TX_ID 0x500
#define VCU_DIAG_RATE_LIMIT_MS 50

void resetVCU();
void configureVCUDiagnosticsMessage();
void sendVCUDiagnostics();

#endif /* INC_VCU_STATE_H_ */
