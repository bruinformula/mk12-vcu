/*
 * prchg.c
 *
 *  Created on: Jan 26, 2026
 *      Author: ishanchitale
 */

#include "prchg.h"

static FDCAN_TxHeaderTypeDef PRCHG_TxHeader;
static uint8_t PRCHG_TxData[8];
volatile bool precharge_response_received = false;
volatile PrechargeState precharge_state = PRECHARGE_IDLE;

void configurePrechargeMessage() {
	PRCHG_TxHeader.Identifier = BMS_PRCHG_TX_ID;
	PRCHG_TxHeader.IdType = FDCAN_STANDARD_ID;
	PRCHG_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	PRCHG_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	PRCHG_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	PRCHG_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	PRCHG_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	PRCHG_TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	PRCHG_TxHeader.MessageMarker = 0;

	PRCHG_TxData[0] = 1;
}

void sendPrechargeRequest() {
	configurePrechargeMessage();
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &PRCHG_TxHeader, PRCHG_TxData);
	precharge_response_received = false;
    precharge_state = PRECHARGE_WAITING;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start_IT(&htim3); // START PRECHARGE TIMER (6 SECONDS)
}

void processPrechargeResponse() {
	if (precharge_state != PRECHARGE_WAITING) return;
	precharge_response_received = true;

	if (RxData1[0] == 1) {
		precharge_state = PRECHARGE_SUCCESS;
	} else {
		precharge_state = PRECHARGE_FAILURE;
	}
}

void checkPrechargeStatus() {
#if PRECHARGE_MODE == PRECHARGE_MODE_BYPASS

	precharge_response_received = true;
	precharge_state = PRECHARGE_SUCCESS;
	vcu_state = VCU_PRECHARGED;

#elif PRECHARGE_MODE == PRECHARGE_MODE_NORMAL

	if (precharge_response_received == false) {
		precharge_state = PRECHARGE_TIMEOUT;
		vcu_state = VCU_CAN_FAULT;
		Error_Handler();
	} else {
		if (precharge_state == PRECHARGE_SUCCESS) {
			vcu_state = VCU_PRECHARGED;
		}
	}

#endif
}
