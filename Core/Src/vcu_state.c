/*
 * vcu_state.c
 *
 *  Created on: Mar 31, 2026
 *      Author: ishanchitale
 */

#include "vcu_state.h"

volatile VCU_STATE vcu_state = VCU_IDLE;
volatile bool rtd_button_pressed;
volatile bool prchg_button_pressed;

void resetVCU() {
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);

	HAL_ADC_Stop_DMA(&hadc3);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // Disable RTD + PRCHG Buttons

	precharge_state = PRECHARGE_IDLE;
	precharge_response_received = false;
	vcu_state = VCU_IDLE;
	rtd_button_pressed = false; // Clamp button state
	prchg_button_pressed = false; // Clamp button state
	resetPlausibilityChecks();
	requestedTorque = 0.0f;
	resetTorqueFilter();

	for (uint8_t lockout_frames_sent = 0;
			lockout_frames_sent < 5 && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0;
			++lockout_frames_sent) {
		sendTorqueRequest(0, 0, 0);
	}

	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_VAL, 3); // Start pedals
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Re-enable RTD + PRCHG Buttons
}

static FDCAN_TxHeaderTypeDef VCU_Diagnostics_TxHeader;
static uint8_t VCU_Diagnostics_TxData[8];

void configureVCUDiagnosticsMessage() {
  VCU_Diagnostics_TxHeader.Identifier = VCU_DIAGNOSTICS_TX_ID;
  VCU_Diagnostics_TxHeader.IdType = FDCAN_STANDARD_ID;
  VCU_Diagnostics_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  VCU_Diagnostics_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  VCU_Diagnostics_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  VCU_Diagnostics_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  VCU_Diagnostics_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  VCU_Diagnostics_TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
  VCU_Diagnostics_TxHeader.MessageMarker = 0;
}

static uint32_t last_vcu_diag_send = 0;
void sendVCUDiagnostics() {
  if (HAL_GetTick() - last_vcu_diag_send < VCU_DIAG_RATE_LIMIT_MS) return; // 20Hz Rate Limit

  int16_t carspeed = (int16_t)inverter_diagnostics.inverter_carspeed;
  int16_t requestedTorque_i = (int16_t)requestedTorque;
  int8_t apps1 = (int8_t)(pedal_percents[0] * 100.0f);
  int8_t apps2 = (int8_t)(pedal_percents[1] * 100.0f);
  int8_t bse = (int8_t)(pedal_percents[2] * 100.0f);

  uint8_t imd_fault = (HAL_GPIO_ReadPin(IMD_FAULT_GPIO_Port, IMD_FAULT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
  uint8_t rtd_state = (vcu_state == VCU_DRIVE) ? 1 : 0;
  uint8_t precharge_relay = (precharge_state == PRECHARGE_SUCCESS) ? 1 : 0;
  uint8_t air_pos = (inverter_diagnostics.inverter_voltage > 100.0f) ? 1 : 0;
  uint8_t air_neg = (inverter_diagnostics.inverter_voltage > 100.0f) ? 1 : 0;
  uint8_t crosscheck = (!plausibility_checks.crosscheck_plausible) ? 1 : 0;
  uint8_t apps_plausible = (plausibility_checks.apps_plausible) ? 1 : 0;
  uint8_t looking_for_rtd = (vcu_state == VCU_PRECHARGED) ? 1 : 0;

  uint8_t flags = (imd_fault) |
                  (rtd_state << 1) |
                  (precharge_relay << 2) |
                  (air_pos << 3) |
                  (air_neg << 4) |
                  (crosscheck << 5) |
                  (apps_plausible << 6) |
                  (looking_for_rtd << 7);

  VCU_Diagnostics_TxData[0] = (uint8_t)(carspeed & 0xFF);
  VCU_Diagnostics_TxData[1] = (uint8_t)((carspeed >> 8) & 0xFF);
  VCU_Diagnostics_TxData[2] = (uint8_t)(requestedTorque_i & 0xFF);
  VCU_Diagnostics_TxData[3] = (uint8_t)((requestedTorque_i >> 8) & 0xFF);
  VCU_Diagnostics_TxData[4] = (uint8_t)apps1;
  VCU_Diagnostics_TxData[5] = (uint8_t)apps2;
  VCU_Diagnostics_TxData[6] = (uint8_t)bse;
  VCU_Diagnostics_TxData[7] = flags;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &VCU_Diagnostics_TxHeader, VCU_Diagnostics_TxData);
      last_vcu_diag_send = HAL_GetTick();
  }
}
