/*
 * motor_control.c
 *
 *  Created on: Jan 31, 2026
 *      Author: ishanchitale
 */

#include "motor_control.h"
#include "vcu_state.h"
#include <math.h> // fabsf()

volatile uint16_t ADC_VAL[3];
float voltage_values[3];
float pedal_percents[3];
float requestedTorque = 0.0f;

PlausibilityChecks plausibility_checks;

InverterDiagnostics inverter_diagnostics;
static FDCAN_TxHeaderTypeDef Inverter_TxHeader;
static uint8_t Inverter_TxData[8];
static FDCAN_TxHeaderTypeDef TorqueDiag_TxHeader;
static uint8_t TorqueDiag_TxData[8];

// ------------------------
// PEDAL PROCCESSING LOGIC
// ------------------------

static TorqueFilter torque_filter;
static bool torque_cut; // last tick's cut decision, for 0x502

void resetTorqueFilter() {
	torqueMap_filterReset(&torque_filter);
}

// One control loop tick of pedal processing. The checks and the torque math
// live in pedal_checks.c and torque_map.c (HAL-free, host-tested); this only
// supplies the time, car speed and loop dt, and updates the shared globals.
void calculateTorqueRequest(const uint16_t adc[3]) {
	uint32_t now_ms = HAL_GetTick();

	pedalChecks_percents(adc, pedal_percents);
	pedalChecks_update(&plausibility_checks, adc, pedal_percents, now_ms);
	torque_cut = pedalChecks_torqueCut(&plausibility_checks, now_ms);

	requestedTorque = torqueMap_request(&torque_filter, torque_cut,
			pedalChecks_appsPercent(pedal_percents), pedal_percents[PEDAL_BSE],
			inverter_diagnostics.inverter_carspeed, CONTROL_LOOP_DT_S);
}

// ----------------------
// CAN TX AND RX
// ----------------------

void configureInverterMessage() {
  Inverter_TxHeader.Identifier = INVERTER_TORQUE_REQUEST_TX_ID;
  Inverter_TxHeader.IdType = FDCAN_STANDARD_ID;
  Inverter_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  Inverter_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  Inverter_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Inverter_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  Inverter_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  Inverter_TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
  Inverter_TxHeader.MessageMarker = 0;

  TorqueDiag_TxHeader = Inverter_TxHeader;
  TorqueDiag_TxHeader.Identifier = TORQUE_DIAG_TX_ID;
}

void sendTorqueRequest(uint16_t requestedTorque_i, uint8_t forward, uint8_t inverter_on) {
  uint8_t msg0 = (uint8_t)(requestedTorque_i & 0xFF);
  uint8_t msg1 = (uint8_t)((requestedTorque_i >> 8) & 0xFF);

  Inverter_TxData[0] = msg0;
  Inverter_TxData[1] = msg1;
  Inverter_TxData[2] = 0;
  Inverter_TxData[3] = 0;
  Inverter_TxData[4] = forward;     // Forward  (0), Backward (1)
  Inverter_TxData[5] = inverter_on; // Inverter Enable (1), Inverter Disable (0)
  Inverter_TxData[6] = 0;           // Default Torque Limits in EEPROM
  Inverter_TxData[7] = 0;           // Default Torque Limits in EEPROM

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Inverter_TxHeader, Inverter_TxData);
}

// 0x502, layout in torque_diag.h. Call right after sendTorqueRequest() in the
// same tick. The counter advances even when the frame is skipped, so a gap in
// the log shows a missed frame.
void sendTorqueDiagnostics() {
  static uint8_t counter;
  TorqueDiag d = {
    .driver_request_nm = torque_filter.target,
    .command_nm = requestedTorque,
    .apps_percent = pedalChecks_appsPercent(pedal_percents),
    .flags = torqueDiag_flags(&plausibility_checks, torque_cut, torque_filter.target),
    .vcu_state = (uint8_t)vcu_state,
    .counter = counter++,
  };
  torqueDiag_pack(&d, TorqueDiag_TxData);

  // Lower priority than 0x0C0 and the lockout frames: leave a FIFO slot free.
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >= 2) {
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TorqueDiag_TxHeader, TorqueDiag_TxData);
  }
}

void processInverter_Voltage() {
  int16_t inverter_dc_volts_raw = (int16_t)((RxData1[1] << 8) | RxData1[0]);
  inverter_diagnostics.inverter_voltage = inverter_dc_volts_raw * 0.1f;
}

void processInverter_RPM() {
  inverter_diagnostics.inverter_rpm = fabsf(
		  (float)((int16_t)(RxData1[2] | (RxData1[3] << 8))));
  inverter_diagnostics.inverter_carspeed = fabsf(
		  (float)(inverter_diagnostics.inverter_rpm)
		  * RPM_TO_CARSPEED_CONVFACTOR);
}

void resetPlausibilityChecks() {
  pedalChecks_reset(&plausibility_checks);
}
