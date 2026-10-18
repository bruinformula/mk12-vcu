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
static uint32_t millis_since_apps_implausible;

InverterDiagnostics inverter_diagnostics;
static FDCAN_TxHeaderTypeDef Inverter_TxHeader;
static uint8_t Inverter_TxData[8];

// ------------------------
// PEDAL PROCCESSING LOGIC
// ------------------------

static float filtered_target_torque = 0.0f;
static bool torque_filter_initialized = false;

void resetTorqueFilter() {
	filtered_target_torque = 0.0f;
	torque_filter_initialized = false;
}

void calculateTorqueRequest() {
	float targetTorque = 0.0f;
	float apps_percent_average;

#if PEDAL_MODE == TWO_APPS
	apps_percent_average = (pedal_percents[0] + pedal_percents[1]) / 2;
#elif PEDAL_MODE == ONLY_APPS1
	apps_percent_average = pedal_percents[0];
#elif PEDAL_MODE == ONLY_APPS2
		apps_percent_average = pedal_percents[1];
#endif

  // ANALYZING PLAUSIBILITY CHECKS!
  bool safety_torque_cut = false;
  if (plausibility_checks.apps1_invalid || plausibility_checks.apps2_invalid) {
	  safety_torque_cut = true;
  }
  if (!plausibility_checks.crosscheck_plausible) {
  		safety_torque_cut = true;
  }
  if (!plausibility_checks.apps_plausible &&
  		(HAL_GetTick() - millis_since_apps_implausible) > APPS_IMPLAUSIBILITY_TIMEOUT_MS) {
  		safety_torque_cut = true;
  }

  if (safety_torque_cut) {
	  // PLAUSIBILITY CHECK FAILED!
	  targetTorque = 0.0f;
  } else {
	  targetTorque = torqueMap_target(apps_percent_average, pedal_percents[2],
			  inverter_diagnostics.inverter_carspeed);
  }

  // LOW PASS FILTER (LPF) + SLEW RATE LIMITER

  // Initialize filter!
  if (!torque_filter_initialized) {
	  torque_filter_initialized = true;
	  filtered_target_torque = targetTorque;
	  requestedTorque = filtered_target_torque;
	  return;
  }

  // Called once per control loop tick, so dt is fixed.
  requestedTorque = torqueMap_filterStep(&filtered_target_torque, targetTorque,
		  requestedTorque, CONTROL_LOOP_DT_S);
}

void validateAPPS() {
// Note that as APPS1 and APPS2 are pressed, raw ADC value DECREASES.
// Handles 3 cases and cuts torque to 0 in case.
// 1. APPS1 or APPS2 are plugged in and not pressed; ADC Value will be larger than ADC_MIN_VAL.
// 2. APPS1 or APPS2 are plugged in and pressed beyond intended range of motion; ADC Value will be smaller than ADC_MAX_VAL.
// 3. APPS1 or APPS2 are unplugged and raw ADC value is ~ 70; this is smaller than ADC_MAX_VAL.

    plausibility_checks.apps1_invalid = (ADC_VAL[0] > (APPS1_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER)) ||
    		(ADC_VAL[0] < (APPS1_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER));
    plausibility_checks.apps2_invalid = (ADC_VAL[1] > (APPS2_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER)) ||
        (ADC_VAL[1] < (APPS2_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER));

    // Torque cut handled in calculateTorqueRequest().
}

void checkAPPS_Plausibility() {
  float pedal_travel_difference_percent =
      fabsf(pedal_percents[0] - pedal_percents[1]);
  bool apps_invalid = (pedal_travel_difference_percent >
                       APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE);

  if (plausibility_checks.apps_plausible == true && apps_invalid == true) {
    millis_since_apps_implausible = HAL_GetTick();
    plausibility_checks.apps_plausible = false;
  }

  if (plausibility_checks.apps_plausible == false) {
    if ((HAL_GetTick() - millis_since_apps_implausible) >
        APPS_IMPLAUSIBILITY_TIMEOUT_MS) {
        // Torque cut handled in calculateTorqueRequest().
    }

    if (apps_invalid == false) {
      plausibility_checks.apps_plausible = true;
    }
  }
}

void checkAPPS_BSE_Crosscheck() {
	float apps_percent_average;

#if PEDAL_MODE == TWO_APPS
	apps_percent_average = (pedal_percents[0] + pedal_percents[1]) / 2;
#elif PEDAL_MODE == ONLY_APPS1
	apps_percent_average = pedal_percents[0];
#elif PEDAL_MODE == ONLY_APPS2
	apps_percent_average = pedal_percents[1];
#endif

  if (plausibility_checks.crosscheck_plausible == true &&
      (apps_percent_average > CROSSCHECK_IMPLAUSIBILITY_PERCENT_DIFFERENCE) &&
      (ADC_VAL[2] > BSE_ACTIVATED_ADC_THRESHOLD)) {
    plausibility_checks.crosscheck_plausible = false;
  }

  if (plausibility_checks.crosscheck_plausible == false) {
    if (apps_percent_average <= CROSSCHECK_RESTORATION_APPS_PERCENT) {
      plausibility_checks.crosscheck_plausible = true;
    } else {
        // Torque cut handled in calculateTorqueRequest().
    }
  }
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
  plausibility_checks.apps1_invalid = false;
  plausibility_checks.apps2_invalid = false;
  plausibility_checks.apps_plausible = false;
  plausibility_checks.crosscheck_plausible = false;
}
