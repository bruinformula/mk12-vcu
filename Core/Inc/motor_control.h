/*
 * inverter.h
 *
 *  Created on: Jan 31, 2026
 *      Author: ishanchitale
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "fdcan.h"
#include "stdbool.h"
#include "stdint.h"
#include "vcu_state.h"

// PEDAL_MODE IS EITHER TWO_APPS, ONLY_APPS1, OR ONLY APPS2
#define TWO_APPS 0
#define ONLY_APPS1 1
#define ONLY_APPS2 2
#define PEDAL_MODE ONLY_APPS2

#define APPS1_ADC_MAX_VAL 320
#define APPS1_ADC_MIN_VAL 1400

#define APPS2_ADC_MAX_VAL 470
#define APPS2_ADC_MIN_VAL 1965

#define BSE_ADC_MAX_VAL 1500
#define BSE_ADC_MIN_VAL 375

#define APPS_OVERSHOOT_BUFFER 125
#define APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE 0.10f
#define APPS_IMPLAUSIBILITY_TIMEOUT_MS 100
#define APPS_INFLECTION_PERCENT 0.05f

#define BSE_IMPLAUSIBILITY_TIMEOUT_MS 100
#define BSE_ACTIVATED_ADC_THRESHOLD 1000

#define CROSSCHECK_IMPLAUSIBILITY_PERCENT_DIFFERENCE 0.25f
#define CROSSCHECK_RESTORATION_APPS_PERCENT 0.05f

#define RPM_TO_CARSPEED_CONVFACTOR                                             \
  (59.0f * 32.0f * 3.14159f * 60.0f) / (12.0f * 39370.1f) // Based on Tire Measurements
#define RC_TIME_CONSTANT 0.05f // Tau, RC Time Constant for digital LPF
#define SLEW_RATE_LIMIT 2000.0f // Adjust according to event

#define MAX_TORQUE 80 // Adjust according to event
#define MIN_TORQUE 0
#define REGEN_BASELINE_TORQUE 0
#define REGEN_MAX_TORQUE -30

// Plausibility/calibration constants exposed to the GUI protocol
// (Core/Src/gui_protocol.c) as live params. Defaults mirror the #defines
// above; edit via GUI_CMD SET_PARAM, VCU_IDLE only. Deliberately does NOT
// include any torque-shaping constant (MAX/MIN/REGEN_MAX_TORQUE,
// SLEW_RATE_LIMIT, RC_TIME_CONSTANT, APPS_INFLECTION_PERCENT) -- those stay
// plain #defines, not live-tunable, until torque request tuning is
// revisited later.
typedef struct VCU_Params {
  float apps1_adc_max_val;
  float apps1_adc_min_val;
  float apps2_adc_max_val;
  float apps2_adc_min_val;
  float bse_adc_max_val;
  float bse_adc_min_val;
  float apps_overshoot_buffer;
  float apps_implausibility_percent;
  float apps_implausibility_timeout_ms;
  float bse_implausibility_timeout_ms; // not yet wired to any check
  float bse_activated_adc_threshold;
  float crosscheck_implausibility_percent;
  float crosscheck_restoration_apps_percent;
} VCU_Params;
extern VCU_Params vcu_params;

extern volatile uint16_t ADC_VAL[3];
extern float voltage_values[3];
extern float pedal_percents[3];
extern float requestedTorque;

typedef struct PlausibilityChecks {
  bool apps1_invalid;
  bool apps2_invalid;
  bool apps_plausible;
  bool crosscheck_plausible;
} PlausibilityChecks;
extern PlausibilityChecks plausibility_checks;

typedef struct InverterDiagnostics {
  float inverter_rpm;
  float inverter_voltage;
  float inverter_carspeed;
} InverterDiagnostics;
extern InverterDiagnostics inverter_diagnostics;

void resetTorqueFilter();
void calculateTorqueRequest();
void validateAPPS();
void checkAPPS_Plausibility();
void checkAPPS_BSE_Crosscheck();
void configureInverterMessage();
void sendTorqueRequest(uint16_t requestedTorque, uint8_t forward,
                       uint8_t inverter_on);
void processInverter_Voltage();
void processInverter_RPM();
void resetPlausibilityChecks();

#endif /* INC_MOTOR_CONTROL_H_ */
