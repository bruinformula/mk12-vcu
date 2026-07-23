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
