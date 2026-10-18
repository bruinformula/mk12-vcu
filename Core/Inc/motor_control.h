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
#include "torque_map.h"
#include "pedal_checks.h" // pedal calibration, PEDAL_MODE, plausibility thresholds
#include "torque_diag.h" // 0x502 torque diagnostics frame

#define RPM_TO_CARSPEED_CONVFACTOR                                             \
  (59.0f * 32.0f * 3.14159f * 60.0f) / (12.0f * 39370.1f) // Based on Tire Measurements

// Pedal/torque control loop rate. Must match TIM6 in mk12-vcu.ioc:
// 192 MHz APB1 timer clock / (Prescaler 191 + 1) / (Period 999 + 1) = 1 kHz.
#define CONTROL_LOOP_HZ 1000
#define CONTROL_LOOP_DT_S (1.0f / CONTROL_LOOP_HZ)

// Torque map, filter, and regen constants live in torque_map.h

extern volatile uint16_t ADC_VAL[3];
extern float voltage_values[3];
extern float pedal_percents[3];
extern float requestedTorque;

extern PlausibilityChecks plausibility_checks;

typedef struct InverterDiagnostics {
  float inverter_rpm;
  float inverter_voltage;
  float inverter_carspeed;
} InverterDiagnostics;
extern InverterDiagnostics inverter_diagnostics;

void resetTorqueFilter();
void calculateTorqueRequest(const uint16_t adc[3]);
void configureInverterMessage();
void sendTorqueRequest(uint16_t requestedTorque, uint8_t forward,
                       uint8_t inverter_on);
void sendTorqueDiagnostics();
void processInverter_Voltage();
void processInverter_RPM();
void resetPlausibilityChecks();

#endif /* INC_MOTOR_CONTROL_H_ */
