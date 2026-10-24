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
#include "power_limit.h" // EV.3.3.1 torque cap

// Pedal/torque control loop rate. Must match TIM6 in mk12-vcu.ioc:
// 192 MHz APB1 timer clock / (Prescaler 191 + 1) / (Period 999 + 1) = 1 kHz.
#define CONTROL_LOOP_HZ 1000
#define CONTROL_LOOP_DT_S (1.0f / CONTROL_LOOP_HZ)

// Torque map, filter, regen and car speed constants live in torque_map.h

extern volatile uint16_t ADC_VAL[3];
extern float voltage_values[3];
extern float pedal_percents[3];
extern float requestedTorque;

extern PlausibilityChecks plausibility_checks;

typedef struct InverterDiagnostics {
  float inverter_rpm;      // signed, from 0x0A5
  float inverter_voltage;    // DC bus, from 0x0A7
  float inverter_dc_current; // DC bus, from 0x0A6, positive = out of the battery
  float dc_power_w;          // voltage x current: what the Energy Meter sees (EV.3.3.1), roughly
  float inverter_carspeed; // km/h, positive forward (torqueMap_carspeedKph)
  volatile uint32_t rpm_rx_ms; // HAL_GetTick() of the last 0x0A5, for the stale check
} InverterDiagnostics;
extern InverterDiagnostics inverter_diagnostics;

void resetTorqueFilter();
void calculateTorqueRequest(const uint16_t adc[3]);
void configureInverterMessage();
// Queues one 0x0C0 frame. Returns false if the TX FIFO was full (not sent).
bool sendTorqueRequest(uint16_t requestedTorque, uint8_t forward,
                       uint8_t inverter_on);
// Inverter lockout frames (0 Nm, enable = 0). Waits up to LOCKOUT_TX_WAIT_MS
// for the first FIFO slot. Returns how many were queued (0 only if CAN is down).
uint8_t sendInverterLockoutFrames(void);
void sendTorqueDiagnostics();
void processInverter_Voltage();
void processInverter_Current();
void processInverter_RPM();
void resetPlausibilityChecks();

#endif /* INC_MOTOR_CONTROL_H_ */
