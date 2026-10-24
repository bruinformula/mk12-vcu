/*
 * power_limit.h
 *
 * EV.3.3.1 power limit as pure functions: no HAL, CAN, or global state.
 * Host tested in tests/test_power_limit.c.
 *
 * The rule: the Energy Meter (at the accumulator outlet) must not see more
 * than 80 kW for 100 ms straight, or as a 500 ms moving average (EV.3.4.1).
 * Each run with a violation loses its best time (EV.3.5.1), and every one in
 * Endurance costs 60 s (EV.3.5.2).
 *
 * The cap works from motor speed alone: drive torque is limited so that
 * torque x speed / DRIVETRAIN_EFFICIENCY stays under POWER_LIMIT_W. No
 * current sensor in the loop, so nothing to tune or go unstable; the margin
 * covers what the model gets wrong.
 *
 * At MAX_TORQUE 80 Nm the cap only binds above ~7600 rpm, past the
 * inverter's 6000 rpm Max_Speed, so today it never changes the command. It
 * starts to matter once MAX_TORQUE goes past ~100 Nm (the inverter EEPROM
 * Motor_Torque_Limit is 231 Nm).
 */

#ifndef INC_POWER_LIMIT_H_
#define INC_POWER_LIMIT_H_

#include <stdint.h>

// DC power the cap aims for: 5 kW under EV.3.3.1 for meter and model error.
#define POWER_LIMIT_W 75000.0f
#define POWER_RULE_W 80000.0f // EV.3.3.1, for the tests

// Motor + inverter efficiency the cap assumes (DC power = mechanical power /
// this). Low on purpose: assuming worse than real gives a lower cap. VERIFY
// on the dyno or car (0x0A6 DC current x 0x0A7 DC voltage against torque x
// speed at high power), then raise it to win back the margin.
#define DRIVETRAIN_EFFICIENCY 0.85f

// Speed the cap assumes when 0x0A5 is stale: the inverter's Max_Speed
// (EEPROM 2026-09-19). The fastest the motor can turn gives the lowest cap.
#define MOTOR_MAX_RPM 6000.0f

// Highest drive torque (Nm) at this motor speed, at most MAX_TORQUE. Either
// sign of rpm.
float powerLimit_capNm(float motor_rpm);

// The motor speed to cap with: the last 0x0A5 rpm, or MOTOR_MAX_RPM once it's
// older than CARSPEED_STALE_MS. Not raised for the frame's age (unlike the
// regen speed check): the rule allows 100 ms over, and frames come every 20 ms.
// Tick wraparound safe.
float powerLimit_capRpm(float motor_rpm, uint32_t now_ms, uint32_t last_rx_ms);

#endif /* INC_POWER_LIMIT_H_ */
