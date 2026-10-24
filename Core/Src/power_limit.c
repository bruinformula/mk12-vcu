/*
 * power_limit.c
 *
 * EV.3.3.1 power limit. See power_limit.h.
 */

#include "power_limit.h"
#include "torque_map.h" // MAX_TORQUE, CARSPEED_STALE_MS

#define RPM_TO_RAD_S (2.0f * 3.14159265f / 60.0f)

float powerLimit_capNm(float motor_rpm) {
	float omega = (motor_rpm < 0.0f ? -motor_rpm : motor_rpm) * RPM_TO_RAD_S;
	float cap = POWER_LIMIT_W * DRIVETRAIN_EFFICIENCY;
	// Same as omega * MAX_TORQUE < cap, without dividing by a speed near 0.
	if (omega * MAX_TORQUE <= cap) {
		return MAX_TORQUE;
	}
	return cap / omega;
}

float powerLimit_capRpm(float motor_rpm, uint32_t now_ms, uint32_t last_rx_ms) {
	// Unsigned subtraction: still the right age when HAL_GetTick() wraps.
	if (now_ms - last_rx_ms > CARSPEED_STALE_MS) {
		return MOTOR_MAX_RPM;
	}
	return motor_rpm;
}
