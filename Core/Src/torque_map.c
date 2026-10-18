/*
 * torque_map.c
 *
 * Pedal-to-torque pipeline. See torque_map.h.
 */

#include "torque_map.h"
#include <math.h> // powf() for the progressive torque curve

// POSITIVE TORQUE REGION
static float driverTorque(float apps_percent) {
	float pedal_normalized = (apps_percent - APPS_INFLECTION_PERCENT) /
			(1.0f - APPS_INFLECTION_PERCENT);

	// OLD LINEAR MAP (kept for reference / instant revert - just swap the line below back in):
	// float torque = (MAX_TORQUE - MIN_TORQUE) * pedal_normalized;

	// PROGRESSIVE MAP: softer near tip-in, steeper near full pedal.
	// TORQUE_CURVE_EXPONENT == 1.0f reproduces the old linear line above exactly.
	float torque = (MAX_TORQUE - MIN_TORQUE) * powf(pedal_normalized, TORQUE_CURVE_EXPONENT);

	// GENERAL TORQUE CLAMPING
	if (torque >= MAX_TORQUE) {
		torque = MAX_TORQUE;
	}

	if (torque <= MIN_TORQUE) {
		torque = MIN_TORQUE;
	}
	return torque;
}

// REGEN REGION
static float regenTorque(float apps_percent, float bse_percent, float carspeed) {
	if (carspeed < REGEN_MIN_CARSPEED || bse_percent > REGEN_BRAKE_CANCEL_PERCENT) {
		// Cancel regen if car speed is low, or brake pedal is pressed (>5%).
		// Commanding regen while brake pedal is pressed causes large phase currents and trips inverter!
		return 0.0f;
	}

	// Command regen!
	return (REGEN_MAX_TORQUE - REGEN_BASELINE_TORQUE) *
			((APPS_INFLECTION_PERCENT - apps_percent) / APPS_INFLECTION_PERCENT);
}

float torqueMap_target(float apps_percent, float bse_percent, float carspeed) {
	if (apps_percent >= APPS_INFLECTION_PERCENT) {
		return driverTorque(apps_percent);
	}
	return regenTorque(apps_percent, bse_percent, carspeed);
}

// LOW PASS FILTER (LPF) + SLEW RATE LIMITER
float torqueMap_filterStep(float *filtered, float target, float prev_output, float dt) {
	float alpha = dt / (RC_TIME_CONSTANT + dt);

	*filtered = (target * alpha) + (*filtered * (1.0f - alpha));
	float max_step = SLEW_RATE_LIMIT*dt;

	if (*filtered > prev_output + max_step) {
		return prev_output + max_step;
	} else if (*filtered < prev_output - max_step) {
		return prev_output - max_step;
	}
	return *filtered;
}
