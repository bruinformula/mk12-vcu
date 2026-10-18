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

static float clamp01(float x) {
	if (x < 0.0f) return 0.0f;
	if (x > 1.0f) return 1.0f;
	return x;
}

float torqueMap_target(float apps_percent, float bse_percent, float carspeed) {
	// Readings inside the overshoot buffer can land just outside [0, 1].
	// Unclamped, -8% APPS travel would command ~-80 Nm of regen.
	apps_percent = clamp01(apps_percent);
	bse_percent = clamp01(bse_percent);

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

void torqueMap_filterReset(TorqueFilter *f) {
	f->filtered = 0.0f;
	f->output = 0.0f;
}

float torqueMap_request(TorqueFilter *f, bool torque_cut, float apps_percent,
		float bse_percent, float carspeed, float dt) {
	if (torque_cut) {
		// T.4.2.5 / T.4.3.3 / EV.4.7: power must stop immediately, so a cut
		// skips the filter. Zeroing its state makes torque ramp back up from
		// 0 Nm once the fault clears.
		torqueMap_filterReset(f);
		return 0.0f;
	}

	float target = torqueMap_target(apps_percent, bse_percent, carspeed);
	f->output = torqueMap_filterStep(&f->filtered, target, f->output, dt);
	return f->output;
}
