/*
 * torque_map.c
 *
 * Pedal-to-torque pipeline. See torque_map.h.
 */

#include "torque_map.h"

// Straight line, 0 to MAX_TORQUE over the pedal (MK11 feel).
const float torqueMap_table[TORQUE_MAP_POINTS] = {
	0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f,
};

bool torqueMap_tableIsValid(const float *table, int points) {
	if (points < 2 || table[0] != 0.0f || table[points - 1] != 1.0f) {
		return false;
	}
	for (int i = 1; i < points; i++) {
		if (table[i] < table[i - 1] || table[i] > 1.0f) {
			return false;
		}
	}
	return true;
}

// POSITIVE TORQUE REGION
static float driverTorque(float apps_percent) {
	float pedal_normalized = (apps_percent - APPS_INFLECTION_PERCENT) /
			(1.0f - APPS_INFLECTION_PERCENT);

	// Linear interpolation between the two nearest table points.
	float position = pedal_normalized * (TORQUE_MAP_POINTS - 1);
	if (position < 0.0f) position = 0.0f;
	int i = (int)position;
	float fraction = torqueMap_table[TORQUE_MAP_POINTS - 1];
	if (i < TORQUE_MAP_POINTS - 1) {
		fraction = torqueMap_table[i] +
				(torqueMap_table[i + 1] - torqueMap_table[i]) * (position - i);
	}
	float torque = MIN_TORQUE + (MAX_TORQUE - MIN_TORQUE) * fraction;

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
	f->target = 0.0f;
}

float torqueMap_request(TorqueFilter *f, bool torque_cut, float apps_percent,
		float bse_percent, float carspeed, float dt) {
	float target = torqueMap_target(apps_percent, bse_percent, carspeed);

	if (torque_cut) {
		// T.4.2.5 / T.4.3.3 / EV.4.7: power must stop immediately, so a cut
		// skips the filter. Zeroing its state makes torque ramp back up from
		// 0 Nm once the fault clears.
		torqueMap_filterReset(f);
		f->target = target;
		return 0.0f;
	}

	f->target = target;
	float output = torqueMap_filterStep(&f->filtered, target, f->output, dt);

	// EV.3.1.3: the VCU may only reduce the driver's request, never add to it.
	// Keep the output between 0 Nm and the target: torque (and regen) build up
	// through the filter, but lifting off follows the pedal down immediately,
	// and switching between drive and regen passes through 0 Nm.
	float lo = (target < 0.0f) ? target : 0.0f;
	float hi = (target > 0.0f) ? target : 0.0f;
	if (output > hi || output < lo) {
		output = (output > hi) ? hi : lo;
		// Restart the LPF from here so pressing again ramps up smoothly.
		f->filtered = output;
	}

	f->output = output;
	return f->output;
}
