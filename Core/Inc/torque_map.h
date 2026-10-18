/*
 * torque_map.h
 *
 * Pedal-to-torque pipeline as pure functions: no HAL, CAN, or global state.
 * Compiles on the host (tools/torque_curve_sim.c) as well as on the VCU, so
 * the sim and future unit tests run the exact code that drives the car.
 */

#ifndef INC_TORQUE_MAP_H_
#define INC_TORQUE_MAP_H_

#include <stdbool.h>

#define APPS_INFLECTION_PERCENT 0.05f

#define RC_TIME_CONSTANT 0.05f // Tau, RC Time Constant for digital LPF
#define SLEW_RATE_LIMIT 2000.0f // Adjust according to event

#define MAX_TORQUE 80 // Adjust according to event
#define MIN_TORQUE 0
#define REGEN_BASELINE_TORQUE 0
#define REGEN_MAX_TORQUE -30

#define REGEN_MIN_CARSPEED 5.0f // No regen below this car speed
#define REGEN_BRAKE_CANCEL_PERCENT 0.05f // No regen once brake pedal passes this

// Shapes the positive-torque pedal curve. 1.0 = old linear map (identical output).
// >1.0 = progressive: softer near tip-in, steeper near full pedal.
// Tune this and re-run tools/torque_curve_sim.c before touching this value on real hardware.
#define TORQUE_CURVE_EXPONENT 1.8f

// Target torque (Nm) for a plausible pedal input. Positive above
// APPS_INFLECTION_PERCENT, regen below it. Pedal inputs are clamped to [0, 1].
// Fault handling is the caller's job.
float torqueMap_target(float apps_percent, float bse_percent, float carspeed);

// One step of the LPF + slew rate limiter. Updates *filtered and returns the
// new output torque given the previous output and dt in seconds.
float torqueMap_filterStep(float *filtered, float target, float prev_output, float dt);

typedef struct TorqueFilter {
	float filtered; // LPF state
	float output;   // last commanded torque (after the slew limiter)
} TorqueFilter;

// Back to 0 Nm. Torque ramps up from here through the filter.
void torqueMap_filterReset(TorqueFilter *f);

// Torque command for one control loop tick. A torque cut returns 0 Nm on the
// same tick (no filtering) and resets the filter; otherwise the map's target
// goes through the LPF + slew limiter.
float torqueMap_request(TorqueFilter *f, bool torque_cut, float apps_percent,
		float bse_percent, float carspeed, float dt);

#endif /* INC_TORQUE_MAP_H_ */
