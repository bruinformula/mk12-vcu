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

// Pedal map: fraction of MAX_TORQUE at TORQUE_MAP_POINTS evenly spaced points
// of pedal travel, from APPS_INFLECTION_PERCENT (first entry) to full pedal
// (last). Linear in between. Defined in torque_map.c; must pass
// torqueMap_tableIsValid() or the VCU halts at boot.
// To tune: edit the table, run `make -C tests` and tools/torque_curve_sim.c.
#define TORQUE_MAP_POINTS 11
extern const float torqueMap_table[TORQUE_MAP_POINTS];

// True if a map starts at 0, ends at 1, stays within [0, 1], and never
// decreases (more pedal must never mean less torque).
bool torqueMap_tableIsValid(const float *table, int points);

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
// same tick (no filtering) and resets the filter. Otherwise the map's target
// goes through the LPF + slew limiter, and the result is kept between 0 Nm and
// the target (EV.3.1.3), so the filter only ever slows torque coming in.
float torqueMap_request(TorqueFilter *f, bool torque_cut, float apps_percent,
		float bse_percent, float carspeed, float dt);

#endif /* INC_TORQUE_MAP_H_ */
