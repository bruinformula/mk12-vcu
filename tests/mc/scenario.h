/*
 * scenario.h
 *
 * Random driving for the Monte Carlo tests: what the driver and the car are
 * really doing, tick by tick. Pedal segments (hold, step, ramp, noisy),
 * brake stabs, speed that drifts and sometimes jumps, and torque cuts, with
 * values picked on and around every threshold in torque_map.h.
 *
 * Phase 1 feeds these straight into torqueMap_request(). Later phases put
 * sensor and CAN models in between, so the oracle can still check against
 * the truth.
 */

#ifndef TESTS_MC_SCENARIO_H_
#define TESTS_MC_SCENARIO_H_

#include <stdbool.h>
#include <stdint.h>
#include "rng.h"

// One scenario tick is one 1 kHz control loop tick (CONTROL_LOOP_HZ in
// motor_control.h, not included: it pulls in HAL).
#define MC_TICK_MS 1

typedef struct ScenarioInputs {
	float apps;     // pedal fraction, can overshoot [0, 1] like real readings
	float bse;      // brake fraction
	float carspeed; // km/h, negative rolling back
	bool torque_cut;
} ScenarioInputs;

typedef struct Scenario {
	Rng rng;
	bool nonfinite;    // also inject NaN/Inf inputs (off in CI, see test_mc.c)
	int segment_ticks; // ticks left in the current pedal segment
	int cut_ticks;     // ticks left in the current torque cut
	float apps, apps_goal, apps_rate, apps_noise; // rate in fraction per tick
	float bse;
	float speed, accel; // km/h, km/h per tick
} Scenario;

void scenario_init(Scenario *s, uint64_t seed, bool nonfinite);
ScenarioInputs scenario_step(Scenario *s);

#endif /* TESTS_MC_SCENARIO_H_ */
