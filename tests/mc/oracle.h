/*
 * oracle.h
 *
 * Rules every torque command must follow, checked on every tick of every
 * Monte Carlo run. They check limits, not exact values (the unit tests pin
 * those), and they never reimplement the map: a rule is something that must
 * hold however the table and constants are tuned.
 */

#ifndef TESTS_MC_ORACLE_H_
#define TESTS_MC_ORACLE_H_

#include <stdint.h>
#include "scenario.h"
#include "sensors.h"
#include "torque_map.h"

enum {
	RULE_FINITE,      // output and filter state are never NaN/Inf
	RULE_BOUNDS,      // REGEN_MAX_TORQUE <= target, output <= MAX_TORQUE
	RULE_EV313,       // output between 0 and the driver's target (EV.3.1.3)
	RULE_TARGET_ECHO, // f->target matches torqueMap_target() for the same inputs
	RULE_CUT,         // a cut gives 0 Nm on the same tick and resets the filter
	RULE_REGEN_GATES, // regen only if enabled, pedal in the regen band, >= 5 km/h (EV.3.3.3), brake off
	RULE_DRIVE_SLEW,  // drive torque builds no faster than SLEW_RATE_LIMIT
	RULE_REGEN_RAMP,  // regen builds no faster than REGEN_RAMP_NM_S
	RULE_MONOTONIC,   // more pedal never gives a lower target
	RULE_LIFT_NO_DRIVE, // pedal below APPS_INFLECTION_PERCENT never asks for drive torque
	RULE_OVERTRAVEL,  // a reading past either end of a pedal acts like that end
	// Phase 2, whole pedal chain, checked against the truth:
	RULE_RANGE_FAULT, // a sensor out of range > 100 ms cuts torque, latched until leaving drive (T.4.2.9, T.4.3.3)
	RULE_RANGE_INSTANT, // ...and on the same tick: firmware choice, stricter than the 100 ms the rules allow
	RULE_APPS_DISAGREE, // APPS more than 10% apart for > 100 ms cuts torque (T.4.2.4, T.4.2.5)
	RULE_EV47,        // brake + APPS > 25% cuts torque until APPS <= 5% (EV.4.7)
	RULE_NO_FALSE_CUT, // nothing wrong, no cut: healthy sensors never trip a check
	// Phase 3, true car speed behind a stale/lossy 0x0A5:
	RULE_REGEN_TRUE_SPEED, // regen only while the car is really doing >= 5 km/h forward (EV.3.3.3)
	RULE_COUNT
};

extern const char *const RULE_NAMES[RULE_COUNT];

typedef struct OracleTick {
	ScenarioInputs in;     // what the driver and car are really doing
	float dt;
	float prev_output;     // command on the previous tick
	float output;          // torqueMap_request() return value
	const TorqueFilter *f; // filter state after the call
	float apps_higher;     // a pedal position >= in.apps, for RULE_MONOTONIC
} OracleTick;

// Map rules this tick breaks, as a bitmask (bit n = rule n).
uint32_t oracle_check(const OracleTick *t);

// Phase 2: what the oracle remembers between ticks (timers, latch, EV.4.7
// hold), in two versions: "definite" (the rule demands a cut) and
// "possible" (the firmware may justifiably have cut).
typedef struct ChainOracle {
	int out_definite_ticks, out_possible_ticks; // consecutive ticks out of range
	int disagree_definite_ticks;
	bool latched_definite, latched_possible;
	bool ev47_definite, ev47_possible;
} ChainOracle;

typedef struct ChainTick {
	const Sensors *sensors; // each reading's span of counts this tick
	bool torque_cut;        // pedalChecks_torqueCut() this tick
	float output;           // torqueMap_request() this tick
	float true_speed_kph;   // the car's real speed, not the 0x0A5 one
} ChainTick;

// Power-on state, as after pedalChecks_reset(): EV.4.7 holds until the
// pedal is released.
void chain_oracle_init(ChainOracle *o);

// Phase 2 rules this tick breaks. *armed gets the rules that demanded
// something this tick (for coverage: a rule never armed proved nothing).
uint32_t chain_check(ChainOracle *o, const ChainTick *t, uint32_t *armed);

#endif /* TESTS_MC_ORACLE_H_ */
