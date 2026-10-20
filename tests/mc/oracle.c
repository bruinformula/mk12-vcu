/*
 * oracle.c
 *
 * Per-tick rules for the Monte Carlo tests. See oracle.h.
 */

#include <math.h>
#include "oracle.h"
#include "pedal_checks.h"

#define TOL 1e-3f // Nm: float rounding, not a tuning margin
#define BIT(rule) (1u << (rule))

const char *const RULE_NAMES[RULE_COUNT] = {
	[RULE_FINITE] = "finite",
	[RULE_BOUNDS] = "bounds",
	[RULE_EV313] = "ev3.1.3",
	[RULE_TARGET_ECHO] = "target_echo",
	[RULE_CUT] = "cut",
	[RULE_REGEN_GATES] = "regen_gates",
	[RULE_DRIVE_SLEW] = "drive_slew",
	[RULE_REGEN_RAMP] = "regen_ramp",
	[RULE_MONOTONIC] = "monotonic",
	[RULE_LIFT_NO_DRIVE] = "lift_no_drive",
	[RULE_OVERTRAVEL] = "overtravel",
	[RULE_RANGE_FAULT] = "range_fault",
	[RULE_RANGE_INSTANT] = "range_instant",
	[RULE_APPS_DISAGREE] = "apps_disagree",
	[RULE_EV47] = "ev4.7",
	[RULE_NO_FALSE_CUT] = "no_false_cut",
	[RULE_REGEN_TRUE_SPEED] = "regen_true_speed",
};

static bool same(float a, float b) {
	return a == b || (isnan(a) && isnan(b));
}

uint32_t oracle_check(const OracleTick *t) {
	const ScenarioInputs *in = &t->in;
	float out = t->output;
	float prev = t->prev_output;
	float target = torqueMap_target(in->apps, in->bse, in->carspeed);
	uint32_t broken = 0;

	if (!isfinite(out) || !isfinite(t->f->filtered) || !isfinite(t->f->output)) {
		broken |= BIT(RULE_FINITE);
	}

	if (out > MAX_TORQUE + TOL || out < REGEN_MAX_TORQUE - TOL ||
			target > MAX_TORQUE + TOL || target < REGEN_MAX_TORQUE - TOL) {
		broken |= BIT(RULE_BOUNDS);
	}

	if (out > fmaxf(target, 0.0f) + TOL || out < fminf(target, 0.0f) - TOL) {
		broken |= BIT(RULE_EV313);
	}

	if (!same(t->f->target, target)) {
		broken |= BIT(RULE_TARGET_ECHO);
	}

	if (in->torque_cut && (out != 0.0f || t->f->filtered != 0.0f || t->f->output != 0.0f)) {
		broken |= BIT(RULE_CUT);
	}

	// The legal limit is REGEN_MIN_CARSPEED; the fade starting higher is tuning.
	if (out < 0.0f && (!REGEN_ENABLED || in->torque_cut ||
			!(in->apps < REGEN_END_PERCENT) ||
			!(in->carspeed >= REGEN_MIN_CARSPEED) ||
			!(in->bse <= REGEN_BRAKE_CANCEL_PERCENT))) {
		broken |= BIT(RULE_REGEN_GATES);
	}

	// Releasing either way isn't limited: only building away from 0 Nm is.
	if (out > 0.0f && out > fmaxf(prev, 0.0f) + SLEW_RATE_LIMIT * t->dt + TOL) {
		broken |= BIT(RULE_DRIVE_SLEW);
	}
	if (out < 0.0f && out < fminf(prev, 0.0f) - REGEN_RAMP_NM_S * t->dt - TOL) {
		broken |= BIT(RULE_REGEN_RAMP);
	}

	float higher = torqueMap_target(t->apps_higher, in->bse, in->carspeed);
	if (higher < target - TOL) {
		broken |= BIT(RULE_MONOTONIC);
	}

	if (in->apps < APPS_INFLECTION_PERCENT && target > 0.0f) {
		broken |= BIT(RULE_LIFT_NO_DRIVE);
	}

	// Readings in the overshoot buffer land outside [0, 1]; they must not ask
	// for more (or deeper regen) than the pedal's own end stop.
	float apps_end = fminf(fmaxf(in->apps, 0.0f), 1.0f);
	float bse_end = fminf(fmaxf(in->bse, 0.0f), 1.0f);
	if ((apps_end != in->apps || bse_end != in->bse) &&
			!same(target, torqueMap_target(apps_end, bse_end, in->carspeed))) {
		broken |= BIT(RULE_OVERTRAVEL);
	}

	return broken;
}

// --------------------------------------------------------------------------
// Phase 2: the whole pedal chain
//
// Noise is bounded, so each reading lies in a known span of counts. A rule
// demands a cut only when the whole span breaks it ("definitely"), and
// no_false_cut only checks ticks where nothing in any span could justify a
// cut. Ticks in between are left alone.
// --------------------------------------------------------------------------

// Rule limits, from the rulebook. The ADC thresholds below are the team's
// calibration (pedal_checks.h), which is configuration, not logic.
#define TIMEOUT_TICKS (100 / MC_TICK_MS) // T.4.2.5, T.4.2.9, T.4.3.3: 100 ms
#define APPS_MAX_DIFF 0.10f              // T.4.2.4: 10 points of pedal travel
#define EV47_TRIGGER 0.25f               // EV.4.7: brake + APPS over 25% cuts...
#define REGEN_MIN_KPH 5.0f               // EV.3.3.3: no regen below 5 km/h
#define EV47_RELEASE 0.05f               // ...until APPS is back to 5%
// Which tick a 100 ms timer fires on (> or >=, timestamp taken before or
// after the check) is the firmware's call. Allow it 2 ms either way.
#define SLACK_TICKS 2

typedef struct Span {
	float lo, hi;
} Span;

static const float RELEASED[3] = { APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, BSE_ADC_MIN_VAL };
static const float PRESSED[3] = { APPS1_ADC_MAX_VAL, APPS2_ADC_MAX_VAL, BSE_ADC_MAX_VAL };
// Valid reading, inclusive; anything outside is open/short or overtravel.
static const float VALID_LO[3] = {
	APPS1_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER, APPS2_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER,
	BSE_ADC_OPEN_CIRCUIT_VAL,
};
static const float VALID_HI[3] = {
	APPS1_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER, APPS2_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER,
	BSE_ADC_SHORT_CIRCUIT_VAL,
};

#define ADC_FULL_SCALE 4095.0f

// Clamped like the ADC: two railed sensors read the same, whatever their
// ideal counts, so the firmware rightly sees them agree.
static Span counts_span(const SensorChannel *c) {
	return (Span){
		fminf(fmaxf(c->ideal - c->bound, 0.0f), ADC_FULL_SCALE),
		fminf(fmaxf(c->ideal + c->bound, 0.0f), ADC_FULL_SCALE),
	};
}

// Counts -> pedal travel (0 released, 1 full).
static Span travel_span(Span counts, int sensor) {
	float a = (counts.lo - RELEASED[sensor]) / (PRESSED[sensor] - RELEASED[sensor]);
	float b = (counts.hi - RELEASED[sensor]) / (PRESSED[sensor] - RELEASED[sensor]);
	return (Span){ fminf(a, b), fmaxf(a, b) };
}

// The APPS travel PEDAL_MODE uses for torque and EV.4.7.
static Span apps_used(Span t1, Span t2) {
#if PEDAL_MODE == TWO_APPS
	return (Span){ (t1.lo + t2.lo) / 2, (t1.hi + t2.hi) / 2 };
#elif PEDAL_MODE == ONLY_APPS1
	(void)t2;
	return t1;
#elif PEDAL_MODE == ONLY_APPS2
	(void)t1;
	return t2;
#elif PEDAL_MODE == MIN_APPS
	return (Span){ fminf(t1.lo, t2.lo), fminf(t1.hi, t2.hi) };
#endif
}

void chain_oracle_init(ChainOracle *o) {
	*o = (ChainOracle){ .ev47_definite = true, .ev47_possible = true };
}

uint32_t chain_check(ChainOracle *o, const ChainTick *t, uint32_t *armed) {
	const SensorChannel *ch = t->sensors->ch;
	Span c[3];
	bool out_definite = false, out_possible = false;
	for (int i = 0; i < 3; i++) {
		c[i] = counts_span(&ch[i]);
		out_definite |= c[i].lo > VALID_HI[i] || c[i].hi < VALID_LO[i];
		out_possible |= c[i].hi > VALID_HI[i] || c[i].lo < VALID_LO[i];
	}
	o->out_definite_ticks = out_definite ? o->out_definite_ticks + 1 : 0;
	o->out_possible_ticks = out_possible ? o->out_possible_ticks + 1 : 0;
	if (o->out_definite_ticks > TIMEOUT_TICKS + SLACK_TICKS) o->latched_definite = true;
	if (o->out_possible_ticks > TIMEOUT_TICKS - SLACK_TICKS) o->latched_possible = true;

	Span t1 = travel_span(c[PEDAL_APPS1], PEDAL_APPS1);
	Span t2 = travel_span(c[PEDAL_APPS2], PEDAL_APPS2);
	float diff_min = fmaxf(fmaxf(t1.lo - t2.hi, t2.lo - t1.hi), 0.0f);
	float diff_max = fmaxf(t1.hi - t2.lo, t2.hi - t1.lo);
	bool disagree_possible = diff_max > APPS_MAX_DIFF;
	o->disagree_definite_ticks = diff_min > APPS_MAX_DIFF ? o->disagree_definite_ticks + 1 : 0;

	// EV.4.7 in the firmware's order: trigger, then release on the same tick.
	Span apps = apps_used(t1, t2);
	Span bse = c[PEDAL_BSE];
	if (apps.lo > EV47_TRIGGER && bse.lo > BSE_ACTIVATED_ADC_THRESHOLD) o->ev47_definite = true;
	if (apps.hi > EV47_TRIGGER && bse.hi > BSE_ACTIVATED_ADC_THRESHOLD) o->ev47_possible = true;
	if (apps.lo <= EV47_RELEASE) o->ev47_definite = false;
	if (apps.hi <= EV47_RELEASE) o->ev47_possible = false;

	bool cut = t->output == 0.0f; // torque, not just the flag: that's what matters
	uint32_t broken = 0, demands = 0;

	if (o->latched_definite) {
		demands |= BIT(RULE_RANGE_FAULT);
		if (!cut) broken |= BIT(RULE_RANGE_FAULT);
	}
	if (out_definite) {
		demands |= BIT(RULE_RANGE_INSTANT);
		if (!cut) broken |= BIT(RULE_RANGE_INSTANT);
	}
	if (o->disagree_definite_ticks > TIMEOUT_TICKS + SLACK_TICKS) {
		demands |= BIT(RULE_APPS_DISAGREE);
		if (!cut) broken |= BIT(RULE_APPS_DISAGREE);
	}
	if (o->ev47_definite) {
		demands |= BIT(RULE_EV47);
		if (!cut) broken |= BIT(RULE_EV47);
	}
	if (!o->latched_possible && !out_possible && !disagree_possible && !o->ev47_possible) {
		demands |= BIT(RULE_NO_FALSE_CUT);
		if (t->torque_cut) broken |= BIT(RULE_NO_FALSE_CUT);
	}

	if (t->output < 0.0f) {
		demands |= BIT(RULE_REGEN_TRUE_SPEED);
		if (!(t->true_speed_kph >= REGEN_MIN_KPH)) broken |= BIT(RULE_REGEN_TRUE_SPEED);
	}

	*armed = demands;
	return broken;
}
