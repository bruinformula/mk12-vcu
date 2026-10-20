/*
 * scenario.c
 *
 * Random driving for the Monte Carlo tests. See scenario.h.
 */

#include "scenario.h"
#include "torque_map.h"

#define COUNT(a) ((int)(sizeof(a) / sizeof((a)[0])))

// Values on every threshold in torque_map.h, plus overtravel past both ends
// (the overshoot buffer lets real readings land up to ~12% outside [0, 1]).
static const float APPS_EDGES[] = {
	-0.12f, 0.0f, REGEN_END_PERCENT, APPS_INFLECTION_PERCENT, 0.5f, 1.0f, 1.12f,
};
static const float BSE_EDGES[] = {
	-0.12f, 0.0f, REGEN_BRAKE_CANCEL_PERCENT, 1.0f, 1.12f,
};
static const float SPEED_EDGES[] = {
	-10.0f, 0.0f, REGEN_MIN_CARSPEED, REGEN_FADE_LO_KPH, REGEN_FADE_HI_KPH, 120.0f,
};
static const float NONFINITE[] = { NAN, INFINITY, -INFINITY };

// An edge value or a hair either side of it: off-by-one comparisons
// (< vs <=) only show up right on a threshold.
static float near_edge(Rng *r, const float *edges, int count) {
	float v = rng_pick(r, edges, count);
	switch (rng_int(r, 0, 2)) {
	case 0: return v - 1e-5f;
	case 1: return v + 1e-5f;
	default: return v;
	}
}

static float pick_apps(Rng *r) {
	if (rng_chance(r, 0.3f)) return near_edge(r, APPS_EDGES, COUNT(APPS_EDGES));
	return rng_uniform(r, -0.12f, 1.12f);
}

static float pick_bse(Rng *r) {
	if (rng_chance(r, 0.5f)) return 0.0f; // mostly off the brake
	if (rng_chance(r, 0.4f)) return near_edge(r, BSE_EDGES, COUNT(BSE_EDGES));
	return rng_uniform(r, -0.12f, 1.12f);
}

static float pick_speed(Rng *r) {
	if (rng_chance(r, 0.4f)) return near_edge(r, SPEED_EDGES, COUNT(SPEED_EDGES));
	return rng_uniform(r, -10.0f, 120.0f);
}

void scenario_init(Scenario *s, uint64_t seed, bool nonfinite) {
	*s = (Scenario){ .rng = { seed }, .nonfinite = nonfinite };
	s->speed = pick_speed(&s->rng);
}

static void new_segment(Scenario *s) {
	Rng *r = &s->rng;
	s->segment_ticks = rng_int(r, 1, 300);
	s->apps_noise = rng_chance(r, 0.3f) ? rng_uniform(r, 0.0f, 0.02f) : 0.0f;
	s->apps_goal = s->apps;
	s->apps_rate = 0.0f;

	switch (rng_int(r, 0, 2)) {
	case 0: // hold
		break;
	case 1: // step: stomp or lift in one tick
		s->apps = s->apps_goal = pick_apps(r);
		break;
	default: // ramp at 0.1 to 20 full pedals per second
		s->apps_goal = pick_apps(r);
		s->apps_rate = rng_uniform(r, 0.0001f, 0.02f);
		break;
	}

	if (rng_chance(r, 0.3f)) s->bse = pick_bse(r);
	// A jump stands in for a stale or glitched speed reading.
	if (rng_chance(r, 0.1f)) s->speed = pick_speed(r);
	s->accel = rng_uniform(r, -0.03f, 0.03f); // up to 30 km/h per second
	if (s->cut_ticks <= 0 && rng_chance(r, 0.1f)) s->cut_ticks = rng_int(r, 1, 200);
}

ScenarioInputs scenario_step(Scenario *s) {
	Rng *r = &s->rng;
	if (s->segment_ticks <= 0) new_segment(s);
	s->segment_ticks--;

	if (s->apps < s->apps_goal) {
		s->apps = fminf(s->apps + s->apps_rate, s->apps_goal);
	} else if (s->apps > s->apps_goal) {
		s->apps = fmaxf(s->apps - s->apps_rate, s->apps_goal);
	}
	s->speed = fminf(fmaxf(s->speed + s->accel, -15.0f), 130.0f);

	ScenarioInputs in = {
		.apps = s->apps + (s->apps_noise > 0.0f ? rng_gauss(r, s->apps_noise) : 0.0f),
		.bse = s->bse,
		.carspeed = s->speed,
		.torque_cut = s->cut_ticks > 0 || rng_chance(r, 0.001f), // plus 1-tick blips
	};
	if (s->cut_ticks > 0) s->cut_ticks--;

	if (s->nonfinite && rng_chance(r, 0.001f)) {
		float bad = rng_pick(r, NONFINITE, COUNT(NONFINITE));
		switch (rng_int(r, 0, 2)) {
		case 0: in.apps = bad; break;
		case 1: in.bse = bad; break;
		default: in.carspeed = bad; break;
		}
	}
	return in;
}
