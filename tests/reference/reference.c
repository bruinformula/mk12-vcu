/*
 * reference.c
 *
 * Reference outputs (docs/profiles.md): the real torque_map.c
 * and power_limit.c run over a fixed input sweep, one CSV row per output.
 * Every other implementation of the pipeline (vcu.lua in mk12-assetto-sim,
 * the editor's preview) is tested against these files.
 *
 * Built once per profile by `make -C tests profiles`. With PROFILE_HEADER
 * set, the profile's values replace the firmware's tunables; without it,
 * this is the firmware exactly as it is (the check that base.yaml matches).
 *
 * Until the firmware wires profile_generated.h into the firmware headers, the
 * swap happens here: the firmware .c files are #included into this file,
 * the tunables are #undef'd and redefined in between, and `const` is
 * defined away so the pedal table can be filled in at startup. Nothing in
 * Core/ changes.
 *
 * CSV columns (empty where they don't apply):
 *   kind      map | speed | cap | trace
 *   scenario  trace name
 *   tick      trace step, from 0
 *   dt        trace step length (s)
 *   apps bse  pedal travel, 0..1 (before torqueMap_target's clamp)
 *   speed_kph car speed fed to the pipeline; for kind=speed, the output
 *   motor_rpm input to powerLimit_capNm / torqueMap_carspeedKph
 *   cut       1 if the tick is a torque cut
 *   target_nm torqueMap_target (map) or f.target, the driver's request (trace)
 *   cap_nm    powerLimit_capNm
 *   out_nm    torqueMap_requestLimited output, what goes to the inverter
 */

// Every system header the firmware files use, before `const` goes away.
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef PROFILE_HEADER
#define const
#endif
#include "torque_map.h"
#include "power_limit.h"
#ifdef PROFILE_HEADER
#undef const
#undef MAX_TORQUE
#undef REGEN_END_PERCENT
#undef APPS_INFLECTION_PERCENT
#undef REGEN_LIFTOFF_TORQUE
#undef REGEN_FADE_LO_KPH
#undef REGEN_FADE_HI_KPH
#undef REGEN_RAMP_NM_S
#undef RC_TIME_CONSTANT
#undef SLEW_RATE_LIMIT
#include PROFILE_HEADER
#define const
#endif
#include "../../Core/Src/torque_map.c"
#include "../../Core/Src/power_limit.c"
#ifdef PROFILE_HEADER
#undef const
#else
#define PROFILE_NAME "firmware"
#endif

static void row(const char *kind, const char *scenario, int tick, float dt,
		float apps, float bse, float speed, float rpm, int cut,
		float target, float cap, float out) {
	// %.9g: every float prints exactly, so two builds can be compared bit for bit.
	printf("%s,%s,", kind, scenario);
	if (tick >= 0) printf("%d,%.9g,", tick, dt); else printf(",,");
	if (!isnan(apps)) printf("%.9g,%.9g,", apps, bse); else printf(",,");
	if (!isnan(speed)) printf("%.9g,", speed); else printf(",");
	if (!isnan(rpm)) printf("%.9g,", rpm); else printf(",");
	if (cut >= 0) printf("%d,", cut); else printf(",");
	if (!isnan(target)) printf("%.9g,", target); else printf(",");
	if (!isnan(cap)) printf("%.9g,", cap); else printf(",");
	if (!isnan(out)) printf("%.9g\n", out); else printf("\n");
}

// Map: target torque over pedal, brake and speed. Covers the clamp below 0
// and above 1, every band edge and the regen speed fade.
static void sweepMap(void) {
	static const float bses[] = { 0.0f, 0.04f, 0.06f, 0.5f };
	static const float speeds[] = { -10.0f, 0.0f, 4.9f, 5.0f, 8.0f, 11.0f, 14.0f, 20.0f, 27.0f, 60.0f, 120.0f };
	for (int a = -5; a <= 105; a++) {
		float apps = a / 100.0f;
		for (unsigned b = 0; b < sizeof bses / sizeof bses[0]; b++) {
			for (unsigned s = 0; s < sizeof speeds / sizeof speeds[0]; s++) {
				row("map", "", -1, 0, apps, bses[b], speeds[s], NAN, -1,
						torqueMap_target(apps, bses[b], speeds[s]), NAN, NAN);
			}
		}
	}
	// Finer around the bands, where profiles move the edges.
	for (int a = 0; a <= 250; a++) {
		float apps = a / 1000.0f;
		row("map", "", -1, 0, apps, 0.0f, 60.0f, NAN, -1, torqueMap_target(apps, 0.0f, 60.0f), NAN, NAN);
	}
}

static void sweepSpeedAndCap(void) {
	for (int rpm = -2000; rpm <= 8000; rpm += 250) {
		row("speed", "", -1, 0, NAN, 0, torqueMap_carspeedKph((int16_t)rpm), (float)rpm, -1, NAN, NAN, NAN);
	}
	for (int rpm = -2000; rpm <= 8000; rpm += 100) {
		row("cap", "", -1, 0, NAN, 0, NAN, (float)rpm, -1, NAN, powerLimit_capNm((float)rpm), NAN);
	}
}

// Trace inputs at time t (s). Speed and motor rpm are kept consistent
// through torqueMap_carspeedKph.
typedef struct Inputs {
	float apps, bse, speed, rpm;
	bool cut;
} Inputs;

static float kphPerRpm(void) {
	return torqueMap_carspeedKph(1000) / 1000.0f;
}

static Inputs at(float apps, float bse, float speed, bool cut) {
	Inputs in = { apps, bse, speed, speed / kphPerRpm(), cut };
	return in;
}

// Steps in and out of drive, then lift off into regen at speed.
static Inputs tipIn(float t) {
	float apps = (t < 0.1f) ? 0.0f : (t < 0.5f) ? 1.0f : (t < 0.8f) ? 0.5f : 0.0f;
	return at(apps, 0.0f, 40.0f, false);
}

// Slow ramp up and down through every band.
static Inputs ramp(float t) {
	float apps = (t < 0.8f) ? t / 0.8f : (t < 1.6f) ? (1.6f - t) / 0.8f : 0.0f;
	return at(apps, 0.0f, 60.0f, false);
}

// Lifted off, slowing from 40 km/h to a stop through the fade and the
// cutoff, with a brake press that cancels regen and a light one that doesn't.
static Inputs coastdown(float t) {
	float speed = 40.0f - 20.0f * t;
	if (speed < 0.0f) speed = 0.0f;
	float bse = (t > 0.4f && t < 0.6f) ? 0.3f : (t > 0.8f && t < 1.0f) ? 0.03f : 0.0f;
	float apps = (t > 1.2f && t < 1.25f) ? 0.02f : 0.0f;
	return at(apps, bse, speed, false);
}

// Torque cuts: a long one mid-drive, a single tick, and one during regen.
static Inputs cuts(float t) {
	float apps = (t < 0.6f) ? 0.8f : 0.0f;
	bool cut = (t >= 0.2f && t < 0.25f) || (t >= 0.4f && t < 0.401f) || (t >= 0.8f && t < 0.85f);
	return at(apps, 0.0f, 50.0f, cut);
}

// Full pedal while the motor spins up past the power cap knee.
static Inputs powerRun(float t) {
	Inputs in = at(1.0f, 0.0f, 0.0f, false);
	in.rpm = 7000.0f * t / 1.5f;
	in.speed = in.rpm * kphPerRpm();
	return in;
}

// Pseudo-random pedal, brake and speed at the sim's 333 Hz physics step:
// lots of drive/coast/regen transitions. Fixed seed, so always the same.
static uint32_t rng = 12345u;
static float rnd(void) {
	rng = rng * 1664525u + 1013904223u;
	return (rng >> 8) / 16777216.0f;
}
static Inputs held;
static Inputs mixed(float t) {
	static int last_slot = -1;
	int slot = (int)(t / 0.05f);
	if (slot != last_slot) {
		last_slot = slot;
		float r = rnd();
		float apps = (r < 0.3f) ? 0.0f : (r < 0.4f) ? rnd() * 0.12f : rnd() * 1.05f;
		float bse = (rnd() < 0.15f) ? rnd() * 0.4f : 0.0f;
		float speed = rnd() * 90.0f - 5.0f;
		held = at(apps, bse, speed, rnd() < 0.03f);
	}
	return held;
}

static void trace(const char *name, Inputs (*inputs)(float), float dt, float seconds) {
	TorqueFilter f;
	torqueMap_filterReset(&f);
	int ticks = (int)(seconds / dt + 0.5f);
	for (int i = 0; i < ticks; i++) {
		Inputs in = inputs(i * dt);
		float cap = powerLimit_capNm(in.rpm);
		float out = torqueMap_requestLimited(&f, in.cut, in.apps, in.bse, in.speed, cap, dt);
		row("trace", name, i, dt, in.apps, in.bse, in.speed, in.rpm, in.cut, f.target, cap, out);
	}
}

int main(void) {
#ifdef PROFILE_HEADER
	static const float table[TORQUE_MAP_POINTS] = TORQUE_MAP_TABLE;
	memcpy((float *)torqueMap_table, table, sizeof table);
#endif
	if (!torqueMap_tableIsValid(torqueMap_table, TORQUE_MAP_POINTS)) {
		fprintf(stderr, "reference: %s pedal table fails torqueMap_tableIsValid\n", PROFILE_NAME);
		return 1;
	}
	printf("# Reference outputs for profile %s. Generated by make -C tests reference-update; do not edit.\n",
			PROFILE_NAME);
	printf("# REGEN_ENABLED %d. Columns: tests/reference/reference.c.\n", REGEN_ENABLED);
	printf("kind,scenario,tick,dt,apps,bse,speed_kph,motor_rpm,cut,target_nm,cap_nm,out_nm\n");
	sweepMap();
	sweepSpeedAndCap();
	trace("tip_in", tipIn, 0.001f, 1.2f);
	trace("ramp", ramp, 0.001f, 1.8f);
	trace("coastdown", coastdown, 0.001f, 2.2f);
	trace("cuts", cuts, 0.001f, 1.0f);
	trace("power_run", powerRun, 0.001f, 1.5f);
	trace("mixed_333hz", mixed, 1.0f / 333.0f, 6.0f);
	return 0;
}
