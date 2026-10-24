/*
 * test_exhaustive.c
 *
 * Exhaustive and grid sweeps of the pedal checks, torque map, filter, car
 * speed and power cap: every input the firmware can see, where that's
 * small enough (every ADC count, every APPS1 x APPS2 pair, every 0x0A5 rpm),
 * and fine grids where it isn't (floats). No randomness, so a failure is the
 * same every run and prints the exact input.
 *
 * Timing (100 ms timers) can't be enumerated tick by tick, so it's swept
 * "bounded exhaustively": every sequence of up to three fault/good runs, with
 * run lengths picked around each boundary, checked against a small model of
 * the documented behavior.
 *
 * Built with regen on and off, and with MIN_APPS (tests/Makefile).
 * Run from the repo root:  make -C tests
 */

#include "check.h"
#include "pedal_checks.h"
#include "power_limit.h"
#include "torque_map.h"

#define DT_S 0.001f
#define ADC_COUNTS 4096
#define EPS 1e-4f

// Report only the first few failures of a sweep, not millions.
static int sweep_fails;
#define SWEEP_CHECK(cond, ...) do { \
	if (!(cond)) { \
		if (sweep_fails++ < 5) { printf("  FAIL %s:%d: %s  at ", __FILE__, __LINE__, #cond); \
			printf(__VA_ARGS__); printf("\n"); } \
		checks_failed++; \
	} \
} while (0)

static PlausibilityChecks pc;
static uint32_t now;

static bool tick_raw(uint16_t a1, uint16_t a2, uint16_t bse) {
	uint16_t adc[3] = { a1, a2, bse };
	float p[3];
	now++;
	pedalChecks_percents(adc, p);
	pedalChecks_update(&pc, adc, p, now);
	return pedalChecks_torqueCut(&pc, now);
}

// Fresh state, then one released tick so the startup cut clears.
static void start_driving(void) {
	now = 10000;
	pedalChecks_reset(&pc);
	tick_raw(APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, BSE_ADC_MIN_VAL);
}

static bool apps1_valid(int c) {
	return c >= APPS1_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER && c <= APPS1_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER;
}
static bool apps2_valid(int c) {
	return c >= APPS2_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER && c <= APPS2_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER;
}
static bool bse_valid(int c) {
	return c >= BSE_ADC_OPEN_CIRCUIT_VAL && c <= BSE_ADC_SHORT_CIRCUIT_VAL;
}

// Raw counts for APPS travel (0..1); APPS read lower when pressed.
static uint16_t apps1_adc(float t) {
	return (uint16_t)(APPS1_ADC_MIN_VAL - t * (APPS1_ADC_MIN_VAL - APPS1_ADC_MAX_VAL) + 0.5f);
}
static uint16_t apps2_adc(float t) {
	return (uint16_t)(APPS2_ADC_MIN_VAL - t * (APPS2_ADC_MIN_VAL - APPS2_ADC_MAX_VAL) + 0.5f);
}

// -------------------------------------------------
// Every APPS1 x APPS2 pair (16.7 M), one tick each
// -------------------------------------------------

static void test_every_apps_pair(void) {
	// From a healthy driving state, one tick at (a1, a2), brake off:
	//  - a reading is flagged invalid exactly when it's outside its range,
	//    and any invalid reading cuts that same tick (T.4.2.9, T.4.2.10)
	//  - two valid readings never cut on the first tick (the 100 ms grace),
	//    and are flagged implausible exactly when > 10% apart (T.4.2.4)
	sweep_fails = 0;
	for (int a1 = 0; a1 < ADC_COUNTS; a1++) {
		for (int a2 = 0; a2 < ADC_COUNTS; a2++) {
			start_driving();
			bool cut = tick_raw((uint16_t)a1, (uint16_t)a2, BSE_ADC_MIN_VAL);
			bool v1 = apps1_valid(a1), v2 = apps2_valid(a2);
			SWEEP_CHECK(pc.apps1_invalid == !v1, "a1=%d a2=%d", a1, a2);
			SWEEP_CHECK(pc.apps2_invalid == !v2, "a1=%d a2=%d", a1, a2);
			if (!v1 || !v2) {
				SWEEP_CHECK(cut, "a1=%d a2=%d", a1, a2);
				continue;
			}
			uint16_t adc[3] = { (uint16_t)a1, (uint16_t)a2, BSE_ADC_MIN_VAL };
			float p[3];
			pedalChecks_percents(adc, p);
			bool apart = fabsf(p[PEDAL_APPS1] - p[PEDAL_APPS2]) > APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE;
			SWEEP_CHECK(!cut, "a1=%d a2=%d", a1, a2);
			SWEEP_CHECK(pc.apps_plausible == !apart, "a1=%d a2=%d", a1, a2);
		}
	}
}

static void test_apps_percent_monotonic_in_each_sensor(void) {
	// Pressing either sensor further (lower counts) never lowers the APPS
	// travel used for torque, in whatever PEDAL_MODE this is built with.
	sweep_fails = 0;
	static float p1[ADC_COUNTS], p2[ADC_COUNTS];
	for (int c = 0; c < ADC_COUNTS; c++) {
		uint16_t adc[3] = { (uint16_t)c, (uint16_t)c, BSE_ADC_MIN_VAL };
		float p[3];
		pedalChecks_percents(adc, p);
		p1[c] = p[PEDAL_APPS1];
		p2[c] = p[PEDAL_APPS2];
	}
	for (int a1 = 0; a1 < ADC_COUNTS; a1++) {
		for (int a2 = 1; a2 < ADC_COUNTS; a2++) {
			float lower[3] = { p1[a1], p2[a2 - 1], 0 }, higher[3] = { p1[a1], p2[a2], 0 };
			SWEEP_CHECK(pedalChecks_appsPercent(lower) >= pedalChecks_appsPercent(higher), "a1=%d a2=%d", a1, a2);
			float l1[3] = { p1[a2 - 1], p2[a1], 0 }, h1[3] = { p1[a2], p2[a1], 0 };
			SWEEP_CHECK(pedalChecks_appsPercent(l1) >= pedalChecks_appsPercent(h1), "a1=%d a2=%d", a2, a1);
		}
	}
}

// ---------------------
// Every BSE count
// ---------------------

static void test_every_bse_count(void) {
	// With 30% throttle (over EV.4.7's 25%): cut exactly when the BSE is out
	// of range or pressed. RTD accepts exactly the pressed, in-range counts.
	sweep_fails = 0;
	for (int b = 0; b < ADC_COUNTS; b++) {
		start_driving();
		bool cut = tick_raw(apps1_adc(0.3f), apps2_adc(0.3f), (uint16_t)b);
		bool pressed = b > BSE_ACTIVATED_ADC_THRESHOLD;
		SWEEP_CHECK(pc.bse_invalid == !bse_valid(b), "bse=%d", b);
		SWEEP_CHECK(cut == (!bse_valid(b) || pressed), "bse=%d", b);
		SWEEP_CHECK(pedalChecks_brakeHeldForRTD((uint16_t)b) == (bse_valid(b) && pressed), "bse=%d", b);

		// Without throttle, braking alone never cuts.
		start_driving();
		cut = tick_raw(APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, (uint16_t)b);
		SWEEP_CHECK(cut == !bse_valid(b), "bse=%d no throttle", b);
	}
}

// ------------------------------------------------
// Torque map: pedal x speed x brake grid (~80 M)
// ------------------------------------------------

static void test_map_grid(void) {
	// Bounds, EV.3.3.3 (no regen under 5 km/h), no regen with the brake on
	// or backward, no drive torque below the inflection, and more pedal (or
	// less speed, in regen) never gives less torque.
	sweep_fails = 0;
	const float bses[] = { 0.0f, REGEN_BRAKE_CANCEL_PERCENT, REGEN_BRAKE_CANCEL_PERCENT + 0.001f, 1.0f };
	for (int bi = 0; bi < 4; bi++) {
		float bse = bses[bi];
		for (int si = 0; si <= 2000; si++) {
			float speed = -50.0f + si * 0.1f;
			float prev = -1e9f;
			for (int ai = 0; ai <= 10000; ai++) {
				float apps = ai * 1e-4f;
				float t = torqueMap_target(apps, bse, speed);
				SWEEP_CHECK(t >= REGEN_MAX_TORQUE && t <= MAX_TORQUE, "apps=%g bse=%g v=%g t=%g", apps, bse, speed, t);
				if (t < 0.0f) {
					SWEEP_CHECK(REGEN_ENABLED && speed >= REGEN_MIN_CARSPEED &&
							bse <= REGEN_BRAKE_CANCEL_PERCENT && apps < REGEN_END_PERCENT,
							"apps=%g bse=%g v=%g t=%g", apps, bse, speed, t);
				}
				if (t > 0.0f) SWEEP_CHECK(apps > APPS_INFLECTION_PERCENT, "apps=%g t=%g", apps, t);
				SWEEP_CHECK(t >= prev - EPS, "apps=%g bse=%g v=%g t=%g prev=%g", apps, bse, speed, t, prev);
				prev = t;
				if (si > 0 && apps < REGEN_END_PERCENT) {
					float slower = torqueMap_target(apps, bse, speed - 0.1f);
					SWEEP_CHECK(t <= slower + EPS, "apps=%g v=%g", apps, speed);
				}
			}
		}
	}
}

// ---------------------------------------------------------------
// One filter step from every state on a grid: invariants that hold
// whatever state the filter is in, reachable or not (~45 M steps)
// ---------------------------------------------------------------

static void test_request_step_from_any_state(void) {
	// EV.3.1.3 (output between 0 and the capped target), slew limit on the
	// way up, REGEN_RAMP_NM_S on the way into regen, cut = 0 Nm.
	sweep_fails = 0;
	const float speeds[] = { 0.0f, 4.99f, 5.0f, 8.0f, 14.0f, 30.0f };
	const float bses[] = { 0.0f, 0.06f };
	const float caps[] = { MAX_TORQUE, 40.0f, 0.0f };
	for (int ci = 0; ci < 3; ci++)
	for (int si = 0; si < 6; si++)
	for (int bi = 0; bi < 2; bi++)
	for (int ai = 0; ai <= 100; ai++) {
		float apps = ai * 0.01f;
		float target = torqueMap_target(apps, bses[bi], speeds[si]);
		float t = target > caps[ci] ? caps[ci] : target;
		float lo = t < 0.0f ? t : 0.0f, hi = t > 0.0f ? t : 0.0f;
		for (int fi = -15; fi <= 40; fi++)
		for (int oi = -60; oi <= 160; oi++) {
			float prev = oi * 0.5f;
			TorqueFilter f = { .filtered = fi * 2.0f, .output = prev, .target = 0.0f };
			float out = torqueMap_requestLimited(&f, false, apps, bses[bi], speeds[si], caps[ci], DT_S);
			SWEEP_CHECK(out >= lo - EPS && out <= hi + EPS, "apps=%g v=%g cap=%g prev=%g out=%g", apps, speeds[si], caps[ci], prev, out);
			if (out > 0.0f) {
				float base = prev > 0.0f ? prev : 0.0f;
				SWEEP_CHECK(out <= base + SLEW_RATE_LIMIT * DT_S + EPS, "prev=%g out=%g", prev, out);
			}
			if (out < 0.0f) {
				float base = prev < 0.0f ? prev : 0.0f;
				SWEEP_CHECK(out >= base - REGEN_RAMP_NM_S * DT_S - EPS, "prev=%g out=%g", prev, out);
			}
			SWEEP_CHECK(f.output == out && f.target == target, "state not saved");
			TorqueFilter g = { .filtered = fi * 2.0f, .output = prev, .target = 0.0f };
			SWEEP_CHECK(torqueMap_requestLimited(&g, true, apps, bses[bi], speeds[si], caps[ci], DT_S) == 0.0f, "cut");
		}
	}
}

// ---------------------------------------
// Every 0x0A5 rpm, at every age to stale
// ---------------------------------------

static void test_every_rpm_speed(void) {
	// Sign follows the rpm, speed rises with rpm, and the trusted speed is
	// never above the reported one, never negative from a forward one, and
	// 0 once stale.
	sweep_fails = 0;
	float prev = -1e9f;
	for (int rpm = -32768; rpm <= 32767; rpm++) {
		float v = torqueMap_carspeedKph((int16_t)rpm);
		SWEEP_CHECK((rpm * MOTOR_FORWARD_RPM_SIGN > 0) == (v > 0.0f), "rpm=%d", rpm);
		SWEEP_CHECK(MOTOR_FORWARD_RPM_SIGN > 0 ? v >= prev : v <= prev || rpm == -32768, "rpm=%d", rpm);
		prev = v;
		for (uint32_t age = 0; age <= CARSPEED_STALE_MS + 5; age++) {
			float tv = torqueMap_trustedCarspeed(v, 5000 + age, 5000);
			SWEEP_CHECK(tv <= v || v < 0.0f, "rpm=%d age=%u", rpm, (unsigned)age);
			if (v > 0.0f) SWEEP_CHECK(tv >= 0.0f, "rpm=%d age=%u", rpm, (unsigned)age);
			if (age > CARSPEED_STALE_MS) SWEEP_CHECK(tv == 0.0f, "rpm=%d age=%u", rpm, (unsigned)age);
		}
	}
}

static void test_every_rpm_power_cap(void) {
	sweep_fails = 0;
	for (int rpm = -32768; rpm <= 32767; rpm++) {
		float cap = powerLimit_capNm((float)rpm);
		double w = cap * fabs((double)rpm) * (2.0 * 3.14159265 / 60.0) / DRIVETRAIN_EFFICIENCY;
		SWEEP_CHECK(cap > 0.0f && cap <= MAX_TORQUE, "rpm=%d cap=%g", rpm, cap);
		SWEEP_CHECK(w <= POWER_LIMIT_W * 1.0001, "rpm=%d cap=%g w=%g", rpm, cap, w);
	}
}

// ------------------------------------------------------------------
// Timing, bounded exhaustive: every sequence of up to 3 (fault, good)
// runs, run lengths around each boundary, against a model of the docs
// ------------------------------------------------------------------

// Fault run lengths (ms): around the 100 ms limit. Good run lengths: around
// PEDAL_FAULT_CLEAR_MS, but not right on it (±1 tick is the firmware's call).
static const int FAULT_LENS[] = { 1, 2, 50, 99, 100, 101, 102, 150 };
static const int GOOD_LENS[] = { 1, 2, 50, PEDAL_FAULT_CLEAR_MS - 2, PEDAL_FAULT_CLEAR_MS + 3, 150 };
#define N_FAULT (int)(sizeof FAULT_LENS / sizeof FAULT_LENS[0])
#define N_GOOD (int)(sizeof GOOD_LENS / sizeof GOOD_LENS[0])
#define MAX_RUNS 3

// The documented behavior, written from run lengths instead of timestamps:
// a fault's timer starts at its first tick; good stretches shorter than
// PEDAL_FAULT_CLEAR_MS don't end it. Calls step(fault) once per tick and
// gets back how long the current fault has lasted (or -1 when good).
typedef struct Model {
	bool open;
	int start, good_ticks, t;
} Model;

static int model_step(Model *m, bool fault) {
	m->t++;
	if (fault) {
		if (!m->open) { m->open = true; m->start = m->t; }
		m->good_ticks = 0;
		return m->t - m->start;
	}
	if (m->open && ++m->good_ticks > PEDAL_FAULT_CLEAR_MS) m->open = false;
	return -1;
}

typedef bool (*TickFn)(bool fault);

static bool apps_tick(bool fault) {
	// 30% vs 50%: disagree. 40%/40%: agree.
	return fault ? tick_raw(apps1_adc(0.3f), apps2_adc(0.5f), BSE_ADC_MIN_VAL)
	             : tick_raw(apps1_adc(0.4f), apps2_adc(0.4f), BSE_ADC_MIN_VAL);
}

static bool bse_tick(bool fault) {
	// Unplugged BSE (open circuit) with the pedals released: a range fault only.
	return tick_raw(APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, fault ? 70 : BSE_ADC_MIN_VAL);
}

static int sequences;

// Run one sequence and compare every tick with the model.
static void run_sequence(const int *fault_len, const int *good_len, int runs, bool latch) {
	start_driving();
	Model m = { 0 };
	bool latched = false;
	sequences++;
	for (int r = 0; r < runs; r++) {
		for (int phase = 0; phase < 2; phase++) {
			bool fault = phase == 0;
			int len = fault ? fault_len[r] : good_len[r];
			for (int i = 0; i < len; i++) {
				int age = model_step(&m, fault);
				bool cut;
				if (latch) {
					cut = bse_tick(fault);
					if (age > SENSOR_FAULT_LATCH_MS) latched = true;
					SWEEP_CHECK(pc.sensor_fault_latched == latched, "run %d %s tick %d (seq %d)", r, fault ? "fault" : "good", i, sequences);
					SWEEP_CHECK(cut == (fault || latched), "run %d %s tick %d (seq %d)", r, fault ? "fault" : "good", i, sequences);
				} else {
					cut = apps_tick(fault);
					SWEEP_CHECK(cut == (age > APPS_IMPLAUSIBILITY_TIMEOUT_MS), "run %d %s tick %d age %d (seq %d)", r, fault ? "fault" : "good", i, age, sequences);
				}
			}
		}
	}
}

static void sweep_runs(bool latch) {
	int fl[MAX_RUNS], gl[MAX_RUNS];
	sweep_fails = 0;
	sequences = 0;
	for (int runs = 1; runs <= MAX_RUNS; runs++) {
		int combos = 1;
		for (int r = 0; r < runs; r++) combos *= N_FAULT * N_GOOD;
		for (int c = 0; c < combos; c++) {
			int k = c;
			for (int r = 0; r < runs; r++) {
				fl[r] = FAULT_LENS[k % N_FAULT]; k /= N_FAULT;
				gl[r] = GOOD_LENS[k % N_GOOD]; k /= N_GOOD;
			}
			run_sequence(fl, gl, runs, latch);
		}
	}
}

static void test_apps_disagreement_timing(void) {
	// T.4.2.5: cut on every disagreeing tick more than 100 ms into a
	// disagreement (good gaps under PEDAL_FAULT_CLEAR_MS don't restart it),
	// and never while the APPS agree.
	sweep_runs(false);
	printf("  %d sequences\n", sequences);
}

static void test_range_latch_timing(void) {
	// T.4.3.3: cut while out of range; latched (cut for good) once a fault,
	// with short gaps, has lasted more than 100 ms.
	sweep_runs(true);
	printf("  %d sequences\n", sequences);
}

int main(void) {
	printf("regen %d, PEDAL_MODE %d, MAX_TORQUE %d\n", REGEN_ENABLED, PEDAL_MODE, MAX_TORQUE);
	RUN(test_every_apps_pair);
	RUN(test_apps_percent_monotonic_in_each_sensor);
	RUN(test_every_bse_count);
	RUN(test_map_grid);
	RUN(test_request_step_from_any_state);
	RUN(test_every_rpm_speed);
	RUN(test_every_rpm_power_cap);
	RUN(test_apps_disagreement_timing);
	RUN(test_range_latch_timing);
	return CHECK_SUMMARY();
}
