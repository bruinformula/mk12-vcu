/*
 * test_mc.c
 *
 * Monte Carlo test: thousands of random 2 s drives through the real firmware
 * code, every tick checked against the rules in oracle.h. Unit tests check
 * the cases we thought of; this looks for the ones we didn't.
 *
 * Two levels, both run by default:
 *   map    phase 1: random inputs straight into torqueMap_request()
 *   chain  phases 2-3: true pedal positions -> sensor model with
 *          calibration error, noise and wiring faults (sensors.h) -> raw ADC
 *          counts -> pedalChecks -> torqueMap_request(), the same calls in
 *          the same order as calculateTorqueRequest() in motor_control.c.
 *          Car speed comes from a car model (vehicle.h) driven by the
 *          commanded torque, through a lossy, late 0x0A5 (inverter_can.h).
 *
 * Every run has its own seed, printed on failure, and a seed replays the
 * same run exactly (same build). tests/Makefile builds this with regen off,
 * regen on, and regen on with PEDAL_MODE=MIN_APPS.
 *
 * From the repo root, after `make -C tests`:
 *   tests/build/test_mc                          CI run: fixed seed, pass/fail
 *   tests/build/test_mc --runs 200000 --seed 7   bigger sweep, other seed
 *   tests/build/test_mc --level chain --replay 0x1f2e... --trace trace.csv
 *                                                one run, tick by tick, as CSV
 *   tests/build/test_mc --level map --nonfinite  also feed NaN/Inf inputs
 *
 * --nonfinite is off in CI: the firmware's inputs come from ADC counts and
 * an int16 rpm, so they can't be NaN/Inf. It shows what the map does if
 * that ever changes.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "oracle.h"
#include "pedal_checks.h"
#include "rng.h"
#include "scenario.h"
#include "inverter_can.h"
#include "sensors.h"
#include "torque_map.h"
#include "vehicle.h"

#define DT_S (MC_TICK_MS / 1000.0f)
#define TICKS_PER_RUN 2000 // 2 s of driving per run
#define DEFAULT_RUNS 5000
#define DEFAULT_SEED 1
#define MAX_REPORTS 5 // failing ticks printed in full; the rest are only counted

enum { LEVEL_MAP, LEVEL_CHAIN, LEVELS };
static const char *const LEVEL_NAMES[LEVELS] = { "map", "chain" };

typedef struct Stats {
	long ticks, drive_ticks, regen_ticks, max_torque_ticks, cut_ticks;
	long violations[RULE_COUNT];
	long armed[RULE_COUNT]; // chain: ticks each rule demanded something
	long faults[FAULT_TYPES]; // chain: faults injected
	long stale_ticks;         // chain: 0x0A5 older than CARSPEED_STALE_MS
	float min_regen_kph;      // chain: lowest true speed with regen commanded
	int reports;
} Stats;

static void report_rules(uint64_t seed, int tick, uint32_t broken) {
	printf("  FAIL run 0x%016llx tick %d:", (unsigned long long)seed, tick);
	for (int r = 0; r < RULE_COUNT; r++) {
		if (broken & (1u << r)) printf(" %s", RULE_NAMES[r]);
	}
	printf("\n");
}

static void count_tick(Stats *st, float out, bool cut, uint32_t broken) {
	st->ticks++;
	if (out > 0.0f) st->drive_ticks++;
	if (out < 0.0f) st->regen_ticks++;
	// The LPF only approaches the target, so "at max" means within 1%.
	if (out >= 0.99f * MAX_TORQUE) st->max_torque_ticks++;
	if (cut) st->cut_ticks++;
	for (int r = 0; r < RULE_COUNT; r++) {
		if (broken & (1u << r)) st->violations[r]++;
	}
}

// Phase 1: one 2 s drive, random inputs straight into the map.
static uint32_t run_map(uint64_t seed, bool nonfinite, Stats *st, FILE *trace) {
	Scenario s;
	scenario_init(&s, seed, nonfinite);
	// Own stream, so the monotonic probe doesn't change the drive itself.
	Rng probe = { rng_mix(seed) };
	TorqueFilter f;
	torqueMap_filterReset(&f);
	float prev = 0.0f;
	uint32_t run_broken = 0;

	if (trace) fprintf(trace, "tick,t_s,apps,bse,carspeed,torque_cut,target,output,filtered,broken\n");
	for (int tick = 0; tick < TICKS_PER_RUN; tick++) {
		ScenarioInputs in = scenario_step(&s);
		float out = torqueMap_request(&f, in.torque_cut, in.apps, in.bse, in.carspeed, DT_S);
		OracleTick t = {
			.in = in, .dt = DT_S, .prev_output = prev, .output = out, .f = &f,
			.apps_higher = in.apps + rng_uniform(&probe, 0.0f, 0.2f),
		};
		uint32_t broken = oracle_check(&t);
		count_tick(st, out, in.torque_cut, broken);

		if (broken && st->reports++ < MAX_REPORTS) {
			report_rules(seed, tick, broken);
			printf("       apps=%.6f bse=%.6f speed=%.4f cut=%d prev=%.4f out=%.4f target=%.4f\n",
					in.apps, in.bse, in.carspeed, in.torque_cut, prev, out, f.target);
		}
		if (trace) {
			fprintf(trace, "%d,%.3f,%.6f,%.6f,%.4f,%d,%.4f,%.4f,%.4f,0x%x\n",
					tick, tick * DT_S, in.apps, in.bse, in.carspeed, in.torque_cut,
					f.target, out, f.filtered, broken);
		}
		run_broken |= broken;
		prev = out;
	}
	return run_broken;
}

static float clamp01(float x) {
	return fminf(fmaxf(x, 0.0f), 1.0f);
}

// Phases 2-3: one 2 s drive through sensors, pedal checks, map and car.
static uint32_t run_chain(uint64_t seed, Stats *st, FILE *trace) {
	Scenario s;
	scenario_init(&s, seed, false);
	// Own streams, so faults and the probe don't change the drive itself.
	Sensors sens;
	sensors_init(&sens, rng_mix(seed ^ 0x5E5501ull), TICKS_PER_RUN);
	Rng probe = { rng_mix(seed) };
	Vehicle car;
	vehicle_init(&car, rng_mix(seed ^ 0xCA2ull));
	InverterCan can;
	inverter_can_init(&can, rng_mix(seed ^ 0xCA4ull), TICKS_PER_RUN);
	for (int i = 0; i < 3; i++) {
		if (sens.ch[i].fault != FAULT_NONE) st->faults[sens.ch[i].fault]++;
	}

	// Entering drive: the firmware resets both (motor_control.c).
	PlausibilityChecks pc;
	pedalChecks_reset(&pc);
	TorqueFilter f;
	torqueMap_filterReset(&f);
	ChainOracle o;
	chain_oracle_init(&o);
	// HAL_GetTick() can be anywhere, including about to wrap.
	uint32_t now_ms = rng_chance(&probe, 0.2f) ? UINT32_MAX - (uint32_t)rng_int(&probe, 0, TICKS_PER_RUN)
			: (uint32_t)rng_u64(&probe);
	// No 0x0A5 yet: the speed starts stale (0 km/h, no regen).
	float inverter_carspeed = 0.0f;
	uint32_t rpm_rx_ms = now_ms - 1000;
	float prev = 0.0f;
	uint32_t run_broken = 0;

	if (trace) {
		fprintf(trace, "tick,t_s,now_ms,true_apps,true_bse,true_kph,vcu_kph,adc_apps1,adc_apps2,adc_bse,"
				"faulted,apps_percent,bse_percent,torque_cut,target,output,broken\n");
	}
	for (int tick = 0; tick < TICKS_PER_RUN; tick++, now_ms += MC_TICK_MS) {
		ScenarioInputs truth = scenario_step(&s);
		// Real pedals can't go past their stops; overtravel in the readings
		// comes from the sensor model.
		truth.apps = clamp01(truth.apps);
		truth.bse = clamp01(truth.bse);
		uint16_t adc[3];
		sensors_read(&sens, tick, truth.apps, truth.bse, adc);
		// processInverter_RPM(), when a 0x0A5 arrives.
		int16_t rpm;
		if (inverter_can_rx(&can, tick, vehicle_motor_rpm(&car), &rpm)) {
			inverter_carspeed = torqueMap_carspeedKph(rpm);
			rpm_rx_ms = now_ms;
		}

		// Same calls, same order as calculateTorqueRequest() in motor_control.c.
		float carspeed = torqueMap_trustedCarspeed(inverter_carspeed, now_ms, rpm_rx_ms);
		float percents[3];
		pedalChecks_percents(adc, percents);
		pedalChecks_update(&pc, adc, percents, now_ms);
		bool cut = pedalChecks_torqueCut(&pc, now_ms);
		float apps_percent = pedalChecks_appsPercent(percents);
		float out = torqueMap_request(&f, cut, apps_percent, percents[PEDAL_BSE],
				carspeed, DT_S);

		// Map rules see what the map saw; chain rules see the truth.
		OracleTick t = {
			.in = { apps_percent, percents[PEDAL_BSE], carspeed, cut },
			.dt = DT_S, .prev_output = prev, .output = out, .f = &f,
			.apps_higher = apps_percent + rng_uniform(&probe, 0.0f, 0.2f),
		};
		ChainTick ct = {
			.sensors = &sens, .torque_cut = cut, .output = out, .true_speed_kph = car.speed_kph,
		};
		uint32_t armed;
		uint32_t broken = oracle_check(&t) | chain_check(&o, &ct, &armed);
		count_tick(st, out, cut, broken);
		for (int r = 0; r < RULE_COUNT; r++) {
			if (armed & (1u << r)) st->armed[r]++;
		}
		if (now_ms - rpm_rx_ms > CARSPEED_STALE_MS) st->stale_ticks++;
		if (out < 0.0f) st->min_regen_kph = fminf(st->min_regen_kph, car.speed_kph);

		int faulted = sens.ch[0].faulted | sens.ch[1].faulted << 1 | sens.ch[2].faulted << 2;
		if (broken && st->reports++ < MAX_REPORTS) {
			report_rules(seed, tick, broken);
			printf("       truth apps=%.4f bse=%.4f kph=%.3f | adc %u %u %u faulted=0x%x"
					" | apps%%=%.4f bse%%=%.4f kph=%.3f cut=%d out=%.4f\n",
					truth.apps, truth.bse, car.speed_kph, adc[0], adc[1], adc[2], faulted,
					apps_percent, percents[PEDAL_BSE], carspeed, cut, out);
		}
		if (trace) {
			fprintf(trace, "%d,%.3f,%u,%.4f,%.4f,%.4f,%.4f,%u,%u,%u,0x%x,%.4f,%.4f,%d,%.4f,%.4f,0x%x\n",
					tick, tick * DT_S, (unsigned)now_ms, truth.apps, truth.bse, car.speed_kph, carspeed,
					adc[0], adc[1], adc[2], faulted, apps_percent, percents[PEDAL_BSE],
					cut, f.target, out, broken);
		}
		run_broken |= broken;
		prev = out;
		vehicle_step(&car, out, truth.bse, DT_S);
	}
	return run_broken;
}

static double pct(long n, long total) {
	return total ? 100.0 * n / total : 0.0;
}

// Runs one level and prints its summary. Returns true if every rule held.
static bool run_level(int level, long runs, uint64_t seed, bool replay, uint64_t replay_seed,
		bool nonfinite, FILE *trace, const char *prog) {
	Stats st = { .min_regen_kph = INFINITY };
	long failed_runs = 0;
	uint64_t first_failed = 0;

	for (long i = 0; i < runs; i++) {
		uint64_t run_seed = replay ? replay_seed : rng_mix(rng_mix(seed) + (uint64_t)i);
		// Trace only the first run: a trace is for looking at one drive.
		FILE *tr = i == 0 ? trace : NULL;
		uint32_t broken = level == LEVEL_MAP ? run_map(run_seed, nonfinite, &st, tr)
				: run_chain(run_seed, &st, tr);
		if (broken && failed_runs++ == 0) first_failed = run_seed;
	}

	if (replay) {
		printf("Monte Carlo %s replay: run 0x%016llx", LEVEL_NAMES[level],
				(unsigned long long)replay_seed);
	} else {
		printf("Monte Carlo %s: %ld runs x %d ticks, seed %llu", LEVEL_NAMES[level], runs,
				TICKS_PER_RUN, (unsigned long long)seed);
	}
	printf(", regen %s, PEDAL_MODE %d%s\n", REGEN_ENABLED ? "on" : "off", PEDAL_MODE,
			nonfinite && level == LEVEL_MAP ? ", NaN/Inf inputs" : "");
	printf("  coverage: drive %.1f%%, near MAX_TORQUE %.1f%%, regen %.1f%%, cut %.1f%% of ticks\n",
			pct(st.drive_ticks, st.ticks), pct(st.max_torque_ticks, st.ticks),
			pct(st.regen_ticks, st.ticks), pct(st.cut_ticks, st.ticks));
	if (level == LEVEL_CHAIN) {
		printf("  faults injected:");
		for (int k = FAULT_NONE + 1; k < FAULT_TYPES; k++) {
			printf(" %s %ld", FAULT_NAMES[k], st.faults[k]);
		}
		printf("\n  speed: 0x0A5 stale %.1f%% of ticks; lowest true speed with regen ", pct(st.stale_ticks, st.ticks));
		if (isinf(st.min_regen_kph)) {
			printf("n/a (no regen)\n");
		} else {
			printf("%.2f km/h (limit 5)\n", st.min_regen_kph);
		}
	}

	bool ok = true;
	int last_rule = level == LEVEL_MAP ? RULE_OVERTRAVEL : RULE_COUNT - 1;
	for (int r = 0; r <= last_rule; r++) {
		printf("%s %-14s", st.violations[r] ? "FAIL" : "ok  ", RULE_NAMES[r]);
		if (st.violations[r]) printf(" %ld ticks", st.violations[r]);
		if (r > RULE_OVERTRAVEL) printf(" (armed %.1f%% of ticks)", pct(st.armed[r], st.ticks));
		printf("\n");
		if (st.violations[r]) ok = false;
	}

	// A generator that never reaches a region proves nothing about it.
	if (!replay) {
		bool reached = st.drive_ticks && st.max_torque_ticks && st.cut_ticks &&
				(!REGEN_ENABLED || st.regen_ticks);
		for (int r = RULE_OVERTRAVEL + 1; level == LEVEL_CHAIN && r < RULE_COUNT; r++) {
			if (r == RULE_REGEN_TRUE_SPEED && !REGEN_ENABLED) continue; // can't arm
			reached = reached && st.armed[r];
		}
		if (!reached) {
			printf("FAIL coverage: some region or rule was never reached\n");
			ok = false;
		}
	}

	if (failed_runs) {
		printf("%ld of %ld runs failed. Replay the first one with:\n"
				"  %s --level %s --replay 0x%016llx%s --trace trace.csv\n",
				failed_runs, runs, prog, LEVEL_NAMES[level], (unsigned long long)first_failed,
				nonfinite ? " --nonfinite" : "");
	}
	printf("\n");
	return ok;
}

static int usage(const char *prog) {
	fprintf(stderr, "usage: %s [--level map|chain] [--runs N] [--seed S] [--replay RUN_SEED]"
			" [--trace FILE.csv] [--nonfinite]\n", prog);
	return 2;
}

int main(int argc, char **argv) {
	long runs = DEFAULT_RUNS;
	uint64_t seed = DEFAULT_SEED;
	int only_level = -1; // -1: both
	bool replay = false, nonfinite = false;
	uint64_t replay_seed = 0;
	const char *trace_path = NULL;

	for (int i = 1; i < argc; i++) {
		bool has_value = i + 1 < argc;
		if (!strcmp(argv[i], "--level") && has_value) {
			i++;
			only_level = !strcmp(argv[i], "map") ? LEVEL_MAP : !strcmp(argv[i], "chain") ? LEVEL_CHAIN : -2;
			if (only_level == -2) return usage(argv[0]);
		} else if (!strcmp(argv[i], "--runs") && has_value) {
			runs = strtol(argv[++i], NULL, 0);
		} else if (!strcmp(argv[i], "--seed") && has_value) {
			seed = strtoull(argv[++i], NULL, 0);
		} else if (!strcmp(argv[i], "--replay") && has_value) {
			replay = true;
			replay_seed = strtoull(argv[++i], NULL, 0);
		} else if (!strcmp(argv[i], "--trace") && has_value) {
			trace_path = argv[++i];
		} else if (!strcmp(argv[i], "--nonfinite")) {
			nonfinite = true;
		} else {
			return usage(argv[0]);
		}
	}
	// A replay or trace is one drive at one level.
	if ((replay || trace_path) && only_level < 0) {
		fprintf(stderr, "--replay and --trace need --level map or --level chain\n");
		return 2;
	}
	if (replay) runs = 1;
	if (runs < 1) return usage(argv[0]);

	FILE *trace = NULL;
	if (trace_path) {
		trace = fopen(trace_path, "w");
		if (!trace) {
			perror(trace_path);
			return 2;
		}
	}

	bool ok = true;
	for (int level = 0; level < LEVELS; level++) {
		if (only_level >= 0 && level != only_level) continue;
		ok = run_level(level, runs, seed, replay, replay_seed, nonfinite, trace, argv[0]) && ok;
	}
	if (trace) fclose(trace);

	printf("%s\n", ok ? "all rules held" : "MONTE CARLO FAILED");
	return ok ? 0 : 1;
}
