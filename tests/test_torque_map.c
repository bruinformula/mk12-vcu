/*
 * test_torque_map.c
 *
 * Host tests for Core/Src/torque_map.c (the exact file the car runs).
 *
 * These are characterization tests: they pin what the map and filter do
 * TODAY, including known problems (marked CURRENT BEHAVIOR). When a later
 * phase changes behavior on purpose, the matching test should be changed
 * in the same commit, so every behavior change is visible in review.
 *
 * Built twice by tests/Makefile: as shipped (REGEN_ENABLED 0, regen off) and
 * with -DREGEN_ENABLED=1, so the regen code stays covered while it's off.
 *
 * Run from the repo root:  make -C tests
 */

#include "check.h"
#include "torque_map.h"

// Must match CONTROL_LOOP_HZ in motor_control.h (not included: it pulls in HAL).
#define DT_S 0.001f

#define NO_BRAKE 0.0f
#define STOPPED 0.0f
#define MOVING 10.0f // above REGEN_MIN_CARSPEED

// ---------------------------------
// torqueMap_target: positive torque
// ---------------------------------

static void test_zero_torque_at_inflection(void) {
	CHECK_NEAR(torqueMap_target(APPS_INFLECTION_PERCENT, NO_BRAKE, STOPPED), 0.0, 1e-4);
}

static void test_full_pedal_gives_max_torque(void) {
	CHECK_NEAR(torqueMap_target(1.0f, NO_BRAKE, STOPPED), MAX_TORQUE, 1e-4);
}

static void test_overtravel_clamped_to_max_torque(void) {
	CHECK_NEAR(torqueMap_target(1.2f, NO_BRAKE, STOPPED), MAX_TORQUE, 1e-4);
}

// Pedal % at a position along the map (0 = inflection, 1 = full pedal).
static float apps_at(float travel) {
	return APPS_INFLECTION_PERCENT + travel * (1.0f - APPS_INFLECTION_PERCENT);
}

static void test_half_travel_gives_half_torque(void) {
	// Default map is a straight line (MK11 feel). Retuning the table should
	// change this test in the same commit.
	CHECK_NEAR(torqueMap_target(apps_at(0.5f), NO_BRAKE, STOPPED), MAX_TORQUE / 2.0, 1e-3);
}

static void test_shipped_table_is_valid(void) {
	// Same check the VCU runs at boot (main.c).
	CHECK(torqueMap_tableIsValid(torqueMap_table, TORQUE_MAP_POINTS));
}

static void test_map_hits_table_points(void) {
	for (int i = 0; i < TORQUE_MAP_POINTS; i++) {
		float travel = (float)i / (TORQUE_MAP_POINTS - 1);
		CHECK_NEAR(torqueMap_target(apps_at(travel), NO_BRAKE, STOPPED),
				MIN_TORQUE + (MAX_TORQUE - MIN_TORQUE) * torqueMap_table[i], 1e-3);
	}
}

static void test_map_interpolates_between_points(void) {
	// Halfway between two table points is the average of the two.
	for (int i = 0; i < TORQUE_MAP_POINTS - 1; i++) {
		float travel = (i + 0.5f) / (TORQUE_MAP_POINTS - 1);
		double expected = (MAX_TORQUE - MIN_TORQUE) * (torqueMap_table[i] + torqueMap_table[i + 1]) / 2.0;
		CHECK_NEAR(torqueMap_target(apps_at(travel), NO_BRAKE, STOPPED), MIN_TORQUE + expected, 1e-3);
	}
}

static void test_table_validator_rejects_bad_maps(void) {
	const float good[] = { 0.0f, 0.3f, 0.3f, 1.0f }; // flat spots are allowed
	const float dips[] = { 0.0f, 0.5f, 0.4f, 1.0f }; // more pedal, less torque
	const float no_zero[] = { 0.1f, 0.5f, 0.8f, 1.0f }; // torque at the inflection point
	const float short_top[] = { 0.0f, 0.3f, 0.6f, 0.9f }; // never reaches MAX_TORQUE
	const float overshoot[] = { 0.0f, 1.2f, 1.2f, 1.0f };
	CHECK(torqueMap_tableIsValid(good, 4));
	CHECK(!torqueMap_tableIsValid(dips, 4));
	CHECK(!torqueMap_tableIsValid(no_zero, 4));
	CHECK(!torqueMap_tableIsValid(short_top, 4));
	CHECK(!torqueMap_tableIsValid(overshoot, 4));
	CHECK(!torqueMap_tableIsValid(good, 1));
}

static void test_positive_torque_is_monotonic(void) {
	float prev = torqueMap_target(APPS_INFLECTION_PERCENT, NO_BRAKE, STOPPED);
	for (int i = 1; i <= 1000; i++) {
		float apps = APPS_INFLECTION_PERCENT + (1.0f - APPS_INFLECTION_PERCENT) * i / 1000.0f;
		float t = torqueMap_target(apps, NO_BRAKE, STOPPED);
		CHECK(t >= prev);
		prev = t;
	}
}

// -----------------------
// torqueMap_target: regen
// -----------------------

#if !REGEN_ENABLED
static void test_regen_disabled_coasts(void) {
	// Regen off: every pedal below the inflection point gives 0 Nm, at any
	// speed, including the negative readings from the overshoot buffer.
	for (int a = -10; a < (int)(APPS_INFLECTION_PERCENT * 1000); a++) {
		for (int v = 0; v <= 120; v += 5) {
			CHECK(torqueMap_target(a / 1000.0f, NO_BRAKE, (float)v) == 0.0f);
		}
	}
}
#endif

static void test_no_regen_below_min_carspeed(void) {
	CHECK_NEAR(torqueMap_target(0.0f, NO_BRAKE, REGEN_MIN_CARSPEED - 0.1f), 0.0, 1e-4);
}

static void test_regen_cutoff_meets_ev_3_3_3(void) {
	// EV.3.3.3: no regen between 0 and 5 km/h. Tuning must not lower this.
	CHECK(REGEN_MIN_CARSPEED >= 5.0f);
}

static void test_no_regen_with_brake_pressed(void) {
	CHECK_NEAR(torqueMap_target(0.0f, REGEN_BRAKE_CANCEL_PERCENT + 0.01f, MOVING), 0.0, 1e-4);
}

#if REGEN_ENABLED
static void test_regen_allowed_at_min_carspeed(void) {
	CHECK(torqueMap_target(0.0f, NO_BRAKE, REGEN_MIN_CARSPEED) < 0.0f);
}

static void test_regen_scales_linearly_below_inflection(void) {
	CHECK_NEAR(torqueMap_target(0.0f, NO_BRAKE, MOVING), REGEN_MAX_TORQUE, 1e-4);
	CHECK_NEAR(torqueMap_target(APPS_INFLECTION_PERCENT / 2, NO_BRAKE, MOVING),
			REGEN_MAX_TORQUE / 2.0, 1e-4);
}

static void test_negative_pedal_regen_clamped(void) {
	// Pedal % goes negative when the ADC reads above APPS2_ADC_MIN_VAL but still
	// inside the 125-count overshoot buffer (worst case -125/1495 = -8.4%).
	// Before clamping, that commanded ~-80 Nm against the -30 Nm limit.
	float apps = -125.0f / (1965 - 470);
	CHECK_NEAR(torqueMap_target(apps, NO_BRAKE, MOVING), REGEN_MAX_TORQUE, 1e-4);
}

static void test_regen_never_exceeds_limit(void) {
	for (int i = -50; i <= 10; i++) {
		CHECK(torqueMap_target(i / 100.0f, NO_BRAKE, MOVING) >= REGEN_MAX_TORQUE);
	}
}
#endif

// -------------------------------------
// torqueMap_filterStep at 1 kHz (TIM6)
// -------------------------------------

// Runs n filter ticks toward target from a settled start, returns the output.
static float run_filter(float start, float target, int n) {
	float filtered = start, out = start;
	for (int i = 0; i < n; i++) {
		out = torqueMap_filterStep(&filtered, target, out, DT_S);
	}
	return out;
}

static void test_filter_holds_steady_state(void) {
	CHECK_NEAR(run_filter(40.0f, 40.0f, 1000), 40.0, 1e-4);
}

static void test_filter_step_matches_first_order_lpf(void) {
	// Closed form for the discrete LPF: y[n] = target * (1 - (1 - alpha)^n).
	double alpha = DT_S / (RC_TIME_CONSTANT + DT_S);
	int n_tau = (int)(RC_TIME_CONSTANT / DT_S); // 50 ticks
	CHECK_NEAR(run_filter(0.0f, 80.0f, n_tau), 80.0 * (1.0 - pow(1.0 - alpha, n_tau)), 0.01);
}

static void test_filter_never_overshoots_step(void) {
	float filtered = 0.0f, out = 0.0f, prev = 0.0f;
	for (int i = 0; i < 2000; i++) {
		out = torqueMap_filterStep(&filtered, 80.0f, out, DT_S);
		CHECK(out >= prev && out <= 80.0f);
		prev = out;
	}
}

static void test_slew_limit_caps_each_tick(void) {
	// Force the LPF state far from the last output so the slew limiter binds.
	float filtered = 100.0f;
	float out = torqueMap_filterStep(&filtered, 100.0f, 0.0f, DT_S);
	CHECK_NEAR(out, SLEW_RATE_LIMIT * DT_S, 1e-4); // 2 Nm per tick
}

static void test_slew_limit_inactive_for_normal_steps(void) {
	// At 1 kHz the LPF moves at most alpha * step per tick, so the 2000 Nm/s
	// slew limit only binds for steps above ~102 Nm. A full 0 -> 80 Nm step
	// is shaped entirely by the LPF.
	float filtered = 0.0f;
	float out = torqueMap_filterStep(&filtered, 80.0f, 0.0f, DT_S);
	CHECK_NEAR(out, filtered, 1e-6);
	CHECK(out < SLEW_RATE_LIMIT * DT_S);
}

static void test_raw_lpf_decays_slowly(void) {
	// The LPF on its own is symmetric: from 80 Nm it is still at ~11 Nm 100 ms
	// after the target drops to 0, and below 1 Nm only at ~222 ms.
	// torqueMap_request clamps this (EV.3.1.3), see test_request_lift_off_*.
	CHECK_NEAR(run_filter(80.0f, 0.0f, 100), 11.04, 0.05);
	CHECK(run_filter(80.0f, 0.0f, 221) > 1.0f);
	CHECK(run_filter(80.0f, 0.0f, 222) < 1.0f);
}

// ----------------------------------------
// torqueMap_request: cut + filter per tick
// ----------------------------------------

// Settle the filter at full pedal, returns the steady output.
static float settle_full_pedal(TorqueFilter *f) {
	float out = 0.0f;
	torqueMap_filterReset(f);
	for (int i = 0; i < 2000; i++) {
		out = torqueMap_request(f, false, 1.0f, NO_BRAKE, STOPPED, DT_S);
	}
	return out;
}

static void test_request_starts_from_zero(void) {
	// After a reset the first tick ramps from 0 Nm instead of jumping to target.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	double alpha = DT_S / (RC_TIME_CONSTANT + DT_S);
	CHECK_NEAR(torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S), MAX_TORQUE * alpha, 1e-4);
}

static void test_request_cut_is_instant(void) {
	// T.4.2.5 / T.4.3.3 / EV.4.7: 0 Nm on the same tick as the fault.
	TorqueFilter f;
	CHECK_NEAR(settle_full_pedal(&f), MAX_TORQUE, 0.01);
	CHECK_NEAR(torqueMap_request(&f, true, 1.0f, NO_BRAKE, STOPPED, DT_S), 0.0, 1e-6);
}

static void test_request_ramps_up_after_cut_clears(void) {
	// Once the fault clears, torque comes back through the filter from 0 Nm,
	// not as a step back to where it was before the cut.
	TorqueFilter f;
	settle_full_pedal(&f);
	torqueMap_request(&f, true, 1.0f, NO_BRAKE, STOPPED, DT_S);
	double alpha = DT_S / (RC_TIME_CONSTANT + DT_S);
	CHECK_NEAR(torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S), MAX_TORQUE * alpha, 1e-4);
}

// -----------------------------------------------
// EV.3.1.3: output never more than driver request
// -----------------------------------------------

static void test_request_lift_off_is_instant(void) {
	TorqueFilter f;
	settle_full_pedal(&f);
	CHECK_NEAR(torqueMap_request(&f, false, APPS_INFLECTION_PERCENT, NO_BRAKE, STOPPED, DT_S), 0.0, 1e-6);
}

static void test_request_partial_lift_follows_pedal(void) {
	TorqueFilter f;
	settle_full_pedal(&f);
	float apps = 0.6f;
	CHECK_NEAR(torqueMap_request(&f, false, apps, NO_BRAKE, STOPPED, DT_S),
			torqueMap_target(apps, NO_BRAKE, STOPPED), 1e-4);
}

static void test_request_repress_after_lift_ramps(void) {
	// After a lift-off the filter restarts from the lower torque, so pressing
	// again is smoothed instead of jumping back to the old filter state.
	TorqueFilter f;
	settle_full_pedal(&f);
	torqueMap_request(&f, false, APPS_INFLECTION_PERCENT, NO_BRAKE, STOPPED, DT_S);
	double alpha = DT_S / (RC_TIME_CONSTANT + DT_S);
	CHECK_NEAR(torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S), MAX_TORQUE * alpha, 1e-4);
}

#if REGEN_ENABLED
static void test_request_regen_engages_gradually(void) {
	// Regen is a request too: it builds through the filter, never past it.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	double alpha = DT_S / (RC_TIME_CONSTANT + DT_S);
	CHECK_NEAR(torqueMap_request(&f, false, 0.0f, NO_BRAKE, MOVING, DT_S), REGEN_MAX_TORQUE * alpha, 1e-4);
}

static void test_request_drive_to_regen_passes_through_zero(void) {
	TorqueFilter f;
	settle_full_pedal(&f);
	CHECK_NEAR(torqueMap_request(&f, false, 0.0f, NO_BRAKE, MOVING, DT_S), 0.0, 1e-6);
	float out = torqueMap_request(&f, false, 0.0f, NO_BRAKE, MOVING, DT_S);
	CHECK(out < 0.0f && out > REGEN_MAX_TORQUE);
}
#endif

static void test_request_never_exceeds_driver_request(void) {
	// Property check over a long pseudo-random drive: every tick, the output
	// has the same sign as the map's target and is no bigger than it.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	unsigned seed = 12345;
	float apps = 0.0f, speed = 0.0f;
	int drive_ticks = 0, regen_ticks = 0;
	int failed_before = checks_failed;
	for (int i = 0; i < 200000; i++) {
		seed = seed * 1103515245u + 12345u;
		int r = (int)((seed >> 16) % 1000);
		if (r < 5) apps = (r % 2) ? 1.0f : 0.0f; // slam or lift
		else apps += (r - 500) / 20000.0f;
		if (apps < -0.1f) apps = -0.1f;
		if (apps > 1.1f) apps = 1.1f;
		speed += ((int)(seed >> 8) % 3 - 1) * 0.05f; // random walk through the 5 km/h regen cutoff
		if (speed < 0.0f) speed = 0.0f;
		if (speed > 30.0f) speed = 30.0f;

		float target = torqueMap_target(apps, NO_BRAKE, speed);
		float out = torqueMap_request(&f, false, apps, NO_BRAKE, speed, DT_S);
		CHECK(f.target == target);
		if (target >= 0.0f) CHECK(out >= 0.0f && out <= target + 1e-5f);
		else CHECK(out <= 0.0f && out >= target - 1e-5f);
		if (checks_failed != failed_before) return; // one failure is enough, don't flood the output
		drive_ticks += out > 1.0f;
		regen_ticks += out < -1.0f;
	}
	// Make sure the drive actually exercised both directions (or, with regen
	// off, that it never regenerated).
	CHECK(drive_ticks > 10000);
	CHECK(REGEN_ENABLED ? regen_ticks > 10000 : regen_ticks == 0);
}

static void test_request_cut_blocks_regen(void) {
	// A cut targets 0 Nm, so it also stops regen.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	CHECK_NEAR(torqueMap_request(&f, true, 0.0f, NO_BRAKE, MOVING, DT_S), 0.0, 1e-6);
}

static void test_request_records_driver_request(void) {
	// f.target is what 0x502 reports as the driver request: the map target,
	// before the filter, and still set on a cut tick so the log shows what
	// the driver wanted while torque was cut.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S);
	CHECK_NEAR(f.target, MAX_TORQUE, 1e-4);
	CHECK(f.output < f.target); // still ramping, so they differ
	torqueMap_request(&f, true, 0.6f, NO_BRAKE, STOPPED, DT_S);
	CHECK_NEAR(f.target, torqueMap_target(0.6f, NO_BRAKE, STOPPED), 1e-6);
	CHECK_NEAR(f.output, 0.0, 1e-6);
}

static void test_filter_reset_clears_state(void) {
	TorqueFilter f;
	torqueMap_filterReset(&f);
	torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S);
	torqueMap_filterReset(&f);
	CHECK_NEAR(f.output, 0.0, 1e-6);
	CHECK_NEAR(f.filtered, 0.0, 1e-6);
	CHECK_NEAR(f.target, 0.0, 1e-6);
}

int main(void) {
	RUN(test_zero_torque_at_inflection);
	RUN(test_full_pedal_gives_max_torque);
	RUN(test_overtravel_clamped_to_max_torque);
	RUN(test_half_travel_gives_half_torque);
	RUN(test_shipped_table_is_valid);
	RUN(test_map_hits_table_points);
	RUN(test_map_interpolates_between_points);
	RUN(test_table_validator_rejects_bad_maps);
	RUN(test_positive_torque_is_monotonic);

#if !REGEN_ENABLED
	RUN(test_regen_disabled_coasts);
#endif
	RUN(test_no_regen_below_min_carspeed);
	RUN(test_regen_cutoff_meets_ev_3_3_3);
	RUN(test_no_regen_with_brake_pressed);
#if REGEN_ENABLED
	RUN(test_regen_allowed_at_min_carspeed);
	RUN(test_regen_scales_linearly_below_inflection);
	RUN(test_negative_pedal_regen_clamped);
	RUN(test_regen_never_exceeds_limit);
#endif

	RUN(test_filter_holds_steady_state);
	RUN(test_filter_step_matches_first_order_lpf);
	RUN(test_filter_never_overshoots_step);
	RUN(test_slew_limit_caps_each_tick);
	RUN(test_slew_limit_inactive_for_normal_steps);
	RUN(test_raw_lpf_decays_slowly);

	RUN(test_request_starts_from_zero);
	RUN(test_request_cut_is_instant);
	RUN(test_request_ramps_up_after_cut_clears);
	RUN(test_request_lift_off_is_instant);
	RUN(test_request_partial_lift_follows_pedal);
	RUN(test_request_repress_after_lift_ramps);
#if REGEN_ENABLED
	RUN(test_request_regen_engages_gradually);
	RUN(test_request_drive_to_regen_passes_through_zero);
#endif
	RUN(test_request_never_exceeds_driver_request);
	RUN(test_request_cut_blocks_regen);
	RUN(test_request_records_driver_request);
	RUN(test_filter_reset_clears_state);

	return CHECK_SUMMARY();
}
