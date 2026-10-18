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

static void test_half_travel_follows_curve(void) {
	// Halfway between inflection and full pedal: 80 * 0.5^1.8 = 22.97 Nm.
	// CURRENT BEHAVIOR: Phase 4 replaces the 1.8 curve with a lookup table.
	float apps = APPS_INFLECTION_PERCENT + 0.5f * (1.0f - APPS_INFLECTION_PERCENT);
	CHECK_NEAR(torqueMap_target(apps, NO_BRAKE, STOPPED), 22.974, 0.01);
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

static void test_no_regen_below_min_carspeed(void) {
	CHECK_NEAR(torqueMap_target(0.0f, NO_BRAKE, REGEN_MIN_CARSPEED - 0.1f), 0.0, 1e-4);
}

static void test_regen_allowed_at_min_carspeed(void) {
	CHECK(torqueMap_target(0.0f, NO_BRAKE, REGEN_MIN_CARSPEED) < 0.0f);
}

static void test_no_regen_with_brake_pressed(void) {
	CHECK_NEAR(torqueMap_target(0.0f, REGEN_BRAKE_CANCEL_PERCENT + 0.01f, MOVING), 0.0, 1e-4);
}

static void test_regen_scales_linearly_below_inflection(void) {
	CHECK_NEAR(torqueMap_target(0.0f, NO_BRAKE, MOVING), REGEN_MAX_TORQUE, 1e-4);
	CHECK_NEAR(torqueMap_target(APPS_INFLECTION_PERCENT / 2, NO_BRAKE, MOVING),
			REGEN_MAX_TORQUE / 2.0, 1e-4);
}

static void test_regen_exceeds_max_with_negative_pedal(void) {
	// CURRENT BEHAVIOR (bug, Phase 2 clamps it): pedal % goes negative when the
	// ADC reads above APPS2_ADC_MIN_VAL but still inside the 125-count overshoot
	// buffer, so validateAPPS() doesn't flag it. Worst case for APPS2 is
	// -125/1495 = -8.4%, which commands about -80 Nm against a -30 Nm limit.
	float apps = -125.0f / (1965 - 470);
	CHECK_NEAR(torqueMap_target(apps, NO_BRAKE, MOVING), -80.2, 0.1);
}

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

static void test_fault_cut_is_slow(void) {
	// CURRENT BEHAVIOR (Phase 2 makes the cut instant): a fault sets the target
	// to 0, but the request still goes through the filter. From 80 Nm the car
	// is still commanded ~11 Nm 100 ms later and only drops below 1 Nm at ~222 ms.
	CHECK_NEAR(run_filter(80.0f, 0.0f, 100), 11.04, 0.05);
	CHECK(run_filter(80.0f, 0.0f, 221) > 1.0f);
	CHECK(run_filter(80.0f, 0.0f, 222) < 1.0f);
}

// ----------------------------------------
// torqueMap_request: cut + filter per tick
// ----------------------------------------

static void test_request_first_tick_jumps_to_target(void) {
	// CURRENT BEHAVIOR (Phase 2 reviews this): the first tick after a reset
	// starts the filter at the target, so there is no ramp on that tick.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	CHECK_NEAR(torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S), MAX_TORQUE, 1e-4);
}

static void test_request_cut_ramps_down_through_filter(void) {
	// CURRENT BEHAVIOR (Phase 2 makes the cut instant): same slow decay as
	// test_fault_cut_is_slow, through the path the firmware actually calls.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S);
	float out = 0.0f;
	for (int i = 0; i < 100; i++) {
		out = torqueMap_request(&f, true, 1.0f, NO_BRAKE, STOPPED, DT_S);
	}
	CHECK_NEAR(out, 11.04, 0.05);
}

static void test_request_cut_blocks_regen(void) {
	// A cut targets 0 Nm, so it also stops regen.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	CHECK_NEAR(torqueMap_request(&f, true, 0.0f, NO_BRAKE, MOVING, DT_S), 0.0, 1e-6);
}

static void test_filter_reset_clears_state(void) {
	TorqueFilter f;
	torqueMap_filterReset(&f);
	torqueMap_request(&f, false, 1.0f, NO_BRAKE, STOPPED, DT_S);
	torqueMap_filterReset(&f);
	CHECK(!f.initialized);
	CHECK_NEAR(f.output, 0.0, 1e-6);
	CHECK_NEAR(f.filtered, 0.0, 1e-6);
}

int main(void) {
	RUN(test_zero_torque_at_inflection);
	RUN(test_full_pedal_gives_max_torque);
	RUN(test_overtravel_clamped_to_max_torque);
	RUN(test_half_travel_follows_curve);
	RUN(test_positive_torque_is_monotonic);

	RUN(test_no_regen_below_min_carspeed);
	RUN(test_regen_allowed_at_min_carspeed);
	RUN(test_no_regen_with_brake_pressed);
	RUN(test_regen_scales_linearly_below_inflection);
	RUN(test_regen_exceeds_max_with_negative_pedal);

	RUN(test_filter_holds_steady_state);
	RUN(test_filter_step_matches_first_order_lpf);
	RUN(test_filter_never_overshoots_step);
	RUN(test_slew_limit_caps_each_tick);
	RUN(test_slew_limit_inactive_for_normal_steps);
	RUN(test_fault_cut_is_slow);

	RUN(test_request_first_tick_jumps_to_target);
	RUN(test_request_cut_ramps_down_through_filter);
	RUN(test_request_cut_blocks_regen);
	RUN(test_filter_reset_clears_state);

	return CHECK_SUMMARY();
}
