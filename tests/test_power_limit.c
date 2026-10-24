/*
 * test_power_limit.c
 *
 * Host tests for Core/Src/power_limit.c and torqueMap_requestLimited().
 *
 * Built twice by tests/Makefile: with the shipped MAX_TORQUE (80 Nm, where
 * the cap never binds below MOTOR_MAX_RPM) and with -DMAX_TORQUE=230 (the
 * inverter's own torque limit), where it does. The sweep tests hold in both.
 *
 * Run from the repo root:  make -C tests
 */

#include "check.h"
#include "power_limit.h"
#include "torque_map.h"

#define DT_S 0.001f
#define RPM_TO_RAD_S (2.0f * 3.14159265f / 60.0f)
#define NO_BRAKE 0.0f
#define FAST 30.0f

// DC power the model says this torque takes at this speed.
static double dc_power_w(double torque_nm, double rpm) {
	return torque_nm * fabs(rpm) * RPM_TO_RAD_S / DRIVETRAIN_EFFICIENCY;
}

static void test_full_torque_at_low_speed(void) {
	CHECK_NEAR(powerLimit_capNm(0.0f), MAX_TORQUE, 1e-6);
	CHECK_NEAR(powerLimit_capNm(500.0f), MAX_TORQUE, 1e-6);
}

static void test_cap_holds_power_at_every_speed(void) {
	// Grid over every speed the motor can reach and well past it: the cap
	// never asks for more than POWER_LIMIT_W, and never more than MAX_TORQUE.
	for (int rpm = 0; rpm <= 12000; rpm += 5) {
		float cap = powerLimit_capNm((float)rpm);
		CHECK(cap <= MAX_TORQUE);
		CHECK(cap > 0.0f);
		CHECK(dc_power_w(cap, rpm) <= POWER_LIMIT_W * 1.0001);
	}
}

static void test_cap_only_binds_at_the_limit(void) {
	// Below the speed where MAX_TORQUE reaches POWER_LIMIT_W, full torque;
	// above it, exactly POWER_LIMIT_W (not a lower, overcautious cap).
	double knee_rpm = POWER_LIMIT_W * DRIVETRAIN_EFFICIENCY / (MAX_TORQUE * RPM_TO_RAD_S);
	for (int rpm = 0; rpm <= 12000; rpm += 5) {
		float cap = powerLimit_capNm((float)rpm);
		if (rpm < knee_rpm - 1) CHECK_NEAR(cap, MAX_TORQUE, 1e-4);
		if (rpm > knee_rpm + 1) CHECK_NEAR(dc_power_w(cap, rpm), POWER_LIMIT_W, 1.0);
	}
}

static void test_cap_never_rises_with_speed(void) {
	float prev = powerLimit_capNm(0.0f);
	for (int rpm = 1; rpm <= 12000; rpm++) {
		float cap = powerLimit_capNm((float)rpm);
		CHECK(cap <= prev);
		prev = cap;
	}
}

static void test_cap_same_either_direction(void) {
	for (int rpm = 0; rpm <= 12000; rpm += 250) {
		CHECK_NEAR(powerLimit_capNm((float)-rpm), powerLimit_capNm((float)rpm), 1e-6);
	}
}

static void test_margin_under_rule(void) {
	// Even if the real drivetrain is 100% efficient, the cap is under 80 kW
	// at the motor shaft... (the real one loses more, so less from the pack).
	CHECK(POWER_LIMIT_W < POWER_RULE_W);
	CHECK(DRIVETRAIN_EFFICIENCY <= 1.0f);
	for (int rpm = 0; rpm <= 12000; rpm += 5) {
		double mech_w = powerLimit_capNm((float)rpm) * rpm * RPM_TO_RAD_S;
		CHECK(mech_w <= POWER_RULE_W);
	}
}

static void test_shipped_torque_never_capped(void) {
#if MAX_TORQUE == 80
	// Documents why the cap changes nothing today: at 80 Nm the knee is past
	// the inverter's Max_Speed. If this fails, MAX_TORQUE or the limits moved
	// and the cap is now live: bench check it (docs/bench-plan.md).
	CHECK_NEAR(powerLimit_capNm(MOTOR_MAX_RPM), MAX_TORQUE, 1e-4);
#else
	CHECK(powerLimit_capNm(MOTOR_MAX_RPM) < MAX_TORQUE);
#endif
}

// ---------------------------
// powerLimit_capRpm: staleness
// ---------------------------

static void test_fresh_speed_used_as_is(void) {
	CHECK_NEAR(powerLimit_capRpm(3000.0f, 1000, 1000), 3000.0f, 1e-6);
	CHECK_NEAR(powerLimit_capRpm(-3000.0f, 1050, 1000), -3000.0f, 1e-6);
	CHECK_NEAR(powerLimit_capRpm(3000.0f, 1000 + CARSPEED_STALE_MS, 1000), 3000.0f, 1e-6);
}

static void test_stale_speed_assumes_max_rpm(void) {
	CHECK_NEAR(powerLimit_capRpm(100.0f, 1000 + CARSPEED_STALE_MS + 1, 1000), MOTOR_MAX_RPM, 1e-6);
	CHECK_NEAR(powerLimit_capRpm(100.0f, 1000000, 0), MOTOR_MAX_RPM, 1e-6);
}

static void test_stale_check_across_tick_wrap(void) {
	CHECK_NEAR(powerLimit_capRpm(100.0f, 20, 0xFFFFFFFFu - 20), 100.0f, 1e-6);
	CHECK_NEAR(powerLimit_capRpm(100.0f, 200, 0xFFFFFFFFu - 20), MOTOR_MAX_RPM, 1e-6);
}

// -----------------------------
// torqueMap_requestLimited
// -----------------------------

static void test_unlimited_matches_request(void) {
	// A cap of MAX_TORQUE is torqueMap_request exactly, tick for tick.
	TorqueFilter a, b;
	torqueMap_filterReset(&a);
	torqueMap_filterReset(&b);
	for (int i = 0; i < 2000; i++) {
		float apps = (i % 700) / 700.0f;
		bool cut = (i % 450) == 0;
		float x = torqueMap_request(&a, cut, apps, NO_BRAKE, FAST, DT_S);
		float y = torqueMap_requestLimited(&b, cut, apps, NO_BRAKE, FAST, MAX_TORQUE, DT_S);
		CHECK(x == y);
	}
}

static void test_output_never_above_cap(void) {
	TorqueFilter f;
	torqueMap_filterReset(&f);
	for (int i = 0; i < 3000; i++) {
		float cap = 20.0f + 15.0f * (float)((i / 300) % 3); // steps 20, 35, 50 Nm
		float out = torqueMap_requestLimited(&f, false, 1.0f, NO_BRAKE, FAST, cap, DT_S);
		CHECK(out <= cap + 1e-4f);
		CHECK(out >= 0.0f);
	}
	CHECK_NEAR(f.target, MAX_TORQUE, 1e-4); // the driver's request, uncapped
}

static void test_cap_drop_follows_at_once(void) {
	// Speed rising lowers the cap: the command follows it down that tick,
	// like lifting off (EV.3.1.3 clamp), with no filter lag.
	TorqueFilter f;
	torqueMap_filterReset(&f);
	for (int i = 0; i < 1000; i++) torqueMap_requestLimited(&f, false, 1.0f, NO_BRAKE, FAST, 60.0f, DT_S);
	CHECK_NEAR(torqueMap_requestLimited(&f, false, 1.0f, NO_BRAKE, FAST, 40.0f, DT_S), 40.0, 1e-4);
}

static void test_cap_does_not_touch_regen_or_cut(void) {
	TorqueFilter f;
	torqueMap_filterReset(&f);
	CHECK_NEAR(torqueMap_requestLimited(&f, true, 1.0f, NO_BRAKE, FAST, 10.0f, DT_S), 0.0, 1e-6);
	CHECK_NEAR(torqueMap_requestLimited(&f, false, 1.0f, NO_BRAKE, FAST, -5.0f, DT_S), 0.0, 1e-6);
	TorqueFilter a, b;
	torqueMap_filterReset(&a);
	torqueMap_filterReset(&b);
	for (int i = 0; i < 1000; i++) {
		float x = torqueMap_request(&a, false, 0.0f, NO_BRAKE, FAST, DT_S);
		float y = torqueMap_requestLimited(&b, false, 0.0f, NO_BRAKE, FAST, 0.0f, DT_S);
		CHECK(x == y); // regen (or coast with regen off) is the same with a 0 Nm cap
	}
}

static void test_full_pipeline_power_sweep(void) {
	// Every pedal position at every speed, through the real cap and the real
	// filter: the settled command never takes more than POWER_LIMIT_W.
	for (int rpm = 0; rpm <= 12000; rpm += 100) {
		float cap = powerLimit_capNm(powerLimit_capRpm((float)rpm, 0, 0));
		for (int p = 0; p <= 100; p += 5) {
			TorqueFilter f;
			torqueMap_filterReset(&f);
			float out = 0.0f;
			for (int i = 0; i < 500; i++) {
				out = torqueMap_requestLimited(&f, false, p / 100.0f, NO_BRAKE, FAST, cap, DT_S);
				CHECK(dc_power_w(out, rpm) <= POWER_LIMIT_W * 1.0001);
			}
		}
	}
}

int main(void) {
	printf("MAX_TORQUE %d Nm, cap knee at %.0f rpm\n", MAX_TORQUE,
			POWER_LIMIT_W * DRIVETRAIN_EFFICIENCY / (MAX_TORQUE * RPM_TO_RAD_S));
	RUN(test_full_torque_at_low_speed);
	RUN(test_cap_holds_power_at_every_speed);
	RUN(test_cap_only_binds_at_the_limit);
	RUN(test_cap_never_rises_with_speed);
	RUN(test_cap_same_either_direction);
	RUN(test_margin_under_rule);
	RUN(test_shipped_torque_never_capped);

	RUN(test_fresh_speed_used_as_is);
	RUN(test_stale_speed_assumes_max_rpm);
	RUN(test_stale_check_across_tick_wrap);

	RUN(test_unlimited_matches_request);
	RUN(test_output_never_above_cap);
	RUN(test_cap_drop_follows_at_once);
	RUN(test_cap_does_not_touch_regen_or_cut);
	RUN(test_full_pipeline_power_sweep);
	return CHECK_SUMMARY();
}
