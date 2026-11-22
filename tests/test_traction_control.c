// Host tests for the Simulink TC model wrapper (Block_Diagram_1 v2.0).
// Built twice by tests/Makefile: TC_ENABLED=0 (shadow mode) and TC_ENABLED=1 (active).
#include <math.h>
#include "check.h"
#include "traction_control.h"

static void test_shadow_and_passthrough(void) {
	tractionControl_init();

	// Rolling at 10 m/s (50 rad/s on 0.2 m effective radius), zero slip.
	const float demands[] = { 0.0f, 1.0f, 40.0f, 80.0f, 150.0f, -20.0f };
	for (unsigned i = 0; i < sizeof demands / sizeof demands[0]; i++) {
		float out = tractionControl_applyWheels(demands[i], 50.0f, 50.0f, 50.0f, 50.0f);
		CHECK_NEAR(out, demands[i], 1e-4f);
	}
	CHECK_NEAR(tractionControl_slipRatio(), 0.0f, 1e-4f);
	CHECK_NEAR(tractionControl_slipSpeedMps(), 0.0f, 1e-4f);

	// Compatibility 1-arg wrapper steps with 0 wheel speeds and never produces NaN.
	float out0 = tractionControl_apply(80.0f);
	CHECK(out0 == out0);
	CHECK_NEAR(out0, 80.0f, 1e-4f);
}

static void test_slip_ratio_regime(void) {
	tractionControl_init();

	// 10 m/s front (50 rad/s), 12 m/s rear (60 rad/s) => SR = 0.20 (above 0.136 target).
	// u = (0.136 - 0.20) * -5 = 0.32 => model torque = (1 - 0.32) * 80 = 54.4 Nm.
	float out = tractionControl_applyWheels(80.0f, 50.0f, 50.0f, 60.0f, 60.0f);
	CHECK_NEAR(tractionControl_slipRatio(), 0.20f, 1e-4f);
	CHECK_NEAR(tractionControl_modelTorqueNm(), 54.4f, 1e-3f);
#if TC_ENABLED
	CHECK_NEAR(out, 54.4f, 1e-3f);
#else
	CHECK_NEAR(out, 80.0f, 1e-4f);
#endif

	// Heavy wheelspin: 14 m/s rear (70 rad/s) => SR = 0.40 => u clamped to 1.0 => 0 Nm.
	float out_heavy = tractionControl_applyWheels(80.0f, 50.0f, 50.0f, 70.0f, 70.0f);
	CHECK_NEAR(tractionControl_modelTorqueNm(), 0.0f, 1e-4f);
#if TC_ENABLED
	CHECK_NEAR(out_heavy, 0.0f, 1e-4f);
#else
	CHECK_NEAR(out_heavy, 80.0f, 1e-4f);
#endif
}

static void test_standing_launch_slip_speed(void) {
	tractionControl_init();

	// Standstill (v_front = 0): alpha = 0 -> pure slip-speed controller (target 1.0 m/s).
	// No divide-by-zero / NaN even with 0 front wheel speed.
	// Rear at 3 rad/s (0.6 m/s < 1.0 m/s) => no cut.
	float out_ok = tractionControl_applyWheels(80.0f, 0.0f, 0.0f, 3.0f, 3.0f);
	CHECK(isfinite(tractionControl_slipRatio()));
	CHECK_NEAR(tractionControl_slipSpeedMps(), 0.6f, 1e-4f);
	CHECK_NEAR(tractionControl_modelTorqueNm(), 80.0f, 1e-3f);
	CHECK_NEAR(out_ok, 80.0f, 1e-3f);

	// Rear at 8 rad/s (1.6 m/s) => slip speed excess = 0.6 m/s => u = 0.6 => 32.0 Nm.
	float out_spin = tractionControl_applyWheels(80.0f, 0.0f, 0.0f, 8.0f, 8.0f);
	CHECK_NEAR(tractionControl_slipSpeedMps(), 1.6f, 1e-4f);
	CHECK_NEAR(tractionControl_modelTorqueNm(), 32.0f, 1e-3f);
#if TC_ENABLED
	CHECK_NEAR(out_spin, 32.0f, 1e-3f);
#else
	CHECK_NEAR(out_spin, 80.0f, 1e-4f);
#endif
}

static void test_regen_never_altered(void) {
	tractionControl_init();

	// Negative (regen) torque passes through untouched even with wheel speed mismatch.
	float out = tractionControl_applyWheels(-15.0f, 50.0f, 50.0f, 80.0f, 80.0f);
	CHECK_NEAR(out, -15.0f, 1e-4f);
}

static void test_ev313_never_exceeds_driver(void) {
	tractionControl_init();

	for (int d = 0; d <= 150; d += 15) {
		for (int vf = 0; vf <= 100; vf += 20) {
			for (int vr = 0; vr <= 140; vr += 20) {
				float out = tractionControl_applyWheels((float)d, (float)vf, (float)vf,
						(float)vr, (float)vr);
				CHECK(isfinite(out));
				CHECK(out >= 0.0f && out <= (float)d + 1e-5f);
			}
		}
	}
}

int main(void) {
	RUN(test_shadow_and_passthrough);
	RUN(test_slip_ratio_regime);
	RUN(test_standing_launch_slip_speed);
	RUN(test_regen_never_altered);
	RUN(test_ev313_never_exceeds_driver);
	return CHECK_SUMMARY();
}
