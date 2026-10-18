/*
 * test_pedal_checks.c
 *
 * Host tests for Core/Src/pedal_checks.c (the exact file the car runs).
 *
 * Characterization tests, same rules as test_torque_map.c: they pin what the
 * checks do TODAY. Known gaps are marked CURRENT BEHAVIOR; when Phase 2 fixes
 * one, its test changes in the same commit.
 *
 * Run from the repo root:  make -C tests
 */

#include "check.h"
#include "pedal_checks.h"

#define BRAKE_OFF BSE_ADC_MIN_VAL
#define BRAKE_ON (BSE_ACTIVATED_ADC_THRESHOLD + 1)
#define UNPLUGGED 70 // raw ADC with a sensor disconnected
#define SHORTED 4095 // raw ADC with the signal shorted to 3.3 V

static PlausibilityChecks pc;
static uint32_t now;

// Raw ADC counts for a given APPS travel (0..1). APPS read lower when pressed.
static uint16_t apps1_adc(float travel) {
	return (uint16_t)(APPS1_ADC_MIN_VAL - travel * (APPS1_ADC_MIN_VAL - APPS1_ADC_MAX_VAL) + 0.5f);
}
static uint16_t apps2_adc(float travel) {
	return (uint16_t)(APPS2_ADC_MIN_VAL - travel * (APPS2_ADC_MIN_VAL - APPS2_ADC_MAX_VAL) + 0.5f);
}

// One 1 ms control loop tick with raw ADC inputs. Returns true if torque is cut.
static bool tick_raw(uint16_t a1, uint16_t a2, uint16_t bse) {
	uint16_t adc[3] = { a1, a2, bse };
	float percents[3];
	now++;
	pedalChecks_percents(adc, percents);
	pedalChecks_update(&pc, adc, percents, now);
	return pedalChecks_torqueCut(&pc, now);
}

static bool tick(float apps1, float apps2, uint16_t bse) {
	return tick_raw(apps1_adc(apps1), apps2_adc(apps2), bse);
}

// Fresh state, then release the pedal once so the startup cut clears.
static void start_driving(void) {
	now = 10000;
	pedalChecks_reset(&pc);
	tick(0.0f, 0.0f, BRAKE_OFF);
}

// --------------
// Pedal scaling
// --------------

static void test_percents_span_calibration(void) {
	uint16_t released[3] = { APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, BSE_ADC_MIN_VAL };
	uint16_t full[3] = { APPS1_ADC_MAX_VAL, APPS2_ADC_MAX_VAL, BSE_ADC_MAX_VAL };
	float p[3];
	pedalChecks_percents(released, p);
	for (int i = 0; i < 3; i++) CHECK_NEAR(p[i], 0.0, 1e-6);
	pedalChecks_percents(full, p);
	for (int i = 0; i < 3; i++) CHECK_NEAR(p[i], 1.0, 1e-6);
}

static void test_percents_not_clamped(void) {
	// Deliberate: the APPS1 vs APPS2 check compares raw readings, so overshoot
	// readings stay below 0. torqueMap_target clamps before using them.
	uint16_t adc[3] = { APPS1_ADC_MIN_VAL + 100, APPS2_ADC_MIN_VAL + 100, BSE_ADC_MIN_VAL };
	float p[3];
	pedalChecks_percents(adc, p);
	CHECK(p[PEDAL_APPS1] < 0.0f);
	CHECK(p[PEDAL_APPS2] < 0.0f);
}

static void test_apps_percent_follows_pedal_mode(void) {
	float p[3] = { 0.2f, 0.4f, 0.0f };
#if PEDAL_MODE == ONLY_APPS2
	CHECK_NEAR(pedalChecks_appsPercent(p), 0.4, 1e-6);
#elif PEDAL_MODE == ONLY_APPS1
	CHECK_NEAR(pedalChecks_appsPercent(p), 0.2, 1e-6);
#else
	CHECK_NEAR(pedalChecks_appsPercent(p), 0.3, 1e-6);
#endif
}

// ------------------------------------
// APPS range check (open/short circuit)
// ------------------------------------

static void test_normal_driving_no_cut(void) {
	start_driving();
	for (int i = 0; i <= 100; i++) {
		CHECK(!tick(i / 100.0f, i / 100.0f, BRAKE_OFF));
	}
}

static void test_overshoot_buffer_boundaries(void) {
	start_driving();
	CHECK(!tick_raw(APPS1_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER, apps2_adc(0), BRAKE_OFF));
	CHECK(tick_raw(APPS1_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER + 1, apps2_adc(0), BRAKE_OFF));
	CHECK(pc.apps1_invalid);

	start_driving();
	CHECK(!tick_raw(apps1_adc(1), APPS2_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER, BRAKE_OFF));
	CHECK(tick_raw(apps1_adc(1), APPS2_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER - 1, BRAKE_OFF));
	CHECK(pc.apps2_invalid);
}

static void test_unplugged_apps_cuts_immediately(void) {
	start_driving();
	CHECK(tick_raw(UNPLUGGED, apps2_adc(0.3f), BRAKE_OFF));
	CHECK(pc.apps1_invalid);
	start_driving();
	CHECK(tick_raw(apps1_adc(0.3f), UNPLUGGED, BRAKE_OFF));
	CHECK(pc.apps2_invalid);
}

static void test_apps_range_glitch_recovers(void) {
	// Out of range for up to SENSOR_FAULT_LATCH_MS: cut while it lasts, then
	// torque comes back, so ADC noise can't strand the car.
	start_driving();
	for (int ms = 0; ms <= SENSOR_FAULT_LATCH_MS; ms++) {
		CHECK(tick_raw(UNPLUGGED, apps2_adc(0.5f), BRAKE_OFF));
	}
	CHECK(!pc.sensor_fault_latched);
	CHECK(!tick(0.5f, 0.5f, BRAKE_OFF));
}

static void test_apps_range_fault_latches_after_100ms(void) {
	// T.4.2.9: a real wiring fault stays cut even if the connector flickers
	// back, until the VCU leaves drive.
	start_driving();
	for (int ms = 0; ms <= SENSOR_FAULT_LATCH_MS + 1; ms++) {
		tick_raw(apps1_adc(0.5f), UNPLUGGED, BRAKE_OFF);
	}
	CHECK(pc.sensor_fault_latched);
	for (int ms = 0; ms < 1000; ms++) {
		CHECK(tick(0.0f, 0.0f, BRAKE_OFF));
		CHECK(tick(0.5f, 0.5f, BRAKE_OFF));
	}
	pedalChecks_reset(&pc);
	CHECK(!pc.sensor_fault_latched);
	CHECK(!tick(0.0f, 0.0f, BRAKE_OFF));
}

static void test_range_fault_timer_restarts(void) {
	// Two separate short glitches don't add up to a latch.
	start_driving();
	for (int ms = 0; ms < 80; ms++) tick_raw(UNPLUGGED, apps2_adc(0.0f), BRAKE_OFF);
	tick(0.0f, 0.0f, BRAKE_OFF);
	for (int ms = 0; ms < 80; ms++) tick(0.0f, 0.0f, UNPLUGGED);
	CHECK(!pc.sensor_fault_latched);
	CHECK(!tick(0.0f, 0.0f, BRAKE_OFF));
}

// -----------------------------------------
// APPS1 vs APPS2 agreement (T.4.2.4 / .5)
// -----------------------------------------

static void test_disagreement_cuts_after_100ms(void) {
	start_driving();
	// 20% apart: implausible, but allowed for APPS_IMPLAUSIBILITY_TIMEOUT_MS.
	for (int ms = 0; ms <= APPS_IMPLAUSIBILITY_TIMEOUT_MS; ms++) {
		CHECK(!tick(0.3f, 0.5f, BRAKE_OFF));
	}
	CHECK(tick(0.3f, 0.5f, BRAKE_OFF)); // 101 ms
	CHECK(!pc.apps_plausible);
}

static void test_agreement_restores_immediately(void) {
	start_driving();
	for (int ms = 0; ms < 200; ms++) tick(0.3f, 0.5f, BRAKE_OFF);
	CHECK(!tick(0.4f, 0.4f, BRAKE_OFF));
	CHECK(pc.apps_plausible);
}

static void test_short_disagreement_restarts_timer(void) {
	start_driving();
	for (int ms = 0; ms < 80; ms++) CHECK(!tick(0.3f, 0.5f, BRAKE_OFF));
	tick(0.4f, 0.4f, BRAKE_OFF);
	// A new disagreement gets its own full 100 ms, not the 20 ms left over.
	for (int ms = 0; ms <= APPS_IMPLAUSIBILITY_TIMEOUT_MS; ms++) {
		CHECK(!tick(0.3f, 0.5f, BRAKE_OFF));
	}
	CHECK(tick(0.3f, 0.5f, BRAKE_OFF));
}

// ---------------------------------
// APPS / brake crosscheck (EV.4.7)
// ---------------------------------

static void test_brake_with_throttle_cuts_immediately(void) {
	start_driving();
	CHECK(!tick(0.30f, 0.30f, BRAKE_OFF));
	CHECK(tick(0.30f, 0.30f, BRAKE_ON));
	CHECK(!pc.crosscheck_plausible);
}

static void test_crosscheck_thresholds(void) {
	start_driving();
	CHECK(!tick(0.30f, 0.30f, BSE_ACTIVATED_ADC_THRESHOLD)); // brake not over threshold
	CHECK(!tick(0.20f, 0.20f, BRAKE_ON)); // throttle under 25%
}

static void test_crosscheck_holds_until_apps_below_5_percent(void) {
	start_driving();
	tick(0.30f, 0.30f, BRAKE_ON);
	CHECK(tick(0.30f, 0.30f, BRAKE_OFF)); // releasing the brake is not enough
	CHECK(tick(0.10f, 0.10f, BRAKE_OFF));
	CHECK(tick(0.06f, 0.06f, BRAKE_OFF));
	CHECK(!tick(0.04f, 0.04f, BRAKE_OFF));
	CHECK(!tick(0.30f, 0.30f, BRAKE_OFF)); // throttle back on works
}

// --------------
// Reset / startup
// --------------

static void test_reset_requires_pedal_release(void) {
	// Entering drive with the throttle held gives no torque until it's released.
	now = 10000;
	pedalChecks_reset(&pc);
	CHECK(tick(0.5f, 0.5f, BRAKE_OFF));
	CHECK(tick(0.5f, 0.5f, BRAKE_OFF));
	CHECK(!tick(0.0f, 0.0f, BRAKE_OFF));
	CHECK(!tick(0.5f, 0.5f, BRAKE_OFF));
}

static void test_reset_disagreement_cuts_without_grace(void) {
	now = 10000;
	pedalChecks_reset(&pc);
	// APPS2 (the torque sensor) at 0% clears the startup crosscheck cut, so
	// the cut here comes from the APPS disagreement alone.
	CHECK(tick(0.2f, 0.0f, BRAKE_OFF));
	CHECK(pc.crosscheck_plausible);
	CHECK(!pc.apps_plausible);
}

// ---------------------
// BSE (brake sensor)
// ---------------------

static void test_bse_range_boundaries(void) {
	start_driving();
	CHECK(!tick(0.0f, 0.0f, BSE_ADC_OPEN_CIRCUIT_VAL));
	CHECK(tick(0.0f, 0.0f, BSE_ADC_OPEN_CIRCUIT_VAL - 1));
	CHECK(pc.bse_invalid);
	start_driving();
	CHECK(!tick(0.0f, 0.0f, BSE_ADC_SHORT_CIRCUIT_VAL));
	CHECK(tick(0.0f, 0.0f, BSE_ADC_SHORT_CIRCUIT_VAL + 1));
	CHECK(pc.bse_invalid);
}

static void test_hard_braking_is_not_a_fault(void) {
	start_driving();
	for (int ms = 0; ms < 1000; ms++) {
		CHECK(!tick(0.0f, 0.0f, BSE_ADC_MAX_VAL));
	}
	CHECK(!pc.bse_invalid);
	CHECK(!pc.sensor_fault_latched);
}

static void test_bse_open_circuit_cuts_and_latches(void) {
	// T.4.3.3 / T.4.3.4: unplugged BSE cuts torque, latched after 100 ms.
	start_driving();
	CHECK(tick(0.5f, 0.5f, UNPLUGGED));
	for (int ms = 0; ms <= SENSOR_FAULT_LATCH_MS; ms++) tick(0.5f, 0.5f, UNPLUGGED);
	CHECK(pc.sensor_fault_latched);
	CHECK(tick(0.0f, 0.0f, BRAKE_OFF)); // plugging it back in doesn't clear it
}

static void test_bse_short_circuit_cuts_and_latches(void) {
	start_driving();
	CHECK(tick(0.0f, 0.0f, SHORTED));
	for (int ms = 0; ms <= SENSOR_FAULT_LATCH_MS; ms++) tick(0.0f, 0.0f, SHORTED);
	CHECK(pc.sensor_fault_latched);
	CHECK(tick(0.0f, 0.0f, BRAKE_OFF));
}

int main(void) {
	RUN(test_percents_span_calibration);
	RUN(test_percents_not_clamped);
	RUN(test_apps_percent_follows_pedal_mode);

	RUN(test_normal_driving_no_cut);
	RUN(test_overshoot_buffer_boundaries);
	RUN(test_unplugged_apps_cuts_immediately);
	RUN(test_apps_range_glitch_recovers);
	RUN(test_apps_range_fault_latches_after_100ms);
	RUN(test_range_fault_timer_restarts);

	RUN(test_disagreement_cuts_after_100ms);
	RUN(test_agreement_restores_immediately);
	RUN(test_short_disagreement_restarts_timer);

	RUN(test_brake_with_throttle_cuts_immediately);
	RUN(test_crosscheck_thresholds);
	RUN(test_crosscheck_holds_until_apps_below_5_percent);

	RUN(test_reset_requires_pedal_release);
	RUN(test_reset_disagreement_cuts_without_grace);

	RUN(test_bse_range_boundaries);
	RUN(test_hard_braking_is_not_a_fault);
	RUN(test_bse_open_circuit_cuts_and_latches);
	RUN(test_bse_short_circuit_cuts_and_latches);

	return CHECK_SUMMARY();
}
