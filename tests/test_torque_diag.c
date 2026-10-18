/*
 * test_torque_diag.c
 *
 * Host tests for Core/Src/torque_diag.c: the 0x502 byte layout. Anything
 * that decodes 0x502 (DBC, logger scripts) depends on these bytes, so a
 * change here means updating the layout in torque_diag.h and README.md too.
 *
 * Run from the repo root:  make -C tests
 */

#include <string.h>

#include "check.h"
#include "torque_diag.h"

// ---------------------
// Torque scaling (B0-3)
// ---------------------

static void test_deci_nm_scales_and_truncates(void) {
	CHECK(torqueDiag_deciNm(0.0f) == 0);
	CHECK(torqueDiag_deciNm(80.0f) == 800);
	CHECK(torqueDiag_deciNm(-30.0f) == -300);
	CHECK(torqueDiag_deciNm(12.34f) == 123); // toward zero
	CHECK(torqueDiag_deciNm(-12.34f) == -123);
}

static void test_deci_nm_matches_old_torque_cast(void) {
	// 0x0C0 used to send (int)(requestedTorque*10). It now uses
	// torqueDiag_deciNm, which must give the same value over the whole
	// regen-to-max range so the inverter sees no change.
	int mismatches = 0;
	for (int i = -40000; i <= 90000; i++) {
		float nm = i * 0.001f;
		if (torqueDiag_deciNm(nm) != (int16_t)(int)(nm * 10)) mismatches++;
	}
	CHECK(mismatches == 0);
}

static void test_deci_nm_saturates(void) {
	CHECK(torqueDiag_deciNm(5000.0f) == INT16_MAX);
	CHECK(torqueDiag_deciNm(-5000.0f) == INT16_MIN);
	CHECK(torqueDiag_deciNm(0.0f / 0.0f) == 0);
}

// ----------------
// Fault flags (B5)
// ----------------

// A healthy driving state: no flags.
static PlausibilityChecks healthy(void) {
	PlausibilityChecks pc;
	memset(&pc, 0, sizeof pc);
	pc.apps_plausible = true;
	pc.crosscheck_plausible = true;
	return pc;
}

static void test_flags_clear_when_healthy(void) {
	PlausibilityChecks pc = healthy();
	CHECK(torqueDiag_flags(&pc, false, 40.0f) == 0);
}

static void test_each_flag_has_its_own_bit(void) {
	PlausibilityChecks pc;

	pc = healthy(); pc.apps1_invalid = true;
	CHECK(torqueDiag_flags(&pc, false, 0.0f) == 0x01);
	pc = healthy(); pc.apps2_invalid = true;
	CHECK(torqueDiag_flags(&pc, false, 0.0f) == 0x02);
	pc = healthy(); pc.bse_invalid = true;
	CHECK(torqueDiag_flags(&pc, false, 0.0f) == 0x04);
	pc = healthy(); pc.apps_plausible = false;
	CHECK(torqueDiag_flags(&pc, false, 0.0f) == 0x08);
	pc = healthy(); pc.crosscheck_plausible = false;
	CHECK(torqueDiag_flags(&pc, false, 0.0f) == 0x10);
	pc = healthy(); pc.sensor_fault_latched = true;
	CHECK(torqueDiag_flags(&pc, false, 0.0f) == 0x20);
	pc = healthy();
	CHECK(torqueDiag_flags(&pc, true, 0.0f) == 0x40);
	CHECK(torqueDiag_flags(&pc, false, -5.0f) == 0x80);
}

static void test_flags_after_power_on(void) {
	// pedalChecks_reset starts both checks implausible (torque cut until the
	// pedal is released), and that should be visible on the bus.
	PlausibilityChecks pc;
	pedalChecks_reset(&pc);
	CHECK(torqueDiag_flags(&pc, true, 0.0f) ==
			(TORQUE_DIAG_APPS_IMPLAUSIBLE | TORQUE_DIAG_CROSSCHECK_IMPLAUSIBLE |
			 TORQUE_DIAG_TORQUE_CUT));
}

static void test_flags_show_latched_bse_fault(void) {
	// Bench check: unplug the BSE for > 100 ms in drive. 0x502 must say why
	// torque is cut (BSE invalid + latched), which 0x500 can't.
	PlausibilityChecks pc;
	pedalChecks_reset(&pc);
	uint16_t released[3] = { APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, BSE_ADC_MIN_VAL };
	uint16_t unplugged[3] = { APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, 70 };
	float percents[3];
	uint32_t now = 10000;

	pedalChecks_percents(released, percents);
	pedalChecks_update(&pc, released, percents, now);
	CHECK(torqueDiag_flags(&pc, pedalChecks_torqueCut(&pc, now), 0.0f) == 0);

	pedalChecks_percents(unplugged, percents);
	for (int ms = 0; ms <= SENSOR_FAULT_LATCH_MS + 1; ms++) {
		pedalChecks_update(&pc, unplugged, percents, ++now);
	}
	uint8_t flags = torqueDiag_flags(&pc, pedalChecks_torqueCut(&pc, now), 0.0f);
	CHECK(flags & TORQUE_DIAG_BSE_INVALID);
	CHECK(flags & TORQUE_DIAG_SENSOR_FAULT_LATCHED);
	CHECK(flags & TORQUE_DIAG_TORQUE_CUT);
}

// ----------
// Full frame
// ----------

static void test_pack_golden_frame(void) {
	TorqueDiag d = {
		.driver_request_nm = 40.0f,   // 400 = 0x0190
		.command_nm = -12.5f,         // -125 = 0xFF83
		.apps_percent = 0.5f,         // 100
		.flags = TORQUE_DIAG_TORQUE_CUT | TORQUE_DIAG_BSE_INVALID,
		.vcu_state = 2,
		.counter = 0xAB,
	};
	uint8_t data[8];
	uint8_t expected[8] = { 0x90, 0x01, 0x83, 0xFF, 100, 0x44, 2, 0xAB };
	torqueDiag_pack(&d, data);
	CHECK(memcmp(data, expected, 8) == 0);
}

static void test_pack_apps_half_percent(void) {
	TorqueDiag d;
	memset(&d, 0, sizeof d);
	uint8_t data[8];

	d.apps_percent = 0.0f;   torqueDiag_pack(&d, data); CHECK(data[4] == 0);
	d.apps_percent = 1.0f;   torqueDiag_pack(&d, data); CHECK(data[4] == 200);
	d.apps_percent = 0.0026f; torqueDiag_pack(&d, data); CHECK(data[4] == 1); // rounds
	d.apps_percent = 1.1f;   torqueDiag_pack(&d, data); CHECK(data[4] == 220); // overshoot kept
	d.apps_percent = -0.08f; torqueDiag_pack(&d, data); CHECK(data[4] == 0);
	d.apps_percent = 3.0f;   torqueDiag_pack(&d, data); CHECK(data[4] == 255);
	d.apps_percent = 0.0f / 0.0f; torqueDiag_pack(&d, data); CHECK(data[4] == 0);
}

int main(void) {
	RUN(test_deci_nm_scales_and_truncates);
	RUN(test_deci_nm_matches_old_torque_cast);
	RUN(test_deci_nm_saturates);

	RUN(test_flags_clear_when_healthy);
	RUN(test_each_flag_has_its_own_bit);
	RUN(test_flags_after_power_on);
	RUN(test_flags_show_latched_bse_fault);

	RUN(test_pack_golden_frame);
	RUN(test_pack_apps_half_percent);

	return CHECK_SUMMARY();
}
