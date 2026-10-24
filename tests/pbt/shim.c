/*
 * shim.c
 *
 * Flat C entry points into the real firmware files for the property-based
 * tests (test_properties.py, via ctypes). Holds one PlausibilityChecks and
 * one TorqueFilter so Python never has to mirror the struct layouts.
 * Built as a shared library by `make -C tests pbt`.
 */

#include "pedal_checks.h"
#include "power_limit.h"
#include "torque_diag.h"
#include "torque_map.h"
#include <math.h>

static PlausibilityChecks pc;
static TorqueFilter tf;

// Constants, read by Python with ctypes in_dll so the tests never go stale.
const int pbt_regen_enabled = REGEN_ENABLED;
const int pbt_pedal_mode = PEDAL_MODE;
const int pbt_apps1_valid_lo = APPS1_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER;
const int pbt_apps1_valid_hi = APPS1_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER;
const int pbt_apps2_valid_lo = APPS2_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER;
const int pbt_apps2_valid_hi = APPS2_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER;
const int pbt_apps1_released = APPS1_ADC_MIN_VAL;
const int pbt_apps2_released = APPS2_ADC_MIN_VAL;
const int pbt_apps1_full = APPS1_ADC_MAX_VAL;
const int pbt_apps2_full = APPS2_ADC_MAX_VAL;
const int pbt_bse_released = BSE_ADC_MIN_VAL;
const int pbt_bse_valid_lo = BSE_ADC_OPEN_CIRCUIT_VAL;
const int pbt_bse_valid_hi = BSE_ADC_SHORT_CIRCUIT_VAL;
const int pbt_bse_pressed = BSE_ACTIVATED_ADC_THRESHOLD;
const float pbt_max_torque = MAX_TORQUE;
const float pbt_regen_max_torque = REGEN_MAX_TORQUE;
const float pbt_regen_min_speed = REGEN_MIN_CARSPEED;
const float pbt_regen_brake_cancel = REGEN_BRAKE_CANCEL_PERCENT;
const float pbt_slew = SLEW_RATE_LIMIT;
const float pbt_regen_ramp = REGEN_RAMP_NM_S;
const int pbt_stale_ms = CARSPEED_STALE_MS;
const float pbt_power_limit_w = POWER_LIMIT_W;
const float pbt_efficiency = DRIVETRAIN_EFFICIENCY;
const float pbt_motor_max_rpm = MOTOR_MAX_RPM;

// --- pedal checks ---

void pbt_pedal_reset(void) {
	pedalChecks_reset(&pc);
}

// Bits: 1 torque cut, 2 latched, 4 APPS1 invalid, 8 APPS2 invalid, 16 BSE invalid
int pbt_pedal_tick(int a1, int a2, int bse, uint32_t now_ms) {
	uint16_t adc[3] = { (uint16_t)a1, (uint16_t)a2, (uint16_t)bse };
	float p[3];
	pedalChecks_percents(adc, p);
	pedalChecks_update(&pc, adc, p, now_ms);
	return (pedalChecks_torqueCut(&pc, now_ms) ? 1 : 0) |
			(pc.sensor_fault_latched ? 2 : 0) |
			(pc.apps1_invalid ? 4 : 0) | (pc.apps2_invalid ? 8 : 0) | (pc.bse_invalid ? 16 : 0);
}

// |APPS1 - APPS2| travel and the APPS travel used for torque, in float as
// the firmware computes them (so a reading right on 10% compares the same).
float pbt_apps_diff(int a1, int a2) {
	uint16_t adc[3] = { (uint16_t)a1, (uint16_t)a2, BSE_ADC_MIN_VAL };
	float p[3];
	pedalChecks_percents(adc, p);
	return fabsf(p[PEDAL_APPS1] - p[PEDAL_APPS2]);
}

float pbt_apps_used(int a1, int a2) {
	uint16_t adc[3] = { (uint16_t)a1, (uint16_t)a2, BSE_ADC_MIN_VAL };
	float p[3];
	pedalChecks_percents(adc, p);
	return pedalChecks_appsPercent(p);
}

int pbt_brake_held_for_rtd(int bse) {
	return pedalChecks_brakeHeldForRTD((uint16_t)bse);
}

// --- torque map ---

float pbt_target(float apps, float bse, float speed) {
	return torqueMap_target(apps, bse, speed);
}

void pbt_filter_reset(void) {
	torqueMap_filterReset(&tf);
}

float pbt_request(int cut, float apps, float bse, float speed, float cap, float dt) {
	return torqueMap_requestLimited(&tf, cut != 0, apps, bse, speed, cap, dt);
}

float pbt_filter_target(void) {
	return tf.target;
}

float pbt_carspeed(int rpm) {
	return torqueMap_carspeedKph((int16_t)rpm);
}

float pbt_trusted_speed(float speed, uint32_t now_ms, uint32_t rx_ms) {
	return torqueMap_trustedCarspeed(speed, now_ms, rx_ms);
}

// --- power limit ---

float pbt_power_cap(float rpm) {
	return powerLimit_capNm(rpm);
}

float pbt_power_cap_rpm(float rpm, uint32_t now_ms, uint32_t rx_ms) {
	return powerLimit_capRpm(rpm, now_ms, rx_ms);
}

// --- 0x502 ---

int pbt_deci_nm(float nm) {
	return torqueDiag_deciNm(nm);
}
