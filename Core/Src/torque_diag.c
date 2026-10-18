/*
 * torque_diag.c
 *
 * Packing for the 0x502 torque diagnostics frame. Layout in torque_diag.h.
 */

#include "torque_diag.h"

int16_t torqueDiag_deciNm(float nm) {
	float deci = nm * 10.0f;
	if (deci >= 32767.0f) return INT16_MAX;
	if (deci <= -32768.0f) return INT16_MIN;
	if (deci != deci) return 0; // NaN
	return (int16_t)deci;
}

// 0 = released, 1 = full -> 0.5 % counts, rounded, saturated to [0, 255].
static uint8_t halfPercent(float percent) {
	float counts = percent * 200.0f + 0.5f;
	if (!(counts > 0.0f)) return 0; // also catches NaN
	if (counts >= 255.0f) return 255;
	return (uint8_t)counts;
}

uint8_t torqueDiag_flags(const PlausibilityChecks *pc, bool torque_cut,
		float driver_request_nm) {
	uint8_t flags = 0;
	if (pc->apps1_invalid) flags |= TORQUE_DIAG_APPS1_INVALID;
	if (pc->apps2_invalid) flags |= TORQUE_DIAG_APPS2_INVALID;
	if (pc->bse_invalid) flags |= TORQUE_DIAG_BSE_INVALID;
	if (!pc->apps_plausible) flags |= TORQUE_DIAG_APPS_IMPLAUSIBLE;
	if (!pc->crosscheck_plausible) flags |= TORQUE_DIAG_CROSSCHECK_IMPLAUSIBLE;
	if (pc->sensor_fault_latched) flags |= TORQUE_DIAG_SENSOR_FAULT_LATCHED;
	if (torque_cut) flags |= TORQUE_DIAG_TORQUE_CUT;
	if (driver_request_nm < 0.0f) flags |= TORQUE_DIAG_REGEN;
	return flags;
}

void torqueDiag_pack(const TorqueDiag *d, uint8_t data[8]) {
	uint16_t request = (uint16_t)torqueDiag_deciNm(d->driver_request_nm);
	uint16_t command = (uint16_t)torqueDiag_deciNm(d->command_nm);

	data[0] = (uint8_t)(request & 0xFF);
	data[1] = (uint8_t)(request >> 8);
	data[2] = (uint8_t)(command & 0xFF);
	data[3] = (uint8_t)(command >> 8);
	data[4] = halfPercent(d->apps_percent);
	data[5] = d->flags;
	data[6] = d->vcu_state;
	data[7] = d->counter;
}
