/*
 * pedal_checks.c
 *
 * Pedal sensor scaling and plausibility checks. See pedal_checks.h.
 */

#include "pedal_checks.h"
#include <math.h> // fabsf()

void pedalChecks_percents(const uint16_t adc[3], float percents[3]) {
	percents[PEDAL_APPS1] = ((float) adc[PEDAL_APPS1] - APPS1_ADC_MIN_VAL) / (APPS1_ADC_MAX_VAL - APPS1_ADC_MIN_VAL);
	percents[PEDAL_APPS2] = ((float) adc[PEDAL_APPS2] - APPS2_ADC_MIN_VAL) / (APPS2_ADC_MAX_VAL - APPS2_ADC_MIN_VAL);
	percents[PEDAL_BSE] = ((float) adc[PEDAL_BSE] - BSE_ADC_MIN_VAL) / (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL);
}

float pedalChecks_appsPercent(const float percents[3]) {
#if PEDAL_MODE == TWO_APPS
	return (percents[PEDAL_APPS1] + percents[PEDAL_APPS2]) / 2;
#elif PEDAL_MODE == ONLY_APPS1
	return percents[PEDAL_APPS1];
#elif PEDAL_MODE == ONLY_APPS2
	return percents[PEDAL_APPS2];
#elif PEDAL_MODE == MIN_APPS
	return fminf(percents[PEDAL_APPS1], percents[PEDAL_APPS2]);
#else
#error "Unknown PEDAL_MODE"
#endif
}

static bool sensorOutOfRange(const PlausibilityChecks *pc) {
	return pc->apps1_invalid || pc->apps2_invalid || pc->bse_invalid;
}

static void validateAPPS(PlausibilityChecks *pc, const uint16_t adc[3]) {
// Note that as APPS1 and APPS2 are pressed, raw ADC value DECREASES.
// Handles 3 cases and cuts torque to 0 in case.
// 1. APPS1 or APPS2 are plugged in and not pressed; ADC Value will be larger than ADC_MIN_VAL.
// 2. APPS1 or APPS2 are plugged in and pressed beyond intended range of motion; ADC Value will be smaller than ADC_MAX_VAL.
// 3. APPS1 or APPS2 are unplugged and raw ADC value is ~ 70; this is smaller than ADC_MAX_VAL.

	pc->apps1_invalid = (adc[PEDAL_APPS1] > (APPS1_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER)) ||
			(adc[PEDAL_APPS1] < (APPS1_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER));
	pc->apps2_invalid = (adc[PEDAL_APPS2] > (APPS2_ADC_MIN_VAL + APPS_OVERSHOOT_BUFFER)) ||
			(adc[PEDAL_APPS2] < (APPS2_ADC_MAX_VAL - APPS_OVERSHOOT_BUFFER));
}

// T.4.3.4: BSE reads HIGHER as the brake is pressed. Out of range = open or short circuit.
static void validateBSE(PlausibilityChecks *pc, const uint16_t adc[3]) {
	pc->bse_invalid = (adc[PEDAL_BSE] < BSE_ADC_OPEN_CIRCUIT_VAL) ||
			(adc[PEDAL_BSE] > BSE_ADC_SHORT_CIRCUIT_VAL);
}

// Out of range cuts torque right away (see pedalChecks_torqueCut). If it
// lasts longer than SENSOR_FAULT_LATCH_MS it's a real wiring fault, not ADC
// noise, so latch it: a flickering connector must not keep giving torque back.
static void checkSensorRanges(PlausibilityChecks *pc, const uint16_t adc[3], uint32_t now_ms) {
	bool was_out_of_range = sensorOutOfRange(pc);
	validateAPPS(pc, adc);
	validateBSE(pc, adc);

	if (!sensorOutOfRange(pc)) {
		return;
	}
	if (!was_out_of_range) {
		pc->out_of_range_since_ms = now_ms;
	} else if ((now_ms - pc->out_of_range_since_ms) > SENSOR_FAULT_LATCH_MS) {
		pc->sensor_fault_latched = true;
	}
}

// T.4.2.4 / T.4.2.5: APPS1 and APPS2 must agree within 10% pedal travel.
// Disagreement only cuts torque once it has lasted APPS_IMPLAUSIBILITY_TIMEOUT_MS.
static void checkAPPS_Plausibility(PlausibilityChecks *pc, const float percents[3], uint32_t now_ms) {
	float pedal_travel_difference_percent =
			fabsf(percents[PEDAL_APPS1] - percents[PEDAL_APPS2]);
	bool apps_invalid = (pedal_travel_difference_percent >
			APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE);

	if (pc->apps_plausible == true && apps_invalid == true) {
		pc->apps_implausible_since_ms = now_ms;
		pc->apps_plausible = false;
	}

	if (pc->apps_plausible == false && apps_invalid == false) {
		pc->apps_plausible = true;
		pc->apps_no_grace = false; // later disagreements get the 100 ms
	}
}

// EV.4.7: brake pressed with APPS over 25% cuts torque until APPS drops to 5%.
static void checkAPPS_BSE_Crosscheck(PlausibilityChecks *pc, const uint16_t adc[3], const float percents[3]) {
	float apps_percent = pedalChecks_appsPercent(percents);

	if (pc->crosscheck_plausible == true &&
			(apps_percent > CROSSCHECK_IMPLAUSIBILITY_PERCENT_DIFFERENCE) &&
			(adc[PEDAL_BSE] > BSE_ACTIVATED_ADC_THRESHOLD)) {
		pc->crosscheck_plausible = false;
	}

	if (pc->crosscheck_plausible == false &&
			apps_percent <= CROSSCHECK_RESTORATION_APPS_PERCENT) {
		pc->crosscheck_plausible = true;
	}
}

void pedalChecks_update(PlausibilityChecks *pc, const uint16_t adc[3],
		const float percents[3], uint32_t now_ms) {
	checkSensorRanges(pc, adc, now_ms);
	checkAPPS_Plausibility(pc, percents, now_ms);
	checkAPPS_BSE_Crosscheck(pc, adc, percents);
}

bool pedalChecks_torqueCut(const PlausibilityChecks *pc, uint32_t now_ms) {
	if (sensorOutOfRange(pc) || pc->sensor_fault_latched) {
		return true;
	}
	if (!pc->crosscheck_plausible) {
		return true;
	}
	if (!pc->apps_plausible && (pc->apps_no_grace ||
			(now_ms - pc->apps_implausible_since_ms) > APPS_IMPLAUSIBILITY_TIMEOUT_MS)) {
		return true;
	}
	return false;
}

void pedalChecks_reset(PlausibilityChecks *pc) {
	pc->apps1_invalid = false;
	pc->apps2_invalid = false;
	pc->bse_invalid = false;
	pc->apps_plausible = false;
	pc->crosscheck_plausible = false;
	pc->sensor_fault_latched = false;
	pc->out_of_range_since_ms = 0;
	// An APPS disagreement right after reset cuts torque immediately (no
	// 100 ms grace). That used to rely on the timestamp being 0, which stops
	// working for 100 ms when HAL_GetTick() wraps to 0 (found by tests/mc).
	pc->apps_no_grace = true;
	pc->apps_implausible_since_ms = 0;
}

bool pedalChecks_brakeHeldForRTD(uint16_t bse_adc) {
	// Same short-circuit bound as validateBSE, so RTD and the torque cut agree
	// on what a valid reading is.
	return bse_adc > BSE_ACTIVATED_ADC_THRESHOLD &&
			bse_adc <= BSE_ADC_SHORT_CIRCUIT_VAL;
}
