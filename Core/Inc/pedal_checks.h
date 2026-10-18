/*
 * pedal_checks.h
 *
 * Pedal sensor scaling and plausibility checks (APPS range, APPS1 vs APPS2,
 * APPS/brake crosscheck) as pure functions: no HAL, CAN, or global state.
 * Time comes in as now_ms so the host tests (tests/test_pedal_checks.c)
 * can drive the 100 ms timeouts directly.
 */

#ifndef INC_PEDAL_CHECKS_H_
#define INC_PEDAL_CHECKS_H_

#include <stdbool.h>
#include <stdint.h>

// PEDAL_MODE IS EITHER TWO_APPS, ONLY_APPS1, OR ONLY APPS2
#define TWO_APPS 0
#define ONLY_APPS1 1
#define ONLY_APPS2 2
#define PEDAL_MODE ONLY_APPS2

// Raw ADC counts. APPS1/APPS2 read LOWER as the pedal is pressed (MAX < MIN).
#define APPS1_ADC_MAX_VAL 320
#define APPS1_ADC_MIN_VAL 1400

#define APPS2_ADC_MAX_VAL 470
#define APPS2_ADC_MIN_VAL 1965

#define BSE_ADC_MAX_VAL 1500
#define BSE_ADC_MIN_VAL 375

#define APPS_OVERSHOOT_BUFFER 125
#define APPS_IMPLAUSIBILITY_PERCENT_DIFFERENCE 0.10f
#define APPS_IMPLAUSIBILITY_TIMEOUT_MS 100

#define BSE_IMPLAUSIBILITY_TIMEOUT_MS 100
#define BSE_ACTIVATED_ADC_THRESHOLD 1000

#define CROSSCHECK_IMPLAUSIBILITY_PERCENT_DIFFERENCE 0.25f
#define CROSSCHECK_RESTORATION_APPS_PERCENT 0.05f

// ADC_VAL / pedal_percents index order
#define PEDAL_APPS1 0
#define PEDAL_APPS2 1
#define PEDAL_BSE 2

typedef struct PlausibilityChecks {
  bool apps1_invalid;
  bool apps2_invalid;
  bool apps_plausible;
  bool crosscheck_plausible;
  uint32_t apps_implausible_since_ms;
} PlausibilityChecks;

// Raw ADC counts -> pedal travel, 0 = released, 1 = full. Not clamped:
// readings outside the calibrated range give values below 0 or above 1.
void pedalChecks_percents(const uint16_t adc[3], float percents[3]);

// The APPS travel used for torque and the crosscheck, chosen by PEDAL_MODE.
float pedalChecks_appsPercent(const float percents[3]);

// Run every check for one control loop tick.
void pedalChecks_update(PlausibilityChecks *pc, const uint16_t adc[3],
		const float percents[3], uint32_t now_ms);

// True if any check currently requires torque to be cut.
bool pedalChecks_torqueCut(const PlausibilityChecks *pc, uint32_t now_ms);

// Back to the power-on state. Both checks start implausible, so torque stays
// cut until the APPS agree and the pedal has been released below 5%.
void pedalChecks_reset(PlausibilityChecks *pc);

#endif /* INC_PEDAL_CHECKS_H_ */
