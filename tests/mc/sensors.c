/*
 * sensors.c
 *
 * Pedal sensor model for the Monte Carlo tests. See sensors.h.
 */

#include <limits.h>
#include "pedal_checks.h"
#include "sensors.h"

#define ADC_FULL_SCALE 4095.0f
#define OPEN_COUNTS 70.0f // unplugged, through the pull-down (pedal_checks.c)

// Healthy sensors. Pessimistic guesses until bench data replaces them
// (phase 5): offset and gain error up to 1.5% of travel each, noise up to
// 4 counts RMS, clipped at 3 sigma.
#define CAL_OFFSET_MAX 0.015f
#define CAL_GAIN_MAX 0.015f
#define NOISE_SIGMA_MAX 4.0f
#define NOISE_CLIP 3.0f

const char *const FAULT_NAMES[FAULT_TYPES] = {
	[FAULT_NONE] = "none",
	[FAULT_OPEN] = "open",
	[FAULT_SHORT] = "short",
	[FAULT_STUCK] = "stuck",
	[FAULT_OFFSET] = "offset",
	[FAULT_INTERMITTENT] = "intermittent",
};

// Counts at released and full travel: the calibration in pedal_checks.h.
static const float RELEASED[3] = { APPS1_ADC_MIN_VAL, APPS2_ADC_MIN_VAL, BSE_ADC_MIN_VAL };
static const float PRESSED[3] = { APPS1_ADC_MAX_VAL, APPS2_ADC_MAX_VAL, BSE_ADC_MAX_VAL };

void sensors_init(Sensors *s, uint64_t seed, int run_ticks) {
	*s = (Sensors){ .rng = { seed } };
	Rng *r = &s->rng;

	for (int i = 0; i < 3; i++) {
		SensorChannel *c = &s->ch[i];
		c->offset = rng_uniform(r, -CAL_OFFSET_MAX, CAL_OFFSET_MAX);
		c->gain = 1.0f + rng_uniform(r, -CAL_GAIN_MAX, CAL_GAIN_MAX);
		c->noise = rng_uniform(r, 0.0f, NOISE_SIGMA_MAX);
	}

	int faults = rng_chance(r, 0.4f) ? 0 : rng_chance(r, 0.75f) ? 1 : 2;
	for (int k = 0; k < faults; k++) {
		SensorChannel *c = &s->ch[rng_int(r, 0, 2)];
		if (c->fault != FAULT_NONE) continue; // one fault per sensor
		c->fault = rng_int(r, FAULT_NONE + 1, FAULT_TYPES - 1);
		c->fault_start = rng_int(r, 0, run_ticks - 1);
		// Some faults are glitches either side of the 100 ms timeouts, the
		// rest last the whole run.
		c->fault_end = rng_chance(r, 0.4f) ? c->fault_start + rng_int(r, 1, 200) : INT_MAX;
		c->fault_offset = (rng_chance(r, 0.5f) ? 1.0f : -1.0f) * rng_uniform(r, 0.15f, 0.5f);
		c->flicker_p = rng_uniform(r, 0.005f, 0.3f);
	}
}

void sensors_read(Sensors *s, int tick, float apps, float bse, uint16_t adc[3]) {
	Rng *r = &s->rng;
	const float travel[3] = { apps, apps, bse };

	for (int i = 0; i < 3; i++) {
		SensorChannel *c = &s->ch[i];
		c->faulted = c->fault != FAULT_NONE && tick >= c->fault_start && tick < c->fault_end;

		float t = travel[i] * c->gain + c->offset;
		if (c->faulted && c->fault == FAULT_OFFSET) t += c->fault_offset;
		float ideal = RELEASED[i] + (PRESSED[i] - RELEASED[i]) * t;

		if (c->faulted) {
			switch (c->fault) {
			case FAULT_OPEN:
				ideal = OPEN_COUNTS;
				break;
			case FAULT_SHORT:
				ideal = ADC_FULL_SCALE;
				break;
			case FAULT_STUCK:
				if (tick == c->fault_start) c->stuck_counts = ideal;
				ideal = c->stuck_counts;
				break;
			case FAULT_INTERMITTENT:
				if (rng_chance(r, c->flicker_p)) c->flicker_open = !c->flicker_open;
				if (c->flicker_open) ideal = OPEN_COUNTS;
				break;
			default:
				break;
			}
		}

		float noise = c->noise * fminf(fmaxf(rng_gauss(r, 1.0f), -NOISE_CLIP), NOISE_CLIP);
		adc[i] = (uint16_t)roundf(fminf(fmaxf(ideal + noise, 0.0f), ADC_FULL_SCALE));
		c->ideal = ideal;
		c->bound = NOISE_CLIP * c->noise + 0.5f + 1e-3f; // noise + rounding
	}
}
