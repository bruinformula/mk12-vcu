/*
 * inverter_can.c
 *
 * 0x0A5 timing model for the Monte Carlo tests. See inverter_can.h.
 */

#include <limits.h>
#include "inverter_can.h"
#include "scenario.h" // MC_TICK_MS

void inverter_can_init(InverterCan *c, uint64_t seed, int run_ticks) {
	*c = (InverterCan){ .rng = { seed } };
	Rng *r = &c->rng;
	// The real period is set in the inverter's EEPROM (the bench plan sends
	// 0x0A5 at 100 Hz). Any of these must be safe.
	c->period_ms = rng_uniform(r, 3.0f, 20.0f);
	c->jitter_ms = rng_uniform(r, 0.0f, 1.0f);
	c->latency_ms = rng_uniform(r, 0.1f, 1.5f);
	c->loss_p = rng_chance(r, 0.5f) ? 0.0f : rng_uniform(r, 0.0f, 0.2f);
	c->next_tx_ms = rng_uniform(r, 0.0f, c->period_ms);

	c->outage_start = c->outage_end = INT_MAX;
	if (rng_chance(r, 0.3f)) {
		c->outage_start = rng_int(r, 0, run_ticks - 1);
		c->outage_end = c->outage_start + rng_int(r, 20, 500) / MC_TICK_MS;
	}
}

bool inverter_can_rx(InverterCan *c, int tick, float motor_rpm, int16_t *rpm) {
	float now_ms = (float)(tick * MC_TICK_MS);
	bool rx = false;

	if (c->in_flight && now_ms >= c->arrive_ms) {
		*rpm = c->in_flight_rpm;
		c->in_flight = false;
		rx = true;
	}

	if (now_ms >= c->next_tx_ms) {
		bool silent = tick >= c->outage_start && tick < c->outage_end;
		if (!silent && !rng_chance(&c->rng, c->loss_p)) {
			float clamped = fminf(fmaxf(roundf(motor_rpm), INT16_MIN), INT16_MAX);
			c->in_flight_rpm = (int16_t)clamped;
			c->in_flight = true;
			c->arrive_ms = c->next_tx_ms + c->latency_ms;
		}
		c->next_tx_ms += c->period_ms + rng_uniform(&c->rng, -c->jitter_ms, c->jitter_ms);
	}
	return rx;
}
