/*
 * inverter_can.h
 *
 * 0x0A5 (motor rpm) timing for the Monte Carlo tests: how the inverter's
 * speed reaches the VCU. Each run gets a random broadcast period with
 * jitter, random frame loss (bus errors, a full RX FIFO) and sometimes an
 * outage (inverter reset, bus-off) of 20-500 ms.
 *
 * A frame carries the rpm at the moment it was sent, so the VCU's speed is
 * always a little old, and very old during an outage until the 100 ms stale
 * guard drops it.
 */

#ifndef TESTS_MC_INVERTER_CAN_H_
#define TESTS_MC_INVERTER_CAN_H_

#include <stdbool.h>
#include <stdint.h>
#include "rng.h"

typedef struct InverterCan {
	Rng rng;
	float period_ms;  // broadcast period
	float jitter_ms;  // +- on each period
	float latency_ms; // send to RX interrupt
	float loss_p;     // chance each frame is lost
	int outage_start, outage_end; // ticks [start, end) the inverter sends nothing

	float next_tx_ms;
	bool in_flight;
	int16_t in_flight_rpm;
	float arrive_ms;
} InverterCan;

void inverter_can_init(InverterCan *c, uint64_t seed, int run_ticks);

// Called once per tick with the true motor rpm. True if a 0x0A5 reaches the
// VCU this tick, with the rpm it carries.
bool inverter_can_rx(InverterCan *c, int tick, float motor_rpm, int16_t *rpm);

#endif /* TESTS_MC_INVERTER_CAN_H_ */
