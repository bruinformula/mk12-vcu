/*
 * sensors.h
 *
 * Truth -> what the VCU reads: the two APPS and the BSE as raw ADC counts,
 * with per-run calibration error, noise and injected wiring faults.
 *
 * Noise is clipped to a known bound, so every reading lies in a known span
 * of counts (ideal +- bound). That lets the oracle tell "definitely out of
 * range" from "possibly", and only demand a cut where there's no doubt.
 */

#ifndef TESTS_MC_SENSORS_H_
#define TESTS_MC_SENSORS_H_

#include <stdbool.h>
#include <stdint.h>
#include "rng.h"

enum {
	FAULT_NONE,
	FAULT_OPEN,         // unplugged: reads ~70 counts through the pull-down
	FAULT_SHORT,        // shorted to the sensor supply: full scale
	FAULT_STUCK,        // frozen where it was when the fault started
	FAULT_OFFSET,       // reads 15-50% pedal travel off (bent bracket, slipped magnet)
	FAULT_INTERMITTENT, // flickers between open and good (loose connector)
	FAULT_TYPES
};

extern const char *const FAULT_NAMES[FAULT_TYPES];

typedef struct SensorChannel {
	// Healthy calibration error, fixed for the run.
	float offset; // pedal travel added
	float gain;   // ~1
	float noise;  // sigma, counts

	// Injected fault, active on ticks [fault_start, fault_end).
	int fault;
	int fault_start, fault_end;
	float fault_offset;  // FAULT_OFFSET: pedal travel added
	float flicker_p;     // FAULT_INTERMITTENT: chance per tick of toggling
	bool flicker_open;
	float stuck_counts;

	// This tick.
	bool faulted;
	float ideal; // counts before noise
	float bound; // |adc - ideal| <= bound
} SensorChannel;

typedef struct Sensors {
	Rng rng;
	SensorChannel ch[3]; // PEDAL_APPS1, PEDAL_APPS2, PEDAL_BSE
} Sensors;

// Random calibration error and 0-2 random faults for one run of run_ticks.
void sensors_init(Sensors *s, uint64_t seed, int run_ticks);

// Raw ADC counts for true pedal and brake travel (0 = released, 1 = full).
void sensors_read(Sensors *s, int tick, float apps, float bse, uint16_t adc[3]);

#endif /* TESTS_MC_SENSORS_H_ */
