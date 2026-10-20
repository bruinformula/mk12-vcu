/*
 * rng.h
 *
 * Seeded random numbers for the Monte Carlo tests. SplitMix64: tiny, fast,
 * and a seed always gives the same sequence in the same build, which is what
 * makes a failing run replayable. Not rand(): its sequence differs between
 * C libraries.
 */

#ifndef TESTS_MC_RNG_H_
#define TESTS_MC_RNG_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct Rng {
	uint64_t state;
} Rng;

static inline uint64_t rng_u64(Rng *r) {
	uint64_t z = (r->state += 0x9E3779B97F4A7C15ull);
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
	z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
	return z ^ (z >> 31);
}

// Scrambles a number into a seed. Run i of base seed S uses
// rng_mix(rng_mix(S) + i), so nearby base seeds don't share runs.
static inline uint64_t rng_mix(uint64_t x) {
	Rng r = { x };
	return rng_u64(&r);
}

// Uniform in [0, 1).
static inline double rng_unit(Rng *r) {
	return (double)(rng_u64(r) >> 11) * 0x1.0p-53;
}

// Uniform in [lo, hi).
static inline float rng_uniform(Rng *r, float lo, float hi) {
	return lo + (hi - lo) * (float)rng_unit(r);
}

static inline bool rng_chance(Rng *r, float p) {
	return rng_unit(r) < p;
}

// Integer in [lo, hi].
static inline int rng_int(Rng *r, int lo, int hi) {
	return lo + (int)(rng_unit(r) * (hi - lo + 1));
}

// Normal with mean 0 (Box-Muller).
static inline float rng_gauss(Rng *r, float sigma) {
	double u1 = 1.0 - rng_unit(r); // (0, 1]: log(0) is -inf
	double u2 = rng_unit(r);
	return sigma * (float)(sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2));
}

static inline float rng_pick(Rng *r, const float *values, int count) {
	return values[rng_int(r, 0, count - 1)];
}

#endif /* TESTS_MC_RNG_H_ */
