/*
 * torque_map.h
 *
 * Pedal-to-torque pipeline as pure functions: no HAL, CAN, or global state.
 * Compiles on the host (tools/torque_curve_sim.c) as well as on the VCU, so
 * the sim and future unit tests run the exact code that drives the car.
 */

#ifndef INC_TORQUE_MAP_H_
#define INC_TORQUE_MAP_H_

#include <stdbool.h>
#include <stdint.h>

// Pedal bands (Cascadia's layout): regen below REGEN_END_PERCENT, coast at
// 0 Nm up to APPS_INFLECTION_PERCENT, drive torque above it. The coast band
// keeps a resting foot from regenerating.
#define REGEN_END_PERCENT 0.03f
#define APPS_INFLECTION_PERCENT 0.08f

#define RC_TIME_CONSTANT 0.05f // Tau, RC Time Constant for digital LPF
#define SLEW_RATE_LIMIT 2000.0f // Adjust according to event

#define MAX_TORQUE 80 // Adjust according to event
#define MIN_TORQUE 0
#define REGEN_LIFTOFF_TORQUE -5.0f // Regen at 0% pedal, full speed. Start small, raise with the drivers
#define REGEN_MAX_TORQUE -30 // Hard limit: no regen command goes past this, whatever the tuning

// Regen kill switch. 0: pedal below APPS_INFLECTION_PERCENT coasts at 0 Nm
// and regenTorque() is never used. feature/regen ships with 1 (that's what it
// is for); feature/torque-mapping keeps 0 until this branch passes its bench
// checks. The tests build both ways.
#ifndef REGEN_ENABLED
#define REGEN_ENABLED 1
#endif

#define REGEN_MIN_CARSPEED 5.0f // No regen below this car speed (km/h), EV.3.3.3
// Regen fades in with speed: none below REGEN_FADE_LO_KPH, full above
// REGEN_FADE_HI_KPH, linear in between. So on a coast-down regen is already
// 0 Nm well before the 5 km/h cutoff.
#define REGEN_FADE_LO_KPH 8.0f
#define REGEN_FADE_HI_KPH 20.0f
#define REGEN_RAMP_NM_S 40.0f // Regen builds no faster than this (on top of the filter)
#define REGEN_BRAKE_CANCEL_PERCENT 0.05f // No regen once brake pedal passes this

// Car speed from motor rpm (0x0A5 bytes 2-3). VERIFY ON CAR (bench plan 2.9).
// The tire is the smallest plausible one: if the real tire is bigger, speed
// reads low, which is the safe direction for EV.3.3.3 (regen starts later).
#define GEAR_RATIO (59.0f / 12.0f) // motor turns per wheel turn
#define TIRE_DIAMETER_IN 16.0f
// Sign of the motor rpm when the car rolls forward. VERIFY ON CAR. If it's
// wrong, forward speed reads negative and regen never engages (safe).
#define MOTOR_FORWARD_RPM_SIGN 1
// No 0x0A5 for this long means the speed is stale and reads 0 (no regen).
#define CARSPEED_STALE_MS 100

// Car speed in km/h from motor rpm: positive forward, negative rolling back.
float torqueMap_carspeedKph(int16_t motor_rpm);

// carspeed if the last 0x0A5 arrived within CARSPEED_STALE_MS of now_ms,
// else 0 (regen off, never on). Tick wraparound safe.
float torqueMap_trustedCarspeed(float carspeed, uint32_t now_ms, uint32_t last_rx_ms);

// Pedal map: fraction of MAX_TORQUE at TORQUE_MAP_POINTS evenly spaced points
// of pedal travel, from APPS_INFLECTION_PERCENT (first entry) to full pedal
// (last). Linear in between. Defined in torque_map.c; must pass
// torqueMap_tableIsValid() or the VCU halts at boot.
// To tune: edit the table, run `make -C tests` and tools/torque_curve_sim.c.
#define TORQUE_MAP_POINTS 11
extern const float torqueMap_table[TORQUE_MAP_POINTS];

// True if a map starts at 0, ends at 1, stays within [0, 1], and never
// decreases (more pedal must never mean less torque).
bool torqueMap_tableIsValid(const float *table, int points);

// Target torque (Nm) for a plausible pedal input. Positive above
// APPS_INFLECTION_PERCENT, 0 Nm (coast) down to REGEN_END_PERCENT, regen
// below that if REGEN_ENABLED. carspeed is forward-signed km/h, so rolling
// backward never regens. Pedal inputs are clamped to [0, 1].
// Fault handling is the caller's job.
float torqueMap_target(float apps_percent, float bse_percent, float carspeed);

// One step of the LPF + slew rate limiter. Updates *filtered and returns the
// new output torque given the previous output and dt in seconds.
float torqueMap_filterStep(float *filtered, float target, float prev_output, float dt);

typedef struct TorqueFilter {
	float filtered; // LPF state
	float output;   // last commanded torque (after the slew limiter)
	float target;   // last pedal map target (the driver's request), for diagnostics
} TorqueFilter;

// Back to 0 Nm. Torque ramps up from here through the filter.
void torqueMap_filterReset(TorqueFilter *f);

// Torque command for one control loop tick. A torque cut returns 0 Nm on the
// same tick (no filtering) and resets the filter. Otherwise the map's target
// goes through the LPF + slew limiter, and the result is kept between 0 Nm and
// the target (EV.3.1.3), so the filter only ever slows torque coming in.
// Regen also builds no faster than REGEN_RAMP_NM_S, and stops instantly.
// f->target is set to the map target on every tick, cut or not.
float torqueMap_request(TorqueFilter *f, bool torque_cut, float apps_percent,
		float bse_percent, float carspeed, float dt);

#endif /* INC_TORQUE_MAP_H_ */
