/*
 * torque_diag.h
 *
 * Torque diagnostics CAN frame (0x502): what the driver asked for, what was
 * sent to the inverter, and why torque was cut. Packing is a pure function
 * (no HAL, CAN, or global state) so tests/test_torque_diag.c pins the byte
 * layout. Sent from the control loop right after the 0x0C0 torque frame, so
 * every field comes from the same tick.
 *
 * 0x502, 8 bytes, little-endian, 100 Hz while torque frames are being sent:
 *   B0-1  driver request (pedal map target, before filter)  int16  0.1 Nm
 *   B2-3  final command (same value as 0x0C0 B0-1)          int16  0.1 Nm
 *   B4    APPS travel used for torque (PEDAL_MODE)          uint8  0.5 %
 *   B5    fault flags, TORQUE_DIAG_* bits below
 *   B6    vcu_state (VCU_STATE enum)
 *   B7    rolling counter, +1 every send slot; a gap means a frame was
 *         skipped (TX FIFO full) or lost
 */

#ifndef INC_TORQUE_DIAG_H_
#define INC_TORQUE_DIAG_H_

#include <stdbool.h>
#include <stdint.h>
#include "pedal_checks.h"

#define TORQUE_DIAG_TX_ID 0x502

// B5 bits
#define TORQUE_DIAG_APPS1_INVALID          (1u << 0) // APPS1 out of range this tick
#define TORQUE_DIAG_APPS2_INVALID          (1u << 1)
#define TORQUE_DIAG_BSE_INVALID            (1u << 2)
#define TORQUE_DIAG_APPS_IMPLAUSIBLE       (1u << 3) // APPS1 vs APPS2 (T.4.2.4)
#define TORQUE_DIAG_CROSSCHECK_IMPLAUSIBLE (1u << 4) // APPS + brake (EV.4.7)
#define TORQUE_DIAG_SENSOR_FAULT_LATCHED   (1u << 5) // stays set until leaving drive
#define TORQUE_DIAG_TORQUE_CUT             (1u << 6) // command forced to 0 Nm this tick
#define TORQUE_DIAG_REGEN                  (1u << 7) // driver request < 0

typedef struct TorqueDiag {
	float driver_request_nm;
	float command_nm;
	float apps_percent; // 0 = released, 1 = full; may overshoot slightly
	uint8_t flags;
	uint8_t vcu_state;
	uint8_t counter;
} TorqueDiag;

// Nm -> 0.1 Nm counts, truncated toward zero like the original 0x0C0 cast,
// saturated to the int16 range. Used for 0x0C0 too, so B2-3 always matches it.
int16_t torqueDiag_deciNm(float nm);

// B5 for one tick.
uint8_t torqueDiag_flags(const PlausibilityChecks *pc, bool torque_cut,
		float driver_request_nm);

void torqueDiag_pack(const TorqueDiag *d, uint8_t data[8]);

#endif /* INC_TORQUE_DIAG_H_ */
