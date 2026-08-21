/*
 * gui_protocol.h
 *
 * Command/response protocol for the mk12-vcu-gui bench tool, carried on
 * FDCAN2 (GUI_CMD_RX_ID / GUI_RESP_TX_ID in fdcan.h). Never wired to FDCAN1
 * (drive-critical bus).
 *
 * Scope right now: connectivity (PING, reports vcu_state + whether this is a
 * bench build), live plausibility/calibration constants (GET_PARAM/
 * SET_PARAM), and fault injection for bench testing driven off inverter/
 * sensor feedback overrides. Nothing here touches torque requests or any
 * torque-shaping constant (MAX/MIN/REGEN_MAX_TORQUE, SLEW_RATE_LIMIT,
 * RC_TIME_CONSTANT, APPS_INFLECTION_PERCENT) -- those stay fixed firmware
 * #defines, explicitly out of scope until revisited later, informed by
 * inverter feedback data and proper software-based testing first.
 *
 * SET_PARAM/FAULT_INJECT/FAULT_CLEAR are rejected with GUI_STATUS_ERR_STATE
 * unless vcu_state == VCU_IDLE. Fault injection is additionally only enabled
 * at all when VCU_BENCH_MODE is defined at compile time -- it must never be
 * present in a competition build.
 */

#ifndef INC_GUI_PROTOCOL_H_
#define INC_GUI_PROTOCOL_H_

#include "fdcan.h"
#include "stdbool.h"
#include "stdint.h"

// Command frame (8 bytes): [opcode][id][value f32 LE][sequence][reserved]
// Response frame (8 bytes): [opcode][status][id][value f32 LE][sequence]
typedef enum {
  GUI_OP_PING = 0x00,
  GUI_OP_GET_PARAM = 0x01,
  GUI_OP_SET_PARAM = 0x02,
  GUI_OP_FAULT_INJECT = 0x05,
  GUI_OP_FAULT_CLEAR = 0x06,
} GUI_Opcode;

typedef enum {
  GUI_STATUS_OK = 0,
  GUI_STATUS_ERR_STATE = 1,   // command requires VCU_IDLE (or bench mode)
  GUI_STATUS_ERR_UNKNOWN = 2, // unknown param/fault id
  GUI_STATUS_ERR_RANGE = 3,   // value outside param min/max
} GUI_Status;

// Param IDs, matching the fields of VCU_Params in motor_control.h. Only
// plausibility/calibration constants -- no torque-shaping constant is here.
typedef enum {
  GUI_PARAM_APPS1_ADC_MAX_VAL = 0,
  GUI_PARAM_APPS1_ADC_MIN_VAL = 1,
  GUI_PARAM_APPS2_ADC_MAX_VAL = 2,
  GUI_PARAM_APPS2_ADC_MIN_VAL = 3,
  GUI_PARAM_BSE_ADC_MAX_VAL = 4,
  GUI_PARAM_BSE_ADC_MIN_VAL = 5,
  GUI_PARAM_APPS_OVERSHOOT_BUFFER = 6,
  GUI_PARAM_APPS_IMPLAUSIBILITY_PERCENT = 7,
  GUI_PARAM_APPS_IMPLAUSIBILITY_TIMEOUT_MS = 8,
  GUI_PARAM_BSE_IMPLAUSIBILITY_TIMEOUT_MS = 9, // not yet wired to any check
  GUI_PARAM_BSE_ACTIVATED_ADC_THRESHOLD = 10,
  GUI_PARAM_CROSSCHECK_IMPLAUSIBILITY_PERCENT = 11,
  GUI_PARAM_CROSSCHECK_RESTORATION_APPS_PERCENT = 12,
  GUI_PARAM_COUNT = 13,
} GUI_ParamId;

// Fault IDs for GUI_OP_FAULT_INJECT / GUI_OP_FAULT_CLEAR (VCU_BENCH_MODE only).
// The "value" field carries the forced value where applicable (e.g. forced
// ADC counts, forced RPM); ignored for boolean-style faults.
typedef enum {
  GUI_FAULT_FORCE_APPS1_ADC = 0,
  GUI_FAULT_FORCE_APPS2_ADC = 1,
  GUI_FAULT_FORCE_BSE_ADC = 2,
  GUI_FAULT_FORCE_APPS1_INVALID = 3,
  GUI_FAULT_FORCE_APPS2_INVALID = 4,
  GUI_FAULT_FORCE_CROSSCHECK_IMPLAUSIBLE = 5,
  GUI_FAULT_FORCE_INVERTER_RPM = 6,
  GUI_FAULT_FORCE_INVERTER_VOLTAGE = 7,
  GUI_FAULT_COUNT = 8,
} GUI_FaultId;

#ifdef VCU_BENCH_MODE
typedef struct GUI_FaultOverrides {
  bool active[GUI_FAULT_COUNT];
  float value[GUI_FAULT_COUNT];
} GUI_FaultOverrides;
extern GUI_FaultOverrides gui_fault_overrides;
#endif

void configureGuiProtocolMessage();
void handleGuiCommand(uint16_t msg_id);

#endif /* INC_GUI_PROTOCOL_H_ */
