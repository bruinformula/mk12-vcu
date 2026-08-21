/*
 * gui_protocol.c
 *
 * See gui_protocol.h for protocol layout and the safety framing (VCU_IDLE
 * gate on writes, VCU_BENCH_MODE gate on fault injection).
 */

#include "gui_protocol.h"
#include "motor_control.h"
#include "vcu_state.h"
#include <string.h>

static FDCAN_TxHeaderTypeDef GuiResp_TxHeader;

#ifdef VCU_BENCH_MODE
GUI_FaultOverrides gui_fault_overrides;
#endif

typedef struct GUI_ParamEntry {
  GUI_ParamId id;
  float *ptr;
  float min;
  float max;
} GUI_ParamEntry;

static GUI_ParamEntry gui_param_table[GUI_PARAM_COUNT] = {
  {GUI_PARAM_APPS1_ADC_MAX_VAL, &vcu_params.apps1_adc_max_val, 0.0f, 4095.0f},
  {GUI_PARAM_APPS1_ADC_MIN_VAL, &vcu_params.apps1_adc_min_val, 0.0f, 4095.0f},
  {GUI_PARAM_APPS2_ADC_MAX_VAL, &vcu_params.apps2_adc_max_val, 0.0f, 4095.0f},
  {GUI_PARAM_APPS2_ADC_MIN_VAL, &vcu_params.apps2_adc_min_val, 0.0f, 4095.0f},
  {GUI_PARAM_BSE_ADC_MAX_VAL, &vcu_params.bse_adc_max_val, 0.0f, 4095.0f},
  {GUI_PARAM_BSE_ADC_MIN_VAL, &vcu_params.bse_adc_min_val, 0.0f, 4095.0f},
  {GUI_PARAM_APPS_OVERSHOOT_BUFFER, &vcu_params.apps_overshoot_buffer, 0.0f, 1000.0f},
  {GUI_PARAM_APPS_IMPLAUSIBILITY_PERCENT, &vcu_params.apps_implausibility_percent, 0.0f, 1.0f},
  {GUI_PARAM_APPS_IMPLAUSIBILITY_TIMEOUT_MS, &vcu_params.apps_implausibility_timeout_ms, 0.0f, 5000.0f},
  {GUI_PARAM_BSE_IMPLAUSIBILITY_TIMEOUT_MS, &vcu_params.bse_implausibility_timeout_ms, 0.0f, 5000.0f},
  {GUI_PARAM_BSE_ACTIVATED_ADC_THRESHOLD, &vcu_params.bse_activated_adc_threshold, 0.0f, 4095.0f},
  {GUI_PARAM_CROSSCHECK_IMPLAUSIBILITY_PERCENT, &vcu_params.crosscheck_implausibility_percent, 0.0f, 1.0f},
  {GUI_PARAM_CROSSCHECK_RESTORATION_APPS_PERCENT, &vcu_params.crosscheck_restoration_apps_percent, 0.0f, 1.0f},
};

void configureGuiProtocolMessage() {
  GuiResp_TxHeader.Identifier = GUI_RESP_TX_ID;
  GuiResp_TxHeader.IdType = FDCAN_STANDARD_ID;
  GuiResp_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  GuiResp_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  GuiResp_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  GuiResp_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  GuiResp_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  GuiResp_TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
  GuiResp_TxHeader.MessageMarker = 0;
}

static void sendGuiResponse(uint8_t opcode, uint8_t status, uint8_t id,
                             float value, uint8_t sequence) {
  uint8_t resp[8] = {0};
  resp[0] = opcode;
  resp[1] = status;
  resp[2] = id;
  memcpy(&resp[3], &value, sizeof(float));
  resp[7] = sequence;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0) {
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &GuiResp_TxHeader, resp);
  }
}

static GUI_ParamEntry *findParam(uint8_t id) {
  if (id >= GUI_PARAM_COUNT) return NULL;
  return &gui_param_table[id];
}

void handleGuiCommand(uint16_t msg_id) {
  if (msg_id != GUI_CMD_RX_ID) return;

  uint8_t opcode = RxData2[0];
  uint8_t id = RxData2[1];
  float value;
  memcpy(&value, &RxData2[2], sizeof(float));
  uint8_t sequence = RxData2[6];

  bool is_write = (opcode == GUI_OP_SET_PARAM) ||
                   (opcode == GUI_OP_FAULT_INJECT) || (opcode == GUI_OP_FAULT_CLEAR);
  if (is_write && vcu_state != VCU_IDLE) {
    sendGuiResponse(opcode, GUI_STATUS_ERR_STATE, id, 0.0f, sequence);
    return;
  }

  switch (opcode) {
  case GUI_OP_PING: {
#ifdef VCU_BENCH_MODE
    float bench_mode_flag = 1.0f;
#else
    float bench_mode_flag = 0.0f;
#endif
    sendGuiResponse(GUI_OP_PING, GUI_STATUS_OK, (uint8_t)vcu_state, bench_mode_flag, sequence);
    break;
  }

  case GUI_OP_GET_PARAM: {
    GUI_ParamEntry *entry = findParam(id);
    if (!entry) {
      sendGuiResponse(opcode, GUI_STATUS_ERR_UNKNOWN, id, 0.0f, sequence);
      break;
    }
    sendGuiResponse(opcode, GUI_STATUS_OK, id, *entry->ptr, sequence);
    break;
  }

  case GUI_OP_SET_PARAM: {
    GUI_ParamEntry *entry = findParam(id);
    if (!entry) {
      sendGuiResponse(opcode, GUI_STATUS_ERR_UNKNOWN, id, 0.0f, sequence);
      break;
    }
    if (value < entry->min || value > entry->max) {
      sendGuiResponse(opcode, GUI_STATUS_ERR_RANGE, id, *entry->ptr, sequence);
      break;
    }
    *entry->ptr = value;
    sendGuiResponse(opcode, GUI_STATUS_OK, id, *entry->ptr, sequence);
    break;
  }

  case GUI_OP_FAULT_INJECT:
  case GUI_OP_FAULT_CLEAR: {
#ifdef VCU_BENCH_MODE
    if (id >= GUI_FAULT_COUNT) {
      sendGuiResponse(opcode, GUI_STATUS_ERR_UNKNOWN, id, 0.0f, sequence);
      break;
    }
    gui_fault_overrides.active[id] = (opcode == GUI_OP_FAULT_INJECT);
    gui_fault_overrides.value[id] = value;
    sendGuiResponse(opcode, GUI_STATUS_OK, id, value, sequence);
#else
    // Fault injection is compiled out of non-bench builds. Never respond OK.
    sendGuiResponse(opcode, GUI_STATUS_ERR_STATE, id, 0.0f, sequence);
#endif
    break;
  }

  default:
    sendGuiResponse(opcode, GUI_STATUS_ERR_UNKNOWN, id, 0.0f, sequence);
    break;
  }
}
