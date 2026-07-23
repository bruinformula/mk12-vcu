/*
 * prchg.h
 *
 *  Created on: Jan 26, 2026
 *      Author: ishanchitale
 */

#ifndef INC_PRCHG_H_
#define INC_PRCHG_H_

#include "stdbool.h"
#include "fdcan.h"
#include "gpio.h"
#include "tim.h"
#include "vcu_state.h"

#define PRECHARGE_MODE_NORMAL 0
#define PRECHARGE_MODE_BYPASS 1
#define PRECHARGE_MODE PRECHARGE_MODE_NORMAL
#define PRECHARGE_TIMEOUT_MS 6000

typedef enum {
    PRECHARGE_IDLE = 0,
    PRECHARGE_WAITING,
    PRECHARGE_SUCCESS,
    PRECHARGE_FAILURE,
	PRECHARGE_TIMEOUT,
} PrechargeState;
extern volatile PrechargeState precharge_state;
extern volatile bool precharge_response_received;

void configurePrechargeMessage();
void sendPrechargeRequest();
void processPrechargeResponse();
void checkPrechargeStatus();

#endif /* INC_PRCHG_H_ */
