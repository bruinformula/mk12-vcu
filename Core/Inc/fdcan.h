/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.h
 * @brief   This file contains all the function prototypes for
 *          the fdcan.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN Private defines */

// TX ID'S
#define BMS_PRCHG_TX_ID 0x6B3
#define INVERTER_TORQUE_REQUEST_TX_ID 0x0C0
#define VCU_COOLING_CMD_TX_ID 0x501

// RX ID's
#define BMS_PRCHG_RX_ID 0x6B4
#define BMS_PRCHG_IGNORE_RX_ID 0x6B5
#define SHUTDOWN_POWER_LOST_RX_ID 0x6B6
#define BMS_TEMP_RX_ID 0x6B1
#define INVERTER_VOLTAGE_RX_ID 0x0A7
#define INVERTER_RPM_RX_ID 0x0A5
#define INVERTER_TEMP_RX_ID 0x0A0

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);

/* USER CODE BEGIN Prototypes */
extern FDCAN_RxHeaderTypeDef RxHeader1;
extern uint8_t RxData1[8];
extern FDCAN_RxHeaderTypeDef RxHeader2;
extern uint8_t RxData2[8];

void FDCAN1_Rx_Handler(void);
void FDCAN2_Rx_Handler(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
