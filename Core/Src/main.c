/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2s.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "motor_control.h"
#include "audio.h"
#include "prchg.h"
#include "cooling.h"
#include "vcu_state.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// RTD_SOUND_MODE == RTD_SOUND_NORMAL will attempt to play sound over I2S.
// RTD_SOUND_MODE == RTD_SOUND_OVERRIDE will assume successful speaker playback.
#define RTD_SOUND_NORMAL 0
#define RTD_SOUND_OVERRIDE 1
#define RTD_SOUND_MODE RTD_SOUND_NORMAL

#define INVERTER_LOCKOUT_TIME_MS 150
#define INVERTER_RATE_LIMIT_MS 10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t fdcan1_debug_cb = 0;
volatile uint32_t fdcan2_debug_cb = 0;

volatile uint32_t fdcan_rx_count = 0;          /* Counter for received messages */
volatile uint32_t fdcan_tx_count = 0;          /* Counter for transmitted messages */
volatile uint32_t fdcan_rx_error_count = 0;    /* RX error counter */

volatile uint8_t rtd_debug;
volatile uint8_t prchg_debug;

volatile uint32_t inverter_lockout_start;
volatile uint32_t inverter_tx_rate_limiter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	voltage_values[0] = (ADC_VAL[0]/4095.0)*3.3; // CHANNEL 0: APPS1 (0 - 1.65V)
	voltage_values[1] = (ADC_VAL[1]/4095.0)*3.3; // CHANNEL 6: APPS2 (0 - 2.24V)
	voltage_values[2] = (ADC_VAL[2]/4095.0)*3.3; // CHANNEL 7: BSE (0 - 2.24V)

	if (vcu_state != VCU_DRIVE) return;
	if (HAL_GetTick() - inverter_lockout_start < INVERTER_LOCKOUT_TIME_MS) return;

	pedal_percents[0] = ((float) ADC_VAL[0] - APPS1_ADC_MIN_VAL) / (APPS1_ADC_MAX_VAL - APPS1_ADC_MIN_VAL);
	pedal_percents[1] = ((float) ADC_VAL[1] - APPS2_ADC_MIN_VAL) / (APPS2_ADC_MAX_VAL - APPS2_ADC_MIN_VAL);
	pedal_percents[2] = ((float) ADC_VAL[2] - BSE_ADC_MIN_VAL) / (BSE_ADC_MAX_VAL - BSE_ADC_MIN_VAL);

	validateAPPS();
	checkAPPS_Plausibility();
	checkAPPS_BSE_Crosscheck();
	calculateTorqueRequest();

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0 &&
			HAL_GetTick() - inverter_tx_rate_limiter > INVERTER_RATE_LIMIT_MS) {
		sendTorqueRequest( (int)(requestedTorque*10), 0, 1);
		inverter_tx_rate_limiter = HAL_GetTick();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PRCHG_BTN_Pin) {
		prchg_debug++;
		if (HAL_GPIO_ReadPin(PRCHG_BTN_GPIO_Port, PRCHG_BTN_Pin) == GPIO_PIN_SET) {
			prchg_button_pressed = true;
		    __HAL_TIM_SET_COUNTER(&htim1, 0);
		    HAL_TIM_Base_Start_IT(&htim1);
		} else {
			// Button Released!
			prchg_button_pressed = false;
		    HAL_TIM_Base_Stop_IT(&htim1);
		}
	}

	if (GPIO_Pin == RTD_BTN_Pin) {
		rtd_debug++;
		if (HAL_GPIO_ReadPin(RTD_BTN_GPIO_Port, RTD_BTN_Pin) == GPIO_PIN_SET) {
			rtd_button_pressed = true;
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            HAL_TIM_Base_Start_IT(&htim2);
		} else {
			// Button Released!
			rtd_button_pressed = false;
            HAL_TIM_Base_Stop_IT(&htim2);
		}
	}
}

static bool sound_success;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
    	if (prchg_button_pressed == true) {
    		HAL_TIM_Base_Stop_IT(&htim1);
    		if (vcu_state != VCU_IDLE) return;
    		sendPrechargeRequest();
    	}
    }

    if (htim->Instance == TIM2) {
    	if (rtd_button_pressed == true) {
    		HAL_TIM_Base_Stop_IT(&htim2);
    		if (vcu_state != VCU_PRECHARGED) return;

    		if (ADC_VAL[2] > BSE_ACTIVATED_ADC_THRESHOLD) {

    			// ATTEMPT TO ENTER DRIVE MODE!

    			// NOTE: Works without disabling/re-enabling CAN?
    			// Most likely due to interrupt priority.
    			// I2S DMA Priority > CAN RX Priority.

    			HAL_ADC_Stop_DMA(&hadc3); // Disable pedals!
    			HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // Disable buttons!
    			rtd_button_pressed = false; // Clamp button state
    			prchg_button_pressed = false; // Clamp button state

#if RTD_SOUND_MODE == RTD_SOUND_NORMAL
    			sound_success = playReadyToDriveSound();
#elif RTD_SOUND_MODE == RTD_SOUND_OVERRIDE
    			sound_success = true;
#endif

    			if (!sound_success) {
        			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Re-enable buttons
        			HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_VAL, 3); // Re-enable pedals
        			return;
    			}

    			// INVERTER LOCKOUT FRAMES NECESSARY FOR START!
    			for (uint8_t lockout_frames_sent = 0;
    					lockout_frames_sent < 5 && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0;
    					++lockout_frames_sent) {
    				sendTorqueRequest(0, 0, 0);
    			}
    			inverter_lockout_start = HAL_GetTick();

    			vcu_state = VCU_DRIVE;
    			HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_VAL, 3);
    		}
    	}
    }

    if (htim->Instance == TIM3) {
    	HAL_TIM_Base_Stop_IT(&htim3);
    	checkPrechargeStatus();
    }
}

// FDCAN1 Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	fdcan1_debug_cb++;
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
	{

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, RxData1) != HAL_OK)
		{
			fdcan_rx_error_count++;
			return;
		}

		fdcan_rx_count++;
		FDCAN1_Rx_Handler();
	}
}

// FDCAN2 Callback
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	fdcan2_debug_cb++;
	if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
	{

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, RxData2) != HAL_OK)
		{
			fdcan_rx_error_count++;
			return;
		}

		fdcan_rx_count++;
		FDCAN2_Rx_Handler();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_I2S2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // FDCAN1 FILTER SETUP
  FDCAN_FilterTypeDef sFilterConfig_1 = {0};
  sFilterConfig_1.IdType = FDCAN_STANDARD_ID;
  sFilterConfig_1.FilterIndex = 0;
  sFilterConfig_1.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig_1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig_1.FilterID1 = 0x000;
  sFilterConfig_1.FilterID2 = 0x7FF;
  sFilterConfig_1.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig_1) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  // FDCAN2 FILTER SETUP
  FDCAN_FilterTypeDef sFilterConfig_2 = {0};
  sFilterConfig_2.IdType = FDCAN_STANDARD_ID;
  sFilterConfig_2.FilterIndex = 0;
  sFilterConfig_2.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig_2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig_2.FilterID1 = 0x000;
  sFilterConfig_2.FilterID2 = 0x7FF;
  sFilterConfig_2.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig_2) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  // Start FDCAN1
  if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
	  Error_Handler();
  }

  // Start FDCAN2
  if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK) {
	  Error_Handler();
  }

  // Activate the notification for new data in FIFO0 for FDCAN1
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
	  /* Notification Error */
	  Error_Handler();
  }

  // Activate the notification for new data in FIFO1 for FDCAN2
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
	  /* Notification Error */
	  Error_Handler();
  }

  configureInverterMessage();
  configureVCUDiagnosticsMessage();
  configureCoolingCmdMsg();

  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC_VAL, 3);

  // ISHAN CHITALE - BRUIN FORMULA RACING SOFTWARE LEAD, 2025-2026

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  sendCoolingCmd();
	  sendVCUDiagnostics();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	for (uint8_t lockout_frames_sent = 0;
			lockout_frames_sent < 5 && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0;
			++lockout_frames_sent) {
		sendTorqueRequest(0, 0, 0);
	}

  HAL_I2S_DMAStop(&hi2s2);
  HAL_ADC_Stop_DMA(&hadc3);
  __disable_irq();

  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
