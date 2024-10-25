/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_lpm_if.h"
#include "stm32_lpm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* TMPSVC */
  uint8_t               Tmpchar_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t				TMP_TIMER_ID;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
extern ADC_HandleTypeDef hadc1;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TMPTIMER_INTERVAL (5000000/CFG_TS_TICK_VAL)
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* TMPSVC */
static void Custom_Tmpchar_Update_Char(void);
static void Custom_Tmpchar_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void readTemperature(void);
static void TMP_TIMER_ISR(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* TMPSVC */
    case CUSTOM_STM_TMPCHAR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TMPCHAR_READ_EVT */

      /* USER CODE END CUSTOM_STM_TMPCHAR_READ_EVT */
      break;

    case CUSTOM_STM_TMPCHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TMPCHAR_NOTIFY_ENABLED_EVT */
    	UTIL_SEQ_SetTask(1<<CFG_TASK_READ_TEMPERATURE, CFG_SCH_PRIO_0);
      /* USER CODE END CUSTOM_STM_TMPCHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_TMPCHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TMPCHAR_NOTIFY_DISABLED_EVT */
    	UTIL_SEQ_PauseTask(1<<CFG_TASK_READ_TEMPERATURE);
      /* USER CODE END CUSTOM_STM_TMPCHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
    	UTIL_SEQ_SetTask(1<<CFG_TASK_READ_TEMPERATURE, CFG_SCH_PRIO_0);
    	HW_TS_Start(Custom_App_Context.TMP_TIMER_ID, TMPTIMER_INTERVAL);
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
    	UTIL_SEQ_PauseTask(1<<CFG_TASK_READ_TEMPERATURE);
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	UTIL_SEQ_RegTask(1<<CFG_TASK_READ_TEMPERATURE, UTIL_SEQ_RFU, readTemperature);
	HW_TS_Create(CFG_TIM_TMPSVC_ID_ISR, &(Custom_App_Context.TMP_TIMER_ID), hw_ts_SingleShot, TMP_TIMER_ISR);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

void readTemperature(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_Delay(5);
	uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
	UpdateCharData[0] = (uint8_t)((adc_value >> 8) & 0x0F);
	UpdateCharData[1] = (uint8_t)(adc_value & 0xFF);
	Custom_STM_App_Update_Char(CUSTOM_STM_TMPCHAR, (uint8_t *)UpdateCharData);
	APP_DBG_MSG("UpdateCharData[0]: %u", UpdateCharData[0]);
	APP_DBG_MSG("UpdateCharData[1]: %u", UpdateCharData[1]);
	HAL_ADC_Stop(&hadc1);
	HW_TS_Start(Custom_App_Context.TMP_TIMER_ID, TMPTIMER_INTERVAL);
	return;
}

void TMP_TIMER_ISR(void)
{
	HW_TS_Stop(Custom_App_Context.TMP_TIMER_ID);
	UTIL_SEQ_SetTask(1<<CFG_TASK_READ_TEMPERATURE, CFG_SCH_PRIO_0);
	return;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* TMPSVC */
__USED void Custom_Tmpchar_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Tmpchar_UC_1*/

  /* USER CODE END Tmpchar_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_TMPCHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Tmpchar_UC_Last*/

  /* USER CODE END Tmpchar_UC_Last*/
  return;
}

void Custom_Tmpchar_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Tmpchar_NS_1*/

  /* USER CODE END Tmpchar_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_TMPCHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Tmpchar_NS_Last*/

  /* USER CODE END Tmpchar_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
