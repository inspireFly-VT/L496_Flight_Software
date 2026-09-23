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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    float    battery_voltage;
    float    cpu_temperature;
    float    current_draw;
    float    charge_current;
    uint32_t uptime_sec;
} SOH_t;

typedef enum {
    SOH_OK = 0,
    SOH_ERR_TEMP,
    SOH_ERR_VOLTAGE
} SOH_Status_t;

typedef enum {
    STATE_RISE,
    STATE_STARTUP,
    STATE_SOH_INITIAL,
    STATE_FIX_IT,
    STATE_CPU_HOT,
    STATE_LOW_POWER,
    STATE_RECEIVE_PAYLOAD,
    STATE_HANDSHAKE,
    STATE_COMM_MODE,
    STATE_CRASH,
    STATE_COUNT
} FlightState_t;

typedef FlightState_t (*StateHandler_t)(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Battery voltage limits
 * SOH_VBAT_LOW: If voltage hits this, go to low power mode.
 * The system will only check voltage and sleep.
 * SOH_VBAT_CUTOFF: The hardware battery protector cuts power here.
 * The MCU cannot stop this. This is just for reference.
 *
 * SOH_TEMP_HOT: Max heat limit for the CPU.
 * This is a temporary value for now.
 */
#define SOH_VBAT_LOW     3.3f
#define SOH_VBAT_CUTOFF  2.916f
#define SOH_TEMP_HOT     85.0f

/* 3-Hour Mission Settings */
#define MISSION_DURATION_SEC 10800
#define FAST_MODE_DURATION   900    /* 15 mins fast data collection */
#define PAYLOAD_MAX_SIZE     1024   /* Room for more data points */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static FlightState_t g_current_state = STATE_RISE;
static SOH_t         g_soh;

/*
 * Data storage area.
 * The child picks up data and saves it here.
 * When the parent asks, the child sends this data.
 * The size might change later when we know the sensor count.
 */
static uint8_t  g_payload_buf[PAYLOAD_MAX_SIZE];
static uint16_t g_payload_len = 0;

volatile uint8_t g_sleep_done = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

static FlightState_t state_rise(void);
static FlightState_t state_startup(void);
static FlightState_t state_soh_initial(void);
static FlightState_t state_fix_it(void);
static FlightState_t state_cpu_hot(void);
static FlightState_t state_low_power(void);
static FlightState_t state_receive_payload(void);
static FlightState_t state_handshake(void);
static FlightState_t state_comm_mode(void);
static FlightState_t state_crash(void);

static void         getSOH(SOH_t *soh);
static void         gatherSOH(void);
static SOH_Status_t checkSOH(SOH_t *soh);
static void         collectData(void);
static void         writeToPayload(void);
static void         enter_stop_mode(void);
static void         enter_critical_stop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const StateHandler_t state_table[STATE_COUNT] = {
    [STATE_RISE]            = state_rise,
    [STATE_STARTUP]         = state_startup,
    [STATE_SOH_INITIAL]     = state_soh_initial,
    [STATE_FIX_IT]          = state_fix_it,
    [STATE_CPU_HOT]         = state_cpu_hot,
    [STATE_LOW_POWER]       = state_low_power,
    [STATE_RECEIVE_PAYLOAD] = state_receive_payload,
    [STATE_HANDSHAKE]       = state_handshake,
    [STATE_COMM_MODE]       = state_comm_mode,
    [STATE_CRASH]           = state_crash
};


static FlightState_t state_rise(void) {
    /* Wait for power to get steady. Sleep instead of just waiting. */
    printf("[STATE] RISE: Waiting for power...\r\n");
    for (int i = 0; i < 20; i++) {
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(50);
    }
    return STATE_STARTUP;
}

static FlightState_t state_startup(void) {
    printf("[STATE] STARTUP: Starting parts...\r\n");
    /* Place for part setup and hello signal to parent */
    return STATE_SOH_INITIAL;
}

static FlightState_t state_soh_initial(void) {
    printf("[STATE] SOH_INITIAL: Checking system health...\r\n");

    getSOH(&g_soh);   /* Read battery and sensors */
    gatherSOH();      /* Sort the read values */

    if (checkSOH(&g_soh) != SOH_OK) {
        return STATE_FIX_IT;
    }

    return STATE_RECEIVE_PAYLOAD;
}

static FlightState_t state_fix_it(void) {
    printf("[STATE] FIX_IT: Finding the error...\r\n");

    SOH_Status_t status = checkSOH(&g_soh);

    if (status == SOH_ERR_TEMP) {
        return STATE_CPU_HOT;
    }
    else if (status == SOH_ERR_VOLTAGE) {
        return STATE_LOW_POWER;
    }

    return STATE_SOH_INITIAL;
}

static FlightState_t state_cpu_hot(void) {
    /* CPU is too hot. Sleep to let it cool down. */
    printf("[STATE] CPU_HOT: Cooling down...\r\n");

    while (1) {
        HAL_IWDG_Refresh(&hiwdg);
        enter_stop_mode();

        getSOH(&g_soh); /* Check heat again */
        if (g_soh.cpu_temperature <= SOH_TEMP_HOT) {
            printf("[STATE] CPU_HOT: Cool enough now...\r\n");
            return STATE_SOH_INITIAL;
        }
    }
}

static FlightState_t state_low_power(void) {
    /* Battery is low. Only check battery and sleep. */
    printf("[STATE] LOW_POWER: Battery low, stopping tasks");

    while (1) {
        HAL_IWDG_Refresh(&hiwdg);
        enter_critical_stop(); /* Deep sleep to save power */

        getSOH(&g_soh); /* Check battery voltage */

        if (g_soh.battery_voltage > SOH_VBAT_LOW) {
            printf("[STATE] LOW_POWER: Battery okay now...\r\n");
            return STATE_SOH_INITIAL;
        }
    }
}

static FlightState_t state_receive_payload(void) {
    /* Get data and save it. Child does this on its own time. */
    printf("[STATE] RECEIVE_PAYLOAD: Saving data...\r\n");
    collectData();
    writeToPayload();

    /* 3-Hour Mission: Skip sleep for the first 15 mins to get more data. */
    if (g_soh.uptime_sec < FAST_MODE_DURATION) {
        HAL_Delay(100);
        return STATE_SOH_INITIAL;
    }

    /* Sleep until parent talks to us */
    enter_stop_mode();

    return STATE_HANDSHAKE;
}

static FlightState_t state_handshake(void) {
    /* Wait for parent signal. Sleep while waiting. */
    printf("[STATE] HANDSHAKE: Looking for Parent...\r\n");
    enter_stop_mode();
    return STATE_COMM_MODE;
}

static FlightState_t state_comm_mode(void) {
    /* Send the saved data to parent and clear the memory. */
    printf("[STATE] COMM_MODE: Sending data to Parent (%d bytes)...\r\n", g_payload_len);
    g_payload_len = 0;
    enter_stop_mode();
    return STATE_SOH_INITIAL;
}

static FlightState_t state_crash(void) {
    /* Stop here if something goes very wrong. */
	while (1) {
	        HAL_IWDG_Refresh(&hiwdg);
	    }
	    return STATE_CRASH;
}

/* --- Helper Code --- */

static void getSOH(SOH_t *soh) {
    /* Will read battery and heat sensors later. */
    soh->uptime_sec = HAL_GetTick() / 1000;
}

static void gatherSOH(void) {
    /* Check sensor data once parts are picked. */
}

static SOH_Status_t checkSOH(SOH_t *soh) {
	if (soh->cpu_temperature > SOH_TEMP_HOT)  return SOH_ERR_TEMP;

		/* Check for sensor errors. */
	    if (soh->battery_voltage < SOH_VBAT_LOW && soh->battery_voltage > 0.5f) {
	        return SOH_ERR_VOLTAGE;
	    }

	    return SOH_OK;
}

static void collectData(void) {
    /* Get data from sensors. */
}

static void writeToPayload(void) {
    /* Put sensor data into the storage area if there is room. */
    if (g_payload_len + sizeof(SOH_t) < PAYLOAD_MAX_SIZE) {
        memcpy(&g_payload_buf[g_payload_len], &g_soh, sizeof(SOH_t));
        g_payload_len += sizeof(SOH_t);
    }
}

static void enter_stop_mode(void) {
    /* Sleep mode. Wake up on the next timer tick. */
    g_sleep_done = 0;
    while (!g_sleep_done) {
        HAL_IWDG_Refresh(&hiwdg);
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    }
    SystemClock_Config();
}

static void enter_critical_stop(void) {
    /* Deep sleep for low battery. Can be made stronger later. */
    g_sleep_done = 0;
    while (!g_sleep_done) {
        HAL_IWDG_Refresh(&hiwdg);
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    }
    SystemClock_Config();
}

/* Every printf() ends up here; we send the text out two ways so we can always read it */
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)ptr, (uint16_t)len, 100);   /* Send over USB to a serial terminal; works without the debugger */
    for (int i = 0; i < len; i++) {
        ITM_SendChar(ptr[i]);                                           /* Also show in CubeIDE's debug console when debugging */
    }
    return len;                                                         /* Tell printf all the text was sent */
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

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();
  MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
  g_current_state = STATE_RISE;
  printf("\r\n[BOOT] Flight Software Online\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    HAL_IWDG_Refresh(&hiwdg);

    if (g_current_state >= STATE_COUNT)
        g_current_state = STATE_CRASH;

    g_current_state = state_table[g_current_state]();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief IWDG Initialization Function
  */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 1000;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  */
static void MX_LPUART1_UART_Init(void)
{
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;                /* Serial speed most terminals use by default (was 209700) */
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;  /* Send full 8-bit characters so text isn't garbled (was 7) */
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x1;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USB_OTG_FS Initialization Function
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
    /* Timer alarm went off. Wake the MCU from sleep. */
    g_sleep_done = 1;

    /* Set the next alarm for 1 second later. */
    RTC_AlarmTypeDef sAlarm;
    HAL_RTC_GetAlarm(hrtc, &sAlarm, RTC_ALARM_A, FORMAT_BIN);
    if (sAlarm.AlarmTime.Seconds > 58) {
        sAlarm.AlarmTime.Seconds = 0;
    } else {
        sAlarm.AlarmTime.Seconds = sAlarm.AlarmTime.Seconds + 1;
    }
    while (HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN) != HAL_OK) {}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
