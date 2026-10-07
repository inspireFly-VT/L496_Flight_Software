/**
 * power.c - Power-mode control and test for the ChildQube flight board (STM32L496)
 *
 * Step 3: run modes (Range 1, Range 2, low-power run), proven by register reads.
 * Step 4: sleep and low-power sleep, proven by a timer that only freezes
 *         while the CPU sleeps, compared against the real-time clock.
 * Step 5: Stop 0, 1 and 2, proven by the millisecond tick freezing while the
 *         RTC keeps counting, the chip waking on MSI, and the Stop level register.
 */
#include "power.h"
#include <stdio.h>

#define HOLD_MS 5000u   /* Time spent in each mode so current can be measured */

extern UART_HandleTypeDef hlpuart1;      /* Serial port, set up in main.c */
extern RTC_HandleTypeDef hrtc;           /* Real-time clock, shared with main.c */
extern void SystemClock_Config(void);    /* Normal flight clock setup, in main.c */

/* ============================== Serial helpers ============================== */

/* Waits until the last character has fully left the serial port.
 * Must be called before changing clocks, or the end of a message gets garbled. */
static void serial_flush(void)
{
    while (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TC) == RESET) {}
}

/* Recalculates the serial baud rate for the new clock speed.
 * Must be called after changing clocks, or all output after it is garbage. */
static void serial_reinit(void)
{
    if (HAL_UART_Init(&hlpuart1) != HAL_OK) Error_Handler();
}

/* ============================== Register reads ============================== */

static uint32_t voltage_range(void)     /* 1 = full speed allowed, 2 = power-saving */
{
    return (PWR->CR1 & PWR_CR1_VOS) >> PWR_CR1_VOS_Pos;
}

static int voltage_settling(void)       /* 1 = voltage still changing to the new range */
{
    return (PWR->SR2 & PWR_SR2_VOSF) != 0;
}

static int lp_regulator_active(void)    /* 1 = low-power regulator is actually running */
{
    return (PWR->SR2 & PWR_SR2_REGLPF) != 0;
}

static const char *clock_source_name(void)
{
    switch (__HAL_RCC_GET_SYSCLK_SOURCE()) {
        case RCC_SYSCLKSOURCE_STATUS_MSI:    return "MSI";
        case RCC_SYSCLKSOURCE_STATUS_HSI:    return "HSI16";
        case RCC_SYSCLKSOURCE_STATUS_HSE:    return "HSE";
        case RCC_SYSCLKSOURCE_STATUS_PLLCLK: return "PLL";
        default:                             return "unknown";
    }
}

void power_print_state(void)
{
    printf("  CPU clock: %lu Hz from %s (bus clock %lu Hz)\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(), clock_source_name(),
           (unsigned long)HAL_RCC_GetHCLKFreq());
    printf("  Voltage range: %lu   Low-power regulator: requested=%d active=%d\r\n",
           (unsigned long)voltage_range(), (PWR->CR1 & PWR_CR1_LPR) != 0,
           lp_regulator_active());
}

/* ============================== Clock switching ============================= */

/* Runs the CPU straight from the MSI oscillator at the given speed, with the PLL off.
 * Order matters: move off the PLL first, then turn it off, then change MSI speed. */
static void clock_from_msi(uint32_t msi_range, uint32_t flash_latency)
{
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;   /* CPU runs directly from MSI */
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;        /* No dividers: bus = CPU speed */
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    osc.PLL.PLLState   = RCC_PLL_OFF;            /* PLL no longer needed: turn it off */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    osc.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState            = RCC_MSI_ON;
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    osc.MSIClockRange       = msi_range;         /* New CPU speed */
    osc.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    /* Flash wait states needed for this speed (slower CPU needs fewer) */
    if (HAL_RCC_ClockConfig(&clk, flash_latency) != HAL_OK) Error_Handler();
}

void power_set_run_range1(void)
{
    if (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
        return;                                  /* Already on normal flight clocks */
    }
    serial_flush();
    if (PWR->CR1 & PWR_CR1_LPR) {
        HAL_PWREx_DisableLowPowerRunMode();      /* Back to the main regulator first */
    }
    SystemClock_Config();                        /* Raises voltage to Range 1, then 32 MHz PLL */
    serial_reinit();
}

void power_set_run_range2(void)
{
    serial_flush();
    if (PWR->CR1 & PWR_CR1_LPR) {
        HAL_PWREx_DisableLowPowerRunMode();
    }
    clock_from_msi(RCC_MSIRANGE_9, FLASH_LATENCY_3);   /* 24 MHz; 3 wait states needed in Range 2 */
    /* Lower the voltage only AFTER slowing down: Range 2 can't support over 26 MHz */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) Error_Handler();
    serial_reinit();
}

void power_set_low_power_run(void)
{
    serial_flush();
    clock_from_msi(RCC_MSIRANGE_5, FLASH_LATENCY_0);   /* 2 MHz: the most low-power run allows */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) Error_Handler();
    HAL_PWREx_EnableLowPowerRunMode();                 /* Hand power over to the low-power regulator */

    uint32_t start = HAL_GetTick();                    /* Wait for the switch-over to finish */
    while (!lp_regulator_active() && (HAL_GetTick() - start) < 100u) {}
    serial_reinit();
}

/* ============================ Real-time clock =============================== */
/* The RTC runs from the LSI oscillator (32 kHz) and keeps counting in every
 * low-power mode, so it is the reference we compare everything against. */

#define LSI_HZ      32000u
#define WAKE_HZ     (LSI_HZ / 16u)      /* Wake-up timer counts at RTC clock / 16 = 2000 per second */

/* Sets up the RTC for test mode (the flight software's MX_RTC_Init is skipped). */
static void rtc_init(void)
{
    hrtc.Instance            = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = 127;     /* 32000 / 128 / 250 = exactly 1 tick per second on LSI */
    hrtc.Init.SynchPrediv    = 249;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) Error_Handler();   /* Also selects LSI as the RTC clock */
}

/* Current RTC time in milliseconds since midnight. */
static uint32_t rtc_ms(void)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);       /* Must follow GetTime to unlock the registers */
    uint32_t frac = ((t.SecondFraction - t.SubSeconds) * 1000u) / (t.SecondFraction + 1u);
    return (t.Hours * 3600u + t.Minutes * 60u + t.Seconds) * 1000u + frac;
}

static uint32_t rtc_ms_since(uint32_t start)
{
    uint32_t now = rtc_ms();
    return (now >= start) ? (now - start) : (now + 86400000u - start);   /* Handles midnight rollover */
}

/* Starts the wake-up timer. Its event wakes the CPU without needing an interrupt
 * handler, because we sleep with WFE and "send event on pending" turned on. */
static void wake_timer_start(uint32_t seconds)
{
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds * WAKE_HZ - 1u,
                                    RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK) Error_Handler();
}

static int wake_timer_fired(void)
{
    return __HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTF) != 0;
}

static void wake_timer_stop(void)
{
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
}

/* ============================ Sleep proof timer ============================= */
/* TIM2 is set so its clock switches off ONLY while the CPU is in Sleep.
 * If TIM2 barely moves while the RTC counts 5 s, the CPU really slept. */

#define TIM2_HZ 10000u                  /* TIM2 counts 10,000 per second (0.1 ms per count) */

static void tim2_start(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_SLEEP_DISABLE();               /* Key line: no TIM2 clock during Sleep */

    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) tim_clk *= 2u;   /* Timers run at 2x bus speed when the bus is divided */

    TIM2->CR1 = 0;
    TIM2->PSC = tim_clk / TIM2_HZ - 1u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;                            /* Load the prescaler now */
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;
}

static void tim2_stop(void)
{
    TIM2->CR1 = 0;
    __HAL_RCC_TIM2_CLK_SLEEP_ENABLE();                 /* Restore the default */
    __HAL_RCC_TIM2_CLK_DISABLE();
}

/* =============================== Sleep modes ================================ */

/* Shared by both sleep modes. Returns the TIM2 count and RTC time spent asleep. */
static void sleep_for(uint32_t seconds, uint32_t regulator,
                      uint32_t *tim2_ms, uint32_t *rtc_ms_out)
{
    serial_flush();                     /* Let the last message finish before sleeping */
    tim2_start();
    wake_timer_start(seconds);
    HAL_SuspendTick();                  /* Otherwise the 1 ms tick wakes the CPU every millisecond */

    uint32_t rtc_start = rtc_ms();
    uint32_t tim_start = TIM2->CNT;

    do {
        HAL_PWR_EnterSLEEPMode(regulator, PWR_SLEEPENTRY_WFE);   /* CPU stops here until an event */
    } while (!wake_timer_fired());      /* Ignore any other event and go back to sleep */

    uint32_t tim_end = TIM2->CNT;
    HAL_ResumeTick();

    *tim2_ms    = (tim_end - tim_start) / (TIM2_HZ / 1000u);
    *rtc_ms_out = rtc_ms_since(rtc_start);
    wake_timer_stop();
    tim2_stop();
}

static uint32_t g_last_tim2_ms, g_last_rtc_ms;     /* Evidence from the most recent sleep */

void power_enter_sleep(uint32_t seconds)
{
    sleep_for(seconds, PWR_MAINREGULATOR_ON, &g_last_tim2_ms, &g_last_rtc_ms);
}

void power_enter_low_power_sleep(uint32_t seconds)
{
    sleep_for(seconds, PWR_LOWPOWERREGULATOR_ON, &g_last_tim2_ms, &g_last_rtc_ms);
}

/* ================================ Stop modes ================================ */

/* Re-reads the RTC after Stop: its readable copy of the time is stale on wake-up. */
static void rtc_resync(void)
{
    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    HAL_RTC_WaitForSynchro(&hrtc);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
}

/* Evidence captured from the most recent Stop, before anything changes it */
static uint32_t g_stop_tick_ms;      /* How far the 1 ms tick moved (should be ~0) */
static uint32_t g_stop_rtc_ms;       /* How far the RTC moved (should be ~5000) */
static uint32_t g_stop_wake_clock;   /* Clock running right after wake (should be MSI) */
static uint32_t g_stop_lpms;         /* Stop level recorded in PWR_CR1 (0, 1 or 2) */

void power_enter_stop(uint32_t level, uint32_t seconds)
{
    serial_flush();                     /* Let the last message finish before clocks stop */
    wake_timer_start(seconds);

    uint32_t rtc_start  = rtc_ms();
    uint32_t tick_start = HAL_GetTick();

    do {
        switch (level) {                /* CPU and almost all clocks stop here until the RTC wakes us */
            case 0:  HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFE); break;
            case 1:  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFE); break;
            default: HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE); break;
        }
    } while (!wake_timer_fired());      /* Ignore any other event and stop again */

    /* Capture evidence first: restoring the clocks would overwrite it */
    g_stop_wake_clock = __HAL_RCC_GET_SYSCLK_SOURCE();
    g_stop_tick_ms    = HAL_GetTick() - tick_start;
    g_stop_lpms       = (PWR->CR1 & PWR_CR1_LPMS) >> PWR_CR1_LPMS_Pos;

    power_set_run_range1();             /* Chip wakes on slow MSI: bring back the 32 MHz PLL */
    rtc_resync();
    g_stop_rtc_ms = rtc_ms_since(rtc_start);
    wake_timer_stop();
}

/* ================================== Tests =================================== */

static void print_reset_cause(void)
{
    uint32_t csr = RCC->CSR;                                   /* Reset flags live in this register */
    int woke_from_standby = __HAL_PWR_GET_FLAG(PWR_FLAG_SB);   /* Set only when waking from Standby */

    printf("Reset cause:");
    if (woke_from_standby)          printf(" STANDBY-WAKE");
    if (csr & RCC_CSR_LPWRRSTF)     printf(" LOW-POWER");
    if (csr & RCC_CSR_WWDGRSTF)     printf(" WINDOW-WATCHDOG");
    if (csr & RCC_CSR_IWDGRSTF)     printf(" WATCHDOG");
    if (csr & RCC_CSR_SFTRSTF)      printf(" SOFTWARE");
    if (csr & RCC_CSR_BORRSTF)      printf(" POWER-ON/BROWNOUT");
    if (csr & RCC_CSR_PINRSTF)      printf(" RESET-PIN");
    if (csr & RCC_CSR_OBLRSTF)      printf(" OPTION-BYTES");
    if (csr & RCC_CSR_FWRSTF)       printf(" FIREWALL");
    printf("\r\n");

    __HAL_RCC_CLEAR_RESET_FLAGS();                             /* Clear so the next boot reports fresh flags */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
}

/* Prints Pass or Fail for one test and returns 1 if it passed */
static int report(const char *name, int pass)
{
    printf("  [%s] %s\r\n\r\n", pass ? "PASS" : "FAIL", name);
    return pass;
}

static void hold(void)
{
    printf("  Holding %lu s - measure current now\r\n", (unsigned long)(HOLD_MS / 1000u));
    HAL_Delay(HOLD_MS);
}

static int test_run_range1(void)
{
    printf("-- Run, Range 1 (normal flight clocks)\r\n");
    power_set_run_range1();
    power_print_state();
    hold();
    /* Proof: voltage register reads Range 1 and it has finished settling */
    return report("Run Range 1", voltage_range() == 1 && !voltage_settling() &&
                                 __HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK);
}

static int test_run_range2(void)
{
    printf("-- Run, Range 2 (reduced voltage, 24 MHz)\r\n");
    power_set_run_range2();
    power_print_state();
    hold();
    /* Proof: voltage register reads Range 2 and the CPU really runs at 24 MHz */
    return report("Run Range 2", voltage_range() == 2 && !voltage_settling() &&
                                 HAL_RCC_GetHCLKFreq() == 24000000u);
}

static int test_low_power_run(void)
{
    printf("-- Low-power run (2 MHz, low-power regulator)\r\n");
    power_set_low_power_run();
    power_print_state();
    hold();
    /* Proof: the chip's own status flag confirms the low-power regulator is running */
    int pass = lp_regulator_active() && voltage_range() == 2 &&
               HAL_RCC_GetHCLKFreq() <= 2000000u;
    return report("Low-power run", pass);
}

/* Sleep proof: RTC shows ~5 s passed, but TIM2 (no clock in Sleep) barely moved. */
static int sleep_evidence_ok(void)
{
    printf("  Asleep: RTC counted %lu ms, TIM2 (stops in Sleep) counted %lu ms\r\n",
           (unsigned long)g_last_rtc_ms, (unsigned long)g_last_tim2_ms);
    int rtc_ok = g_last_rtc_ms >= (HOLD_MS * 9u / 10u) && g_last_rtc_ms <= (HOLD_MS * 13u / 10u);
    int tim_ok = g_last_tim2_ms * 10u < g_last_rtc_ms;   /* Awake less than 10% of the time */
    return rtc_ok && tim_ok;
}

static int test_sleep(void)
{
    printf("-- Sleep (CPU stopped, main regulator)\r\n");
    power_set_run_range1();
    power_print_state();
    printf("  Sleeping %lu s - measure current now\r\n", (unsigned long)(HOLD_MS / 1000u));
    power_enter_sleep(HOLD_MS / 1000u);
    return report("Sleep", sleep_evidence_ok());
}

static int test_low_power_sleep(void)
{
    printf("-- Low-power sleep (CPU stopped, low-power regulator)\r\n");
    power_set_low_power_run();
    power_print_state();
    printf("  Sleeping %lu s - measure current now\r\n", (unsigned long)(HOLD_MS / 1000u));
    power_enter_low_power_sleep(HOLD_MS / 1000u);
    int lp_on = lp_regulator_active();                 /* Still on the low-power regulator after waking */
    printf("  Low-power regulator active after wake: %d\r\n", lp_on);
    return report("Low-power sleep", sleep_evidence_ok() && lp_on);
}

static int test_stop(uint32_t level)
{
    printf("-- Stop %lu\r\n", (unsigned long)level);
    power_set_run_range1();
    power_print_state();
    printf("  Stopping %lu s - measure current now\r\n", (unsigned long)(HOLD_MS / 1000u));
    power_enter_stop(level, HOLD_MS / 1000u);

    printf("  In Stop: RTC counted %lu ms, 1 ms tick counted %lu ms\r\n",
           (unsigned long)g_stop_rtc_ms, (unsigned long)g_stop_tick_ms);
    printf("  Woke on clock: %s   Stop level register (LPMS): %lu\r\n",
           g_stop_wake_clock == RCC_SYSCLKSOURCE_STATUS_MSI ? "MSI" : "not MSI",
           (unsigned long)g_stop_lpms);

    int rtc_ok   = g_stop_rtc_ms >= (HOLD_MS * 9u / 10u) && g_stop_rtc_ms <= (HOLD_MS * 13u / 10u);
    int tick_ok  = g_stop_tick_ms * 10u < g_stop_rtc_ms;            /* Tick frozen almost the whole time */
    int clock_ok = g_stop_wake_clock == RCC_SYSCLKSOURCE_STATUS_MSI; /* Stop always switches off the PLL */
    int level_ok = g_stop_lpms == level;

    char name[8] = "Stop 0";
    name[5] = (char)('0' + level);
    return report(name, rtc_ok && tick_ok && clock_ok && level_ok);
}

void power_test_run(void)
{
    printf("\r\n===== POWER MODE TEST =====\r\n");
    print_reset_cause();
    rtc_init();
    HAL_PWR_EnableSEVOnPend();       /* Let the RTC wake-up event wake the CPU from WFE */
    HAL_DBGMCU_DisableDBGSleepMode();/* A debugger must not keep clocks running in Sleep */
    HAL_DBGMCU_DisableDBGStopMode(); /* ...or in Stop */

    int passed = 0, total = 0;
    passed += test_run_range1();     total++;
    passed += test_run_range2();     total++;
    passed += test_low_power_run();  total++;
    passed += test_sleep();          total++;
    passed += test_low_power_sleep(); total++;
    passed += test_stop(0);          total++;
    passed += test_stop(1);          total++;
    passed += test_stop(2);          total++;

    power_set_run_range1();          /* Back to normal clocks */
    printf("===== %d/%d PASSED =====\r\n", passed, total);
    printf("Press RESET to run again. Heartbeat every 5 s:\r\n");

    uint32_t seconds = 0;
    while (1) {
        HAL_Delay(5000);
        seconds += 5;
        printf("  alive %lu s\r\n", (unsigned long)seconds);
    }
}
