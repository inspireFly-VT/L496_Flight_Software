/**
 * power.c - Power-mode control and test for the ChildQube flight board (STM32L496)
 *
 * Step 3: run modes. Switches between Range 1, Range 2 and low-power run,
 * and proves each switch by reading the chip's power and clock registers.
 */
#include "power.h"
#include <stdio.h>

#define HOLD_MS 5000u   /* Time spent in each mode so current can be measured */

extern UART_HandleTypeDef hlpuart1;      /* Serial port, set up in main.c */
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

/* Prints PASS/FAIL for one test and returns 1 if it passed */
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

void power_test_run(void)
{
    printf("\r\n===== POWER MODE TEST =====\r\n");
    print_reset_cause();

    int passed = 0, total = 0;
    passed += test_run_range1();     total++;
    passed += test_run_range2();     total++;
    passed += test_low_power_run();  total++;

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
