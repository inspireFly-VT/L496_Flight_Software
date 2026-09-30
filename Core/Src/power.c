/**
 * power.c - Power-mode control and test for the ChildQube flight board (STM32L496)
 *
 * Step 2: skeleton only. Prints why the board reset and its current power
 * settings, then shows a heartbeat. Power modes are added in later steps.
 */
#include "power.h"
#include <stdio.h>

/* Prints which reset flags are set (why the chip last restarted), then clears them. */
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

/* Returns a readable name for the clock currently driving the CPU. */
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
    uint32_t vos = (PWR->CR1 & PWR_CR1_VOS) >> PWR_CR1_VOS_Pos;   /* Voltage range: 1 = full speed, 2 = power saving */
    int lpr      = (PWR->CR1 & PWR_CR1_LPR) != 0;                 /* 1 = low-power regulator requested */
    int reglpf   = (PWR->SR2 & PWR_SR2_REGLPF) != 0;              /* 1 = low-power regulator actually running */

    printf("  CPU clock: %lu Hz from %s (bus clock %lu Hz)\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(), clock_source_name(),
           (unsigned long)HAL_RCC_GetHCLKFreq());
    printf("  Voltage range: %lu   Low-power regulator: requested=%d active=%d\r\n",
           (unsigned long)vos, lpr, reglpf);
}

void power_test_run(void)
{
    printf("\r\n===== POWER MODE TEST =====\r\n");
    print_reset_cause();
    power_print_state();
    printf("No power modes added yet. Heartbeat every 5 s:\r\n");

    /* If the watchdog were running, the board would reset within ~4 s.
     * A heartbeat past 5 s shows test mode correctly left it off. */
    uint32_t seconds = 0;
    while (1) {
        HAL_Delay(5000);
        seconds += 5;
        printf("  alive %lu s\r\n", (unsigned long)seconds);
    }
}
