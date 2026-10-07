/**
 * power.h - Power-mode control and test for the ChildQube flight board (STM32L496)
 */
#ifndef POWER_H
#define POWER_H

#include "main.h"

/* 1 = boot into the power-mode test instead of the flight software.
 * 0 = run the normal flight software (set back to 0 before merging to main). */
#define POWER_TEST 1

/* ---- Run modes: the CPU keeps running, at different speed/power levels ---- */

/* Normal flight clocks: 32 MHz CPU, full-strength voltage (Range 1). */
void power_set_run_range1(void);

/* 24 MHz CPU at reduced voltage (Range 2): same work, less power. */
void power_set_run_range2(void);

/* 2 MHz CPU on the low-power regulator: slowest running mode, least power. */
void power_set_low_power_run(void);

/* ---- Sleep modes: the CPU stops, peripherals keep running ---- */

/* Sleep for the given number of seconds; the RTC wake-up timer wakes the chip. */
void power_enter_sleep(uint32_t seconds);

/* Low-power sleep: same, but entered from low-power run on the low-power regulator.
 * Call power_set_low_power_run() first. */
void power_enter_low_power_sleep(uint32_t seconds);

/* ---- Stop modes: almost all clocks frozen, memory and registers kept ---- */

/* Enters Stop 0, 1 or 2 for the given number of seconds (RTC wake-up timer),
 * then restores the normal flight clocks. The program continues where it left off.
 * Stop 0 = fastest wake-up, most current. Stop 2 = lowest current (~1 uA range). */
void power_enter_stop(uint32_t level, uint32_t seconds);

/* ---- Test and diagnostics ---- */

/* Runs the power-mode test. Never returns. */
void power_test_run(void);

/* Prints the current clock speed and power settings over serial. */
void power_print_state(void);

#endif /* POWER_H */
