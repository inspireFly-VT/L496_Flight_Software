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

/* ---- Test and diagnostics ---- */

/* Runs the power-mode test. Never returns. */
void power_test_run(void);

/* Prints the current clock speed and power settings over serial. */
void power_print_state(void);

#endif /* POWER_H */
