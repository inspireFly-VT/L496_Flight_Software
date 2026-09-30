/**
 * power.h - Power-mode control and test for the ChildQube flight board (STM32L496)
 */
#ifndef POWER_H
#define POWER_H

#include "main.h"

/* 1 = boot into the power-mode test instead of the flight software.
 * 0 = run the normal flight software (set back to 0 before merging to main). */
#define POWER_TEST 1

/* Runs the power-mode test. Never returns. */
void power_test_run(void);

/* Prints the current clock speed and power settings over serial. */
void power_print_state(void);

#endif /* POWER_H */
