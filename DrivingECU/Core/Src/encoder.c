/*
 * encoder.c
 *
 *  Created on: Apr 4, 2025
 *      Author: PC
 */
/*
 * encoder.c
 *
 *  Created on: Dec 12, 2023
 *      Author: Ahmed (Modified for a 64MHz system clock and 600 P/R encoder)
 */

#include "main.h"
#include "encoder.h"

// Use volatile for variables modified in interrupt routines.
volatile int16_t my_speed = 0;   // Measured RPM (updated every DT seconds)
volatile int16_t counter = 0;    // Encoder pulse counter within one sampling period

// Return the last calculated RPM
int16_t Encoder_get_speed(void)
{
    return my_speed;
}

// Return the raw pulse count (for debugging if needed)
int16_t Encoder_get_counter(void)
{
    return counter;
}

// Timer interrupt callback: This function is called every DT seconds.
// In CubeMX, configure TIM2 (or another timer) to produce an update event every 0.1 s.
// For a 64 MHz timer clock, you can achieve a 0.1 s period by using:
//   Prescaler = 6399  (i.e., 64,000,000 / (6399+1) = 10,000 Hz)
//   Auto-Reload (ARR) = 999  (10,000 ticks per second -> 1000 ticks in 0.1 s)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Verify that this is the correct timer interrupt (e.g., TIM2)
    if (htim->Instance == TIM2)
    {
        // Calculate RPM:
        // (counter / NO_PULSES) gives revolutions in DT seconds.
        // Multiply by (60 / DT) to convert to RPM.
        // For DT = 0.1 and NO_PULSES = 600:
        //   RPM = (counter * 60.0f) / (600 * 0.1f)
        // With DT = 0.1, the formula simplifies: RPM = counter.
        my_speed = (int16_t)((counter * 60.0f) / (NO_PULSES * DT));

        // Reset the pulse counter for the next sampling period
        counter = 0;
    }
}

// External interrupt callback for handling encoder channel edges.
// Adjust the GPIO pin numbers to match your wiring.
// In this example, assume PA11 (Channel A) and PA12 (Channel B) are used.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Basic quadrature decoding using external interrupts
    if (GPIO_Pin == GPIO_PIN_11)  // Encoder Channel A
    {
        // Check the state of Channel B (assumed on PA12)
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET)
            counter++;  // Forward rotation
        else
            counter--;  // Reverse rotation
    }
    else if (GPIO_Pin == GPIO_PIN_12) // Encoder Channel B
    {
        // Check the state of Channel A (assumed on PA11)
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET)
            counter++;  // Forward rotation
        else
            counter--;  // Reverse rotation
    }
}


