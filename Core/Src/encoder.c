/*
 * encoder.c
 *
 *  Created on: Apr 4, 2025
 *      Author: PC (modified by ChatGPT)
 */

#include "main.h"
#include "encoder.h"

// ==========================
// === Encoder Constants ===
// ==========================
#define NO_PULSES 600
#define DT 0.1f  // Sampling time in seconds

// =============================
// === Encoder 1 Variables  ===
// =============================
volatile int32_t counter_one = 0;          // Pulse count within sampling period
volatile int32_t total_counter_one = 0;    // Total pulse count since start
volatile int16_t my_speed_one = 0;         // Measured RPM
volatile int16_t angle_one = 0;            // Angle in degrees

// =============================
// === Encoder 2 Variables  ===
// =============================
volatile int32_t counter_two = 0;
volatile int32_t total_counter_two = 0;
volatile int16_t my_speed_two = 0;
volatile int16_t angle_two = 0;

// ===============================
// === Public Access Functions ===
// ===============================

int16_t Encoder_get_speed_one(void) {
    return my_speed_one;
}

int16_t Encoder_get_speed_two(void) {
    return my_speed_two;
}

int16_t Encoder_get_pos_one(void) {
    angle_one = (int16_t)((total_counter_one * 360.0f) / NO_PULSES);
    return angle_one;
}

int16_t Encoder_get_pos_two(void) {
    angle_two = (int16_t)((total_counter_two * 360.0f) / NO_PULSES);
    return angle_two;
}

int32_t Encoder_get_counter_one(void) {
    return counter_one;
}

int32_t Encoder_get_counter_two(void) {
    return counter_two;
}

// =======================================
// === Timer Period Elapsed Callback  ===
// =======================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2)  // Make sure this is the correct timer
    {
        my_speed_one = (int16_t)((counter_one * 60.0f) / (NO_PULSES * DT));
        my_speed_two = (int16_t)((counter_two * 60.0f) / (NO_PULSES * DT));

        counter_one = 0;
        counter_two = 0;
    }
}

// ===================================
// === External Interrupt Callback ===
// ===================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // === Encoder 1: PA11 (A1), PA12 (B1) ===
    if (GPIO_Pin == GPIO_PIN_11) {  // Channel A1
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET) {
            counter_one++;
            total_counter_one++;
        }
    } else if (GPIO_Pin == GPIO_PIN_12) {  // Channel B1
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
            counter_one--;
            total_counter_one--;
        }
    }

    // === Encoder 2: PB14 (A2), PB15 (B2) ===
    else if (GPIO_Pin == GPIO_PIN_15) {  // Channel A2
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
            counter_two--;
            total_counter_two--;
        }
    } else if (GPIO_Pin == GPIO_PIN_14) {  // Channel B2
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) {
            counter_two++;
            total_counter_two++;
        }
    }
}

