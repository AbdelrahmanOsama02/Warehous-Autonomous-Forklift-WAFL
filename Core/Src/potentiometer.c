/*
 * potentiometer.c
 *
 *  Created on: Dec 13, 2023
 *      Author: Ahmed
 */

#include "main.h"
#include "potentiometer.h"

static uint16_t pot_raw_value = 0;
static uint8_t pot_percentage = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC2) {
        pot_raw_value = HAL_ADC_GetValue(hadc);
        pot_percentage = (uint8_t)((pot_raw_value * 100UL) / 4095);  // scale to 0–100%
    }
    // Optional: restart ADC if you're not in continuous mode
    HAL_ADC_Start_IT(hadc);
}

uint8_t Potentiometer_GetPercentage(void)
{
    return pot_percentage;
}

uint16_t Potentiometer_GetRawValue(void)
{
    return pot_raw_value;
}
uint16_t Potentiometer_GetAnglePerTurn(void)
{
    return (uint16_t)((pot_raw_value * 360UL) / 4095);  // One turn (0–360°)
}

uint16_t Potentiometer_GetTotalAngle(void)
{
    return (uint16_t)((pot_raw_value * 3600UL) / 4095);  // 10 turns (0–3600°)
}
