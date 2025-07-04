#ifndef INC_POTENTIOMETER_H_
#define INC_POTENTIOMETER_H_

#include <stdint.h>

#define PI 3.14159265359f
#define DEG_TO_RAD(deg) ((deg) * (PI / 180.0f))

uint8_t Potentiometer_GetPercentage(void);
uint16_t Potentiometer_GetRawValue(void);
uint16_t Potentiometer_GetAnglePerTurn(void);
uint16_t Potentiometer_GetTotalAngle(void);

#endif /* INC_POTENTIOMETER_H_ */
