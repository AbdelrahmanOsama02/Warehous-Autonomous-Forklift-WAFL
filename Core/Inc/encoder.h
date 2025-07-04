#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>

int16_t Encoder_get_speed_one(void);
int16_t Encoder_get_speed_two(void);

int16_t Encoder_get_pos_one(void);
int16_t Encoder_get_pos_two(void);

int32_t Encoder_get_counter_one(void);
int32_t Encoder_get_counter_two(void);

#endif /* INC_ENCODER_H_ */
