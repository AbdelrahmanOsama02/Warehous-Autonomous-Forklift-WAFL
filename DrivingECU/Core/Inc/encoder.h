/*
 * encoder.h
 *
 *  Created on: Dec 12, 2023
 *      Author: Ahmed (Modified for a 64MHz system clock and 600 P/R encoder)
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

// Sampling interval in seconds; this should match your timer interrupt period.
#define DT 0.1f

// Pulses per revolution for your 600 P/R encoder
#define NO_PULSES 600

// Function prototypes
int16_t Encoder_get_speed(void);
int16_t Encoder_get_counter(void);

#endif /* INC_ENCODER_H_ */
