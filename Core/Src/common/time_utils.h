/*
 * time_utils.h
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/common/time_utils.h
 */

#ifndef TIME_UTILS_H_
#define TIME_UTILS_H_



/**
 * Function implementing delay() function in milliseconds
 * - blocking function
 * - hardware specific
 * @param ms number of milliseconds to wait
 */
void _delay(unsigned long ms);

/**
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
unsigned long _micros(void);

#endif /* TIME_UTILS_H_ */
