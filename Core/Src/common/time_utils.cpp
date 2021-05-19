/*
 * time_utils.cpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/common/time_utils.cpp
 */

#include "time_utils.h"
#include "stm32h7xx_hal.h"

// function buffering delay()
void _delay(unsigned long ms){
	HAL_Delay(ms);
}

// function buffering _micros()
unsigned long _micros(void){
	//return HAL_GetTick()*1000; //get microseconds
	return TIM2->CNT; //counter should be set to 1MHz
}

