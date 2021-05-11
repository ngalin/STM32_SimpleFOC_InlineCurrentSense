/*
 * hardware_api.h
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/current_sense/hardware_api.h
 */

#ifndef HARDWARE_API_H_
#define HARDWARE_API_H_

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

/**
 *  function reading an ADC value and returning the read voltage
 *
 * @param pinA - the arduino pin to be read (it has to be ADC pin)
 */
float _readADCVoltage(const int pinA);

/**
 * function converting ADC reading to voltage
 *
 */
float _ADCValueToVoltage(int raw_adc);

/**
 *  function assigning ADC pins
 *
 * @param pinA - adc pin A
 * @param pinB - adc pin B
 * @param pinC - adc pin C
 */
void _configureADC(const int pinA,const int pinB,const int pinC = NOT_SET);

#endif /* HARDWARE_API_H_ */
