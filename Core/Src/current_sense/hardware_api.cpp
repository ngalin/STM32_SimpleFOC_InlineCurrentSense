/*
 * hardware_api.cpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/current_sense/hardware_specific/generic_mcu.cpp
 */

#include "hardware_api.h"
#include "main.h"

#define _ADC_VOLTAGE 3.3
#define _ADC_RESOLUTION 65536.0

// adc counts to voltage conversion ratio
// some optimizing for faster execution
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

// function reading an ADC value and returning the read voltage
float _readADCVoltage(const int pin){
	int raw_adc;
	if (pin == phaseA) {
		raw_adc = _readADCVoltage_pinA();
	}
	if (pin == phaseB) {
		raw_adc = _readADCVoltage_pinB();
	}
  return raw_adc * _ADC_CONV;
}

float _ADCValueToVoltage(int raw_adc) {
	return raw_adc * _ADC_CONV;
}
//// function assigning input pins to ADC
//void _configureADC(const int pinA,const int pinB){
//	pinA = phaseA;
//	pinB = phaseB;
//}

