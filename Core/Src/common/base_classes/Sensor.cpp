/*
 * Sensor.cpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/common/base_classes/Sensor.cpp
 */

#include "Sensor.hpp"

#include "../foc_utils.h"
#include "../time_utils.h"

 /**
 * returns 0 if it does need search for absolute zero
 * 0 - magnetic sensor (& encoder with index which is found)
 * 1 - encoder with index (with index not found yet)
 */
int Sensor::needsSearch(){
    return 0;
}

 /** get current angular velocity (rad/s)*/
float Sensor::getVelocity(){

    // calculate sample time
    unsigned long now_us = _micros();
    float Ts = (now_us - velocity_calc_timestamp)*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

    // current angle
    float angle_c = getAngle();
    // velocity calculation
    float vel = (angle_c - angle_prev)/Ts;

    // save variables for future pass
    angle_prev = angle_c;
    velocity_calc_timestamp = now_us;
    return vel;
}

void Sensor::setAngle(float angle_value) {
	angle_prev = angle_value;
}
