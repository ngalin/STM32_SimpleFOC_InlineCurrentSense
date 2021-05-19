/*
 * pid.hpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/common/pid.h
 */


#ifndef PID_HPP_
#define PID_HPP_



#include "time_utils.h"
#include "foc_utils.h"

/**
 *  PID controller class
 */
class PIDController
{
public:
    /**
     *
     * @param P - Proportional gain
     * @param I - Integral gain
     * @param D - Derivative gain
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator() (float error);

    float P; //!< Proportional gain
    float I; //!< Integral gain
    float D; //!< Derivative gain
    float output_ramp; //!< Maximum speed of change of the output value
    float limit; //!< Maximum output value

protected:
    float integral_prev; //!< last integral component value
    float error_prev; //!< last tracking error value
    unsigned long timestamp_prev; //!< Last execution timestamp
    float output_prev;  //!< last pid output value
    float Ts; //!<time interval in microseconds
};


#endif /* PID_HPP_ */
