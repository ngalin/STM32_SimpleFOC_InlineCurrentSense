/*
 * Sensor.hpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/common/base_classes/Sensor.h
 */


#ifndef SENSOR_HPP_
#define SENSOR_HPP_

/**
 *  Direction structure
 */
enum Direction{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
};


/**
 *  Pullup configuration structure
 */
enum Pullup{
    INTERN, //!< Use internal pullups
    EXTERN //!< Use external pullups
};

/**
 *  Sensor abstract class definition
 * Each sensor needs to have these functions implemented
 */
class Sensor{
    public:

        /** get current angle (rad) */
        virtual float getAngle()=0;
        /** get current angular velocity (rad/s)*/
        virtual float getVelocity();

        /**
         * returns 0 if it does need search for absolute zero
         * 0 - magnetic sensor (& encoder with index which is found)
         * 1 - encoder with index (with index not found yet)
         */
        virtual int needsSearch();

        /** NAT: set angle to zero:         */
        virtual void setAngle(float);

    private:
        // velocity calculation variables
        float angle_prev=0; //!< angle in previous velocity calculation step
        long velocity_calc_timestamp=0; //!< last velocity calculation timestamp
};



#endif /* SENSOR_HPP_ */
