/*
 * defaults.h
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/common/defaults.h
 */

#ifndef DEFAULTS_H_
#define DEFAULTS_H_

// default configuration values
// change this file to optimal values for your application

#define DEF_POWER_SUPPLY 24.0 //!< default power supply voltage
// velocity PI controller params
#define DEF_PID_VEL_P 0.5 //!< default PID controller P value
#define DEF_PID_VEL_I 10.0 //!<  default PID controller I value
#define DEF_PID_VEL_D 0.0 //!<  default PID controller D value
#define DEF_PID_VEL_RAMP 1000.0 //!< default PID controller voltage ramp value
#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY) //!< default PID controller voltage limit

// current sensing PID values
// for stm32
#define DEF_PID_CURR_P 0.01 //NAT changed this, 3 //!< default PID controller P value
#define DEF_PID_CURR_I 1 //NAT changed this, 300.0 //!<  default PID controller I value
#define DEF_PID_CURR_D 0.0 //!<  default PID controller D value
#define DEF_PID_CURR_RAMP 50//NAT changed this, 1e11 //!< default PID controller voltage ramp value
#define DEF_PID_CURR_LIMIT 4//NAT changed this, (DEF_POWER_SUPPLY) //!< default PID controller voltage limit
#define DEF_CURR_FILTER_Tf 0.005 //!< default current filter time constant

// default current limit values
#define DEF_CURRENT_LIM 2 //!< 2Amps current limit by default

// default monitor downsample
#define DEF_MON_DOWNSMAPLE 100 //!< default monitor downsample
#define DEF_MOTION_DOWNSMAPLE 0 //!< default motion downsample - disable

// angle P params
#define DEF_P_ANGLE_P 20.0 //!< default P controller P value
#define DEF_VEL_LIM 20.0 //!< angle velocity limit default

// index search
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1.0 //!< default index search velocity
// align voltage
#define DEF_VOLTAGE_SENSOR_ALIGN 3.0 //!< default voltage for sensor and motor zero alignment
// low pass filter velocity
#define DEF_VEL_FILTER_Tf 0.005 //!< default velocity filter time constant



#endif /* DEFAULTS_H_ */
