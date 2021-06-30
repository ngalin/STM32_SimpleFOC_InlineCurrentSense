/*
 * SimpleFOC.hpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/SimpleFOC.h
 */

#ifndef SRC_SIMPLEFOC_HPP_
#define SRC_SIMPLEFOC_HPP_

/*!
 * @file SimpleFOC.h ported from "https://github.com/simplefoc/Arduino-FOC/blob/v2.1"
 *
 * @mainpage Simple Field Oriented Control BLDC motor control library
 *
 * @section intro_sec Introduction
 * This version only supports a single STM32 MCU development board at the moment. See README for the
 * required pin configuration to support SimpleFOCShield v2.0.2
 *
 * @section dependencies Supported Hardware
 *  - Motors
 *    - BLDC motors
 * - Drivers
 *    - BLDC drivers
 * - Position sensors
 *    - Encoders
 *    - Open-loop control
 * - Microcontrollers
 *    - STM32H743ZI2
 */


#include "BLDCMotor.hpp"
#include "sensors/Encoder.hpp"
#include "sensors/MagneticSensorSPI.hpp"
#include "motor_drivers/BLDCDriver3PWM.hpp"
#include "current_sense/InlineCurrentSense.hpp"




#endif /* SRC_SIMPLEFOC_HPP_ */
