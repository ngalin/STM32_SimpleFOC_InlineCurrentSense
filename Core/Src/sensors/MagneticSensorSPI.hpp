/*
 * MagneticSensorSPI.hpp
 *
 *  Created on: 25 Jun 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/master/src/sensors/MagneticSensorSPI.h
 */


#ifndef MAGNETICSENSORSPI_HPP_
#define MAGNETICSENSORSPI_HPP_

#include "math.h"
#include "stdint.h"
#include "main.h"

#include "../common/base_classes/Sensor.hpp"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

//AS5048A datasheet: https://media.digikey.com/pdf/Data%20Sheets/Austriamicrosystems%20PDFs/AS5048A,B.pdf
#define AS5048A_CPR 16384
#define AS5048A_ANGLE_REG 0x3FFF
#define AS5048A_ERROR_REG 0x0001
#define AS5048A_PROGCTL_REG 0x0003
#define AS5048A_OTPHIGH_REG 0x0016
#define AS5048A_OTPLOW_REG 0x0017
#define AS5048A_DIAGNOSTICS_REG 0x3FFD
#define AS5048A_MAGNITUDE_REG 0x3FFE
#define AS5048A_PARITY 0x8000
#define AS5048A_RW 0x4000
#define AS5048A_ERRFLG 0x4000
#define AS5048A_RESULT_MASK 0x3FFF

class MagneticSensorSPI: public Sensor{
 public:
    /**
     *  MagneticSensorSPI class constructor
     * SPI chip select pin == D10 (PD14 on STM32H743)
     * sensor resolution bit number == 14 for AS5048A magnetic sensor
     * angle read register == 0x3FFF for AS5048A magnetic sensor,
     * basically, this is a dummy constructor - to remind myself that am using magnetic sensor and not encoder or some other position sensor
     */
    MagneticSensorSPI(void);

    void init(void);

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle() override;
    /** get current angular velocity (rad/s) **/
    float getVelocity() override;


  private:
    float cpr = AS5048A_CPR; //!< Maximum range of the magnetic sensor
    int angle_register = AS5048A_ANGLE_REG; //!< SPI angle register to read
      /** Read one SPI register value */
    uint16_t read(uint16_t angle_register);
    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset; //!<number of full rotations made
    float angle_data_prev; //!< angle in previous position calculation step

    // velocity calculation variables
    float angle_prev; //!< angle in previous velocity calculation step
    long velocity_calc_timestamp; //!< last velocity calculation timestamp
};


#endif /* MAGNETICSENSORSPI_HPP_ */
