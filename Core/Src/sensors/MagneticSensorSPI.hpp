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

//#define DEF_ANGLE_REGISTER 0x3FFF
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

//struct MagneticSensorSPIConfig_s  {
//  int spi_mode;
//  long clock_speed;
//  int bit_resolution;
//  int angle_register;
//  int data_start_bit;
//  int command_rw_bit;
//  int command_parity_bit;
//};
// typical configuration structures
//extern MagneticSensorSPIConfig_s AS5147_SPI,AS5048_SPI,AS5047_SPI, MA730_SPI;

class MagneticSensorSPI: public Sensor{
 public:
    /**
     *  MagneticSensorSPI class constructor
     * @param cs  SPI chip select pin
     * @param bit_resolution   sensor resolution bit number
     * @param angle_register  (optional) angle read register - default 0x3FFF
     */
    MagneticSensorSPI(int cs, float bit_resolution, int angle_register = 0);
    /**
     *  MagneticSensorSPI class constructor
     * @param config   SPI config
     * @param cs  SPI chip select pin
     */
    //MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs);

    /** sensor initialise pins */
//    void init(SPIClass* _spi = &SPI);
    void init(void);

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle() override;
    /** get current angular velocity (rad/s) **/
    float getVelocity() override;

    // returns the spi mode (phase/polarity of read/writes) i.e one of SPI_MODE0 | SPI_MODE1 | SPI_MODE2 | SPI_MODE3
   //int spi_mode;

    /* returns the speed of the SPI clock signal */
    //long clock_speed;


  private:
    float cpr; //!< Maximum range of the magnetic sensor
    // spi variables
    int angle_register = AS5048A_RESULT_MASK; //!< SPI angle register to read
    //int chip_select_pin; //!< SPI chip select pin
	//SPISettings settings; //!< SPI settings variable
    // spi functions
    /** Stop SPI communication */
    //void close();
    /** Read one SPI register value */
    uint16_t read(uint16_t angle_register);
    /** Calculate parity value  */
   // uint8_t spiCalcEvenParity(uint16_t value);

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

    //int bit_resolution; //!< the number of bites of angle data
    //int command_parity_bit; //!< the bit where parity flag is stored in command
    //int command_rw_bit; //!< the bit where read/write flag is stored in command
    //int data_start_bit; //!< the the position of first bit

    //SPIClass* spi;
};


#endif /* MAGNETICSENSORSPI_HPP_ */
