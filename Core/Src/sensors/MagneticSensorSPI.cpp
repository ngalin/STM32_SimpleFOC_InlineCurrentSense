/*
 * MagneticSensorSPI.hpp
 *
 *  Created on: 25 Jun 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/master/src/sensors/MagneticSensorSPI.cpp
 */

#include "MagneticSensorSPI.hpp"

// MagneticSensorSPI() //dummy constructor - just to remind myself that am using magnetic sensor, and not encoder
MagneticSensorSPI::MagneticSensorSPI(void){
}

//void MagneticSensorSPI::init(SPIClass* _spi){
void MagneticSensorSPI::init(void) {
	// velocity calculation init
	angle_prev = 0;
	velocity_calc_timestamp = _micros();

	// full rotations tracking number
	full_rotation_offset = 0;
	angle_data_prev = getRawCount();
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSPI::getAngle(){
  // raw data from the sensor
  float angle_data = getRawCount();

  // tracking the number of rotations
  // in order to expand angle range form [0,2PI]
  // to basically infinity
  float d_angle = angle_data - angle_data_prev;
  // if overflow happened track it as full rotation
  if(abs(d_angle) > (0.8*cpr) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI;
  // save the current angle value for the next steps
  // in order to know if overflow happened
  angle_data_prev = angle_data;

  // return the full angle
  // (number of full rotations)*2PI + current sensor angle
  return full_rotation_offset + ( angle_data / (float)cpr) * _2PI;
}

// Shaft velocity calculation
float MagneticSensorSPI::getVelocity(){
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


// function reading the raw counter of the magnetic sensor
int MagneticSensorSPI::getRawCount(){
	return (int) MagneticSensorSPI::read(angle_register);
}

  /*
  * Read a register from the sensor
  * Takes the address of the register as a 16 bit word
  * Returns the value of the register
  */
uint16_t MagneticSensorSPI::read(uint16_t address_register){
	uint16_t command = address_register | AS5048A_PARITY | AS5048A_RW; // set r=1 and parity=1, result is 0xFFFF
	return _SPI_read(command)&AS5048A_RESULT_MASK;
}
