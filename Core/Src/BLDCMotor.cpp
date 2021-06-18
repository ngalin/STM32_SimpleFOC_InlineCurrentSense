/*
 * BLDCMotor.cpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/BLDCMotor.cpp
 */


#include "BLDCMotor.hpp"
#include "math.h"
#include "main.h"

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
BLDCMotor::BLDCMotor(int pp, float _R)
: FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // torque control type is voltage by default
  torque_controller = TorqueControlType::voltage;
  // anti-cogging request
  anti_cogging = false;
}


/**
	Link the driver which controls the motor
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
void BLDCMotor::init() {
//  if(monitor_port) monitor_port->println(F("MOT: Init"));

  // if no current sensing and the user has set the phase resistance of the motor use current limit to calculate the voltage limit
  if( !current_sense && _isset(phase_resistance)) {
    float new_voltage_limit = current_limit * (phase_resistance); // v_lim = current_lim / (3/2 phase resistance) - worst case
    // use it if it is less then voltage_limit set by the user
    voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
  }
  // sanity check for the voltage limit configuration
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // update the controller limits
  if(current_sense){
    // current control loop controls voltage
    PID_current_q.limit = voltage_limit;
    PID_current_d.limit = voltage_limit;
    // velocity control loop controls current
    PID_velocity.limit = current_limit;
  }else{
    PID_velocity.limit = voltage_limit;
  }
  P_angle.limit = velocity_limit;

  _delay(500);
  // enable motor
//  if(monitor_port) monitor_port->println(F("MOT: Enable driver."));
  enable();
  _delay(500);
}


// disable motor driver
void BLDCMotor::disable()
{
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void BLDCMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // motor status update
  enabled = 1;
}

/**
  FOC functions
*/
// FOC initialization function
int  BLDCMotor::initFOC( float zero_electric_offset, Direction _sensor_direction) {
  int exit_flag = 1;
  // align motor if necessary
  // alignment necessary for encoders!
  if(_isset(zero_electric_offset)){
    // absolute zero offset provided - no need to align
    zero_electric_angle = zero_electric_offset;
    // set the sensor direction - default CW
    sensor_direction = _sensor_direction;
  }

  // sensor and motor alignment - can be skipped
  // by setting motor.sensor_direction and motor.zero_electric_angle
  _delay(500);
  if(sensor) exit_flag *= alignSensor();
//  else if(monitor_port) monitor_port->println(F("MOT: No sensor."));

  // aligning the current sensor - can be skipped
  // checks if driver phases are the same as current sense phases
  // and checks the direction of measurement.
  _delay(500);
  if(exit_flag){
    if(current_sense) exit_flag *= alignCurrentSense();
//    else if(monitor_port) monitor_port->println(F("MOT: No current sense."));
  }

  _delay(500);
  if (exit_flag) {
	  if(anti_cogging) exit_flag *= antiCoggingCalibration();
  }

  if(exit_flag){
//    if(monitor_port) monitor_port->println(F("MOT: Ready."));
  }else{
//    if(monitor_port) monitor_port->println(F("MOT: Init FOC failed."));
    disable();
  }

  return exit_flag;
}

// Calibarthe the motor and current sense phases
int BLDCMotor::alignCurrentSense() {
  int exit_flag = 1; // success

//  if(monitor_port) monitor_port->println(F("MOT: Align current sense."));

  // align current sense and the driver
  exit_flag = current_sense->driverAlign(driver, voltage_sensor_align);
  if(!exit_flag){
    // error in current sense - phase either not measured or bad connection
//    if(monitor_port) monitor_port->println(F("MOT: Align error!"));
    exit_flag = 0;
  }else{
    // output the alignment status flag
//    if(monitor_port) monitor_port->print(F("MOT: Success: "));
//    if(monitor_port) monitor_port->println(exit_flag);
  }

  return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int BLDCMotor::alignSensor() {
  int exit_flag = 1; //success
//  if(monitor_port) monitor_port->println(F("MOT: Align sensor."));

  // if unknown natural direction
  if(!_isset(sensor_direction)){
    // check if sensor needs zero search
    if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
    // stop init if not found index
    if(!exit_flag) return exit_flag;

    // find natural direction
    // move one electrical revolution forward
    for (int i = 0; i <=500; i++ ) {
      float angle = _3PI_2 + _2PI * i / 500.0;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
      _delay(2);//FIXME - Nat changed from 2 to 2
    }
    // take and angle in the middle
    float mid_angle = sensor->getAngle();
    // move one electrical revolution backwards
    for (int i = 500; i >=0; i-- ) {
      float angle = _3PI_2 + _2PI * i / 500.0 ;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
      _delay(2);//FIXME - Nat changed from 2 to 2
    }
    float end_angle = sensor->getAngle();
    setPhaseVoltage(0, 0, 0);
    _delay(200);
    // determine the direction the sensor moved
    if (mid_angle == end_angle) {
//      if(monitor_port) monitor_port->println(F("MOT: Failed to notice movement"));
      return 0; // failed calibration
    } else if (mid_angle < end_angle) {
//      if(monitor_port) monitor_port->println(F("MOT: sensor_direction==CCW"));
      sensor_direction = Direction::CCW;
    } else{
//      if(monitor_port) monitor_port->println(F("MOT: sensor_direction==CW"));
      sensor_direction = Direction::CW;
    }
    // check pole pair number
//    if(monitor_port) monitor_port->print(F("MOT: PP check: "));
    float moved =  fabs(mid_angle - end_angle);
    est_pp = _2PI / moved;

    if( fabs(moved*pole_pairs - _2PI) > 0.5 ) { // 0.5 is arbitrary number it can be lower or higher!
//      if(monitor_port) monitor_port->print(F("fail - estimated pp:"));
//      if(monitor_port) monitor_port->println(_2PI/moved,4);
    }//else if(monitor_port) monitor_port->println(F("OK!"));

  }//else if(monitor_port) monitor_port->println(F("MOT: Skip dir calib."));

  // zero electric angle not known
  if(!_isset(zero_electric_angle)){
    // align the electrical phases of the motor and sensor
    // set angle -90(270 = 3PI/2) degrees
    setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);
    _delay(700);
    float tmp = sensor->getAngle();
    zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*tmp, pole_pairs));
    _delay(20);
//    if(monitor_port){
//      monitor_port->print(F("MOT: Zero elec. angle: "));
//      monitor_port->println(zero_electric_angle);
//    }
    // stop everything
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  }//else if(monitor_port) monitor_port->println(F("MOT: Skip offset calib."));
  return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int BLDCMotor::absoluteZeroSearch() {

//  if(monitor_port) monitor_port->println(F("MOT: Index search..."));
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5*_2PI);
    // call important for some sensors not to loose count
    // not needed for the search
   // sensor->getAngle(); //NG: may have been necessary in some instances to increase time of loop
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found
//  if(monitor_port){
//    if(sensor->needsSearch()) monitor_port->println(F("MOT: Error: Not found!"));
//    else monitor_port->println(F("MOT: Success!"));
//  }
  return !sensor->needsSearch();
}

// Anti-cogging algorithm implementation, see: http://www.roboticsproceedings.org/rss10/p42.pdf
int BLDCMotor::antiCoggingCalibration() {
	int exit_flag = true;
//	float limit_vel = velocity_limit;
//	float limit_volt = 2;//voltage_limit;
//	velocity_limit = velocity_index_search;
//	voltage_limit = 2;//voltage_sensor_align;
//
//	//need to put motor in open-loop position control:
//	MotionControlType requested_controller = controller;
//	FOCModulationType requested_modulation = foc_modulation;
//	TorqueControlType requested_torque_controller = torque_controller;
//
//	controller = MotionControlType::angle;
//	foc_modulation = FOCModulationType::SinePWM;
//	torque_controller = TorqueControlType::foc_current;
//
//	//set position control PID coefficients: //TODO - fix these...may not work with all motors?
//	//---save old values:
//	float prev_P_angle_P = P_angle.P;
//	float prev_P_angle_I = P_angle.I;
//	float prev_P_angle_D = P_angle.D;
//	float prev_P_angle_limit = P_angle.limit;
//
//	//---now set wanted values:
//	P_angle.P = 0.5;
//	P_angle.I = 0.0;
//	P_angle.D = 0;
//	P_angle.limit = 20;
//	LPF_angle.Tf = 0.001;

	float angle = 0;
	float angle_increment = _2PI / 3;//8192;
	int idx = 0;
	//calculate various parameters for each encoder position:
//    for (int angle = 0; angle <= _2PI; angle += angle_increment ) {
////      setPhaseVoltage(voltage_sensor_align, 0,  angle);
////      current = current_sense->getFOCCurrents(electrical_angle);
////      float Iq = LPF_current_q(current.q);
//    	target = angle;
//    	move();
//    	for (int i = 0; i < 1000; i++) {
//    		loopFOC();
//    	//	_delay(1);
//    	}
//      _delay(2000);
//    }

//	for (angle = 0; angle <= _2PI; angle += angle_increment) {
//		move(angle);
//		loopFOC();
//
//		float prev_Iq = LPF_current_q(current.q);
//		float diff_Iq = 10;//some initially large value
//		//TODO - need to implement say a filter - whereby 3 successive Iq current values
//		//need to be below some threshold, close to zero.
////		while (fabs(diff_Iq) > 0.05 || abs(shaft_velocity) > 1) { //arbitrary accuracy/stability of Iq
////			loopFOC();
////			move();
//////		    shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
//////		    current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);
////			float new_Iq = LPF_current_q(current.q);
////			diff_Iq = new_Iq - prev_Iq;
////			prev_Iq = new_Iq;
////		}
//		_delay(5000); //arbitrary value
//	}

	setPhaseVoltage(0,  0, 0);
	//set all modified values back:
//	P_angle.P = prev_P_angle_P;
//	P_angle.I = prev_P_angle_I;
//	P_angle.D = prev_P_angle_D;
//	P_angle.limit = prev_P_angle_limit;
//	controller = requested_controller;
//	foc_modulation = requested_modulation;
//	torque_controller = requested_torque_controller;
	return exit_flag;
}


// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void BLDCMotor::loopFOC() {
  // if disabled do nothing
	//HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET); //debug for timing the FOC loop
  if(!enabled) return;
  // if open-loop do nothing
  if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;

  // shaft angle
  shaft_angle = shaftAngle();
  // electrical angle - need shaftAngle to be called first
  electrical_angle = electricalAngle();
//	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); //debug for timing the FOC loop
	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET); //debug for timing the FOC loop
  switch (torque_controller) {
    case TorqueControlType::voltage:
    	current.q = current_sense->getDCCurrent(electrical_angle);
    	current.q = LPF_current_q(current.q);
      // no need to do anything really
      break;
    case TorqueControlType::dc_current:
      if(!current_sense) return;
      // read overall current magnitude
      current.q = current_sense->getDCCurrent(electrical_angle);
      // filter the value values
      current.q = LPF_current_q(current.q);
      // calculate the phase voltage
      voltage.q = PID_current_q(current_sp - current.q);
      voltage.d = 0;
      break;
    case TorqueControlType::foc_current:
//        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); //debug for timing the FOC loop
      if(!current_sense) return;
      // read dq currents
      current = current_sense->getFOCCurrents(electrical_angle);
      // filter values
    //  current.q = LPF_current_q(current.q);
      //current.d = LPF_current_d(current.d);
      // calculate the phase voltages
      voltage.q = PID_current_q(current_sp - current.q);
      voltage.d = PID_current_d(-current.d);
      break;
    default:
      // no torque control selected
//      if(monitor_port) monitor_port->println(F("MOT: no torque control selected!"));
      break;
  }
//	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); //debug for timing the FOC loop
  // set the phase voltage - FOC heart function :)
//	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET); //debug for timing the FOC loop
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
//	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); //debug for timing the FOC loop
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void BLDCMotor::move(float new_target) {
  // if disabled do nothing
  if(!enabled) return;
  // downsampling (optional)
  if(motion_cnt++ < motion_downsample) return;
  motion_cnt = 0;
  // set internal target variable
  if(_isset(new_target)) target = new_target;
  // get angular velocity
  shaft_velocity = shaftVelocity();

  switch (controller) {
    case MotionControlType::torque:
      if(torque_controller == TorqueControlType::voltage) // if voltage torque control
        if(!_isset(phase_resistance))  voltage.q = target;
        else voltage.q =  target*phase_resistance;
      else
        current_sp = target; // if current/foc_current torque control
      break;
    case MotionControlType::angle:
      // angle set point
      shaft_angle_sp = target;
      // calculate velocity set point
      shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
      // if torque controlled through voltage
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q = current_sp*phase_resistance;
        voltage.d = 0;
      }
      break;
    case MotionControlType::velocity:
      // velocity set point
      shaft_velocity_sp = target;
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if current/foc_current torque control
      // if torque controlled through voltage control
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q = current_sp*phase_resistance;
        voltage.d = 0;
      }
      break;
    case MotionControlType::velocity_openloop:
      // velocity control in open loop
      shaft_velocity_sp = target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
    case MotionControlType::angle_openloop:
      // angle control in open loop
      shaft_angle_sp = target;
      voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
  }
}


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usaage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {
	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET); //debug for timing the FOC loop
  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
//    case FOCModulationType::Trapezoid_120 :
//      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
//      static int trap_120_map[6][3] = {
//        {_HIGH_IMPEDANCE,1,-1},{-1,1,_HIGH_IMPEDANCE},{-1,_HIGH_IMPEDANCE,1},{_HIGH_IMPEDANCE,-1,1},{1,-1,_HIGH_IMPEDANCE},{1,_HIGH_IMPEDANCE,-1} // each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
//      };
//      // static int trap_120_state = 0;
//      sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
//      // centering the voltages around either
//      // modulation_centered == true > driver.volage_limit/2
//      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
//      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
//
//      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
//        Ua= center;
//        Ub = trap_120_map[sector][1] * Uq + center;
//        Uc = trap_120_map[sector][2] * Uq + center;
//        driver->setPhaseState(_HIGH_IMPEDANCE, _ACTIVE, _ACTIVE); // disable phase if possible
//      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
//        Ua = trap_120_map[sector][0] * Uq + center;
//        Ub = center;
//        Uc = trap_120_map[sector][2] * Uq + center;
//        driver->setPhaseState(_ACTIVE, _HIGH_IMPEDANCE, _ACTIVE);// disable phase if possible
//      }else{
//        Ua = trap_120_map[sector][0] * Uq + center;
//        Ub = trap_120_map[sector][1] * Uq + center;
//        Uc = center;
//        driver->setPhaseState(_ACTIVE,_ACTIVE, _HIGH_IMPEDANCE);// disable phase if possible
//      }
//
//    break;
//
//    case FOCModulationType::Trapezoid_150 :
//      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
//      static int trap_150_map[12][3] = {
//        {_HIGH_IMPEDANCE,1,-1},{-1,1,-1},{-1,1,_HIGH_IMPEDANCE},{-1,1,1},{-1,_HIGH_IMPEDANCE,1},{-1,-1,1},{_HIGH_IMPEDANCE,-1,1},{1,-1,1},{1,-1,_HIGH_IMPEDANCE},{1,-1,-1},{1,_HIGH_IMPEDANCE,-1},{1,1,-1} // each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
//      };
//      // static int trap_150_state = 0;
//      sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
//      // centering the voltages around either
//      // modulation_centered == true > driver.volage_limit/2
//      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
//      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
//
//      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
//        Ua= center;
//        Ub = trap_150_map[sector][1] * Uq + center;
//        Uc = trap_150_map[sector][2] * Uq + center;
//        driver->setPhaseState(_HIGH_IMPEDANCE, _ACTIVE, _ACTIVE); // disable phase if possible
//      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
//        Ua = trap_150_map[sector][0] * Uq + center;
//        Ub = center;
//        Uc = trap_150_map[sector][2] * Uq + center;
//        driver->setPhaseState(_ACTIVE, _HIGH_IMPEDANCE, _ACTIVE);// disable phase if possible
//      }else{
//        Ua = trap_150_map[sector][0] * Uq + center;
//        Ub = trap_150_map[sector][1] * Uq + center;
//        Uc = center;
//        driver->setPhaseState(_ACTIVE, _ACTIVE, _HIGH_IMPEDANCE);// disable phase if possible
//      }
//
//    break;

    case FOCModulationType::SinePWM :
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation

      // angle normalization in between 0 and 2pi
      // only necessary if using _sin and _cos - approximation functions
      angle_el = _normalizeAngle(angle_el);
      _ca = _cos(angle_el);
      _sa = _sin(angle_el);
      // Inverse park transform
      Ualpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      Ubeta =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

      // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
      center = driver->voltage_limit/2;
      // Clarke transform
      Ua = Ualpha + center;
      Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta + center;
      Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta + center;

      if (!modulation_centered) {
        float Umin = fmin(Ua, fmin(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }

      break;

    case FOCModulationType::SpaceVectorPWM :
      // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
      // https://www.youtube.com/watch?v=QMSWUMEAejg
//    	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET); //debug for timing the FOC loop
      // the algorithm goes
      // 1) Ualpha, Ubeta
      // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
      // 3) angle_el = atan2(Ubeta, Ualpha)
      //
      // equivalent to 2)  because the magnitude does not change is:
      // Uout = sqrt(Ud^2 + Uq^2)
      // equivalent to 3) is
      // angle_el = angle_el + atan2(Uq,Ud)

      float Uout;
      // a bit of optitmisation
      if(Ud){ // only if Ud and Uq set
        // _sqrt is an approx of sqrt (3-4% error)
        Uout = _sqrt(Ud*Ud + Uq*Uq) / driver->voltage_limit;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
      }else{// only Uq available - no need for atan2 and sqrt
        Uout = Uq / driver->voltage_limit;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + _PI_2);
      }
      // find the sector we are in currently
      sector = floor(angle_el / _PI_3) + 1;
      // calculate the duty cycles
      float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
      float T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
      // two versions possible
      float T0 = 0; // pulled to 0 - better for low power supply voltage
      if (modulation_centered) {
        T0 = 1 - T1 - T2; //modulation_centered around driver->voltage_limit/2
      }

      // calculate the duty cycles(times)
      float Ta,Tb,Tc;
      switch(sector){
        case 1:
          Ta = T1 + T2 + T0/2;
          Tb = T2 + T0/2;
          Tc = T0/2;
          break;
        case 2:
          Ta = T1 +  T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T0/2;
          break;
        case 3:
          Ta = T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T2 + T0/2;
          break;
        case 4:
          Ta = T0/2;
          Tb = T1+ T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 5:
          Ta = T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 6:
          Ta = T1 + T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T0/2;
          break;
        default:
         // possible error state
          Ta = 0;
          Tb = 0;
          Tc = 0;
      }

      // calculate the phase voltages and center
      Ua = Ta*driver->voltage_limit;
      Ub = Tb*driver->voltage_limit;
      Uc = Tc*driver->voltage_limit;
      break;

  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); //debug for timing the FOC loop
}



// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float BLDCMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  // for display purposes
  shaft_velocity = target_velocity;

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance;

  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float BLDCMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance;
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}
