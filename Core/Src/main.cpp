/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "sensors/Encoder.hpp"
#include "SimpleFOC.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
BLDCMotor motor = BLDCMotor(7);//,0.039);//,0.039); //(pp,phase_resistance)
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);
Encoder encoder = Encoder(2, 3, 2048, 11);//, 11);//, 4); //these pins, and values are actually hardcoded
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, phaseA, phaseB); //pins are hardcoded
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
PhaseCurrent_s currents;
float current_magnitude = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware specific
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
	int pwm_range = htim1.Instance->ARR;
	htim1.Instance->CCR1 = (int)(dc_a * pwm_range);
	htim1.Instance->CCR2 = (int)(dc_c * pwm_range);
	htim1.Instance->CCR4 = (int)(dc_b * pwm_range);
}

// function reading an ADC value and returning the read voltage
float _readADCVoltage_pinA(void){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	int raw_reading = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return raw_reading;
}

float _readADCVoltage_pinB(void) {
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
	int raw_reading = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
	return raw_reading;
}

int _calibrate_phaseA(void) {
	int sum = 0;
	//calibration process, the calculated calibration value (offset) should be automatically applied
	//during subsequent conversions
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	for (int i = 0; i < 100; i++) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		sum += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	return (float)(sum/100.0);
}

int _calibrate_phaseB(void) {
	int sum = 0;
	//calibration process, the calculated calibration value (offset) should be automatically applied
	//during subsequent conversions
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	for (int i = 0; i < 100; i++) {
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
		sum += HAL_ADC_GetValue(&hadc3);
		HAL_ADC_Stop(&hadc3);
	}
	return (float)(sum/100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int loopIdx = 10000;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    //closed loop velocity example:
  encoder.init();
  //link the motor to the sensor:
  motor.linkSensor(&encoder);
  //driver config:
  driver.voltage_power_supply = 24;
  driver.init();
  //link the motor and the driver:
  motor.linkDriver(&driver);
  //aligning the voltage [V]
 // motor.voltage_sensor_align = 2;
  //index search velocity [rad/s]
  motor.velocity_index_search = 0.1; //needs to be low otherwise index search fails.
  //set motion control loop to be used
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;//SinePWM;
  motor.controller = MotionControlType::angle;//angle;//velocity;//torque;
  motor.torque_controller = TorqueControlType::voltage;//voltage;//foc_current;//dc_current;//voltage;
//  motor.controller = MotionControlType::torque;

  //controller config:
  //see default parameters in defaults.h
  motor.PID_current_q.P = 0.5;
  motor.PID_current_q.I = 0.1;
  motor.PID_current_q.D = 0;
  motor.PID_current_q.output_ramp = 50;
  motor.PID_current_q.limit = 1;

  motor.PID_current_d.P = 0.5;
  motor.PID_current_d.I = 0.1;
  motor.PID_current_d.D = 0;
  motor.PID_current_d.output_ramp = 50;
  motor.PID_current_d.limit = 1;

  motor.LPF_current_q.Tf = 0.1;
  motor.LPF_current_d.Tf = 0.1;

//  // angle P params
//  motor.P_angle.P = 10;
//  motor.P_angle.I = 0.005;
//  motor.P_angle.D = 0.05;

	//---now set wanted values: - from cogging calibration
	motor.P_angle.P = 0.5;
//	motor.P_angle.I = 0.01;
//	motor.P_angle.D = 0;
//	motor.P_angle.limit = 1000;
	motor.LPF_angle.Tf = 0.001;


  motor.PID_velocity.P = 0.05;//0.5;
  motor.PID_velocity.I = 1;//10;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;
  //motor.PID_velocity.limit = 24;//FIXME - this doesn't initialise correctly
  motor.velocity_limit = 20; //FIXME - NAT

  //velocity PI controller parameters
//  motor.PID_velocity.P = 0.01;//0.2;//0.05;
//  motor.PID_velocity.I = 0;//20;
//  motor.PID_velocity.D = 0;
  //default voltage_power_supply
  motor.voltage_limit = 4;//12;
  //jerk control using voltage voltage ramp
  //default value is 300 volts per sec, ~0.3V/millisec
 // motor.PID_velocity.output_ramp = 1000;
  //velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.1; //0.01;
//  //angle loop controller
//  motor.P_angle.P = 20;
//  //angle loop velocity limit
//  motor.velocity_limit = 20;
  //current sense init and linking
  current_sense.gain_b *=-1;
  // skip alignment
  current_sense.skip_align = true; //true;


  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  //9th April, 2021:
  //TODO - here, align Iq - this may already be done with zero_angle, but should not this come before phase/driver alignment?
  //why index search doesn't work?
  //--seems like index search fails due to speed of index search set too high (20rad/s) causing
  //--motor not to be able to move to requested angle, before next position call was mad.
  //--either adding during index search while loop: BLDCMotor::absoluteZeroSearch()
  //--or decreasing "motor.velocity_index_search" to 0.1 fixes this problem.

  //Reset pulse counter to 0 on index detection? this would affect the velocity calculations
  //but resetting the pulse counter at some point is necessary otherwise counter will overflow.

  //* Implement anti-cogging - i.e. cogging compensation
  //* Check why "pulse_per_second" fluctuates between 999 and 1999 when motor spinning

  //10th April, 2021:
  /* working on the anti-cogging feature. Can't get the angle/foc_current control to work w/i the calibration loop
   * so tried to bring it out into the while(1) loop below, but can't get the motor to iterate through a set of
   * target angles here either - why? Is the _delay(2000) necessary? Why can't I set the motor.target value at
   * a lower rate then loopFOC and have motor.move() also running at a lower rate than loopFOC()?
   *
   */

  //16th April, 2021:
  /* Looking into the functions loopFOC() and move([target optional]) it seems that in velocity mode, they are
   * best run at the same freq. Within the move() function, in angle control mode both the velocity set point (based on error
   * between angle set point and measured angle) and current (based on error between new velocity set point and measured
   * velocity). However, it is probably best to have the current set point
   * set points are calculated within the same function. However, it is
   */

  //22nd April, 2021:
  /* Working on getting parity between Arduino and STM32 environments.
   * The 'natural' directions of spin were found to be different. Had to switch the assignment of encoder to GPIO pins in
   * main.h to get matching (CCW) estimates.
   * Next need to make sure that the zero-offset angle is calculated the same. Why/how can it be different?
   */

  //23rd April, 2021:
  /* Removed TIM4, now using TIM1CH4 for PWMA_Pin9 (which is actually pin D4). This is so that I don't need to synchronise
   * the timer channels. As all three PWM channels are now driven by the same clock, they should always be in sync.
   * Undid the switching of encoder pin assignment. The better way to achieve the same effect is to switch the pins the
   * three PWM signals are assigned - see function: _writeDutyCycle3PWM().
   * Am still not sure what PWM (1 or 2) mode is used, the code is not clear: https://github.com/simplefoc/Arduino-FOC/blob/40336919658f35790c96164dc37a6395fe8b4526/src/drivers/hardware_specific/stm32_mcu.cpp
   * Tried both PWM1 and PWM2, but neither lead to the same (as arduino) zero angle offset. In arduino I get 2.9877 radians, and
   * with this code for all possible combinations of a/b/c to U/V/W channels (and PWM1 or PWM2) I can't get the same offset angle.
   *
   * Also, need to look at correctly sampling the current from the phases. Maybe sampling many times quickly and smoothing out the
   * values, or integrating over a long(er) time...
   */


  //4th May, 2021:
  /* Currently code causes the motor to short circuit during enable where setPwm(0,0,0) - don't understand why.
   *
   */

  //6th May, 2021:
  /* The cause of motor short (noted above) is due to the fact that had moved the PWM signals for the motor phases
   * from D15/E11/E14 on STM board to E9/E11/E14 so that they could all be driven by the same clock. However, this
   * change also needed to include a hardware change - physically connecting pin E9 (arduino D4) to pin D15 (arduino D9).
   *
   * Okay, so motor calibrating, and finding zero-offset angle of 2.05 radians. Doesn't match with Arduino code.
   *
   * Motor also doesn't spin properly when motor.target = 1, which in Arduino causes a smooth spin. Will investigate
   * possible poor current estimates.
   *
   * Problem with current readings may be to do with the fact that the timing of readings is falling at different times within
   * the PWM signal causing various inaccurate readings. Need to trigger the ADC sampling every time a PWM pulse is triggered.
   * Tried to setup ADC to trigger on TIM1 counter overflow - added callback function, with LED toggle code to verify that this works.
   * Tomorrow need to debug, and get working.
   *
   */

  //7th May, 2021:
  /* Working on getting the current readings accurately - timed by the PWM freq. signal (TIMx_ARR).
   *
   */

  //10th May, 2021:
  /* fix current measurement calculations.
   * Why does motor 'click' and current steps high to low?
   */
  motor.anti_cogging = false;


  //initialise motor
  motor.init();
  //align sensor and start FOC
  motor.initFOC();

  _delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  motor.target = 1;//2;//0.6;//3;//0.5;// 0.25;
 // motor.controller = MotionControlType::velocity;//angle;//velocity;//torque;
  float targets[] = {0, 10, 20, 30, 40, 50, 60, 50, 40, 30, 20, 10};
  int i = 0;
  int idx = 0;

//  driver.setPwm(0,0,3);
  while (1)
  {
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
	    currents = current_sense.getPhaseCurrents();
	    current_magnitude = current_sense.getDCCurrent();
//
//	  //angular set point example for PID tuning
	  motor.loopFOC();
	  motor.move();

	  if (idx % loopIdx == 0) {
		  motor.target = targets[i];
		  i++;
		 // i = i % 12;
	  }
	  if (i >= 12) {
		  i = 0;
	  }
	  idx++;
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 19;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;//DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;//ADC_EXTERNALTRIG_T1_TRGO;//ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;//ADC_EXTERNALTRIGCONVEDGE_RISING;//ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;//ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;//DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;//ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 4800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;//TIM_TRGO_UPDATE;//TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2; //TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enable_Pin8_GPIO_Port, Enable_Pin8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enable_Pin8_Pin */
  GPIO_InitStruct.Pin = Enable_Pin8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Enable_Pin8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EncoderAU_Pin3_Pin */
  GPIO_InitStruct.Pin = EncoderAU_Pin3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EncoderAU_Pin3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EncoderBV_Pin2_Pin */
  GPIO_InitStruct.Pin = EncoderBV_Pin2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EncoderBV_Pin2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EncoderIW_Pin11_Pin */
  GPIO_InitStruct.Pin = EncoderIW_Pin11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EncoderIW_Pin11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == EncoderAU_Pin3_Pin) {
		encoder.handleA();
	}
	if (GPIO_Pin == EncoderBV_Pin2_Pin) {
		encoder.handleB();
	}
	if (GPIO_Pin == EncoderIW_Pin11_Pin) {
		encoder.handleIndex();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
