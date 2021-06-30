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
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SimpleFOC.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROLE_MASTER  // when defined MASTER clock settings are programmed, else, SLAVE clock settings are programmed
#define SENSOR_MAGNETIC //position sensor is MAGNETIC (AS5048A)
//#define SENSOR_ENCODER //position sensor is ENCODER
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TEST_SPI 0 //SPI cable super flaky - need to run this test sometimes
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BLDCMotor motor = BLDCMotor(7);//,0.039);//,0.039); //(pp,phase_resistance)
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);
#ifdef SENSOR_ENCODER
	Encoder sensor = Encoder(2, 3, 2048, 11); //these pins, and values are actually hardcoded
#else //SENSOR_MAGNETIC
	//Magnetic sensor class supports only AS5048A at the moment: https://media.digikey.com/pdf/Data%20Sheets/Austriamicrosystems%20PDFs/AS5048A,B.pdf
	MagneticSensorSPI sensor = MagneticSensorSPI(); //CS = D10 (PD14 on STM32H743), resolution bits = 14, angle register address = 0x3fff); //these pins are all hardcoded
#endif

InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, phaseA, phaseB); //pins are hardcoded
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	int raw_reading = HAL_ADC_GetValue(&hadc1); //assigned to a variable prior to return for debugging purposes
	return raw_reading;
}

float _readADCVoltage_pinB(void) {
	int raw_reading = HAL_ADC_GetValue(&hadc3); //assigned to a variable prior to return for debugging purposes
	return raw_reading;
}

int _calibrate_phaseA(void) {
	int sum = 0;
	//calibration process, the calculated calibration value (offset) should be automatically applied
	//during subsequent conversions
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	for (int i = 0; i < 100; i++) {
		sum += HAL_ADC_GetValue(&hadc1);
	}
	return (float)(sum/100.0);
}

int _calibrate_phaseB(void) {
	int sum = 0;
	//calibration process, the calculated calibration value (offset) should be automatically applied
	//during subsequent conversions
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	for (int i = 0; i < 100; i++) {
		sum += HAL_ADC_GetValue(&hadc3);
	}
	return (float)(sum/100);
}

uint16_t _SPI_read(uint16_t read_command) {
	uint8_t register_value[2] = {0x00, 0x00};
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&read_command, &register_value[0], 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	return ((register_value[1] << 8) | register_value[0]);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int loopIdx = 100000;
float time_loop;
bool initialisation_complete = false;
bool run_foc_loop = false;
int inc_mov_loop = 0;

float copy_target = 0;
float lk_shaft_velocity;
float lk_shaft_angle;
float lk_current_sp;

float copy_PID_Iq_P = 0.5;
float copy_PID_Iq_I = 0.1;
float copy_PID_Iq_D = 0.01;
float copy_PID_Iq_ramp = 50;
float copy_PID_Iq_limit = 2;
float copy_PID_Iq_Tf = 0.01;

float copy_PID_Id_P = 0.5;
float copy_PID_Id_I = 0.1;
float copy_PID_Id_D = 0.01;
float copy_PID_Id_ramp = 50;
float copy_PID_Id_limit = 1;
float copy_PID_Id_Tf = 0.01;

float copy_PID_velocity_P = 0;// 0.1;//0.05;
float copy_PID_velocity_I = 0;//0.5;//0.5;
float copy_PID_velocity_D = 0;// 0.001;//0;
float copy_PID_velocity_ramp = 0;// 5000;//1000;
float copy_PID_velocity_limit = 0;// 24;//2;
float copy_PID_velocity_Tf = 0;// 0;//0.01;//0;

float copy_PID_angle_P = 0;// 10;//0.5;
float copy_PID_angle_I = 0;// 1;//0;
float copy_PID_angle_D = 0;// 0;//0;
float copy_PID_angle_ramp = 0;// 100;//1e9;
float copy_PID_angle_limit = 0;// 10;//20;
float copy_PID_angle_Tf = 0;// 0;//0.001;



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
  MX_TIM2_Init();
  MX_SPI1_Init(); //maybe need to move this call into the magneticsensorSPI class - init


  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1); //25kHz timer - need interrupt to trigger FOC loop at this frequency
  HAL_TIM_Base_Start_IT(&htim2); //timer set to 1MHz used by _micros(), will overflow every 4295 seconds - should do something about this overflow... TODO

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  HAL_ADC_Start(&hadc1); //ADC set to run in continuous conversion mode - hence we start conversions here
  HAL_ADC_Start(&hadc3); //ADC set to run in continuous conversion mode - hence we start conversions here

//  //closed loop velocity example:
//#if SENSOR == ENCODER
//  encoder.init();
//  //link the motor to the sensor:
//  motor.linkSensor(&encoder);
//#elif SENSOR == MAGNETIC
//  sensor.init();
//  motor.linkSensor(&sensor);
//#endif
sensor.init();

motor.linkSensor(&sensor);
 //driver config:
 driver.voltage_power_supply = 10;//24;
 driver.init();
 //link the motor and the driver:
 motor.linkDriver(&driver);
 //aligning the voltage [V]
 motor.voltage_sensor_align = 2;
 //index search velocity [rad/s]
 motor.velocity_index_search = 0.5; //needs to be low otherwise index search fails. Tested to 10rad/s - doesn't fail anymore.
 //set motion control loop to be used
 motor.foc_modulation = FOCModulationType::SpaceVectorPWM;//SpaceVectorPWM;//SinePWM;
 motor.controller = MotionControlType::velocity;//angle;//velocity;//torque;
 motor.torque_controller = TorqueControlType::foc_current;//voltage;//foc_current;//dc_current;//voltage;
//  motor.controller = MotionControlType::torque;

 //current sense init and linking
 current_sense.gain_b *=-1;
 // skip alignment
 current_sense.skip_align = true; //true;

 current_sense.init();
 motor.linkCurrentSense(&current_sense);

 motor.anti_cogging = false;

 //initialise motor
 motor.init();
 //align sensor and start FOC
 motor.initFOC();

 _delay(100);

 //controller config:
 //see default parameters in defaults.h
 motor.PID_current_q.P = copy_PID_Iq_P;
 motor.PID_current_q.I = copy_PID_Iq_I;
 motor.PID_current_q.D = copy_PID_Iq_D;
 motor.PID_current_q.output_ramp = copy_PID_Iq_ramp;
 motor.PID_current_q.limit = copy_PID_Iq_limit;
 motor.LPF_current_q.Tf = copy_PID_Iq_Tf;

 motor.PID_current_d.P = copy_PID_Id_P;
 motor.PID_current_d.I = copy_PID_Id_I;
 motor.PID_current_d.D = copy_PID_Id_D;
 motor.PID_current_d.output_ramp = copy_PID_Id_ramp;
 motor.PID_current_d.limit = copy_PID_Id_limit;
 motor.LPF_current_d.Tf = copy_PID_Id_Tf;

 motor.PID_velocity.P = copy_PID_velocity_P;
 motor.PID_velocity.I = copy_PID_velocity_I;
 motor.PID_velocity.D = copy_PID_velocity_D;
 motor.PID_velocity.output_ramp = copy_PID_velocity_ramp;
 motor.PID_velocity.limit = copy_PID_velocity_limit;
 motor.LPF_velocity.Tf = copy_PID_velocity_Tf;

 motor.P_angle.P = copy_PID_angle_P;
 motor.P_angle.I = copy_PID_angle_I;
 motor.P_angle.D = copy_PID_angle_D;
 motor.P_angle.output_ramp = copy_PID_angle_ramp;
 motor.P_angle.limit = copy_PID_angle_limit;
 motor.LPF_angle.Tf = copy_PID_angle_Tf;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 motor.target = 0;//2;//0.6;//3;//0.5;// 0.25;
// motor.controller = MotionControlType::velocity;//angle;//velocity;//torque;
// float targets[] = {0, 1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1};
 //float targets[] = {2.269, 0.8727}; //angle mode
// float targets[] = {4, 8}; //velocity mode
// int i = 0;
 int idx = 0;

 initialisation_complete = true;


if (TEST_SPI) { //connections are currently super flaky - need to run this sometimes
	uint8_t SPI_Read[4] = {0xFF,0xFF,0x00,0x00};
	uint16_t buf;
	float buf_angle = 0;
	while (1) {
		_delay(1);
	    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	    HAL_SPI_TransmitReceive(&hspi1, &SPI_Read[0], &SPI_Read[2], 1, HAL_MAX_DELAY);
	    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	    buf = ((SPI_Read[3]<<8)|SPI_Read[2])&AS5048A_RESULT_MASK;
	    buf_angle = ((float)buf)/AS5048A_CPR * 2 * _PI;
	    buf_angle = buf_angle; //dummy assignment to avoid -Werror about unused variable
	}
}

 while (1)
 {
//	 //read TIM2->CH1 values:
//	 unsigned int test = TIM2->CNT;
//	    /* USER CODE END WHILE */
//
//	    /* USER CODE BEGIN 3 */
//	    currents = current_sense.getPhaseCurrents();
//	    current_magnitude = current_sense.getDCCurrent();
//
//	  //angular set point example for PID tuning
//		GPIOB -> ODR |= GPIO_PIN_0;
//	 time_prev = _micros();

	 lk_shaft_velocity = motor.shaft_velocity;
	 lk_shaft_angle = motor.shaft_angle;
	 lk_current_sp = motor.current_sp;

//	 motor.PID_current_q.P = copy_PID_Iq_P;
//	 motor.PID_current_q.I = copy_PID_Iq_I;
//	 motor.PID_current_q.D = copy_PID_Iq_D;
//	 motor.PID_current_q.output_ramp = copy_PID_Iq_ramp;
//	 motor.PID_current_q.limit = copy_PID_Iq_limit;
//	 motor.LPF_current_q.Tf = copy_PID_Iq_Tf;
//
//	 motor.PID_current_d.P = copy_PID_Id_P;
//	 motor.PID_current_d.I = copy_PID_Id_I;
//	 motor.PID_current_d.D = copy_PID_Id_D;
//	 motor.PID_current_d.output_ramp = copy_PID_Id_ramp;
//	 motor.PID_current_d.limit = copy_PID_Id_limit;
//	 motor.LPF_current_d.Tf = copy_PID_Id_Tf;
//
//	 motor.PID_velocity.P = copy_PID_velocity_P;
//	 motor.PID_velocity.I = copy_PID_velocity_I;
//	 motor.PID_velocity.D = copy_PID_velocity_D;
//	 motor.PID_velocity.output_ramp = copy_PID_velocity_ramp;
//	 motor.PID_velocity.limit = copy_PID_velocity_limit;
//	 motor.LPF_velocity.Tf = copy_PID_velocity_Tf;
//
//	 motor.P_angle.P = copy_PID_angle_P;
//	 motor.P_angle.I = copy_PID_angle_I;
//	 motor.P_angle.D = copy_PID_angle_D;
//	 motor.P_angle.output_ramp = copy_PID_angle_ramp;
//	 motor.P_angle.limit = copy_PID_angle_limit;
//	 motor.LPF_angle.Tf = copy_PID_angle_Tf;

	 if (run_foc_loop) {
	  motor.loopFOC();
	  //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	  run_foc_loop = false;
	}
//	  time_loop = _micros() - time_prev;
//		GPIOB -> ODR &= ~GPIO_PIN_0;

//	  if (idx % 1000 == 0) { //velocity mode set to 1000
//		  motor.move();
//		// idx = 1;
//	  }

	  if (inc_mov_loop >= 2000) {
		  motor.move();
		  inc_mov_loop = 0;
		 // HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		  //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	  }
	// motor.target = copy_target;

//	  if (idx % loopIdx == 0) {
//		 // motor.target += targets[i];
//		 // motor.copy_target += targets[i];
//		  motor.target += targets[i]; //when in 'angle' mode
//		  //motor.target = targets[i]; //when in 'velocity' mode
//		  copy_target = motor.target;
//
//		  i++;
//	  }
//	  if (i >= 2) {
//		  i = 0;
//	  }
	  idx++;
 }

 /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
#ifdef ROLE_MASTER
/*
  * MASTER SystemClock_Config
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 1;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_15);
}
#else
/**
  * @brief System Clock Configuration
  * @retval None
  * SLAVE SystemClock_Config
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 1;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}
#endif

/* USER CODE BEGIN 4 */
#ifdef SENSOR_ENCODER
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
#endif

// Callback: timer2 has rolled over. Occurs every 4295 seconds.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	if (htim == &htim1) {
		//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		//HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); //verified that this should toggle the pin at 25kHz
		if (initialisation_complete) {
			//now run FOC loop:
			//HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET); //debug for timing the FOC loop
			run_foc_loop = true; //motor.loopFOC();
			inc_mov_loop++;
			//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); //debug for timing the FOC loop
		}
	}

	if (htim == &htim2) { //when this timer overflows - should do something as lots of calculations will go to shit
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	}

//	if (htim == &htim4) { //encoder should have tracked 2048 increments
//		//  encoder_time = HAL_GetTick();
//		//  encoder_ticks = TIM2->CNT;
//		rotations += 1;
//	}
}
//
//// This is called when SPI transmit is done
//void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi)
//{
//  // Set CS pin to high
// // HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//}
//
//// This is called when SPI receive is done
//void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
//{
//  // Set CS pin to high
//  //HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//}


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
