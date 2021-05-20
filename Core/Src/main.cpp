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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SimpleFOC.hpp"
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

/* USER CODE BEGIN PV */
BLDCMotor motor = BLDCMotor(7);//,0.039);//,0.039); //(pp,phase_resistance)
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);
Encoder encoder = Encoder(2, 3, 2048, 11);//, 11);//, 4); //these pins, and values are actually hardcoded
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int loopIdx = 10000;
float time_loop;
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_ADC_Start(&hadc1); //ADC set to run in continuous conversion mode - hence we start conversions here
  HAL_ADC_Start(&hadc3); //ADC set to run in continuous conversion mode - hence we start conversions here


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
 motor.velocity_index_search = 0.5; //needs to be low otherwise index search fails. Tested to 10rad/s - doesn't fail anymore.
 //set motion control loop to be used
 motor.foc_modulation = FOCModulationType::SpaceVectorPWM;//SinePWM;
 motor.controller = MotionControlType::velocity;//angle;//velocity;//torque;
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
	motor.P_angle.P = 1;
//	motor.P_angle.I = 0.01;
//	motor.P_angle.D = 0;
//	motor.P_angle.limit = 1000;

 motor.PID_velocity.P = 0.05;//0.5;
 motor.PID_velocity.I = 1;//10;
 motor.PID_velocity.D = 0;
//  motor.PID_velocity.output_ramp = 1000;
 //motor.PID_velocity.limit = 24;//FIXME - this doesn't initialise correctly
// motor.velocity_limit = 20; //FIXME - NAT

 //velocity PI controller parameters
//  motor.PID_velocity.P = 0.01;//0.2;//0.05;
//  motor.PID_velocity.I = 0;//20;
//  motor.PID_velocity.D = 0;
 //default voltage_power_supply
 motor.voltage_limit = 2;//12;
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
 float targets[] = {0, 1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1};
 int i = 0;
 int idx = 0;

//  driver.setPwm(0,0,3);

 float time_prev;


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
		GPIOB -> ODR |= GPIO_PIN_0;
	 time_prev = _micros();
	  motor.loopFOC();
	  time_loop = _micros() - time_prev;
		GPIOB -> ODR &= ~GPIO_PIN_0;


	  if (idx % 1000 == 0) {
		  motor.move();
		  idx = 1;
	  }
	  idx++;

//	  if (idx % loopIdx == 0) {
//		  motor.target = targets[i];
//		  i++;
//		 // i = i % 12;
//	  }
//	  if (i >= 12) {
//		  i = 0;
//	  }
//	  idx++;
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
