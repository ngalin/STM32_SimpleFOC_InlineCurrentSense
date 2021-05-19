# STM32_SimpleFOC_InlineCurrentSense
Port of the SimpleFOC code to STM32 environment. (Original code here: https://github.com/simplefoc/Arduino-FOC/tree/v2.1)



Current hardware used:

* Nucleo-H743ZI2,  caution board number is: MB1364, which means the correct user manual is: 
  https://www.st.com/resource/en/user_manual/dm00499160-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf
* SimpleFOCShield v2.0.2
* Motor: low resistance (39mOhm phase resistance) [D5065](https://odriverobotics.com/shop/odrive-custom-motor-d5065) with 7 pole pairs



SimpleFOCShield is configured with the following connections: 

| Signal              | Arduino pin name            | STM32 pin name | Function    |
| ------------------- | --------------------------- | -------------- | ----------- |
| PWM A               | 9 (**broken off, see below) | PE14           | TIM1(CH4)   |
| PWM B               | 5                           | PE11           | TIM1(CH2)   |
| PWM C               | 6                           | PE9            | TIM1(CH1)   |
| Enable              | 8                           | PF3            | GPIO_Output |
| Encoder A           | 3                           | PE13           | GPIO_EXTI3  |
| Encoder B           | 2                           | PG14           | GPIO_EXTI14 |
| Encoder Idx         | 11                          | PB5            | GPIO_EXTI5  |
| Phase A             | A0                          | PA3            | ADC1_INP15  |
| Phase B             | A2                          | PC3_c          | ADC3_INP1   |
| Range               | (3.3V)                      | n/a            | n/a         |
| Enable 8V regulator | (true)                      | n/a            | n/a         |

** In order to route the TIM1(CH4) PWM signal which is configure to be output onto pin PE14 of the STM board, pin 9 on the SimpleFOC shield is removed and a jumper lead is used to Arduino pin D4 to Arduino pin D9. 



Issues:

* phase current measurement is not accurate, hence cannot use true FOC



Improvements:

* avoid pulse_counter overflow. Count number of rotations, and keep pulse_counter between 0 - CPR

* switch to dedicated timer hardware for encoder pulse count. Requires hardware signal re-routing

* given accurate current measurements not possible, implement cogging torque measurements in voltage mode

  ​			

Unresolved questions:
* operating in FOC position control, encoder pulse counter does not increment properly when motor is externally pushed out of position. Hence, motor can be successfully pushed away from target position set point. Would expect this not to happen
* regeneration support
* bandwidth of control loop - probably affects the maximum update position rate



TODO:

* simulate motion of engine under external combustion torques with pre-defined position changes
* update enclosure design to fit onto motor shaft.
