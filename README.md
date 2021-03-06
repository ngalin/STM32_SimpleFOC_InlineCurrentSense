# STM32_SimpleFOC_InlineCurrentSense
[UPDATE 30/06/21]: Added support for AS5048A position sensor.



Port of the SimpleFOC code to STM32 environment. (Original code here: https://github.com/simplefoc/Arduino-FOC/tree/v2.1)

Current hardware used:

* Nucleo-H743ZI2,  caution board number is: MB1364, which means the correct user manual is: 
  https://www.st.com/resource/en/user_manual/dm00499160-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf
* SimpleFOCShield v2.0.2
* Motor: low resistance (39mOhm phase resistance) [D5065](https://odriverobotics.com/shop/odrive-custom-motor-d5065) with 7 pole pairs
* Motor: gimbal iPower [GM2804 with AS5048A](https://shop.iflight-rc.com/ipower-gm2804-gimbal-motor-with-as5048a-encoder-pro288) encoder



SimpleFOCShield is configured with the following connections: 

| Signal                 | Arduino pin name            | STM32 pin name | Function    |
| ---------------------- | --------------------------- | -------------- | ----------- |
| PWM A                  | 9 (**broken off, see below) | PE14           | TIM1(CH4)   |
| PWM B                  | 5                           | PE11           | TIM1(CH2)   |
| PWM C                  | 6                           | PE9            | TIM1(CH1)   |
| Enable                 | 8                           | PF3            | GPIO_Output |
| Encoder A              | 3                           | PE13           | GPIO_EXTI3  |
| Encoder B              | 2                           | PG14           | GPIO_EXTI14 |
| Encoder Idx            | 11                          | PB5            | GPIO_EXTI5  |
| Phase A                | A0                          | PA3            | ADC1_INP15  |
| Phase B                | A2                          | PC3_c          | ADC3_INP1   |
| SCK (CLK to SPI slave) | D13                         | PA5            | SPI1_CLK    |
| MISO                   | D12                         | PA6            | SPI1_MISO   |
| MOSI                   | n/a (*** can't use D11)     | PA7            | SPI1_MOSI   |
| CS (chip select)       | D10                         | PD14           | SPI1_CS     |
| Range                  | (3.3V)                      | n/a            | n/a         |
| Enable 8V regulator    | (true)                      | n/a            | n/a         |

** In order to route the TIM1(CH4) PWM signal which is configure to be output onto pin PE14 of the STM board, pin 9 on the SimpleFOC shield is removed and a jumper lead is used to Arduino pin D4 to Arduino pin D9. 

*** On Arduino SPI for MOSI would be D11. In our setup EncoderIdx uses this pin. So we route SPI1 MOSI to PA7 of STM32H743 chip instead. For this to work we need to remove the D11 pin from SimpleFOC shield - in the same way we broke off pin9. 

Issues:

* [Resolved] phase current measurement is not accurate, hence cannot use true FOC
  - current limit has to be set ~1-2Amps, and the PID parameters of the Q/D current loops need to be adjusted to accommodate a low-resistance motor. Table included [here](https://community.simplefoc.com/t/simplefoc-v2-understanding-sensor-align/806/22) records working values. 


Improvements to be made:

* avoid pulse_counter overflow. Count number of rotations, and keep pulse_counter between 0 - CPR

* switch to dedicated timer hardware for encoder pulse count. Requires hardware signal re-routing

* implement cogging torque measurements in voltage mode


Unresolved questions:
* operating in FOC position control, encoder pulse counter does not increment properly when motor is externally pushed out of position. Hence, motor can be successfully pushed away from target position set point. Would expect this not to happen
* regeneration support
* bandwidth of control loop - probably affects the maximum update position rate



TODO:

* simulate motion of engine under external combustion torques with pre-defined position changes
* update enclosure design to fit onto motor shaft.
