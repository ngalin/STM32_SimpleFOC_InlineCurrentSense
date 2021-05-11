################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/motor_drivers/BLDCDriver3PWM.cpp 

OBJS += \
./Core/Src/motor_drivers/BLDCDriver3PWM.o 

CPP_DEPS += \
./Core/Src/motor_drivers/BLDCDriver3PWM.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/motor_drivers/BLDCDriver3PWM.o: ../Core/Src/motor_drivers/BLDCDriver3PWM.cpp Core/Src/motor_drivers/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/Src/motor_drivers/BLDCDriver3PWM.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

