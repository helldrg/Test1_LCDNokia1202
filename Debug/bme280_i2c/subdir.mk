################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bme280_i2c/BME280.c 

C_DEPS += \
./bme280_i2c/BME280.d 

OBJS += \
./bme280_i2c/BME280.o 


# Each subdirectory must supply rules for building sources it contributes
bme280_i2c/BME280.o: ../bme280_i2c/BME280.c bme280_i2c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Electromicro/STM32CubeIDE/workspace_1.6.0/Test1_LCDNokia1202/bme280_i2c" -I"C:/Users/Electromicro/STM32CubeIDE/workspace_1.6.0/Test1_LCDNokia1202/LCD_NOKIA1202" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"bme280_i2c/BME280.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

