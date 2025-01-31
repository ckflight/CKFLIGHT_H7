################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/DRIVERS/CK_ADC.c \
../Core/Inc/DRIVERS/CK_BUZZER.c \
../Core/Inc/DRIVERS/CK_CIRCULARBUFFER.c \
../Core/Inc/DRIVERS/CK_GPIO.c \
../Core/Inc/DRIVERS/CK_I2C.c \
../Core/Inc/DRIVERS/CK_LED.c \
../Core/Inc/DRIVERS/CK_MICROCARD.c \
../Core/Inc/DRIVERS/CK_RGB.c \
../Core/Inc/DRIVERS/CK_SOFTSERIAL.c \
../Core/Inc/DRIVERS/CK_SPI.c \
../Core/Inc/DRIVERS/CK_SPI_DMA.c \
../Core/Inc/DRIVERS/CK_SYSTEM.c \
../Core/Inc/DRIVERS/CK_TIME_HAL.c \
../Core/Inc/DRIVERS/CK_UART.c 

OBJS += \
./Core/Inc/DRIVERS/CK_ADC.o \
./Core/Inc/DRIVERS/CK_BUZZER.o \
./Core/Inc/DRIVERS/CK_CIRCULARBUFFER.o \
./Core/Inc/DRIVERS/CK_GPIO.o \
./Core/Inc/DRIVERS/CK_I2C.o \
./Core/Inc/DRIVERS/CK_LED.o \
./Core/Inc/DRIVERS/CK_MICROCARD.o \
./Core/Inc/DRIVERS/CK_RGB.o \
./Core/Inc/DRIVERS/CK_SOFTSERIAL.o \
./Core/Inc/DRIVERS/CK_SPI.o \
./Core/Inc/DRIVERS/CK_SPI_DMA.o \
./Core/Inc/DRIVERS/CK_SYSTEM.o \
./Core/Inc/DRIVERS/CK_TIME_HAL.o \
./Core/Inc/DRIVERS/CK_UART.o 

C_DEPS += \
./Core/Inc/DRIVERS/CK_ADC.d \
./Core/Inc/DRIVERS/CK_BUZZER.d \
./Core/Inc/DRIVERS/CK_CIRCULARBUFFER.d \
./Core/Inc/DRIVERS/CK_GPIO.d \
./Core/Inc/DRIVERS/CK_I2C.d \
./Core/Inc/DRIVERS/CK_LED.d \
./Core/Inc/DRIVERS/CK_MICROCARD.d \
./Core/Inc/DRIVERS/CK_RGB.d \
./Core/Inc/DRIVERS/CK_SOFTSERIAL.d \
./Core/Inc/DRIVERS/CK_SPI.d \
./Core/Inc/DRIVERS/CK_SPI_DMA.d \
./Core/Inc/DRIVERS/CK_SYSTEM.d \
./Core/Inc/DRIVERS/CK_TIME_HAL.d \
./Core/Inc/DRIVERS/CK_UART.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/DRIVERS/%.o Core/Inc/DRIVERS/%.su Core/Inc/DRIVERS/%.cyclo: ../Core/Inc/DRIVERS/%.c Core/Inc/DRIVERS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-DRIVERS

clean-Core-2f-Inc-2f-DRIVERS:
	-$(RM) ./Core/Inc/DRIVERS/CK_ADC.cyclo ./Core/Inc/DRIVERS/CK_ADC.d ./Core/Inc/DRIVERS/CK_ADC.o ./Core/Inc/DRIVERS/CK_ADC.su ./Core/Inc/DRIVERS/CK_BUZZER.cyclo ./Core/Inc/DRIVERS/CK_BUZZER.d ./Core/Inc/DRIVERS/CK_BUZZER.o ./Core/Inc/DRIVERS/CK_BUZZER.su ./Core/Inc/DRIVERS/CK_CIRCULARBUFFER.cyclo ./Core/Inc/DRIVERS/CK_CIRCULARBUFFER.d ./Core/Inc/DRIVERS/CK_CIRCULARBUFFER.o ./Core/Inc/DRIVERS/CK_CIRCULARBUFFER.su ./Core/Inc/DRIVERS/CK_GPIO.cyclo ./Core/Inc/DRIVERS/CK_GPIO.d ./Core/Inc/DRIVERS/CK_GPIO.o ./Core/Inc/DRIVERS/CK_GPIO.su ./Core/Inc/DRIVERS/CK_I2C.cyclo ./Core/Inc/DRIVERS/CK_I2C.d ./Core/Inc/DRIVERS/CK_I2C.o ./Core/Inc/DRIVERS/CK_I2C.su ./Core/Inc/DRIVERS/CK_LED.cyclo ./Core/Inc/DRIVERS/CK_LED.d ./Core/Inc/DRIVERS/CK_LED.o ./Core/Inc/DRIVERS/CK_LED.su ./Core/Inc/DRIVERS/CK_MICROCARD.cyclo ./Core/Inc/DRIVERS/CK_MICROCARD.d ./Core/Inc/DRIVERS/CK_MICROCARD.o ./Core/Inc/DRIVERS/CK_MICROCARD.su ./Core/Inc/DRIVERS/CK_RGB.cyclo ./Core/Inc/DRIVERS/CK_RGB.d ./Core/Inc/DRIVERS/CK_RGB.o ./Core/Inc/DRIVERS/CK_RGB.su ./Core/Inc/DRIVERS/CK_SOFTSERIAL.cyclo ./Core/Inc/DRIVERS/CK_SOFTSERIAL.d ./Core/Inc/DRIVERS/CK_SOFTSERIAL.o ./Core/Inc/DRIVERS/CK_SOFTSERIAL.su ./Core/Inc/DRIVERS/CK_SPI.cyclo ./Core/Inc/DRIVERS/CK_SPI.d ./Core/Inc/DRIVERS/CK_SPI.o ./Core/Inc/DRIVERS/CK_SPI.su ./Core/Inc/DRIVERS/CK_SPI_DMA.cyclo ./Core/Inc/DRIVERS/CK_SPI_DMA.d ./Core/Inc/DRIVERS/CK_SPI_DMA.o ./Core/Inc/DRIVERS/CK_SPI_DMA.su ./Core/Inc/DRIVERS/CK_SYSTEM.cyclo ./Core/Inc/DRIVERS/CK_SYSTEM.d ./Core/Inc/DRIVERS/CK_SYSTEM.o ./Core/Inc/DRIVERS/CK_SYSTEM.su ./Core/Inc/DRIVERS/CK_TIME_HAL.cyclo ./Core/Inc/DRIVERS/CK_TIME_HAL.d ./Core/Inc/DRIVERS/CK_TIME_HAL.o ./Core/Inc/DRIVERS/CK_TIME_HAL.su ./Core/Inc/DRIVERS/CK_UART.cyclo ./Core/Inc/DRIVERS/CK_UART.d ./Core/Inc/DRIVERS/CK_UART.o ./Core/Inc/DRIVERS/CK_UART.su

.PHONY: clean-Core-2f-Inc-2f-DRIVERS

