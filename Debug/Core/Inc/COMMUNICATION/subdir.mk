################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/COMMUNICATION/CK_BLHELIPASS.c \
../Core/Inc/COMMUNICATION/CK_CONFIGURATION.c \
../Core/Inc/COMMUNICATION/CK_INPUTSTREAM.c \
../Core/Inc/COMMUNICATION/CK_MSP.c \
../Core/Inc/COMMUNICATION/CK_PRINTER.c 

OBJS += \
./Core/Inc/COMMUNICATION/CK_BLHELIPASS.o \
./Core/Inc/COMMUNICATION/CK_CONFIGURATION.o \
./Core/Inc/COMMUNICATION/CK_INPUTSTREAM.o \
./Core/Inc/COMMUNICATION/CK_MSP.o \
./Core/Inc/COMMUNICATION/CK_PRINTER.o 

C_DEPS += \
./Core/Inc/COMMUNICATION/CK_BLHELIPASS.d \
./Core/Inc/COMMUNICATION/CK_CONFIGURATION.d \
./Core/Inc/COMMUNICATION/CK_INPUTSTREAM.d \
./Core/Inc/COMMUNICATION/CK_MSP.d \
./Core/Inc/COMMUNICATION/CK_PRINTER.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/COMMUNICATION/%.o Core/Inc/COMMUNICATION/%.su Core/Inc/COMMUNICATION/%.cyclo: ../Core/Inc/COMMUNICATION/%.c Core/Inc/COMMUNICATION/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-COMMUNICATION

clean-Core-2f-Inc-2f-COMMUNICATION:
	-$(RM) ./Core/Inc/COMMUNICATION/CK_BLHELIPASS.cyclo ./Core/Inc/COMMUNICATION/CK_BLHELIPASS.d ./Core/Inc/COMMUNICATION/CK_BLHELIPASS.o ./Core/Inc/COMMUNICATION/CK_BLHELIPASS.su ./Core/Inc/COMMUNICATION/CK_CONFIGURATION.cyclo ./Core/Inc/COMMUNICATION/CK_CONFIGURATION.d ./Core/Inc/COMMUNICATION/CK_CONFIGURATION.o ./Core/Inc/COMMUNICATION/CK_CONFIGURATION.su ./Core/Inc/COMMUNICATION/CK_INPUTSTREAM.cyclo ./Core/Inc/COMMUNICATION/CK_INPUTSTREAM.d ./Core/Inc/COMMUNICATION/CK_INPUTSTREAM.o ./Core/Inc/COMMUNICATION/CK_INPUTSTREAM.su ./Core/Inc/COMMUNICATION/CK_MSP.cyclo ./Core/Inc/COMMUNICATION/CK_MSP.d ./Core/Inc/COMMUNICATION/CK_MSP.o ./Core/Inc/COMMUNICATION/CK_MSP.su ./Core/Inc/COMMUNICATION/CK_PRINTER.cyclo ./Core/Inc/COMMUNICATION/CK_PRINTER.d ./Core/Inc/COMMUNICATION/CK_PRINTER.o ./Core/Inc/COMMUNICATION/CK_PRINTER.su

.PHONY: clean-Core-2f-Inc-2f-COMMUNICATION

