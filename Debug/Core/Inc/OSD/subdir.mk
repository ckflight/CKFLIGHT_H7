################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/OSD/CK_MAX7456.c \
../Core/Inc/OSD/CK_MSP_OSD.c \
../Core/Inc/OSD/CK_OSD.c 

OBJS += \
./Core/Inc/OSD/CK_MAX7456.o \
./Core/Inc/OSD/CK_MSP_OSD.o \
./Core/Inc/OSD/CK_OSD.o 

C_DEPS += \
./Core/Inc/OSD/CK_MAX7456.d \
./Core/Inc/OSD/CK_MSP_OSD.d \
./Core/Inc/OSD/CK_OSD.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/OSD/%.o Core/Inc/OSD/%.su Core/Inc/OSD/%.cyclo: ../Core/Inc/OSD/%.c Core/Inc/OSD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-OSD

clean-Core-2f-Inc-2f-OSD:
	-$(RM) ./Core/Inc/OSD/CK_MAX7456.cyclo ./Core/Inc/OSD/CK_MAX7456.d ./Core/Inc/OSD/CK_MAX7456.o ./Core/Inc/OSD/CK_MAX7456.su ./Core/Inc/OSD/CK_MSP_OSD.cyclo ./Core/Inc/OSD/CK_MSP_OSD.d ./Core/Inc/OSD/CK_MSP_OSD.o ./Core/Inc/OSD/CK_MSP_OSD.su ./Core/Inc/OSD/CK_OSD.cyclo ./Core/Inc/OSD/CK_OSD.d ./Core/Inc/OSD/CK_OSD.o ./Core/Inc/OSD/CK_OSD.su

.PHONY: clean-Core-2f-Inc-2f-OSD

