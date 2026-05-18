################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/FLASH/CK_FLASH.c \
../Core/Inc/FLASH/CK_FLASH_INTERNAL.c \
../Core/Inc/FLASH/CK_FLASH_W25Q128FV.c 

OBJS += \
./Core/Inc/FLASH/CK_FLASH.o \
./Core/Inc/FLASH/CK_FLASH_INTERNAL.o \
./Core/Inc/FLASH/CK_FLASH_W25Q128FV.o 

C_DEPS += \
./Core/Inc/FLASH/CK_FLASH.d \
./Core/Inc/FLASH/CK_FLASH_INTERNAL.d \
./Core/Inc/FLASH/CK_FLASH_W25Q128FV.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/FLASH/%.o Core/Inc/FLASH/%.su Core/Inc/FLASH/%.cyclo: ../Core/Inc/FLASH/%.c Core/Inc/FLASH/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-FLASH

clean-Core-2f-Inc-2f-FLASH:
	-$(RM) ./Core/Inc/FLASH/CK_FLASH.cyclo ./Core/Inc/FLASH/CK_FLASH.d ./Core/Inc/FLASH/CK_FLASH.o ./Core/Inc/FLASH/CK_FLASH.su ./Core/Inc/FLASH/CK_FLASH_INTERNAL.cyclo ./Core/Inc/FLASH/CK_FLASH_INTERNAL.d ./Core/Inc/FLASH/CK_FLASH_INTERNAL.o ./Core/Inc/FLASH/CK_FLASH_INTERNAL.su ./Core/Inc/FLASH/CK_FLASH_W25Q128FV.cyclo ./Core/Inc/FLASH/CK_FLASH_W25Q128FV.d ./Core/Inc/FLASH/CK_FLASH_W25Q128FV.o ./Core/Inc/FLASH/CK_FLASH_W25Q128FV.su

.PHONY: clean-Core-2f-Inc-2f-FLASH

