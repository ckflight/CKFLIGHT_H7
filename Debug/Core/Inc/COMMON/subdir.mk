################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/COMMON/CK_FILTERS.c \
../Core/Inc/COMMON/crc.c \
../Core/Inc/COMMON/maths.c \
../Core/Inc/COMMON/streambuf.c \
../Core/Inc/COMMON/vector.c 

OBJS += \
./Core/Inc/COMMON/CK_FILTERS.o \
./Core/Inc/COMMON/crc.o \
./Core/Inc/COMMON/maths.o \
./Core/Inc/COMMON/streambuf.o \
./Core/Inc/COMMON/vector.o 

C_DEPS += \
./Core/Inc/COMMON/CK_FILTERS.d \
./Core/Inc/COMMON/crc.d \
./Core/Inc/COMMON/maths.d \
./Core/Inc/COMMON/streambuf.d \
./Core/Inc/COMMON/vector.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/COMMON/%.o Core/Inc/COMMON/%.su Core/Inc/COMMON/%.cyclo: ../Core/Inc/COMMON/%.c Core/Inc/COMMON/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-COMMON

clean-Core-2f-Inc-2f-COMMON:
	-$(RM) ./Core/Inc/COMMON/CK_FILTERS.cyclo ./Core/Inc/COMMON/CK_FILTERS.d ./Core/Inc/COMMON/CK_FILTERS.o ./Core/Inc/COMMON/CK_FILTERS.su ./Core/Inc/COMMON/crc.cyclo ./Core/Inc/COMMON/crc.d ./Core/Inc/COMMON/crc.o ./Core/Inc/COMMON/crc.su ./Core/Inc/COMMON/maths.cyclo ./Core/Inc/COMMON/maths.d ./Core/Inc/COMMON/maths.o ./Core/Inc/COMMON/maths.su ./Core/Inc/COMMON/streambuf.cyclo ./Core/Inc/COMMON/streambuf.d ./Core/Inc/COMMON/streambuf.o ./Core/Inc/COMMON/streambuf.su ./Core/Inc/COMMON/vector.cyclo ./Core/Inc/COMMON/vector.d ./Core/Inc/COMMON/vector.o ./Core/Inc/COMMON/vector.su

.PHONY: clean-Core-2f-Inc-2f-COMMON

