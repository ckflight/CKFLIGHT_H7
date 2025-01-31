################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/MOTION/CK_ACC.c \
../Core/Inc/MOTION/CK_BAROMETER.c \
../Core/Inc/MOTION/CK_BNO055.c \
../Core/Inc/MOTION/CK_GYRO.c \
../Core/Inc/MOTION/CK_IMU.c \
../Core/Inc/MOTION/CK_MAGNETO.c 

OBJS += \
./Core/Inc/MOTION/CK_ACC.o \
./Core/Inc/MOTION/CK_BAROMETER.o \
./Core/Inc/MOTION/CK_BNO055.o \
./Core/Inc/MOTION/CK_GYRO.o \
./Core/Inc/MOTION/CK_IMU.o \
./Core/Inc/MOTION/CK_MAGNETO.o 

C_DEPS += \
./Core/Inc/MOTION/CK_ACC.d \
./Core/Inc/MOTION/CK_BAROMETER.d \
./Core/Inc/MOTION/CK_BNO055.d \
./Core/Inc/MOTION/CK_GYRO.d \
./Core/Inc/MOTION/CK_IMU.d \
./Core/Inc/MOTION/CK_MAGNETO.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/MOTION/%.o Core/Inc/MOTION/%.su Core/Inc/MOTION/%.cyclo: ../Core/Inc/MOTION/%.c Core/Inc/MOTION/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-MOTION

clean-Core-2f-Inc-2f-MOTION:
	-$(RM) ./Core/Inc/MOTION/CK_ACC.cyclo ./Core/Inc/MOTION/CK_ACC.d ./Core/Inc/MOTION/CK_ACC.o ./Core/Inc/MOTION/CK_ACC.su ./Core/Inc/MOTION/CK_BAROMETER.cyclo ./Core/Inc/MOTION/CK_BAROMETER.d ./Core/Inc/MOTION/CK_BAROMETER.o ./Core/Inc/MOTION/CK_BAROMETER.su ./Core/Inc/MOTION/CK_BNO055.cyclo ./Core/Inc/MOTION/CK_BNO055.d ./Core/Inc/MOTION/CK_BNO055.o ./Core/Inc/MOTION/CK_BNO055.su ./Core/Inc/MOTION/CK_GYRO.cyclo ./Core/Inc/MOTION/CK_GYRO.d ./Core/Inc/MOTION/CK_GYRO.o ./Core/Inc/MOTION/CK_GYRO.su ./Core/Inc/MOTION/CK_IMU.cyclo ./Core/Inc/MOTION/CK_IMU.d ./Core/Inc/MOTION/CK_IMU.o ./Core/Inc/MOTION/CK_IMU.su ./Core/Inc/MOTION/CK_MAGNETO.cyclo ./Core/Inc/MOTION/CK_MAGNETO.d ./Core/Inc/MOTION/CK_MAGNETO.o ./Core/Inc/MOTION/CK_MAGNETO.su

.PHONY: clean-Core-2f-Inc-2f-MOTION

