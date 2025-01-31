################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/SENSORS/CK_BMP280.c \
../Core/Inc/SENSORS/CK_FXOS8700CQ.c \
../Core/Inc/SENSORS/CK_HMC5983.c \
../Core/Inc/SENSORS/CK_ICM20602.c \
../Core/Inc/SENSORS/CK_ICM42605.c \
../Core/Inc/SENSORS/CK_ICM42688P.c \
../Core/Inc/SENSORS/CK_IIM42652.c \
../Core/Inc/SENSORS/CK_L3GD20H.c \
../Core/Inc/SENSORS/CK_LSM303D.c \
../Core/Inc/SENSORS/CK_MAG3110.c \
../Core/Inc/SENSORS/CK_MLX90393.c \
../Core/Inc/SENSORS/CK_MS5607.c \
../Core/Inc/SENSORS/CK_MS5611.c \
../Core/Inc/SENSORS/CK_QMC5883L.c 

OBJS += \
./Core/Inc/SENSORS/CK_BMP280.o \
./Core/Inc/SENSORS/CK_FXOS8700CQ.o \
./Core/Inc/SENSORS/CK_HMC5983.o \
./Core/Inc/SENSORS/CK_ICM20602.o \
./Core/Inc/SENSORS/CK_ICM42605.o \
./Core/Inc/SENSORS/CK_ICM42688P.o \
./Core/Inc/SENSORS/CK_IIM42652.o \
./Core/Inc/SENSORS/CK_L3GD20H.o \
./Core/Inc/SENSORS/CK_LSM303D.o \
./Core/Inc/SENSORS/CK_MAG3110.o \
./Core/Inc/SENSORS/CK_MLX90393.o \
./Core/Inc/SENSORS/CK_MS5607.o \
./Core/Inc/SENSORS/CK_MS5611.o \
./Core/Inc/SENSORS/CK_QMC5883L.o 

C_DEPS += \
./Core/Inc/SENSORS/CK_BMP280.d \
./Core/Inc/SENSORS/CK_FXOS8700CQ.d \
./Core/Inc/SENSORS/CK_HMC5983.d \
./Core/Inc/SENSORS/CK_ICM20602.d \
./Core/Inc/SENSORS/CK_ICM42605.d \
./Core/Inc/SENSORS/CK_ICM42688P.d \
./Core/Inc/SENSORS/CK_IIM42652.d \
./Core/Inc/SENSORS/CK_L3GD20H.d \
./Core/Inc/SENSORS/CK_LSM303D.d \
./Core/Inc/SENSORS/CK_MAG3110.d \
./Core/Inc/SENSORS/CK_MLX90393.d \
./Core/Inc/SENSORS/CK_MS5607.d \
./Core/Inc/SENSORS/CK_MS5611.d \
./Core/Inc/SENSORS/CK_QMC5883L.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/SENSORS/%.o Core/Inc/SENSORS/%.su Core/Inc/SENSORS/%.cyclo: ../Core/Inc/SENSORS/%.c Core/Inc/SENSORS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-SENSORS

clean-Core-2f-Inc-2f-SENSORS:
	-$(RM) ./Core/Inc/SENSORS/CK_BMP280.cyclo ./Core/Inc/SENSORS/CK_BMP280.d ./Core/Inc/SENSORS/CK_BMP280.o ./Core/Inc/SENSORS/CK_BMP280.su ./Core/Inc/SENSORS/CK_FXOS8700CQ.cyclo ./Core/Inc/SENSORS/CK_FXOS8700CQ.d ./Core/Inc/SENSORS/CK_FXOS8700CQ.o ./Core/Inc/SENSORS/CK_FXOS8700CQ.su ./Core/Inc/SENSORS/CK_HMC5983.cyclo ./Core/Inc/SENSORS/CK_HMC5983.d ./Core/Inc/SENSORS/CK_HMC5983.o ./Core/Inc/SENSORS/CK_HMC5983.su ./Core/Inc/SENSORS/CK_ICM20602.cyclo ./Core/Inc/SENSORS/CK_ICM20602.d ./Core/Inc/SENSORS/CK_ICM20602.o ./Core/Inc/SENSORS/CK_ICM20602.su ./Core/Inc/SENSORS/CK_ICM42605.cyclo ./Core/Inc/SENSORS/CK_ICM42605.d ./Core/Inc/SENSORS/CK_ICM42605.o ./Core/Inc/SENSORS/CK_ICM42605.su ./Core/Inc/SENSORS/CK_ICM42688P.cyclo ./Core/Inc/SENSORS/CK_ICM42688P.d ./Core/Inc/SENSORS/CK_ICM42688P.o ./Core/Inc/SENSORS/CK_ICM42688P.su ./Core/Inc/SENSORS/CK_IIM42652.cyclo ./Core/Inc/SENSORS/CK_IIM42652.d ./Core/Inc/SENSORS/CK_IIM42652.o ./Core/Inc/SENSORS/CK_IIM42652.su ./Core/Inc/SENSORS/CK_L3GD20H.cyclo ./Core/Inc/SENSORS/CK_L3GD20H.d ./Core/Inc/SENSORS/CK_L3GD20H.o ./Core/Inc/SENSORS/CK_L3GD20H.su ./Core/Inc/SENSORS/CK_LSM303D.cyclo ./Core/Inc/SENSORS/CK_LSM303D.d ./Core/Inc/SENSORS/CK_LSM303D.o ./Core/Inc/SENSORS/CK_LSM303D.su ./Core/Inc/SENSORS/CK_MAG3110.cyclo ./Core/Inc/SENSORS/CK_MAG3110.d ./Core/Inc/SENSORS/CK_MAG3110.o ./Core/Inc/SENSORS/CK_MAG3110.su ./Core/Inc/SENSORS/CK_MLX90393.cyclo ./Core/Inc/SENSORS/CK_MLX90393.d ./Core/Inc/SENSORS/CK_MLX90393.o ./Core/Inc/SENSORS/CK_MLX90393.su ./Core/Inc/SENSORS/CK_MS5607.cyclo ./Core/Inc/SENSORS/CK_MS5607.d ./Core/Inc/SENSORS/CK_MS5607.o ./Core/Inc/SENSORS/CK_MS5607.su ./Core/Inc/SENSORS/CK_MS5611.cyclo ./Core/Inc/SENSORS/CK_MS5611.d ./Core/Inc/SENSORS/CK_MS5611.o ./Core/Inc/SENSORS/CK_MS5611.su ./Core/Inc/SENSORS/CK_QMC5883L.cyclo ./Core/Inc/SENSORS/CK_QMC5883L.d ./Core/Inc/SENSORS/CK_QMC5883L.o ./Core/Inc/SENSORS/CK_QMC5883L.su

.PHONY: clean-Core-2f-Inc-2f-SENSORS

