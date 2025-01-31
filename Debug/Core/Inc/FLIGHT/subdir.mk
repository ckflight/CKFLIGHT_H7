################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/FLIGHT/CK_ADJUSTMENT.c \
../Core/Inc/FLIGHT/CK_ALTITUDE.c \
../Core/Inc/FLIGHT/CK_CRSF.c \
../Core/Inc/FLIGHT/CK_DSHOT.c \
../Core/Inc/FLIGHT/CK_ESC.c \
../Core/Inc/FLIGHT/CK_FAULTHANDLER.c \
../Core/Inc/FLIGHT/CK_GPS.c \
../Core/Inc/FLIGHT/CK_LAND.c \
../Core/Inc/FLIGHT/CK_LOG.c \
../Core/Inc/FLIGHT/CK_MIXER.c \
../Core/Inc/FLIGHT/CK_NAVIGATION.c \
../Core/Inc/FLIGHT/CK_PERIPHERAL.c \
../Core/Inc/FLIGHT/CK_PID.c \
../Core/Inc/FLIGHT/CK_PWM.c \
../Core/Inc/FLIGHT/CK_RC.c \
../Core/Inc/FLIGHT/CK_RECEIVER.c \
../Core/Inc/FLIGHT/CK_SBUS.c \
../Core/Inc/FLIGHT/CK_SMARTAUDIO.c \
../Core/Inc/FLIGHT/pid_init.c \
../Core/Inc/FLIGHT/simplified_tuning.c 

OBJS += \
./Core/Inc/FLIGHT/CK_ADJUSTMENT.o \
./Core/Inc/FLIGHT/CK_ALTITUDE.o \
./Core/Inc/FLIGHT/CK_CRSF.o \
./Core/Inc/FLIGHT/CK_DSHOT.o \
./Core/Inc/FLIGHT/CK_ESC.o \
./Core/Inc/FLIGHT/CK_FAULTHANDLER.o \
./Core/Inc/FLIGHT/CK_GPS.o \
./Core/Inc/FLIGHT/CK_LAND.o \
./Core/Inc/FLIGHT/CK_LOG.o \
./Core/Inc/FLIGHT/CK_MIXER.o \
./Core/Inc/FLIGHT/CK_NAVIGATION.o \
./Core/Inc/FLIGHT/CK_PERIPHERAL.o \
./Core/Inc/FLIGHT/CK_PID.o \
./Core/Inc/FLIGHT/CK_PWM.o \
./Core/Inc/FLIGHT/CK_RC.o \
./Core/Inc/FLIGHT/CK_RECEIVER.o \
./Core/Inc/FLIGHT/CK_SBUS.o \
./Core/Inc/FLIGHT/CK_SMARTAUDIO.o \
./Core/Inc/FLIGHT/pid_init.o \
./Core/Inc/FLIGHT/simplified_tuning.o 

C_DEPS += \
./Core/Inc/FLIGHT/CK_ADJUSTMENT.d \
./Core/Inc/FLIGHT/CK_ALTITUDE.d \
./Core/Inc/FLIGHT/CK_CRSF.d \
./Core/Inc/FLIGHT/CK_DSHOT.d \
./Core/Inc/FLIGHT/CK_ESC.d \
./Core/Inc/FLIGHT/CK_FAULTHANDLER.d \
./Core/Inc/FLIGHT/CK_GPS.d \
./Core/Inc/FLIGHT/CK_LAND.d \
./Core/Inc/FLIGHT/CK_LOG.d \
./Core/Inc/FLIGHT/CK_MIXER.d \
./Core/Inc/FLIGHT/CK_NAVIGATION.d \
./Core/Inc/FLIGHT/CK_PERIPHERAL.d \
./Core/Inc/FLIGHT/CK_PID.d \
./Core/Inc/FLIGHT/CK_PWM.d \
./Core/Inc/FLIGHT/CK_RC.d \
./Core/Inc/FLIGHT/CK_RECEIVER.d \
./Core/Inc/FLIGHT/CK_SBUS.d \
./Core/Inc/FLIGHT/CK_SMARTAUDIO.d \
./Core/Inc/FLIGHT/pid_init.d \
./Core/Inc/FLIGHT/simplified_tuning.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/FLIGHT/%.o Core/Inc/FLIGHT/%.su Core/Inc/FLIGHT/%.cyclo: ../Core/Inc/FLIGHT/%.c Core/Inc/FLIGHT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-FLIGHT

clean-Core-2f-Inc-2f-FLIGHT:
	-$(RM) ./Core/Inc/FLIGHT/CK_ADJUSTMENT.cyclo ./Core/Inc/FLIGHT/CK_ADJUSTMENT.d ./Core/Inc/FLIGHT/CK_ADJUSTMENT.o ./Core/Inc/FLIGHT/CK_ADJUSTMENT.su ./Core/Inc/FLIGHT/CK_ALTITUDE.cyclo ./Core/Inc/FLIGHT/CK_ALTITUDE.d ./Core/Inc/FLIGHT/CK_ALTITUDE.o ./Core/Inc/FLIGHT/CK_ALTITUDE.su ./Core/Inc/FLIGHT/CK_CRSF.cyclo ./Core/Inc/FLIGHT/CK_CRSF.d ./Core/Inc/FLIGHT/CK_CRSF.o ./Core/Inc/FLIGHT/CK_CRSF.su ./Core/Inc/FLIGHT/CK_DSHOT.cyclo ./Core/Inc/FLIGHT/CK_DSHOT.d ./Core/Inc/FLIGHT/CK_DSHOT.o ./Core/Inc/FLIGHT/CK_DSHOT.su ./Core/Inc/FLIGHT/CK_ESC.cyclo ./Core/Inc/FLIGHT/CK_ESC.d ./Core/Inc/FLIGHT/CK_ESC.o ./Core/Inc/FLIGHT/CK_ESC.su ./Core/Inc/FLIGHT/CK_FAULTHANDLER.cyclo ./Core/Inc/FLIGHT/CK_FAULTHANDLER.d ./Core/Inc/FLIGHT/CK_FAULTHANDLER.o ./Core/Inc/FLIGHT/CK_FAULTHANDLER.su ./Core/Inc/FLIGHT/CK_GPS.cyclo ./Core/Inc/FLIGHT/CK_GPS.d ./Core/Inc/FLIGHT/CK_GPS.o ./Core/Inc/FLIGHT/CK_GPS.su ./Core/Inc/FLIGHT/CK_LAND.cyclo ./Core/Inc/FLIGHT/CK_LAND.d ./Core/Inc/FLIGHT/CK_LAND.o ./Core/Inc/FLIGHT/CK_LAND.su ./Core/Inc/FLIGHT/CK_LOG.cyclo ./Core/Inc/FLIGHT/CK_LOG.d ./Core/Inc/FLIGHT/CK_LOG.o ./Core/Inc/FLIGHT/CK_LOG.su ./Core/Inc/FLIGHT/CK_MIXER.cyclo ./Core/Inc/FLIGHT/CK_MIXER.d ./Core/Inc/FLIGHT/CK_MIXER.o ./Core/Inc/FLIGHT/CK_MIXER.su ./Core/Inc/FLIGHT/CK_NAVIGATION.cyclo ./Core/Inc/FLIGHT/CK_NAVIGATION.d ./Core/Inc/FLIGHT/CK_NAVIGATION.o ./Core/Inc/FLIGHT/CK_NAVIGATION.su ./Core/Inc/FLIGHT/CK_PERIPHERAL.cyclo ./Core/Inc/FLIGHT/CK_PERIPHERAL.d ./Core/Inc/FLIGHT/CK_PERIPHERAL.o ./Core/Inc/FLIGHT/CK_PERIPHERAL.su ./Core/Inc/FLIGHT/CK_PID.cyclo ./Core/Inc/FLIGHT/CK_PID.d ./Core/Inc/FLIGHT/CK_PID.o ./Core/Inc/FLIGHT/CK_PID.su ./Core/Inc/FLIGHT/CK_PWM.cyclo ./Core/Inc/FLIGHT/CK_PWM.d ./Core/Inc/FLIGHT/CK_PWM.o ./Core/Inc/FLIGHT/CK_PWM.su ./Core/Inc/FLIGHT/CK_RC.cyclo ./Core/Inc/FLIGHT/CK_RC.d ./Core/Inc/FLIGHT/CK_RC.o ./Core/Inc/FLIGHT/CK_RC.su ./Core/Inc/FLIGHT/CK_RECEIVER.cyclo ./Core/Inc/FLIGHT/CK_RECEIVER.d ./Core/Inc/FLIGHT/CK_RECEIVER.o ./Core/Inc/FLIGHT/CK_RECEIVER.su ./Core/Inc/FLIGHT/CK_SBUS.cyclo ./Core/Inc/FLIGHT/CK_SBUS.d ./Core/Inc/FLIGHT/CK_SBUS.o ./Core/Inc/FLIGHT/CK_SBUS.su ./Core/Inc/FLIGHT/CK_SMARTAUDIO.cyclo ./Core/Inc/FLIGHT/CK_SMARTAUDIO.d ./Core/Inc/FLIGHT/CK_SMARTAUDIO.o ./Core/Inc/FLIGHT/CK_SMARTAUDIO.su ./Core/Inc/FLIGHT/pid_init.cyclo ./Core/Inc/FLIGHT/pid_init.d ./Core/Inc/FLIGHT/pid_init.o ./Core/Inc/FLIGHT/pid_init.su ./Core/Inc/FLIGHT/simplified_tuning.cyclo ./Core/Inc/FLIGHT/simplified_tuning.d ./Core/Inc/FLIGHT/simplified_tuning.o ./Core/Inc/FLIGHT/simplified_tuning.su

.PHONY: clean-Core-2f-Inc-2f-FLIGHT

