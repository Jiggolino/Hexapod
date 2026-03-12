################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IMU/LSM6DSO/lsm6dso_reg.c 

OBJS += \
./Drivers/IMU/LSM6DSO/lsm6dso_reg.o 

C_DEPS += \
./Drivers/IMU/LSM6DSO/lsm6dso_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IMU/LSM6DSO/%.o Drivers/IMU/LSM6DSO/%.su Drivers/IMU/LSM6DSO/%.cyclo: ../Drivers/IMU/LSM6DSO/%.c Drivers/IMU/LSM6DSO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IMU-2f-LSM6DSO

clean-Drivers-2f-IMU-2f-LSM6DSO:
	-$(RM) ./Drivers/IMU/LSM6DSO/lsm6dso_reg.cyclo ./Drivers/IMU/LSM6DSO/lsm6dso_reg.d ./Drivers/IMU/LSM6DSO/lsm6dso_reg.o ./Drivers/IMU/LSM6DSO/lsm6dso_reg.su

.PHONY: clean-Drivers-2f-IMU-2f-LSM6DSO

