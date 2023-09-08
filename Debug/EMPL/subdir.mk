################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../EMPL/inv_mpu.c \
../EMPL/inv_mpu_dmp_motion_driver.c 

OBJS += \
./EMPL/inv_mpu.o \
./EMPL/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./EMPL/inv_mpu.d \
./EMPL/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
EMPL/%.o EMPL/%.su EMPL/%.cyclo: ../EMPL/%.c EMPL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/APP" -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/EMPL" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-EMPL

clean-EMPL:
	-$(RM) ./EMPL/inv_mpu.cyclo ./EMPL/inv_mpu.d ./EMPL/inv_mpu.o ./EMPL/inv_mpu.su ./EMPL/inv_mpu_dmp_motion_driver.cyclo ./EMPL/inv_mpu_dmp_motion_driver.d ./EMPL/inv_mpu_dmp_motion_driver.o ./EMPL/inv_mpu_dmp_motion_driver.su

.PHONY: clean-EMPL

