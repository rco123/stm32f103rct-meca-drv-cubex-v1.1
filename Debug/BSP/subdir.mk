################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/bsp_encoder.c \
../BSP/bsp_motor.c \
../BSP/bsp_mpu9250.c \
../BSP/bsp_mpuiic.c \
../BSP/bsp_usart.c 

OBJS += \
./BSP/bsp_encoder.o \
./BSP/bsp_motor.o \
./BSP/bsp_mpu9250.o \
./BSP/bsp_mpuiic.o \
./BSP/bsp_usart.o 

C_DEPS += \
./BSP/bsp_encoder.d \
./BSP/bsp_motor.d \
./BSP/bsp_mpu9250.d \
./BSP/bsp_mpuiic.d \
./BSP/bsp_usart.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o BSP/%.su BSP/%.cyclo: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/APP" -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/EMPL" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/bsp_encoder.cyclo ./BSP/bsp_encoder.d ./BSP/bsp_encoder.o ./BSP/bsp_encoder.su ./BSP/bsp_motor.cyclo ./BSP/bsp_motor.d ./BSP/bsp_motor.o ./BSP/bsp_motor.su ./BSP/bsp_mpu9250.cyclo ./BSP/bsp_mpu9250.d ./BSP/bsp_mpu9250.o ./BSP/bsp_mpu9250.su ./BSP/bsp_mpuiic.cyclo ./BSP/bsp_mpuiic.d ./BSP/bsp_mpuiic.o ./BSP/bsp_mpuiic.su ./BSP/bsp_usart.cyclo ./BSP/bsp_usart.d ./BSP/bsp_usart.o ./BSP/bsp_usart.su

.PHONY: clean-BSP

