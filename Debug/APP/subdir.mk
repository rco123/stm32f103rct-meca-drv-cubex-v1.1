################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/app_bat.c \
../APP/app_math.c \
../APP/app_motion.c \
../APP/app_pid.c \
../APP/cJSON.c \
../APP/debug.c \
../APP/protocol.c 

OBJS += \
./APP/app_bat.o \
./APP/app_math.o \
./APP/app_motion.o \
./APP/app_pid.o \
./APP/cJSON.o \
./APP/debug.o \
./APP/protocol.o 

C_DEPS += \
./APP/app_bat.d \
./APP/app_math.d \
./APP/app_motion.d \
./APP/app_pid.d \
./APP/cJSON.d \
./APP/debug.d \
./APP/protocol.d 


# Each subdirectory must supply rules for building sources it contributes
APP/%.o APP/%.su APP/%.cyclo: ../APP/%.c APP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/BSP" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/APP" -I"C:/Users/sgkim/Desktop/stm32f103rct-meca-drv-cubex-v1.1/EMPL" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-APP

clean-APP:
	-$(RM) ./APP/app_bat.cyclo ./APP/app_bat.d ./APP/app_bat.o ./APP/app_bat.su ./APP/app_math.cyclo ./APP/app_math.d ./APP/app_math.o ./APP/app_math.su ./APP/app_motion.cyclo ./APP/app_motion.d ./APP/app_motion.o ./APP/app_motion.su ./APP/app_pid.cyclo ./APP/app_pid.d ./APP/app_pid.o ./APP/app_pid.su ./APP/cJSON.cyclo ./APP/cJSON.d ./APP/cJSON.o ./APP/cJSON.su ./APP/debug.cyclo ./APP/debug.d ./APP/debug.o ./APP/debug.su ./APP/protocol.cyclo ./APP/protocol.d ./APP/protocol.o ./APP/protocol.su

.PHONY: clean-APP

