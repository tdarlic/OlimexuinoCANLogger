################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../startup/startup_stm32f103xb.S 

OBJS += \
./startup/startup_stm32f103xb.o 

S_UPPER_DEPS += \
./startup/startup_stm32f103xb.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -x assembler-with-cpp -DDEBUG -DSTM32F103xB -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/CMSIS/Include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FatFs/src" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Inc" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


