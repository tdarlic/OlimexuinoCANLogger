################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o: ../Middlewares/Third_Party/FatFs/src/option/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DSTM32F103xB -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/CMSIS/Include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FatFs/src" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Inc" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


