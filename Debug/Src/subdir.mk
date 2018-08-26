################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/fatfs.c \
../Src/freertos.c \
../Src/main.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_hal_timebase_TIM.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/user_diskio.c 

OBJS += \
./Src/fatfs.o \
./Src/freertos.o \
./Src/main.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_hal_timebase_TIM.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/user_diskio.o 

C_DEPS += \
./Src/fatfs.d \
./Src/freertos.d \
./Src/main.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_hal_timebase_TIM.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/user_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DSTM32F103xB -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/CMSIS/Include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FatFs/src" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Inc" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/tdarlic/STM32Toolchain/projects/OlimexuinoFreeRtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


