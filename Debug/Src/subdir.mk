################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ForwardKinematics.c \
../Src/InverseKinematics.c \
../Src/Trajectory.c \
../Src/can.c \
../Src/dma.c \
../Src/fmpi2c.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/matrix_dyngsy.c \
../Src/spi.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f4xx.c \
../Src/usart.c 

OBJS += \
./Src/ForwardKinematics.o \
./Src/InverseKinematics.o \
./Src/Trajectory.o \
./Src/can.o \
./Src/dma.o \
./Src/fmpi2c.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/matrix_dyngsy.o \
./Src/spi.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f4xx.o \
./Src/usart.o 

C_DEPS += \
./Src/ForwardKinematics.d \
./Src/InverseKinematics.d \
./Src/Trajectory.d \
./Src/can.d \
./Src/dma.d \
./Src/fmpi2c.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/matrix_dyngsy.d \
./Src/spi.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f4xx.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ddd/Desktop/driver/HAL-062/SUB-HAL-062-manipboard/HAL-062_manipboard/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/ForwardKinematics.d ./Src/ForwardKinematics.o ./Src/ForwardKinematics.su ./Src/InverseKinematics.d ./Src/InverseKinematics.o ./Src/InverseKinematics.su ./Src/Trajectory.d ./Src/Trajectory.o ./Src/Trajectory.su ./Src/can.d ./Src/can.o ./Src/can.su ./Src/dma.d ./Src/dma.o ./Src/dma.su ./Src/fmpi2c.d ./Src/fmpi2c.o ./Src/fmpi2c.su ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/matrix_dyngsy.d ./Src/matrix_dyngsy.o ./Src/matrix_dyngsy.su ./Src/spi.d ./Src/spi.o ./Src/spi.su ./Src/stm32f4xx_hal_msp.d ./Src/stm32f4xx_hal_msp.o ./Src/stm32f4xx_hal_msp.su ./Src/stm32f4xx_it.d ./Src/stm32f4xx_it.o ./Src/stm32f4xx_it.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su ./Src/usart.d ./Src/usart.o ./Src/usart.su

.PHONY: clean-Src

