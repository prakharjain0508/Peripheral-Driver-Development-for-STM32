################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/014uart_case.c 

OBJS += \
./Src/014uart_case.o 

C_DEPS += \
./Src/014uart_case.d 


# Each subdirectory must supply rules for building sources it contributes
Src/014uart_case.o: ../Src/014uart_case.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"D:/MS/E-Courses/Microcontroller and Embedded Driver Development/Projects/MCU1/stm32f446xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/014uart_case.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

