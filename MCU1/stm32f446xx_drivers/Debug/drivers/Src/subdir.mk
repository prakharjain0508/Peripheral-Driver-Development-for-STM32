################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f446xx_gpio_driver.c \
../drivers/Src/stm32f446xx_i2c_driver.c \
../drivers/Src/stm32f446xx_rcc_driver.c \
../drivers/Src/stm32f446xx_spi_driver.c \
../drivers/Src/stm32f446xx_usart_driver.c 

OBJS += \
./drivers/Src/stm32f446xx_gpio_driver.o \
./drivers/Src/stm32f446xx_i2c_driver.o \
./drivers/Src/stm32f446xx_rcc_driver.o \
./drivers/Src/stm32f446xx_spi_driver.o \
./drivers/Src/stm32f446xx_usart_driver.o 

C_DEPS += \
./drivers/Src/stm32f446xx_gpio_driver.d \
./drivers/Src/stm32f446xx_i2c_driver.d \
./drivers/Src/stm32f446xx_rcc_driver.d \
./drivers/Src/stm32f446xx_spi_driver.d \
./drivers/Src/stm32f446xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f446xx_gpio_driver.o: ../drivers/Src/stm32f446xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"D:/MS/E-Courses/Microcontroller and Embedded Driver Development/Projects/MCU1/stm32f446xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f446xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f446xx_i2c_driver.o: ../drivers/Src/stm32f446xx_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"D:/MS/E-Courses/Microcontroller and Embedded Driver Development/Projects/MCU1/stm32f446xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f446xx_i2c_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f446xx_rcc_driver.o: ../drivers/Src/stm32f446xx_rcc_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"D:/MS/E-Courses/Microcontroller and Embedded Driver Development/Projects/MCU1/stm32f446xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f446xx_rcc_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f446xx_spi_driver.o: ../drivers/Src/stm32f446xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"D:/MS/E-Courses/Microcontroller and Embedded Driver Development/Projects/MCU1/stm32f446xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f446xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f446xx_usart_driver.o: ../drivers/Src/stm32f446xx_usart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"D:/MS/E-Courses/Microcontroller and Embedded Driver Development/Projects/MCU1/stm32f446xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f446xx_usart_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

