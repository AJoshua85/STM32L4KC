################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/STM32L4x_gpio_driver.c \
../Drivers/Src/STM32L4x_i2c_driver.c \
../Drivers/Src/STM34L4x_spi_driver.c 

OBJS += \
./Drivers/Src/STM32L4x_gpio_driver.o \
./Drivers/Src/STM32L4x_i2c_driver.o \
./Drivers/Src/STM34L4x_spi_driver.o 

C_DEPS += \
./Drivers/Src/STM32L4x_gpio_driver.d \
./Drivers/Src/STM32L4x_i2c_driver.d \
./Drivers/Src/STM34L4x_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/STM32L4x_gpio_driver.o: ../Drivers/Src/STM32L4x_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_L432KC -DSTM32L4 -DSTM32 -DSTM32L432KCUx -DDEBUG -c -I../Inc -I"C:/Users/Avinash/STM32CubeIDE/workspace_1.2.0/STM32L4KC/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/STM32L4x_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/STM32L4x_i2c_driver.o: ../Drivers/Src/STM32L4x_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_L432KC -DSTM32L4 -DSTM32 -DSTM32L432KCUx -DDEBUG -c -I../Inc -I"C:/Users/Avinash/STM32CubeIDE/workspace_1.2.0/STM32L4KC/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/STM32L4x_i2c_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/STM34L4x_spi_driver.o: ../Drivers/Src/STM34L4x_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_L432KC -DSTM32L4 -DSTM32 -DSTM32L432KCUx -DDEBUG -c -I../Inc -I"C:/Users/Avinash/STM32CubeIDE/workspace_1.2.0/STM32L4KC/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/STM34L4x_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

