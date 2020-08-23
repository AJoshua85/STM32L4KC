################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MasterTransRecv.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/MasterTransRecv.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/MasterTransRecv.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/MasterTransRecv.o: ../Src/MasterTransRecv.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_L432KC -DSTM32L4 -DSTM32 -DSTM32L432KCUx -DDEBUG -c -I../Inc -I"C:/Users/Avinash/STM32CubeIDE/workspace_1.2.0/STM32L4KC/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/MasterTransRecv.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_L432KC -DSTM32L4 -DSTM32 -DSTM32L432KCUx -DDEBUG -c -I../Inc -I"C:/Users/Avinash/STM32CubeIDE/workspace_1.2.0/STM32L4KC/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DNUCLEO_L432KC -DSTM32L4 -DSTM32 -DSTM32L432KCUx -DDEBUG -c -I../Inc -I"C:/Users/Avinash/STM32CubeIDE/workspace_1.2.0/STM32L4KC/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

