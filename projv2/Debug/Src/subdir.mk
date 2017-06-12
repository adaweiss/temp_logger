################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/gpio.c \
../Src/lcd.c \
../Src/main.c \
../Src/qspi.c \
../Src/quadspi.c \
../Src/rtc.c \
../Src/sai.c \
../Src/stm32l476g_discovery.c \
../Src/stm32l476g_discovery_glass_lcd.c \
../Src/stm32l476g_discovery_qspi.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c \
../Src/usart.c 

OBJS += \
./Src/adc.o \
./Src/gpio.o \
./Src/lcd.o \
./Src/main.o \
./Src/qspi.o \
./Src/quadspi.o \
./Src/rtc.o \
./Src/sai.o \
./Src/stm32l476g_discovery.o \
./Src/stm32l476g_discovery_glass_lcd.o \
./Src/stm32l476g_discovery_qspi.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o \
./Src/usart.o 

C_DEPS += \
./Src/adc.d \
./Src/gpio.d \
./Src/lcd.d \
./Src/main.d \
./Src/qspi.d \
./Src/quadspi.d \
./Src/rtc.d \
./Src/sai.d \
./Src/stm32l476g_discovery.d \
./Src/stm32l476g_discovery_glass_lcd.d \
./Src/stm32l476g_discovery_qspi.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/Ada/Documents/STM/projv2/Inc" -I"C:/Users/Ada/Documents/STM/projv2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Ada/Documents/STM/projv2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Ada/Documents/STM/projv2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Ada/Documents/STM/projv2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


