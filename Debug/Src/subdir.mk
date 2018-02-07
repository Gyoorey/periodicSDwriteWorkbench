################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ADXL345.c \
../Src/Communication.c \
../Src/adc.c \
../Src/bsp_driver_sd.c \
../Src/dac.c \
../Src/dma.c \
../Src/fatfs.c \
../Src/fatfs_platform.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/opamp.c \
../Src/rng.c \
../Src/rtc.c \
../Src/sd_diskio.c \
../Src/sdmmc.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c \
../Src/tim.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/ADXL345.o \
./Src/Communication.o \
./Src/adc.o \
./Src/bsp_driver_sd.o \
./Src/dac.o \
./Src/dma.o \
./Src/fatfs.o \
./Src/fatfs_platform.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/opamp.o \
./Src/rng.o \
./Src/rtc.o \
./Src/sd_diskio.o \
./Src/sdmmc.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o \
./Src/tim.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/ADXL345.d \
./Src/Communication.d \
./Src/adc.d \
./Src/bsp_driver_sd.d \
./Src/dac.d \
./Src/dma.d \
./Src/fatfs.d \
./Src/fatfs_platform.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/opamp.d \
./Src/rng.d \
./Src/rtc.d \
./Src/sd_diskio.d \
./Src/sdmmc.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d \
./Src/tim.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Inc" -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Middlewares/Third_Party/FatFs/src" -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Drivers/CMSIS/Include" -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Keil_v5/Projects/shooting/periodicSDwriteWorkbench/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


