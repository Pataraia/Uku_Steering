################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/can.c \
../Src/gpio.c \
../Src/main.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/system_stm32f3xx.c \
../Src/tim.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/adc.o \
./Src/can.o \
./Src/gpio.o \
./Src/main.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/system_stm32f3xx.o \
./Src/tim.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/adc.d \
./Src/can.d \
./Src/gpio.d \
./Src/main.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/system_stm32f3xx.d \
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
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F303xC '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -I"C:/Users/Iseauto/Desktop/SteeringPWM/Inc" -I"C:/Users/Iseauto/Desktop/SteeringPWM/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/Iseauto/Desktop/SteeringPWM/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Iseauto/Desktop/SteeringPWM/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/Iseauto/Desktop/SteeringPWM/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/Iseauto/Desktop/SteeringPWM/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/Iseauto/Desktop/SteeringPWM/Drivers/CMSIS/Include"  -O2 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


