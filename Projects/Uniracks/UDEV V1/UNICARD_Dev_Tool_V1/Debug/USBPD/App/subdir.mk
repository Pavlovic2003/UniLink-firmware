################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USBPD/App/usbpd.c 

OBJS += \
./USBPD/App/usbpd.o 

C_DEPS += \
./USBPD/App/usbpd.d 


# Each subdirectory must supply rules for building sources it contributes
USBPD/App/%.o USBPD/App/%.su USBPD/App/%.cyclo: ../USBPD/App/%.c USBPD/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_SNK -DUSBPDCORE_LIB_PD3_FULL -D_RTOS -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USBPD/App -I../USBPD -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G4XX/inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Admin/git/UniLink-firmware/Projects/Uniracks/UDEV V1/UNICARD_Dev_Tool_V1/UDEV_V1_libs" -I"C:/Users/Admin/git/UniLink-firmware/libraries/General C" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USBPD-2f-App

clean-USBPD-2f-App:
	-$(RM) ./USBPD/App/usbpd.cyclo ./USBPD/App/usbpd.d ./USBPD/App/usbpd.o ./USBPD/App/usbpd.su

.PHONY: clean-USBPD-2f-App

