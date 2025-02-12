################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UDEV_V1_libs/RS485.c \
../UDEV_V1_libs/UDEV1_LEDcontrol.c \
../UDEV_V1_libs/UDEV1_LEDpresets.c \
../UDEV_V1_libs/UDEV1_RS485.c \
../UDEV_V1_libs/UNICARD1_lib.c 

OBJS += \
./UDEV_V1_libs/RS485.o \
./UDEV_V1_libs/UDEV1_LEDcontrol.o \
./UDEV_V1_libs/UDEV1_LEDpresets.o \
./UDEV_V1_libs/UDEV1_RS485.o \
./UDEV_V1_libs/UNICARD1_lib.o 

C_DEPS += \
./UDEV_V1_libs/RS485.d \
./UDEV_V1_libs/UDEV1_LEDcontrol.d \
./UDEV_V1_libs/UDEV1_LEDpresets.d \
./UDEV_V1_libs/UDEV1_RS485.d \
./UDEV_V1_libs/UNICARD1_lib.d 


# Each subdirectory must supply rules for building sources it contributes
UDEV_V1_libs/%.o UDEV_V1_libs/%.su UDEV_V1_libs/%.cyclo: ../UDEV_V1_libs/%.c UDEV_V1_libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_SNK -DUSBPDCORE_LIB_PD3_FULL -D_RTOS -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USBPD/App -I../USBPD -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G4XX/inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Admin/git/UniLink-firmware/Projects/Uniracks/UDEV V1/UNICARD_Dev_Tool_V1/UDEV_V1_libs" -I"C:/Users/Admin/git/UniLink-firmware/libraries/General C" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UDEV_V1_libs

clean-UDEV_V1_libs:
	-$(RM) ./UDEV_V1_libs/RS485.cyclo ./UDEV_V1_libs/RS485.d ./UDEV_V1_libs/RS485.o ./UDEV_V1_libs/RS485.su ./UDEV_V1_libs/UDEV1_LEDcontrol.cyclo ./UDEV_V1_libs/UDEV1_LEDcontrol.d ./UDEV_V1_libs/UDEV1_LEDcontrol.o ./UDEV_V1_libs/UDEV1_LEDcontrol.su ./UDEV_V1_libs/UDEV1_LEDpresets.cyclo ./UDEV_V1_libs/UDEV1_LEDpresets.d ./UDEV_V1_libs/UDEV1_LEDpresets.o ./UDEV_V1_libs/UDEV1_LEDpresets.su ./UDEV_V1_libs/UDEV1_RS485.cyclo ./UDEV_V1_libs/UDEV1_RS485.d ./UDEV_V1_libs/UDEV1_RS485.o ./UDEV_V1_libs/UDEV1_RS485.su ./UDEV_V1_libs/UNICARD1_lib.cyclo ./UDEV_V1_libs/UNICARD1_lib.d ./UDEV_V1_libs/UNICARD1_lib.o ./UDEV_V1_libs/UNICARD1_lib.su

.PHONY: clean-UDEV_V1_libs

