################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USBPD/usbpd_dpm_core.c \
../USBPD/usbpd_dpm_user.c \
../USBPD/usbpd_pwr_if.c \
../USBPD/usbpd_pwr_user.c \
../USBPD/usbpd_vdm_user.c 

OBJS += \
./USBPD/usbpd_dpm_core.o \
./USBPD/usbpd_dpm_user.o \
./USBPD/usbpd_pwr_if.o \
./USBPD/usbpd_pwr_user.o \
./USBPD/usbpd_vdm_user.o 

C_DEPS += \
./USBPD/usbpd_dpm_core.d \
./USBPD/usbpd_dpm_user.d \
./USBPD/usbpd_pwr_if.d \
./USBPD/usbpd_pwr_user.d \
./USBPD/usbpd_vdm_user.d 


# Each subdirectory must supply rules for building sources it contributes
USBPD/%.o USBPD/%.su USBPD/%.cyclo: ../USBPD/%.c USBPD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_SNK -DUSBPDCORE_LIB_PD3_FULL -D_RTOS -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USBPD/App -I../USBPD -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G4XX/inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Admin/git/UDEV-V1/UNICARD_Dev_Tool_V0/UDEV_V1_libs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USBPD

clean-USBPD:
	-$(RM) ./USBPD/usbpd_dpm_core.cyclo ./USBPD/usbpd_dpm_core.d ./USBPD/usbpd_dpm_core.o ./USBPD/usbpd_dpm_core.su ./USBPD/usbpd_dpm_user.cyclo ./USBPD/usbpd_dpm_user.d ./USBPD/usbpd_dpm_user.o ./USBPD/usbpd_dpm_user.su ./USBPD/usbpd_pwr_if.cyclo ./USBPD/usbpd_pwr_if.d ./USBPD/usbpd_pwr_if.o ./USBPD/usbpd_pwr_if.su ./USBPD/usbpd_pwr_user.cyclo ./USBPD/usbpd_pwr_user.d ./USBPD/usbpd_pwr_user.o ./USBPD/usbpd_pwr_user.su ./USBPD/usbpd_vdm_user.cyclo ./USBPD/usbpd_vdm_user.d ./USBPD/usbpd_vdm_user.o ./USBPD/usbpd_vdm_user.su

.PHONY: clean-USBPD

