################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_WPAN/ble/core/template/osal.c 

OBJS += \
./Middlewares/ST/STM32_WPAN/ble/core/template/osal.o 

C_DEPS += \
./Middlewares/ST/STM32_WPAN/ble/core/template/osal.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_WPAN/ble/core/template/%.o Middlewares/ST/STM32_WPAN/ble/core/template/%.su Middlewares/ST/STM32_WPAN/ble/core/template/%.cyclo: ../Middlewares/ST/STM32_WPAN/ble/core/template/%.c Middlewares/ST/STM32_WPAN/ble/core/template/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB15xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-ble-2f-core-2f-template

clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-ble-2f-core-2f-template:
	-$(RM) ./Middlewares/ST/STM32_WPAN/ble/core/template/osal.cyclo ./Middlewares/ST/STM32_WPAN/ble/core/template/osal.d ./Middlewares/ST/STM32_WPAN/ble/core/template/osal.o ./Middlewares/ST/STM32_WPAN/ble/core/template/osal.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-ble-2f-core-2f-template

