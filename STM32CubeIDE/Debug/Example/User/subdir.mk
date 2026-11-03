################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/main.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/stm32u5xx_hal_msp.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/stm32u5xx_it.c \
../Example/User/syscalls.c \
../Example/User/sysmem.c 

OBJS += \
./Example/User/main.o \
./Example/User/stm32u5xx_hal_msp.o \
./Example/User/stm32u5xx_it.o \
./Example/User/syscalls.o \
./Example/User/sysmem.o 

C_DEPS += \
./Example/User/main.d \
./Example/User/stm32u5xx_hal_msp.d \
./Example/User/stm32u5xx_it.d \
./Example/User/syscalls.d \
./Example/User/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Example/User/main.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/main.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Example/User/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32u5xx_hal_msp.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/stm32u5xx_hal_msp.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Example/User/stm32u5xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32u5xx_it.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/stm32u5xx_it.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Example/User/stm32u5xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/%.o Example/User/%.su Example/User/%.cyclo: ../Example/User/%.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Example-2f-User

clean-Example-2f-User:
	-$(RM) ./Example/User/main.cyclo ./Example/User/main.d ./Example/User/main.o ./Example/User/main.su ./Example/User/stm32u5xx_hal_msp.cyclo ./Example/User/stm32u5xx_hal_msp.d ./Example/User/stm32u5xx_hal_msp.o ./Example/User/stm32u5xx_hal_msp.su ./Example/User/stm32u5xx_it.cyclo ./Example/User/stm32u5xx_it.d ./Example/User/stm32u5xx_it.o ./Example/User/stm32u5xx_it.su ./Example/User/syscalls.cyclo ./Example/User/syscalls.d ./Example/User/syscalls.o ./Example/User/syscalls.su ./Example/User/sysmem.cyclo ./Example/User/sysmem.d ./Example/User/sysmem.o ./Example/User/sysmem.su

.PHONY: clean-Example-2f-User

