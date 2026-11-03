################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_commands.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_main.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_scheduler.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_state.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/bsp_hw.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/console.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/fault_manager.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/ma782_encoder.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/motion_control.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/motor_driver.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/pi_controller.c \
F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/position_calc.c 

OBJS += \
./Application/Src/app_commands.o \
./Application/Src/app_main.o \
./Application/Src/app_scheduler.o \
./Application/Src/app_state.o \
./Application/Src/bsp_hw.o \
./Application/Src/console.o \
./Application/Src/fault_manager.o \
./Application/Src/ma782_encoder.o \
./Application/Src/motion_control.o \
./Application/Src/motor_driver.o \
./Application/Src/pi_controller.o \
./Application/Src/position_calc.o 

C_DEPS += \
./Application/Src/app_commands.d \
./Application/Src/app_main.d \
./Application/Src/app_scheduler.d \
./Application/Src/app_state.d \
./Application/Src/bsp_hw.d \
./Application/Src/console.d \
./Application/Src/fault_manager.d \
./Application/Src/ma782_encoder.d \
./Application/Src/motion_control.d \
./Application/Src/motor_driver.d \
./Application/Src/pi_controller.d \
./Application/Src/position_calc.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Src/app_commands.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_commands.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/app_commands.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/app_main.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_main.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/app_main.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/app_scheduler.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_scheduler.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/app_scheduler.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/app_state.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/app_state.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/app_state.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/bsp_hw.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/bsp_hw.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/bsp_hw.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/console.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/console.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/console.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/fault_manager.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/fault_manager.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/fault_manager.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/ma782_encoder.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/ma782_encoder.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/ma782_encoder.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/motion_control.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/motion_control.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/motion_control.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/motor_driver.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/motor_driver.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/motor_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/pi_controller.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/pi_controller.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/pi_controller.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/position_calc.o: F:/TMC\ Motion\ Control/Constar\ Motor/Volumetric\ code/TMC4671_MA782_V4_5_Volumetric_Print_Only_time_Count/STM32U575_Volumetric_Firmware/Src/position_calc.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32U575xx -DUSE_HAL_DRIVER -c -I../../Inc -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../../Drivers/STM32U5xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/Src/position_calc.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Src

clean-Application-2f-Src:
	-$(RM) ./Application/Src/app_commands.cyclo ./Application/Src/app_commands.d ./Application/Src/app_commands.o ./Application/Src/app_commands.su ./Application/Src/app_main.cyclo ./Application/Src/app_main.d ./Application/Src/app_main.o ./Application/Src/app_main.su ./Application/Src/app_scheduler.cyclo ./Application/Src/app_scheduler.d ./Application/Src/app_scheduler.o ./Application/Src/app_scheduler.su ./Application/Src/app_state.cyclo ./Application/Src/app_state.d ./Application/Src/app_state.o ./Application/Src/app_state.su ./Application/Src/bsp_hw.cyclo ./Application/Src/bsp_hw.d ./Application/Src/bsp_hw.o ./Application/Src/bsp_hw.su ./Application/Src/console.cyclo ./Application/Src/console.d ./Application/Src/console.o ./Application/Src/console.su ./Application/Src/fault_manager.cyclo ./Application/Src/fault_manager.d ./Application/Src/fault_manager.o ./Application/Src/fault_manager.su ./Application/Src/ma782_encoder.cyclo ./Application/Src/ma782_encoder.d ./Application/Src/ma782_encoder.o ./Application/Src/ma782_encoder.su ./Application/Src/motion_control.cyclo ./Application/Src/motion_control.d ./Application/Src/motion_control.o ./Application/Src/motion_control.su ./Application/Src/motor_driver.cyclo ./Application/Src/motor_driver.d ./Application/Src/motor_driver.o ./Application/Src/motor_driver.su ./Application/Src/pi_controller.cyclo ./Application/Src/pi_controller.d ./Application/Src/pi_controller.o ./Application/Src/pi_controller.su ./Application/Src/position_calc.cyclo ./Application/Src/position_calc.d ./Application/Src/position_calc.o ./Application/Src/position_calc.su

.PHONY: clean-Application-2f-Src

