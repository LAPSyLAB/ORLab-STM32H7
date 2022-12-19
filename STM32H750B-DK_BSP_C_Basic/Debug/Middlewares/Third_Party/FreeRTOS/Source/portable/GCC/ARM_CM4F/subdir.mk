################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.o Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.su: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.c Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Utilities/Fonts" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP/Components/ft5336" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP/Components/mt25tl01g" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP/Components/mt48lc4m32b2" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP/Components/rk043fn48h" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP/Components/wm8994" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP/Components/Common" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Middlewares/ST/STM32_Audio/Addons/PDM/Inc" -I"F:/Delovni/WorkSpaces_Local/STM32H750B-DK/STM32H750B-DK_Working_projects/STM32H750B-DK_BSP_C_Basic/Drivers/BSP/Components" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F

clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F:
	-$(RM) ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F

