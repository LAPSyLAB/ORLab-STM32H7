################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Utilities/Fonts/font12.c \
../Core/Utilities/Fonts/font16.c \
../Core/Utilities/Fonts/font20.c \
../Core/Utilities/Fonts/font24.c \
../Core/Utilities/Fonts/font8.c 

OBJS += \
./Core/Utilities/Fonts/font12.o \
./Core/Utilities/Fonts/font16.o \
./Core/Utilities/Fonts/font20.o \
./Core/Utilities/Fonts/font24.o \
./Core/Utilities/Fonts/font8.o 

C_DEPS += \
./Core/Utilities/Fonts/font12.d \
./Core/Utilities/Fonts/font16.d \
./Core/Utilities/Fonts/font20.d \
./Core/Utilities/Fonts/font24.d \
./Core/Utilities/Fonts/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Utilities/Fonts/%.o Core/Utilities/Fonts/%.su: ../Core/Utilities/Fonts/%.c Core/Utilities/Fonts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Core/Utilities/lcd" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Core/Utilities/Fonts" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP/Components/ft5336" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP/Components/mt25tl01g" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP/Components/mt48lc4m32b2" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP/Components/rk043fn48h" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP/Components/wm8994" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP/Components/Common" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Middlewares/ST/STM32_Audio/Addons/PDM/Inc" -I"/Users/patriciobulic/STM32CubeIDE/STM32H750B/H7-BSP-LCD-OS/Drivers/BSP/Components" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Utilities-2f-Fonts

clean-Core-2f-Utilities-2f-Fonts:
	-$(RM) ./Core/Utilities/Fonts/font12.d ./Core/Utilities/Fonts/font12.o ./Core/Utilities/Fonts/font12.su ./Core/Utilities/Fonts/font16.d ./Core/Utilities/Fonts/font16.o ./Core/Utilities/Fonts/font16.su ./Core/Utilities/Fonts/font20.d ./Core/Utilities/Fonts/font20.o ./Core/Utilities/Fonts/font20.su ./Core/Utilities/Fonts/font24.d ./Core/Utilities/Fonts/font24.o ./Core/Utilities/Fonts/font24.su ./Core/Utilities/Fonts/font8.d ./Core/Utilities/Fonts/font8.o ./Core/Utilities/Fonts/font8.su

.PHONY: clean-Core-2f-Utilities-2f-Fonts

