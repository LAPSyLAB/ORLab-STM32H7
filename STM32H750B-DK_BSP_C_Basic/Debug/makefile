################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/Components/wm8994/subdir.mk
-include Drivers/BSP/Components/mt48lc4m32b2/subdir.mk
-include Drivers/BSP/Components/mt25tl01g/subdir.mk
-include Drivers/BSP/Components/ft5336/subdir.mk
-include Drivers/BSP/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := STM32H750B-DK_BSP_C_Basic
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32H750B-DK_BSP_C_Basic.elf \

MAP_FILES += \
STM32H750B-DK_BSP_C_Basic.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32H750B-DK_BSP_C_Basic.list \


# All Target
all: main-build

# Main-build Target
main-build: STM32H750B-DK_BSP_C_Basic.elf secondary-outputs

# Tool invocations
STM32H750B-DK_BSP_C_Basic.elf STM32H750B-DK_BSP_C_Basic.map: $(OBJS) $(USER_OBJS) F:\Delovni\WorkSpaces_Local\STM32H750B-DK\STM32H750B-DK_Working_projects\STM32H750B-DK_BSP_C_Basic\STM32H750XBHX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM32H750B-DK_BSP_C_Basic.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"F:\Delovni\WorkSpaces_Local\STM32H750B-DK\STM32H750B-DK_Working_projects\STM32H750B-DK_BSP_C_Basic\STM32H750XBHX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32H750B-DK_BSP_C_Basic.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32H750B-DK_BSP_C_Basic.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32H750B-DK_BSP_C_Basic.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STM32H750B-DK_BSP_C_Basic.elf STM32H750B-DK_BSP_C_Basic.list STM32H750B-DK_BSP_C_Basic.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
