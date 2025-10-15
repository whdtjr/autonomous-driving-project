################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include micro_xrcedds_client/src/c/core/session/stream/subdir.mk
-include micro_xrcedds_client/src/c/core/session/subdir.mk
-include micro_xrcedds_client/src/c/core/serialization/subdir.mk
-include micro_xrcedds_client/src/c/core/log/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := stm32_wifi_ros2_bridge_demo
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
stm32_wifi_ros2_bridge_demo.elf \

MAP_FILES += \
stm32_wifi_ros2_bridge_demo.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm32_wifi_ros2_bridge_demo.list \


# All Target
all: main-build

# Main-build Target
main-build: stm32_wifi_ros2_bridge_demo.elf secondary-outputs

# Tool invocations
stm32_wifi_ros2_bridge_demo.elf stm32_wifi_ros2_bridge_demo.map: $(OBJS) $(USER_OBJS) C:\Users\STC\WorkSpace\autonomous-driving-project\STM\stm32_wifi_ros2_bridge_demo\STM32F411RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "stm32_wifi_ros2_bridge_demo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\STC\WorkSpace\autonomous-driving-project\STM\stm32_wifi_ros2_bridge_demo\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm32_wifi_ros2_bridge_demo.map" -Wl,--gc-sections -static -L"C:\stm32_microros\firmware\third_party\microros\lib" -Wl,--gc-sections --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm32_wifi_ros2_bridge_demo.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm32_wifi_ros2_bridge_demo.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout stm32_wifi_ros2_bridge_demo.elf stm32_wifi_ros2_bridge_demo.list stm32_wifi_ros2_bridge_demo.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
