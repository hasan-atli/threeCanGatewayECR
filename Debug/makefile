################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G0xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := threeCanGatewayECR_G0B1CB
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
threeCanGatewayECR_G0B1CB.elf \

MAP_FILES += \
threeCanGatewayECR_G0B1CB.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
threeCanGatewayECR_G0B1CB.list \

OBJCOPY_HEX += \
threeCanGatewayECR_G0B1CB.hex \


# All Target
all: main-build

# Main-build Target
main-build: threeCanGatewayECR_G0B1CB.elf secondary-outputs

# Tool invocations
threeCanGatewayECR_G0B1CB.elf threeCanGatewayECR_G0B1CB.map: $(OBJS) $(USER_OBJS) D:\canBusGateWay_ECR\workspace_CanGatewayEcr\threeCanGatewayECR_G0B1CB\STM32G0B1CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "threeCanGatewayECR_G0B1CB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"D:\canBusGateWay_ECR\workspace_CanGatewayEcr\threeCanGatewayECR_G0B1CB\STM32G0B1CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="threeCanGatewayECR_G0B1CB.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

threeCanGatewayECR_G0B1CB.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "threeCanGatewayECR_G0B1CB.list"
	@echo 'Finished building: $@'
	@echo ' '

threeCanGatewayECR_G0B1CB.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "threeCanGatewayECR_G0B1CB.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout threeCanGatewayECR_G0B1CB.elf threeCanGatewayECR_G0B1CB.hex threeCanGatewayECR_G0B1CB.list threeCanGatewayECR_G0B1CB.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
