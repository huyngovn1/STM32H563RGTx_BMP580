################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USBX/App/subdir.mk
-include Middlewares/ST/usbx/common/usbx_stm32_device_controllers/subdir.mk
-include Middlewares/ST/usbx/common/usbx_device_classes/src/subdir.mk
-include Middlewares/ST/usbx/common/core/src/subdir.mk
-include Drivers/STM32H5xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := HW_FC_V4_STM32H563RG_ICM42670_BMP580
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.elf \

MAP_FILES += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.list \

OBJCOPY_HEX += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.hex \

OBJCOPY_BIN += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.bin \

OBJCOPY_VERILOG += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.verilog \

OBJCOPY_SREC += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.srec \

OBJCOPY_SYMBOLSREC += \
HW_FC_V4_STM32H563RG_ICM42670_BMP580.symbolsrec \


# All Target
all: main-build

# Main-build Target
main-build: HW_FC_V4_STM32H563RG_ICM42670_BMP580.elf secondary-outputs

# Tool invocations
HW_FC_V4_STM32H563RG_ICM42670_BMP580.elf HW_FC_V4_STM32H563RG_ICM42670_BMP580.map: $(OBJS) $(USER_OBJS) C:\Users\huy\Downloads\HW_FC_V4_STM32H563RG\STM32H563RGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "HW_FC_V4_STM32H563RG_ICM42670_BMP580.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"C:\Users\huy\Downloads\HW_FC_V4_STM32H563RG\STM32H563RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="HW_FC_V4_STM32H563RG_ICM42670_BMP580.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

HW_FC_V4_STM32H563RG_ICM42670_BMP580.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "HW_FC_V4_STM32H563RG_ICM42670_BMP580.list"
	@echo 'Finished building: $@'
	@echo ' '

HW_FC_V4_STM32H563RG_ICM42670_BMP580.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "HW_FC_V4_STM32H563RG_ICM42670_BMP580.hex"
	@echo 'Finished building: $@'
	@echo ' '

HW_FC_V4_STM32H563RG_ICM42670_BMP580.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "HW_FC_V4_STM32H563RG_ICM42670_BMP580.bin"
	@echo 'Finished building: $@'
	@echo ' '

HW_FC_V4_STM32H563RG_ICM42670_BMP580.verilog: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O verilog $(EXECUTABLES) "HW_FC_V4_STM32H563RG_ICM42670_BMP580.verilog"
	@echo 'Finished building: $@'
	@echo ' '

HW_FC_V4_STM32H563RG_ICM42670_BMP580.srec: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O srec $(EXECUTABLES) "HW_FC_V4_STM32H563RG_ICM42670_BMP580.srec"
	@echo 'Finished building: $@'
	@echo ' '

HW_FC_V4_STM32H563RG_ICM42670_BMP580.symbolsrec: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O symbolsrec $(EXECUTABLES) "HW_FC_V4_STM32H563RG_ICM42670_BMP580.symbolsrec"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) HW_FC_V4_STM32H563RG_ICM42670_BMP580.bin HW_FC_V4_STM32H563RG_ICM42670_BMP580.elf HW_FC_V4_STM32H563RG_ICM42670_BMP580.hex HW_FC_V4_STM32H563RG_ICM42670_BMP580.list HW_FC_V4_STM32H563RG_ICM42670_BMP580.map HW_FC_V4_STM32H563RG_ICM42670_BMP580.srec HW_FC_V4_STM32H563RG_ICM42670_BMP580.symbolsrec HW_FC_V4_STM32H563RG_ICM42670_BMP580.verilog default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN) $(OBJCOPY_VERILOG) $(OBJCOPY_SREC) $(OBJCOPY_SYMBOLSREC)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
