################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Konnect_STM32_Sigfox_Ultrasonic.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Konnect_STM32_Sigfox_Ultrasonic.list \


# All Target
all:
	+@make --no-print-directory main-build && make --no-print-directory post-build

# Main-build Target
main-build: Konnect_STM32_Sigfox_Ultrasonic.elf secondary-outputs

# Tool invocations
Konnect_STM32_Sigfox_Ultrasonic.elf: $(OBJS) $(USER_OBJS) D:\Github\warefab\Warefab-Konnect-Sigfox-Dev-Board\Extensions\Konnect_STM32_Sigfox_Ultrasonic\STM32L052C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Konnect_STM32_Sigfox_Ultrasonic.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"D:\Github\warefab\Warefab-Konnect-Sigfox-Dev-Board\Extensions\Konnect_STM32_Sigfox_Ultrasonic\STM32L052C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Konnect_STM32_Sigfox_Ultrasonic.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Konnect_STM32_Sigfox_Ultrasonic.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Konnect_STM32_Sigfox_Ultrasonic.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy.exe -O ihex "Konnect_STM32_Sigfox_Ultrasonic.elf" "Konnect_STM32_Sigfox_Ultrasonic.hex"
	arm-none-eabi-objcopy.exe -O binary "Konnect_STM32_Sigfox_Ultrasonic.elf" "Konnect_STM32_Sigfox_Ultrasonic.bin"
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
