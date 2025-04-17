################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f746zgtx.s 

OBJS += \
./Core/Startup/startup_stm32f746zgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32f746zgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I../../../ibs/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include -I../../../libs/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include -I../../libs/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include -I../../libs/LibL6474/inc -I../../libs/LibRTOSConsole/inc -I"C:/Users/hans-/Nextcloud/Documents/DHBW/Labor/Labor_HWNahesProg_Echtzeitsysteme/Semester4/Solution/stepper/Core/Inc/Console" -I"C:/Users/hans-/Nextcloud/Documents/DHBW/Labor/Labor_HWNahesProg_Echtzeitsysteme/Semester4/Solution/stepper/Core/Inc/Stepper" -I"C:/Users/hans-/Nextcloud/Documents/DHBW/Labor/Labor_HWNahesProg_Echtzeitsysteme/Semester4/Solution/stepper/Core/Inc/Spindle" -I../../libs/LibSpindle/inc -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f746zgtx.d ./Core/Startup/startup_stm32f746zgtx.o

.PHONY: clean-Core-2f-Startup

