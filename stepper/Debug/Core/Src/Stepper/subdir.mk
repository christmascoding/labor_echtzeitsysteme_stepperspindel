################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/carl/Documents/Uni/Spindel_Project_Hardwarenahe_Programmierung_von_Echtzeitsystemen/labor_echtzeitsysteme_stepperspindel/libs/LibL6474/src/LibL6474x.c 

OBJS += \
./Core/Src/Stepper/LibL6474x.o 

C_DEPS += \
./Core/Src/Stepper/LibL6474x.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Stepper/LibL6474x.o: /home/carl/Documents/Uni/Spindel_Project_Hardwarenahe_Programmierung_von_Echtzeitsystemen/labor_echtzeitsysteme_stepperspindel/libs/LibL6474/src/LibL6474x.c Core/Src/Stepper/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -DUSE_FULL_ASSERT -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../../libs/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include -I../../libs/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1 -I../../libs/LibL6474/inc -I"/home/carl/Documents/Uni/Spindel_Project_Hardwarenahe_Programmierung_von_Echtzeitsystemen/labor_echtzeitsysteme_stepperspindel/stepper/Core/Inc/Stepper" -I../../libs/LibRTOSConsole/inc -I"/home/carl/Documents/Uni/Spindel_Project_Hardwarenahe_Programmierung_von_Echtzeitsystemen/labor_echtzeitsysteme_stepperspindel/stepper/Core/Inc/Console" -I"/home/carl/Documents/Uni/Spindel_Project_Hardwarenahe_Programmierung_von_Echtzeitsystemen/labor_echtzeitsysteme_stepperspindel/stepper/Core/Inc/Spindle" -I../../libs/LibSpindle/inc -O0 -ffunction-sections -fdata-sections -Wall -Wextra -pedantic -Wmissing-include-dirs -Wswitch-default -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Stepper

clean-Core-2f-Src-2f-Stepper:
	-$(RM) ./Core/Src/Stepper/LibL6474x.cyclo ./Core/Src/Stepper/LibL6474x.d ./Core/Src/Stepper/LibL6474x.o ./Core/Src/Stepper/LibL6474x.su

.PHONY: clean-Core-2f-Src-2f-Stepper

