################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c 

OBJS += \
./Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Source/ComplexMathFunctions/%.o Drivers/CMSIS/DSP/Source/ComplexMathFunctions/%.su Drivers/CMSIS/DSP/Source/ComplexMathFunctions/%.cyclo: ../Drivers/CMSIS/DSP/Source/ComplexMathFunctions/%.c Drivers/CMSIS/DSP/Source/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=4 -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Zhenyu/MySource/Git/Prj_SignalProcess_FIR-IIR/Drivers/CMSIS/DSP/Include" -I"C:/Users/Zhenyu/MySource/Git/Prj_SignalProcess_FIR-IIR/Drivers/CMSIS/DSP/PrivateInclude" -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-ComplexMathFunctions

clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-ComplexMathFunctions:
	-$(RM) ./Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.cyclo ./Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.d ./Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.o ./Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-ComplexMathFunctions

