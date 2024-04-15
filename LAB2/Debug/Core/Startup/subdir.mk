################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/BasicMathFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/BayesFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/CommonTables" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/ComplexMathFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/ControllerFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/DistanceFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/FastMathFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/FilteringFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/InterpolationFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/MatrixFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/QuaternionMathFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/StatisticsFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/SupportFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/SVMFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/TransformFunctions" -I"C:/FIBO/2566_2/FRA222/LAB2_PID/LAB2_MicroconPID/LAB2/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

