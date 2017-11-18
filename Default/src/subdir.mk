################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/FusionEKF.cpp \
../src/kalman_filter.cpp \
../src/main.cpp \
../src/tools.cpp 

OBJS += \
./src/FusionEKF.o \
./src/kalman_filter.o \
./src/main.o \
./src/tools.o 

CPP_DEPS += \
./src/FusionEKF.d \
./src/kalman_filter.d \
./src/main.d \
./src/tools.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


