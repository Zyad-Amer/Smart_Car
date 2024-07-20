################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../IR_Sensor.c \
../Smart_Car.c \
../gpio.c \
../icu.c \
../lcd.c \
../pwm.c \
../uart.c \
../ultrasonic.c 

OBJS += \
./IR_Sensor.o \
./Smart_Car.o \
./gpio.o \
./icu.o \
./lcd.o \
./pwm.o \
./uart.o \
./ultrasonic.o 

C_DEPS += \
./IR_Sensor.d \
./Smart_Car.d \
./gpio.d \
./icu.d \
./lcd.d \
./pwm.d \
./uart.d \
./ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega32 -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


