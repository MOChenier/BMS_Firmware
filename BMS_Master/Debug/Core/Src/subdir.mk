################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/can_bus.c \
../Core/Src/charger.c \
../Core/Src/in_op.c \
../Core/Src/interupts.c \
../Core/Src/main.c \
../Core/Src/misc.c \
../Core/Src/retarget.c \
../Core/Src/slave_com.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/can_bus.o \
./Core/Src/charger.o \
./Core/Src/in_op.o \
./Core/Src/interupts.o \
./Core/Src/main.o \
./Core/Src/misc.o \
./Core/Src/retarget.o \
./Core/Src/slave_com.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/can_bus.d \
./Core/Src/charger.d \
./Core/Src/in_op.d \
./Core/Src/interupts.d \
./Core/Src/main.d \
./Core/Src/misc.d \
./Core/Src/retarget.d \
./Core/Src/slave_com.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/can_bus.cyclo ./Core/Src/can_bus.d ./Core/Src/can_bus.o ./Core/Src/can_bus.su ./Core/Src/charger.cyclo ./Core/Src/charger.d ./Core/Src/charger.o ./Core/Src/charger.su ./Core/Src/in_op.cyclo ./Core/Src/in_op.d ./Core/Src/in_op.o ./Core/Src/in_op.su ./Core/Src/interupts.cyclo ./Core/Src/interupts.d ./Core/Src/interupts.o ./Core/Src/interupts.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/misc.cyclo ./Core/Src/misc.d ./Core/Src/misc.o ./Core/Src/misc.su ./Core/Src/retarget.cyclo ./Core/Src/retarget.d ./Core/Src/retarget.o ./Core/Src/retarget.su ./Core/Src/slave_com.cyclo ./Core/Src/slave_com.d ./Core/Src/slave_com.o ./Core/Src/slave_com.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

