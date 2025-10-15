################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/discovery.c 

OBJS += \
./micro_xrcedds_client/src/c/profile/discovery/discovery.o 

C_DEPS += \
./micro_xrcedds_client/src/c/profile/discovery/discovery.d 


# Each subdirectory must supply rules for building sources it contributes
micro_xrcedds_client/src/c/profile/discovery/discovery.o: C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/discovery.c micro_xrcedds_client/src/c/profile/discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/shared_memory" -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/util" -I"C:/stm32_microros/firmware/third_party/microros/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-discovery

clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-discovery:
	-$(RM) ./micro_xrcedds_client/src/c/profile/discovery/discovery.cyclo ./micro_xrcedds_client/src/c/profile/discovery/discovery.d ./micro_xrcedds_client/src/c/profile/discovery/discovery.o ./micro_xrcedds_client/src/c/profile/discovery/discovery.su

.PHONY: clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-discovery

