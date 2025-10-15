################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.c 

OBJS += \
./micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.o 

C_DEPS += \
./micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.o: C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.c micro_xrcedds_client/src/c/profile/transport/stream_framing/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/shared_memory" -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/util" -I"C:/stm32_microros/firmware/third_party/microros/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-transport-2f-stream_framing

clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-transport-2f-stream_framing:
	-$(RM) ./micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.cyclo ./micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.d ./micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.o ./micro_xrcedds_client/src/c/profile/transport/stream_framing/stream_framing_protocol.su

.PHONY: clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-transport-2f-stream_framing

