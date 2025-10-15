################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.c \
C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.c \
C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.c \
C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.c \
C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.c 

OBJS += \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.o \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.o \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.o \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.o \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.o 

C_DEPS += \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.d \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.d \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.d \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.d \
./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.d 


# Each subdirectory must supply rules for building sources it contributes
micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.o: C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.c micro_xrcedds_client/src/c/profile/discovery/transport/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/shared_memory" -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/util" -I"C:/stm32_microros/firmware/third_party/microros/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.o: C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.c micro_xrcedds_client/src/c/profile/discovery/transport/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/shared_memory" -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/util" -I"C:/stm32_microros/firmware/third_party/microros/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.o: C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.c micro_xrcedds_client/src/c/profile/discovery/transport/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/shared_memory" -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/util" -I"C:/stm32_microros/firmware/third_party/microros/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.o: C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.c micro_xrcedds_client/src/c/profile/discovery/transport/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/shared_memory" -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/util" -I"C:/stm32_microros/firmware/third_party/microros/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.o: C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.c micro_xrcedds_client/src/c/profile/discovery/transport/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/profile/shared_memory" -I"C:/stm32_microros/firmware/third_party/micro_xrcedds_client/src/c/util" -I"C:/stm32_microros/firmware/third_party/microros/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-discovery-2f-transport

clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-discovery-2f-transport:
	-$(RM) ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.cyclo ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.d ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.o ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_freertos_plus_tcp.su ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.cyclo ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.d ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.o ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix.su ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.cyclo ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.d ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.o ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_posix_nopoll.su ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.cyclo ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.d ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.o ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_rtems_bsd_net.su ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.cyclo ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.d ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.o ./micro_xrcedds_client/src/c/profile/discovery/transport/udp_transport_datagram_windows.su

.PHONY: clean-micro_xrcedds_client-2f-src-2f-c-2f-profile-2f-discovery-2f-transport

