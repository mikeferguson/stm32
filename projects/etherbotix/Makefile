# Program name
PRG         := etherbotix
# Architecture
MCU         := STM32F4XX
MCU_FLAGS   := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
MCU_FLAGS   += -DARM_MATH_CM4 -D__FPU_PRESENT
# Library location
LIB_PATH    := ../../libraries
# Linker Script
LD_SCRIPT   := STM32F407VET6.ld

# tools
CC          := arm-none-eabi-gcc
CXX         := arm-none-eabi-g++
LD          := arm-none-eabi-ld
AR          := arm-none-eabi-ar
AS          := arm-none-eabi-gcc
CP          := arm-none-eabi-objcopy
OD          := arm-none-eabi-objdump
SIZE        := arm-none-eabi-size

# Flags
INCLUDES    := -I. -I$(LIB_PATH)/CMSIS/Include -I$(LIB_PATH)/ST/inc
CFLAGS      := -c -D$(MCU) $(MCU_FLAGS) -fno-common -g3 -Os -ffunction-sections
CXXFLAGS    := $(CFLAGS) -fno-exceptions
LDFLAGS     := -L$(LIB_PATH)/ST/ld -T$(LD_SCRIPT) --gc-sections -v
CPFLAGS     := -Obinary -v
ODFLAGS     := -S -t
ARFLAGS     := 

BUILD_PATH  := build
BUILDDIRS   := $(BUILD_PATH)

# Build main stuff
TGT_BIN     := $(BUILD_PATH)/main.o $(BUILD_PATH)/startup_f4.o $(BUILD_PATH)/system_stm32f4xx.o
TGT_BIN 	+= $(BUILD_PATH)/netconf.o # This is the setup for stm32 ethernet + lwip

all: $(BUILD_PATH)/$(PRG).bin
clean:
	rm -rf build

# Build STM32_CPP
INCLUDES    += -I$(LIB_PATH)/libcpp/inc
BUILDDIRS   += $(BUILD_PATH)/stm32_cpp
TGT_BIN     += $(BUILD_PATH)/stm32_cpp/analog_sampler.o
TGT_BIN     += $(BUILD_PATH)/stm32_cpp/delay.o
TGT_BIN     += $(BUILD_PATH)/stm32_cpp/string.o

$(BUILD_PATH)/stm32_cpp/%.o: $(LIB_PATH)/libcpp/src/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $<

# Build LWIP
BUILDDIRS   += $(BUILD_PATH)/lwip/core $(BUILD_PATH)/lwip/core/ipv4 \
               $(BUILD_PATH)/lwip/netif $(BUILD_PATH)/lwip/port \
               $(BUILD_PATH)/lwip/port/STM32F4x7

LWIP_INCLUDES := -I. \
               -I$(LIB_PATH)/lwip/port/STM32F4x7 \
               -I$(LIB_PATH)/lwip/src/include/ipv4 \
               -I$(LIB_PATH)/lwip/src/include

TGT_BIN     += $(BUILD_PATH)/lwip/core/init.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/mem.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/memp.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/netif.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/pbuf.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/raw.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/stats.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/sys.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/tcp.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/tcp_in.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/tcp_out.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/udp.o

TGT_BIN    += $(BUILD_PATH)/lwip/core/ipv4/autoip.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/ipv4/icmp.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/ipv4/inet.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/ipv4/inet_chksum.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/ipv4/ip_addr.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/ipv4/ip.o
TGT_BIN     += $(BUILD_PATH)/lwip/core/ipv4/ip_frag.o

TGT_BIN     += $(BUILD_PATH)/lwip/netif/etharp.o

TGT_BIN     += $(BUILD_PATH)/lwip/port/STM32F4x7/ethernetif.o

$(BUILD_PATH)/lwip/core/%.o: $(LIB_PATH)/lwip/src/core/%.c
	$(CC) $(CFLAGS) $(LWIP_INCLUDES) -o $@ $<

$(BUILD_PATH)/lwip/core/ipv4/%.o: $(LIB_PATH)/lwip/src/core/ipv4/%.c
	$(CC) $(CFLAGS) $(LWIP_INCLUDES) -o $@ $<

$(BUILD_PATH)/lwip/netif/%.o: $(LIB_PATH)/lwip/src/netif/%.c
	$(CC) $(CFLAGS) $(LWIP_INCLUDES) -o $@ $<

$(BUILD_PATH)/lwip/port/%.o: $(LIB_PATH)/lwip/port/%.c
	$(CC) $(CFLAGS) $(LWIP_INCLUDES) $(INCLUDES) -o $@ $<

# Build STM32F4x7_ETH_Driver
INCLUDES    += -I$(LIB_PATH)/STM32F4x7_ETH_Driver/inc
BUILDDIRS   += $(BUILD_PATH)/eth_driver
TGT_BIN     += $(BUILD_PATH)/eth_driver/stm32f4x7_eth.o

$(BUILD_PATH)/eth_driver/%.o: $(LIB_PATH)/STM32F4x7_ETH_Driver/src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $<

# Build STM32F4xx_StdPeriph_Driver
INCLUDES    += -I$(LIB_PATH)/STM32F4xx_StdPeriph_Driver/inc
CFLAGS      += -DUSE_STDPERIPH_DRIVER
BUILDDIRS   += $(BUILD_PATH)/std_periph
#TGT_BIN    += $(BUILD_PATH)/std_periph/misc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_adc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_bkp.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_can.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_cec.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_crc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_dac.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_dbgmcu.o
TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_dma.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_exti.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_flash.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_fsmc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_gpio.o
TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_i2c.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_iwdg.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_pwr.o
TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_rcc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_rtc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_sdio.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_spi.o
TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_syscfg.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_tim.o
TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_usart.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_wwdg.o

$(BUILD_PATH)/std_periph/%.o: $(LIB_PATH)/STM32F4xx_StdPeriph_Driver/src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $<

# Primary build rules
$(BUILD_PATH)/%.o: %.c
	$(CC) $(CFLAGS) -std=c99 $(INCLUDES) $(LWIP_INCLUDES) -o $@ $<

$(BUILD_PATH)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LWIP_INCLUDES) -o $@ $<

$(BUILD_PATH)/$(PRG).bin: $(BUILD_PATH)/$(PRG).elf
	$(CP) $(CPFLAGS) $(BUILD_PATH)/$(PRG).elf $@
	$(OD) $(ODFLAGS) -d $(BUILD_PATH)/$(PRG).elf > $(BUILD_PATH)/$(PRG).lst
	@echo "Final Size:"
	@$(SIZE) $<

$(BUILD_PATH)/$(PRG).elf: $(BUILDDIRS) $(TGT_BIN) 
	$(LD) $(LDFLAGS) -o $@ $(TGT_BIN) $(LIBS)

$(BUILDDIRS):
	@mkdir -p $@

# Generate .gdbinit
.PHONY: .gdbinit
.gdbinit: 
	echo "target remote localhost:3333" > .gdbinit
	echo "mon reset init" >> .gdbinit
	echo "" >> .gdbinit
	echo "def flash" >> .gdbinit
	echo " file $(BUILD_PATH)/$(PRG).elf" >> .gdbinit
	echo " mon reset halt" >> .gdbinit
	echo " mon flash erase_address pad 0x08000000 [file size $(PWD)/$(BUILD_PATH)/$(PRG).bin]" >> .gdbinit
	echo " mon flash write_bank 0 $(PWD)/$(BUILD_PATH)/$(PRG).bin 0" >> .gdbinit
	echo " mon reset init" >> .gdbinit
	echo "end" >> .gdbinit
	echo "" >> .gdbinit
	echo "def reset" >> .gdbinit
	echo " mon reset halt" >> .gdbinit
	echo " mon reset init" >> .gdbinit
	echo "end" >> .gdbinit
