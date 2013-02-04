# Program name
PRG         := template_f4
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

# DSP library support -- to use this, need to run "make" in CMSIS folder
LDFLAGS     += -L$(LIB_PATH)/CMSIS/Lib/
LIBS        = -larm_cortexM4lf_math

BUILD_PATH  := build
BUILDDIRS   := $(BUILD_PATH)

# Build main stuff
TGT_BIN     := $(BUILD_PATH)/main.o $(BUILD_PATH)/startup_f4.o $(BUILD_PATH)/system_stm32f4xx.o

all: $(BUILD_PATH)/$(PRG).bin
clean:
	rm -rf build

# Build STM32_CPP
INCLUDES    += -I$(LIB_PATH)/libcpp/inc
BUILDDIRS   += $(BUILD_PATH)/stm32_cpp
#TGT_BIN     += $(BUILD_PATH)/stm32_cpp/test.o

$(BUILD_PATH)/stm32_cpp/%.o: $(LIB_PATH)/libcpp/src/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $<

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
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_dma.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_exti.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_flash.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_fsmc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_gpio.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_i2c.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_iwdg.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_pwr.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_rcc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_rtc.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_sdio.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_spi.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_tim.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_usart.o
#TGT_BIN    += $(BUILD_PATH)/std_periph/stm32f4xx_wwdg.o

$(BUILD_PATH)/std_periph/%.o: $(LIB_PATH)/STM32F4xx_StdPeriph_Driver/src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $<

# Primary build rules
$(BUILD_PATH)/%.o: %.c
	$(CC) $(CFLAGS) -std=c99 $(INCLUDES) -o $@ $<

$(BUILD_PATH)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $<

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
	echo "mon soft_reset_halt" >> .gdbinit
	echo "mon reset init" >> .gdbinit
	echo "" >> .gdbinit
	echo "def flash" >> .gdbinit
	echo " file $(BUILD_PATH)/$(PRG).elf" >> .gdbinit
	echo " mon soft_reset_halt" >> .gdbinit
	echo " mon stm32f2x mass_erase 0" >> .gdbinit
	echo " mon flash write_bank 0 $(PWD)/$(BUILD_PATH)/$(PRG).bin 0" >> .gdbinit
	echo " mon soft_reset_halt" >> .gdbinit
	echo " mon reset init" >> .gdbinit
	echo "end" >> .gdbinit
	echo "" >> .gdbinit
	echo "def reset" >> .gdbinit
	echo " mon soft_reset_halt" >> .gdbinit
	echo " mon reset init" >> .gdbinit
	echo "end" >> .gdbinit


