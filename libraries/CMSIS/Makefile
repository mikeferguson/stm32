##
# Makefile to build DSP_Lib into libraries
##

ARCH        := M4lf
LIB         := Lib/libarm_cortex$(ARCH)_math.a

# Architectures to build for
ARCHS       := M3l M4l M4lf
M3l_FLAGS   := -mthumb -mcpu=cortex-m3 -DARM_MATH_CM3
M4l_FLAGS   := -mthumb -mcpu=cortex-m4 -DARM_MATH_CM4
M4lf_FLAGS  := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 -D__FPU_PRESENT

# Directories
DSPLIB      := DSP_Lib
DSPDIRS     := BasicMathFunctions          \
               CommonTables                \
               ComplexMathFunctions        \
               ControllerFunctions         \
               FastMathFunctions           \
               FilteringFunctions          \
               MatrixFunctions             \
               StatisticsFunctions         \
               SupportFunctions            \
               TransformFunctions

# Build Paths
BUILD_PATH  := Lib/build
BUILDDIRS   := $(BUILD_PATH)
BUILDDIRS   += $(foreach dir,$(DSPDIRS),$(BUILD_PATH)/$(DSPLIB)/$(dir))

# Targets to build
LIBS        := $(foreach tgt,$(ARCHS),Lib/libarm_cortex$(tgt)_math.a)
SRCS        := $(wildcard $(DSPLIB)/*/*.c)
OBJS        := $(patsubst %.c, %.o, $(SRCS))
OBJS        := $(foreach obj,$(OBJS),$(BUILD_PATH)/$(obj))

# Tools
CC          := arm-none-eabi-gcc
CXX         := arm-none-eabi-g++
LD          := arm-none-eabi-ld
AR          := arm-none-eabi-ar
AS          := arm-none-eabi-gcc
CP          := arm-none-eabi-objcopy
OD          := arm-none-eabi-objdump
SIZE        := arm-none-eabi-size

# Flags
INCLUDES    := -I./Include
CFLAGS      := -c -fno-common -g3 -Os -ffunction-sections
CXXFLAGS    := $(CFLAGS) -fno-exceptions
CPFLAGS     := -Obinary -v
ODFLAGS     := -S -t
ARFLAGS     := 


all: $(BUILDDIRS) $(OBJS) $(LIB)
clean:
	rm -rf $(BUILD_PATH)

$(BUILD_PATH)/%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) $($(ARCH)_FLAGS) -o $@ $<

$(LIB): $(OBJS)
	$(AR) rcs $@ $?

$(BUILDDIRS):
	@mkdir -p $@

