# STM32 Makefile for GNU Toolchain
#
# Usage:
#       make test
################################################################################
TARGET = hello
# ~ points to current users home directory
# Need to use the $(wildcard ...) function to force expansion when setting variables.
PATH_PROJECT_DIR = $(wildcard ~)/STM32/test4/

DEVICE = STM32F103XB
DEVICE_LOWER = stm32f103xb
DEVICE_FAMILY = STM32F1xx
DEVICE_FAMILY_LOWER = stm32f1xx

# Location of the STM32CUBE Files
STMCUBEROOT = $(wildcard ~)/Downloads/STM32Cube_FW_F1_V1.3.0/
STMCUBE_PATH = Drivers/CMSIS/Device/ST/STM32F1xx/
STMCUBE_PATH2 = Drivers/CMSIS/Include/

# NOTE: Using local modified version of Cube Linker, Cube version (V1.3.0)buggy
#LINKER = $(STMCUBEROOT)$(STMCUBE_PATH)Source/Templates/gcc/linker/$(DEVICE)_FLASH.ld
LINKER = $(DEVICE)_FLASH.ld
# TODO: Check to see if there is a LINKER in the project directory and if so use
#       that over the Cube LINKER

STARTUP_PATH = $(STMCUBEROOT)$(STMCUBE_PATH)Source/Templates/gcc/
STARTUP = $(STMCUBEROOT)$(STMCUBE_PATH)Source/Templates/gcc/startup_$(DEVICE_LOWER).s
# TODO: Check to see if there is a STARTUP in the project directory and
#       if so use that over the Cube STARTUP
SYSTEM_PATH = $(STMCUBEROOT)$(STMCUBE_PATH)Source/Templates/
SYSTEM = $(STMCUBEROOT)$(STMCUBE_PATH)Source/Templates/system_$(DEVICE_FAMILY_LOWER).c

# MCPU variable for GCC
MCPU = cortex-m3

# When Quiet set to '@', do not echo command
QUIET = @

################################################################################
# Toolchain
################################################################################
PREFIX 	= arm-none-eabi
CC 		= $(PREFIX)-gcc
CXX		= $(PREFIX)-g++
AR		= $(PREFIX)-ar
OBJCOPY	= $(PREFIX)-objcopy
OBJDUMP	= $(PREFIX)-objdump
SIZE	= $(PREFIX)-size
GDB		= $(PREFIX)-gdb
################################################################################

# Check that the main project file is available
PROJECT_FILE := $(wildcard $(TARGET).c $(TARGET).cpp)
ifdef PROJECT_FILE
ifneq "$(words $(PROJECT_FILE))" "1"
$(error More than one project file in this directory!)
endif
endif
# TODO: Add a check for a missing project file

# Automatically determine the sources
SOURCES := $(wildcard *.c *.cc *.cpp *.C *.s)
SOURCES += $(STARTUP)
SOURCES += $(SYSTEM)
DIR = $(dir $(TARGET))

OBJECTS += $(wildcard *.o)
################################################################################
# Flags
################################################################################
# Enable all warnings
CPPFLAGS += -Wall
# Add '-g' to compiler flag to embed debug info (doesn't end up in .hex)
CPPFLAGS += -g
# Specify the target processor
CPPFLAGS += -mcpu=$(MCPU)
# Compile for little endian target
CPPFLAGS += -mlittle-endian
# Generate core that executes in Thumb states <----------- What does this mean??
CPPFLAGS += -mthumb
# Device compiling for
#CPPFLAGS += -D$(DEVICE)
CPPFLAGS += -DSTM32F103xB
# Optimise for size
CPPFLAGS += -Os
# Compile but do not link
CPPFLAGS += -c

# Add a -I prefix to each library path.
CPPFLAGS += $(addprefix -I, $(LIBS))

# Libaray Search Paths
# Path to $(DEVICE).h system_$(DEVICE).h
LIBS += $(STMCUBEROOT)$(STMCUBE_PATH)Include/
# Path to cmsis_ghh.h core_cm3.h
LIBS += $(STMCUBEROOT)$(STMCUBE_PATH2)

#LDFLAGS += -Wl, --gc-sections
#LDFLAGS += --gc-sections -Wl,
#LDFLAGS += -Map=$(TARGET).map
#LDFLAGS += $(addprefix -I, $(LIBS))
LDFLAGS += -mcpu=$(MCPU)
LDFLAGS += -mlittle-endian
LDFLAGS += -mthumb
LDFLAGS += -DSTM32F103xB
LDFLAGS += -T$(LINKER)
LDFLAGS += -Wl,--gc-sections
# Enable Semihosting  <---------------------------Make notes as to what this is.
LDFLAGS += --specs=rdimon.specs -lc -lrdimon
################################################################################

# TODO: Enhance to take into account different case of file extension
# Compiling Object Files From C Files
%.o : %.c
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From C Files in Cube32 Dir
%.o : $(SYSTEM_PATH)%.c
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From C++ Files
%.o : %.cpp
	$(CXX) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From S Files
%.o : %.s
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From S Files in Cube32 Dir
%.o : $(STARTUP_PATH)%.s
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)

################################################################################
# Default Rule
.DEFAULT_GOAL := all
################################################################################
# Rules
################################################################################

# PHONY - Targets that do not represent files.
.PHONY: all clean debug upload target size

all:
	@echo $(SOURCES)
	@echo $(LIBS)
	@echo ""
	@echo ""
	@echo "LDFLAGS: $(LDFLAGS)"
	@echo ""
	@echo ~
	@echo $(OBJECTS)

target: $(TARGET).hex

# $(basename filename) - returns filename without suffix
# $(addsuffix suffix, name) - adds suffix to name
# Look through the list of sources that need to be compiled into object files.
# Use basename to remove the source suffix (i.e. .c), then add a .o suffix and
# store in OBJECTS.
OBJECTS := $(addsuffix .o, $(notdir $(basename $(SOURCES))))

# Covert elf binary to hex
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@


#Compile elf binary from object files
$(TARGET).elf: $(OBJECTS)
	@echo "[LD] $(TARGET).elf"
	@echo " Linking: $(OBJECTS)"
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

size: $(TARGET).elf
	$(SIZE) -A -d $(TARGET).elf

# TODO: Add a check for local .gdbinit
upload: $(TARGET).hex
	@echo "[UPLOAD] $(TARGET).hex"
	@echo "[GDB]"
	$(GDB) $(TARGET).hex -s $(TARGET).o
# The line above starts GDB, it passes the hex binary

# For the debug target compile with an added DEBUG definition that is
# interpreted by the pre-processor. i.e. #ifdef DEBUG ...... #endif
debug: CPPFLAGS += -DDEBUG
debug: $(TARGET).elf
	@echo "[UPLOAD] $(TARGET).elf"
	@echo "[GDB]"
	$(GDB) $(TARGET).elf

# Clean up build files
clean:
	@echo "[REMOVE] $(TARGET).elf"; rm -f $(TARGET).elf
	@echo "[REMOVE] $(TARGET).map"; rm -f $(TARGET).map
	@echo "[REMOVE] $(TARGET).lst"; rm -f $(TARGET).lst
	@echo "[REMOVE] $(TARGET).hex"; rm -f $(TARGET).hex
	@echo "[REMOVE] *.o"; rm *.o
