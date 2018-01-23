# STM32 Makefile for GNU Toolchain
#
# Usage:
#		make		- Builds "target" by default
#		make all	- Builds "target" by default
#		make target - Builds "target" by default
#		make size	- Reports size of .elf binary
#		make upload - Upload built .hex via GDB
#		make debug  - Upload built .elf via GDB (with Text User Interface)
#		make clean  - Clean build files from project directory
#       make mx 	- Launches STM32CubeMX (if installed)
################################################################################
######################### START CONFIG SECTION #################################
################################################################################
#Uncomment and set the following as required
# TARGET = default_target
# PATH_PROJECT_DIR = $(CURDIR)

DEVICE = STM32F103XB
DEVICE_STM_HEADER = STM32F103xB
DEVICE_LOWER = stm32f103xb
DEVICE_FAMILY = STM32F1xx
DEVICE_FAMILY_LOWER = stm32f1xx

# Location of the STM32CUBE Files
STMCUBEMX = $(wildcard ~)/Software/STM32CubeMX
STMCUBE_REPO = $(STMCUBEMX)/Repository/STM32Cube_FW_F1_V1.6.0/

STMCUBE_PATH = Drivers/CMSIS/Device/ST/STM32F1xx/
STMCUBE_PATH2 = Drivers/CMSIS/Include/

CMSIS_LINKER_PATH =$(STMCUBE_REPO)$(STMCUBE_PATH)Source/Templates/gcc/linker/
CMSIS_LINKER = $(CMSIS_LINKER_PATH)$(DEVICE)_FLASH.ld

CMSIS_STARTUP_PATH = $(STMCUBE_REPO)$(STMCUBE_PATH)Source/Templates/gcc/
CMSIS_STARTUP = $(CMSIS_STARTUP_PATH)startup_$(DEVICE_LOWER).s

CMSIS_SYSTEM_PATH = $(STMCUBE_REPO)$(STMCUBE_PATH)Source/Templates/
CMSIS_SYSTEM = $(CMSIS_SYSTEM_PATH)system_$(DEVICE_FAMILY_LOWER).c

# Custom File Paths
CUSTOM_LINKER = Custom_$(DEVICE)_FLASH.ld
CUSTOM_STARTUP = startup_$(DEVICE_LOWER).s
CUSTOM_SYSTEM = system_stm32f1xx.c
################################################################################
######################## END CONFIG SECTION ####################################
################################################################################
# Set the TARGET name if not defined
ifndef TARGET
TARGET = default_target
endif
# Set the path to the project directory if not defined.
# Default project directory set to $(CURDIR), directory of executing Makefile.
ifndef PATH_PROJECT_DIR
PATH_PROJECT_DIR = $(CURDIR)
endif
################################################################################
# Insert Blank Line in Terminal Output
$(info )
################################################################################
# NOTE: Use a local modified version of Cube Linker, Cube version (V1.3.0)buggy
# If a local Linker file is found use that, otherwise revert to Cube version.
ifeq ("","$(wildcard $(PATH_PROJECT_DIR)*.ld)")
# Using CMSIS Linker
	LINKER = $(CMSIS_LINKER)
$(info [CMSIS]  [LINKER] $(LINKER))
else
# Using Custom Project Linker
	LINKER = $(CUSTOM_LINKER)
$(info [CUSTOM] [LINKER] $(LINKER))
endif
################################################################################
# Check for project specific startup file to use instead of CMSIS version.
# TODO: Improve how Makefile handles custom startup files with different names.
ifeq ("","$(wildcard $(PATH_PROJECT_DIR)startup*.s)")
# Using CMSIS Startup File
	STARTUP = $(CMSIS_STARTUP)
$(info [CMSIS]  [STARTUP] $(STARTUP))
else
# Using Custom Project Startup File
	STARTUP = $(CUSTOM_STARTUP)
$(info [CUSTOM] [STARTUP] $(STARTUP))
endif
################################################################################
# Check for project specific system file to use instead of CMSIS version.
# TODO: Improve how Makefile handles custom system files with different names.
ifeq ("","$(wildcard $(PATH_PROJECT_DIR)system*.c)")
# Using CMSIS System File
	SYSTEM = $(CMSIS_SYSTEM)
$(info [CMSIS]  [SYSTEM] $(SYSTEM))
else
# Using Project System File
	SYSTEM = $(CUSTOM_SYSTEM)
$(info [CUSTOM] [SYSTEM] $(SYSTEM))
endif
################################################################################
# Check for .gdbint file
ifeq ("","$(wildcard $(PATH_PROJECT_DIR).gdbinit)")
$(warning Unable to locate .gdbinit, you might want to create one; it's useful)
endif
################################################################################
# Insert Blank Line in Terminal Output
$(info )

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
ifeq ("","$(wildcard $(PATH_PROJECT_DIR)$(TARGET).c $(PATH_PROJECT_DIR)$(TARGET).cpp)")
$(warning Target Source Not Found)
$(error Expected $(PATH_PROJECT_DIR)$(TARGET).c or $(PATH_PROJECT_DIR)$(TARGET).cpp)
endif
# Check for too many project files
PROJECT_FILE := $(wildcard $(TARGET).c $(TARGET).cpp)
ifneq "$(words $(PROJECT_FILE))" "1"
$(warning More than one project file in this directory!)
$(error Remove or Rename: $(PROJECT_FILE))
endif
################################################################################
# Automatically determine the sources
SOURCES += $(wildcard *.c *.cc *.cpp *.C *.s)
SOURCES += $(STARTUP)
$(info [Sources] $(SOURCES))
# TODO: Check if this is needed when using the CMSIS SYSTEM file.
#       Could be made part of the CMSIS/CUSTOM SYSTEM file check.
#SOURCES += $(SYSTEM) # Not req if system file in proj dir. Fail if added twice.
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
# Generate core that executes in Thumb states <------TODO: What does this mean??
CPPFLAGS += -mthumb
# Device compiling for
CPPFLAGS += -D$(DEVICE_STM_HEADER)
# Optimise for size
CPPFLAGS += -Os
# Compile but do not link
CPPFLAGS += -c

# Add a -I prefix to each library path.
CPPFLAGS += $(addprefix -I, $(LIBS))

# Libaray Search Paths
# Path to $(DEVICE).h system_$(DEVICE).h
LIBS += $(STMCUBE_REPO)$(STMCUBE_PATH)Include/
# Path to cmsis_ghh.h core_cm3.h
LIBS += $(STMCUBE_REPO)$(STMCUBE_PATH2)

#LDFLAGS += -Wl, --gc-sections
#LDFLAGS += --gc-sections -Wl,
#LDFLAGS += -Map=$(TARGET).map
#LDFLAGS += $(addprefix -I, $(LIBS))
LDFLAGS += -mcpu=$(MCPU)
LDFLAGS += -mlittle-endian
LDFLAGS += -mthumb
LDFLAGS += -D$(DEVICE_STM_HEADER)
LDFLAGS += -T$(LINKER)
LDFLAGS += -Wl,--gc-sections
# Enable Semihosting  <-------------------- TODO: Make notes as to what this is.
LDFLAGS += --specs=rdimon.specs -lc -lrdimon
################################################################################
# Compiling Object Files From C Files
%.o : %.c
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
%.o : %.C
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From C Files in Cube32 Dir
%.o : $(SYSTEM_PATH)%.c
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From C++ Files
%.o : %.cpp
	$(CXX) $(CPPFLAGS) $< -o $(notdir $@)
%.o : %.CPP
	$(CXX) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From S Files
%.o : %.s
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
%.o : %.S
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
# Compiling Object Files From S Files in Cube32 Dir
%.o : $(CMSIS_STARTUP_PATH)%.s
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)

################################################################################
# Default Rule
.DEFAULT_GOAL := all
################################################################################
# Rules
################################################################################

# PHONY - Targets that do not represent files.
.PHONY: all clean debug upload target size mx

all: target

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
	@echo ""
	@echo "[LD] $(TARGET).elf"
	@echo "[LINKING] $(OBJECTS)"
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

size: $(TARGET).elf
	$(SIZE) -A -d $(TARGET).elf
	$(SIZE) -B $(TARGET).elf

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
	$(GDB) $(TARGET).elf -tui

# Clean up build files
clean:
	@echo "[REMOVE] $(TARGET).elf"; rm -f $(TARGET).elf
	@echo "[REMOVE] $(TARGET).map"; rm -f $(TARGET).map
	@echo "[REMOVE] $(TARGET).lst"; rm -f $(TARGET).lst
	@echo "[REMOVE] $(TARGET).hex"; rm -f $(TARGET).hex
	@echo "[REMOVE] *.o"; rm *.o

# Launch STMCubeMX GUI
mx:
	$(STMCUBEMX)/STM32CubeMX
