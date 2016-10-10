# Provide definitions ahead of including Master Makefile
################################################################################
$(info Set Project Definitions)
# Define the Project Path
PATH_PROJECT_DIR = $(wildcard ~)/Git/STM32/I2CTest/
PATH_PROJECT_LIB = $(wildcard ~)/Git/STM32/libraries/
LIBS += $(PATH_PROJECT_LIB)
$(info [Library Path] $(LIBS))
# Compile Project Library Files
%.o : $(PATH_PROJECT_LIB)%.cpp
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)

SOURCES += $(wildcard $(PATH_PROJECT_LIB)*.cpp)
OBJECTS += $(wildcard $(PATH_PROJECT_LIB)*.o)
$(info [Library Sources] $(SOURCES))
$(info [Library Objects] $(OBJECTS))
CPPFLAGS += -std=gnu++11
# Use Newlib-nano
LDFLAGS += --specs=nano.specs
# Define the Target Name
TARGET = i2c_test
# Include the Master Makefile
$(info Include Master Makefile)
include ../Makefile

