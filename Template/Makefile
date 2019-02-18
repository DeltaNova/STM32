# Provide definitions ahead of including Master Makefile
################################################################################
$(info Set Project Definitions)
# Define the Project Path
PATH_PROJECT_DIR = $(wildcard ~)/Git/Public/STM32/Template/
PATH_PROJECT_LIB = $(wildcard ~)/Git/Public/STM32/libraries/
PATH_PROJECT_LIB2 = $(wildcard ~)/Git/Public/STM32/libraries/sensors/
PATH_PROJECT_LIB3 = $(wildcard ~)/Git/Public/STM32/libraries/displays/
LIBS += $(PATH_PROJECT_LIB)
LIBS += $(PATH_PROJECT_LIB2)
LIBS += $(PATH_PROJECT_LIB3)

$(info [Library Path] $(LIBS))
# Compile Project Library Files
%.o : $(PATH_PROJECT_LIB)%.cpp
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
%.o : $(PATH_PROJECT_LIB2)%.cpp
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
%.o : $(PATH_PROJECT_LIB3)%.cpp
	$(CC) $(CPPFLAGS) $< -o $(notdir $@)
$(info [Library Sources] $(SOURCES))

SOURCES += $(wildcard $(PATH_PROJECT_LIB)*.cpp)
SOURCES += $(wildcard $(PATH_PROJECT_LIB2)*.cpp)
SOURCES += $(wildcard $(PATH_PROJECT_LIB3)*.cpp)

# Not required as all object files are built in the project directory
#OBJECTS += $(wildcard $(PATH_PROJECT_LIB)*.o)
#$(info [Library Objects] $(OBJECTS))

CPPFLAGS += -std=gnu++17
# Use Newlib-nano
LDFLAGS += --specs=nano.specs 
# Define the Target Name
TARGET = template
# Include the Master Makefile
$(info Include Master Makefile)
include ../Makefile

