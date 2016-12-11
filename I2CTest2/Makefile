BINARY = i2ctest2
SCRIPT_DIR=..
OPENCM3_DIR = ../libopencm3
# Linker Script
LDSCRIPT = $(OPENCM3_DIR)/lib/stm32/f1/stm32f103x8.ld

default: $(BINARY).bin

### Custom Makefile Targets

SIZE	= $(PREFIX)-size

# Upload the program via gdb
upload: $(BINARY).hex
	@echo "[UPLOAD] $(BINARY).hex"
	@echo "[GDB]"
	$(GDB) $(BINARY).hex -s $(BINARY).o

# For the debug target compile with an added DEBUG definition that is
# interpreted by the pre-processor. i.e. #ifdef DEBUG ...... #endif
# debug: CPPFLAGS += -DDEBUG
debug: $(BINARY).elf
	@echo "[UPLOAD] $(BINARY).elf"
	@echo "[GDB]"
	$(GDB) $(BINARY).elf -tui

size: $(BINARY).elf
	$(SIZE) -A -d $(BINARY).elf
	$(SIZE) -B $(BINARY).elf

.PHONY: upload debug size

include ../libopencm3.target.mk
