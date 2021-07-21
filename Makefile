# Copyright (c) 2021, Robert Ryszard Paciorek <rrp@opcode.eu.org>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

BIN = minimalModbus
all: $(BIN).bin

OBJ = main.o modbusRTU.o
modbusRTU.o: modbusRTU.c modbusRTU.h modbusRTU_Config.h modbusRTU_Debug.h


# Serial device for stm32flash
SERIAL ?= /dev/ttyUSB0

# Device specific things
DEVICE ?= stm32f103c8t6

# LibopenCM3 location
OPENCM3_DIR ?= libopencm3

# ARM toolchain
TOOL_PREFIX  ?= /usr/bin/arm-none-eabi
CC           := $(TOOL_PREFIX)-gcc
LD           := $(TOOL_PREFIX)-gcc
AR           := $(TOOL_PREFIX)-ar
AS           := $(TOOL_PREFIX)-as
OBJCOPY      := $(TOOL_PREFIX)-objcopy
OBJDUMP      := $(TOOL_PREFIX)-objdump
GDB          := $(TOOL_PREFIX)-gdb

LDFLAGS += --static -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(*).map -Wl,--gc-sections

# libopencm3 makefiles that generate .ld file for the specified device and:
#  - set $(LDSCRIPT) variable and add rule to generate this file
#  - update $(ARCH_FLAGS) variable (set -mcpu and other platform depended flags)
#  - update $(CPPFLAGS) and $(LDFLAGS) variables (set -I and -L to libopencm3 dirs)
#  - update $(LDLIBS) variable (add platform depended lib, see $(LIBNAME))
include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(*).o -c $(*).c

%.bin: $(OBJ) $(LDSCRIPT)
	$(LD) $(LDFLAGS) $(OBJ) $(LDLIBS) $(ARCH_FLAGS) -o $(BIN).elf
	$(OBJCOPY) -Obinary $(BIN).elf $(BIN).bin

clean:
	rm -f *.o *.d *.elf *.bin *.map generated.*

install: $(BIN).bin
	stm32flash -w $< $(SERIAL)
	stm32flash -g 0x0 $(SERIAL)

run:
	stm32flash -g 0x0 $(SERIAL)
