BIN2C = ../../../../src/helper/bin2char.sh

# Toolchain used in makefile
CROSS_COMPILE ?= arm-none-eabi-
CC             = $(CROSS_COMPILE)gcc
CPLUS          = $(CROSS_COMPILE)g++
CPP            = $(CROSS_COMPILE)cpp
LD             = $(CROSS_COMPILE)gcc
AS             = $(CROSS_COMPILE)as
OBJCOPY        =$(CROSS_COMPILE)objcopy
OBJDUMP        =$(CROSS_COMPILE)objdump
OBJSIZE        =$(CROSS_COMPILE)size

TARGET         = npcx_algo
OBJS          := npcx_flash.o
FLAGS          = -mthumb -Os -ffunction-sections -fdata-sections -g -gdwarf-3
FLAGS         += -gstrict-dwarf -Wall -fno-strict-aliasing --asm

CFLAGS         = -c -I. -mcpu=cortex-m4 -fpack-struct
LDFLAGS        = -Wl,-Map,lfw.map -Wl,-T"npcx_flash.lds" -nostartfiles

all: $(TARGET).inc

# Implicit rules
%.o: %.c
	-@ echo CC  $@	from  $<
	@$(CC) $< $(FLAGS) $(CFLAGS) -o $@


$(TARGET).elf: $(OBJS)
	-@ echo LD  $@	from  $<
	@$(LD)  -o $@ $< $(FLAGS) $(LDFLAGS)

%.bin: %.elf
	-@ echo OBJCOPY  $@	from  $<
	-@ $(OBJCOPY) $< -O binary $@

%.inc: %.bin
	@echo 'Building target: $@'
	@echo 'Invoking Bin2Char Script'
	$(BIN2C) < $< > $@
	rm $< $*.elf
	@echo 'Finished building target: $@'
	@echo ' '

clean:
	@echo 'Cleaning Targets and Build Artifacts'
	rm -rf *.inc *.bin *.elf *.map
	rm -rf *.o *.d
	@echo 'Finished clean'
	@echo ' '

.PRECIOUS: %.bin

.PHONY: all clean
