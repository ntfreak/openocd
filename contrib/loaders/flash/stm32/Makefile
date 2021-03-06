BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump


AFLAGS = -static -nostartfiles -mlittle-endian -Wa,-EL
CFLAGS = -c -mthumb -nostdlib -nostartfiles -Os -g -fPIC

all: stm32f1x.inc stm32f2x.inc stm32h7x.inc stm32l4x.inc stm32lx.inc

.PHONY: clean

%.elf: %.S
	$(CC) $(AFLAGS) $< -o $@

stm32l4x.elf: stm32l4x.c
	$(CC) $(CFLAGS) -mcpu=cortex-m0plus -fstack-usage -Wa,-adhln=$(<:.c=.lst) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc
