BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS =  -c -mthumb -mcpu=cortex-m0 -O3 -g

all: bluenrg-x_write.inc

.PHONY: clean

.INTERMEDIATE: bluenrg-x_write.o

%.o: %.c
	$(CC) $(CFLAGS) -Wall -Wextra -Wa,-adhln=$*.lst $< -o $@

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.lst *.bin *.inc
