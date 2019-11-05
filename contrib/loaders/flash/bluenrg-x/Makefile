BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS =  -c -mthumb -mcpu=cortex-m0 -O3 -g

all:bluenrg-2_write.inc bluenrg-lp_write.inc

.PHONY: clean

.INTERMEDIATE: bluenrg-x_write.o

bluenrg-2_write.o: bluenrg-x_write.c
	$(CC) $(CFLAGS) -Wall -Wextra -Wa,-adhln=$*.lst $< -o $@

bluenrg-lp_write.o: bluenrg-x_write.c
	$(CC) $(CFLAGS) -D BLUENRG_LP -Wall -Wextra -Wa,-adhln=$*.lst $< -o $@

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.lst *.bin *.inc
