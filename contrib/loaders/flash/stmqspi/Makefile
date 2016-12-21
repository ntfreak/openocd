BIN2C = ../../../../src/helper/bin2char.sh

SRCS=stmqspi_erase_check.S stmqspi_crc32.S stmqspi_read.S stmqspi_write.S \
     stmoctospi_erase_check.S stmoctospi_crc32.S stmoctospi_read.S stmoctospi_write.S
OBJS=$(patsubst %.S,%.inc,$(SRCS))

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump
LD=$(CROSS_COMPILE)ld

all: $(OBJS)

%.o: %.S Makefile
	$(CC) -Wall -Werror -Wa,-adhlmn -o $@ -c $< > $(@:.o=.lst)
	@enscript -Easm -T 4 -G -o - $(@:.o=.lst) | ps2pdf - $(@:.o=.pdf) || true

%.elf: %.o
	$(LD) -s -defsym=_start=0 -o $@ $<

%.bin: %.elf
	$(OBJCOPY) -S -O binary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.elf *.lst *.pdf *.bin *.inc

.PHONY:	all clean

.INTERMEDIATE: $(patsubst %.S,%.o,$(SRCS)) $(patsubst %.S,%.elf,$(SRCS)) $(patsubst %.S,%.bin,$(SRCS))
