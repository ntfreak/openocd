BIN2C = ../../../../src/helper/bin2char.sh

SRCS=cmspi_read.S cmspi_write.S cmspi_erase_check.S cmspi_verify.S \
	cmdpi_read.S cmdpi_write.S cmdpi_erase_check.S cmdpi_verify.S \
	cmqpi_read.S cmqpi_write.S cmqpi_erase_check.S cmqpi_verify.S \
	cmi2c_read.S cmi2c_write.S cmi2c_erase_check.S cmi2c_verify.S \
	cmi2c_calib.S cmi2c_scan.S

OBJS=$(patsubst %.S,%.inc,$(SRCS))

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump
LD=$(CROSS_COMPILE)ld

all: $(OBJS)

%.o: %.S cmspi.s ../../../../src/flash/nor/cmspi.h
	$(CC) -Wall -Werror -Wa,-adlmn -o $@ -c $< > $(@:.o=.lst)
	@enscript -Easm -T 4 -G -o - $(@:.o=.lst) | ps2pdf - $(@:.o=.pdf) || true

%.elf: %.o
	$(LD) -s -defsym=_start=0 -o $@ $<

%.bin: %.elf
	$(OBJCOPY) -S -O binary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f $(patsubst %.S,%.o,$(SRCS)) $(patsubst %.S,%.elf,$(SRCS)) $(patsubst %.S,%.bin,$(SRCS))
	-rm -f $(patsubst %.S,%.inc,$(SRCS)) $(patsubst %.S,%.pdf,$(SRCS)) $(patsubst %.S,%.lst,$(SRCS))

.PHONY:	all clean

.INTERMEDIATE: $(patsubst %.S,%.o,$(SRCS)) $(patsubst %.S,%.elf,$(SRCS)) $(patsubst %.S,%.bin,$(SRCS))
