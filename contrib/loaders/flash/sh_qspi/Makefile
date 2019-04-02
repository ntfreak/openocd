CROSS_COMPILE=arm-linux-gnueabihf-
BIN2C = ../../../../src/helper/bin2char.sh

TGT = sh_qspi
ASRC += sh_qspi.S
LDS = sh_qspi.ld

OBJS += $(ASRC:.S=.o)

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump
LD=$(CROSS_COMPILE)ld
NM=$(CROSS_COMPILE)nm
SIZE=$(CROSS_COMPILE)size

CFLAGS=-Os -Wall -nostartfiles -marm -nostdinc -ffreestanding -mabi=aapcs-linux -mword-relocations -fno-pic -mno-unaligned-access -ffunction-sections -fdata-sections -fno-common -msoft-float -pipe -march=armv7-a -mtune=generic-armv7-a
LDFLAGS=-T$(LDS) -nostdlib -Map=$(TGT).map

all: $(TGT).inc

%.o: %.S
	$(CC) $(CFLAGS) -c $^ -o $@

$(TGT).elf: $(OBJS)
	$(LD) $(LDFLAGS) $^ -o $@

$(TGT).bin: $(TGT).elf
	$(OBJCOPY) $< -O binary $@
	$(NM) -n $(TGT).elf > $(TGT).sym
	$(SIZE) $(TGT).elf

$(TGT).inc: $(TGT).bin
	$(BIN2C) < $< > $@

clean:
	rm -rf *.elf *.hex *.map *.o *.disasm *.sym
