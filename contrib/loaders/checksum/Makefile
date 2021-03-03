BIN2C = ../../../src/helper/bin2char.sh

ARM_CROSS_COMPILE ?= arm-none-eabi-
ARM_AS      ?= $(ARM_CROSS_COMPILE)as
ARM_OBJCOPY ?= $(ARM_CROSS_COMPILE)objcopy

ARM_AFLAGS = -EL

RISCV_CROSS_COMPILE ?= riscv64-unknown-elf-
RISCV_CC      ?= $(RISCV_CROSS_COMPILE)gcc
RISCV_OBJCOPY ?= $(RISCV_CROSS_COMPILE)objcopy
RISCV32_CFLAGS = -march=rv32e -mabi=ilp32e -nostdlib -nostartfiles -Os -fPIC
RISCV64_CFLAGS = -march=rv64i -mabi=lp64 -nostdlib -nostartfiles -Os -fPIC

all:	arm riscv

arm: armv4_5_crc.inc armv7m_crc.inc

riscv:	riscv32_crc.inc riscv64_crc.inc

armv4_5_%.elf: armv4_5_%.s
	$(ARM_AS) $(ARM_AFLAGS) $< -o $@

armv4_5_%.bin: armv4_5_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

armv7m_%.elf: armv7m_%.s
	$(ARM_AS) $(ARM_AFLAGS) $< -o $@

armv7m_%.bin: armv7m_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

riscv32_%.elf:	riscv_%.c
	$(RISCV_CC) $(RISCV32_CFLAGS) $< -o $@

riscv64_%.elf:	riscv_%.c
	$(RISCV_CC) $(RISCV64_CFLAGS) $< -o $@

riscv%.bin:	riscv%.elf
	$(RISCV_OBJCOPY) -Obinary $< $@

clean:
	-rm -f *.elf *.bin *.inc
