# $Header: $
# This will make the test program for ARM.

PROC=arm
TYPE=none-linux-gnueabi
LDSCRIPT=ldscript

PATH:=/opt/freescale/usr/local/gcc-4.1.2-glibc-2.5-nptl-3/arm-none-linux-gnueabi/bin/:$(PATH)
CC=$(PROC)-$(TYPE)-gcc
AS=$(PROC)-$(TYPE)-as
AR=$(PROC)-$(TYPE)-ar
LD=$(PROC)-$(TYPE)-ld
NM=$(PROC)-$(TYPE)-nm
OBJDUMP=$(PROC)-$(TYPE)-objdump
CFLAGS= -g -c -mcpu=arm1136j-s

all: test.elf

# Make a little endian image:
# In Eclipse, add the line :
#    source gdbinit 
# to : Run -> Debug... (menu) -> Commands (tab): Commands (listbox)
# To start gdb from a window use : arm-elf-gdb --command=gdbinit
test.elf: test.c Makefile ldscript crt0.S
	$(CC) $(CFLAGS) -o crt0.o crt0.S
	$(CC) $(CFLAGS) -o test.o test.c
	$(LD) -g -v -T$(LDSCRIPT) -o test.elf crt0.o test.o 
	$(NM) test.elf

	
dump:
	$(OBJDUMP) --all-headers test.elf

dump_test:
	$(OBJDUMP) --disassemble test.elf

dump_full:
	$(OBJDUMP) --full-contents test.elf

clean:
	-/bin/rm -f *.o *~ test.elf
