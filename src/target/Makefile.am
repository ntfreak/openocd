if OOCD_TRACE
OOCD_TRACE_FILES = %D%/oocd_trace.c
else
OOCD_TRACE_FILES =
endif

%C%_libtarget_la_LIBADD = %D%/openrisc/libopenrisc.la \
	%D%/riscv/libriscv.la

%C%_libtarget_la_CPPFLAGS = $(AM_CPPFLAGS)

STARTUP_TCL_SRCS += %D%/startup.tcl

noinst_LTLIBRARIES += %D%/libtarget.la
%C%_libtarget_la_SOURCES = \
	$(TARGET_CORE_SRC) \
	$(ARM_DEBUG_SRC) \
	$(ARMV4_5_SRC) \
	$(ARMV6_SRC) \
	$(ARMV7_SRC) \
	$(ARM_MISC_SRC) \
	$(AVR32_SRC) \
	$(MIPS32_SRC) \
	$(NDS32_SRC) \
	$(STM8_SRC) \
	$(INTEL_IA32_SRC) \
	$(ESIRISC_SRC) \
        $(ARC_SRC) \
	%D%/avrt.c \
	%D%/dsp563xx.c \
	%D%/dsp563xx_once.c \
	%D%/dsp5680xx.c \
	%D%/hla_target.c \
	$(ARMV8_SRC) \
	$(MIPS64_SRC)

if HAVE_CAPSTONE
%C%_libtarget_la_CPPFLAGS += $(CAPSTONE_CFLAGS)
%C%_libtarget_la_LIBADD += $(CAPSTONE_LIBS)
endif

TARGET_CORE_SRC = \
	%D%/algorithm.c \
	%D%/register.c \
	%D%/image.c \
	%D%/breakpoints.c \
	%D%/target.c \
	%D%/target_request.c \
	%D%/testee.c \
	%D%/semihosting_common.c \
	%D%/smp.c \
	%D%/rtt.c

ARMV4_5_SRC = \
	%D%/armv4_5.c \
	%D%/armv4_5_mmu.c \
	%D%/armv4_5_cache.c \
	$(ARM7_9_SRC)

ARM7_9_SRC = \
	%D%/arm7_9_common.c \
	%D%/arm7tdmi.c \
	%D%/arm720t.c \
	%D%/arm9tdmi.c \
	%D%/arm920t.c \
	%D%/arm966e.c \
	%D%/arm946e.c \
	%D%/arm926ejs.c \
	%D%/feroceon.c

ARM_MISC_SRC = \
	%D%/fa526.c \
	%D%/xscale.c

ARMV6_SRC = \
	%D%/arm11.c \
	%D%/arm11_dbgtap.c

ARMV7_SRC = \
	%D%/armv7m.c \
	%D%/armv7m_trace.c \
	%D%/cortex_m.c \
	%D%/armv7a.c \
	%D%/armv7a_mmu.c \
	%D%/cortex_a.c \
	%D%/ls1_sap.c \
	%D%/mem_ap.c

ARMV8_SRC = \
	%D%/armv8_dpm.c \
	%D%/armv8_opcodes.c \
	%D%/aarch64.c \
	%D%/a64_disassembler.c \
	%D%/armv8.c \
	%D%/armv8_cache.c

ARM_DEBUG_SRC = \
	%D%/arm_dpm.c \
	%D%/arm_jtag.c \
	%D%/arm_disassembler.c \
	%D%/arm_simulator.c \
	%D%/arm_semihosting.c \
	%D%/arm_adi_v5.c \
	%D%/arm_dap.c \
	%D%/armv7a_cache.c \
	%D%/armv7a_cache_l2x.c \
	%D%/adi_v5_dapdirect.c \
	%D%/adi_v5_jtag.c \
	%D%/adi_v5_swd.c \
	%D%/embeddedice.c \
	%D%/trace.c \
	%D%/etb.c \
	%D%/etm.c \
	$(OOCD_TRACE_FILES) \
	%D%/etm_dummy.c \
	%D%/arm_cti.c

AVR32_SRC = \
	%D%/avr32_ap7k.c \
	%D%/avr32_jtag.c \
	%D%/avr32_mem.c \
	%D%/avr32_regs.c

MIPS32_SRC = \
	%D%/mips32.c \
	%D%/mips_m4k.c \
	%D%/mips32_pracc.c \
	%D%/mips32_dmaacc.c \
	%D%/mips_ejtag.c

MIPS64_SRC = \
	%D%/mips64.c \
	%D%/mips32_pracc.c \
	%D%/mips64_pracc.c \
	%D%/mips_mips64.c \
	%D%/trace.c \
	%D%/mips_ejtag.c

NDS32_SRC = \
	%D%/nds32.c \
	%D%/nds32_reg.c \
	%D%/nds32_cmd.c \
	%D%/nds32_disassembler.c \
	%D%/nds32_tlb.c \
	%D%/nds32_v2.c \
	%D%/nds32_v3_common.c \
	%D%/nds32_v3.c \
	%D%/nds32_v3m.c \
	%D%/nds32_aice.c

STM8_SRC = \
	%D%/stm8.c

INTEL_IA32_SRC = \
	%D%/quark_x10xx.c \
	%D%/quark_d20xx.c \
	%D%/lakemont.c \
	%D%/x86_32_common.c

ESIRISC_SRC = \
	%D%/esirisc.c \
	%D%/esirisc_jtag.c \
	%D%/esirisc_trace.c

ARC_SRC = \
        %D%/arc.c \
        %D%/arc_cmd.c \
        %D%/arc_jtag.c \
        %D%/arc_mem.c

%C%_libtarget_la_SOURCES += \
	%D%/algorithm.h \
	%D%/arm.h \
	%D%/arm_dpm.h \
	%D%/arm_jtag.h \
	%D%/arm_adi_v5.h \
	%D%/armv7a_cache.h \
	%D%/armv7a_cache_l2x.h \
	%D%/armv7a_mmu.h \
	%D%/arm_disassembler.h \
	%D%/a64_disassembler.h \
	%D%/arm_opcodes.h \
	%D%/arm_simulator.h \
	%D%/arm_semihosting.h \
	%D%/arm7_9_common.h \
	%D%/arm7tdmi.h \
	%D%/arm720t.h \
	%D%/arm9tdmi.h \
	%D%/arm920t.h \
	%D%/arm926ejs.h \
	%D%/arm966e.h \
	%D%/arm946e.h \
	%D%/arm11.h \
	%D%/arm11_dbgtap.h \
	%D%/armv4_5.h \
	%D%/armv4_5_mmu.h \
	%D%/armv4_5_cache.h \
	%D%/armv7a.h \
	%D%/armv7m.h \
	%D%/armv7m_trace.h \
	%D%/armv8.h \
	%D%/armv8_dpm.h \
	%D%/armv8_opcodes.h \
	%D%/armv8_cache.h \
	%D%/avrt.h \
	%D%/dsp563xx.h \
	%D%/dsp563xx_once.h \
	%D%/dsp5680xx.h \
	%D%/breakpoints.h \
	%D%/cortex_m.h \
	%D%/cortex_a.h \
	%D%/aarch64.h \
	%D%/embeddedice.h \
	%D%/etb.h \
	%D%/etm.h \
	%D%/etm_dummy.h \
	%D%/image.h \
	%D%/mips32.h \
	%D%/mips64.h \
	%D%/mips_m4k.h \
	%D%/mips_mips64.h \
	%D%/mips_ejtag.h \
	%D%/mips32_pracc.h \
	%D%/mips32_dmaacc.h \
	%D%/mips64_pracc.h \
	%D%/oocd_trace.h \
	%D%/register.h \
	%D%/target.h \
	%D%/target_type.h \
	%D%/trace.h \
	%D%/target_request.h \
	%D%/trace.h \
	%D%/xscale.h \
	%D%/smp.h \
	%D%/avr32_ap7k.h \
	%D%/avr32_jtag.h \
	%D%/avr32_mem.h \
	%D%/avr32_regs.h \
	%D%/nds32.h \
	%D%/nds32_cmd.h \
	%D%/nds32_disassembler.h \
	%D%/nds32_edm.h \
	%D%/nds32_insn.h \
	%D%/nds32_reg.h \
	%D%/nds32_tlb.h \
	%D%/nds32_v2.h \
	%D%/nds32_v3_common.h \
	%D%/nds32_v3.h \
	%D%/nds32_v3m.h \
	%D%/nds32_aice.h \
	%D%/semihosting_common.h \
	%D%/stm8.h \
	%D%/lakemont.h \
	%D%/x86_32_common.h \
	%D%/arm_cti.h \
	%D%/esirisc.h \
	%D%/esirisc_jtag.h \
	%D%/esirisc_regs.h \
	%D%/esirisc_trace.h \
	%D%/arc.h \
	%D%/arc_cmd.h \
	%D%/arc_jtag.h \
	%D%/arc_mem.h \
	%D%/rtt.h

include %D%/openrisc/Makefile.am
include %D%/riscv/Makefile.am
