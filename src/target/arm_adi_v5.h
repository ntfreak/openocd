/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2014 by Alamy Liu                                       *
 *   alamy.liu@gmail.com                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef ARM_ADI_V5_H
#define ARM_ADI_V5_H

/**
 * @file
 * This defines formats and data structures used to talk to ADIv5 entities.
 * Those include a DAP, different types of Debug Port (DP), and memory mapped
 * resources accessed through a MEM-AP.
 */

#include <helper/list.h>
#include "arm_jtag.h"

/* three-bit ACK values for SWD access (sent LSB first) */
#define SWD_ACK_OK    0x1
#define SWD_ACK_WAIT  0x2
#define SWD_ACK_FAULT 0x4

#define DPAP_WRITE		0
#define DPAP_READ		1

#define BANK_REG(bank, reg)	(((bank) << 4) | (reg))

/* A[3:0] for DP registers; A[1:0] are always zero.
 * - JTAG accesses all of these via JTAG_DP_DPACC, except for
 *   IDCODE (JTAG_DP_IDCODE) and ABORT (JTAG_DP_ABORT).
 * - SWD accesses these directly, sometimes needing SELECT.CTRLSEL
 */
#define DP_IDCODE		BANK_REG(0x0, 0x0)	/* SWD: read */
#define DP_ABORT		BANK_REG(0x0, 0x0)	/* SWD: write */
#define DP_CTRL_STAT	BANK_REG(0x0, 0x4)	/* r/w */
#define DP_RESEND		BANK_REG(0x0, 0x8)	/* SWD: read */
#define DP_SELECT		BANK_REG(0x0, 0x8)	/* JTAG: r/w; SWD: write */
#define DP_RDBUFF		BANK_REG(0x0, 0xC)	/* read-only */
#define DP_TARGETSEL	BANK_REG(0x0, 0xC)	/* SWD/DPv2: wo */
#define DP_DLCR			BANK_REG(0x1, 0x4)	/* SWD: rw */
#define DP_TARGETID		BANK_REG(0x2, 0x4)	/* SWD/DPv2: ro */
#define DP_EVENTSTAT	BANK_REG(0x4, 0x4)	/* SWD/DPv2: ro */
#define DP_DLPIDR		BANK_REG(0x3, 0x4)	/* SWD/DPv2: ro */

#define	DLCR_TO_TRN(dlcr)	((uint32_t)(1 + ((3 & (dlcr)) >> 8)))	/* 1..4 clocks */

/* Fields of the DP's AP ABORT register */
#define DAPABORT        (1UL << 0)
#define STKCMPCLR       (1UL << 1) /* SWD-only */
#define STKERRCLR       (1UL << 2) /* SWD-only */
#define WDERRCLR        (1UL << 3) /* SWD-only */
#define ORUNERRCLR      (1UL << 4) /* SWD-only */

/* Fields of the DP's CTRL/STAT register */
#define CORUNDETECT     (1UL << 0)
#define SSTICKYORUN     (1UL << 1)
/* 3:2 - transaction mode (e.g. pushed compare) */
#define SSTICKYCMP      (1UL << 4)
#define SSTICKYERR      (1UL << 5)
#define READOK          (1UL << 6) /* SWD-only */
#define WDATAERR        (1UL << 7) /* SWD-only */
/* 11:8 - mask lanes for pushed compare or verify ops */
/* 21:12 - transaction counter */
#define CDBGRSTREQ      (1UL << 26)
#define CDBGRSTACK      (1UL << 27)
#define CDBGPWRUPREQ    (1UL << 28)
#define CDBGPWRUPACK    (1UL << 29)
#define CSYSPWRUPREQ    (1UL << 30)
#define CSYSPWRUPACK    (1UL << 31)

/* MEM-AP register addresses */
#define MEM_AP_REG_CSW		0x00
#define MEM_AP_REG_TAR		0x04
#define MEM_AP_REG_TAR64	0x08		/* RW: Large Physical Address Extension */
#define MEM_AP_REG_DRW		0x0C		/* RW: Data Read/Write register */
#define MEM_AP_REG_BD0		0x10		/* RW: Banked Data register 0-3 */
#define MEM_AP_REG_BD1		0x14
#define MEM_AP_REG_BD2		0x18
#define MEM_AP_REG_BD3		0x1C
#define MEM_AP_REG_MBT		0x20		/* --: Memory Barrier Transfer register */
#define MEM_AP_REG_BASE64	0xF0		/* RO: Debug Base Address (LA) register */
#define MEM_AP_REG_CFG		0xF4		/* RO: Configuration register */
#define MEM_AP_REG_BASE		0xF8		/* RO: Debug Base Address register */
/* Generic AP register address */
#define AP_REG_IDR			0xFC		/* RO: Identification Register */

/* Fields of the MEM-AP's CSW register */
#define CSW_SIZE_MASK		(7UL << 0)
#define CSW_SIZE_8BIT		(0)			/*       Byte (  8-bits) */
#define CSW_SIZE_16BIT		(1)			/*   Halfword ( 16-bits) */
#define CSW_SIZE_32BIT		(2)			/*       Word ( 32-bits) */
#define CSW_SIZE_64BIT		(3)			/* Doubleword ( 64-bits) */
#define CSW_SIZE_128BIT		(4)			/*          - (128-bits) */
#define CSW_SIZE_256BIT		(5)			/*          - (256-bits) */

#define CSW_ADDRINC_MASK	(3UL << 4)
#define CSW_ADDRINC_OFF		(0UL << 4)
#define CSW_ADDRINC_SINGLE	(1UL << 4)
#define CSW_ADDRINC_PACKED	(2UL << 4)
#define CSW_DEVICE_EN		(1UL << 6)
#define CSW_TRIN_PROG		(1UL << 7)	/* Transfer in Progress */
#define CSW_MODE_MASK		(0xF << 8)
#define CSW_MODE_BASIC		(0UL << 8)
#define CSW_MODE_BARRIER	(1UL << 8)
#define CSW_TYPE_MASK		(0xF << 12)
#define CSW_SPID_EN		(1UL << 23)
/* 30:24 - implementation-defined! */
#define CSW_HPROT		(1UL << 25) /* ? */
#define CSW_MASTER_DEBUG	(1UL << 29) /* ? */
#define CSW_SPROT		(1UL << 30)
#define CSW_DBG_SW_ENABLE	(1UL << 31)

/* Bit definition of AP_REG_IDR (0xFC)
 *
 * Revision				bits[31:28]
 * JEP106 continuation code		bits[27:24], 4-bits
 * JEP106 identity code			bits[23:17], 7-bits
 * Class				bits[16:13]
 * (reserved, SBZ)			bits[12: 8]
 * AP identification			bits[ 7: 0]
 *   Variant	bits[7:4]
 *   Type		bits[3:0]
 */
#define IDR_ID_TYPE_SHIFT       (0)
#define IDR_ID_TYPE_MASK        (0xF << IDR_ID_TYPE_SHIFT)
#define IDR_ID_VART_SHIFT       (4)
#define IDR_ID_VART_MASK        (0xF << IDR_ID_VART_SHIFT)
#define IDR_CLASS_SHIFT         (13)
#define IDR_CLASS_MASK          (0xF << IDR_CLASS_SHIFT)
#define IDR_JEP106_ID_SHIFT     (17)
#define IDR_JEP106_ID_MASK      (0x7F << IDR_JEP106_ID_SHIFT)
#define IDR_JEP106_CONT_SHIFT   (24)
#define IDR_JEP106_CONT_MASK    (0xF << IDR_JEP106_CONT_SHIFT)
#define IDR_REV_SHIFT           (28)
#define IDR_REV_MASK            (0xF << IDR_REV_SHIFT)


/*
 * CoreSight Component register addresses
 */
#define CS_REG_ITCTRL			(0xF00)	/* Integration Mode Control register */
#define CS_REG_CLAIMSET			(0xFA0)	/* Claim Tag Set register */
#define CS_REG_CLAIMCLR			(0xFA4)	/* Claim Tag Clear register */
#define CS_REG_DEVAFF0			(0xFA8)	/* Device Affinity register 0 */
#define CS_REG_DEVAFF1			(0xFAC)	/* Device Affinity register 1 */
#define CS_REG_LAR			(0xFB0)	/* Lock Access Register */
#define CS_REG_LSR			(0xFB4)	/* Lock Status Register */
#define CS_REG_AUTHSTATUS		(0xFB8)	/* Authentication Status register */
#define CS_REG_DEVARCH			(0xFBC)	/* Device Architecture register */
#define CS_REG_DEVID2			(0xFC0)	/* Device ID register 2 */
#define CS_REG_DEVID1			(0xFC4)	/* Device ID register 1 */
#define CS_REG_DEVID			(0xFC8)	/* Device ID register */
#define CS_REG_DEVTYPE			(0xFCC)	/* Device Type Identifier register */
#define CS_REG_PIDR4			(0xFD0)	/* Peripheral Identification Registers */
#define CS_REG_PIDR5			(0xFD4)	/*   PIDR0-PIDR7 */
#define CS_REG_PIDR6			(0xFD8)
#define CS_REG_PIDR7			(0xFDC)
#define CS_REG_PIDR0			(0xFE0)
#define CS_REG_PIDR1			(0xFE4)
#define CS_REG_PIDR2			(0xFE8)
#define CS_REG_PIDR3			(0xFEC)
#define CS_REG_CIDR0			(0xFF0)	/* Component Identification Registers */
#define CS_REG_CIDR1			(0xFF4)	/*   CIDR0-CIDR3 */
#define CS_REG_CIDR2			(0xFF8)
#define CS_REG_CIDR3			(0xFFC)


typedef enum cid_class {	/* CID[15:12] */
	CC_VERIFICATION		= 0x0,	/* Generic verification component */
	CC_ROM			= 0x1,	/* ROM Table */
	CC_DEBUG		= 0x9,	/* Debug (CoreSight) component */
	CC_PTB			= 0xB,	/* Peripheral Test Block (PTB) */
	CC_DESS			= 0xD,	/* OptimoDE Data Engine SubSystem component */
	CC_IP			= 0xE,	/* Generic IP component */
	CC_PCELL		= 0xF	/* PrimeCell peripheral */
} cid_class_t;

#define	JEP106_ARM		(0x43B)
#define	PID_PART_CORTEX_A57	(0xD07)
#define	PID_PART_CORTEX_A53	(0xD03)

/* Fields of the MEM-AP's IDR register */
#define IDR_REV     (0xFUL << 28)
#define IDR_JEP106  (0x7FFUL << 17)
#define IDR_CLASS   (0xFUL << 13)
#define IDR_VARIANT (0xFUL << 4)
#define IDR_TYPE    (0xFUL << 0)

#define IDR_JEP106_ARM 0x04760000

#define DP_SELECT_APSEL 0xFF000000
#define DP_SELECT_APBANK 0x000000F0
#define DP_SELECT_DPBANK 0x0000000F
#define DP_SELECT_INVALID 0x00FFFF00 /* Reserved bits one */

/**
 * This represents an ARM Debug Interface (v5) Access Port (AP).
 * Most common is a MEM-AP, for memory access.
 */
struct adiv5_ap {
	/**
	 * DAP this AP belongs to.
	 */
	struct adiv5_dap *dap;

	/**
	 * Number of this AP.
	 */
	uint8_t ap_num;

	/**
	 * Default value for (MEM-AP) AP_REG_CSW register.
	 */
	uint32_t csw_default;

	/**
	 * Cache for (MEM-AP) AP_REG_CSW register value.  This is written to
	 * configure an access mode, such as autoincrementing AP_REG_TAR during
	 * word access.  "-1" indicates no cached value.
	 */
	uint32_t csw_value;

	/**
	 * Cache for (MEM-AP) AP_REG_TAR register value This is written to
	 * configure the address being read or written
	 * "-1" indicates no cached value.
	 */
	uint32_t tar_value;

	/**
	 * Configures how many extra tck clocks are added after starting a
	 * MEM-AP access before we try to read its status (and/or result).
	 */
	uint32_t memaccess_tck;

	/* Size of TAR autoincrement block, ARM ADI Specification requires at least 10 bits */
	uint32_t tar_autoincr_block;

	/* true if packed transfers are supported by the MEM-AP */
	bool packed_transfers;

	/* true if unaligned memory access is not supported by the MEM-AP */
	bool unaligned_access_bad;
};


/**
 * This represents an ARM Debug Interface (v5) Debug Access Port (DAP).
 * A DAP has two types of component:  one Debug Port (DP), which is a
 * transport agent; and at least one Access Port (AP), controlling
 * resource access.
 *
 * There are two basic DP transports: JTAG, and ARM's low pin-count SWD.
 * Accordingly, this interface is responsible for hiding the transport
 * differences so upper layer code can largely ignore them.
 *
 * When the chip is implemented with JTAG-DP or SW-DP, the transport is
 * fixed as JTAG or SWD, respectively.  Chips incorporating SWJ-DP permit
 * a choice made at board design time (by only using the SWD pins), or
 * as part of setting up a debug session (if all the dual-role JTAG/SWD
 * signals are available).
 */
struct adiv5_dap {
	const struct dap_ops *ops;

	/* dap transaction list for WAIT support */
	struct list_head cmd_journal;

	struct jtag_tap *tap;
	/* Control config */
	uint32_t dp_ctrl_stat;

	struct adiv5_ap ap[256];

	/* The current manually selected AP by the "dap apsel" command */
	uint32_t apsel;

	/**
	 * Cache for DP_SELECT register. A value of DP_SELECT_INVALID
	 * indicates no cached value and forces rewrite of the register.
	 */
	uint32_t select;

	/* information about current pending SWjDP-AHBAP transaction */
	uint8_t  ack;

	/**
	 * Holds the pointer to the destination word for the last queued read,
	 * for use with posted AP read sequence optimization.
	 */
	uint32_t *last_read;

	/* The TI TMS470 and TMS570 series processors use a BE-32 memory ordering
	 * despite lack of support in the ARMv7 architecture. Memory access through
	 * the AHB-AP has strange byte ordering these processors, and we need to
	 * swizzle appropriately. */
	bool ti_be_32_quirks;

	/**
	 * Signals that an attempt to reestablish communication afresh
	 * should be performed before the next access.
	 */
	bool do_reconnect;
};

/**
 * Transport-neutral representation of queued DAP transactions, supporting
 * both JTAG and SWD transports.  All submitted transactions are logically
 * queued, until the queue is executed by run().  Some implementations might
 * execute transactions as soon as they're submitted, but no status is made
 * available until run().
 */
struct dap_ops {
	/** DP register read. */
	int (*queue_dp_read)(struct adiv5_dap *dap, unsigned reg,
			uint32_t *data);
	/** DP register write. */
	int (*queue_dp_write)(struct adiv5_dap *dap, unsigned reg,
			uint32_t data);

	/** AP register read. */
	int (*queue_ap_read)(struct adiv5_ap *ap, unsigned reg,
			uint32_t *data);
	/** AP register write. */
	int (*queue_ap_write)(struct adiv5_ap *ap, unsigned reg,
			uint32_t data);

	/** AP operation abort. */
	int (*queue_ap_abort)(struct adiv5_dap *dap, uint8_t *ack);

	/** Executes all queued DAP operations. */
	int (*run)(struct adiv5_dap *dap);

	/** Executes all queued DAP operations but doesn't check
	 * sticky error conditions */
	int (*sync)(struct adiv5_dap *dap);
};

typedef struct _mem_ap_regs {
	uint32_t	csw;
	uint32_t	tar;
	uint32_t	tar_la;
	/* We do NOT hold DRW, BD0-3. These registers cannot be read until
	 * the memory access has completed */
	uint32_t	mbt;
	uint32_t	base_la;
	uint32_t	cfg;
	uint32_t	base;

	/* ----- Register alike values ----- */

	/* base address from base_la/base register values (New or Legacy) */
	/* Value might be changed after parsing ROM Table */
	uintmax_t	baseaddr;

	uint64_t	PID;	/* bits[7:0] of each PIDR */
	uint32_t	CID;	/* bits[7:0] of each CIDR */

	uint32_t	memtype;	/* ROM Table class only */
} mem_ap_regs_t;

static inline bool is_valid_baseaddr(struct _mem_ap_regs *r)
{
	assert(r != NULL);
	return (r->base != 0xFFFFFFFF);
}
#define	has_mem_ap_entry	is_valid_baseaddr	/* function alias */

typedef struct _rom_entry {
	int32_t addr_ofst;	/* Relative base address of the component (signed) */

	unsigned int power_domain_id:5;	/* Power domain of the component */
	bool power_domain_id_valid:1;	/* Indicates if the Power domain ID is valid */
	bool format:1;		/* 32-bit ROM Table format */
	bool present:1;		/* Entry present */

	uint64_t	PID;	/* bits[7:0] of each PIDR */
	uint32_t	CID;	/* bits[7:0] of each CIDR */

#if 0
	/* WARNING: not sharing(union) with PID.
	 * C compiler does NOT guarantee bit fields mapping.
	 */
	struct _pid {		/* bit fields of PID */
		uint8_t 4kb_log2_count:4;
		uint8_t jep106_cont:4;
		uint8_t revand:4;
		uint8_t cust:4;
		uint8_t rev:4;
		uint8_t is_jep106:1;
		uint8_t jep106_id:7;
		uint16_t partnum:12;
	} pid;
#endif

} rom_entry_t;

/*
 * IDR[27:24]: JEP106 Continuation code
 * IDR[23:17]: JEP106 ID code
 */
static inline uint16_t idr_get_jep106(uint32_t idr)
{
	uint16_t jep106;

	jep106 = (((idr >> IDR_JEP106_CONT_SHIFT) & 0x0F) << 8);	/* bit[27:24] */
	jep106 |= ((idr >> IDR_JEP106_ID_SHIFT) & 0x7F);		/* bit[23:17] */

	return jep106;
}

static inline uint8_t idr_get_id_type(uint32_t idr)
{
	return ((idr & IDR_ID_TYPE_MASK) >> IDR_ID_TYPE_SHIFT);
}

/* NOTE: JEP106 Continuation code & ID code
 *	For easy understanding (i.e.: ARM(0x4, 0x3B) -> 0x43B)
 *	Use 8-bit to hold 7-bit ID code (bit[7] is not used).
 *	Continuation code will be at bit[12:9]
 *	Thus, JEP106_ARM would be 0x43B
 */
/*
 * PID[35:32]: JEP106 Continuation code (ARM is 0x4)
 * PID[19]   : Uses JEP106 ID code      (should be 0b1)
 * PID[18:12]: JEP106 ID code           (ARM is 0x3B)
  */
static inline uint16_t pid_get_jep106(uint64_t pid)
{
	uint16_t jep106;

	jep106 = (((pid >> 32) & 0x0F) << 8);	/* bit[35:32] */
	jep106 |= ((pid >> 12) & 0x7F);		/* bit[18:12] */

	return jep106;	/* jep106 == 0x43B for ARM */
}

/* bit[11:0] of PID */
static inline uint16_t pid_get_partnum(uint64_t pid)
{
	return (uint16_t)(pid & 0x0FFF);
}


extern void jtag_dump_queue(const char *func, int line);


/*
 * Access Port classes (IDR[16:13])
 */
enum ap_class {
	AP_CLASS_NONE   = 0x00000,  /* No class defined */
	AP_CLASS_MEM_AP = 0x10000,  /* MEM-AP */

	AP_CLASS_JTAG = 0x0,	/* No defined class (JTAG-AP) */
	AP_CLASS_MEM  = 0x8	/* MEM-AP (AHB, APB, AXI) */
};

/*
 * Access Port types (IDR[3:0])
 */
enum ap_type {
	AP_TYPE_JTAG_AP = 0x0,  /* JTAG-AP - JTAG master for controlling other JTAG devices */
	AP_TYPE_AHB_AP  = 0x1,  /* AHB Memory-AP */
	AP_TYPE_APB_AP  = 0x2,  /* APB Memory-AP */
	AP_TYPE_AXI_AP  = 0x4,  /* AXI Memory-AP */
};

/**
 * Queue a DP register read.
 * Note that not all DP registers are readable; also, that JTAG and SWD
 * have slight differences in DP register support.
 *
 * @param dap The DAP used for reading.
 * @param reg The two-bit number of the DP register being read.
 * @param data Pointer saying where to store the register's value
 * (in host endianness).
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_dp_read(struct adiv5_dap *dap,
		unsigned reg, uint32_t *data)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_dp_read(dap, reg, data);
}

/**
 * Queue a DP register write.
 * Note that not all DP registers are writable; also, that JTAG and SWD
 * have slight differences in DP register support.
 *
 * @param dap The DAP used for writing.
 * @param reg The two-bit number of the DP register being written.
 * @param data Value being written (host endianness)
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_dp_write(struct adiv5_dap *dap,
		unsigned reg, uint32_t data)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_dp_write(dap, reg, data);
}

/**
 * Queue an AP register read.
 *
 * @param ap The AP used for reading.
 * @param reg The number of the AP register being read.
 * @param data Pointer saying where to store the register's value
 * (in host endianness).
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_read(struct adiv5_ap *ap,
		unsigned reg, uint32_t *data)
{
	assert(ap->dap->ops != NULL);
	return ap->dap->ops->queue_ap_read(ap, reg, data);
}

/**
 * Queue an AP register write.
 *
 * @param ap The AP used for writing.
 * @param reg The number of the AP register being written.
 * @param data Value being written (host endianness)
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_write(struct adiv5_ap *ap,
		unsigned reg, uint32_t data)
{
	assert(ap->dap->ops != NULL);
	return ap->dap->ops->queue_ap_write(ap, reg, data);
}

/**
 * Queue an AP abort operation.  The current AP transaction is aborted,
 * including any update of the transaction counter.  The AP is left in
 * an unknown state (so it must be re-initialized).  For use only after
 * the AP has reported WAIT status for an extended period.
 *
 * @param dap The DAP used for writing.
 * @param ack Pointer to where transaction status will be stored.
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	assert(dap->ops != NULL);
	return dap->ops->queue_ap_abort(dap, ack);
}

/**
 * Perform all queued DAP operations, and clear any errors posted in the
 * CTRL_STAT register when they are done.  Note that if more than one AP
 * operation will be queued, one of the first operations in the queue
 * should probably enable CORUNDETECT in the CTRL/STAT register.
 *
 * @param dap The DAP used.
 *
 * @return ERROR_OK for success, else a fault code.
 */
static inline int dap_run(struct adiv5_dap *dap)
{
	assert(dap->ops != NULL);
	return dap->ops->run(dap);
}

static inline int dap_sync(struct adiv5_dap *dap)
{
	assert(dap->ops != NULL);
	if (dap->ops->sync)
		return dap->ops->sync(dap);
	return ERROR_OK;
}

static inline int dap_dp_read_atomic(struct adiv5_dap *dap, unsigned reg,
				     uint32_t *value)
{
	int retval;

	retval = dap_queue_dp_read(dap, reg, value);
	if (retval != ERROR_OK)
		return retval;

	return dap_run(dap);
}

static inline int dap_dp_write_atomic(struct adiv5_dap *dap, unsigned reg,
				      uint32_t value)
{
	int retval;

	retval = dap_queue_dp_write(dap, reg, value);
	if (retval != ERROR_OK)
		return retval;

	return dap_run(dap);
}

static inline int dap_dp_reg_set_bits(
	struct adiv5_dap *dap, unsigned reg, uint32_t bit_mask)
{
	int rc;
	uint32_t regval = 0;	/* CAUTION: Must clear it, or garbage data
				   in stack will pollute read data.
				   Didn't dig the reason, but verified */

	/*   In the case that this function is invoked by dap_power_on() to
	 * clear error bits, dap_queue_dp_read() should be used,
	 * not dap_dp_read_atomic().  Thus, dap_power_on() is designed in he way
	 * to clear error bits directly.
	 *
	 * Reason:
	 *   This is a two steps function: READ-(set)-WRITE.
	 * Error bits (SSTICKYERR | SSTICKYCMP | SSTICKYORUN) might have been
	 * there when we READ it (that's why we want to clear it).
	 * With dap_dp_read_aotmic() function call, jtagdp_transaction_endcheck()
	 * will throw ERROR_JTAG_DEVICE_ERROR before we have a chance to clear
	 * it (Write 1 to clear).
	 */
	rc = dap_queue_dp_read(dap, reg, &regval);
	if (rc != ERROR_OK)
		return rc;

	regval |= bit_mask;

	return dap_dp_write_atomic(dap, reg, regval);
}

static inline int dap_dp_reg_clear_bits(
	struct adiv5_dap *dap, unsigned reg, uint32_t bit_mask)
{
	int rc;
	uint32_t regval = 0;	/* CAUTION: Must clear to zero */

	rc = dap_queue_dp_read(dap, reg, &regval);
	if (rc != ERROR_OK)
		return rc;

	regval &= ~bit_mask;

	return dap_dp_write_atomic(dap, reg, regval);
}

static inline int dap_dp_poll_register(struct adiv5_dap *dap, unsigned reg,
				       uint32_t mask, uint32_t value, int timeout)
{
	assert(timeout > 0);
	assert((value & mask) == value);

	int ret;
	uint32_t regval;
	LOG_DEBUG("DAP: poll %x, mask 0x%08" PRIx32 ", value 0x%08" PRIx32,
		  reg, mask, value);
	do {
		ret = dap_dp_read_atomic(dap, reg, &regval);
		if (ret != ERROR_OK)
			return ret;

		if ((regval & mask) == value)
			break;

		alive_sleep(10);
	} while (--timeout);

	if (!timeout) {
		LOG_DEBUG("DAP: poll %x timeout", reg);
		return ERROR_WAIT;
	} else {
		return ERROR_OK;
	}
}

/* Queued MEM-AP memory mapped single word transfers. */
int mem_ap_read_u32(struct adiv5_ap *ap,
		uint32_t address, uint32_t *value);
int mem_ap_write_u32(struct adiv5_ap *ap,
		uint32_t address, uint32_t value);

/* Synchronous MEM-AP memory mapped single word transfers. */
int mem_ap_read_atomic_u32(struct adiv5_ap *ap,
		uint32_t address, uint32_t *value);
int mem_ap_write_atomic_u32(struct adiv5_ap *ap,
		uint32_t address, uint32_t value);

/* Synchronous MEM-AP memory mapped single word bits operation */
int mem_ap_set_bits_u32(struct adiv5_dap *swjdp,
		uint32_t address, uint32_t bit_mask);
int mem_ap_clear_bits_u32(struct adiv5_dap *swjdp,
		uint32_t address, uint32_t bit_mask);

/* Synchronous MEM-AP memory mapped bus block transfers. */
int mem_ap_read_buf(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);
int mem_ap_write_buf(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);

/* Synchronous, non-incrementing buffer functions for accessing fifos. */
int mem_ap_read_buf_noincr(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);
int mem_ap_write_buf_noincr(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address);

/* Create DAP struct */
struct adiv5_dap *dap_init(void);

/* Initialisation of the debug system, power domains and registers */
int dap_dp_init(struct adiv5_dap *dap);
int mem_ap_init(struct adiv5_ap *ap);
int debugport_init(struct adiv5_dap *dap);

/* Probe the AP for ROM Table location */
int dap_get_debugbase(struct adiv5_ap *ap,
			uint32_t *dbgbase, uint32_t *apid);

/* Probe Access Ports to find a particular type */
int dap_find_ap(struct adiv5_dap *dap,
			enum ap_type type_to_find,
			struct adiv5_ap **ap_out);

static inline struct adiv5_ap *dap_ap(struct adiv5_dap *dap, uint8_t ap_num)
{
	return &dap->ap[ap_num];
}

/* Lookup CoreSight component */
int dap_lookup_cs_component(struct adiv5_ap *ap,
			uint32_t dbgbase, uint8_t type, uint32_t *addr, int32_t *idx);
/* Look up CoreSight component in ROM Table */
int dap_romtable_lookup_cs_component(
	struct adiv5_dap *dap,
	uintmax_t rombase, uint8_t l_devtype,
	uint16_t l_jep106, uint16_t l_partnum,
	int32_t *l_index,
	uint32_t *found_base);

struct target;

/* Put debug link into SWD mode */
int dap_to_swd(struct target *target);

/* Put debug link into JTAG mode */
int dap_to_jtag(struct target *target);

extern const struct command_registration dap_command_handlers[];

#endif
