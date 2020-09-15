#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/arm_opcodes.h>
#include <target/armv7m.h>


/**
 * @file
 * flash programming support for NXP PN7462 device.
 *
 */

#define PN7462_FLASH_START_ADDRESS  0x00203000
#define PN7462_FLASH_END_ADDRESS    (PN7462_FLASH_START_ADDRESS + (158*1024) - 1)
#define PN7462_FLASH_SIZE                       (PN7462_FLASH_END_ADDRESS - \
		PN7462_FLASH_START_ADDRESS + 1)									/*
														 *Total
														 *Size
														 *of
														 *the
														 *PAGEFLASH
														 *=
														 *160
														 *KBytes
														 **/
#define PN7462_FLASH_PAGE_SIZE      128	/* FLASH is organized as 1280 pages x 128 bytes = 160KBytes
					 **/
#define PN7462_FLASH_PAGE_COUNT     1280


#define PN7462_EEPROM_MEM_START_ADDRESS     0x00201000UL
#define PN7462_EEPROM_USER_START_ADDRESS    (PN7462_EEPROM_MEM_START_ADDRESS + 512)
#define PN7462_EEPROM_SECURITY_ROW_ADDRESS  0x00201000UL/* Address of security row in EEPROM */
#define PN7462_EEPROM_DATA_START_ADDRESS    0x00201004UL
#define PN7462_EEPROM_DATA_END_ADDRESS      0x00201FFFUL

#define PN7462_EEPROM_DATA_SIZE             (PN7462_EEPROM_DATA_END_ADDRESS - \
		PN7462_EEPROM_DATA_START_ADDRESS + 1)
#define PN7462_EEPROM_SIZE                  (PN7462_EEPROM_DATA_END_ADDRESS - \
		PN7462_EEPROM_MEM_START_ADDRESS + 1)
#define PN7462_EEPROM_PAGE_SIZE             64UL/* EEPROM is organized as 64 pages x 64 bytes = 4096
						 *bytes */
#define PN7462_EEPROM_PAGE_COUNT            64
#define PN7462_EEPROM_START_PAGE_NUMBER     0

#define PN7462_WITHIN_EEPROM(ADD)								\
	((ADD) >= PN7462_EEPROM_MEM_START_ADDRESS	    \
		&&												\
		(ADD) <= PN7462_EEPROM_DATA_END_ADDRESS		\
	)

#define PN7462_WITHIN_PAGEFLASH(ADD)						\
	((ADD) >= PN7462_FLASH_START_ADDRESS			\
		&&									      \
		(ADD) <= PN7462_FLASH_END_ADDRESS)

#define FIND_NEXT_64BYTE_BOUNDARY(x) ((((x) & (~(64UL - 1))) + 64UL) - 1)


/**
* \name  EECTRL General Control Register (RW)
* <b>Reset value:</b> 0x001F0000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CTRL                                                                                     ( \
		0x00200000UL)												/**<
															 *rw-*/
#define EE_CTRL_TESTBUS_SELECT_MASK                                                                 ( \
		0x001F0000UL)												/**<
															 *rw
															 *-
															 *Selects
															 *the
															 *internal
															 *EECTRL
															 *Test
															 *Bus.
															 **/
#define EE_CTRL_TESTBUS_SELECT_POS                                                                  ( \
		16UL)
#define EE_CTRL_ECC_PF_AHB_ERROR_ENABLE_MASK                                                        ( \
		0x00008000UL)												/**<
															 *rw
															 *-
															 *When
															 *
															 *1
															 *
															 *enables
															 *the
															 *AHB
															 *error
															 *generation
															 *when
															 *PFLASH
															 *read
															 *data
															 *cannot
															 *be
															 *corrected
															 *by
															 *the
															 *ECC
															 *mechanism.
															 **/
#define EE_CTRL_ECC_PF_AHB_ERROR_ENABLE_POS                                                         ( \
		15UL)
#define EE_CTRL_PFLASH_READ_PREFETCH_DIS_MASK                                                       ( \
		0x00004000UL)												/**<
															 *rw
															 *-
															 *1
															 *disables
															 *the
															 *PAGEFLASH
															 *read
															 *prefetching
															 *feature.
															 **/
#define EE_CTRL_PFLASH_READ_PREFETCH_DIS_POS                                                        ( \
		14UL)
#define EE_CTRL_BLOCK_1_COD_MASK                                                                    ( \
		0x00002000UL)												/**<
															 *rw
															 *-
															 *Block
															 *Mode
															 *for
															 *PAGEFLASH_1.
															 **/
#define EE_CTRL_BLOCK_1_COD_POS                                                                     ( \
		13UL)
#define EE_CTRL_BNWSENS_1_COD_MASK                                                                  ( \
		0x00001000UL)												/**<
															 *rw
															 *-
															 *Voltage
															 *Drop
															 *Sensor
															 *Enable
															 *for
															 *PAGEFLASH_1.
															 **/
#define EE_CTRL_BNWSENS_1_COD_POS                                                                   ( \
		12UL)
#define EE_CTRL_SKIPPRG_1_COD_MASK                                                                  ( \
		0x00000800UL)												/**<
															 *rw
															 *-
															 *Skip
															 *Program
															 *if
															 *erase
															 *fails
															 *for
															 *PAGEFLASH_1.
															 **/
#define EE_CTRL_SKIPPRG_1_COD_POS                                                                   ( \
		11UL)
#define EE_CTRL_STOP_1_COD_MASK                                                                     ( \
		0x00000400UL)												/**<
															 *rw
															 *-
															 *Stop
															 *Ramp-Up
															 *at
															 *Low
															 *Power
															 *for
															 *PAGEFLASH_1
															 *.
															 **/
#define EE_CTRL_STOP_1_COD_POS                                                                      ( \
		10UL)
#define EE_CTRL_PFLASH_DOUT_SYNCHRO_DIS_MASK                                                        ( \
		0x00000200UL)												/**<
															 *rw
															 *-
															 *Reserved.
															 **/
#define EE_CTRL_PFLASH_DOUT_SYNCHRO_DIS_POS                                                         ( \
		9UL)
#define EE_CTRL_POWER_DOWN_1_COD_MASK                                                               ( \
		0x00000100UL)												/**<
															 *rw
															 *-
															 *Power
															 *Down
															 *for
															 *PAGEFLASH_1
															 *Memory.
															 **/
#define EE_CTRL_POWER_DOWN_1_COD_POS                                                                ( \
		8UL)
#define EE_CTRL_BLOCK_0_COD_MASK                                                                    ( \
		0x00000080UL)												/**<
															 *rw
															 *-
															 *Block
															 *Mode
															 *for
															 *PAGEFLASH_0.
															 **/
#define EE_CTRL_BLOCK_0_COD_POS                                                                     ( \
		7UL)
#define EE_CTRL_BNWSENS_0_COD_MASK                                                                  ( \
		0x00000040UL)												/**<
															 *rw
															 *-
															 *When
															 *
															 *0
															 *
															 *output
															 *PAGEFLASH
															 *data
															 *is
															 *synchronized
															 *with
															 *the
															 *system
															 *clock
															 *to
															 *ensure
															 *that
															 *following
															 *ECC
															 *calculation
															 *is
															 *made
															 *on
															 *stable
															 *data
															 *(recommanded
															 *by
															 *the
															 *memories
															 *providers).
															 *It
															 *is
															 *automatically
															 *set
															 *to
															 *
															 *1
															 *
															 *if
															 *FAST_COD
															 *=
															 *
															 *1
															 *..
															 **/
#define EE_CTRL_BNWSENS_0_COD_POS                                                                   ( \
		6UL)
#define EE_CTRL_SKIPPRG_0_COD_MASK                                                                  ( \
		0x00000020UL)												/**<
															 *rw
															 *-
															 *Skip
															 *Program
															 *if
															 *erase
															 *fails
															 *for
															 *PAGEFLASH_0.
															 **/
#define EE_CTRL_SKIPPRG_0_COD_POS                                                                   ( \
		5UL)
#define EE_CTRL_STOP_0_COD_MASK                                                                     ( \
		0x00000010UL)												/**<
															 *rw
															 *-
															 *Stop
															 *Ramp-Up
															 *at
															 *Low
															 *Power
															 *for
															 *PAGEFLASH_0
															 *.
															 **/
#define EE_CTRL_STOP_0_COD_POS                                                                      ( \
		4UL)
#define EE_CTRL_FAST_COD_MASK                                                                       ( \
		0x00000008UL)												/**<
															 *rw
															 *-
															 *Fast
															 *Access
															 *Mode
															 *for
															 *both
															 *PAGEFLASH_0
															 *and
															 *PAGEFLASH_1.
															 **/
#define EE_CTRL_FAST_COD_POS                                                                        ( \
		3UL)
#define EE_CTRL_POWER_DOWN_0_COD_MASK                                                               ( \
		0x00000004UL)												/**<
															 *rw
															 *-
															 *Power
															 *Down
															 *for
															 *PAGEFLASH_0
															 *Memory.
															 **/
#define EE_CTRL_POWER_DOWN_0_COD_POS                                                                ( \
		2UL)
#define EE_CTRL_FAST_DAT_MASK                                                                       ( \
		0x00000002UL)												/**<
															 *rw
															 *-
															 *Fast
															 *EEPROM
															 *Access
															 *Mode.
															 **/
#define EE_CTRL_FAST_DAT_POS                                                                        ( \
		1UL)
#define EE_CTRL_POWER_DOWN_DAT_MASK                                                                 ( \
		0x00000001UL)												/**<
															 *rw
															 *-
															 *Power
															 *Down
															 *for
															 *EEPROM
															 *Memory.
															 **/
#define EE_CTRL_POWER_DOWN_DAT_POS                                                                  ( \
		0UL)
#define EE_CTRL__RESET_VALUE                                                                        ( \
		0x001F0000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name EECTRL Dynamic Control Register (WO)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_DYN                                                                                      ( \
		0x00200004UL)												/**<
															 *-w-*/
#define EE_DYN_EE_RST_1_COD_MASK                                                                    ( \
		0x00800000UL)												/**<
															 *w-
															 *-
															 *Resets
															 *the
															 *PAGEFLASH_1
															 *Controller.
															 **/
#define EE_DYN_EE_RST_1_COD_POS                                                                     ( \
		23UL)
#define EE_DYN_CRC_CLEAR_1_COD_MASK                                                                 ( \
		0x00400000UL)												/**<
															 *w-
															 *-
															 *CRC
															 *Clear
															 *Function
															 *for
															 *PAGEFLASH_1
															 *fulldump
															 *Readout.
															 **/
#define EE_DYN_CRC_CLEAR_1_COD_POS                                                                  ( \
		22UL)
#define EE_DYN_FULL_DUMP_READ_1_COD_MASK                                                            ( \
		0x00200000UL)												/**<
															 *w-
															 *-
															 *Performs
															 *a
															 *Full
															 *Readout
															 *of
															 *the
															 *PAGEFLASH_1
															 *with
															 *CRC
															 *Calculation.
															 **/
#define EE_DYN_FULL_DUMP_READ_1_COD_POS                                                             ( \
		21UL)
#define EE_DYN_PROG_1_COD_MASK                                                                      ( \
		0x00100000UL)												/**<
															 *w-
															 *-
															 *Starts
															 *Programming
															 *PAGEFLASH_1
															 *Row
															 *from
															 *the
															 *Page
															 *Register.
															 **/
#define EE_DYN_PROG_1_COD_POS                                                                       ( \
		20UL)
#define EE_DYN_EE_RST_0_COD_MASK                                                                    ( \
		0x00080000UL)												/**<
															 *w-
															 *-
															 *Resets
															 *the
															 *PAGEFLASH_0
															 *Controller.
															 **/
#define EE_DYN_EE_RST_0_COD_POS                                                                     ( \
		19UL)
#define EE_DYN_CRC_CLEAR_0_COD_MASK                                                                 ( \
		0x00040000UL)												/**<
															 *w-
															 *-
															 *CRC
															 *Clear
															 *Function
															 *for
															 *PAGEFLASH_0
															 *fulldump
															 *Readout.
															 **/
#define EE_DYN_CRC_CLEAR_0_COD_POS                                                                  ( \
		18UL)
#define EE_DYN_FULL_DUMP_READ_0_COD_MASK                                                            ( \
		0x00020000UL)												/**<
															 *w-
															 *-
															 *Performs
															 *a
															 *Full
															 *Readout
															 *of
															 *the
															 *PAGEFLASH_0
															 *with
															 *CRC
															 *Calculation.
															 **/
#define EE_DYN_FULL_DUMP_READ_0_COD_POS                                                             ( \
		17UL)
#define EE_DYN_PROG_0_COD_MASK                                                                      ( \
		0x00010000UL)												/**<
															 *w-
															 *-
															 *Starts
															 *Programming
															 *PAGEFLASH_0
															 *Row
															 *from
															 *the
															 *Page
															 *Register.
															 **/
#define EE_DYN_PROG_0_COD_POS                                                                       ( \
		16UL)
#define EE_DYN_SWD_LOCK_ENABLE_MASK                                                                 ( \
		0x00000010UL)												/**<
															 *w-
															 *-
															 *Lock
															 *Control
															 *of
															 *the
															 *SWD
															 *Interface
															 *in
															 *an
															 *irreversible
															 *way.
															 *Writing
															 *'0'
															 *in
															 *this
															 *register
															 *has
															 *no
															 *influence
															 *and
															 *will
															 *not
															 *unlock
															 *the
															 *SWD..
															 **/
#define EE_DYN_SWD_LOCK_ENABLE_POS                                                                  ( \
		4UL)
#define EE_DYN_EE_RST_DAT_MASK                                                                      ( \
		0x00000008UL)												/**<
															 *w-
															 *-
															 *Resets
															 *the
															 *EEPROM
															 *Controller.
															 **/
#define EE_DYN_EE_RST_DAT_POS                                                                       ( \
		3UL)
#define EE_DYN_CRC_CLEAR_DAT_MASK                                                                   ( \
		0x00000004UL)												/**<
															 *w-
															 *-
															 *CRC
															 *Clear
															 *Function
															 *for
															 *EEPROM
															 *fulldump
															 *Readout.
															 **/
#define EE_DYN_CRC_CLEAR_DAT_POS                                                                    ( \
		2UL)
#define EE_DYN_FULL_DUMP_READ_DAT_MASK                                                              ( \
		0x00000002UL)												/**<
															 *w-
															 *-
															 *Performs
															 *a
															 *Full
															 *Readout
															 *of
															 *the
															 *EEPROM
															 *with
															 *CRC
															 *Calculation.
															 **/
#define EE_DYN_FULL_DUMP_READ_DAT_POS                                                               ( \
		1UL)
#define EE_DYN_PROG_DAT_MASK                                                                        ( \
		0x00000001UL)												/**<
															 *w-
															 *-
															 *Starts
															 *Programming
															 *EEPROM
															 *Row
															 *from
															 *the
															 *Page
															 *Register.
															 **/
#define EE_DYN_PROG_DAT_POS                                                                         ( \
		0UL)
#define EE_DYN__RESET_VALUE                                                                         ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name EECTRL Status Register (RO)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_STAT_DAT                                                                                 ( \
		0x00200008UL)												/**<
															 *r--*/
#define EE_STAT_DAT_SWD_LOCK_MASK                                                                   ( \
		0x01000000UL)												/**<
															 *r-
															 *-
															 *SWD
															 *lock
															 *status
															 *of
															 *the
															 *eectrl_swd_lock_o
															 *output
															 *pin
															 *only
															 *updated
															 *after
															 *reset*.
															 **/
#define EE_STAT_DAT_SWD_LOCK_POS                                                                    ( \
		24UL)
#define EE_STAT_DAT_DFT_LOCK_MASK                                                                   ( \
		0x00800000UL)												/**<
															 *r-
															 *-
															 *DFT
															 *lock
															 *status
															 *of
															 *the
															 *eectrl_dft_lock_o
															 *output
															 *pin
															 *only
															 *updated
															 *after
															 *reset*.
															 **/
#define EE_STAT_DAT_DFT_LOCK_POS                                                                    ( \
		23UL)
#define EE_STAT_DAT_ALL1_DAT_MASK                                                                   ( \
		0x00400000UL)												/**<
															 *r-
															 *-
															 *SFI
															 *(Single
															 *Fault
															 *Injection)
															 *High
															 *Detection
															 *Flag
															 *for
															 *the
															 *EEPROM.
															 **/
#define EE_STAT_DAT_ALL1_DAT_POS                                                                    ( \
		22UL)
#define EE_STAT_DAT_ALL0_DAT_MASK                                                                   ( \
		0x00200000UL)												/**<
															 *r-
															 *-
															 *SFI
															 *(Single
															 *Fault
															 *Injection)
															 *Low
															 *Detection
															 *Flag
															 *for
															 *the
															 *EEPROM.
															 **/
#define EE_STAT_DAT_ALL0_DAT_POS                                                                    ( \
		21UL)
#define EE_STAT_DAT_BNWDROP_DAT_MASK                                                                ( \
		0x00100000UL)												/**<
															 *r-
															 *-
															 *BNW
															 *Sensor
															 *Signal
															 *for
															 *EEPROM.
															 **/
#define EE_STAT_DAT_BNWDROP_DAT_POS                                                                 ( \
		20UL)
#define EE_STAT_DAT_TMANALOG_DAT_MASK                                                               ( \
		0x00080000UL)												/**<
															 *r-
															 *-
															 *Analog
															 *Test
															 *Mode
															 *active.
															 **/
#define EE_STAT_DAT_TMANALOG_DAT_POS                                                                ( \
		19UL)
#define EE_STAT_DAT_EE_EDO_DAT_MASK                                                                 ( \
		0x0007FFF8UL)												/**<
															 *r-
															 *-
															 *EDO
															 *Parity
															 *Lines
															 *for
															 *the
															 *EEPROM.
															 **/
#define EE_STAT_DAT_EE_EDO_DAT_POS                                                                  ( \
		3UL)
#define EE_STAT_DAT_READOUT_ONGOING_DAT_MASK                                                        ( \
		0x00000004UL)												/**<
															 *r-
															 *-
															 *FullDump
															 *Readout
															 *ongoing
															 *for
															 *the
															 *EEPROM.
															 **/
#define EE_STAT_DAT_READOUT_ONGOING_DAT_POS                                                         ( \
		2UL)
#define EE_STAT_DAT_PROG_DAT_MASK                                                                   ( \
		0x00000002UL)												/**<
															 *r-
															 *-
															 *Indicator
															 *if
															 *Programming
															 *is
															 *ongoing
															 *for
															 *the
															 *EEPROM.
															 **/
#define EE_STAT_DAT_PROG_DAT_POS                                                                    ( \
		1UL)
#define EE_STAT_DAT_HVERR_DAT_MASK                                                                  ( \
		0x00000001UL)												/**<
															 *r-
															 *-
															 *HV
															 *Error
															 *Signal
															 *for
															 *EEPROM.
															 **/
#define EE_STAT_DAT_HVERR_DAT_POS                                                                   ( \
		0UL)
#define EE_STAT_DAT__RESET_VALUE                                                                    ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name EECTRL Status Register (RO)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_STAT_COD                                                                                 ( \
		0x0020000CUL)												/**<
															 *r--*/
#define EE_STAT_COD_ECC_READ_INVALID_1_COD_MASK                                                     ( \
		0x02000000UL)												/**<
															 *r-
															 *-
															 *1
															 *means
															 *that
															 *read
															 *access
															 *cannot
															 *be
															 *corrected.
															 *There
															 *are
															 *more
															 *than
															 *one
															 *bit
															 *error..
															 **/
#define EE_STAT_COD_ECC_READ_INVALID_1_COD_POS                                                      ( \
		25UL)
#define EE_STAT_COD_ECC_READ_CORRECT_1_COD_MASK                                                     ( \
		0x01000000UL)												/**<
															 *r-
															 *-
															 *0
															 *means
															 *that
															 *ther
															 *is
															 *at
															 *teast
															 *one
															 *bit
															 *error,
															 *check
															 *the
															 *ECC_READ_INVALID_1_COD
															 *to
															 *confirm
															 *if
															 *it
															 *has
															 *been
															 *corrected
															 *or
															 *not;
															 *1
															 *mean
															 *that
															 *not
															 *bit
															 *error
															 *occured.
															 **/
#define EE_STAT_COD_ECC_READ_CORRECT_1_COD_POS                                                      ( \
		24UL)
#define EE_STAT_COD_EE_EDO_1_COD_MASK                                                               ( \
		0x00FC0000UL)												/**<
															 *r-
															 *-
															 *EDO
															 *Parity
															 *Lines
															 *for
															 *the
															 *PAGEFLASH_1
															 *32-bit
															 *data.
															 **/
#define EE_STAT_COD_EE_EDO_1_COD_POS                                                                ( \
		18UL)
#define EE_STAT_COD_READOUT_ONGOING_1_COD_MASK                                                      ( \
		0x00020000UL)												/**<
															 *r-
															 *-
															 *Full
															 *or
															 *Partial
															 *Dump
															 *Readout
															 *ongoing
															 *for
															 *the
															 *entire
															 *PAGEFLASH_1.
															 **/
#define EE_STAT_COD_READOUT_ONGOING_1_COD_POS                                                       ( \
		17UL)
#define EE_STAT_COD_DROPSENS_1_COD_MASK                                                             ( \
		0x00010000UL)												/**<
															 *r-
															 *-
															 *Drop
															 *Sensor
															 *Output
															 *Signal
															 *for
															 *PAGEFLASH_1.
															 **/
#define EE_STAT_COD_DROPSENS_1_COD_POS                                                              ( \
		16UL)
#define EE_STAT_COD_VMPOK_1_COD_MASK                                                                ( \
		0x00008000UL)												/**<
															 *r-
															 *-
															 *Margin
															 *Voltage
															 *Flag
															 *for
															 *PAGEFLASH_1.
															 **/
#define EE_STAT_COD_VMPOK_1_COD_POS                                                                 ( \
		15UL)
#define EE_STAT_COD_PROG_1_COD_MASK                                                                 ( \
		0x00004000UL)												/**<
															 *r-
															 *-
															 *Indicator
															 *if
															 *Programming
															 *is
															 *ongoing
															 *for
															 *the
															 *PAGEFLASH_1.
															 **/
#define EE_STAT_COD_PROG_1_COD_POS                                                                  ( \
		14UL)
#define EE_STAT_COD_HVERR_1_COD_MASK                                                                ( \
		0x00002000UL)												/**<
															 *r-
															 *-
															 *HV
															 *Error
															 *Signal
															 *for
															 *PAGEFLASH_1.
															 **/
#define EE_STAT_COD_HVERR_1_COD_POS                                                                 ( \
		13UL)
#define EE_STAT_COD_ECC_READ_INVALID_0_COD_MASK                                                     ( \
		0x00001000UL)												/**<
															 *r-
															 *-
															 *1
															 *means
															 *that
															 *read
															 *access
															 *cannot
															 *be
															 *corrected.
															 *There
															 *are
															 *more
															 *than
															 *one
															 *bit
															 *error..
															 **/
#define EE_STAT_COD_ECC_READ_INVALID_0_COD_POS                                                      ( \
		12UL)
#define EE_STAT_COD_ECC_READ_CORRECT_0_COD_MASK                                                     ( \
		0x00000800UL)												/**<
															 *r-
															 *-
															 *0
															 *means
															 *that
															 *ther
															 *is
															 *at
															 *teast
															 *one
															 *bit
															 *error,
															 *check
															 *the
															 *ECC_READ_INVALID_0_COD
															 *to
															 *confirm
															 *if
															 *it
															 *has
															 *been
															 *corrected
															 *or
															 *not;
															 *1
															 *mean
															 *that
															 *not
															 *bit
															 *error
															 *occured.
															 **/
#define EE_STAT_COD_ECC_READ_CORRECT_0_COD_POS                                                      ( \
		11UL)
#define EE_STAT_COD_EE_EDO_0_COD_MASK                                                               ( \
		0x000007E0UL)												/**<
															 *r-
															 *-
															 *EDO
															 *Parity
															 *Lines
															 *for
															 *the
															 *PAGEFLASH_0
															 *32-bit
															 *data.
															 **/
#define EE_STAT_COD_EE_EDO_0_COD_POS                                                                ( \
		5UL)
#define EE_STAT_COD_READOUT_ONGOING_0_COD_MASK                                                      ( \
		0x00000010UL)												/**<
															 *r-
															 *-
															 *Full
															 *or
															 *Partial
															 *Dump
															 *Readout
															 *ongoing
															 *for
															 *the
															 *entire
															 *PAGEFLASH_0.
															 **/
#define EE_STAT_COD_READOUT_ONGOING_0_COD_POS                                                       ( \
		4UL)
#define EE_STAT_COD_DROPSENS_0_COD_MASK                                                             ( \
		0x00000008UL)												/**<
															 *r-
															 *-
															 *Drop
															 *Sensor
															 *Output
															 *Signal
															 *for
															 *PAGEFLASH_0.
															 **/
#define EE_STAT_COD_DROPSENS_0_COD_POS                                                              ( \
		3UL)
#define EE_STAT_COD_VMPOK_0_COD_MASK                                                                ( \
		0x00000004UL)												/**<
															 *r-
															 *-
															 *Margin
															 *Voltage
															 *Flag
															 *for
															 *PAGEFLASH_0.
															 **/
#define EE_STAT_COD_VMPOK_0_COD_POS                                                                 ( \
		2UL)
#define EE_STAT_COD_PROG_0_COD_MASK                                                                 ( \
		0x00000002UL)												/**<
															 *r-
															 *-
															 *Indicator
															 *if
															 *Programming
															 *is
															 *ongoing
															 *for
															 *the
															 *PAGEFLASH_0.
															 **/
#define EE_STAT_COD_PROG_0_COD_POS                                                                  ( \
		1UL)
#define EE_STAT_COD_HVERR_0_COD_MASK                                                                ( \
		0x00000001UL)												/**<
															 *r-
															 *-
															 *HV
															 *Error
															 *Signal
															 *for
															 *PAGEFLASH_0.
															 **/
#define EE_STAT_COD_HVERR_0_COD_POS                                                                 ( \
		0UL)
#define EE_STAT_COD__RESET_VALUE                                                                    ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name Data EEPROM CRC Register (RO)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> r--
* @{
*/
#define EE_CRC_DAT                                                                                  ( \
		0x00200010UL)												/**<
															 *r--*/
#define EE_CRC_DAT_EE_CRC_DAT_MASK                                                                  ( \
		0xFFFFFFFFUL)												/**<
															 *r-
															 *-
															 *EEPROM
															 *CRC
															 *Value
															 *after
															 *a
															 *memory
															 *fulldump.
															 **/
#define EE_CRC_DAT_EE_CRC_DAT_POS                                                                   ( \
		0UL)
#define EE_CRC_DAT__RESET_VALUE                                                                     ( \
		0xFFFFFFFFUL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name Data EEPROM CRC Calculation Stat/End Addresses Register (RW)
* <b>Reset value:</b> 0x07FF0000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_DAT_ADDR                                                                             ( \
		0x00200014UL)												/**<
															 *rw-*/
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_END_MASK                                                    ( \
		0x0FFF0000UL)												/**<
															 *rw
															 *-
															 *EEPROM
															 *CRC
															 *Calculation
															 *End
															 *Address
															 *correponding
															 *to
															 *a
															 *native
															 *16-bit
															 *data
															 *access
															 *(AHB
															 *memory
															 *map
															 *divided
															 *by
															 *2).
															 **/
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_END_POS                                                     ( \
		16UL)
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_START_MASK                                                  ( \
		0x00000FFFUL)												/**<
															 *rw
															 *-
															 *EEPROM
															 *CRC
															 *Calculation
															 *Start
															 *Address
															 *
															 *correponding
															 *to
															 *a
															 *native
															 *16-bit
															 *data
															 *access
															 *(AHB
															 *memory
															 *map
															 *divided
															 *by
															 *2).
															 **/
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_START_POS                                                   ( \
		0UL)
#define EE_CRC_DAT_ADDR__RESET_VALUE                                                                ( \
		0x07FF0000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_1 CRC Init Value Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_1_COD_INIT                                                                           ( \
		0x00200018UL)												/**<
															 *rw-*/
#define EE_CRC_1_COD_INIT_EE_CRC_1_COD_INIT_MASK                                                    ( \
		0xFFFFFFFFUL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_1
															 *CRC
															 *Init
															 *Value.
															 **/
#define EE_CRC_1_COD_INIT_EE_CRC_1_COD_INIT_POS                                                     ( \
		0UL)
#define EE_CRC_1_COD_INIT__RESET_VALUE                                                              ( \
		0xFFFFFFFFUL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_1 CRC Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> r--
* @{
*/
#define EE_CRC_1_COD                                                                                ( \
		0x0020001CUL)												/**<
															 *r--*/
#define EE_CRC_1_COD_EE_CRC_1_COD_MASK                                                              ( \
		0xFFFFFFFFUL)												/**<
															 *r-
															 *-
															 *PAGEFLASH_1
															 *CRC
															 *Value
															 *after
															 *a
															 *memory
															 *fulldump.
															 **/
#define EE_CRC_1_COD_EE_CRC_1_COD_POS                                                               ( \
		0UL)
#define EE_CRC_1_COD__RESET_VALUE                                                                   ( \
		0xFFFFFFFFUL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_1 CRC Calculation Stat/End Addresses Register (RW)
* <b>Reset value:</b> 0x4FFF0000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_1_COD_ADDR                                                                           ( \
		0x00200020UL)												/**<
															 *rw-*/
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_END_MASK                                                ( \
		0xFFFF0000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_1
															 *CRC
															 *Calculation
															 *End
															 *Address
															 *correponding
															 *to
															 *a
															 *native
															 *32-bit
															 *data
															 *access
															 *(AHB
															 *memory
															 *map
															 *divided
															 *by
															 *8).
															 **/
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_END_POS                                                 ( \
		16UL)
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_START_MASK                                              ( \
		0x0000FFFFUL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_1
															 *CRC
															 *Calculation
															 *Start
															 *Address
															 *
															 *correponding
															 *to
															 *a
															 *native
															 *32-bit
															 *data
															 *access
															 *(AHB
															 *memory
															 *map
															 *divided
															 *by
															 *8).
															 **/
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_START_POS                                               ( \
		0UL)
#define EE_CRC_1_COD_ADDR__RESET_VALUE                                                              ( \
		0x4FFF0000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_0 CRC Init Value Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_0_COD_INIT                                                                           ( \
		0x00200024UL)												/**<
															 *rw-*/
#define EE_CRC_0_COD_INIT_EE_CRC_0_COD_INIT_MASK                                                    ( \
		0xFFFFFFFFUL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_0
															 *CRC
															 *Init
															 *Value.
															 **/
#define EE_CRC_0_COD_INIT_EE_CRC_0_COD_INIT_POS                                                     ( \
		0UL)
#define EE_CRC_0_COD_INIT__RESET_VALUE                                                              ( \
		0xFFFFFFFFUL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_0 CRC Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> r--
* @{
*/
#define EE_CRC_0_COD                                                                                ( \
		0x00200028UL)												/**<
															 *r--*/
#define EE_CRC_0_COD_EE_CRC_0_COD_MASK                                                              ( \
		0xFFFFFFFFUL)												/**<
															 *r-
															 *-
															 *PAGEFLASH_0
															 *CRC
															 *Value
															 *after
															 *a
															 *memory
															 *fulldump.
															 **/
#define EE_CRC_0_COD_EE_CRC_0_COD_POS                                                               ( \
		0UL)
#define EE_CRC_0_COD__RESET_VALUE                                                                   ( \
		0xFFFFFFFFUL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_0 CRC Calculation Stat/End Addresses Register (RW)
* <b>Reset value:</b> 0x4FFF0000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_0_COD_ADDR                                                                           ( \
		0x0020002CUL)												/**<
															 *rw-*/
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_END_MASK                                                ( \
		0xFFFF0000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_0
															 *CRC
															 *Calculation
															 *End
															 *Address
															 *correponding
															 *to
															 *a
															 *native
															 *32-bit
															 *data
															 *access
															 *(AHB
															 *memory
															 *map
															 *divided
															 *by
															 *8).
															 **/
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_END_POS                                                 ( \
		16UL)
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_START_MASK                                              ( \
		0x0000FFFFUL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_0
															 *CRC
															 *Calculation
															 *Start
															 *Address
															 *
															 *correponding
															 *to
															 *a
															 *native
															 *32-bit
															 *data
															 *access
															 *(AHB
															 *memory
															 *map
															 *divided
															 *by
															 *8).
															 **/
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_START_POS                                               ( \
		0UL)
#define EE_CRC_0_COD_ADDR__RESET_VALUE                                                              ( \
		0x4FFF0000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name EEPROM Test Support Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_TST_DAT                                                                                  ( \
		0x00200030UL)												/**<
															 *rw-*/
#define EE_TST_DAT_CRC_CALC_WITH_ECC_MASK                                                           ( \
		0x04000000UL)												/**<
															 *rw
															 *-
															 *EEPROM
															 *Dump
															 *CRC
															 *Calculation
															 *Mode
															 *(0b:
															 *without
															 *ECC,
															 *1b:
															 *with
															 *ECC).
															 **/
#define EE_TST_DAT_CRC_CALC_WITH_ECC_POS                                                            ( \
		26UL)
#define EE_TST_DAT_EE_DISECCINV_MASK                                                                ( \
		0x02000000UL)												/**<
															 *rw
															 *-
															 *EEPROM
															 *Disable
															 *Parity
															 *Inversion.
															 **/
#define EE_TST_DAT_EE_DISECCINV_POS                                                                 ( \
		25UL)
#define EE_TST_DAT_EE_ECCTRAN_MASK                                                                  ( \
		0x01000000UL)												/**<
															 *rw
															 *-
															 *EEPROM
															 *Bypass
															 *Error
															 *Correction.
															 **/
#define EE_TST_DAT_EE_ECCTRAN_POS                                                                   ( \
		24UL)
#define EE_TST_DAT_EE_EDI_MASK                                                                      ( \
		0x00FFFF00UL)												/**<
															 *rw
															 *-
															 *EDI
															 *parity
															 *lines
															 *for
															 *EEPROM
															 *when
															 *error
															 *correction
															 *is
															 *bypassed..
															 **/
#define EE_TST_DAT_EE_EDI_POS                                                                       ( \
		8UL)
#define EE_TST_DAT_EE_CTRL_MASK                                                                     ( \
		0x000000FFUL)												/**<
															 *rw
															 *-
															 *EEPROM
															 *Test
															 *Mode.
															 **/
#define EE_TST_DAT_EE_CTRL_POS                                                                      ( \
		0UL)
#define EE_TST_DAT__RESET_VALUE                                                                     ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_1 Test Support Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_TST_1_COD                                                                                ( \
		0x00200034UL)												/**<
															 *rw-*/
#define EE_TST_1_COD_CRC_CALC_WITH_ECC_MASK                                                         ( \
		0x00010000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_1
															 *Dump
															 *CRC
															 *Calculation
															 *Mode
															 *(0b:
															 *without
															 *ECC,
															 *1b:
															 *with
															 *ECC).
															 **/
#define EE_TST_1_COD_CRC_CALC_WITH_ECC_POS                                                          ( \
		16UL)
#define EE_TST_1_COD_EE_DISECCINV_MASK                                                              ( \
		0x00008000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_1
															 *Disable
															 *Parity
															 *Inversion.
															 **/
#define EE_TST_1_COD_EE_DISECCINV_POS                                                               ( \
		15UL)
#define EE_TST_1_COD_EE_ECCTRAN_MASK                                                                ( \
		0x00004000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_1
															 *Bypass
															 *Error
															 *Correction.
															 **/
#define EE_TST_1_COD_EE_ECCTRAN_POS                                                                 ( \
		14UL)
#define EE_TST_1_COD_EE_EDI_MASK                                                                    ( \
		0x00003F00UL)												/**<
															 *rw
															 *-
															 *EDI
															 *parity
															 *lines
															 *for
															 *PAGEFLASH_1
															 *when
															 *error
															 *correction
															 *is
															 *bypassed.
															 **/
#define EE_TST_1_COD_EE_EDI_POS                                                                     ( \
		8UL)
#define EE_TST_1_COD_EE_CTRL_MASK                                                                   ( \
		0x000000FFUL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_1
															 *Test
															 *Mode.
															 **/
#define EE_TST_1_COD_EE_CTRL_POS                                                                    ( \
		0UL)
#define EE_TST_1_COD__RESET_VALUE                                                                   ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PAGEFLASH_0 Test Support Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_TST_0_COD                                                                                ( \
		0x00200038UL)												/**<
															 *rw-*/
#define EE_TST_0_COD_CRC_CALC_WITH_ECC_MASK                                                         ( \
		0x00010000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_0
															 *Dump
															 *CRC
															 *Calculation
															 *Mode
															 *(0b:
															 *without
															 *ECC,
															 *1b:
															 *with
															 *ECC).
															 **/
#define EE_TST_0_COD_CRC_CALC_WITH_ECC_POS                                                          ( \
		16UL)
#define EE_TST_0_COD_EE_DISECCINV_MASK                                                              ( \
		0x00008000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_0
															 *Disable
															 *Parity
															 *Inversion.
															 **/
#define EE_TST_0_COD_EE_DISECCINV_POS                                                               ( \
		15UL)
#define EE_TST_0_COD_EE_ECCTRAN_MASK                                                                ( \
		0x00004000UL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_0
															 *Bypass
															 *Error
															 *Correction.
															 **/
#define EE_TST_0_COD_EE_ECCTRAN_POS                                                                 ( \
		14UL)
#define EE_TST_0_COD_EE_EDI_MASK                                                                    ( \
		0x00003F00UL)												/**<
															 *rw
															 *-
															 *EDI
															 *parity
															 *lines
															 *for
															 *PAGEFLASH_0
															 *when
															 *error
															 *correction
															 *is
															 *bypassed.
															 **/
#define EE_TST_0_COD_EE_EDI_POS                                                                     ( \
		8UL)
#define EE_TST_0_COD_EE_CTRL_MASK                                                                   ( \
		0x000000FFUL)												/**<
															 *rw
															 *-
															 *PAGEFLASH_0
															 *Test
															 *Mode.
															 **/
#define EE_TST_0_COD_EE_CTRL_POS                                                                    ( \
		0UL)
#define EE_TST_0_COD__RESET_VALUE                                                                   ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name Memories Trimming Values Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rwm
* @{
*/
#define EE_TRIMM                                                                                    ( \
		0x0020003CUL)												/**<
															 *rwm*/
#define EE_TRIMM_HVTRIMW_1_COD_MASK                                                                 ( \
		0x00F00000UL)												/**<
															 *rw
															 *-
															 *HV
															 *Program
															 *Trimming
															 *Value
															 *for
															 *the
															 *PAGEFLASH_1.
															 **/
#define EE_TRIMM_HVTRIMW_1_COD_POS                                                                  ( \
		20UL)
#define EE_TRIMM_HVTRIME_1_COD_MASK                                                                 ( \
		0x000F0000UL)												/**<
															 *rw
															 *-
															 *HV
															 *Erase
															 *Trimming
															 *value
															 *for
															 *the
															 *PAGEFLASH_1.
															 **/
#define EE_TRIMM_HVTRIME_1_COD_POS                                                                  ( \
		16UL)
#define EE_TRIMM_HVTRIMW_0_COD_MASK                                                                 ( \
		0x0000F000UL)												/**<
															 *rw
															 *-
															 *HV
															 *Program
															 *Trimming
															 *Value
															 *for
															 *the
															 *PAGEFLASH_0.
															 **/
#define EE_TRIMM_HVTRIMW_0_COD_POS                                                                  ( \
		12UL)
#define EE_TRIMM_HVTRIME_0_COD_MASK                                                                 ( \
		0x00000F00UL)												/**<
															 *rw
															 *-
															 *HV
															 *Erase
															 *Trimming
															 *value
															 *for
															 *the
															 *PAGEFLASH_0.
															 **/
#define EE_TRIMM_HVTRIME_0_COD_POS                                                                  ( \
		8UL)
#define EE_TRIMM_HVTRIMW_DAT_MASK                                                                   ( \
		0x000000F0UL)												/**<
															 *rw
															 *-
															 *HV
															 *Program
															 *Trimming
															 *Value
															 *for
															 *the
															 *EEPROM.
															 **/
#define EE_TRIMM_HVTRIMW_DAT_POS                                                                    ( \
		4UL)
#define EE_TRIMM_HVTRIME_DAT_MASK                                                                   ( \
		0x0000000FUL)												/**<
															 *rw
															 *-
															 *HV
															 *Erase
															 *Trimming
															 *value
															 *for
															 *the
															 *EEPROM.
															 **/
#define EE_TRIMM_HVTRIME_DAT_POS                                                                    ( \
		0UL)
#define EE_TRIMM__RESET_VALUE                                                                       ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_ECC_PF_AHB_ERROR_ADDR                                                                    ( \
		0x00200044UL)												/**<
															 *rw-*/
#define EE_ECC_PF_AHB_ERROR_ADDR_ECC_PF_AHB_ERROR_ADDR_MASK                                         ( \
		0x0003FFFFUL)												/**<
															 *r-
															 *-
															 *PFLASH
															 *ECC
															 *Error
															 *Address.
															 **/
#define EE_ECC_PF_AHB_ERROR_ADDR_ECC_PF_AHB_ERROR_ADDR_POS                                          ( \
		0UL)
#define EE_ECC_PF_AHB_ERROR_ADDR__RESET_VALUE                                                       ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_CLR_ENABLE                                                                           ( \
		0x00200FD8UL)												/**<
															 *-w-*/
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_ENABLE_MASK                         ( \
		0x00000200UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_ENABLE_POS                          ( \
		9UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_CLR_ENABLE_MASK                             ( \
		0x00000100UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_CLR_ENABLE_POS                              ( \
		8UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_ENABLE_MASK                         ( \
		0x00000080UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_ENABLE_POS                          ( \
		7UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_CLR_ENABLE_MASK                             ( \
		0x00000040UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_CLR_ENABLE_POS                              ( \
		6UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_1_COD_INT_CLR_ENABLE_MASK                                        ( \
		0x00000020UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_HVERR_1_COD_INT_CLR_ENABLE_POS                                         ( \
		5UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_0_COD_INT_CLR_ENABLE_MASK                                        ( \
		0x00000010UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_HVERR_0_COD_INT_CLR_ENABLE_POS                                         ( \
		4UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_DAT_INT_CLR_ENABLE_MASK                                          ( \
		0x00000008UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_HVERR_DAT_INT_CLR_ENABLE_POS                                           ( \
		3UL)
#define EE_INT_CLR_ENABLE_EE_PROG_1_COD_COMPLETED_INT_CLR_ENABLE_MASK                               ( \
		0x00000004UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Programming
															 *Completed
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_PROG_1_COD_COMPLETED_INT_CLR_ENABLE_POS                                ( \
		2UL)
#define EE_INT_CLR_ENABLE_EE_PROG_0_COD_COMPLETED_INT_CLR_ENABLE_MASK                               ( \
		0x00000002UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Programming
															 *Completed
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_PROG_0_COD_COMPLETED_INT_CLR_ENABLE_POS                                ( \
		1UL)
#define EE_INT_CLR_ENABLE_EE_PROG_DAT_COMPLETED_INT_CLR_ENABLE_MASK                                 ( \
		0x00000001UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *Programming
															 *Completed
															 *Interrupt
															 *Clear
															 *Enable
															 *Command.
															 **/
#define EE_INT_CLR_ENABLE_EE_PROG_DAT_COMPLETED_INT_CLR_ENABLE_POS                                  ( \
		0UL)
#define EE_INT_CLR_ENABLE__RESET_VALUE                                                              ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_SET_ENABLE                                                                           ( \
		0x00200FDCUL)												/**<
															 *-w-*/
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_ENABLE_MASK                         ( \
		0x00000200UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_ENABLE_POS                          ( \
		9UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_SET_ENABLE_MASK                             ( \
		0x00000100UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_SET_ENABLE_POS                              ( \
		8UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_ENABLE_MASK                         ( \
		0x00000080UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_ENABLE_POS                          ( \
		7UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_SET_ENABLE_MASK                             ( \
		0x00000040UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_SET_ENABLE_POS                              ( \
		6UL)
#define EE_INT_SET_ENABLE_EE_HVERR_1_COD_INT_SET_ENABLE_MASK                                        ( \
		0x00000020UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_HVERR_1_COD_INT_SET_ENABLE_POS                                         ( \
		5UL)
#define EE_INT_SET_ENABLE_EE_HVERR_0_COD_INT_SET_ENABLE_MASK                                        ( \
		0x00000010UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_HVERR_0_COD_INT_SET_ENABLE_POS                                         ( \
		4UL)
#define EE_INT_SET_ENABLE_EE_HVERR_DAT_INT_SET_ENABLE_MASK                                          ( \
		0x00000008UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_HVERR_DAT_INT_SET_ENABLE_POS                                           ( \
		3UL)
#define EE_INT_SET_ENABLE_EE_PROG_1_COD_COMPLETED_INT_SET_ENABLE_MASK                               ( \
		0x00000004UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Programming
															 *Completed
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_PROG_1_COD_COMPLETED_INT_SET_ENABLE_POS                                ( \
		2UL)
#define EE_INT_SET_ENABLE_EE_PROG_0_COD_COMPLETED_INT_SET_ENABLE_MASK                               ( \
		0x00000002UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Programming
															 *Completed
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_PROG_0_COD_COMPLETED_INT_SET_ENABLE_POS                                ( \
		1UL)
#define EE_INT_SET_ENABLE_EE_PROG_DAT_COMPLETED_INT_SET_ENABLE_MASK                                 ( \
		0x00000001UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *Programming
															 *Completed
															 *Interrupt
															 *Set
															 *Enable
															 *Command.
															 **/
#define EE_INT_SET_ENABLE_EE_PROG_DAT_COMPLETED_INT_SET_ENABLE_POS                                  ( \
		0UL)
#define EE_INT_SET_ENABLE__RESET_VALUE                                                              ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_INT_STATUS                                                                               ( \
		0x00200FE0UL)												/**<
															 *r--*/
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_STATUS_MASK                                 ( \
		0x00000200UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_STATUS_POS                                  ( \
		9UL)
#define EE_INT_STATUS_EE_ECC_READ_INVALID_1_COD_INT_STATUS_MASK                                     ( \
		0x00000100UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_ECC_READ_INVALID_1_COD_INT_STATUS_POS                                      ( \
		8UL)
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_STATUS_MASK                                 ( \
		0x00000080UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_STATUS_POS                                  ( \
		7UL)
#define EE_INT_STATUS_EE_ECC_READ_INVALID_0_COD_INT_STATUS_MASK                                     ( \
		0x00000040UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_ECC_READ_INVALID_0_COD_INT_STATUS_POS                                      ( \
		6UL)
#define EE_INT_STATUS_EE_HVERR_1_COD_INT_STATUS_MASK                                                ( \
		0x00000020UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_HVERR_1_COD_INT_STATUS_POS                                                 ( \
		5UL)
#define EE_INT_STATUS_EE_HVERR_0_COD_INT_STATUS_MASK                                                ( \
		0x00000010UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_HVERR_0_COD_INT_STATUS_POS                                                 ( \
		4UL)
#define EE_INT_STATUS_EE_HVERR_DAT_INT_STATUS_MASK                                                  ( \
		0x00000008UL)												/**<
															 *r-
															 *-
															 *EEPROM
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_HVERR_DAT_INT_STATUS_POS                                                   ( \
		3UL)
#define EE_INT_STATUS_EE_PROG_1_COD_COMPLETED_INT_STATUS_MASK                                       ( \
		0x00000004UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *Programming
															 *Completed
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_PROG_1_COD_COMPLETED_INT_STATUS_POS                                        ( \
		2UL)
#define EE_INT_STATUS_EE_PROG_0_COD_COMPLETED_INT_STATUS_MASK                                       ( \
		0x00000002UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *Programming
															 *Completed
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_PROG_0_COD_COMPLETED_INT_STATUS_POS                                        ( \
		1UL)
#define EE_INT_STATUS_EE_PROG_DAT_COMPLETED_INT_STATUS_MASK                                         ( \
		0x00000001UL)												/**<
															 *r-
															 *-
															 *EEPROM
															 *Programming
															 *Completed
															 *Interrupt
															 *Status
															 *Variable.
															 **/
#define EE_INT_STATUS_EE_PROG_DAT_COMPLETED_INT_STATUS_POS                                          ( \
		0UL)
#define EE_INT_STATUS__RESET_VALUE                                                                  ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_INT_ENABLE                                                                               ( \
		0x00200FE4UL)												/**<
															 *r--*/
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_ENABLE_MASK                                 ( \
		0x00000200UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_ENABLE_POS                                  ( \
		9UL)
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_ENABLE_MASK                                     ( \
		0x00000100UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_ENABLE_POS                                      ( \
		8UL)
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_ENABLE_MASK                                 ( \
		0x00000080UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_ENABLE_POS                                  ( \
		7UL)
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_ENABLE_MASK                                     ( \
		0x00000040UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_ENABLE_POS                                      ( \
		6UL)
#define EE_INT_ENABLE_EE_HVERR_1_COD_INT_ENABLE_MASK                                                ( \
		0x00000020UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_HVERR_1_COD_INT_ENABLE_POS                                                 ( \
		5UL)
#define EE_INT_ENABLE_EE_HVERR_0_COD_INT_ENABLE_MASK                                                ( \
		0x00000010UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_HVERR_0_COD_INT_ENABLE_POS                                                 ( \
		4UL)
#define EE_INT_ENABLE_EE_HVERR_DAT_INT_ENABLE_MASK                                                  ( \
		0x00000008UL)												/**<
															 *r-
															 *-
															 *EEPROM
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_HVERR_DAT_INT_ENABLE_POS                                                   ( \
		3UL)
#define EE_INT_ENABLE_EE_PROG_1_COD_COMPLETED_INT_ENABLE_MASK                                       ( \
		0x00000004UL)												/**<
															 *r-
															 *-
															 *PFLASH_1
															 *Programming
															 *Completed
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_PROG_1_COD_COMPLETED_INT_ENABLE_POS                                        ( \
		2UL)
#define EE_INT_ENABLE_EE_PROG_0_COD_COMPLETED_INT_ENABLE_MASK                                       ( \
		0x00000002UL)												/**<
															 *r-
															 *-
															 *PFLASH_0
															 *Programming
															 *Completed
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_PROG_0_COD_COMPLETED_INT_ENABLE_POS                                        ( \
		1UL)
#define EE_INT_ENABLE_EE_PROG_DAT_COMPLETED_INT_ENABLE_MASK                                         ( \
		0x00000001UL)												/**<
															 *r-
															 *-
															 *EEPROM
															 *Programming
															 *Completed
															 *Interrupt
															 *Enable
															 *Variable.
															 **/
#define EE_INT_ENABLE_EE_PROG_DAT_COMPLETED_INT_ENABLE_POS                                          ( \
		0UL)
#define EE_INT_ENABLE__RESET_VALUE                                                                  ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_CLR_STATUS                                                                           ( \
		0x00200FE8UL)												/**<
															 *-w-*/
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_STATUS_MASK                         ( \
		0x00000200UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_STATUS_POS                          ( \
		9UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_1_COD_INT_CLR_STATUS_MASK                             ( \
		0x00000100UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_1_COD_INT_CLR_STATUS_POS                              ( \
		8UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_STATUS_MASK                         ( \
		0x00000080UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_STATUS_POS                          ( \
		7UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_0_COD_INT_CLR_STATUS_MASK                             ( \
		0x00000040UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_0_COD_INT_CLR_STATUS_POS                              ( \
		6UL)
#define EE_INT_CLR_STATUS_EE_HVERR_1_COD_INT_CLR_STATUS_MASK                                        ( \
		0x00000020UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_HVERR_1_COD_INT_CLR_STATUS_POS                                         ( \
		5UL)
#define EE_INT_CLR_STATUS_EE_HVERR_0_COD_INT_CLR_STATUS_MASK                                        ( \
		0x00000010UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_HVERR_0_COD_INT_CLR_STATUS_POS                                         ( \
		4UL)
#define EE_INT_CLR_STATUS_EE_HVERR_DAT_INT_CLR_STATUS_MASK                                          ( \
		0x00000008UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_HVERR_DAT_INT_CLR_STATUS_POS                                           ( \
		3UL)
#define EE_INT_CLR_STATUS_EE_PROG_1_COD_COMPLETED_INT_CLR_STATUS_MASK                               ( \
		0x00000004UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Programming
															 *Completed
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_PROG_1_COD_COMPLETED_INT_CLR_STATUS_POS                                ( \
		2UL)
#define EE_INT_CLR_STATUS_EE_PROG_0_COD_COMPLETED_INT_CLR_STATUS_MASK                               ( \
		0x00000002UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Programming
															 *Completed
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_PROG_0_COD_COMPLETED_INT_CLR_STATUS_POS                                ( \
		1UL)
#define EE_INT_CLR_STATUS_EE_PROG_DAT_COMPLETED_INT_CLR_STATUS_MASK                                 ( \
		0x00000001UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *Programming
															 *Completed
															 *Interrupt
															 *Clear
															 *Status
															 *Command.
															 **/
#define EE_INT_CLR_STATUS_EE_PROG_DAT_COMPLETED_INT_CLR_STATUS_POS                                  ( \
		0UL)
#define EE_INT_CLR_STATUS__RESET_VALUE                                                              ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_SET_STATUS                                                                           ( \
		0x00200FECUL)												/**<
															 *-w-*/
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_STATUS_MASK                         ( \
		0x00000200UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_STATUS_POS                          ( \
		9UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_1_COD_INT_SET_STATUS_MASK                             ( \
		0x00000100UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_1_COD_INT_SET_STATUS_POS                              ( \
		8UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_STATUS_MASK                         ( \
		0x00000080UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Not
															 *Correct
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_STATUS_POS                          ( \
		7UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_0_COD_INT_SET_STATUS_MASK                             ( \
		0x00000040UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Invalid
															 *ECC
															 *Read
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_0_COD_INT_SET_STATUS_POS                              ( \
		6UL)
#define EE_INT_SET_STATUS_EE_HVERR_1_COD_INT_SET_STATUS_MASK                                        ( \
		0x00000020UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_HVERR_1_COD_INT_SET_STATUS_POS                                         ( \
		5UL)
#define EE_INT_SET_STATUS_EE_HVERR_0_COD_INT_SET_STATUS_MASK                                        ( \
		0x00000010UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_HVERR_0_COD_INT_SET_STATUS_POS                                         ( \
		4UL)
#define EE_INT_SET_STATUS_EE_HVERR_DAT_INT_SET_STATUS_MASK                                          ( \
		0x00000008UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *High
															 *Voltage
															 *Error
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_HVERR_DAT_INT_SET_STATUS_POS                                           ( \
		3UL)
#define EE_INT_SET_STATUS_EE_PROG_1_COD_COMPLETED_INT_SET_STATUS_MASK                               ( \
		0x00000004UL)												/**<
															 *w-
															 *-
															 *PFLASH_1
															 *Programming
															 *Completed
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_PROG_1_COD_COMPLETED_INT_SET_STATUS_POS                                ( \
		2UL)
#define EE_INT_SET_STATUS_EE_PROG_0_COD_COMPLETED_INT_SET_STATUS_MASK                               ( \
		0x00000002UL)												/**<
															 *w-
															 *-
															 *PFLASH_0
															 *Programming
															 *Completed
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_PROG_0_COD_COMPLETED_INT_SET_STATUS_POS                                ( \
		1UL)
#define EE_INT_SET_STATUS_EE_PROG_DAT_COMPLETED_INT_SET_STATUS_MASK                                 ( \
		0x00000001UL)												/**<
															 *w-
															 *-
															 *EEPROM
															 *Programming
															 *Completed
															 *Interrupt
															 *Set
															 *Status
															 *Command.
															 **/
#define EE_INT_SET_STATUS_EE_PROG_DAT_COMPLETED_INT_SET_STATUS_POS                                  ( \
		0UL)
#define EE_INT_SET_STATUS__RESET_VALUE                                                              ( \
		0x00000000UL)												/**<
															 *value
															 *after
															 *reset
															 **/
/**
* @}
*/


/* static void pn7462_flash_hw_reset(struct target *target) */
/* { */
/*	target_write_u32(target, EE_DYN, EE_DYN_EE_RST_0_COD_POS); */
/*	target_write_u32(target, EE_DYN, EE_DYN_EE_RST_1_COD_POS); */
/* } */

/*
 * flash bank pn7462 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(pn7462_flash_bank_command)
{
	if (CMD_ARGC < 5)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static int pn7462_write_flash(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	if (!PN7462_WITHIN_PAGEFLASH(bank->base + offset)) {
		LOG_ERROR(
			"pn7462_write_flash: given address to write (%lu) not within flash memory address space.",
			bank->base + offset);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (count % 128 != 0) {
		LOG_ERROR("pn7462_write_flash: given size (%u) is not a multiple of 128.", count);

		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	uint32_t bytes_remaining = count;
	uint32_t index = 0;

	while (bytes_remaining >= 128) {
		target_write_buffer(bank->target, bank->base + offset + index, 128, buffer + index);
		index += 128;
		bytes_remaining -= 128;

		target_write_u32(bank->target, EE_DYN,
			(EE_DYN_PROG_0_COD_MASK | EE_DYN_PROG_1_COD_MASK));

		uint32_t eestatcod;
		target_read_u32(bank->target, EE_STAT_COD, &eestatcod);

		if ((eestatcod & (1 << EE_STAT_COD_HVERR_0_COD_POS)) ||
			(eestatcod & (1 << EE_STAT_COD_HVERR_1_COD_POS)))
			return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int pn7462_write_eeprom(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	if (!PN7462_WITHIN_EEPROM(bank->base + offset)) {
		LOG_ERROR(
			"pn7462_write_eeprom: given address to write (%lu) not within eeprom memory address space.",
			bank->base + offset);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (count % 64 != 0) {
		LOG_ERROR("pn7462_write_eeprom: given size (%u) is not a multiple of 64.", count);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	uint32_t bytes_remaining = count;
	uint32_t index = 0;

	while (bytes_remaining >= 64) {
		target_write_buffer(bank->target, bank->base + offset + index, 64, buffer + index);
		index += 64;
		bytes_remaining -= 64;

		target_write_u32(bank->target, EE_DYN, 1 << EE_DYN_PROG_DAT_POS);

		uint32_t eestatdat;
		target_read_u32(bank->target, EE_STAT_DAT, &eestatdat);

		if (eestatdat & (1 << EE_STAT_DAT_HVERR_DAT_POS)) {
			LOG_ERROR("pn7462_write_eeprom: EE_STAT_DAT_HVERR_DAT_POS error");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int pn7462_write(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}


	LOG_DEBUG("pn7462_write called with address 0x%lx, count:%d. isflash: %d. iseeprom: %d",
		bank->base + offset,
		count,
		PN7462_WITHIN_PAGEFLASH(bank->base + offset),
		PN7462_WITHIN_EEPROM(bank->base + offset)
		);

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (PN7462_WITHIN_PAGEFLASH(bank->base + offset))
		return pn7462_write_flash(bank, buffer, offset, count);
	else if (PN7462_WITHIN_EEPROM(bank->base + offset))
		return pn7462_write_eeprom(bank, buffer, offset, count);
	else
		return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int pn7462_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int lverr = ERROR_OK;

	/* if first and last equals 0, erase eeprom */
	/* if first and last equals 1, erase flash */

	if (first == 0 && last == 0) {
		const uint8_t empty_page_eeprom[64] = { 0 };
		const uint8_t empty_page_flash[128] = { 0 };

		const uint8_t *empty_page = NULL;
		if (bank->bus_width == 64)
			empty_page = empty_page_eeprom;
		else if (bank->bus_width == 128)
			empty_page = empty_page_flash;
		else
			return ERROR_FLASH_SECTOR_NOT_ERASED;

		for (unsigned i = 0; i * bank->bus_width < bank->sectors[0].size; i++) {
			lverr = pn7462_write(bank, empty_page, i * bank->bus_width, bank->bus_width);
			if (lverr != ERROR_OK)
				return lverr;
		}
	} else
		return ERROR_FLASH_SECTOR_NOT_ERASED;

	return ERROR_OK;
}

static int pn7462_probe(struct flash_bank *bank)
{
	/* the flash controller hides the sectors. */

	/* create one sector for the flash memory and one for eeprom */
	bank->sectors = malloc(sizeof(struct flash_sector));

	bank->sectors[0].is_protected = 0;
	bank->sectors[0].size = bank->size;
	bank->sectors[0].offset = 0;

	bank->num_sectors = 1;

	return ERROR_OK;
}

static int pn7462_erase_check(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

static int get_pn7462_info(struct flash_bank *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "pn7462 flash");

	return ERROR_OK;
}

static const struct command_registration pn7462_command_handlers[] = {
	{
		.name = "pn7462",
		.mode = COMMAND_ANY,
		.help = "pn7462 flash command group",
		.usage = "",
		.chain = NULL,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver pn7462_flash = {
	.name = "pn7462",
	.commands = pn7462_command_handlers,
	.flash_bank_command = pn7462_flash_bank_command,
	.erase = pn7462_erase,
	.write = pn7462_write,
	.read = default_flash_read,
	.probe = pn7462_probe,
	.auto_probe = pn7462_probe,
	.erase_check = pn7462_erase_check,
	.info = get_pn7462_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
