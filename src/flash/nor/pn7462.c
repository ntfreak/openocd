/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This implements the flash driver for the NXP PN7462.
 * Copyright (C) 2020 Rick Veens <rickveens92@gmail.com>
 */

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
		PN7462_FLASH_START_ADDRESS + 1)
/* Total Size of the PAGEFLASH = 160 KBytes */
#define PN7462_FLASH_PAGE_SIZE      128
/* FLASH is organized as 1280 pages x 128 bytes = 160KBytes */
#define PN7462_FLASH_PAGE_COUNT     1280


#define PN7462_EEPROM_MEM_START_ADDRESS     0x00201000UL
#define PN7462_EEPROM_USER_START_ADDRESS    (PN7462_EEPROM_MEM_START_ADDRESS + 512)
/* Address of security row in EEPROM */
#define PN7462_EEPROM_SECURITY_ROW_ADDRESS  0x00201000UL
#define PN7462_EEPROM_DATA_START_ADDRESS    0x00201004UL
#define PN7462_EEPROM_DATA_END_ADDRESS      0x00201FFFUL

#define PN7462_EEPROM_DATA_SIZE             (PN7462_EEPROM_DATA_END_ADDRESS - \
		PN7462_EEPROM_DATA_START_ADDRESS + 1)
#define PN7462_EEPROM_SIZE                  (PN7462_EEPROM_DATA_END_ADDRESS - \
		PN7462_EEPROM_MEM_START_ADDRESS + 1)
/* EEPROM is organized as 64 pages x 64 bytes = 4096bytes */
#define PN7462_EEPROM_PAGE_SIZE             64UL
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
#define EE_CTRL                                                              (0x00200000UL)
#define EE_CTRL_TESTBUS_SELECT_MASK                                          (0x001F0000UL)
#define EE_CTRL_TESTBUS_SELECT_POS                                           (16UL)
#define EE_CTRL_ECC_PF_AHB_ERROR_ENABLE_MASK                                 (0x00008000UL)
#define EE_CTRL_ECC_PF_AHB_ERROR_ENABLE_POS                                  (15UL)
#define EE_CTRL_PFLASH_READ_PREFETCH_DIS_MASK                                (0x00004000UL)
#define EE_CTRL_PFLASH_READ_PREFETCH_DIS_POS                                 (14UL)
#define EE_CTRL_BLOCK_1_COD_MASK                                             (0x00002000UL)
#define EE_CTRL_BLOCK_1_COD_POS                                              (13UL)
#define EE_CTRL_BNWSENS_1_COD_MASK                                           (0x00001000UL)
#define EE_CTRL_BNWSENS_1_COD_POS                                            (12UL)
#define EE_CTRL_SKIPPRG_1_COD_MASK                                           (0x00000800UL)
#define EE_CTRL_SKIPPRG_1_COD_POS                                            (11UL)
#define EE_CTRL_STOP_1_COD_MASK                                              (0x00000400UL)
#define EE_CTRL_STOP_1_COD_POS                                               (10UL)
#define EE_CTRL_PFLASH_DOUT_SYNCHRO_DIS_MASK                                 (0x00000200UL)
#define EE_CTRL_PFLASH_DOUT_SYNCHRO_DIS_POS                                  (9UL)
#define EE_CTRL_POWER_DOWN_1_COD_MASK                                        (0x00000100UL)
#define EE_CTRL_POWER_DOWN_1_COD_POS                                         (8UL)
#define EE_CTRL_BLOCK_0_COD_MASK                                             (0x00000080UL)
#define EE_CTRL_BLOCK_0_COD_POS                                              (7UL)
#define EE_CTRL_BNWSENS_0_COD_MASK                                           (0x00000040UL)
#define EE_CTRL_BNWSENS_0_COD_POS                                            (6UL)
#define EE_CTRL_SKIPPRG_0_COD_MASK                                           (0x00000020UL)
#define EE_CTRL_SKIPPRG_0_COD_POS                                            (5UL)
#define EE_CTRL_STOP_0_COD_MASK                                              (0x00000010UL)
#define EE_CTRL_STOP_0_COD_POS                                               (4UL)
#define EE_CTRL_FAST_COD_MASK                                                (0x00000008UL)
#define EE_CTRL_FAST_COD_POS                                                 (3UL)
#define EE_CTRL_POWER_DOWN_0_COD_MASK                                        (0x00000004UL)
#define EE_CTRL_POWER_DOWN_0_COD_POS                                         (2UL)
#define EE_CTRL_FAST_DAT_MASK                                                (0x00000002UL)
#define EE_CTRL_FAST_DAT_POS                                                 (1UL)
#define EE_CTRL_POWER_DOWN_DAT_MASK                                          (0x00000001UL)
#define EE_CTRL_POWER_DOWN_DAT_POS                                           (0UL)
#define EE_CTRL__RESET_VALUE                                                 (0x001F0000UL)
/**
* @}
*/


/**
* \name EECTRL Dynamic Control Register (WO)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_DYN                                                               (0x00200004UL)
#define EE_DYN_EE_RST_1_COD_MASK                                             (0x00800000UL)
#define EE_DYN_EE_RST_1_COD_POS                                              (23UL)
#define EE_DYN_CRC_CLEAR_1_COD_MASK                                          (0x00400000UL)
#define EE_DYN_CRC_CLEAR_1_COD_POS                                           (22UL)
#define EE_DYN_FULL_DUMP_READ_1_COD_MASK                                     (0x00200000UL)
#define EE_DYN_FULL_DUMP_READ_1_COD_POS                                      (21UL)
#define EE_DYN_PROG_1_COD_MASK                                               (0x00100000UL)
#define EE_DYN_PROG_1_COD_POS                                                (20UL)
#define EE_DYN_EE_RST_0_COD_MASK                                             (0x00080000UL)
#define EE_DYN_EE_RST_0_COD_POS                                              (19UL)
#define EE_DYN_CRC_CLEAR_0_COD_MASK                                          (0x00040000UL)
#define EE_DYN_CRC_CLEAR_0_COD_POS                                           (18UL)
#define EE_DYN_FULL_DUMP_READ_0_COD_MASK                                     (0x00020000UL)
#define EE_DYN_FULL_DUMP_READ_0_COD_POS                                      (17UL)
#define EE_DYN_PROG_0_COD_MASK                                               (0x00010000UL)
#define EE_DYN_PROG_0_COD_POS                                                (16UL)
#define EE_DYN_SWD_LOCK_ENABLE_MASK                                          (0x00000010UL)
#define EE_DYN_SWD_LOCK_ENABLE_POS                                           (4UL)
#define EE_DYN_EE_RST_DAT_MASK                                               (0x00000008UL)
#define EE_DYN_EE_RST_DAT_POS                                                (3UL)
#define EE_DYN_CRC_CLEAR_DAT_MASK                                            (0x00000004UL)
#define EE_DYN_CRC_CLEAR_DAT_POS                                             (2UL)
#define EE_DYN_FULL_DUMP_READ_DAT_MASK                                       (0x00000002UL)
#define EE_DYN_FULL_DUMP_READ_DAT_POS                                        (1UL)
#define EE_DYN_PROG_DAT_MASK                                                 (0x00000001UL)
#define EE_DYN_PROG_DAT_POS                                                  (0UL)
#define EE_DYN__RESET_VALUE                                                  (0x00000000UL)
/**
* @}
*/


/**
* \name EECTRL Status Register (RO)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_STAT_DAT                                                          (0x00200008UL)
#define EE_STAT_DAT_SWD_LOCK_MASK                                            (0x01000000UL)
#define EE_STAT_DAT_SWD_LOCK_POS                                             (24UL)
#define EE_STAT_DAT_DFT_LOCK_MASK                                            (0x00800000UL)
#define EE_STAT_DAT_DFT_LOCK_POS                                             (23UL)
#define EE_STAT_DAT_ALL1_DAT_MASK                                            (0x00400000UL)
#define EE_STAT_DAT_ALL1_DAT_POS                                             (22UL)
#define EE_STAT_DAT_ALL0_DAT_MASK                                            (0x00200000UL)
#define EE_STAT_DAT_ALL0_DAT_POS                                             (21UL)
#define EE_STAT_DAT_BNWDROP_DAT_MASK                                         (0x00100000UL)
#define EE_STAT_DAT_BNWDROP_DAT_POS                                          (20UL)
#define EE_STAT_DAT_TMANALOG_DAT_MASK                                        (0x00080000UL)
#define EE_STAT_DAT_TMANALOG_DAT_POS                                         (19UL)
#define EE_STAT_DAT_EE_EDO_DAT_MASK                                          (0x0007FFF8UL)
#define EE_STAT_DAT_EE_EDO_DAT_POS                                           (3UL)
#define EE_STAT_DAT_READOUT_ONGOING_DAT_MASK                                 (0x00000004UL)
#define EE_STAT_DAT_READOUT_ONGOING_DAT_POS                                  (2UL)
#define EE_STAT_DAT_PROG_DAT_MASK                                            (0x00000002UL)
#define EE_STAT_DAT_PROG_DAT_POS                                             (1UL)
#define EE_STAT_DAT_HVERR_DAT_MASK                                           (0x00000001UL)
#define EE_STAT_DAT_HVERR_DAT_POS                                            (0UL)
#define EE_STAT_DAT__RESET_VALUE                                             (0x00000000UL)
/**
* @}
*/


/**
* \name EECTRL Status Register (RO)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_STAT_COD                                                          (0x0020000CUL)
#define EE_STAT_COD_ECC_READ_INVALID_1_COD_MASK                              (0x02000000UL)
#define EE_STAT_COD_ECC_READ_INVALID_1_COD_POS                               (25UL)
#define EE_STAT_COD_ECC_READ_CORRECT_1_COD_MASK                              (0x01000000UL)
#define EE_STAT_COD_ECC_READ_CORRECT_1_COD_POS                               (24UL)
#define EE_STAT_COD_EE_EDO_1_COD_MASK                                        (0x00FC0000UL)
#define EE_STAT_COD_EE_EDO_1_COD_POS                                         (18UL)
#define EE_STAT_COD_READOUT_ONGOING_1_COD_MASK                               (0x00020000UL)
#define EE_STAT_COD_READOUT_ONGOING_1_COD_POS                                (17UL)
#define EE_STAT_COD_DROPSENS_1_COD_MASK                                      (0x00010000UL)
#define EE_STAT_COD_DROPSENS_1_COD_POS                                       (16UL)
#define EE_STAT_COD_VMPOK_1_COD_MASK                                         (0x00008000UL)
#define EE_STAT_COD_VMPOK_1_COD_POS                                          (15UL)
#define EE_STAT_COD_PROG_1_COD_MASK                                          (0x00004000UL)
#define EE_STAT_COD_PROG_1_COD_POS                                           (14UL)
#define EE_STAT_COD_HVERR_1_COD_MASK                                         (0x00002000UL)
#define EE_STAT_COD_HVERR_1_COD_POS                                          (13UL)
#define EE_STAT_COD_ECC_READ_INVALID_0_COD_MASK                              (0x00001000UL)
#define EE_STAT_COD_ECC_READ_INVALID_0_COD_POS                               (12UL)
#define EE_STAT_COD_ECC_READ_CORRECT_0_COD_MASK                              (0x00000800UL)
#define EE_STAT_COD_ECC_READ_CORRECT_0_COD_POS                               (11UL)
#define EE_STAT_COD_EE_EDO_0_COD_MASK                                        (0x000007E0UL)
#define EE_STAT_COD_EE_EDO_0_COD_POS                                         (5UL)
#define EE_STAT_COD_READOUT_ONGOING_0_COD_MASK                               (0x00000010UL)
#define EE_STAT_COD_READOUT_ONGOING_0_COD_POS                                (4UL)
#define EE_STAT_COD_DROPSENS_0_COD_MASK                                      (0x00000008UL)
#define EE_STAT_COD_DROPSENS_0_COD_POS                                       (3UL)
#define EE_STAT_COD_VMPOK_0_COD_MASK                                         (0x00000004UL)
#define EE_STAT_COD_VMPOK_0_COD_POS                                          (2UL)
#define EE_STAT_COD_PROG_0_COD_MASK                                          (0x00000002UL)
#define EE_STAT_COD_PROG_0_COD_POS                                           (1UL)
#define EE_STAT_COD_HVERR_0_COD_MASK                                         (0x00000001UL)
#define EE_STAT_COD_HVERR_0_COD_POS                                          (0UL)
#define EE_STAT_COD__RESET_VALUE                                             (0x00000000UL)
/**
* @}
*/


/**
* \name Data EEPROM CRC Register (RO)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> r--
* @{
*/
#define EE_CRC_DAT                                                           (0x00200010UL)
#define EE_CRC_DAT_EE_CRC_DAT_MASK                                           (0xFFFFFFFFUL)
#define EE_CRC_DAT_EE_CRC_DAT_POS                                            (0UL)
#define EE_CRC_DAT__RESET_VALUE                                              (0xFFFFFFFFUL)
/**
* @}
*/


/**
* \name Data EEPROM CRC Calculation Stat/End Addresses Register (RW)
* <b>Reset value:</b> 0x07FF0000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_DAT_ADDR                                                      (0x00200014UL)
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_END_MASK                             (0x0FFF0000UL)
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_END_POS                              (16UL)
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_START_MASK                           (0x00000FFFUL)
#define EE_CRC_DAT_ADDR_EE_CRC_DAT_ADDR_START_POS                            (0UL)
#define EE_CRC_DAT_ADDR__RESET_VALUE                                         (0x07FF0000UL)
/**
* @}
*/


/**
* \name PAGEFLASH_1 CRC Init Value Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_1_COD_INIT                                                    (0x00200018UL)
#define EE_CRC_1_COD_INIT_EE_CRC_1_COD_INIT_MASK                             (0xFFFFFFFFUL)
#define EE_CRC_1_COD_INIT_EE_CRC_1_COD_INIT_POS                              (0UL)
#define EE_CRC_1_COD_INIT__RESET_VALUE                                       (0xFFFFFFFFUL)
/**
* @}
*/


/**
* \name PAGEFLASH_1 CRC Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> r--
* @{
*/
#define EE_CRC_1_COD                                                         (0x0020001CUL)
#define EE_CRC_1_COD_EE_CRC_1_COD_MASK                                       (0xFFFFFFFFUL)
#define EE_CRC_1_COD_EE_CRC_1_COD_POS                                        (0UL)
#define EE_CRC_1_COD__RESET_VALUE                                            (0xFFFFFFFFUL)
/**
* @}
*/


/**
* \name PAGEFLASH_1 CRC Calculation Stat/End Addresses Register (RW)
* <b>Reset value:</b> 0x4FFF0000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_1_COD_ADDR                                                    (0x00200020UL)
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_END_MASK                         (0xFFFF0000UL)
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_END_POS                          (16UL)
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_START_MASK                       (0x0000FFFFUL)
#define EE_CRC_1_COD_ADDR_EE_CRC_1_COD_ADDR_START_POS                        (0UL)
#define EE_CRC_1_COD_ADDR__RESET_VALUE                                       (0x4FFF0000UL)
/**
* @}
*/


/**
* \name PAGEFLASH_0 CRC Init Value Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_0_COD_INIT                                                    (0x00200024UL)
#define EE_CRC_0_COD_INIT_EE_CRC_0_COD_INIT_MASK                             (0xFFFFFFFFUL)
#define EE_CRC_0_COD_INIT_EE_CRC_0_COD_INIT_POS                              (0UL)
#define EE_CRC_0_COD_INIT__RESET_VALUE                                       (0xFFFFFFFFUL)
/**
* @}
*/


/**
* \name PAGEFLASH_0 CRC Register (RW)
* <b>Reset value:</b> 0xFFFFFFFF\n
* <b>Access:</b> r--
* @{
*/
#define EE_CRC_0_COD                                                         (0x00200028UL)
#define EE_CRC_0_COD_EE_CRC_0_COD_MASK                                       (0xFFFFFFFFUL)
#define EE_CRC_0_COD_EE_CRC_0_COD_POS                                        (0UL)
#define EE_CRC_0_COD__RESET_VALUE                                            (0xFFFFFFFFUL)
/**
* @}
*/


/**
* \name PAGEFLASH_0 CRC Calculation Stat/End Addresses Register (RW)
* <b>Reset value:</b> 0x4FFF0000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_CRC_0_COD_ADDR                                                    (0x0020002CUL)
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_END_MASK                         (0xFFFF0000UL)
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_END_POS                          (16UL)
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_START_MASK                       (0x0000FFFFUL)
#define EE_CRC_0_COD_ADDR_EE_CRC_0_COD_ADDR_START_POS                        (0UL)
#define EE_CRC_0_COD_ADDR__RESET_VALUE                                       (0x4FFF0000UL)
/**
* @}
*/


/**
* \name EEPROM Test Support Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_TST_DAT                                                           (0x00200030UL)
#define EE_TST_DAT_CRC_CALC_WITH_ECC_MASK                                    (0x04000000UL)
#define EE_TST_DAT_CRC_CALC_WITH_ECC_POS                                     (26UL)
#define EE_TST_DAT_EE_DISECCINV_MASK                                         (0x02000000UL)
#define EE_TST_DAT_EE_DISECCINV_POS                                          (25UL)
#define EE_TST_DAT_EE_ECCTRAN_MASK                                           (0x01000000UL)
#define EE_TST_DAT_EE_ECCTRAN_POS                                            (24UL)
#define EE_TST_DAT_EE_EDI_MASK                                               (0x00FFFF00UL)
#define EE_TST_DAT_EE_EDI_POS                                                (8UL)
#define EE_TST_DAT_EE_CTRL_MASK                                              (0x000000FFUL)
#define EE_TST_DAT_EE_CTRL_POS                                               (0UL)
#define EE_TST_DAT__RESET_VALUE                                              (0x00000000UL)
/**
* @}
*/


/**
* \name PAGEFLASH_1 Test Support Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_TST_1_COD                                                         (0x00200034UL)
#define EE_TST_1_COD_CRC_CALC_WITH_ECC_MASK                                  (0x00010000UL)
#define EE_TST_1_COD_CRC_CALC_WITH_ECC_POS                                   (16UL)
#define EE_TST_1_COD_EE_DISECCINV_MASK                                       (0x00008000UL)
#define EE_TST_1_COD_EE_DISECCINV_POS                                        (15UL)
#define EE_TST_1_COD_EE_ECCTRAN_MASK                                         (0x00004000UL)
#define EE_TST_1_COD_EE_ECCTRAN_POS                                          (14UL)
#define EE_TST_1_COD_EE_EDI_MASK                                             (0x00003F00UL)
#define EE_TST_1_COD_EE_EDI_POS                                              (8UL)
#define EE_TST_1_COD_EE_CTRL_MASK                                            (0x000000FFUL)
#define EE_TST_1_COD_EE_CTRL_POS                                             (0UL)
#define EE_TST_1_COD__RESET_VALUE                                            (0x00000000UL)
/**
* @}
*/


/**
* \name PAGEFLASH_0 Test Support Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_TST_0_COD                                                         (0x00200038UL)
#define EE_TST_0_COD_CRC_CALC_WITH_ECC_MASK                                  (0x00010000UL)
#define EE_TST_0_COD_CRC_CALC_WITH_ECC_POS                                   (16UL)
#define EE_TST_0_COD_EE_DISECCINV_MASK                                       (0x00008000UL)
#define EE_TST_0_COD_EE_DISECCINV_POS                                        (15UL)
#define EE_TST_0_COD_EE_ECCTRAN_MASK                                         (0x00004000UL)
#define EE_TST_0_COD_EE_ECCTRAN_POS                                          (14UL)
#define EE_TST_0_COD_EE_EDI_MASK                                             (0x00003F00UL)
#define EE_TST_0_COD_EE_EDI_POS                                              (8UL)
#define EE_TST_0_COD_EE_CTRL_MASK                                            (0x000000FFUL)
#define EE_TST_0_COD_EE_CTRL_POS                                             (0UL)
#define EE_TST_0_COD__RESET_VALUE                                            (0x00000000UL)
/**
* @}
*/


/**
* \name Memories Trimming Values Register (RW)
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rwm
* @{
*/
#define EE_TRIMM                                                             (0x0020003CUL)
#define EE_TRIMM_HVTRIMW_1_COD_MASK                                          (0x00F00000UL)
#define EE_TRIMM_HVTRIMW_1_COD_POS                                           (20UL)
#define EE_TRIMM_HVTRIME_1_COD_MASK                                          (0x000F0000UL)
#define EE_TRIMM_HVTRIME_1_COD_POS                                           (16UL)
#define EE_TRIMM_HVTRIMW_0_COD_MASK                                          (0x0000F000UL)
#define EE_TRIMM_HVTRIMW_0_COD_POS                                           (12UL)
#define EE_TRIMM_HVTRIME_0_COD_MASK                                          (0x00000F00UL)
#define EE_TRIMM_HVTRIME_0_COD_POS                                           (8UL)
#define EE_TRIMM_HVTRIMW_DAT_MASK                                            (0x000000F0UL)
#define EE_TRIMM_HVTRIMW_DAT_POS                                             (4UL)
#define EE_TRIMM_HVTRIME_DAT_MASK                                            (0x0000000FUL)
#define EE_TRIMM_HVTRIME_DAT_POS                                             (0UL)
#define EE_TRIMM__RESET_VALUE                                                (0x00000000UL)
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> rw-
* @{
*/
#define EE_ECC_PF_AHB_ERROR_ADDR                                             (0x00200044UL)
#define EE_ECC_PF_AHB_ERROR_ADDR_ECC_PF_AHB_ERROR_ADDR_MASK                  (0x0003FFFFUL)
#define EE_ECC_PF_AHB_ERROR_ADDR_ECC_PF_AHB_ERROR_ADDR_POS                   (0UL)
#define EE_ECC_PF_AHB_ERROR_ADDR__RESET_VALUE                                (0x00000000UL)
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_CLR_ENABLE                                                    (0x00200FD8UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_ENABLE_MASK  (0x00000200UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_ENABLE_POS   (9UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_CLR_ENABLE_MASK      (0x00000100UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_CLR_ENABLE_POS       (8UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_ENABLE_MASK  (0x00000080UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_ENABLE_POS   (7UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_CLR_ENABLE_MASK      (0x00000040UL)
#define EE_INT_CLR_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_CLR_ENABLE_POS       (6UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_1_COD_INT_CLR_ENABLE_MASK                 (0x00000020UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_1_COD_INT_CLR_ENABLE_POS                  (5UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_0_COD_INT_CLR_ENABLE_MASK                 (0x00000010UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_0_COD_INT_CLR_ENABLE_POS                  (4UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_DAT_INT_CLR_ENABLE_MASK                   (0x00000008UL)
#define EE_INT_CLR_ENABLE_EE_HVERR_DAT_INT_CLR_ENABLE_POS                    (3UL)
#define EE_INT_CLR_ENABLE_EE_PROG_1_COD_COMPLETED_INT_CLR_ENABLE_MASK        (0x00000004UL)
#define EE_INT_CLR_ENABLE_EE_PROG_1_COD_COMPLETED_INT_CLR_ENABLE_POS         (2UL)
#define EE_INT_CLR_ENABLE_EE_PROG_0_COD_COMPLETED_INT_CLR_ENABLE_MASK        (0x00000002UL)
#define EE_INT_CLR_ENABLE_EE_PROG_0_COD_COMPLETED_INT_CLR_ENABLE_POS         (1UL)
#define EE_INT_CLR_ENABLE_EE_PROG_DAT_COMPLETED_INT_CLR_ENABLE_MASK          (0x00000001UL)
#define EE_INT_CLR_ENABLE_EE_PROG_DAT_COMPLETED_INT_CLR_ENABLE_POS           (0UL)
#define EE_INT_CLR_ENABLE__RESET_VALUE                                       (0x00000000UL)
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_SET_ENABLE                                                    (0x00200FDCUL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_ENABLE_MASK  (0x00000200UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_ENABLE_POS   (9UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_SET_ENABLE_MASK      (0x00000100UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_SET_ENABLE_POS       (8UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_ENABLE_MASK  (0x00000080UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_ENABLE_POS   (7UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_SET_ENABLE_MASK      (0x00000040UL)
#define EE_INT_SET_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_SET_ENABLE_POS       (6UL)
#define EE_INT_SET_ENABLE_EE_HVERR_1_COD_INT_SET_ENABLE_MASK                 (0x00000020UL)
#define EE_INT_SET_ENABLE_EE_HVERR_1_COD_INT_SET_ENABLE_POS                  (5UL)
#define EE_INT_SET_ENABLE_EE_HVERR_0_COD_INT_SET_ENABLE_MASK                 (0x00000010UL)
#define EE_INT_SET_ENABLE_EE_HVERR_0_COD_INT_SET_ENABLE_POS                  (4UL)
#define EE_INT_SET_ENABLE_EE_HVERR_DAT_INT_SET_ENABLE_MASK                   (0x00000008UL)
#define EE_INT_SET_ENABLE_EE_HVERR_DAT_INT_SET_ENABLE_POS                    (3UL)
#define EE_INT_SET_ENABLE_EE_PROG_1_COD_COMPLETED_INT_SET_ENABLE_MASK        (0x00000004UL)
#define EE_INT_SET_ENABLE_EE_PROG_1_COD_COMPLETED_INT_SET_ENABLE_POS         (2UL)
#define EE_INT_SET_ENABLE_EE_PROG_0_COD_COMPLETED_INT_SET_ENABLE_MASK        (0x00000002UL)
#define EE_INT_SET_ENABLE_EE_PROG_0_COD_COMPLETED_INT_SET_ENABLE_POS         (1UL)
#define EE_INT_SET_ENABLE_EE_PROG_DAT_COMPLETED_INT_SET_ENABLE_MASK          (0x00000001UL)
#define EE_INT_SET_ENABLE_EE_PROG_DAT_COMPLETED_INT_SET_ENABLE_POS           (0UL)
#define EE_INT_SET_ENABLE__RESET_VALUE                                       (0x00000000UL)
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_INT_STATUS                                                        (0x00200FE0UL)
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_STATUS_MASK          (0x00000200UL)
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_STATUS_POS           (9UL)
#define EE_INT_STATUS_EE_ECC_READ_INVALID_1_COD_INT_STATUS_MASK              (0x00000100UL)
#define EE_INT_STATUS_EE_ECC_READ_INVALID_1_COD_INT_STATUS_POS               (8UL)
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_STATUS_MASK          (0x00000080UL)
#define EE_INT_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_STATUS_POS           (7UL)
#define EE_INT_STATUS_EE_ECC_READ_INVALID_0_COD_INT_STATUS_MASK              (0x00000040UL)
#define EE_INT_STATUS_EE_ECC_READ_INVALID_0_COD_INT_STATUS_POS               (6UL)
#define EE_INT_STATUS_EE_HVERR_1_COD_INT_STATUS_MASK                         (0x00000020UL)
#define EE_INT_STATUS_EE_HVERR_1_COD_INT_STATUS_POS                          (5UL)
#define EE_INT_STATUS_EE_HVERR_0_COD_INT_STATUS_MASK                         (0x00000010UL)
#define EE_INT_STATUS_EE_HVERR_0_COD_INT_STATUS_POS                          (4UL)
#define EE_INT_STATUS_EE_HVERR_DAT_INT_STATUS_MASK                           (0x00000008UL)
#define EE_INT_STATUS_EE_HVERR_DAT_INT_STATUS_POS                            (3UL)
#define EE_INT_STATUS_EE_PROG_1_COD_COMPLETED_INT_STATUS_MASK                (0x00000004UL)
#define EE_INT_STATUS_EE_PROG_1_COD_COMPLETED_INT_STATUS_POS                 (2UL)
#define EE_INT_STATUS_EE_PROG_0_COD_COMPLETED_INT_STATUS_MASK                (0x00000002UL)
#define EE_INT_STATUS_EE_PROG_0_COD_COMPLETED_INT_STATUS_POS                 (1UL)
#define EE_INT_STATUS_EE_PROG_DAT_COMPLETED_INT_STATUS_MASK                  (0x00000001UL)
#define EE_INT_STATUS_EE_PROG_DAT_COMPLETED_INT_STATUS_POS                   (0UL)
#define EE_INT_STATUS__RESET_VALUE                                           (0x00000000UL)
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> r--
* @{
*/
#define EE_INT_ENABLE                                                        (0x00200FE4UL)
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_ENABLE_MASK          (0x00000200UL)
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_1_COD_INT_ENABLE_POS           (9UL)
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_ENABLE_MASK              (0x00000100UL)
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_1_COD_INT_ENABLE_POS               (8UL)
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_ENABLE_MASK          (0x00000080UL)
#define EE_INT_ENABLE_EE_ECC_READ_NOT_CORRECT_0_COD_INT_ENABLE_POS           (7UL)
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_ENABLE_MASK              (0x00000040UL)
#define EE_INT_ENABLE_EE_ECC_READ_INVALID_0_COD_INT_ENABLE_POS               (6UL)
#define EE_INT_ENABLE_EE_HVERR_1_COD_INT_ENABLE_MASK                         (0x00000020UL)
#define EE_INT_ENABLE_EE_HVERR_1_COD_INT_ENABLE_POS                          (5UL)
#define EE_INT_ENABLE_EE_HVERR_0_COD_INT_ENABLE_MASK                         (0x00000010UL)
#define EE_INT_ENABLE_EE_HVERR_0_COD_INT_ENABLE_POS                          (4UL)
#define EE_INT_ENABLE_EE_HVERR_DAT_INT_ENABLE_MASK                           (0x00000008UL)
#define EE_INT_ENABLE_EE_HVERR_DAT_INT_ENABLE_POS                            (3UL)
#define EE_INT_ENABLE_EE_PROG_1_COD_COMPLETED_INT_ENABLE_MASK                (0x00000004UL)
#define EE_INT_ENABLE_EE_PROG_1_COD_COMPLETED_INT_ENABLE_POS                 (2UL)
#define EE_INT_ENABLE_EE_PROG_0_COD_COMPLETED_INT_ENABLE_MASK                (0x00000002UL)
#define EE_INT_ENABLE_EE_PROG_0_COD_COMPLETED_INT_ENABLE_POS                 (1UL)
#define EE_INT_ENABLE_EE_PROG_DAT_COMPLETED_INT_ENABLE_MASK                  (0x00000001UL)
#define EE_INT_ENABLE_EE_PROG_DAT_COMPLETED_INT_ENABLE_POS                   (0UL)
#define EE_INT_ENABLE__RESET_VALUE                                           (0x00000000UL)
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_CLR_STATUS                                                    (0x00200FE8UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_STATUS_MASK  (0x00000200UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_CLR_STATUS_POS   (9UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_1_COD_INT_CLR_STATUS_MASK      (0x00000100UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_1_COD_INT_CLR_STATUS_POS       (8UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_STATUS_MASK  (0x00000080UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_CLR_STATUS_POS   (7UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_0_COD_INT_CLR_STATUS_MASK      (0x00000040UL)
#define EE_INT_CLR_STATUS_EE_ECC_READ_INVALID_0_COD_INT_CLR_STATUS_POS       (6UL)
#define EE_INT_CLR_STATUS_EE_HVERR_1_COD_INT_CLR_STATUS_MASK                 (0x00000020UL)
#define EE_INT_CLR_STATUS_EE_HVERR_1_COD_INT_CLR_STATUS_POS                  (5UL)
#define EE_INT_CLR_STATUS_EE_HVERR_0_COD_INT_CLR_STATUS_MASK                 (0x00000010UL)
#define EE_INT_CLR_STATUS_EE_HVERR_0_COD_INT_CLR_STATUS_POS                  (4UL)
#define EE_INT_CLR_STATUS_EE_HVERR_DAT_INT_CLR_STATUS_MASK                   (0x00000008UL)
#define EE_INT_CLR_STATUS_EE_HVERR_DAT_INT_CLR_STATUS_POS                    (3UL)
#define EE_INT_CLR_STATUS_EE_PROG_1_COD_COMPLETED_INT_CLR_STATUS_MASK        (0x00000004UL)
#define EE_INT_CLR_STATUS_EE_PROG_1_COD_COMPLETED_INT_CLR_STATUS_POS         (2UL)
#define EE_INT_CLR_STATUS_EE_PROG_0_COD_COMPLETED_INT_CLR_STATUS_MASK        (0x00000002UL)
#define EE_INT_CLR_STATUS_EE_PROG_0_COD_COMPLETED_INT_CLR_STATUS_POS         (1UL)
#define EE_INT_CLR_STATUS_EE_PROG_DAT_COMPLETED_INT_CLR_STATUS_MASK          (0x00000001UL)
#define EE_INT_CLR_STATUS_EE_PROG_DAT_COMPLETED_INT_CLR_STATUS_POS           (0UL)
#define EE_INT_CLR_STATUS__RESET_VALUE                                       (0x00000000UL)
/**
* @}
*/


/**
* \name PFLASH ECC Error Address
* <b>Reset value:</b> 0x00000000\n
* <b>Access:</b> -w-
* @{
*/
#define EE_INT_SET_STATUS                                                    (0x00200FECUL)
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_STATUS_MASK  (0x00000200UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_1_COD_INT_SET_STATUS_POS   (9UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_1_COD_INT_SET_STATUS_MASK      (0x00000100UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_1_COD_INT_SET_STATUS_POS       (8UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_STATUS_MASK  (0x00000080UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_NOT_CORRECT_0_COD_INT_SET_STATUS_POS   (7UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_0_COD_INT_SET_STATUS_MASK      (0x00000040UL)
#define EE_INT_SET_STATUS_EE_ECC_READ_INVALID_0_COD_INT_SET_STATUS_POS       (6UL)
#define EE_INT_SET_STATUS_EE_HVERR_1_COD_INT_SET_STATUS_MASK                 (0x00000020UL)
#define EE_INT_SET_STATUS_EE_HVERR_1_COD_INT_SET_STATUS_POS                  (5UL)
#define EE_INT_SET_STATUS_EE_HVERR_0_COD_INT_SET_STATUS_MASK                 (0x00000010UL)
#define EE_INT_SET_STATUS_EE_HVERR_0_COD_INT_SET_STATUS_POS                  (4UL)
#define EE_INT_SET_STATUS_EE_HVERR_DAT_INT_SET_STATUS_MASK                   (0x00000008UL)
#define EE_INT_SET_STATUS_EE_HVERR_DAT_INT_SET_STATUS_POS                    (3UL)
#define EE_INT_SET_STATUS_EE_PROG_1_COD_COMPLETED_INT_SET_STATUS_MASK        (0x00000004UL)
#define EE_INT_SET_STATUS_EE_PROG_1_COD_COMPLETED_INT_SET_STATUS_POS         (2UL)
#define EE_INT_SET_STATUS_EE_PROG_0_COD_COMPLETED_INT_SET_STATUS_MASK        (0x00000002UL)
#define EE_INT_SET_STATUS_EE_PROG_0_COD_COMPLETED_INT_SET_STATUS_POS         (1UL)
#define EE_INT_SET_STATUS_EE_PROG_DAT_COMPLETED_INT_SET_STATUS_MASK          (0x00000001UL)
#define EE_INT_SET_STATUS_EE_PROG_DAT_COMPLETED_INT_SET_STATUS_POS           (0UL)
#define EE_INT_SET_STATUS__RESET_VALUE                                       (0x00000000UL)



/* static void pn7462_flash_hw_reset(struct target *target)
 * {
 *	target_write_u32(target, EE_DYN, EE_DYN_EE_RST_0_COD_POS);
 *	target_write_u32(target, EE_DYN, EE_DYN_EE_RST_1_COD_POS);
 * } */

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
			(unsigned long) bank->base + offset);
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
			(unsigned long) bank->base + offset);
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
		(unsigned long) bank->base + offset,
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

	/* if first and last equals 0, erase eeprom
	 * if first and last equals 1, erase flash */

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

	/* create one sector for the flash memory and eeprom */
	bank->sectors = malloc(sizeof(struct flash_sector));

	if (bank->sectors == NULL)
		return ERROR_FAIL;

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
