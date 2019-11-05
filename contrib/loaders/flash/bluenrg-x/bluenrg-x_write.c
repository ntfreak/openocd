/* To be built with arm-none-eabi-gcc -c -mthumb -mcpu=cortex-m0 -O3 bluenrgx.c */
/* Then postprocess output of command "arm-none-eabi-objdump -d bluenrgx.o" to make a C array of bytes */

#include <stdint.h>

/* Status Values ----------------------------------------------------------*/
#define SUCCESS             0
#define ERR_UNALIGNED       1
#define ERR_INVALID_ADDRESS 2
#define ERR_INVALID_TYPE    3
#define ERR_WRITE_PROTECTED 4
#define ERR_WRITE_FAILED    5
#define ERR_ERASE_REQUIRED  6
#define ERR_VERIFY_FAILED   7

/* Flash Controller defines ---------------------------------------------------*/
#ifdef BLUENRG_LP
#define FLASH_REG_COMMAND ((volatile uint32_t *)0x40001000)
#define FLASH_REG_CONFIG  ((volatile uint32_t *)0x40001004)
#define FLASH_REG_IRQSTAT ((volatile uint32_t *)0x40001008)
#define FLASH_REG_IRQMASK ((volatile uint32_t *)0x4000100C)
#define FLASH_REG_IRQRAW  ((volatile uint32_t *)0x40001010)
#define FLASH_REG_ADDRESS ((volatile uint32_t *)0x40001018)
#define FLASH_REG_UNLOCKM ((volatile uint32_t *)0x4000101C)
#define FLASH_REG_UNLOCKL ((volatile uint32_t *)0x40001020)
#define FLASH_REG_DATA0   ((volatile uint32_t *)0x40001040)
#define FLASH_REG_DATA1   ((volatile uint32_t *)0x40001044)
#define FLASH_REG_DATA2   ((volatile uint32_t *)0x40001048)
#define FLASH_REG_DATA3   ((volatile uint32_t *)0x4000104C)
#define FLASH_SIZE_REG 0x40001014
#else
#define FLASH_REG_COMMAND ((volatile uint32_t *)0x40100000)
#define FLASH_REG_CONFIG  ((volatile uint32_t *)0x40100004)
#define FLASH_REG_IRQSTAT ((volatile uint32_t *)0x40100008)
#define FLASH_REG_IRQMASK ((volatile uint32_t *)0x4010000C)
#define FLASH_REG_IRQRAW  ((volatile uint32_t *)0x40100010)
#define FLASH_REG_ADDRESS ((volatile uint32_t *)0x40100018)
#define FLASH_REG_UNLOCKM ((volatile uint32_t *)0x4010001C)
#define FLASH_REG_UNLOCKL ((volatile uint32_t *)0x40100020)
#define FLASH_REG_DATA0   ((volatile uint32_t *)0x40100040)
#define FLASH_REG_DATA1   ((volatile uint32_t *)0x40100044)
#define FLASH_REG_DATA2   ((volatile uint32_t *)0x40100048)
#define FLASH_REG_DATA3   ((volatile uint32_t *)0x4010004C)
#define FLASH_SIZE_REG 0x40100014
#endif

#define MFB_MASS_ERASE 0x01
#define MFB_PAGE_ERASE 0x02

#define DO_ERASE  0x0100
#define DO_VERIFY 0x0200
#define FLASH_CMD_ERASE_PAGE 0x11
#define FLASH_CMD_MASSERASE  0x22
#define FLASH_CMD_WRITE      0x33
#define FLASH_CMD_BURSTWRITE 0xCC
#define FLASH_INT_CMDDONE    0x01
#define MFB_BOTTOM          (0x10040000)
#define MFB_SIZE_B          ((16 * (((*(uint32_t *) FLASH_SIZE_REG) + 1) >> 12)) * 1024)
#define MFB_SIZE_W          (MFB_SIZE_B/4)
#define MFB_TOP             (MFB_BOTTOM+MFB_SIZE_B-1)
#define MFB_PAGE_SIZE_B     (2048)
#define MFB_PAGE_SIZE_W     (MFB_PAGE_SIZE_B/4)

#define AREA_ERROR 0x01
#define AREA_MFB   0x04

#define FLASH_WORD_LEN       4

typedef struct {
	volatile uint8_t *wp;
	uint8_t *rp;
} work_area_t;

/* Flash Commands --------------------------------------------------------*/
static inline __attribute__((always_inline)) uint32_t flashWrite(uint32_t address, uint8_t **data,
								 uint32_t writeLength)
{
	uint32_t index, flash_word[4];
	uint8_t i;

	*FLASH_REG_IRQMASK = 0;
	for (index = 0; index < writeLength; index += (FLASH_WORD_LEN*4)) {
		for (i = 0; i < 4; i++)
			flash_word[i] = (*(uint32_t *) (*data + i*4));

		/* Clear the IRQ flags */
		*FLASH_REG_IRQRAW = 0x0000003F;
		/* Load the flash address to write */
		*FLASH_REG_ADDRESS = (uint16_t)((address + index - MFB_BOTTOM) >> 2);
		/* Prepare and load the data to flash */
		*FLASH_REG_DATA0 = flash_word[0];
		*FLASH_REG_DATA1 = flash_word[1];
		*FLASH_REG_DATA2 = flash_word[2];
		*FLASH_REG_DATA3 = flash_word[3];
		/* Flash write command */
		*FLASH_REG_COMMAND = FLASH_CMD_BURSTWRITE;
		/* Wait the end of the flash write command */
		while ((*FLASH_REG_IRQRAW & FLASH_INT_CMDDONE) == 0)
			;
		*data += (FLASH_WORD_LEN * 4);
	}

	return SUCCESS;
}

__attribute__((naked)) __attribute__((noreturn)) void write(uint8_t *work_area_p,
							    uint8_t *fifo_end,
							    uint8_t *target_address,
							    uint32_t count)
{
	uint32_t retval;
	volatile work_area_t *work_area = (work_area_t *) work_area_p;
	uint8_t *fifo_start = (uint8_t *) work_area->rp;

	while (count) {
		volatile int32_t fifo_linear_size;

		/* Wait for some data in the FIFO */
		while (work_area->rp == work_area->wp)
			;
		if (work_area->wp == 0) {
			/* Aborted by other party */
			break;
		}
		if (work_area->rp > work_area->wp) {
			fifo_linear_size = fifo_end-work_area->rp;
		} else {
			fifo_linear_size = (work_area->wp - work_area->rp);
			if (fifo_linear_size < 0)
				fifo_linear_size = 0;
		}
		if (fifo_linear_size < 16) {
			/* We should never get here */
			continue;
		}

		retval = flashWrite((uint32_t) target_address, (uint8_t **) &work_area->rp, fifo_linear_size);
		if (retval != SUCCESS) {
			work_area->rp = (uint8_t *)retval;
			break;
		}
		target_address += fifo_linear_size;
		if (work_area->rp >= fifo_end)
			work_area->rp = fifo_start;
		count -= fifo_linear_size;
	}
	__asm("bkpt 0");
}
