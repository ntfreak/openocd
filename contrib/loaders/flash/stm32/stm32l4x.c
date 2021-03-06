/* SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Copyright (C) 2021 Tarek BOCHKATI
 *   tarek.bouchkati@st.com
 */

#include <stdint.h>
#include "../../../../src/flash/nor/stm32l4x.h"

struct work_area {
	uint8_t *wp;
	uint8_t *rp;
};

static inline __attribute__((always_inline)) void copy_buffer_u32(uint32_t *dst, uint32_t *src, int len)
{
	for (int i = 0; i < len; i++)
		dst[i] = src[i];
}


/* this function is assumes that fifo_size is multiple of flash_word_size
 * this condition is ensured by target_run_flash_async_algorithm
 *
 * IMPORTANT:
 * when updating this loader and re-compiling it, the stack arguments could be placed
 * at a different offset from sp register.
 * according the AAPCS, only the 4 32-bit first arguments are passed in the registers r0 to r4
 * the arguments starting from number 5 is passed in the stack.
 * to make it work, you need to:
 *   - check the generated listing 'stm32l4x.lst
 *   - get the highest offset used with sp:
 *     example: ldr	r3, [sp, #48]
 *   - change the size of 'write_algorithm_sp' working_area to this value + 4
 */

#ifndef DEBUG
__attribute__((naked))
#endif
void write(volatile struct work_area *work_area,
		   uint8_t *fifo_end,
		   uint8_t *target_address,
		   uint32_t count,
		   volatile uint32_t *flash_sr,
		   volatile uint32_t *flash_cr,
		   uint32_t flash_word_size)
{
	uint8_t *rp_cache  = work_area->rp; /* optimization to avoid reading from memory each time */
	uint8_t *fifo_start = rp_cache; /* used to wrap when we reach fifo_end */

	/* enable flash programming */
	*flash_cr = FLASH_PG;

	while (count) {
		uint8_t *wp_cache  = work_area->wp; /* optimization to avoid reading from memory each time */
		if (wp_cache == 0)
			break; /* aborted by target_run_flash_async_algorithm */

		int32_t fifo_size = wp_cache - rp_cache;
		if (fifo_size < 0) {
			/* consider the linear fifo, we will wrap later */
			fifo_size = fifo_end - rp_cache;
		}

		/* wait for at least a flash word */
		while (fifo_size >= flash_word_size) { /* same as checking for non-zero size */
			copy_buffer_u32((uint32_t *)target_address, (uint32_t *)rp_cache, flash_word_size / 4);

			/* update target_address and rp_cache */
			target_address += flash_word_size;
			rp_cache += flash_word_size;

			/* wait for the busy flag */
			while (*flash_sr & FLASH_BSY)
				;

			if (*flash_sr & FLASH_ERROR) {
				work_area->rp = 0; /* set rp to zero 0 on error */
				goto write_end;
			}

			/* wrap if reach the fifo_end, and update rp in memory */
			if (rp_cache >= fifo_end)
				rp_cache = fifo_start;

			/* flush the rp cache value,
			 * so target_run_flash_async_algorithm can fill the circular fifo */
			work_area->rp = rp_cache;

			/* update fifo_size and count */
			fifo_size -= flash_word_size;
			count--;
		}
	}

write_end:
	/* disable flash programming */
	*flash_cr = 0;

	/* soft break the loader */
	__asm("bkpt 0");
}

/* by enabling this define 'DEBUG':
 * the main() function can help help debugging the loader algo
 * note: the application should be linked into RAM */

/* #define DEBUG */

#ifdef DEBUG
#define STM32U5
/* #define SECURE */

#if defined(STM32U5)
#define FLASH_WORD_SIZE   16
#else
#define FLASH_WORD_SIZE   8
#endif

#if defined(STM32WB) || defined(STM32WL)
#define FLASH_BASE        0x58004000
#else
#define FLASH_BASE        0x40022000
#endif

#if defined(STM32L5) || defined(STM32U5)
#ifdef SECURE
#define FLASH_KEYR_OFFSET 0x0c
#define FLASH_SR_OFFSET   0x24
#define FLASH_CR_OFFSET   0x2c
#else
#define FLASH_KEYR_OFFSET 0x08
#define FLASH_SR_OFFSET   0x20
#define FLASH_CR_OFFSET   0x28
#endif
#elif defined(STM32WL_CPU2)
#define FLASH_KEYR_OFFSET 0x08
#define FLASH_SR_OFFSET   0x60
#define FLASH_CR_OFFSET   0x64
#else
#define FLASH_KEYR_OFFSET 0x08
#define FLASH_SR_OFFSET   0x10
#define FLASH_CR_OFFSET   0x14
#endif

#define FLASH_KEYR        (uint32_t *)((FLASH_BASE) + (FLASH_KEYR_OFFSET))
#define FLASH_SR          (uint32_t *)((FLASH_BASE) + (FLASH_SR_OFFSET))
#define FLASH_CR          (uint32_t *)((FLASH_BASE) + (FLASH_CR_OFFSET))

int main()
{
	const int count = 2;
	uint8_t *target_address = (uint8_t *) 0x8000000; /* flash address, should be aligned to FLASH_WORD_SIZE */

	uint32_t work_area[(8 + count * FLASH_WORD_SIZE) / 4]; /* wp, rp, and buffer */
	work_area[0] = (uint32_t) &work_area[2] + count * FLASH_WORD_SIZE; /* wp */
	work_area[1] = (uint32_t) &work_area[2]; /* rp */

	/* unlock the flash */
	*FLASH_KEYR = KEY1;
	*FLASH_KEYR = KEY2;

	/* erase sector 0 */
	*FLASH_CR = FLASH_PER | FLASH_STRT;
	while (*FLASH_SR & FLASH_BSY)
		;

	write((struct work_area *)work_area,
		  (uint8_t *)(work_area + 0x22),
		  target_address,
		  count,
		  FLASH_SR,
		  FLASH_CR,
		  FLASH_WORD_SIZE);

	while (1)
		;
}
#endif /* DEBUG */
