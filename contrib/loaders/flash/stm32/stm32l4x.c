/* SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Copyright (C) 2021 Tarek BOCHKATI
 *   tarek.bouchkati@st.com
 */

#include <stdint.h>
#include "../../../../src/flash/nor/stm32l4x.h"

static inline __attribute__((always_inline)) void copy_buffer_u32(uint32_t *dst, uint32_t *src, int len)
{
	for (int i = 0; i < len; i++)
		dst[i] = src[i];
}

/* this function is assumes that fifo_size is multiple of flash_word_size
 * this condition is ensured by target_run_flash_async_algorithm
 *
 * IMPORTANT:
 * when updating this loader and re-compiling it:
 *   - check the generated listing 'stm32l4x.lst
 *   - get the highest offset used with sp:
 *     example: ldr	r3, [sp, #12]
 *     the highest offset is 12
 *   - change the value of LDR_STACK_SIZE in 'stm32l4x.h' at least to 'offset + 4'
 */

#ifndef DEBUG
__attribute__((naked))
#endif
void write(volatile struct stm32l4_work_area *work_area,
		   uint8_t *fifo_end,
		   uint8_t *target_address,
		   uint32_t count)
{
	uint8_t *rp_cache  = work_area->fifo.rp; /* optimization to avoid reading from memory each time */
	uint8_t *fifo_start = rp_cache; /* used to wrap when we reach fifo_end */

	/* enable flash programming */
	*(work_area->flash_cr) = FLASH_PG;

	while (count) {
		uint8_t *wp_cache  = work_area->fifo.wp; /* optimization to avoid reading from memory each time */
		if (wp_cache == 0)
			break; /* aborted by target_run_flash_async_algorithm */

		int32_t fifo_size = wp_cache - rp_cache;
		if (fifo_size < 0) {
			/* consider the linear fifo, we will wrap later */
			fifo_size = fifo_end - rp_cache;
		}

		/* wait for at least a flash word */
		while (fifo_size >= work_area->flash_word_size) { /* same as checking for non-zero size */
			copy_buffer_u32((uint32_t *)target_address, (uint32_t *)rp_cache, work_area->flash_word_size / 4);

			/* update target_address and rp_cache */
			target_address += work_area->flash_word_size;
			rp_cache += work_area->flash_word_size;

			/* wait for the busy flag */
			while (*(work_area->flash_sr) & work_area->flash_sr_bsy)
				;

			if (*(work_area->flash_sr) & FLASH_ERROR) {
				work_area->fifo.rp = 0; /* set rp to zero 0 on error */
				goto write_end;
			}

			/* wrap if reach the fifo_end, and update rp in memory */
			if (rp_cache >= fifo_end)
				rp_cache = fifo_start;

			/* flush the rp cache value,
			 * so target_run_flash_async_algorithm can fill the circular fifo */
			work_area->fifo.rp = rp_cache;

			/* update fifo_size and count */
			fifo_size -= work_area->flash_word_size;
			count--;
		}
	}

write_end:
	/* disable flash programming */
	*(work_area->flash_cr) = 0;

	/* soft break the loader */
	__asm("bkpt 0");
}

/* by enabling this define 'DEBUG':
 * the main() function can help help debugging the loader algo
 * note: the application should be linked into RAM */

/* #define DEBUG */

#ifdef DEBUG
/* device selector: STM32L5 | STM32U5 | STM32WB | STM32WL | STM32WL_CPU2 | STM32G0Bx | ... */
#define STM32U5

/* when using a secure device, and want to test the secure programming enable this define */
/* #define SECURE */

#if defined(STM32U5)
#  define FLASH_WORD_SIZE   16
#else
#  define FLASH_WORD_SIZE   8
#endif

#if defined(STM32WB) || defined(STM32WL)
#  define FLASH_BASE        0x58004000
#else
#  define FLASH_BASE        0x40022000
#endif

#if defined(STM32G0Bx)
#  define FLASH_BSY_MASK      (FLASH_BSY | FLASH_BSY2)
#else
#  define FLASH_BSY_MASK      FLASH_BSY
#endif

#if defined(STM32L5) || defined(STM32U5)
#  ifdef SECURE
#    define FLASH_KEYR_OFFSET 0x0c
#    define FLASH_SR_OFFSET   0x24
#    define FLASH_CR_OFFSET   0x2c
#  else
#    define FLASH_KEYR_OFFSET 0x08
#    define FLASH_SR_OFFSET   0x20
#    define FLASH_CR_OFFSET   0x28
#  endif
#elif defined(STM32WL_CPU2)
#  define FLASH_KEYR_OFFSET 0x08
#  define FLASH_SR_OFFSET   0x60
#  define FLASH_CR_OFFSET   0x64
#else
#  define FLASH_KEYR_OFFSET 0x08
#  define FLASH_SR_OFFSET   0x10
#  define FLASH_CR_OFFSET   0x14
#endif

#define FLASH_KEYR        (uint32_t *)((FLASH_BASE) + (FLASH_KEYR_OFFSET))
#define FLASH_SR          (uint32_t *)((FLASH_BASE) + (FLASH_SR_OFFSET))
#define FLASH_CR          (uint32_t *)((FLASH_BASE) + (FLASH_CR_OFFSET))

int main()
{
	const uint32_t count = 2;
	const uint32_t buf_size = count * FLASH_WORD_SIZE;
	const uint32_t work_area_size = sizeof(struct stm32l4_work_area) + buf_size;

	uint8_t work_area_buf[work_area_size];
	struct stm32l4_work_area *workarea = (struct stm32l4_work_area *)work_area_buf;

	/* fill the workarea struct */
	workarea->flash_sr = FLASH_SR;
	workarea->flash_cr = FLASH_CR;
	workarea->flash_word_size = FLASH_WORD_SIZE;
	workarea->flash_sr_bsy = FLASH_BSY_MASK;
	/* note: the workarea->stack is not used, in this configuration */

	/* programming the existing memory raw content in workarea->fifo.buf */
	/* feel free to fill the memory with magical values ... */

	workarea->fifo.wp =  (uint8_t *)(&workarea->fifo.buf + buf_size);
	workarea->fifo.rp =  (uint8_t *)&workarea->fifo.buf;

	/* unlock the flash */
	*FLASH_KEYR = KEY1;
	*FLASH_KEYR = KEY2;

	/* erase sector 0 */
	*FLASH_CR = FLASH_PER | FLASH_STRT;
	while (*FLASH_SR & FLASH_BSY)
		;

	/* flash address, should be aligned to FLASH_WORD_SIZE */
	uint8_t *target_address = (uint8_t *) 0x8000000;

	write(workarea,
		  (uint8_t *)(workarea + work_area_size),
		  target_address,
		  count);

	while (1)
		;
}
#endif /* DEBUG */
