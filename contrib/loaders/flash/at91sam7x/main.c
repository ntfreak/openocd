/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/
#include "platform.h"

#include <flash/nor/ocl.h>
#include "dcc.h"
#include "samflash.h"


#define BUFSIZE 1024 /* words, i.e. 4 KiB */
uint32 buffer[1024];

void cmd_flash(uint32 cmd)
{
	unsigned int len;
	uint32 adr;
	uint32 chksum;
	unsigned int bi; /* buffer index */
	unsigned int bi_start; /* receive start mark */
	unsigned int bi_end; /* receive end mark */
	unsigned int ofs;
	int pagenum;
	int result;

	adr = dcc_rd();
	len = cmd&0xffff;
	ofs = adr%flash_page_size;
	bi_start = ofs/4;
	bi_end = (ofs + len + 3)/4;

	if (bi_end > BUFSIZE) {
		dcc_wr(OCL_BUFF_OVER);
		return;
	}

	chksum = OCL_CHKS_INIT;
	for (bi = 0; bi < bi_end; bi++) chksum^=buffer[bi]=dcc_rd();

	if (dcc_rd() != chksum) {
		dcc_wr(OCL_CHKS_FAIL);
		return;
	}

	/* fill in unused positions with unprogrammed values */
	for (bi = 0; bi < bi_start; bi++) buffer[bi]=0xffffffff;
	for (bi = bi_end; bi%flash_page_size; bi++) buffer[bi]=0xffffffff;

	result = 0;
	pagenum = adr/flash_page_size;
	for (bi = 0; bi < bi_end; bi += flash_page_size/4) {
		result = flash_page_program(buffer + bi, pagenum++);
		if (result) break;
	}

	/* verify written data */
	if (!result) result = flash_verify(adr, len, ((uint8 *)buffer) + ofs);

	dcc_wr(OCL_CMD_DONE | result);
}


int main (void)
{
	uint32 cmd;

	for (;;) {
		cmd = dcc_rd();
		switch (cmd&OCL_CMD_MASK) {
			case OCL_PROBE:
				dcc_wr(OCL_CMD_DONE | flash_init());
				dcc_wr(0x100000); /* base */
				dcc_wr(flash_page_count*flash_page_size); /* size */
				dcc_wr(1); /* num_sectors */
				dcc_wr(4096 | ((unsigned long) flash_page_size << 16)); /* buflen and bufalign */
				break;
			case OCL_ERASE_ALL:
				dcc_wr(OCL_CMD_DONE | flash_erase_all());
				break;
			case OCL_FLASH_BLOCK:
				cmd_flash(cmd);
				break;
			default:
				/* unknown command */
				dcc_wr(OCL_CMD_ERR);
				break;
		}
	}

	return 0; /* we shall never get here, just to suppress compiler warning */
}
