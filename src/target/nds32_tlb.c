/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "nds32_aice.h"
#include "nds32_tlb.h"

int nds32_probe_tlb(struct nds32 *nds32, const target_addr_t virtual_address,
		target_addr_t *physical_address)
{
	struct target *target = nds32->target;
	struct aice_port_s *aice = target_to_aice(target);

	return aice_read_tlb(aice, virtual_address, physical_address);
}

static struct page_table_walker_info_s page_table_info[PAGE_SIZE_NUM] = {
	/* 4K page */
	{0xFFC00000, 20, 0x003FF000, 10, 0x00000FFF, 0xFFFFF000, 0xFFFFF000, 0xFFFFF000},
	/* 8K page */
	{0xFF000000, 22, 0x00FFE000, 11, 0x00001FFF, 0xFFFFF000, 0xFFFFE000, 0xFFFFE000},
};

int nds32_walk_page_table(struct nds32 *nds32, const target_addr_t virtual_address,
		target_addr_t *physical_address)
{
	struct target *target = nds32->target;
	uint32_t value_mr1;
	uint32_t load_address;
	uint32_t l1_page_table_entry;
	uint32_t l2_page_table_entry;
	uint32_t page_size_index = nds32->mmu_config.default_min_page_size;
	struct page_table_walker_info_s *page_table_info_p =
		&(page_table_info[page_size_index]);

	/* Read L1 Physical Page Table */
	nds32_get_mapped_reg(nds32, MR1, &value_mr1);
	load_address = (value_mr1 & page_table_info_p->l1_base_mask) |
		((virtual_address & page_table_info_p->l1_offset_mask) >>
		 page_table_info_p->l1_offset_shift);
	/* load_address is physical address */
	nds32_read_buffer(target, load_address, 4, (uint8_t *)&l1_page_table_entry);

	/* Read L2 Physical Page Table */
	if (l1_page_table_entry & 0x1) /* L1_PTE not present */
		return ERROR_FAIL;

	load_address = (l1_page_table_entry & page_table_info_p->l2_base_mask) |
		((virtual_address & page_table_info_p->l2_offset_mask) >>
		 page_table_info_p->l2_offset_shift);
	/* load_address is physical address */
	nds32_read_buffer(target, load_address, 4, (uint8_t *)&l2_page_table_entry);

	if ((l2_page_table_entry & 0x1) != 0x1) /* L2_PTE not valid */
		return ERROR_FAIL;

	*physical_address = (l2_page_table_entry & page_table_info_p->ppn_mask) |
		(virtual_address & page_table_info_p->va_offset_mask);

	return ERROR_OK;
}
