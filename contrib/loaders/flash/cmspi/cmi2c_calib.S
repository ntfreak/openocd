/***************************************************************************
 *   Copyright (C) 2018 by Andreas Bolsch                                  *
 *   andreas.bolsch@mni.thm.de                                             *
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

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

/* Params:
 * r0 - loop counter
 *
 * Clobbered:
 * r0 - r7 */

#include "cmspi.s"

start:
	mov		r4, r0						/* copy loop counter */
	setup_regs_i2c						/* initialize registers */
loop:
	i2c_set_scl							/* no effect, just delay */
	i2c_set_sda							/* no effect, just delay */
	i2c_set_sda							/* no effect, just delay */
	i2c_set_scl							/* no effect, just delay */
	subs	r4, r4, #1					/* decrement loop counter */
	bne		loop						/* again if non-zero */

exit:
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
