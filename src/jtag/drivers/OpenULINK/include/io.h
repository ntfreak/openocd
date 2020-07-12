/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
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

#ifndef __IO_H
#define __IO_H

#include "reg_ezusb.h"

/***************************************************************************
 *  JTAG Signals:                                                          *
 ***************************************************************************
 * TMS ....... Test Mode Select                                            *
 * TCK ....... Test Clock                                                  *
 * TDI ....... Test Data Input  (from device point of view, not JTAG       *
 *             adapter point of view!)                                     *
 * TDO ....... Test Data Output (from device point of view, not JTAG       *
 *             adapter point of view!)                                     *
 * TRST ...... Test Reset: Used to reset the TAP Finite State Machine      *
 *             into the Test Logic Reset state                             *
 * RTCK ...... Return Test Clock                                           *
 * OCDSE ..... Enable/Disable OCDS interface (Infineon specific) - shared  *
 *             with /JEN                                                   *
 * TRAP ...... Trap Condition (Infineon specific) - shared with TSTAT      *
 * BRKIN ..... Hardware Break-In (Infineon specific)                       *
 * BRKOUT .... Hardware Break-Out (Infineon specific)                      *
 * /JEN ...... JTAG-Enable (STMicroelectronics specific) - shared          *
 *             with OCDSE                                                  *
 * TSTAT ..... JTAG ISP Status (STMicroelectronics specific) - shared      *
 *             with TRAP                                                   *
 * RESET ..... Chip Reset (STMicroelectronics specific)                    *
 * /TERR ..... JTAG ISP Error (STMicroelectronics specific) - shared       *
 *             with BRKOUT                                                 *
 ***************************************************************************/

/* PORT A */
#define PIN_U_OE      OUTA0
/* PA1 Not Connected */
#define PIN_OE        OUTA2
/* PA3 Not Connected */
#define PIN_RUN_LED   OUTA4
#define PIN_TDO       PINA5
#define PIN_BRKOUT    PINA6
#define PIN_COM_LED   OUTA7

/* PORT B */
#define PIN_TDI       OUTB0
#define PIN_TMS       OUTB1
#define PIN_TCK       OUTB2
#define PIN_TRST      OUTB3
#define PIN_BRKIN     OUTB4
#define PIN_RESET     OUTB5
#define PIN_OCDSE     OUTB6
#define PIN_TRAP      PINB7

/* JTAG Signals with direction 'OUT' on port B */
#define MASK_PORTB_DIRECTION_OUT (PIN_TDI | PIN_TMS | PIN_TCK | PIN_TRST | PIN_BRKIN | PIN_RESET | PIN_OCDSE)

/* PORT C */
#define PIN_RXD0      PINC0
#define PIN_TXD0      OUTC1
#define PIN_RESET_2   PINC2
/* PC3 Not Connected */
/* PC4 Not Connected */
#define PIN_RTCK      PINC5
#define PIN_WR        OUTC6
/* PC7 Not Connected */

/* LED Macros */
#define SET_RUN_LED()     (OUTA &= ~PIN_RUN_LED)
#define CLEAR_RUN_LED()   (OUTA |=  PIN_RUN_LED)

#define SET_COM_LED()     (OUTA &= ~PIN_COM_LED)
#define CLEAR_COM_LED()   (OUTA |=  PIN_COM_LED)

/* JTAG Pin Macros */
#define GET_TMS()         (PINSB & PIN_TMS)
#define GET_TCK()         (PINSB & PIN_TCK)

#define GET_TDO()         (PINSA & PIN_TDO)
#define GET_BRKOUT()      (PINSA & PIN_BRKOUT)
#define GET_TRAP()        (PINSB & PIN_TRAP)
#define GET_RTCK()        (PINSC & PIN_RTCK)

#define SET_TMS_HIGH()    (OUTB |=  PIN_TMS)
#define SET_TMS_LOW()     (OUTB &= ~PIN_TMS)

#define SET_TCK_HIGH()    (OUTB |=  PIN_TCK)
#define SET_TCK_LOW()     (OUTB &= ~PIN_TCK)

#define SET_TDI_HIGH()    (OUTB |=  PIN_TDI)
#define SET_TDI_LOW()     (OUTB &= ~PIN_TDI)

/* TRST and RESET are low-active and inverted by hardware. SET_HIGH de-asserts
 * the signal (enabling reset), SET_LOW asserts the signal (disabling reset) */
#define SET_TRST_HIGH()   (OUTB |=  PIN_TRST)
#define SET_TRST_LOW()    (OUTB &= ~PIN_TRST)

#define SET_RESET_HIGH()  (OUTB |=  PIN_RESET)
#define SET_RESET_LOW()   (OUTB &= ~PIN_RESET)

#define SET_OCDSE_HIGH()  (OUTB |=  PIN_OCDSE)
#define SET_OCDSE_LOW()   (OUTB &= ~PIN_OCDSE)

#define SET_BRKIN_HIGH()  (OUTB |=  PIN_BRKIN)
#define SET_BRKIN_LOW()   (OUTB &= ~PIN_BRKIN)

#endif
