/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_JTAG_DRIVERS_LIBFTDI_HELPER_H
#define OPENOCD_JTAG_DRIVERS_LIBFTDI_HELPER_H

#include <ftdi.h>

#ifndef HAVE_LIBFTDI_TCIOFLUSH
/* Backward compatibility with libftdi pre 1.5 */

static inline int ftdi_tciflush(struct ftdi_context *ftdi)
{
	return ftdi_usb_purge_rx_buffer(ftdi);
}

static inline int ftdi_tcoflush(struct ftdi_context *ftdi)
{
	return ftdi_usb_purge_tx_buffer(ftdi);
}

static inline int ftdi_tcioflush(struct ftdi_context *ftdi)
{
	return ftdi_usb_purge_buffers(ftdi);
}
#endif

#endif /* OPENOCD_JTAG_DRIVERS_LIBFTDI_HELPER_H */
