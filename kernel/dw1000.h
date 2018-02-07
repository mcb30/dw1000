/*
 * DecaWave DW1000 IEEE 802.15.4 UWB wireless driver
 *
 * Copyright (C) 2018 Michael Brown <mbrown@fensystems.co.uk>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __DW1000_H
#define __DW1000_H

#include <linux/device.h>
#include <linux/regmap.h>

/* Register map parameters */
#define DW1000_WRITE_FLAG_MASK 0x80
#define DW1000_READ_FLAG_MASK 0x00
#define DW1000_DWORD_REG_BITS 6
#define DW1000_DWORD_PAD_BITS 2
#define DW1000_DWORD_VAL_BITS 32

/* Device ID register */
#define DW1000_DEV_ID 0x00

/* Register maps */
struct dw1000_registers {
	struct regmap *dword;
};

/* DW1000 device */
struct dw1000 {
	struct spi_device *spi;
	struct dw1000_registers regs;
};

#endif /* __DW1000_H */
