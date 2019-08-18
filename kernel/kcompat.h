/*
 * DecaWave DW1000 IEEE 802.15.4 UWB wireless driver
 *
 * Copyright (C) 2018 Michael Brown <mbrown@fensystems.co.uk>
 * Copyright (C) 2019 Petri Mattila <petri.mattila@unipart.io>
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

#ifndef __DW1000_KCOMPAT_H
#define __DW1000_KCOMPAT_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/skbuff.h>

#ifndef HAVE_TIMEHIRES
#define HAVE_TIMEHIRES
#warning kernel does not support struct timehires
struct timehires {
	__s64		tv_nsec;		/* nanoseconds */
	__u32		tv_frac;		/* fractional ns */
	__u32		__res;
};
#endif

#ifdef HAVE_HWTSFRAC
static inline ktime_frac_t ns_to_ktime_frac(__u32 frac)
{
	return (ktime_frac_t)(frac);
}
#endif

#ifndef HAVE_HWTSINFO
#warning kernel does not support hwtsinfo
#endif

#ifndef HAVE_HWTSFRAC
#warning kernel does not support hwtsfrac
#endif

#endif /* __DW1000_KCOMPAT_H */
