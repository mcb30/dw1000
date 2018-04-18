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

#ifndef __DW1000_KCOMPAT_H
#define __DW1000_KCOMPAT_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/ktime.h>

#ifndef HAVE_TIMEHIRES
#define HAVE_TIMEHIRES
struct timehires {
	__s64		tv_nsec;		/* nanoseconds */
	__u32		tv_frac;		/* fractional ns */
	__u32		__res;
};
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
typedef __u64	cycle_t;
#endif

#ifdef HAVE_HWTSFRAC
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
static inline ktime_frac_t ns_to_ktime_frac(__u32 frac)
{
	return (ktime_frac_t)(frac);
}
#else
static inline ktime_frac_t ns_to_ktime_frac(__u32 frac)
{
	ktime_frac_t time = { .tf32 = frac };
	return time;
}
#endif
#endif

#endif /* __DW1000_KCOMPAT_H */
