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

#include <linux/timecounter.h>
#include "kcompat.h"

static u64 cc_cyc2ns_backwards(const struct cyclecounter *cc,
			       u64 cycles, u64 mask, u64 *frac)
{
	u64 ns = (u64) cycles;

	ns = (ns * cc->mult) - *frac;
	*frac = (-ns) & mask;
	return (ns >> cc->shift) + 1;
}

u64 timecounter_cyc2time_frac(struct timecounter *tc,
			      u64 cycle_tstamp, u64 *frac)
{
	u64 delta = (cycle_tstamp - tc->cycle_last) & tc->cc->mask;
	u64 nsec = tc->nsec;

	/*
	 * Instead of always treating cycle_tstamp as more recent
	 * than tc->cycle_last, detect when it is too far in the
	 * future and treat it as old time stamp instead.
	 */
	*frac = tc->frac;
	if (delta > tc->cc->mask / 2) {
		delta = (tc->cycle_last - cycle_tstamp) & tc->cc->mask;
		nsec -= cc_cyc2ns_backwards(tc->cc, delta, tc->mask, frac);
	} else {
		nsec += cyclecounter_cyc2ns(tc->cc, delta, tc->mask, frac);
	}

	return nsec;
}
