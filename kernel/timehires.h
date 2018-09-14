/*
 * timehires.h
 *
 * based on code from linux/include/linux/clockcounter.h
 *
 * Copyright (C) 2018 Petri Mattila <petri.mattila@unipart.io>
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
#ifndef __HIRES_COUNTER_H__
#define __HIRES_COUNTER_H__

#include <linux/types.h>
#include <linux/time.h>

#include "kcompat.h"


typedef __u64 cycle_t;

/**
 * struct hires_counter - hardware abstraction for a free running counter
 *
 * @cycle_read:		returns the current cycle value
 * @time_sync:		timestamp of the syncronization point
 * @cycle_sync:		cycles count of the syncronization point
 * @cycle_mask:		bitmask of the running counter bits
 * @ns_mult_fwd:	multiplier for converting cycles to ns, after sync
 * @ns_mult_bwd:	multiplier for converting cycles to ns, before sync
 */
struct hires_counter {
	cycle_t (*cycle_read)(const struct hires_counter *);
	struct timehires time_sync;
	cycle_t cycle_time;
	cycle_t cycle_sync;
	cycle_t cycle_mask;
	cycle_t cycle_max;
	u64 ns_mult_fwd;
	u64 ns_mult_bwd;
};

/* Initialize counter structure */
extern void hires_counter_init(
	struct hires_counter *tc,
	cycle_t (*read)(const struct hires_counter *),
	struct timehires start,
	u32 bits, u64 mult);

/* Syncronise underlying counter with the current time */
extern void hires_counter_sync(
	struct hires_counter *tc);

/* Set conversion multiplier */
extern void hires_counter_setmult(
	struct hires_counter *tc,
	u64 mult_set);

/* Set current time */
extern void hires_counter_settime(
	struct hires_counter *tc,
	struct timehires time_set);

/* Adjust current time */
extern void hires_counter_adjtime(
	struct hires_counter *tc,
	struct timehires time_adj);


/* Convert cycle count to monotonous cycle time */
extern cycle_t hires_counter_cyc2raw(
	struct hires_counter *tc,
	cycle_t cycle_count);

/* Convert cycle count to HiRes timestamp */
extern struct timehires hires_counter_cyc2time(
	struct hires_counter *tc,
	cycle_t cycle_count);


/* Read underlying counter */
static inline cycle_t hires_counter_read(struct hires_counter *tc)
{
	return tc->cycle_read(tc);
}


#endif /* __HIRES_COUNTER_H__ */
