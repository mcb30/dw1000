/*
 * timehires.c
 *
 * based on code from linux/kernel/time/timecounter.c
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

#include <linux/time.h>
#include <linux/export.h>

#include "timehires.h"


typedef union {
	u64 D;
	struct {
#ifdef __BIG_ENDIAN
		u32 H, L;
#else
		u32 L, H;
#endif
	};
} n64;


/**
 * timehires_add - Add two timestamps
 * @a:       Timestamp #1
 * @a:       Timestamp #2
 *
 * Return: a + b
 */
static struct timehires timehires_add(
	struct timehires a,
	struct timehires b)
{
	struct timehires res;
	n64 c,d;

	d.D = (s64) a.tv_frac + b.tv_frac;
	c.D = (s64) a.tv_nsec + b.tv_nsec + (s32) d.H;

	res.tv_nsec = c.D;
	res.tv_frac = d.L;

	return res;
}

/**
 * timehires_sub - Subtract two timestamps
 * @a:		Timestamp #1
 * @a:		Timestamp #2
 *
 * Return: a - b
 */
static struct timehires timehires_sub(
	struct timehires a,
	struct timehires b)
{
	struct timehires res;
	n64 c,d;

	d.D = (s64) a.tv_frac - b.tv_frac;
	c.D = (s64) a.tv_nsec - b.tv_nsec + (s32) d.H;

	res.tv_nsec = c.D;
	res.tv_frac = d.L;

	return res;
}

/**
 * timehires_mull64 - Multiply two u64 numbers for creating a timestamp
 * @cycles:	Cycle count
 * @mull:	Multiplier
 *
 * Return: (cycles * mull) >> 64
 */
static struct timehires timehires_mull64(
	u64 cycles, u64 mull)
{
	struct timehires res;
	n64 x,y,a,b,c,d,e;

	x.D = mull;
	y.D = cycles;

	a.D = (u64) x.L * y.L;
	b.D = (u64) x.L * y.H;
	c.D = (u64) x.H * y.L;
	d.D = (u64) x.H * y.H;

	e.D = (u64) a.H + b.L + c.L;
	d.D = (u64) d.D + b.H + c.H + e.H;

	res.tv_nsec = d.D;
	res.tv_frac = e.D;

	return res;
}


/**
 * hires_counter_init - Initialise a high resolution cycle counter
 * @tc:		Pointer to HiRes counter
 * @start:	Starting time of the counter
 * @bits:	Size of the underlying HW counter
 * @mult:	Multiplier for converting the HW cycles to ns
 */
void hires_counter_init(
	struct hires_counter *tc,
	cycle_t (*read)(const struct hires_counter *),
	struct timehires start,
	u32 bits, u64 mult)
{
	tc->cycle_read = read;
	tc->cycle_sync = read(tc);
	tc->cycle_mask = (1ULL<<bits)-1;
	tc->cycle_max  = (1ULL<<(bits-1));
	tc->time_sync  = start;
	tc->ns_mult1   = mult;
	tc->ns_mult2   = mult;
}

/**
 * hires_counter_sync - Syncronise the counter with the underlying HW
 * @tc:		Pointer to HiRes counter
 *
 * When the underlying cycle counter runs over, this will be handled
 * correctly as long as it does not run over more than once between
 * the sync calls.
 */
void hires_counter_sync(
	struct hires_counter *tc)
{
	struct timehires time_delta;
	cycle_t cycle_delta;
	cycle_t cycle_time;

	cycle_time  = hires_counter_read(tc);
	cycle_delta = (cycle_time - tc->cycle_sync) & tc->cycle_mask;
	time_delta  = timehires_mull64(cycle_delta, tc->ns_mult1);

	tc->cycle_sync = cycle_time;
	tc->time_sync = timehires_add(tc->time_sync, time_delta);
	tc->ns_mult2 = tc->ns_mult1;
}

/**
 * hires_counter_setmult - Set the conversion multiplier
 * @tc:		Pointer to HiRes counter
 * @mult_set:	New multiplier
 *
 */
void hires_counter_setmult(
	struct hires_counter *tc,
	u64 mult_set)
{
	hires_counter_sync(tc);

	tc->ns_mult1 = mult_set;
}

/**
 * hires_counter_settime - Set the current time
 * @tc:		Pointer to HiRes counter
 * @time_set:	New time
 *
 */
void hires_counter_settime(
	struct hires_counter *tc,
	struct timehires time_set)
{
	hires_counter_sync(tc);

	tc->time_sync = time_set;
}

/**
 * hires_counter_adjtime - Adjust the current time
 * @tc:		Pointer to HiRes counter
 * @time_adj:	Time adjustment in nanoseconds
 *
 */
void hires_counter_adjtime(
	struct hires_counter *tc,
	struct timehires time_adj)
{
	hires_counter_sync(tc);

	tc->time_sync = timehires_add(tc->time_sync, time_adj);
}

/**
 * hires_counter_cyc2time - Convert HW cycles to HiRes timestamp
 * @tc:		Pointer to HiRes counter
 * @cycle_time:	Cycle counter value
 *
 * Convert the HW cycle count to 64.32 [ns] HiRes timestamp.
 *
 * Handle the HW counter wrap around correctly.
 *
 * If the HW cycle count is before a recent multiplier change,
 * use the previous multiplier, in order to maintain monotonic time.
 */
struct timehires hires_counter_cyc2time(
	struct hires_counter *tc,
	cycle_t cycle_time)
{
	struct timehires time;
	cycle_t cycle_delta;

	cycle_delta = (cycle_time - tc->cycle_sync) & tc->cycle_mask;

	if (cycle_delta < tc->cycle_max) {
		time = timehires_mull64(cycle_delta, tc->ns_mult1);
		time = timehires_add(tc->time_sync, time);
	}
	else {
		cycle_delta = (tc->cycle_sync - cycle_time) & tc->cycle_mask;

		time = timehires_mull64(cycle_delta, tc->ns_mult2);
		time = timehires_sub(tc->time_sync, time);
	}

	return time;
}

