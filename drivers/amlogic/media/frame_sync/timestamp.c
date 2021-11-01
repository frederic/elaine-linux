/*
 * drivers/amlogic/media/frame_sync/timestamp.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#define DEBUG
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/time64.h>
#include <linux/math64.h>
#include <linux/amlogic/media/frame_sync/tsync.h>
#include <linux/amlogic/media/frame_sync/tsync_pcr.h>
#include <linux/amlogic/media/utils/vdec_reg.h>
#include <linux/amlogic/media/registers/register.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#include <trace/events/meson_atrace.h>


u32 acc_apts_inc;
u32 acc_apts_dec;
u32 acc_pcrscr_inc;
u32 acc_pcrscr_dec;

static DEFINE_SPINLOCK(system_time_lock);
static DEFINE_SPINLOCK(vpts_set_lock);
static s32 system_time_inc_adj;
static struct timespec64 system_time_ts;
static struct timespec64 vpts_set_ts;
static u32 system_time_ts_pts;
static u32 system_time;
static u32 system_time_up;
static u32 audio_pts_up;
static u32 audio_pts_started;
static u32 first_vpts;
static u32 first_checkin_vpts;
static u32 first_checkin_apts;
static u32 first_apts;
static u32 pcrscr_pause_pts;
static bool pcrscr_pause_pts_enable;
static u32 pcrscr_lantcy = 200*90;
static bool vpts_started;
static bool reftime_pause;
static u32 video_pts;
static u32 video_pts_ts;
static u32 audio_pts;

static u32 system_time_scale_base = 1;
static u32 system_time_scale_remainder;

#ifdef MODIFY_TIMESTAMP_INC_WITH_PLL
#define PLL_FACTOR 10000
#define HALF_VSYNC_PERIOD 750
static u32 timestamp_inc_factor = PLL_FACTOR;
void set_timestamp_inc_factor(u32 factor)
{
	timestamp_inc_factor = factor;
}
#endif

bool timestamp_vpts_get_started(void)
{
	return vpts_started;
}

void timestamp_vpts_set_started(bool start)
{
	vpts_started = start;
}

void timestamp_reftime_pause(bool pause)
{
	unsigned long flags;

	spin_lock_irqsave(&system_time_lock, flags);

	reftime_pause = pause;
	if (pause)
		system_time_up = false;

	spin_unlock_irqrestore(&system_time_lock, flags);
}

u32 timestamp_vpts_get(void)
{
	return video_pts;
}
EXPORT_SYMBOL(timestamp_vpts_get);

u32 timestamp_vpts_ts_get(struct timespec64 *ts)
{
	unsigned long flags;
	u32 r;

	spin_lock_irqsave(&vpts_set_lock, flags);

	*ts = vpts_set_ts;
	r = video_pts_ts;

	spin_unlock_irqrestore(&vpts_set_lock, flags);
	return r;
}
EXPORT_SYMBOL(timestamp_vpts_ts_get);

void timestamp_vpts_reset(u32 pts)
{
	video_pts = pts;
}
EXPORT_SYMBOL(timestamp_vpts_reset);

void timestamp_vpts_set(u32 pts)
{
	unsigned long flags;

	spin_lock_irqsave(&vpts_set_lock, flags);

	getrawmonotonic64(&vpts_set_ts);
	video_pts = pts;
	video_pts_ts = pts;

	if (!vpts_started) {
		vpts_started = true;
		pr_info("%lu:%lu first vpts ready\n",
			vpts_set_ts.tv_sec, vpts_set_ts.tv_nsec);
	}

	spin_unlock_irqrestore(&vpts_set_lock, flags);
}
EXPORT_SYMBOL(timestamp_vpts_set);

void timestamp_vpts_inc(s32 val)
{
	video_pts += val;
}
EXPORT_SYMBOL(timestamp_vpts_inc);

u32 timestamp_apts_get(void)
{
	return audio_pts;
}
EXPORT_SYMBOL(timestamp_apts_get);

void timestamp_apts_set(u32 pts)
{
	audio_pts = pts;
}
EXPORT_SYMBOL(timestamp_apts_set);

void timestamp_apts_inc(s32 inc)
{
	if (audio_pts_up) {
#ifdef MODIFY_TIMESTAMP_INC_WITH_PLL
		inc = inc * timestamp_inc_factor / PLL_FACTOR;
#endif
		audio_pts += inc;
	}
}
EXPORT_SYMBOL(timestamp_apts_inc);

void timestamp_apts_enable(u32 enable)
{
	audio_pts_up = enable;
	pr_info("timestamp_apts_enable enable:%x,\n", enable);
}
EXPORT_SYMBOL(timestamp_apts_enable);

void timestamp_apts_start(u32 enable)
{
	audio_pts_started = enable;
	pr_info("audio pts started::::::: %d\n", enable);
}
EXPORT_SYMBOL(timestamp_apts_start);

u32 timestamp_apts_started(void)
{
	return audio_pts_started;
}
EXPORT_SYMBOL(timestamp_apts_started);

u32 timestamp_pcrscr_get(void)
{
	if (tsync_get_mode() == TSYNC_MODE_AMASTER)
		return system_time;

	if (tsdemux_pcrscr_valid_cb && tsdemux_pcrscr_valid_cb()) {
		if (tsync_pcr_demux_pcr_used() == 0) {
			return system_time;
			}
		else {
			if (tsdemux_pcrscr_get_cb)
				return tsdemux_pcrscr_get_cb()-pcrscr_lantcy;
			else
				return system_time;
		}
	} else
	return system_time;
}
EXPORT_SYMBOL(timestamp_pcrscr_get);

u32 timestamp_tsdemux_pcr_get(void)
{
	if (tsdemux_pcrscr_get_cb)
		return tsdemux_pcrscr_get_cb();

	return (u32)-1;
}
EXPORT_SYMBOL(timestamp_tsdemux_pcr_get);

void timestamp_pcrscr_set(u32 pts)
{
	/*pr_info("timestamp_pcrscr_set system time  = %x\n", pts);*/
	ATRACE_COUNTER("PCRSCR",  pts);
	system_time = pts;
}
EXPORT_SYMBOL(timestamp_pcrscr_set);

void timestamp_pcrscr_set_ts(unsigned long tv_sec, unsigned long tv_nsec,
			u32 pts)
{
	unsigned long flags;
	struct timespec64 ts;

	getrawmonotonic64(&ts);

	spin_lock_irqsave(&system_time_lock, flags);

	system_time_ts_pts = pts;
	system_time_ts.tv_sec = tv_sec;
	system_time_ts.tv_nsec = tv_nsec;

	spin_unlock_irqrestore(&system_time_lock, flags);

	pr_debug("timestamp_pcrscr_set_ts %lu.%.9lu: 0x%x\n",
		tv_sec, tv_nsec, pts);
}
EXPORT_SYMBOL(timestamp_pcrscr_set_ts);

void timestamp_firstvpts_set(u32 pts)
{
	first_vpts = pts;
	pr_info("video first pts = %x\n", first_vpts);
}
EXPORT_SYMBOL(timestamp_firstvpts_set);

u32 timestamp_firstvpts_get(void)
{
	return first_vpts;
}
EXPORT_SYMBOL(timestamp_firstvpts_get);

void timestamp_checkin_firstvpts_set(u32 pts)
{
	first_checkin_vpts = pts;
	pr_info("video first checkin pts = %x\n", first_checkin_vpts);
}
EXPORT_SYMBOL(timestamp_checkin_firstvpts_set);

void timestamp_checkin_firstapts_set(u32 pts)
{
	first_checkin_apts = pts;
	pr_info("audio first checkin pts =%x\n", first_checkin_apts);
}
EXPORT_SYMBOL(timestamp_checkin_firstapts_set);

u32 timestamp_checkin_firstvpts_get(void)
{
	return first_checkin_vpts;
}
EXPORT_SYMBOL(timestamp_checkin_firstvpts_get);

u32 timestamp_checkin_firstapts_get(void)
{
	return first_checkin_apts;
}
EXPORT_SYMBOL(timestamp_checkin_firstapts_get);

void timestamp_firstapts_set(u32 pts)
{
	first_apts = pts;
	pr_info("audio first pts = %x\n", first_apts);
}
EXPORT_SYMBOL(timestamp_firstapts_set);

u32 timestamp_firstapts_get(void)
{
	return first_apts;
}
EXPORT_SYMBOL(timestamp_firstapts_get);

static void timestamp_reftime_setup(void)
{
	unsigned long flags;
	struct timespec64 t;

	getrawmonotonic64(&t);

	spin_lock_irqsave(&system_time_lock, flags);

	if (system_time_ts.tv_sec || system_time_ts.tv_nsec) {
		if (timespec64_compare(&t, &system_time_ts) > 0) {
			struct timespec64 t1 = timespec64_sub(t,
				system_time_ts);
			s64 delay_ns = timespec64_to_ns(&t1);
			u32 delay_pts = div64_s64(delay_ns * 90, 1000000LL) &
				0xffffffff;

			pr_info("timestamp_reftime_setup: %lu:%lu > %lu:%lu\n",
				t.tv_sec, t.tv_nsec,
				system_time_ts.tv_sec, system_time_ts.tv_nsec);

			timestamp_pcrscr_set(system_time_ts_pts + delay_pts
				- HALF_VSYNC_PERIOD);

			pr_info("pcrscr set to %x->%x\n",
				system_time_ts_pts,
				system_time_ts_pts + delay_pts
					- HALF_VSYNC_PERIOD);

			system_time_ts.tv_sec = 0;
			system_time_ts.tv_nsec = 0;

			/* whether we start system time or not is
			 * determined by current decoder status is
			 * in paused status or not
			 */
			if (!reftime_pause)
				system_time_up = true;
		}
	}

	spin_unlock_irqrestore(&system_time_lock, flags);
}

void timestamp_pcrscr_inc(s32 inc)
{
	timestamp_reftime_setup();

	if (system_time_up) {
#ifdef MODIFY_TIMESTAMP_INC_WITH_PLL
		inc = inc * timestamp_inc_factor / PLL_FACTOR;
#endif
		if ((tsync_get_mode() == TSYNC_MODE_EXTERNAL) &&
			(tsync_get_external_rate() >= 0)) {
			system_time += tsync_get_external_rate();
		} else {
			system_time += inc + system_time_inc_adj;
			ATRACE_COUNTER("PCRSCR",  system_time);
		}
	}

	if (pcrscr_pause_pts_enable) {
		if ((s32)(system_time - pcrscr_pause_pts) >= 0) {
			system_time_up = 0;
			pcrscr_pause_pts_enable = 0;
		}
	}
}
EXPORT_SYMBOL(timestamp_pcrscr_inc);

void timestamp_pcrscr_inc_scale(s32 inc, u32 base)
{
	timestamp_reftime_setup();

	if (system_time_scale_base != base) {
		system_time_scale_remainder =
			system_time_scale_remainder *
			base / system_time_scale_base;
		system_time_scale_base = base;
	}

	if (system_time_up) {
		u32 r;

		if ((tsync_get_mode() == TSYNC_MODE_EXTERNAL) &&
			tsync_get_external_rate() >= 0) {
			system_time += tsync_get_external_rate();
		} else {
			system_time +=
				div_u64_rem(90000ULL * inc, base, &r) +
				system_time_inc_adj;
			system_time_scale_remainder += r;
			if (system_time_scale_remainder >=
				system_time_scale_base) {
				system_time++;
				system_time_scale_remainder -=
					system_time_scale_base;
			}
		}
	}

	if (pcrscr_pause_pts_enable) {
		if ((s32)(system_time - pcrscr_pause_pts) >= 0) {
			system_time_up = 0;
			pcrscr_pause_pts_enable = 0;
		}
		ATRACE_COUNTER("PCRSCR",  system_time);
	}
}

EXPORT_SYMBOL(timestamp_pcrscr_inc_scale);

void timestamp_pcrscr_set_adj(s32 inc)
{
	system_time_inc_adj = inc;
}
EXPORT_SYMBOL(timestamp_pcrscr_set_adj);

void timestamp_pcrscr_set_adj_pcr(s32 inc)
{
	const struct vinfo_s *info = get_current_vinfo();

	if (inc != 0) {
		system_time_inc_adj =
			900 * info->sync_duration_den /
			(info->sync_duration_num*inc);
	} else
		system_time_inc_adj = 0;
}
EXPORT_SYMBOL(timestamp_pcrscr_set_adj_pcr);

void timestamp_pcrscr_pause_pts(u32 pts)
{
	pcrscr_pause_pts = pts;
	pcrscr_pause_pts_enable = true;
}
EXPORT_SYMBOL(timestamp_pcrscr_pause_pts);

void timestamp_pcrscr_enable(u32 enable)
{
	system_time_up = enable;
	pcrscr_pause_pts_enable = false;
}
EXPORT_SYMBOL(timestamp_pcrscr_enable);

u32 timestamp_pcrscr_enable_state(void)
{
	return system_time_up;
}
EXPORT_SYMBOL(timestamp_pcrscr_enable_state);

MODULE_DESCRIPTION("AMLOGIC time sync management driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Yao <timyao@amlogic.com>");
