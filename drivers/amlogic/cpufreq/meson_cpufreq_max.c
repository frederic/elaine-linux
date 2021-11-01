/*
 * drivers/amlogic/cpufreq/meson_cpufreq_max.c
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/amlogic/scpi_protocol.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/of.h>

#define CPUFREQ_1	2016
#define CPUFREQ_2	1896
#define CPUFREQ_3	1704


static int g12a_maxfreq_get(int cluster)
{
	unsigned char ringinfo[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	int cpuring, sltver, cpufreq, cpuvolt;

	if (scpi_get_ring_value(ringinfo) != 0) {
		pr_err("fail get osc ring efuse info\n");
		return 0;
	}

	cpuring = ringinfo[4] * 20;
	sltver = ringinfo[8];
	cpufreq = 0;
	cpuvolt = 0;

	if ((sltver & 0x20) && (!cluster)) {
		cpufreq = ringinfo[9 + 0] >> 4;
		cpuvolt = ringinfo[9 + 0] & 0xf;
	} else {
		pr_err("no slt efuse info\n");
		return 0;
	}

	if ((cpuring >= 3200) && (cpufreq >= 10) && (cpuvolt <= 10))
		return CPUFREQ_1;

	if ((cpuring >= 3100) && (cpufreq >= 9) && (cpuvolt <= 10))
		return CPUFREQ_2;

	if ((cpuring >= 2900) && (cpufreq >= 8) && (cpuvolt <= 10))
		return CPUFREQ_3;

	return 0;
}

static int g12b_maxfreq_get(int cluster)
{
	unsigned char ringinfo[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned char chipver[4] = {0};
	int cpuring, sltver, cpufreq, cpuvolt;

	if (scpi_get_ring_value(ringinfo) != 0) {
		pr_err("fail get osc ring efuse info\n");
		return 0;
	}

	cpuring = ringinfo[4] * 20;
	sltver = ringinfo[8];
	cpufreq = 0;
	cpuvolt = 0;

	if ((sltver & 0x20) && (!cluster)) {
		cpufreq = ringinfo[9 + 0] >> 4;
		cpuvolt = ringinfo[9 + 0] & 0xf;
	} else if ((sltver & 0x40) && cluster) {
		cpufreq = ringinfo[9 + 1] >> 4;
		cpuvolt = ringinfo[9 + 1] & 0xf;
	} else {
		pr_err("no slt efuse info\n");
	}

	if ((cpuring >= 3200) && (cpufreq >= 10) && (cpuvolt <= 10))
		return CPUFREQ_1;

	if ((cpuring >= 3100) && (cpufreq >= 9) && (cpuvolt <= 10))
		return CPUFREQ_2;

	if ((cpuring >= 2900) && (cpufreq >= 8) && (cpuvolt <= 10))
		return CPUFREQ_3;

	if (scpi_get_chipver_value(chipver) != 0) {
		pr_err("fail get osc ring efuse info\n");
		return 0;
	}

	if (cluster == 0x1) {
		/* es0: bit[4-6] = 1 */
		if (((chipver[1] >> 4) & 0x7) == 0x1)
			return CPUFREQ_3;

		/* es1: bit[4-6] = 3 */
		if (((chipver[1] >> 4) & 0x7) == 0x3)
			return CPUFREQ_2;

		/* es1: default */
		if (chipver[1] == 0x0)
			return CPUFREQ_2;
	}

	return 0;
}

int meson_cpufreq_max(int cluster)
{
	int freq;

	switch (get_cpu_type()) {
	case MESON_CPU_MAJOR_ID_G12A:
		freq = g12a_maxfreq_get(cluster);
		break;
	case MESON_CPU_MAJOR_ID_G12B:
		freq = g12b_maxfreq_get(cluster);
		break;
	default:
		pr_info("Unsupported chip clk measure\n");
		freq = 0;
		break;
	}

	return freq;

}
EXPORT_SYMBOL(meson_cpufreq_max);
