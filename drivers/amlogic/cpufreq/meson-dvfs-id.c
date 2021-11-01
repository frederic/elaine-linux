/*
 * drivers/amlogic/cpufreq/meson-dvfs-id.c
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/topology.h>
#include <linux/delay.h>
#include <linux/amlogic/scpi_protocol.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/arm-smccc.h>

#define GET_DVFS_TABLE_INDEX           0x82000088

static char dvfs_id;
static struct class *dvfsid_cls;
static char *dvfsid_dev = "dvfs_id";

static unsigned int get_cpufreq_table_index(u64 function_id,
					    u64 arg0, u64 arg1, u64 arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc((unsigned long)function_id,
		      (unsigned long)arg0,
		      (unsigned long)arg1,
		      (unsigned long)arg2,
		      0, 0, 0, 0, &res);
	return res.a0;
}

static ssize_t show_dvfsid(struct class *class,
	struct class_attribute *attr, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", dvfs_id);
	return ret;
}

static ssize_t show_ringmsr(struct class *class,
	struct class_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char ringinfo[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	char str[512] = {0};
	char *name[] = {"unknown", "cpu_ring0", "mail_ring1", "dmc_ring", "cpu_ring2", "iddee", "reserve", "iddcpu"};
	int i = 0;

	if (scpi_get_ring_value(ringinfo) != 0) {
		pr_err("fail get osc ring efuse info\n");
		return 0;
	}
	if (is_meson_sm1_cpu()) {
		for (i = 1; i <= 4; i++)
			ret += sprintf(str + ret, "%s:%d KHz, ", name[i], (ringinfo[i] * 20)); //cpu_ring0,mail_ring1,dmc_ring,cpu_ring2

		ret += sprintf(str + ret, "%s:%d uA, ", name[i], (ringinfo[i] * 400)); //iddee
		i += 2;
		ret += sprintf(str + ret, "%s:%d uA, ", name[i], (ringinfo[i] * 400)); //iddcpu
	} else
		pr_err("show iddcpu: not support non-sm1 chip!\n");

	ret = sprintf(buf, "%s\n", str);
	return ret;
}

static struct class_attribute dvfsid_class_attrs[] = {
	__ATTR(dvfs_id, 0444, show_dvfsid, NULL),
	__ATTR(ringmsr, 0444, show_ringmsr, NULL),
	__ATTR_NULL
};

static int dvfsid_probe(struct platform_device *pdev)
{
	dvfs_id = get_cpufreq_table_index(GET_DVFS_TABLE_INDEX, 0, 0, 0);

	pr_info("%s id:%d\n", __func__, dvfs_id);

	dvfsid_cls = kzalloc(sizeof(struct class), GFP_KERNEL);
	if (dvfsid_cls == NULL) {
		pr_err("%s no mem for zalloc\n", __func__);
		return -1;
	}
	dvfsid_cls->name = dvfsid_dev;
	dvfsid_cls->class_attrs = dvfsid_class_attrs;
	if (class_register(dvfsid_cls) < 0) {
		pr_err("failed to class_reg for dvfsid\n");
		return -1;
	}

	return 0;
}

static const struct of_device_id dvfsid_dt_match[] = {
	{ .compatible = "amlogic, dvfs-id" },
	{ /* sentinel */ },
};

static  struct platform_driver dvfsid_platform_driver = {
	.probe		= dvfsid_probe,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "dvfs-id",
		.of_match_table	= dvfsid_dt_match,
	},
};

static int __init meson_dvfsid_init(void)
{
	return  platform_driver_register(&dvfsid_platform_driver);
}
static void __exit meson_dvfsid_exit(void)
{
	kfree(dvfsid_cls);
	platform_driver_unregister(&dvfsid_platform_driver);
}
module_init(meson_dvfsid_init);
module_exit(meson_dvfsid_exit);
