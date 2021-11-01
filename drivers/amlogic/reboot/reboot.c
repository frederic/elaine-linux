/*
 * drivers/amlogic/reboot/reboot.c
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>

#include <asm/system_misc.h>

#include <linux/amlogic/iomap.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/reboot.h>
#include <asm/compiler.h>
#include <linux/kdebug.h>
#include <linux/arm-smccc.h>
#ifdef CONFIG_AMLOGIC_RAMDUMP
#include <linux/amlogic/ramdump.h>
#define RAMDUMP_REPLACE_MSG	"ramdump disabled, replase panic to normal\n"
#endif /* CONFIG_AMLOGIC_RAMDUMP */
#include <linux/slab.h>

static u32 psci_function_id_restart;
static u32 psci_function_id_poweroff;
static char *kernel_panic;
static struct class *sticky_reg_cls;
static char *sticky_reg_dev = "sticky_reg";
static unsigned int *sticky_reg0;
static unsigned int *sticky_reg1;

static u32 parse_reason(const char *cmd)
{
	u32 reboot_reason = MESON_NORMAL_BOOT;

	if (cmd) {
		if (strcmp(cmd, "recovery") == 0 ||
				strcmp(cmd, "factory_reset") == 0)
			reboot_reason = MESON_FACTORY_RESET_REBOOT;
		else if (strcmp(cmd, "update") == 0)
			reboot_reason = MESON_UPDATE_REBOOT;
		else if (strcmp(cmd, "fastboot") == 0)
			reboot_reason = MESON_FASTBOOT_REBOOT;
		else if (strcmp(cmd, "bootloader") == 0)
			reboot_reason = MESON_BOOTLOADER_REBOOT;
		else if (strcmp(cmd, "rpmbp") == 0)
			reboot_reason = MESON_RPMBP_REBOOT;
		else if (strcmp(cmd, "report_crash") == 0)
			reboot_reason = MESON_CRASH_REBOOT;
		else if (strcmp(cmd, "uboot_suspend") == 0)
			reboot_reason = MESON_UBOOT_SUSPEND;
		else if (strcmp(cmd, "quiescent") == 0 ||
				strcmp(cmd, ",quiescent") == 0)
			reboot_reason = MESON_QUIESCENT_REBOOT;
		else if (strcmp(cmd, "recovery,quiescent") == 0 ||
				strcmp(cmd, "factory_reset,quiescent") == 0 ||
				strcmp(cmd, "quiescent,recovery") == 0 ||
				strcmp(cmd, "quiescent,factory_reset") == 0)
			reboot_reason = MESON_RECOVERY_QUIESCENT_REBOOT;
	} else {
		if (kernel_panic) {
			if (strcmp(kernel_panic, "kernel_panic") == 0) {
			#ifdef CONFIG_AMLOGIC_RAMDUMP
				if (ramdump_disabled()) {
					reboot_reason = MESON_NORMAL_BOOT;
					pr_info(RAMDUMP_REPLACE_MSG);
				} else
					reboot_reason = MESON_KERNEL_PANIC;
			#else
				reboot_reason = MESON_KERNEL_PANIC;
			#endif
			}
		}

	}

	pr_info("reboot reason %d\n", reboot_reason);
	return reboot_reason;
}
static noinline int __invoke_psci_fn_smc(u64 function_id, u64 arg0, u64 arg1,
					 u64 arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc((unsigned long)function_id,
			(unsigned long)arg0,
			(unsigned long)arg1,
			(unsigned long)arg2,
			0, 0, 0, 0, &res);
	return res.a0;
}
void meson_smc_restart(u64 function_id, u64 reboot_reason)
{
	__invoke_psci_fn_smc(function_id,
				reboot_reason, 0, 0);
}
void meson_common_restart(char mode, const char *cmd)
{
	u32 reboot_reason = parse_reason(cmd);

	if (psci_function_id_restart)
		meson_smc_restart((u64)psci_function_id_restart,
						(u64)reboot_reason);
}
static void do_aml_restart(enum reboot_mode reboot_mode, const char *cmd)
{
	meson_common_restart(reboot_mode, cmd);
}

static void do_aml_poweroff(void)
{
	/* TODO: Add poweroff capability */
	__invoke_psci_fn_smc(0x82000042, 1, 0, 0);
	__invoke_psci_fn_smc(psci_function_id_poweroff,
				0, 0, 0);
}
static int panic_notify(struct notifier_block *self,
			unsigned long cmd, void *ptr)
{
	kernel_panic = "kernel_panic";

	return NOTIFY_DONE;
}

static struct notifier_block panic_notifier = {
	.notifier_call	= panic_notify,
};

static ssize_t sticky_reg_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int reg0_val;
	unsigned int reg1_val;

	if ((sticky_reg0 != 0) && (sticky_reg1 != 0)) {
		reg0_val = readl(sticky_reg0);
		reg1_val = readl(sticky_reg1);
		ret = sprintf(buf, "sticky_reg0=0x%x, sticky_reg1=0x%x\n",
				reg0_val, reg1_val);
	} else {
		ret = sprintf(buf, "invalid sticky register\n");
	}

	return ret;
}

static ssize_t sticky_reg_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t size)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return size;

	if (val == 0) {
		if ((sticky_reg0 != 0) && (sticky_reg1 != 0)) {
			writel(0x0, sticky_reg0);
			writel(0x0, sticky_reg1);
		}
	}

	return size;
}

static struct class_attribute sticky_reg_class_attrs[] = {
	__ATTR(sticky_reg, 0664, sticky_reg_show, sticky_reg_store),
	__ATTR_NULL
};

static int aml_restart_probe(struct platform_device *pdev)
{
	u32 id;
	int ret;
	unsigned int reg;

	if (!of_property_read_u32(pdev->dev.of_node, "sys_reset", &id)) {
		psci_function_id_restart = id;
		arm_pm_restart = do_aml_restart;
	}

	if (!of_property_read_u32(pdev->dev.of_node, "sys_poweroff", &id)) {
		psci_function_id_poweroff = id;
		pm_power_off = do_aml_poweroff;
	}

	/* P_AO_RTI_STICKY_REG0 */
	if (!of_property_read_u32(pdev->dev.of_node, "sticky_reg0", &reg))
		sticky_reg0 = (unsigned int *)ioremap(reg, 4);

	/* P_AO_RTI_STICKY_REG1 */
	if (!of_property_read_u32(pdev->dev.of_node, "sticky_reg1", &reg))
		sticky_reg1 = (unsigned int *)ioremap(reg, 4);

	sticky_reg_cls = kzalloc(sizeof(struct class), GFP_KERNEL);
	if (sticky_reg_cls == NULL) {
		pr_err("%s no mem for zalloc\n", __func__);
		return -1;
	}

	sticky_reg_cls->name = sticky_reg_dev;
	sticky_reg_cls->class_attrs = sticky_reg_class_attrs;
	if (class_register(sticky_reg_cls) < 0) {
		pr_err("failed to class_reg for sticky reg\n");
		return -1;
	}

	ret = register_die_notifier(&panic_notifier);
	if (ret != 0) {
		pr_err("%s,register die notifier failed,ret =%d!\n",
			__func__, ret);
		return ret;
	}

	/* Register a call for panic conditions. */
	ret = atomic_notifier_chain_register(&panic_notifier_list,
			&panic_notifier);
	if (ret != 0) {
		pr_err("%s,register panic notifier failed,ret =%d!\n",
			__func__, ret);
		return ret;
	}
	return 0;
}

static const struct of_device_id of_aml_restart_match[] = {
	{ .compatible = "aml, reboot", },
	{ .compatible = "amlogic,reboot", },
	{},
};
MODULE_DEVICE_TABLE(of, of_aml_restart_match);

static struct platform_driver aml_restart_driver = {
	.probe = aml_restart_probe,
	.driver = {
		.name = "aml-restart",
		.of_match_table = of_match_ptr(of_aml_restart_match),
	},
};

static int __init aml_restart_init(void)
{
	return platform_driver_register(&aml_restart_driver);
}
device_initcall(aml_restart_init);
