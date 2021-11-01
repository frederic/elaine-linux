/*
 * ALSA SoC Texas Instruments TAS2770 20-W Digital Input Mono Class-D
 * Audio Amplifier with Speaker I/V Sense
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *
 * Author: saiprasad
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#ifdef CONFIG_TAS2770_REGMAP

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/soc.h>

#include "tas2770.h"
#include "tas2770-codec.h"

static const struct reg_default tas2770_reg_defaults[] = {
	{ TAS2770_Page, 0x00 },
	{ TAS2770_SoftwareReset, 0x00 },
	{ TAS2770_PowerControl, 0x0e },
	{ TAS2770_PlaybackConfigurationReg0, 0x10 },
	{ TAS2770_PlaybackConfigurationReg1, 0x01 },
	{ TAS2770_PlaybackConfigurationReg2, 0x00 },
	{ TAS2770_MiscConfigurationReg0, 0x07 },
	{ TAS2770_TDMConfigurationReg1, 0x02 },
	{ TAS2770_TDMConfigurationReg2, 0x0a },
	{ TAS2770_TDMConfigurationReg3, 0x10 },
	{ TAS2770_InterruptMaskReg0, 0xfc },
	{ TAS2770_InterruptMaskReg1, 0xb1 },
	{ TAS2770_InterruptConfiguration, 0x05 },
	{ TAS2770_MiscIRQ, 0x81 },
	{ TAS2770_ClockConfiguration, 0x0c },

};

static bool tas2770_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS2770_Page: /* regmap implementation requires this */
	case TAS2770_SoftwareReset: /* always clears after write */
	case TAS2770_BrownOutPreventionReg0:/* has a self clearing bit */
	case TAS2770_LiveInterruptReg0:
	case TAS2770_LiveInterruptReg1:
	case TAS2770_LatchedInterruptReg0:/* Sticky interrupt flags */
	case TAS2770_LatchedInterruptReg1:/* Sticky interrupt flags */
	case TAS2770_VBATMSB:
	case TAS2770_VBATLSB:
	case TAS2770_TEMPMSB:
	case TAS2770_TEMPLSB:
		return true;
	}
	return false;
}

static bool tas2770_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS2770_LiveInterruptReg0:
	case TAS2770_LiveInterruptReg1:
	case TAS2770_LatchedInterruptReg0:
	case TAS2770_LatchedInterruptReg1:
	case TAS2770_VBATMSB:
	case TAS2770_VBATLSB:
	case TAS2770_TEMPMSB:
	case TAS2770_TEMPLSB:
	case TAS2770_TDMClockdetectionmonitor:
	case TAS2770_RevisionandPGID:
		return false;
	}
	return true;
}
static const struct regmap_config tas2770_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2770_writeable,
	.volatile_reg = tas2770_volatile,
	.reg_defaults = tas2770_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tas2770_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
	.max_register = 1 * 128,
};


static void tas2770_hw_reset(struct tas2770_priv *pTAS2770)
{
	if (gpio_is_valid(pTAS2770->mnResetGPIO)) {
		gpio_direction_output(pTAS2770->mnResetGPIO, 0);
		msleep(5);
		gpio_direction_output(pTAS2770->mnResetGPIO, 1);
		msleep(2);
	}

	pTAS2770->mnCurrentBook = -1;
	pTAS2770->mnCurrentPage = -1;
}

void tas2770_enableIRQ(struct tas2770_priv *pTAS2770, bool enable)
{
	if (enable) {
		if (pTAS2770->mbIRQEnable)
			return;

		if (gpio_is_valid(pTAS2770->mnIRQGPIO))
			enable_irq(pTAS2770->mnIRQ);

		schedule_delayed_work(&pTAS2770->irq_work, msecs_to_jiffies(10));
		pTAS2770->mbIRQEnable = true;
	} else {
		if (!pTAS2770->mbIRQEnable)
			return;

		if (gpio_is_valid(pTAS2770->mnIRQGPIO))
			disable_irq_nosync(pTAS2770->mnIRQ);
		pTAS2770->mbIRQEnable = false;
	}
}

static void irq_work_routine(struct work_struct *work)
{
	struct tas2770_priv *pTAS2770 =
		container_of(work, struct tas2770_priv, irq_work.work);

#ifdef CONFIG_TAS2770_CODEC
	mutex_lock(&pTAS2770->codec_lock);
#endif

	if (pTAS2770->mbRuntimeSuspend) {
		dev_info(pTAS2770->dev, "%s, Runtime Suspended\n", __func__);
		goto end;
	}

	if (!pTAS2770->mbPowerUp) {
		dev_info(pTAS2770->dev, "%s, device not powered\n", __func__);
		goto end;
	}

end:
#ifdef CONFIG_TAS2770_CODEC
	mutex_unlock(&pTAS2770->codec_lock);
#endif
}

static enum hrtimer_restart timer_func(struct hrtimer *timer)
{
	struct tas2770_priv *pTAS2770 = container_of(timer,
		struct tas2770_priv, mtimer);

	if (pTAS2770->mbPowerUp) {
		if (!delayed_work_pending(&pTAS2770->irq_work))
			schedule_delayed_work(&pTAS2770->irq_work,
				msecs_to_jiffies(20));
	}

	return HRTIMER_NORESTART;
}

static irqreturn_t tas2770_irq_handler(int irq, void *dev_id)
{
	struct tas2770_priv *pTAS2770 = (struct tas2770_priv *)dev_id;

	tas2770_enableIRQ(pTAS2770, false);

	/* get IRQ status after 100 ms */
	if (!delayed_work_pending(&pTAS2770->irq_work))
		schedule_delayed_work(&pTAS2770->irq_work,
			msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static int tas2770_runtime_suspend(struct tas2770_priv *pTAS2770)
{
	dev_dbg(pTAS2770->dev, "%s\n", __func__);

	pTAS2770->mbRuntimeSuspend = true;

	if (hrtimer_active(&pTAS2770->mtimer)) {
		dev_dbg(pTAS2770->dev, "cancel die temp timer\n");
		hrtimer_cancel(&pTAS2770->mtimer);
	}

	if (delayed_work_pending(&pTAS2770->irq_work)) {
		dev_dbg(pTAS2770->dev, "cancel IRQ work\n");
		cancel_delayed_work_sync(&pTAS2770->irq_work);
	}

	return 0;
}

#define CHECK_PERIOD	5000	/* 5 second */
static int tas2770_runtime_resume(struct tas2770_priv *pTAS2770)
{
	dev_dbg(pTAS2770->dev, "%s\n", __func__);

	if (pTAS2770->mbPowerUp) {
		if (!hrtimer_active(&pTAS2770->mtimer)) {
			hrtimer_start(&pTAS2770->mtimer,
				ns_to_ktime((u64)CHECK_PERIOD * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
		}

	}

	pTAS2770->mbRuntimeSuspend = false;

	return 0;
}

static int tas2770_parse_dt(struct device *dev, struct tas2770_priv *pTAS2770)
{
	struct device_node *np = dev->of_node;
	int rc = 0, ret = 0;

	rc = of_property_read_u32(np, "ti,asi-format", &pTAS2770->mnASIFormat);
	if (rc) {
		dev_err(pTAS2770->dev, "Looking up %s property in node %s failed %d\n",
			"ti,asi-format", np->full_name, rc);
	} else {
		dev_dbg(pTAS2770->dev, "ti,asi-format=%d",
			pTAS2770->mnASIFormat);
	}

	pTAS2770->mnResetGPIO = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (!gpio_is_valid(pTAS2770->mnResetGPIO)) {
		dev_err(pTAS2770->dev, "Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, pTAS2770->mnResetGPIO);
	} else {
		dev_dbg(pTAS2770->dev, "ti,reset-gpio=%d",
			pTAS2770->mnResetGPIO);
	}

	pTAS2770->mnIRQGPIO = of_get_named_gpio(np, "ti,irq-gpio", 0);
	if (!gpio_is_valid(pTAS2770->mnIRQGPIO)) {
		dev_err(pTAS2770->dev, "Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio", np->full_name, pTAS2770->mnIRQGPIO);
	} else {
		dev_dbg(pTAS2770->dev, "ti,irq-gpio=%d", pTAS2770->mnIRQGPIO);
	}

	of_property_read_u32(np, "ti,left-slot", &pTAS2770->mnLeftSlot);
	if (rc) {
		dev_err(pTAS2770->dev, "Looking up %s property in node %s failed %d\n",
			"ti,left-slot", np->full_name, rc);
	} else {
		dev_dbg(pTAS2770->dev, "ti,left-slot=%d",
			pTAS2770->mnLeftSlot);
	}

	of_property_read_u32(np, "ti,right-slot", &pTAS2770->mnRightSlot);
	if (rc) {
		dev_err(pTAS2770->dev, "Looking up %s property in node %s failed %d\n",
			"ti,right-slot", np->full_name, rc);
	} else {
		dev_dbg(pTAS2770->dev, "ti,right-slot=%d",
			pTAS2770->mnRightSlot);
	}

	of_property_read_u32(np, "ti,imon-slot-no", &pTAS2770->mnImon_slot_no);
	if (rc) {
		dev_err(pTAS2770->dev, "Looking up %s property in node %s failed %d\n",
			"ti,imon-slot-no", np->full_name, rc);
	} else {
		dev_dbg(pTAS2770->dev, "ti,imon-slot-no=%d",
			pTAS2770->mnImon_slot_no);
	}

	of_property_read_u32(np, "ti,vmon-slot-no", &pTAS2770->mnVmon_slot_no);
	if (rc) {
		dev_err(pTAS2770->dev, "Looking up %s property in node %s failed %d\n",
			"ti,vmon-slot-no", np->full_name, rc);
	} else {
		dev_dbg(pTAS2770->dev, "ti,vmon-slot-no=%d",
			pTAS2770->mnVmon_slot_no);
	}

	return ret;
}

static int tas2770_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tas2770_priv *pTAS2770;
	int nResult;

	dev_info(&client->dev, "%s enter\n", __func__);

	pTAS2770 = devm_kzalloc(&client->dev,
		sizeof(struct tas2770_priv), GFP_KERNEL);
	if (pTAS2770 == NULL) {
		nResult = -ENOMEM;
		goto end;
	}

	pTAS2770->dev = &client->dev;
	i2c_set_clientdata(client, pTAS2770);
	dev_set_drvdata(&client->dev, pTAS2770);

	pTAS2770->regmap = devm_regmap_init_i2c(client, &tas2770_i2c_regmap);
	if (IS_ERR(pTAS2770->regmap)) {
		nResult = PTR_ERR(pTAS2770->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
					nResult);
		goto end;
	}

	if (client->dev.of_node)
		tas2770_parse_dt(&client->dev, pTAS2770);

	if (gpio_is_valid(pTAS2770->mnResetGPIO)) {
		nResult = gpio_request(pTAS2770->mnResetGPIO, "TAS2770_RESET");
		if (nResult) {
			dev_err(pTAS2770->dev, "%s: Failed to request gpio %d\n",
				__func__, pTAS2770->mnResetGPIO);
			nResult = -EINVAL;
			goto free_gpio;
		}
	}

	if (gpio_is_valid(pTAS2770->mnIRQGPIO)) {
		nResult = gpio_request(pTAS2770->mnIRQGPIO, "TAS2770-IRQ");
		if (nResult < 0) {
			dev_err(pTAS2770->dev, "%s: GPIO %d request error\n",
				__func__, pTAS2770->mnIRQGPIO);
			goto free_gpio;
		}
		gpio_direction_input(pTAS2770->mnIRQGPIO);
		pTAS2770->mnIRQ = gpio_to_irq(pTAS2770->mnIRQGPIO);
		dev_dbg(pTAS2770->dev, "irq = %d\n", pTAS2770->mnIRQ);
		nResult = request_threaded_irq(pTAS2770->mnIRQ,
					tas2770_irq_handler, NULL,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					client->name, pTAS2770);
		if (nResult < 0) {
			dev_err(pTAS2770->dev,
				"request_irq failed, %d\n", nResult);
			goto free_gpio;
		}
		disable_irq_nosync(pTAS2770->mnIRQ);
		INIT_DELAYED_WORK(&pTAS2770->irq_work, irq_work_routine);
	}

	pTAS2770->hw_reset = tas2770_hw_reset;
	pTAS2770->enableIRQ = tas2770_enableIRQ;
	pTAS2770->runtime_suspend = tas2770_runtime_suspend;
	pTAS2770->runtime_resume = tas2770_runtime_resume;
	pTAS2770->mnCh_size = 0;
	pTAS2770->mnSlot_width = 0;

	tas2770_hw_reset(pTAS2770);
	regmap_write(pTAS2770->regmap, TAS2770_SoftwareReset,
			TAS2770_SoftwareReset_SoftwareReset_Reset);

	mutex_init(&pTAS2770->dev_lock);
	if (nResult < 0)
		goto destroy_mutex;

#ifdef CONFIG_TAS2770_CODEC
	mutex_init(&pTAS2770->codec_lock);
	tas2770_register_codec(pTAS2770);
#endif

	hrtimer_init(&pTAS2770->mtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pTAS2770->mtimer.function = timer_func;

destroy_mutex:
	if (nResult < 0)
		mutex_destroy(&pTAS2770->dev_lock);

free_gpio:
	if (nResult < 0) {
		if (gpio_is_valid(pTAS2770->mnResetGPIO))
			gpio_free(pTAS2770->mnResetGPIO);
		if (gpio_is_valid(pTAS2770->mnIRQGPIO))
			gpio_free(pTAS2770->mnIRQGPIO);
	}

end:
	return nResult;
}

static int tas2770_i2c_remove(struct i2c_client *client)
{
	struct tas2770_priv *pTAS2770 = i2c_get_clientdata(client);

	dev_info(pTAS2770->dev, "%s\n", __func__);

#ifdef CONFIG_TAS2770_CODEC
	tas2770_deregister_codec(pTAS2770);
	mutex_destroy(&pTAS2770->codec_lock);
#endif

	if (gpio_is_valid(pTAS2770->mnResetGPIO))
		gpio_free(pTAS2770->mnResetGPIO);
	if (gpio_is_valid(pTAS2770->mnIRQGPIO))
		gpio_free(pTAS2770->mnIRQGPIO);

	return 0;
}


static const struct i2c_device_id tas2770_i2c_id[] = {
	{ "tas2770", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2770_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2770_of_match[] = {
	{ .compatible = "ti,tas2770" },
	{},
};
MODULE_DEVICE_TABLE(of, tas2770_of_match);
#endif


static struct i2c_driver tas2770_i2c_driver = {
	.driver = {
		.name   = "tas2770",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(tas2770_of_match),
#endif
	},
	.probe      = tas2770_i2c_probe,
	.remove     = tas2770_i2c_remove,
	.id_table   = tas2770_i2c_id,
};

module_i2c_driver(tas2770_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2770 I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif
