/*
 * tcs3472.c - Support for TAOS TCS3472 color light-to-digital converter
 *
 * Copyright (c) 2013 Peter Meerwald <pmeerw@pmeerw.net>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * Color light sensor with 16-bit channels for red, green, blue, clear);
 * 7-bit I2C slave address 0x39 (TCS34721, TCS34723) or 0x29 (TCS34725,
 * TCS34727)
 *
 * Includes auto-adjusting feature, where the driver manages gain and integration
 * time to keep the highest SNR possible without bottoming out.
 *
 * TODO: interrupt support, thresholds, wait time
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/pm.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>

#define TCS3472_DRV_NAME "tcs3472"

#define TCS3472_COMMAND BIT(7)
#define TCS3472_AUTO_INCR BIT(5)

#ifdef CONFIG_USE_TCS3400
#define TCS3472_SWITCH BIT(6)
#endif

#define TCS3472_ENABLE (TCS3472_COMMAND | 0x00)
#define TCS3472_ATIME (TCS3472_COMMAND | 0x01)
#define TCS3472_WTIME (TCS3472_COMMAND | 0x03)
#define TCS3472_AILT (TCS3472_COMMAND | 0x04)
#define TCS3472_AIHT (TCS3472_COMMAND | 0x06)
#define TCS3472_PERS (TCS3472_COMMAND | 0x0c)
#define TCS3472_CONFIG (TCS3472_COMMAND | 0x0d)
#define TCS3472_CONTROL (TCS3472_COMMAND | 0x0f)
#define TCS3472_ID (TCS3472_COMMAND | 0x12)
#define TCS3472_STATUS (TCS3472_COMMAND | 0x13)
#define TCS3472_CDATA (TCS3472_COMMAND | TCS3472_AUTO_INCR | 0x14)
#define TCS3472_RDATA (TCS3472_COMMAND | TCS3472_AUTO_INCR | 0x16)
#define TCS3472_GDATA (TCS3472_COMMAND | TCS3472_AUTO_INCR | 0x18)
#define TCS3472_BDATA (TCS3472_COMMAND | TCS3472_AUTO_INCR | 0x1a)

#ifdef CONFIG_USE_TCS3400
#define TCS3400_AUX (TCS3472_COMMAND | 0x10)
#define TCS3400_IR (TCS3472_COMMAND | TCS3472_SWITCH | 0x00)
#define TCS3400_CICLEAR 0xE6
#define TCS3400_AICLEAR 0xE6
#define TCS3400_NOT_AN_ADDRESS 0xE8

#define TCS3400_ASAT BIT(7)
#endif

#define TCS3472_STATUS_AVALID BIT(0)
#define TCS3472_ENABLE_AEN BIT(1)
#define TCS3472_ENABLE_PON BIT(0)
#define TCS3472_CONTROL_AGAIN_MASK (BIT(0) | BIT(1))

static struct sensor_settings {
	/* From datasheet: integration (ms) = (256 - atime) * 2.78125 */
	const int atime;
	const int gain;
	struct list_head list;
} auto_settings[5] = {
	{112,  1, LIST_HEAD_INIT(auto_settings[0].list)},
	{112,  4, LIST_HEAD_INIT(auto_settings[1].list)},
	{112, 16, LIST_HEAD_INIT(auto_settings[2].list)},
	{112, 64, LIST_HEAD_INIT(auto_settings[3].list)},
	{  0, 64, LIST_HEAD_INIT(auto_settings[4].list)}
};

struct tcs3472_data {
	struct i2c_client *client;
	u8 enable;
	u8 control;
	u8 atime;
#ifdef CONFIG_USE_TCS3400
	u8 ir;
	struct sensor_settings *current_settings;
#endif
	u16 buffer[8]; /* 4 16-bit channels + 64-bit timestamp */
};

#ifdef CONFIG_USE_TCS3400

#ifdef CONFIG_TCS3400_AUTO_ADJUST
/* Process channels so that clients don't have to know about current sensor settings */
#define CHANNEL_TYPE IIO_CHAN_INFO_PROCESSED
#else
#define CHANNEL_TYPE IIO_CHAN_INFO_RAW
#endif

#define TCS3472_CHANNEL(_color, _si, _addr) { \
	.type = IIO_INTENSITY, \
	.modified = 1, \
	.info_mask_separate = BIT(CHANNEL_TYPE), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_CALIBSCALE) | \
		BIT(IIO_CHAN_INFO_INT_TIME) | \
		BIT(IIO_CHAN_INFO_ENABLE), \
	.channel2 = IIO_MOD_LIGHT_##_color, \
	.address = _addr, \
	.scan_index = _si, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = 16, \
		.endianness = IIO_CPU, \
	}, \
}
#else
#define TCS3472_CHANNEL(_color, _si, _addr) { \
	.type = IIO_INTENSITY, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_CALIBSCALE) | \
		BIT(IIO_CHAN_INFO_INT_TIME), \
	.channel2 = IIO_MOD_LIGHT_##_color, \
	.address = _addr, \
	.scan_index = _si, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = 16, \
		.endianness = IIO_CPU, \
	}, \
}
#endif

#ifdef CONFIG_USE_TCS3400
static const int tcs3472_agains[] = { 1, 4, 16, 64 };

/* Precise change is 2.78125 ms, use round numbers for sysfs. */
static const int atime_step_change_usec = 2780;
#else
static const int tcs3472_agains[] = { 1, 4, 16, 60 };
static const int atime_step_change_usec = 2400;
#endif

static const struct iio_chan_spec tcs3472_channels[] = {
	TCS3472_CHANNEL(CLEAR, 0, TCS3472_CDATA),
	TCS3472_CHANNEL(RED, 1, TCS3472_RDATA),
	TCS3472_CHANNEL(GREEN, 2, TCS3472_GDATA),
	TCS3472_CHANNEL(BLUE, 3, TCS3472_BDATA),
#ifdef CONFIG_USE_TCS3400
	/* A "get-all" channel for TCS3400 */
	TCS3472_CHANNEL(RGBC, 4, TCS3400_NOT_AN_ADDRESS),
#endif /* CONFIG_USE_TCS3400 */
	IIO_CHAN_SOFT_TIMESTAMP(5),
};

static LIST_HEAD(settings_list);

static int atime_to_us(int atime)
{
	return (256 - atime) * atime_step_change_usec;
}

static int tcs3472_req_data(struct tcs3472_data *data)
{
	int tries = 50;
	int ret;

	while (tries--) {
		ret = i2c_smbus_read_byte_data(data->client, TCS3472_STATUS);
		if (ret < 0)
			return ret;
		if (ret & TCS3472_STATUS_AVALID)
			break;
		msleep(20);
	}

	if (tries < 0) {
		dev_err(&data->client->dev, "data not ready\n");
		return -EIO;
	}

	return 0;
}

static int tcs3472_apply_gain(struct tcs3472_data *data, int gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tcs3472_agains); i++) {
		if (gain == tcs3472_agains[i]) {
			data->control &= ~TCS3472_CONTROL_AGAIN_MASK;
			data->control |= i;
			return i2c_smbus_write_byte_data(data->client,
							 TCS3472_CONTROL,
							 data->control);
		}
	}
	return -EINVAL;
}

static int tcs3472_apply_atime(struct tcs3472_data *data, int atime)
{
	int ret;

	if (atime < 0 || atime > 255) {
		return -EINVAL;
	}

	ret = i2c_smbus_write_byte_data(data->client,
					TCS3472_ATIME, atime);
	if (ret < 0)
		return -EINVAL;
	data->atime = atime;
	return ret;
}

#ifdef CONFIG_USE_TCS3400
static void tcs3400_adjust_sensor(struct tcs3472_data *data,
				  int *vals, int vals_len);
static void tcs3400_process_values(struct sensor_settings *settings,
				   int *vals, int vals_len);

/* Highest gain and integration time are at the end of the list. */
static void initialize_settings_list(void)
{
	int i;

	if (!list_empty(&settings_list))
		return;

	for (i = 0; i < ARRAY_SIZE(auto_settings); i++)
		list_add_tail(&auto_settings[i].list, &settings_list);
}

static int tcs3400_apply_settings(struct tcs3472_data *data,
				  struct sensor_settings *settings)
{
	int ret;

	ret = tcs3472_apply_gain(data, settings->gain);
	if (ret < 0) {
		dev_err(&data->client->dev, "Failed to change gain to %d\n",
			settings->gain);
		return ret;
	}

	ret = tcs3472_apply_atime(data, settings->atime);
	if (ret < 0) {
		dev_err(&data->client->dev, "Failed to change atime to %d\n",
			settings->atime);
		return ret;
	}

	dev_info(&data->client->dev,
		 "ALS settings: integration_usec: %d , gain: %d\n",
		 atime_to_us(data->atime),
		 tcs3472_agains[data->control & TCS3472_CONTROL_AGAIN_MASK]);

	data->current_settings = settings;
	return ret;
}

static int tcs3400_read_multi(struct tcs3472_data *data,
			      struct iio_chan_spec const *chan,
			      int max_len, int *vals, int *vals_len)
{
	int ret, i, rgbc_index;
	struct sensor_settings initial_settings = *(data->current_settings);

	if (chan->channel2 == IIO_MOD_LIGHT_RGBC) {
		/* Get all channel values */
		*vals_len = min(max_len, 4);
		for (i = 0; i < *vals_len; ++i) {
			/* Rotate the CRGB access order for RGBC output. */
			rgbc_index = (i + *vals_len - 1) % *vals_len;

			vals[rgbc_index] = i2c_smbus_read_word_data(
				data->client, (TCS3472_CDATA + (i*2)) & (~(TCS3472_AUTO_INCR)));
			if (vals[rgbc_index] < 0)
				return vals[rgbc_index];
		}
		ret = IIO_VAL_INT_MULTIPLE;
	} else {
		*vals_len = 1;
		*vals = i2c_smbus_read_word_data(
			data->client, chan->address & (~(TCS3472_AUTO_INCR)));
		ret = IIO_VAL_INT;
	}

	/* Adjusting sensor may affect gain and integration time. */
	tcs3400_adjust_sensor(data, vals, *vals_len);
	tcs3400_process_values(&initial_settings, vals, *vals_len);

	return ret;
}

#ifdef CONFIG_TCS3400_AUTO_ADJUST
static int tcs3400_get_saturation_point(struct tcs3472_data *data)
{
	/* Calculation from TCS3400 data sheet */
	return min((256 - data->atime) * 1024, 65535);
}

static struct sensor_settings *get_adj_settings(struct sensor_settings
						     *current_settings,
						     bool next)
{
	if (next && !list_is_last(&current_settings->list, &settings_list)) {
		return list_next_entry(current_settings, list);
	} else if (!next && current_settings != list_first_entry(
			   &settings_list, struct sensor_settings, list)) {
		return list_prev_entry(current_settings, list);
	}
	return current_settings;
}

static int tcs3400_settings_change(struct tcs3472_data *data, bool up)
{
	struct sensor_settings *new_settings =
	    get_adj_settings(data->current_settings, up);

	if (new_settings == data->current_settings)
		return -1;

	return tcs3400_apply_settings(data, new_settings);
}

static int tcs3400_adjust_up(struct tcs3472_data *data)
{
	return tcs3400_settings_change(data, true);
}

static int tcs3400_adjust_down(struct tcs3472_data *data)
{
	return tcs3400_settings_change(data, false);
}

static int settings_step_change(struct sensor_settings *current_settings,
				struct sensor_settings *new_settings)
{
	int gain_change, atime_change;

	gain_change = DIV_ROUND_UP(new_settings->gain, current_settings->gain);
	atime_change = DIV_ROUND_UP(atime_to_us(new_settings->atime),
				    atime_to_us(current_settings->atime));
	return gain_change * atime_change;
}

/*
 * Checks if an adjustment is needed. Higher sensitivity gives improved SNR, so
 * we want to keep the settings as high as possible without saturating the
 * sensor. Settings are lowered if one of the channels is saturated. Settings
 * are upped whenever doing so won't saturate one of the channels.
 */
static void tcs3400_adjust_sensor(struct tcs3472_data *data,
				  int *vals, int vals_len)
{
	int i, saturation_value, gain_up_margin;
	int step_change;
	bool pull_gain_up = true;
	step_change = settings_step_change(
		data->current_settings,
		get_adj_settings(data->current_settings, true));

	saturation_value = tcs3400_get_saturation_point(data);

	/*
	 * This margin is to prevent revolving gain changes that occur when
	 * sensor_value is on the edge of a gain threshold.
	 */
	gain_up_margin = saturation_value / 10;

	for (i = 0; i < vals_len; ++i) {
		if (vals[i] >= saturation_value ||
		    i2c_smbus_read_byte_data(data->client, TCS3472_STATUS) & TCS3400_ASAT) {
			tcs3400_adjust_down(data);
			i2c_smbus_write_byte_data(data->client, TCS3400_CICLEAR, 1);
			return;
		} else if ((vals[i] * step_change) + gain_up_margin >= saturation_value) {
			pull_gain_up = false;
		}
	}

	if (pull_gain_up)
		tcs3400_adjust_up(data);
}

/*
 * Account for the gain and integration time, so the driver provides clients
 * with a single value that can be mapped to light changes.
 *
 * Less sensitive sensor settings (i.e. lower gain & integration-time),
 * will process the same rgbc results as higher values.
 *
 * Values will be multiplied by the ratio of the maximum settings : current settings
 */
static void tcs3400_process_values(struct sensor_settings *settings,
				   int *vals, int vals_len)
{
	int i, gain_bias;
	const int max_atime = 256;

	gain_bias = 64 / settings->gain;

	for (i = 0; i < vals_len; ++i) {
		vals[i] = DIV_ROUND_CLOSEST((vals[i] * gain_bias * max_atime),
					    (max_atime - settings->atime));
	}
}
#else
static void tcs3400_adjust_sensor(struct tcs3472_data *data,
				  int *vals, int vals_len) {};
static void tcs3400_process_values(struct sensor_settings *settings,
				   int *vals, int vals_len) {};
#endif /* CONFIG_TCS3400_AUTO_ADJUST */

#endif /* CONFIG_USE_TCS3400 */

static int tcs3472_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int max_len, int *vals, int *val_len, long mask)
{
	struct tcs3472_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = tcs3472_req_data(data);
		if (ret < 0) {
			iio_device_release_direct_mode(indio_dev);
			return ret;
		}
#ifdef CONFIG_USE_TCS3400
		ret = tcs3400_read_multi(data, chan, max_len, vals, val_len);
#else
		*vals = i2c_smbus_read_word_data(data->client, chan->address);
		ret = IIO_VAL_INT;
#endif
		iio_device_release_direct_mode(indio_dev);
		return ret;
	case IIO_CHAN_INFO_CALIBSCALE:
		*vals = tcs3472_agains[data->control &
			TCS3472_CONTROL_AGAIN_MASK];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_INT_TIME:
		vals[0] = 0;
		vals[1] = atime_to_us(data->atime);
		return IIO_VAL_INT_PLUS_MICRO;
#ifdef CONFIG_USE_TCS3400
	case IIO_CHAN_INFO_ENABLE:
		*vals = data->ir;
		*vals = *vals >> 7;
		return IIO_VAL_INT;
#endif
	}
	return -EINVAL;
}

static int tcs3472_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct tcs3472_data *data = iio_priv(indio_dev);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val2 != 0)
			return -EINVAL;
		return tcs3472_apply_gain(data, val);
	case IIO_CHAN_INFO_INT_TIME:
		if (val != 0)
			return -EINVAL;
		for (i = 0; i < 256; i++) {
			if (val2 == atime_to_us(i))
				return tcs3472_apply_atime(data, i);
		}
		return -EINVAL;
#ifdef CONFIG_USE_TCS3400
	case IIO_CHAN_INFO_ENABLE:
		if (val2 != 0)
			return -EINVAL;
		data->ir = val << 7;
		return i2c_smbus_write_byte_data(
			data->client, TCS3400_IR, data->ir);
#endif
	}
	return -EINVAL;
}

static irqreturn_t tcs3472_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct tcs3472_data *data = iio_priv(indio_dev);
	int i, j = 0;

	int ret = tcs3472_req_data(data);
	if (ret < 0)
		goto done;

	for_each_set_bit(i, indio_dev->active_scan_mask,
		indio_dev->masklength) {
		ret = i2c_smbus_read_word_data(data->client,
			TCS3472_CDATA + 2*i);
		if (ret < 0)
			goto done;

		data->buffer[j++] = ret;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data->buffer,
		iio_get_time_ns(indio_dev));

done:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static ssize_t tcs3472_show_int_time_available(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	size_t len = 0;
	int i;

	for (i = 1; i <= 256; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06d ",
				 atime_step_change_usec * i);

	/* replace trailing space by newline */
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEV_ATTR_INT_TIME_AVAIL(tcs3472_show_int_time_available);
#ifdef CONFIG_USE_TCS3400
static IIO_CONST_ATTR(calibscale_available, "1 4 16 64");
static IIO_CONST_ATTR(ir_available, "0 1");
#else
static IIO_CONST_ATTR(calibscale_available, "1 4 16 60");
#endif

static struct attribute *tcs3472_attributes[] = {
	&iio_const_attr_calibscale_available.dev_attr.attr,
	&iio_dev_attr_integration_time_available.dev_attr.attr,
#ifdef CONFIG_USE_TCS3400
	&iio_const_attr_ir_available.dev_attr.attr,
#endif
	NULL
};

static const struct attribute_group tcs3472_attribute_group = {
	.attrs = tcs3472_attributes,
};

static const struct iio_info tcs3472_info = {
	.read_raw_multi = tcs3472_read_raw,
	.write_raw = tcs3472_write_raw,
	.attrs = &tcs3472_attribute_group,
	.driver_module = THIS_MODULE,
};

static int tcs3472_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tcs3472_data *data;
	struct iio_dev *indio_dev;
	int ret, i2c_retries;
	const int max_i2c_retries = 4;
	dev_info(&client->dev, "TCS3472 probe starting");

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (indio_dev == NULL) {
		dev_info(&client->dev, "Device allocation failed\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &tcs3472_info;
	indio_dev->name = TCS3472_DRV_NAME;
	indio_dev->channels = tcs3472_channels;
	indio_dev->num_channels = ARRAY_SIZE(tcs3472_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	for (i2c_retries = 0; i2c_retries < max_i2c_retries; i2c_retries++) {
		ret = i2c_smbus_read_byte_data(data->client, TCS3472_ID);
#ifdef CONFIG_USE_TCS3400
		if (ret == 0x93 || ret == 0x90) {
			dev_info(&client->dev, "TCS34003 found\n");
			break;
		}
#else
		if (ret == 0x44) {
			dev_info(&client->dev, "TCS34721/34725 found\n");
			break;
		} else if (ret == 0x4d) {
			dev_info(&client->dev, "TCS34723/34727 found\n");
			break;
		}
#endif
	}

	if (i2c_retries >= max_i2c_retries) {
		if (ret < 0) {
			dev_err(&client->dev, "i2c read failed\n");
			return ret;
		}

		dev_err(&client->dev, "Device id %d not recognized\n", ret);
		return -ENODEV;
	}

	ret = i2c_smbus_read_byte_data(data->client, TCS3472_CONTROL);
	if (ret < 0)
		return ret;
	data->control = ret;

	ret = i2c_smbus_read_byte_data(data->client, TCS3472_ATIME);
	if (ret < 0)
		return ret;
	data->atime = ret;

#ifdef CONFIG_USE_TCS3400
	ret = i2c_smbus_read_byte_data(data->client, TCS3400_IR);
	if (ret < 0)
		return ret;
	data->ir = ret;

	initialize_settings_list();

	tcs3400_apply_settings(data, list_last_entry(&settings_list,
						     struct sensor_settings,
						     list));
#endif

	ret = i2c_smbus_read_byte_data(data->client, TCS3472_ENABLE);
	if (ret < 0)
		return ret;

	/* enable device */
	data->enable = ret | TCS3472_ENABLE_PON | TCS3472_ENABLE_AEN;
	ret = i2c_smbus_write_byte_data(data->client, TCS3472_ENABLE,
		data->enable);
	if (ret < 0)
		return ret;

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
		tcs3472_trigger_handler, NULL);
	if (ret < 0)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto buffer_cleanup;

	return 0;

buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
	return ret;
}

static int tcs3472_powerdown(struct tcs3472_data *data)
{
	return i2c_smbus_write_byte_data(data->client, TCS3472_ENABLE,
		data->enable & ~(TCS3472_ENABLE_AEN | TCS3472_ENABLE_PON));
}

static int tcs3472_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	tcs3472_powerdown(iio_priv(indio_dev));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tcs3472_suspend(struct device *dev)
{
	struct tcs3472_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));
	return tcs3472_powerdown(data);
}

static int tcs3472_resume(struct device *dev)
{
	struct tcs3472_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));
	return i2c_smbus_write_byte_data(data->client, TCS3472_ENABLE,
		data->enable | (TCS3472_ENABLE_AEN | TCS3472_ENABLE_PON));
}
#endif

static SIMPLE_DEV_PM_OPS(tcs3472_pm_ops, tcs3472_suspend, tcs3472_resume);

static const struct i2c_device_id tcs3472_id[] = {
	{ "tcs3472", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tcs3472_id);

static struct i2c_driver tcs3472_driver = {
	.driver = {
		.name	= TCS3472_DRV_NAME,
		.pm	= &tcs3472_pm_ops,
	},
	.probe		= tcs3472_probe,
	.remove		= tcs3472_remove,
	.id_table	= tcs3472_id,
};
module_i2c_driver(tcs3472_driver);

MODULE_AUTHOR("Peter Meerwald <pmeerw@pmeerw.net>");
MODULE_DESCRIPTION("TCS3472 color light sensors driver");
MODULE_LICENSE("GPL");
