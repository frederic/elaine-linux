/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
**
** File:
**     tas2770-codec.c
**
** Description:
**     ALSA SoC driver for TAS2770 20-W Digital Input Mono Class-D Audio Amplifier
**     with Speaker I/V Sense
**
** =============================================================================
*/

#ifdef CONFIG_TAS2770_CODEC
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas2770.h"


#define TAS2770_MDELAY 0xFFFFFFFE

static int tas2770_set_slot(struct snd_soc_codec *codec, int slot_width);

static unsigned int tas2770_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);
	int nResult = 0;
	unsigned int value = 0;

	mutex_lock(&pTAS2770->dev_lock);

	nResult = regmap_read(pTAS2770->regmap, reg, &value);

	if (nResult < 0)
		dev_err(pTAS2770->dev, "%s, ERROR, reg=0x%x, E=%d\n",
			__func__, reg, nResult);
	else
		dev_dbg(pTAS2770->dev, "%s, reg: 0x%x, value: 0x%x\n",
				__func__, reg, value);

	mutex_unlock(&pTAS2770->dev_lock);

	if (nResult >= 0)
		return value;
	else
		return nResult;
}


static int tas2770_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);

	int nResult = 0;

	mutex_lock(&pTAS2770->dev_lock);

	nResult = regmap_write(pTAS2770->regmap, reg, value);
	if (nResult < 0)
		dev_err(pTAS2770->dev, "%s, ERROR, reg=0x%x, E=%d\n",
			__func__, reg, nResult);
	else
		dev_dbg(pTAS2770->dev, "%s, reg: 0x%x, 0x%x\n",
			__func__, reg, value);

	mutex_unlock(&pTAS2770->dev_lock);

	return nResult;

}


static int tas2770_codec_suspend(struct snd_soc_codec *codec)
{
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&pTAS2770->codec_lock);

	dev_dbg(pTAS2770->dev, "%s\n", __func__);

	mutex_unlock(&pTAS2770->codec_lock);
	return ret;
}

static int tas2770_codec_resume(struct snd_soc_codec *codec)
{
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&pTAS2770->codec_lock);

	dev_dbg(pTAS2770->dev, "%s\n", __func__);

	mutex_unlock(&pTAS2770->codec_lock);
	return ret;
}

static const char * const tas2770_ASI1_src[] = {
	"I2C offset", "Left", "Right", "LeftRightDiv2",
};

static SOC_ENUM_SINGLE_DECL(
	tas2770_ASI1_src_enum, TAS2770_TDMConfigurationReg2,
	4, tas2770_ASI1_src);

static const struct snd_kcontrol_new tas2770_asi1_mux =
	SOC_DAPM_ENUM("ASI1 Source", tas2770_ASI1_src_enum);


static int tas2770_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, TAS2770_PowerControl,
			TAS2770_PowerControl_OperationalMode10_Mask,
			TAS2770_PowerControl_OperationalMode10_Mute);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, TAS2770_PowerControl,
			TAS2770_PowerControl_OperationalMode10_Mask,
			TAS2770_PowerControl_OperationalMode10_Shutdown);
		break;

	}
	return 0;

}

static const struct snd_kcontrol_new isense_switch =
	SOC_DAPM_SINGLE("Switch", TAS2770_PowerControl, 3, 1, 1);
static const struct snd_kcontrol_new vsense_switch =
	SOC_DAPM_SINGLE("Switch", TAS2770_PowerControl, 2, 1, 1);

static const struct snd_soc_dapm_widget tas2770_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MUX("ASI1 Sel", SND_SOC_NOPM, 0, 0,
				&tas2770_asi1_mux),
	SND_SOC_DAPM_SWITCH("ISENSE", TAS2770_PowerControl, 3, 1,
			&isense_switch),
	SND_SOC_DAPM_SWITCH("VSENSE", TAS2770_PowerControl, 2, 1,
			&vsense_switch),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0, tas2770_dac_event,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas2770_audio_map[] = {
	{"ASI1 Sel", "I2C offset", "ASI1"},
	{"ASI1 Sel", "Left", "ASI1"},
	{"ASI1 Sel", "Right", "ASI1"},
	{"ASI1 Sel", "LeftRightDiv2", "ASI1"},
	{"DAC", NULL, "ASI1 Sel"},
	{"OUT", NULL, "DAC"},
	{"ISENSE", "Switch", "IMON"},
	{"VSENSE", "Switch", "VMON"},
};


static int tas2770_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);
	dev_dbg(pTAS2770->dev, "%s\n", __func__);

	mutex_lock(&pTAS2770->codec_lock);
	if (mute) {
		snd_soc_update_bits(codec, TAS2770_PowerControl,
			TAS2770_PowerControl_OperationalMode10_Mask,
			TAS2770_PowerControl_OperationalMode10_Mute);
	} else {
		snd_soc_update_bits(codec, TAS2770_PowerControl,
			TAS2770_PowerControl_OperationalMode10_Mask,
			TAS2770_PowerControl_OperationalMode10_Active);
	}
	mutex_unlock(&pTAS2770->codec_lock);
	return 0;
}

static int tas2770_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int slot_width_tmp = 16;

	dev_dbg(pTAS2770->dev, "%s, format: %d\n", __func__,
		params_format(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(codec,
			TAS2770_TDMConfigurationReg2,
			TAS2770_TDMConfigurationReg2_RXWLEN32_Mask,
			TAS2770_TDMConfigurationReg2_RXWLEN32_16Bits);
			pTAS2770->mnCh_size = 16;
			if (pTAS2770->mnSlot_width == 0)
				slot_width_tmp = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
			snd_soc_update_bits(codec,
			TAS2770_TDMConfigurationReg2,
			TAS2770_TDMConfigurationReg2_RXWLEN32_Mask,
			TAS2770_TDMConfigurationReg2_RXWLEN32_24Bits);
			pTAS2770->mnCh_size = 24;
			if (pTAS2770->mnSlot_width == 0)
				slot_width_tmp = 32;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
			snd_soc_update_bits(codec,
			TAS2770_TDMConfigurationReg2,
			TAS2770_TDMConfigurationReg2_RXWLEN32_Mask,
			TAS2770_TDMConfigurationReg2_RXWLEN32_32Bits);
			pTAS2770->mnCh_size = 32;
			if (pTAS2770->mnSlot_width == 0)
				slot_width_tmp = 32;
		break;

	default:
		dev_dbg(pTAS2770->dev, "Not supported params format\n");
		return -EINVAL;
	}

	/* If machine driver did not call set slot width */
	if (pTAS2770->mnSlot_width == 0)
		tas2770_set_slot(codec, slot_width_tmp);

	dev_dbg(pTAS2770->dev, "mnCh_size: %d\n", pTAS2770->mnCh_size);
	snd_soc_update_bits(codec,
		TAS2770_TDMConfigurationReg5,
		TAS2770_TDMConfigurationReg5_VSNSTX_Mask |
		TAS2770_TDMConfigurationReg5_VSNSSLOT50_Mask,
		TAS2770_TDMConfigurationReg5_VSNSTX_Enable |
		pTAS2770->mnVmon_slot_no);
	snd_soc_update_bits(codec,
		TAS2770_TDMConfigurationReg6,
		TAS2770_TDMConfigurationReg6_ISNSTX_Mask |
		TAS2770_TDMConfigurationReg6_ISNSSLOT50_Mask,
		TAS2770_TDMConfigurationReg6_ISNSTX_Enable |
		pTAS2770->mnImon_slot_no);

	dev_dbg(pTAS2770->dev, "%s, sample rate: %d\n", __func__,
		params_rate(params));
	switch (params_rate(params)) {
	case 48000:
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_44_1_48kHz);
			break;
	case 44100:
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz);
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_44_1_48kHz);
			break;
	case 96000:
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_88_2_96kHz);
			break;
	case 19200:
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
			snd_soc_update_bits(codec,
				TAS2770_TDMConfigurationReg0,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2770_TDMConfigurationReg0_SAMPRATE31_176_4_192kHz);
			break;
	default:
			dev_dbg(pTAS2770->dev, "%s, unsupported sample rate\n", __func__);

	}
	return ret;
}

static int tas2770_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	u8 tdm_rx_start_slot = 0, asi_cfg_1 = 0;
	struct snd_soc_codec *codec = dai->codec;
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int value = 0;

	dev_dbg(pTAS2770->dev, "%s, format=0x%x\n", __func__, fmt);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		asi_cfg_1 = 0x00;
		break;
	default:
		dev_err(pTAS2770->dev, "ASI format master is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_dbg(pTAS2770->dev, "INV format: NBNF\n");
		asi_cfg_1 |= TAS2770_TDMConfigurationReg1_RXEDGE_Rising;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dev_dbg(pTAS2770->dev, "INV format: IBNF\n");
		asi_cfg_1 |= TAS2770_TDMConfigurationReg1_RXEDGE_Falling;
		break;
	default:
		dev_err(pTAS2770->dev, "ASI format Inverse is not found\n");
		ret = -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2770_TDMConfigurationReg1,
		TAS2770_TDMConfigurationReg1_RXEDGE_Mask,
		asi_cfg_1);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case (SND_SOC_DAIFMT_I2S):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_DSP_A):
	case (SND_SOC_DAIFMT_DSP_B):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_LEFT_J):
		tdm_rx_start_slot = 0;
		break;
	default:
	dev_err(pTAS2770->dev, "DAI Format is not found, fmt=0x%x\n", fmt);
	ret = -EINVAL;
		break;
	}

	snd_soc_update_bits(codec, TAS2770_TDMConfigurationReg1,
		TAS2770_TDMConfigurationReg1_RXOFFSET51_Mask,
	(tdm_rx_start_slot << TAS2770_TDMConfigurationReg1_RXOFFSET51_Shift));

	snd_soc_update_bits(codec, TAS2770_TDMConfigurationReg3,
		TAS2770_TDMConfigurationReg3_RXSLOTLeft30_Mask,
		(pTAS2770->mnLeftSlot << TAS2770_TDMConfigurationReg3_RXSLOTLeft30_Shift));
	snd_soc_update_bits(codec, TAS2770_TDMConfigurationReg3,
		TAS2770_TDMConfigurationReg3_RXSLOTRight74_Mask,
	(pTAS2770->mnRightSlot << TAS2770_TDMConfigurationReg3_RXSLOTRight74_Shift));

	value = snd_soc_read(codec, TAS2770_TDMConfigurationReg3);
	dev_dbg(pTAS2770->dev, "slot value: 0x%x", value);

	return ret;
}

static int tas2770_set_slot(struct snd_soc_codec *codec, int slot_width)
{
	int ret = 0;
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);

	switch (slot_width) {
	case 16:
	ret = snd_soc_update_bits(codec,
		TAS2770_TDMConfigurationReg2,
		TAS2770_TDMConfigurationReg2_RXSLEN10_Mask,
		TAS2770_TDMConfigurationReg2_RXSLEN10_16Bits);
	break;

	case 24:
	ret = snd_soc_update_bits(codec,
		TAS2770_TDMConfigurationReg2,
		TAS2770_TDMConfigurationReg2_RXSLEN10_Mask,
		TAS2770_TDMConfigurationReg2_RXSLEN10_24Bits);
	break;

	case 32:
	ret = snd_soc_update_bits(codec,
		TAS2770_TDMConfigurationReg2,
		TAS2770_TDMConfigurationReg2_RXSLEN10_Mask,
		TAS2770_TDMConfigurationReg2_RXSLEN10_32Bits);
	break;

	case 0:
	/* Do not change slot width */
	break;

	default:
		dev_dbg(pTAS2770->dev, "slot width not supported");
		ret = -EINVAL;
	}

	if (ret >= 0)
		pTAS2770->mnSlot_width = slot_width;

	return ret;
}

static int tas2770_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	int ret = 0;
	struct snd_soc_codec *codec = dai->codec;
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2770->dev, "%s, tx_mask:%d, rx_mask:%d, slots:%d, slot_width:%d",
			__func__, tx_mask, rx_mask, slots, slot_width);

	ret = tas2770_set_slot(codec, slot_width);

	return ret;
}

static struct snd_soc_dai_ops tas2770_dai_ops = {
	.digital_mute = tas2770_mute,
	.hw_params  = tas2770_hw_params,
	.set_fmt    = tas2770_set_dai_fmt,
	.set_tdm_slot = tas2770_set_dai_tdm_slot,
};

#define TAS2770_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define TAS2770_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
						SNDRV_PCM_RATE_96000 |\
						SNDRV_PCM_RATE_192000\
						)

static struct snd_soc_dai_driver tas2770_dai_driver[] = {
	{
		.name = "tas2770 ASI1",
		.id = 0,
		.playback = {
			.stream_name    = "ASI1 Playback",
			.channels_min   = 2,
			.channels_max   = 2,
			.rates      = TAS2770_RATES,
			.formats    = TAS2770_FORMATS,
		},
		.capture = {
			.stream_name    = "ASI1 Capture",
			.channels_min   = 0,
			.channels_max   = 2,
			.rates          = TAS2770_RATES,
			.formats    = TAS2770_FORMATS,
		},
		.ops = &tas2770_dai_ops,
		.symmetric_rates = 1,
	},
};

static int tas2770_codec_probe(struct snd_soc_codec *codec)
{
	struct tas2770_priv *pTAS2770 = snd_soc_codec_get_drvdata(codec);

	dev_err(pTAS2770->dev, "%s\n", __func__);
	snd_soc_codec_init_regmap(codec, pTAS2770->regmap);

	return 0;
}

static int tas2770_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static DECLARE_TLV_DB_SCALE(tas2770_digital_tlv, 1100, 50, 0);
static DECLARE_TLV_DB_SCALE(tas2770_playback_volume, -12750, 50, 0);

static const struct snd_kcontrol_new tas2770_snd_controls[] = {
	SOC_SINGLE_TLV("Amp Output Level", TAS2770_PlaybackConfigurationReg0,
		0, 0x14, 0,
		tas2770_digital_tlv),
	SOC_SINGLE_TLV("Playback Volume", TAS2770_PlaybackConfigurationReg2,
		0, TAS2770_PlaybackConfigurationReg2_DVCPCM70_Mask, 1,
		tas2770_playback_volume),
};

static struct snd_soc_codec_driver soc_codec_driver_tas2770 = {
	.probe			= tas2770_codec_probe,
	.remove			= tas2770_codec_remove,
	.read			= tas2770_codec_read,
	.write			= tas2770_codec_write,
	.suspend		= tas2770_codec_suspend,
	.resume			= tas2770_codec_resume,
	.component_driver = {
		.controls		= tas2770_snd_controls,
		.num_controls		= ARRAY_SIZE(tas2770_snd_controls),
		.dapm_widgets		= tas2770_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(tas2770_dapm_widgets),
		.dapm_routes		= tas2770_audio_map,
		.num_dapm_routes	= ARRAY_SIZE(tas2770_audio_map),
	},
};

int tas2770_register_codec(struct tas2770_priv *pTAS2770)
{
	int nResult = 0;

	dev_info(pTAS2770->dev, "%s, enter\n", __func__);
	nResult = snd_soc_register_codec(pTAS2770->dev,
		&soc_codec_driver_tas2770,
		tas2770_dai_driver, ARRAY_SIZE(tas2770_dai_driver));

	return nResult;
}

int tas2770_deregister_codec(struct tas2770_priv *pTAS2770)
{
	snd_soc_unregister_codec(pTAS2770->dev);

	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2770 ALSA SOC Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif /* CONFIG_TAS2770_CODEC */
