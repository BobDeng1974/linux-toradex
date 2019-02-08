/*
 * mp.h - Musica Pristina ALSA SoC codec driver
 *
 * Copyright 2018 Welsh Technologies.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* default value of code I2C registers */
static const struct reg_default mp_codec_reg_defaults[] = {
		/*reg_address	reg_value*/
		{ 0x0000,		0x0000 },
};

/* MP codec private structure */
struct mp_codec_priv {
	int fmt;	/* i2s data format */
	struct regmap *regmap;
};

/* custom function to fetch info of PCM playback volume */
static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;
	return 0;
}

/*
 * custom function to get PCM playback volume
 */
static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	int l;
	int r;

	l = 100;
	r = 100;

	ucontrol->value.integer.value[0] = l;
	ucontrol->value.integer.value[1] = r;

	return 0;
}

/*
 * custom function to put PCM playback volume
 */
static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	int l;
	int r;

	l = ucontrol->value.integer.value[0];
	r = ucontrol->value.integer.value[1];

	return 0;
}

static const struct snd_kcontrol_new mp_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = dac_info_volsw,
		.get = dac_get_volsw,
		.put = dac_put_volsw,
	},
};

/* set codec format */
static int mp_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct mp_codec_priv *mp = snd_soc_codec_get_drvdata(codec);

	/* I2S clock and frame master setting. */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}

	/* setting I2S data format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	mp->fmt = fmt;
	return 0;
}

/*
 * set clock according to i2s frame clock.
 */
static int mp_set_clock(struct snd_soc_codec *codec, int frame_rate,
		int frame_width)
{
	switch (frame_rate) {
	case 44100:
	case 48000:
	case 64000:
	case 88200:
	case 96000:
	case 176400:
	case 192000:
	case 352800:
	case 384000:
	case 705600:
	case 768000:
		break;
	default:
		dev_err(codec->dev, "frame rate %d not supported\n",
			frame_rate);
		return -EINVAL;
	}
	switch (frame_width) {
	case 16:
	case 24:
	case 32:
		break;
	default:
		dev_err(codec->dev, "%-d-bit frame width not supported\n", frame_width);
		return -EINVAL;
	}
	snd_soc_write(codec, /*reg_address*/ 0x0000, /*reg_value*/ 0x0000);

	return 0;
}

/*
 * Set PCM DAI params.
 */
static int mp_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret;

	ret = mp_set_clock(codec, params_rate(params), params_width(params));
	if (ret)
		return ret;

	return 0;
}

static int mp_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_dbg(codec->dev, "Starting audio stream\n");
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev_dbg(codec->dev, "Stopping audio stream\n");
		break;
	default:
		break;
	}

	return 0;
}

static const unsigned int mp_rates[] = {
	44100, 48000, 64000, 88200, 96000, 176400, 192000, 352800, 384000,
	705600, 768000,
};

#define MP_CODEC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_pcm_hw_constraint_list mp_rate_constraints = {
	.count = ARRAY_SIZE(mp_rates),
	.list = mp_rates,
};

static int mp_codec_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai) {
		int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &mp_rate_constraints);

	return ret;
}

static const struct snd_soc_dai_ops mp_codec_ops = {
	.startup = mp_codec_startup,
	.hw_params = mp_pcm_hw_params,
	.set_fmt = mp_set_dai_fmt,
	.trigger = mp_dai_trigger,
};

static struct snd_soc_dai_driver mp_dai = {
	.name = "Musica Pristina",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = MP_CODEC_FORMATS,
	},
	.ops = &mp_codec_ops,
};

static int mp_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static int mp_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver mp_codec_driver = {
	.probe = mp_codec_probe,
	.remove = mp_codec_remove,
	.component_driver = {
		.controls		= mp_snd_controls,
		.num_controls	= ARRAY_SIZE(mp_snd_controls),
	},
};

static const struct regmap_config mp_codec_regmap = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 2,

	.max_register = 0xFFFF,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = mp_codec_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(mp_codec_reg_defaults),
};

static int mp_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct mp_codec_priv *mp;
	int ret;

	mp = devm_kzalloc(&client->dev, sizeof(*mp), GFP_KERNEL);
	if (!mp)
		return -ENOMEM;

	i2c_set_clientdata(client, mp);

	mp->regmap = devm_regmap_init_i2c(client, &mp_codec_regmap);
	if (IS_ERR(mp->regmap)) {
		ret = PTR_ERR(mp->regmap);
		dev_err(&client->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_codec(&client->dev, &mp_codec_driver, &mp_dai, 1);
	if (ret)
		return ret;

	return 0;
}

static int mp_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);

	return 0;
}

static const struct i2c_device_id mp_codec_id[] = {
	{"mp_codec", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mp_codec_id);

static const struct of_device_id mp_codec_dt_ids[] = {
	{ .compatible = "mp,codec-i2s", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mp_codec_dt_ids);

static struct i2c_driver mp_i2c_driver = {
	.driver = {
		   .name = "mp_codec",
		   .of_match_table = mp_codec_dt_ids,
		   },
	.probe = mp_i2c_probe,
	.remove = mp_i2c_remove,
	.id_table = mp_codec_id,
};

module_i2c_driver(mp_i2c_driver);

MODULE_DESCRIPTION("Musica Pristina ALSA SoC codec driver");
MODULE_AUTHOR("Francesco Lavra <francescolavra.fl@gmail.com>");
MODULE_LICENSE("GPL");
