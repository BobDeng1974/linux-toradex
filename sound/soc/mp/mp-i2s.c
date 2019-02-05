/*
 * Copyright (C) 2015 Andrea Venturi
 * Andrea Venturi <be17068@iperbole.bo.it>
 *
 * Copyright (C) 2016 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#define MP_I2S_CTRL_REG		0x00
#define MP_I2S_CTRL_SDO_EN_MASK		GENMASK(11, 8)
#define MP_I2S_CTRL_SDO_EN(sdo)			BIT(8 + (sdo))
#define MP_I2S_CTRL_MODE_MASK		BIT(5)
#define MP_I2S_CTRL_MODE_SLAVE			(1 << 5)
#define MP_I2S_CTRL_MODE_MASTER			(0 << 5)
#define MP_I2S_CTRL_TX_EN			BIT(2)
#define MP_I2S_CTRL_RX_EN			BIT(1)
#define MP_I2S_CTRL_GL_EN			BIT(0)

#define MP_I2S_FMT0_REG		0x04
#define MP_I2S_FMT0_LRCLK_POLARITY_MASK	BIT(7)
#define MP_I2S_FMT0_LRCLK_POLARITY_INVERTED		(1 << 7)
#define MP_I2S_FMT0_LRCLK_POLARITY_NORMAL		(0 << 7)
#define MP_I2S_FMT0_BCLK_POLARITY_MASK	BIT(6)
#define MP_I2S_FMT0_BCLK_POLARITY_INVERTED		(1 << 6)
#define MP_I2S_FMT0_BCLK_POLARITY_NORMAL		(0 << 6)
#define MP_I2S_FMT0_SR_MASK			GENMASK(5, 4)
#define MP_I2S_FMT0_SR(sr)				((sr) << 4)
#define MP_I2S_FMT0_WSS_MASK			GENMASK(3, 2)
#define MP_I2S_FMT0_WSS(wss)				((wss) << 2)
#define MP_I2S_FMT0_FMT_MASK			GENMASK(1, 0)
#define MP_I2S_FMT0_FMT_RIGHT_J			(2 << 0)
#define MP_I2S_FMT0_FMT_LEFT_J			(1 << 0)
#define MP_I2S_FMT0_FMT_I2S				(0 << 0)

#define MP_I2S_FMT1_REG		0x08
#define MP_I2S_FIFO_TX_REG		0x0c
#define MP_I2S_FIFO_RX_REG		0x10

#define MP_I2S_FIFO_CTRL_REG		0x14
#define MP_I2S_FIFO_CTRL_FLUSH_TX		BIT(25)
#define MP_I2S_FIFO_CTRL_FLUSH_RX		BIT(24)
#define MP_I2S_FIFO_CTRL_TX_MODE_MASK	BIT(2)
#define MP_I2S_FIFO_CTRL_TX_MODE(mode)		((mode) << 2)
#define MP_I2S_FIFO_CTRL_RX_MODE_MASK	GENMASK(1, 0)
#define MP_I2S_FIFO_CTRL_RX_MODE(mode)		(mode)

#define MP_I2S_FIFO_STA_REG		0x18

#define MP_I2S_DMA_INT_CTRL_REG	0x1c
#define MP_I2S_DMA_INT_CTRL_TX_DRQ_EN	BIT(7)
#define MP_I2S_DMA_INT_CTRL_RX_DRQ_EN	BIT(3)

#define MP_I2S_INT_STA_REG		0x20

#define MP_I2S_CLK_DIV_REG		0x24
#define MP_I2S_CLK_DIV_MCLK_EN		BIT(7)
#define MP_I2S_CLK_DIV_BCLK_MASK		GENMASK(6, 4)
#define MP_I2S_CLK_DIV_BCLK(bclk)			((bclk) << 4)
#define MP_I2S_CLK_DIV_MCLK_MASK		GENMASK(3, 0)
#define MP_I2S_CLK_DIV_MCLK(mclk)			((mclk) << 0)

#define MP_I2S_RX_CNT_REG		0x28
#define MP_I2S_TX_CNT_REG		0x2c

#define MP_I2S_TX_CHAN_SEL_REG	0x30
#define MP_I2S_TX_CHAN_SEL(num_chan)		(((num_chan) - 1) << 0)

#define MP_I2S_TX_CHAN_MAP_REG	0x34
#define MP_I2S_TX_CHAN_MAP(chan, sample)	((sample) << (chan << 2))

#define MP_I2S_RX_CHAN_SEL_REG	0x38
#define MP_I2S_RX_CHAN_MAP_REG	0x3c

struct MP_i2s {
	struct clk	*bus_clk;
	struct clk	*mod_clk;
	struct regmap	*regmap;
	struct reset_control *rst;

	struct snd_dmaengine_dai_dma_data	playback_dma_data;
};

struct MP_i2s_clk_div {
	u8	div;
	u8	val;
};

static const struct MP_i2s_clk_div MP_i2s_bclk_div[] = {
	{ .div = 2, .val = 0 },
	{ .div = 4, .val = 1 },
	{ .div = 6, .val = 2 },
	{ .div = 8, .val = 3 },
	{ .div = 12, .val = 4 },
	{ .div = 16, .val = 5 },
};

static const struct MP_i2s_clk_div MP_i2s_mclk_div[] = {
	{ .div = 1, .val = 0 },
	{ .div = 2, .val = 1 },
	{ .div = 4, .val = 2 },
	{ .div = 6, .val = 3 },
	{ .div = 8, .val = 4 },
	{ .div = 12, .val = 5 },
	{ .div = 16, .val = 6 },
	{ .div = 24, .val = 7 },
};

static int MP_i2s_get_bclk_div(struct MP_i2s *i2s,
				  unsigned int oversample_rate,
				  unsigned int word_size)
{
	int div = oversample_rate / word_size / 2;
	int i;

	for (i = 0; i < ARRAY_SIZE(MP_i2s_bclk_div); i++) {
		const struct MP_i2s_clk_div *bdiv = &MP_i2s_bclk_div[i];

		if (bdiv->div == div)
			return bdiv->val;
	}

	return -EINVAL;
}

static int MP_i2s_get_mclk_div(struct MP_i2s *i2s,
				  unsigned int oversample_rate,
				  unsigned int module_rate,
				  unsigned int sampling_rate)
{
	int div = module_rate / sampling_rate / oversample_rate;
	int i;

	for (i = 0; i < ARRAY_SIZE(MP_i2s_mclk_div); i++) {
		const struct MP_i2s_clk_div *mdiv = &MP_i2s_mclk_div[i];

		if (mdiv->div == div)
			return mdiv->val;
	}

	return -EINVAL;
}

static int MP_i2s_oversample_rates[] = { 128, 192, 256, 384, 512, 768 };

static int MP_i2s_set_clk_rate(struct MP_i2s *i2s,
				  unsigned int rate,
				  unsigned int word_size)
{
	unsigned int clk_rate;
	int bclk_div, mclk_div;
	int ret, i;

	switch (rate) {
	case 176400:
	case 88200:
	case 44100:
	case 22050:
	case 11025:
		clk_rate = 22579200;
		break;

	case 192000:
	case 128000:
	case 96000:
	case 64000:
	case 48000:
	case 32000:
	case 24000:
	case 16000:
	case 12000:
	case 8000:
		clk_rate = 24576000;
		break;

	default:
		return -EINVAL;
	}

	ret = clk_set_rate(i2s->mod_clk, clk_rate);
	if (ret)
		return ret;

	/* Always favor the highest oversampling rate */
	for (i = (ARRAY_SIZE(MP_i2s_oversample_rates) - 1); i >= 0; i--) {
		unsigned int oversample_rate = MP_i2s_oversample_rates[i];

		bclk_div = MP_i2s_get_bclk_div(i2s, oversample_rate,
						  word_size);
		mclk_div = MP_i2s_get_mclk_div(i2s, oversample_rate,
						  clk_rate,
						  rate);

		if ((bclk_div >= 0) && (mclk_div >= 0))
			break;
	}

	if ((bclk_div < 0) || (mclk_div < 0))
		return -EINVAL;

	regmap_write(i2s->regmap, MP_I2S_CLK_DIV_REG,
		     MP_I2S_CLK_DIV_BCLK(bclk_div) |
		     MP_I2S_CLK_DIV_MCLK(mclk_div) |
		     MP_I2S_CLK_DIV_MCLK_EN);

	return 0;
}

static int MP_i2s_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct MP_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int sr, wss;
	u32 width;

	if (params_channels(params) != 2)
		return -EINVAL;

	switch (params_physical_width(params)) {
	case 16:
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	default:
		return -EINVAL;
	}
	i2s->playback_dma_data.addr_width = width;

	switch (params_width(params)) {
	case 16:
		sr = 0;
		wss = 0;
		break;

	default:
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, MP_I2S_FMT0_REG,
			   MP_I2S_FMT0_WSS_MASK | MP_I2S_FMT0_SR_MASK,
			   MP_I2S_FMT0_WSS(wss) | MP_I2S_FMT0_SR(sr));

	return MP_i2s_set_clk_rate(i2s, params_rate(params),
				      params_width(params));
}

static int MP_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct MP_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val;

	/* DAI Mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		val = MP_I2S_FMT0_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = MP_I2S_FMT0_FMT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val = MP_I2S_FMT0_FMT_RIGHT_J;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, MP_I2S_FMT0_REG,
			   MP_I2S_FMT0_FMT_MASK,
			   val);

	/* DAI clock polarity */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		/* Invert both clocks */
		val = MP_I2S_FMT0_BCLK_POLARITY_INVERTED |
			MP_I2S_FMT0_LRCLK_POLARITY_INVERTED;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/* Invert bit clock */
		val = MP_I2S_FMT0_BCLK_POLARITY_INVERTED |
			MP_I2S_FMT0_LRCLK_POLARITY_NORMAL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/* Invert frame clock */
		val = MP_I2S_FMT0_LRCLK_POLARITY_INVERTED |
			MP_I2S_FMT0_BCLK_POLARITY_NORMAL;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		/* Nothing to do for both normal cases */
		val = MP_I2S_FMT0_BCLK_POLARITY_NORMAL |
			MP_I2S_FMT0_LRCLK_POLARITY_NORMAL;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, MP_I2S_FMT0_REG,
			   MP_I2S_FMT0_BCLK_POLARITY_MASK |
			   MP_I2S_FMT0_LRCLK_POLARITY_MASK,
			   val);

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* BCLK and LRCLK master */
		val = MP_I2S_CTRL_MODE_MASTER;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* BCLK and LRCLK slave */
		val = MP_I2S_CTRL_MODE_SLAVE;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, MP_I2S_CTRL_REG,
			   MP_I2S_CTRL_MODE_MASK,
			   val);

	/* Set significant bits in our FIFOs */
	regmap_update_bits(i2s->regmap, MP_I2S_FIFO_CTRL_REG,
			   MP_I2S_FIFO_CTRL_TX_MODE_MASK |
			   MP_I2S_FIFO_CTRL_RX_MODE_MASK,
			   MP_I2S_FIFO_CTRL_TX_MODE(1) |
			   MP_I2S_FIFO_CTRL_RX_MODE(1));
	return 0;
}

static void MP_i2s_start_playback(struct MP_i2s *i2s)
{
	/* Flush TX FIFO */
	regmap_update_bits(i2s->regmap, MP_I2S_FIFO_CTRL_REG,
			   MP_I2S_FIFO_CTRL_FLUSH_TX,
			   MP_I2S_FIFO_CTRL_FLUSH_TX);

	/* Clear TX counter */
	regmap_write(i2s->regmap, MP_I2S_TX_CNT_REG, 0);

	/* Enable TX Block */
	regmap_update_bits(i2s->regmap, MP_I2S_CTRL_REG,
			   MP_I2S_CTRL_TX_EN,
			   MP_I2S_CTRL_TX_EN);

	/* Enable TX DRQ */
	regmap_update_bits(i2s->regmap, MP_I2S_DMA_INT_CTRL_REG,
			   MP_I2S_DMA_INT_CTRL_TX_DRQ_EN,
			   MP_I2S_DMA_INT_CTRL_TX_DRQ_EN);
}


static void MP_i2s_stop_playback(struct MP_i2s *i2s)
{
	/* Disable TX Block */
	regmap_update_bits(i2s->regmap, MP_I2S_CTRL_REG,
			   MP_I2S_CTRL_TX_EN,
			   0);

	/* Disable TX DRQ */
	regmap_update_bits(i2s->regmap, MP_I2S_DMA_INT_CTRL_REG,
			   MP_I2S_DMA_INT_CTRL_TX_DRQ_EN,
			   0);
}

static int MP_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct MP_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			MP_i2s_start_playback(i2s);
		else
			return -EINVAL;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			MP_i2s_stop_playback(i2s);
		else
			return -EINVAL;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int MP_i2s_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct MP_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	/* Enable the whole hardware block */
	regmap_write(i2s->regmap, MP_I2S_CTRL_REG,
		     MP_I2S_CTRL_GL_EN);

	/* Enable the first output line */
	regmap_update_bits(i2s->regmap, MP_I2S_CTRL_REG,
			   MP_I2S_CTRL_SDO_EN_MASK,
			   MP_I2S_CTRL_SDO_EN(0));

	/* Enable the first two channels */
	regmap_write(i2s->regmap, MP_I2S_TX_CHAN_SEL_REG,
		     MP_I2S_TX_CHAN_SEL(2));

	/* Map them to the two first samples coming in */
	regmap_write(i2s->regmap, MP_I2S_TX_CHAN_MAP_REG,
		     MP_I2S_TX_CHAN_MAP(0, 0) | MP_I2S_TX_CHAN_MAP(1, 1));

	return clk_prepare_enable(i2s->mod_clk);
}

static void MP_i2s_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct MP_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	clk_disable_unprepare(i2s->mod_clk);

	/* Disable our output lines */
	regmap_update_bits(i2s->regmap, MP_I2S_CTRL_REG,
			   MP_I2S_CTRL_SDO_EN_MASK, 0);

	/* Disable the whole hardware block */
	regmap_write(i2s->regmap, MP_I2S_CTRL_REG, 0);
}

static const struct snd_soc_dai_ops MP_i2s_dai_ops = {
	.hw_params	= MP_i2s_hw_params,
	.set_fmt	= MP_i2s_set_fmt,
	.shutdown	= MP_i2s_shutdown,
	.startup	= MP_i2s_startup,
	.trigger	= MP_i2s_trigger,
};

static int MP_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct MP_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data, NULL);

	snd_soc_dai_set_drvdata(dai, i2s);

	return 0;
}

static struct snd_soc_dai_driver MP_i2s_dai = {
	.probe = MP_i2s_dai_probe,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &MP_i2s_dai_ops,
	.symmetric_rates = 1,
};

static const struct snd_soc_component_driver MP_i2s_component = {
	.name	= "MP-dai",
};

static bool MP_i2s_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MP_I2S_FIFO_TX_REG:
		return false;

	default:
		return true;
	}
}

static bool MP_i2s_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MP_I2S_FIFO_RX_REG:
	case MP_I2S_FIFO_STA_REG:
		return false;

	default:
		return true;
	}
}

static bool MP_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MP_I2S_FIFO_RX_REG:
	case MP_I2S_INT_STA_REG:
	case MP_I2S_RX_CNT_REG:
	case MP_I2S_TX_CNT_REG:
		return true;

	default:
		return false;
	}
}

static const struct reg_default MP_i2s_reg_defaults[] = {
	{ MP_I2S_CTRL_REG, 0x00000000 },
	{ MP_I2S_FMT0_REG, 0x0000000c },
	{ MP_I2S_FMT1_REG, 0x00004020 },
	{ MP_I2S_FIFO_CTRL_REG, 0x000400f0 },
	{ MP_I2S_DMA_INT_CTRL_REG, 0x00000000 },
	{ MP_I2S_CLK_DIV_REG, 0x00000000 },
	{ MP_I2S_TX_CHAN_SEL_REG, 0x00000001 },
	{ MP_I2S_TX_CHAN_MAP_REG, 0x76543210 },
	{ MP_I2S_RX_CHAN_SEL_REG, 0x00000001 },
	{ MP_I2S_RX_CHAN_MAP_REG, 0x00003210 },
};

static const struct regmap_config MP_i2s_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= MP_I2S_RX_CHAN_MAP_REG,

	.cache_type	= REGCACHE_FLAT,
	.reg_defaults	= MP_i2s_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(MP_i2s_reg_defaults),
	.writeable_reg	= MP_i2s_wr_reg,
	.readable_reg	= MP_i2s_rd_reg,
	.volatile_reg	= MP_i2s_volatile_reg,
};

static int MP_i2s_runtime_resume(struct device *dev)
{
	struct MP_i2s *i2s = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(i2s->bus_clk);
	if (ret) {
		dev_err(dev, "Failed to enable bus clock\n");
		return ret;
	}

	regcache_cache_only(i2s->regmap, false);
	regcache_mark_dirty(i2s->regmap);

	ret = regcache_sync(i2s->regmap);
	if (ret) {
		dev_err(dev, "Failed to sync regmap cache\n");
		goto err_disable_clk;
	}

	return 0;

err_disable_clk:
	clk_disable_unprepare(i2s->bus_clk);
	return ret;
}

static int MP_i2s_runtime_suspend(struct device *dev)
{
	struct MP_i2s *i2s = dev_get_drvdata(dev);

	regcache_cache_only(i2s->regmap, true);

	clk_disable_unprepare(i2s->bus_clk);

	return 0;
}

struct MP_i2s_quirks {
	bool has_reset;
};

static const struct MP_i2s_quirks MP_a10_i2s_quirks = {
	.has_reset	= false,
};

static const struct MP_i2s_quirks sun6i_a31_i2s_quirks = {
	.has_reset	= true,
};

static int MP_i2s_probe(struct platform_device *pdev)
{
	struct MP_i2s *i2s;
	const struct MP_i2s_quirks *quirks;
	struct resource *res;
	void __iomem *regs;
	int irq, ret;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2s);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Can't retrieve our interrupt\n");
		return irq;
	}

	quirks = of_device_get_match_data(&pdev->dev);
	if (!quirks) {
		dev_err(&pdev->dev, "Failed to determine the quirks to use\n");
		return -ENODEV;
	}

	i2s->bus_clk = devm_clk_get(&pdev->dev, "apb");
	if (IS_ERR(i2s->bus_clk)) {
		dev_err(&pdev->dev, "Can't get our bus clock\n");
		return PTR_ERR(i2s->bus_clk);
	}

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &MP_i2s_regmap_config);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev, "Regmap initialisation failed\n");
		return PTR_ERR(i2s->regmap);
	}

	i2s->mod_clk = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(i2s->mod_clk)) {
		dev_err(&pdev->dev, "Can't get our mod clock\n");
		return PTR_ERR(i2s->mod_clk);
	}

	if (quirks->has_reset) {
		i2s->rst = devm_reset_control_get(&pdev->dev, NULL);
		if (IS_ERR(i2s->rst)) {
			dev_err(&pdev->dev, "Failed to get reset control\n");
			return PTR_ERR(i2s->rst);
		}
	}

	if (!IS_ERR(i2s->rst)) {
		ret = reset_control_deassert(i2s->rst);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to deassert the reset control\n");
			return -EINVAL;
		}
	}

	i2s->playback_dma_data.addr = res->start + MP_I2S_FIFO_TX_REG;
	i2s->playback_dma_data.maxburst = 4;

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = MP_i2s_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &MP_i2s_component,
					      &MP_i2s_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI\n");
		goto err_suspend;
	}

	ret = snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM\n");
		goto err_suspend;
	}

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		MP_i2s_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	if (!IS_ERR(i2s->rst))
		reset_control_assert(i2s->rst);

	return ret;
}

static int MP_i2s_remove(struct platform_device *pdev)
{
	struct MP_i2s *i2s = dev_get_drvdata(&pdev->dev);

	snd_dmaengine_pcm_unregister(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		MP_i2s_runtime_suspend(&pdev->dev);

	if (!IS_ERR(i2s->rst))
		reset_control_assert(i2s->rst);

	return 0;
}

static const struct of_device_id MP_i2s_match[] = {
	{
		.compatible = "allwinner,MP-a10-i2s",
		.data = &MP_a10_i2s_quirks,
	},
	{
		.compatible = "allwinner,sun6i-a31-i2s",
		.data = &sun6i_a31_i2s_quirks,
	},
	{}
};
MODULE_DEVICE_TABLE(of, MP_i2s_match);

static const struct dev_pm_ops MP_i2s_pm_ops = {
	.runtime_resume		= MP_i2s_runtime_resume,
	.runtime_suspend	= MP_i2s_runtime_suspend,
};

static struct platform_driver MP_i2s_driver = {
	.probe	= MP_i2s_probe,
	.remove	= MP_i2s_remove,
	.driver	= {
		.name		= "MP-i2s",
		.of_match_table	= MP_i2s_match,
		.pm		= &MP_i2s_pm_ops,
	},
};
module_platform_driver(MP_i2s_driver);

MODULE_AUTHOR("Andrea Venturi <be17068@iperbole.bo.it>");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner A10 I2S driver");
MODULE_LICENSE("GPL");
