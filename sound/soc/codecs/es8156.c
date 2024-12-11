// SPDX-License-Identifier: GPL-2.0
/*
 * es8156.c -- es8156 ALSA SoC audio driver
 * Copyright Everest Semiconductor Co.,Ltd
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include "es8156.h"

#define INVALID_GPIO -1
#define GPIO_LOW  0
#define GPIO_HIGH 1
#define es8156_DEF_VOL 0xBF

//static struct i2c_client *i2c_clt;

//#define KERNEL_4_9_XX
//#define HP_DET_FUNTION
//static struct snd_soc_codec *es8156_codec;

static const struct reg_default es8156_reg_defaults[] = {
	{0x00, 0x1c}, {0x01, 0x20}, {0x02, 0x00}, {0x03, 0x01},
	{0x04, 0x00}, {0x05, 0x04}, {0x06, 0x11}, {0x07, 0x00},
	{0x08, 0x06}, {0x09, 0x00}, {0x0a, 0x50}, {0x0b, 0x50},
	{0x0c, 0x00}, {0x0d, 0x10}, {0x10, 0x40}, {0x10, 0x40},
	{0x11, 0x00}, {0x12, 0x04}, {0x13, 0x11}, {0x14, 0xbf},
	{0x15, 0x00}, {0x16, 0x00}, {0x17, 0xf7}, {0x18, 0x00},
	{0x19, 0x20}, {0x1a, 0x00}, {0x20, 0x16}, {0x21, 0x7f},
	{0x22, 0x00}, {0x23, 0x86}, {0x24, 0x00}, {0x25, 0x07},
	{0xfc, 0x00}, {0xfd, 0x81}, {0xfe, 0x55}, {0xff, 0x10},
};

/* codec private data */
struct es8156_priv {
	struct regmap *regmap;
	unsigned int dmic_amic;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	struct clk *mclk;
	int debounce_time;
	int hp_det_invert;
	struct delayed_work work;

	int spk_ctl_gpio;
	int hp_det_gpio;
	bool muted;
	bool hp_inserted;
	bool spk_active_level;

	int pwr_count;
};

/*
 * es8156_reset
 */
static int es8156_reset(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8156_RESET_REG00, 0x1c);
	usleep_range(5000, 5500);
	return snd_soc_component_write(component, ES8156_RESET_REG00, 0x03);
}

static void es8156_enable_spk(struct es8156_priv *es8156, bool enable)
{
	bool level;

	level = enable ? es8156->spk_active_level : !es8156->spk_active_level;
	gpio_set_value(es8156->spk_ctl_gpio, level);
}

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -9600, 50, 1);

static const struct snd_kcontrol_new es8156_snd_controls[] = {
	/* DAC Digital controls */
	SOC_SINGLE_TLV("DAC Playback Volume", ES8156_VOLUME_CONTROL_REG14,
			  0, 0xff, 1, dac_vol_tlv),

};

static const struct snd_soc_dapm_widget es8156_dapm_widgets[] = {
	/*
	 *DAC
	 */
	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACR", NULL, SND_SOC_NOPM, 0, 0),
	/*
	 *Output Lines
	 */
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
};

static const struct snd_soc_dapm_route es8156_dapm_routes[] = {
	/*
	 * dac route map
	 */
	{ "DACL",  NULL, "Playback" },
	{ "DACR",  NULL, "Playback" },

	{ "LOUT", NULL, "DACL" },
	{ "ROUT", NULL, "DACR" },
};

/*
 * Note that this should be called from init rather than from hw_params.
 */
#if 0
static int es8156_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	return 0;
}
#endif

static int es8156_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	///struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_component *component = codec_dai->component;

	//iface = snd_soc_component_read(component, ES8156_SCLK_MODE_REG02);
	//adciface = snd_soc_component_read(component, ES8323_ADC_IFACE);
	//daciface = snd_soc_component_read(component, ES8156_DAC_SDP_REG11);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM://es8156 master
		snd_soc_component_update_bits(component, ES8156_SCLK_MODE_REG02, 0x01, 0x01);
		break;
	case SND_SOC_DAIFMT_CBS_CFS://es8156 slave
		snd_soc_component_update_bits(component, ES8156_SCLK_MODE_REG02, 0x01, 0x00);
		break;
	default:
		return -EINVAL;
	}
	/* interface format */

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x07, 0x00);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		return -EINVAL;
	case SND_SOC_DAIFMT_LEFT_J:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x07, 0x01);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x07, 0x03);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x07, 0x07);
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		snd_soc_component_update_bits(component, ES8156_SCLK_MODE_REG02, 0x01, 0x00);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		snd_soc_component_update_bits(component, ES8156_SCLK_MODE_REG02, 0x01, 0x01);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		snd_soc_component_update_bits(component, ES8156_SCLK_MODE_REG02, 0x01, 0x01);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		snd_soc_component_update_bits(component, ES8156_SCLK_MODE_REG02, 0x01, 0x00);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int es8156_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	//struct snd_soc_codec *codec = dai->codec;
	struct snd_soc_component *component = dai->component;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x70, 0x30);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x70, 0x10);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x70, 0x00);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		snd_soc_component_update_bits(component, ES8156_DAC_SDP_REG11, 0x70, 0x40);
		break;
	}
	return 0;
}

static int es8156_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	//struct snd_soc_codec *codec = dai->codec;
	//struct es8156_priv *es8156 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_component *component = dai->component;
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(component);

	es8156->muted = mute;
	if (mute) {
		es8156_enable_spk(es8156, false);
		msleep(100);
		snd_soc_component_update_bits(component, ES8156_DAC_MUTE_REG13, 0x08, 0x08);
	} else { //else if (dai->playback_active) {
		snd_soc_component_update_bits(component, ES8156_DAC_MUTE_REG13, 0x08, 0x00);
		if (!es8156->hp_inserted)
			es8156_enable_spk(es8156, true);
	}
	return 0;
}


static int es8156_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	int ret;
	//struct es8156_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct es8156_priv *priv = snd_soc_component_get_drvdata(component);

	printk("%s\n", __func__);

	switch (level)
	{
		case SND_SOC_BIAS_ON:
			break;

		case SND_SOC_BIAS_PREPARE:
			break;

		case SND_SOC_BIAS_STANDBY:
		/*
		 *open i2s clock
		 */
		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_OFF) {
			if (!IS_ERR(priv->mclk)) {
				ret = clk_prepare_enable(priv->mclk);
				snd_soc_component_write(component, ES8156_SCLK_MODE_REG02, 0x04);
				snd_soc_component_write(component, ES8156_ANALOG_SYS1_REG20, 0x2A);
				snd_soc_component_write(component, ES8156_ANALOG_SYS2_REG21, 0x3C);
				snd_soc_component_write(component, ES8156_ANALOG_SYS3_REG22, 0x08);
				snd_soc_component_write(component, ES8156_ANALOG_LP_REG24, 0x07);
				snd_soc_component_write(component, ES8156_ANALOG_SYS4_REG23, 0x00);
				snd_soc_component_write(component, ES8156_TIME_CONTROL1_REG0A, 0x01);
				snd_soc_component_write(component, ES8156_TIME_CONTROL2_REG0B, 0x01);
				snd_soc_component_write(component, ES8156_VOLUME_CONTROL_REG14, 0xBF);
				snd_soc_component_write(component, ES8156_MAINCLOCK_CTL_REG01, 0x21);
				snd_soc_component_write(component, ES8156_P2S_CONTROL_REG0D, 0x14);
				snd_soc_component_write(component, ES8156_MISC_CONTROL3_REG18, 0x00);
				snd_soc_component_write(component, ES8156_CLOCK_ON_OFF_REG08, 0x3F);
				snd_soc_component_write(component, ES8156_RESET_REG00, 0x02);
				snd_soc_component_write(component, ES8156_RESET_REG00, 0x03);
				snd_soc_component_write(component, ES8156_ANALOG_SYS5_REG25, 0x20);
				if (ret) {
					dev_err(component->dev,
						"Failed to enable master clock: %d\n",
						ret);
					//return ret;
				}
			}
		}
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_component_write(component,  ES8156_VOLUME_CONTROL_REG14, 0xBF);
		snd_soc_component_write(component,  ES8156_EQ_CONTROL1_REG19, 0x02);
		snd_soc_component_write(component,  ES8156_ANALOG_SYS2_REG21, 0x1F);
		snd_soc_component_write(component,  ES8156_ANALOG_SYS3_REG22, 0x02);
		snd_soc_component_write(component,  ES8156_ANALOG_SYS5_REG25, 0x21);
		snd_soc_component_write(component,  ES8156_ANALOG_SYS5_REG25, 0x01);
		snd_soc_component_write(component,  ES8156_ANALOG_SYS5_REG25, 0x87);
		snd_soc_component_write(component,  ES8156_MISC_CONTROL3_REG18, 0x01);
		snd_soc_component_write(component,  ES8156_MISC_CONTROL2_REG09, 0x02);
		snd_soc_component_write(component,  ES8156_MISC_CONTROL2_REG09, 0x01);
		snd_soc_component_write(component,  ES8156_CLOCK_ON_OFF_REG08, 0x00);
		/*
		 *close i2s clock
		 */
		if (!IS_ERR(priv->mclk))
			clk_disable_unprepare(priv->mclk);
		break;
	}
	return 0;
}

#define es8156_RATES SNDRV_PCM_RATE_8000_96000

#define es8156_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FORMAT_S32_LE)

static struct snd_soc_dai_ops es8156_ops = {
	.startup = NULL,
	.hw_params = es8156_pcm_hw_params,
	.set_fmt = es8156_set_dai_fmt,
	.set_sysclk = NULL,
	.mute_stream = es8156_mute,
	.shutdown = NULL,
};

static struct snd_soc_dai_driver es8156_dai = {
	.name = "ES8156 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = es8156_RATES,
		.formats = es8156_FORMATS,
	},
	.ops = &es8156_ops,
	.symmetric_rates = 1,
};


static int es8156_init_regs(struct snd_soc_component *component)
{
	/*
	 *set clock and analog power
	 */
	snd_soc_component_write(component, ES8156_SCLK_MODE_REG02, 0x04);
	snd_soc_component_write(component, ES8156_ANALOG_SYS1_REG20, 0x2A);
	snd_soc_component_write(component, ES8156_ANALOG_SYS2_REG21, 0x3C);
	snd_soc_component_write(component, ES8156_ANALOG_SYS3_REG22, 0x08);
	snd_soc_component_write(component, ES8156_ANALOG_LP_REG24, 0x07);
	snd_soc_component_write(component, ES8156_ANALOG_SYS4_REG23, 0x00);
	/*
	 *set powerup time
	 */
	snd_soc_component_write(component, ES8156_TIME_CONTROL1_REG0A, 0x01);
	snd_soc_component_write(component, ES8156_TIME_CONTROL2_REG0B, 0x01);
	/*
	 *set digtal volume
	 */
	snd_soc_component_write(component, ES8156_VOLUME_CONTROL_REG14, 0xBF);
	/*
	 *set MCLK
	 */
	snd_soc_component_write(component, ES8156_MAINCLOCK_CTL_REG01, 0x21);
	snd_soc_component_write(component, ES8156_P2S_CONTROL_REG0D, 0x14);
	snd_soc_component_write(component, ES8156_MISC_CONTROL3_REG18, 0x00);
	snd_soc_component_write(component, ES8156_CLOCK_ON_OFF_REG08, 0x3F);
	snd_soc_component_write(component, ES8156_RESET_REG00, 0x02);
	snd_soc_component_write(component, ES8156_RESET_REG00, 0x03);
	snd_soc_component_write(component, ES8156_ANALOG_SYS5_REG25, 0x20);

	return 0;
}

static int es8156_suspend(struct snd_soc_component *component)
{
	es8156_set_bias_level(component, SND_SOC_BIAS_OFF);
	return 0;
}

static int es8156_resume(struct snd_soc_component *component)
{
	return 0;
}

static int es8156_probe(struct snd_soc_component *component)
{
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(component);
	int ret = 0;
	//es8156_codec = codec;
	printk("%s start\n", __func__);
	es8156->mclk = devm_clk_get(component->dev, "mclk");
	if (PTR_ERR(es8156->mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	printk("%s start 11", __func__);
	ret = clk_prepare_enable(es8156->mclk);
		es8156_reset(component);
		es8156_init_regs(component);
	return ret;
}

static void es8156_remove(struct snd_soc_component *component)
{
	es8156_set_bias_level(component, SND_SOC_BIAS_OFF);
}

static const struct regmap_config es8156_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0xff,
	.cache_type	= REGCACHE_RBTREE,
	.reg_defaults = es8156_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es8156_reg_defaults),
};

static struct snd_soc_component_driver soc_codec_dev_es8156 = {
	.probe =	es8156_probe,
	.remove =	es8156_remove,
	.suspend =	es8156_suspend,
	.resume =	es8156_resume,
	.set_bias_level = es8156_set_bias_level,
#if 0
	.component_driver = {
#endif
	.controls = es8156_snd_controls,
	.num_controls = ARRAY_SIZE(es8156_snd_controls),
	.dapm_widgets = es8156_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es8156_dapm_widgets),
	.dapm_routes = es8156_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(es8156_dapm_routes),
#if 0
	}
#endif
};

/*
 *es8156 7bit i2c address:CE pin:0 0x08 / CE pin:1 0x09
 */
static int es8156_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8156_priv *es8156;
	int ret = -1;
	//int hp_irq;
	//enum of_gpio_flags flags;
	//struct device_node *np = i2c->dev.of_node;

	dev_info(&i2c->dev, "%s start\n", __func__);

	es8156 = devm_kzalloc(&i2c->dev, sizeof(*es8156), GFP_KERNEL);
	if (!es8156)
		return -ENOMEM;

	es8156->debounce_time = 200;
	es8156->hp_det_invert = 0;
	es8156->pwr_count = 0;
	es8156->hp_inserted = false;
	es8156->muted = true;

	es8156->regmap = devm_regmap_init_i2c(i2c, &es8156_regmap_config);
	if (IS_ERR(es8156->regmap)) {
		ret = PTR_ERR(es8156->regmap);
		dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, es8156);
#ifdef HP_DET_FUNTION
	es8156->spk_ctl_gpio = of_get_named_gpio_flags(np,
						       "spk-con-gpio",
						       0,
						       &flags);
	if (es8156->spk_ctl_gpio < 0) {
		dev_info(&i2c->dev, "Can not read property spk_ctl_gpio\n");
		es8156->spk_ctl_gpio = INVALID_GPIO;
	} else {
		es8156->spk_active_level = !(flags & OF_GPIO_ACTIVE_LOW);
		ret = devm_gpio_request_one(&i2c->dev, es8156->spk_ctl_gpio,
					    GPIOF_DIR_OUT, NULL);
		if (ret) {
			dev_err(&i2c->dev, "Failed to request spk_ctl_gpio\n");
			return ret;
		}
		es8156_enable_spk(es8156, false);
	}

	es8156->hp_det_gpio = of_get_named_gpio_flags(np,
						      "hp-det-gpio",
						      0,
						      &flags);
	if (es8156->hp_det_gpio < 0) {
		dev_info(&i2c->dev, "Can not read property hp_det_gpio\n");
		es8156->hp_det_gpio = INVALID_GPIO;
	} else {
		INIT_DELAYED_WORK(&es8156->work, hp_work);
		es8156->hp_det_invert = !!(flags & OF_GPIO_ACTIVE_LOW);
		ret = devm_gpio_request_one(&i2c->dev, es8156->hp_det_gpio,
					    GPIOF_IN, "hp det");
		if (ret < 0)
			return ret;
		hp_irq = gpio_to_irq(es8156->hp_det_gpio);
		ret = devm_request_threaded_irq(&i2c->dev, hp_irq, NULL,
						es8156_irq_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_TRIGGER_RISING |
						IRQF_ONESHOT,
						"es8156_interrupt", es8156);
		if (ret < 0) {
			dev_err(&i2c->dev, "request_irq failed: %d\n", ret);
			return ret;
		}

		schedule_delayed_work(&es8156->work,
				      msecs_to_jiffies(es8156->debounce_time));
	}
#endif

	ret = devm_snd_soc_register_component(&i2c->dev,
				     &soc_codec_dev_es8156,
				     &es8156_dai, 1);

	dev_info(&i2c->dev, "%s end\n", __func__);

	return ret;
}

static  int es8156_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	return 0;
}

/*
static void es8156_i2c_shutdown(struct i2c_client *client)
{
	struct es8156_priv *es8156 = i2c_get_clientdata(client);

	if (es8156_codec != NULL) {
		es8156_enable_spk(es8156, false);
		msleep(20);
		es8156_set_bias_level(es8156_codec, SND_SOC_BIAS_OFF);
	}
}
 */

static const struct i2c_device_id es8156_i2c_id[] = {
	{"es8156", 1},
	{"es8156_1", 2},
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8156_i2c_id);


static const struct of_device_id es8156_of_match[] = {
	{ .compatible = "everest,es8156", },
	{ .compatible = "everest,es8156_1", },
	{ }
};
MODULE_DEVICE_TABLE(of, es8156_of_match);

static struct i2c_driver es8156_i2c_driver = {
	.driver = {
		.name		= "es8156",
		.of_match_table = es8156_of_match,
	},
	.probe    = es8156_i2c_probe,
	.remove   = es8156_i2c_remove,
	//.shutdown = es8156_i2c_shutdown,
	.id_table = es8156_i2c_id,
};

module_i2c_driver(es8156_i2c_driver);

MODULE_DESCRIPTION("ASoC es8156 driver");
MODULE_AUTHOR("Will <pengxiaoxin@everset-semi.com>");
MODULE_LICENSE("GPL");
