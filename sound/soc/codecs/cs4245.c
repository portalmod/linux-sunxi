/*
 * CS4245 ALSA SoC (ASoC) codec driver
 *
 * Author: Rafael Guayer <rafael@musicaloperatingdevices.com>
 *
 * Copyright 2014 Musical Operating Devices LLC.  This file is licensed
 * under the terms of the GNU General Public License version 2.  This
 * program is licensed "as is" without any warranty of any kind, whether
 * express or implied.
 *
 * This is an ASoC device driver for the Cirrus Logic CS4245 codec.
 *
 * Current features/limitations:
 *
 * - Initial implementation based on CS4270 ALSA SoC (ASoC) codec driver from Timur Tabi <timur@freescale.com>
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/i2c.h>
#include <linux/delay.h>

 #include "cs4245.h"

/* Power-on default values for the registers
 *
 * This array contains the power-on default values of the registers, with the
 * exception of the "CHIPID" register (01h).  The lower four bits of that
 * register contain the hardware revision, so it is treated as volatile.
 *
 * Also note that on the CS4245, the first readable register is 1, but ASoC
 * assumes the first register is 0.  Therfore, the array must have an entry for
 * register 0, but we use cs4245_reg_is_readable() to tell ASoC that it can't
 * be read.
 */
static const u8 cs4245_default_reg_cache[CS4245_LASTREG + 1] = {
	0x00, 	/* 0x00 - Padding for ASoC index 0 read */
	0x00, 	/* 0x01 - Chip ID */
	0x01,  	/* 0x02 - Power Control */
	0x08,  	/* 0x03 - DAC Control 1 */
	0x00,  	/* 0x04 - ADC Control */
	0x00,  	/* 0x05 - MCLK Frequency */
	0x40,  	/* 0x06 - Signal Selection */
	0x00,  	/* 0x07 - PGA Ch B Gain Control */
	0x00,  	/* 0x08 - PGA Ch A Gain Control */
	0x19,  	/* 0x09 - Analog Input Control */
	0x00,  	/* 0x0A - DAC Ch A Volume Control */
	0x00,  	/* 0x0B - DAC Ch B Volume Control */
	0xc0,  	/* 0x0C - DAC Control 2 */
	0x00,  	/* 0x0D - Interrupt Status */
	0x00,  	/* 0x0E - Interrupt Mask */
	0x00,  	/* 0x0F - Interrupt Mode MSB */
	0x00  	/* 0x10 - Interrupt Mode LSB */
};

struct cs4245_private {
	enum snd_soc_control_type control_type;
	int mclk;
	int dai_fmt;
	int slave_mode;

};

/**
 * struct cs4245_mode_ratios - clock ratio tables
 * @ratio: the ratio of MCLK to the sample rate
 * @speed_mode: the Speed Mode bits to set in the Mode Control register for
 *              this ratio
 * @mclk: the Ratio Select bits to set in the Mode Control register for this
 *        ratio
 *
 * The data for this chart is taken from Table 2 of the CS4245 reference
 * manual.
 *
 * This table is used to determine how to program the Mode Control register.
 * It is also used by cs4245_set_dai_sysclk() to tell ALSA which sampling
 * rates the CS4245 currently supports.
 *
 * @speed_mode is the corresponding bit pattern to be written to the
 * MODE bits of the Mode Control Register
 *
 * @mclk is the corresponding bit pattern to be wirten to the MCLK bits of
 * the Mode Control Register.
 *
 * In situations where a single ratio is represented by multiple speed
 * modes, we favor the slowest speed.  E.g, for a ratio of 128, we pick
 * double-speed instead of quad-speed. 
 *
 */

// struct cs4245_mode_ratios {
// 	unsigned int ratio;
// 	u8 speed_mode;
// 	u8 mclk;
// };

// static struct cs4245_mode_ratios cs4245_mode_ratios[] = {
// 	{64, CS4245_DAC_FM_QUAD, CS4245_MCLK_1},
// 	{96, CS4245_DAC_FM_QUAD, CS4245_MCLK_1_5},
// 	{128, CS4245_DAC_FM_DOUBLE, CS4245_MCLK_1},
// 	{192, CS4245_DAC_FM_DOUBLE, CS4245_MCLK_1_5},
// 	{256, CS4245_DAC_FM_SINGLE, CS4245_MCLK_1},
// 	{384, CS4245_DAC_FM_SINGLE, CS4245_MCLK_1_5},
// 	{512, CS4245_DAC_FM_SINGLE, CS4245_MCLK_2},
// 	{768, CS4245_DAC_FM_SINGLE, CS4245_MCLK_3},
// 	{1024, CS4245_DAC_FM_SINGLE, CS4245_MCLK_4}
// };

// // The number of MCLK/LRCK ratios supported by the CS4245
// #define NUM_MCLK_RATIOS		ARRAY_SIZE(cs4245_mode_ratios)


/**
 *
 * struct snd_soc_codec
 *
 * SoC Audio Codec device
 *
 * Definition at line 534 of file include/sound/soc.h
 *
*/
static int cs4245_reg_is_readable(struct snd_soc_codec *codec, unsigned int reg)
{
	return (reg >= CS4245_FIRSTREG) && (reg <= CS4245_LASTREG);
}

static int cs4245_reg_is_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	/* Unreadable registers are considered volatile */
	if ((reg < CS4245_FIRSTREG) || (reg > CS4245_LASTREG))
		return 1;

	return reg == CS4245_CHIP_ID;
}

/**
 * cs4245_set_dai_sysclk - determine the CS4245 samples rates.
 * @codec_dai: the codec DAI
 * @clk_id: the clock ID , if MCLK1 or MCLK2
 * @freq: the MCLK input frequency
 * @dir: the clock direction (ignored)
 *
 * This function is used to tell the codec driver what the input MCLK
 * frequency is.
 *
 * The value of MCLK is used to determine which sample rates are supported
 * by the CS4245.  The ratio of MCLK / Fs must be equal to one of nine
 * supported values - 64, 96, 128, 192, 256, 384, 512, 768, and 1024.
 *
 * This function calculates the nine ratios and determines which ones match
 * a standard sample rate.  If there's a match, then it is added to the list
 * of supported sample rates.
 *
 * This function must be called by the machine driver's 'startup' function,
 * otherwise the list of supported sample rates will not be available in
 * time for ALSA.
 *
 * For setups with variable MCLKs, pass 0 as 'freq' argument. This will cause
 * theoretically possible sample rates to be enabled. Call it again with a
 * proper value set one the external clock is set (most probably you would do
 * that from a machine's driver 'hw_param' hook.
 *
 */

 /*
 * struct snd_soc_dai *codec_dai
 * Digital Audio Interface runtime data.
 *
 * Holds runtime data for a DAI.
 *
 * Definition at line 223 of file include/sound/soc-dai.h.
 */

 /* 
 * struct snd_soc_codec
 *
 * SoC Audio Codec device
 *
 * Definition at line 534 of file include/sound/soc.h
 */

 /*
 * The CS4245 has two master clocks, MCLK1 and MCLK2, one for each serial interface, for asynchronous operation.
 * This driver implements only synchronous operation, where the MCLK1 is used and the second serial interface uses it for clock reference.
 */
static int cs4245_set_dai_sysclk(struct snd_soc_dai *codec_dai, 
								int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);
	cs4245->mclk = freq;
	return 0;
}

/*
* TODO: Function description.
*/
static int cs4245_set_dai_clkdiv(struct snd_soc_dai *codec_dai, int div_id, int div)
{
/* cleaning code
	hdmi_parameter.fs_between = div;
*/
	return 0;
}

/**
 * cs4245_set_dai_fmt - configure the codec for the selected audio format
 * @codec_dai: the codec DAI
 * @format: a SND_SOC_DAIFMT_x value indicating the data format
 *
 * This function takes a bitmask of SND_SOC_DAIFMT_x bits and programs the
 * codec accordingly.
 *
 * Currently, this function only supports SND_SOC_DAIFMT_I2S and
 * SND_SOC_DAIFMT_LEFT_J.
 *
 * The CS4245 enables different formats for each of the two serial interfaces.
 * This driver implements only equal formats for both serial interfaces.
 */
 // TODO - Set format on CODEC.
static int cs4245_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);

	/* set DAI format */
	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		cs4245->dai_fmt = format & SND_SOC_DAIFMT_FORMAT_MASK;
		break;
	default:
		dev_err(codec->dev, "invalid dai format\n");
		return -EINVAL;
	}

	/* set master/slave audio interface */
	/* The CS4245 has two serial interfaces, I2S1 and I2S2, which can both work as master or slave.
	 * This driver implements the I2S2 interface allways as slave, with the LRCK2 signal connected do LRCK1 and SCLK2 signal connected to SCLK1.
	 */
	switch (format & SND_SOC_DAIFMT_MASTER_MASK) {
	// case SND_SOC_DAIFMT_CBS_CFS:	/* codec clk & FRM slave  */
	// 	cs4245->slave_mode = 1;
	// 	break;
	case SND_SOC_DAIFMT_CBM_CFM:	/* codec clk & FRM master */
		cs4245->slave_mode = 0;		// TODO - Make it generic. In MOD Duo Sound Card it should allways be master.
		break;
	default:
		/* all other modes are unsupported by the hardware */
		dev_err(codec->dev, "Unknown master/slave configuration\n");
		return -EINVAL;
	}
	return 0;
}

/**
 * cs4245_hw_params - program the CS4245 with the given hardware parameters.
 * @substream: the audio stream
 * @params: the hardware parameters to set
 * @dai: the SOC DAI (ignored)
 *
 * This function programs the hardware with the values provided.
 * Specifically, the sample rate and the data format.
 *
 * The .ops functions are used to provide board-specific data, like input
 * frequencies, to this driver.  This function takes that information,
 * combines it with the hardware parameters provided, and programs the
 * hardware accordingly.
 */

// HARD CODED

static int cs4245_hw_params(struct snd_pcm_substream *substream, 
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
//	struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);
	int ret;
	int reg;

	/* Configure the CODEC registers */
	/* DAC Control */

	reg = CS4245_DAC_FM_SINGLE | CS4245_DAC_MASTER | CS4245_DAC_DIF_I2S;

//	reg = CS4245_DAC_FM_SINGLE | CS4245_DAC_MASTER;
//	if(cs4245->dai_fmt == SND_SOC_DAIFMT_I2S)			/* I2S, up to 24-bit data */
//		reg |= CS4245_DAC_DIF_I2S;
//	else if(cs4245->dai_fmt == SND_SOC_DAIFMT_RIGHT_J)	/* Right-Justified, 24-bit Data */
//		reg |= CS4245_DAC_DIF_RJUST_24;
//	else if(cs4245->dai_fmt == SND_SOC_DAIFMT_LEFT_J)	/* Left Justified, up to 24-bit data */
//		reg |= CS4245_DAC_DIF_LJUST;

	ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, reg);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}
	else
		printk("[I2S] cs4245_hw_params: CS4245 DAC Control 1 register configured\n");

	/* ADC Control */
	reg = CS4245_ADC_FM_SINGLE | CS4245_ADC_DIF_I2S | CS4245_HPF_FREEZE;
	ret = snd_soc_write(codec, CS4245_ADC_CTRL, reg);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}
	else
		printk("[I2S] cs4245_hw_params: CS4245 ADC Control register configured\n");

	/* Master Clock Frequency */
	reg = (CS4245_MCLK_2 << CS4245_MCLK1_SHIFT) | (CS4245_MCLK_2 << CS4245_MCLK2_SHIFT);
	ret = snd_soc_write(codec, CS4245_MCLK_FREQ, reg);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}
	else
		printk("[I2S] cs4245_hw_params: CS4245 Master Clock Frequency register configured\n");

	/* Signal Selection */
	reg = CS4245_A_OUT_SEL_DAC;
	ret = snd_soc_write(codec, CS4245_SIGNAL_SEL, reg);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}
	else
		printk("[I2S] cs4245_hw_params: CS4245 Signal Selection register configured\n");


	/* ADC Input Control */
	reg = CS4245_PGA_SOFT | CS4245_PGA_ZERO | CS4245_SEL_INPUT_4;
	ret = snd_soc_write(codec, CS4245_ANALOG_IN, reg);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}
	else
		printk("[I2S] cs4245_hw_params: CS4245 ADC Input Control register configured\n");


	/* DAC Control 2 */
	reg = CS4245_DAC_SOFT | CS4245_DAC_ZERO;
	ret = snd_soc_write(codec, CS4245_DAC_CTRL_2, reg);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}
	else
		printk("[I2S] cs4245_hw_params: CS4245 DAC COntrol 2 register configured\n");


	return ret;
}

/**
 * cs4245_dai_mute - enable/disable the CS4245 external mute
 * @dai: the SOC DAI
 * @mute: 0 = disable mute, 1 = enable mute
 *
 * This function toggles the mute bits in the DAC Control 1 register. 
 */
static int cs4245_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	// struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);
	int reg3;

	reg3 = snd_soc_read(codec, CS4245_DAC_CTRL_1);

	if (mute)
		reg3 |= CS4245_MUTE_DAC;
	else 
		reg3 &= ~(CS4245_MUTE_DAC);
	return snd_soc_write(codec, CS4245_DAC_CTRL_1, reg3);
}

/* A list of non-DAPM controls that the CS4245 supports */ // - TODO - Add Alsa Mixer Controls
// static const struct snd_kcontrol_new cs4245_snd_controls[] = {
// 	SOC_DOUBLE_R("DAC Volume", CS4245_DAC_A_CTRL, CS4245_DAC_B_CTRL, 0, 0xFF, 1),
// 	SOC_DOUBLE_R("PGA Gain", CS4245_PGA_A_CTRL, CS4245_PGA_B_CTRL, 0, 0xFF, 1),
// };

/**
 * cs4245_probe - ASoC probe function
 * @pdev: platform device
 *
 * This function is called when ASoC has all the pieces it needs to
 * instantiate a sound driver.
 */
static int cs4245_probe(struct snd_soc_codec *codec)
{
	struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);
	int ret;

	/* Tell ASoC what kind of I/O to use to read the registers.  ASoC will
	 * then do the I2C transactions itself.
	 */
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, cs4245->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "failed to set cache I/O (ret=%i)\n", ret);
		return ret;
	}

	/* 
	 * Disable Power Down.
	 */
	ret = snd_soc_update_bits(codec, CS4245_POWER_CTRL, 0x01, 0x00);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}


	// /* Add the non-DAPM controls */ - TODO - Add Alsa Mixer Controls
	// ret = snd_soc_add_codec_controls(codec, cs4245_snd_controls,
	// 			ARRAY_SIZE(cs4245_snd_controls));
	// if (ret < 0) {
	// 	dev_err(codec->dev, "failed to add controls\n");
	// 	return ret;
	// }

	return ret;
}

#define cs4245_remove 	NULL
#define cs4245_soc_suspend	NULL
#define cs4245_soc_resume	NULL

static const struct snd_soc_dai_ops cs4245_dai_ops = {
	.hw_params = cs4245_hw_params,
	.set_sysclk	= cs4245_set_dai_sysclk,
	.set_clkdiv = cs4245_set_dai_clkdiv,
	.set_fmt = cs4245_set_dai_fmt,
	.digital_mute = cs4245_dai_mute,
};

static struct snd_soc_dai_driver cs4245_dai = {
	.name = "cs4245-dai",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 4000,
		.rate_max = 192000,
		.formats = CS4245_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 4000,
		.rate_max = 192000,
		.formats = CS4245_FORMATS,
	},
	.ops = &cs4245_dai_ops,
	.symmetric_rates = 1,	// TODO - In a generic driver for CS4245 is possible to work assynchronous with MCLK1 and MCLK2 with different frequencies
};
//EXPORT_SYMBOL(cs4245_dai);


/*
 * ASoC codec driver structure
 */
static const struct snd_soc_codec_driver soc_codec_device_cs4245 = {
	.probe = cs4245_probe,
	.remove = cs4245_remove,
	.suspend = cs4245_soc_suspend,
	.resume = cs4245_soc_resume,
	.volatile_register = cs4245_reg_is_volatile,
	.readable_register = cs4245_reg_is_readable,
	.reg_cache_size = CS4245_LASTREG + 1,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = cs4245_default_reg_cache,
};

/*
 * cs4245_id - I2C device IDs supported by this driver
 */
static const struct i2c_device_id cs4245_id[] = {
	{"cs4245", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs4245_id);

/**
 * cs4245_i2c_probe - initialize the I2C interface of the CS4245
 * @i2c_client: the I2C client object
 * @id: the I2C device ID (ignored)
 *
 * This function is called whenever the I2C subsystem finds a device that
 * matches the device ID given via a prior call to i2c_add_driver().
 */
static int cs4245_i2c_probe(struct i2c_client *i2c_client,
	const struct i2c_device_id *id)
{
	struct cs4245_private *cs4245;

	int ret;

	/* Verify that we have a CS4245 */

	ret = i2c_smbus_read_byte_data(i2c_client, CS4245_CHIP_ID);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "failed to read i2c at addr %X\n",
		       i2c_client->addr);
		return ret;
	}
	/* The top four bits of the chip ID should be 1100. */
	if ((ret & 0xF0) != 0xC0) {
		dev_err(&i2c_client->dev, "device at addr %X is not a CS4245\n",
		       i2c_client->addr);
		return -ENODEV;
	}

	dev_info(&i2c_client->dev, "found device at i2c address %X\n", i2c_client->addr);
	dev_info(&i2c_client->dev, "hardware revision %X\n", ret & 0xF);

	cs4245 = devm_kzalloc(&i2c_client->dev, sizeof(struct cs4245_private), GFP_KERNEL);
	if (!cs4245) {
		dev_err(&i2c_client->dev, "could not allocate codec\n");
		return -ENOMEM;
	}

	cs4245->control_type = SND_SOC_I2C;

	i2c_set_clientdata(i2c_client, cs4245);


	ret = snd_soc_register_codec(&i2c_client->dev,
			&soc_codec_device_cs4245, &cs4245_dai, 1);
	return ret;
}

/**
 * cs4245_i2c_remove - remove an I2C device
 * @i2c_client: the I2C client object
 *
 * This function is the counterpart to cs4245_i2c_probe().
 */
static int cs4245_i2c_remove(struct i2c_client *i2c_client)
{
	snd_soc_unregister_codec(&i2c_client->dev);
	return 0;
}

/*
 * cs4245_i2c_driver - I2C device identification
 *
 * This structure tells the I2C subsystem how to identify and support a
 * given I2C device type.
 */
static struct i2c_driver cs4245_i2c_driver = {
	.driver = {
		.name = "cs4245-codec",
		.owner = THIS_MODULE,
	},
	.id_table = cs4245_id,
	.probe = cs4245_i2c_probe,
	.remove = cs4245_i2c_remove,
};

static int __init cs4245_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&cs4245_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register CS4245 I2C driver: %d\n",
		       ret);
	}
	return ret;
}
module_init(cs4245_init);

static void __exit cs4245_exit(void)
{
	i2c_del_driver(&cs4245_i2c_driver);
}
module_exit(cs4245_exit);

MODULE_AUTHOR("Rafael Guayer <rafael@musicaloperatingdevices.com>");
MODULE_DESCRIPTION("Cirrus Logic CS4245 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
