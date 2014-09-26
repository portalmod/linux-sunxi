/*
 * mod-duo.c  --  SoC audio for MOD Duo audio device
 *
 * Copyright (c) 2014 Rafael Guayer <rafael@musicaloperatingdevices.com>
 *
 * based on code from:
 *
 *    Wolfson Microelectronics PLC.
 *    Openedhand Ltd.
 *    Liam Girdwood <lrg@slimlogic.co.uk>
 *    Richard Purdie <richard@openedhand.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>

#include "sunxi-i2s.h"
#include "sunxi-i2sdma.h"
#include "../codecs/cs4245.h"


#define GPIO_CODEC_RESET	(0)
#define GPIO_BYPASS_A		(0)
#define GPIO_BYPASS_B		(0)
#define GPIO_SW1_A			(0)
#define GPIO_SW2_A			(0)
#define GPIO_SW3_A			(0)
#define GPIO_SW4_A			(0)
#define GPIO_SW1_B			(0)
#define GPIO_SW2_B			(0)
#define GPIO_SW3_B			(0)
#define GPIO_SW4_B			(0)

#define CHANNEL_A	0
#define CHANNEL_B	1

#define INSTRUMENT 			0
#define LINE 				1
#define MICROPHONE 			2
#define MICROPHONE_GAIN 	3

#define I2C_ADRESS	0b10011000	// 10011xx + R/!W

typedef struct __MCLK_SET_INF
{
    __u32       samp_rate;      // sample rate
    __u16       mult_fs;        // multiply of smaple rate

    __u8        clk_div;        // mpll division
    __u8        mpll;           // select mpll, 0 - 24.576 Mhz, 1 - 22.5792 Mhz

} __mclk_set_inf;


typedef struct __BCLK_SET_INF
{
    __u8        bitpersamp;     // bits per sample
    __u8        clk_div;        // clock division
    __u16       mult_fs;        // multiplay of sample rate

} __bclk_set_inf;


static __bclk_set_inf BCLK_INF[] =
{
    // 16bits per sample
    {16,  4, 128}, {16,  6, 192}, {16,  8, 256},
    {16, 12, 384}, {16, 16, 512},

    //24 bits per sample
    {24,  4, 192}, {24,  8, 384}, {24, 16, 768},

    //32 bits per sample
    {32,  2, 128}, {32,  4, 256}, {32,  6, 384},
    {32,  8, 512}, {32, 12, 768},

    //end flag
    {0xff, 0, 0},
};

static __mclk_set_inf  MCLK_INF[] =
{
    // 8k bitrate
    {  8000, 128, 24, 0}, {  8000, 192, 16, 0}, {  8000, 256, 12, 0},
    {  8000, 384,  8, 0}, {  8000, 512,  6, 0}, {  8000, 768,  4, 0},

    // 16k bitrate
    { 16000, 128, 12, 0}, { 16000, 192,  8, 0}, { 16000, 256,  6, 0},
    { 16000, 384,  4, 0}, { 16000, 768,  2, 0},

    // 32k bitrate
    { 32000, 128,  6, 0}, { 32000, 192,  4, 0}, { 32000, 384,  2, 0},
    { 32000, 768,  1, 0},

    // 64k bitrate
    { 64000, 192,  2, 0}, { 64000, 384,  1, 0},

    //128k bitrate
    {128000, 192,  1, 0},

    // 12k bitrate
    { 12000, 128, 16, 0}, { 12000, 256, 8, 0}, { 12000, 512, 4, 0},

    // 24k bitrate
    { 24000, 128,  8, 0}, { 24000, 256, 4, 0}, { 24000, 512, 2, 0},

    // 48K bitrate
    { 48000, 128,  4, 0}, { 48000, 256,  2, 0}, { 48000, 512, 1, 0},

    // 96k bitrate
    { 96000, 128 , 2, 0}, { 96000, 256,  1, 0},

    //192k bitrate
    {192000, 128,  1, 0},

    //11.025k bitrate
    { 11025, 128, 16, 1}, { 11205, 256,  8, 1}, { 11205, 512,  4, 1},

    //22.05k bitrate
    { 22050, 128,  8, 1}, { 22050, 256,  4, 1},
    { 22050, 512,  2, 1},

    //44.1k bitrate
    { 44100, 128,  4, 1}, { 44100, 256,  2, 1}, { 44100, 512,  1, 1},

    //88.2k bitrate
    { 88200, 128,  2, 1}, { 88200, 256,  1, 1},

    //176.4k bitrate
    {176400, 128, 1, 1},

    //end flag 0xffffffff
    {0xffffffff, 0, 0, 0},
};


/*
*
*/
void mod_duo_set_impedance(int channel, int type)
{
	// switch(channel)
	// {
	// 	case CHANNEL_A:	// Channel A
	// 	{
	// 		switch(type)
	// 		{
	// 			case INSTRUMENT:			// Instrument impedance
	// 			{
	// 				gpio_set_value(GPIO_SW1_A, 1);	// 1: Switch is OFF
	// 				gpio_set_value(GPIO_SW2_A, 1);	// 0: Switch is ON
	// 				gpio_set_value(GPIO_SW3_A, 0);
	// 				gpio_set_value(GPIO_SW4_A, 1);
	// 				break;
	// 			}
	// 			case LINE:					// Line impedance
	// 			{
	// 				gpio_set_value(GPIO_SW1_A, 0);
	// 				gpio_set_value(GPIO_SW2_A, 1);
	// 				gpio_set_value(GPIO_SW3_A, 0);
	// 				gpio_set_value(GPIO_SW4_A, 1);
	// 				break;
	// 			}
	// 			case MICROPHONE:			// Microphone impedance
	// 			{
	// 				gpio_set_value(GPIO_SW1_A, 1);
	// 				gpio_set_value(GPIO_SW2_A, 0);
	// 				gpio_set_value(GPIO_SW3_A, 0);
	// 				gpio_set_value(GPIO_SW4_A, 1);
	// 				break;
	// 			}
	// 			case MICROPHONE_GAIN:		// Microphone impedance with gain stage
	// 			{
	// 				gpio_set_value(GPIO_SW1_A, 1);
	// 				gpio_set_value(GPIO_SW2_A, 0);
	// 				gpio_set_value(GPIO_SW3_A, 1);
	// 				gpio_set_value(GPIO_SW4_A, 0);
	// 				break;
	// 			}
	// 		}
	// 		break;
	// 	}

	// 	case CHANNEL_B:	// Channel B
	// 	{
	// 		switch(type)
	// 		{
	// 			case INSTRUMENT:			// Instrument impedance
	// 			{
	// 				gpio_set_value(GPIO_SW1_B, 1);
	// 				gpio_set_value(GPIO_SW2_B, 1);
	// 				gpio_set_value(GPIO_SW3_B, 0);
	// 				gpio_set_value(GPIO_SW4_B, 1);
	// 				break;
	// 			}
	// 			case LINE:					// Line impedance
	// 			{
	// 				gpio_set_value(GPIO_SW1_B, 0);
	// 				gpio_set_value(GPIO_SW2_B, 1);
	// 				gpio_set_value(GPIO_SW3_B, 0);
	// 				gpio_set_value(GPIO_SW4_B, 1);
	// 				break;
	// 			}
	// 			case MICROPHONE:			// Microphone impedance
	// 			{
	// 				gpio_set_value(GPIO_SW1_B, 1);
	// 				gpio_set_value(GPIO_SW2_B, 0);
	// 				gpio_set_value(GPIO_SW3_B, 0);
	// 				gpio_set_value(GPIO_SW4_B, 1);
	// 				break;
	// 			}
	// 			case MICROPHONE_GAIN:		// Microphone impedance with gain stage
	// 			{
	// 				gpio_set_value(GPIO_SW1_B, 1);
	// 				gpio_set_value(GPIO_SW2_B, 0);
	// 				gpio_set_value(GPIO_SW3_B, 1);
	// 				gpio_set_value(GPIO_SW4_B, 0);
	// 				break;
	// 			}
	// 		}
	// 		break;
	// 	}

	// }
	return;
}

/*
*
*/
void mod_duo_set_bypass(int channel, bool en)
{
	// switch(channel)
	// {
	// 	case CHANNEL_A:	// Channel A
	// 	{
	// 		if(en)
	// 			gpio_set_value(GPIO_BYPASS_A, 1);	// Bypass ON: Pin = HIGH
	// 		else
	// 			gpio_set_value(GPIO_BYPASS_A, 0);	// Bypass OFF: PIN = LOW
	// 		break;
	// 	}
	// 	case CHANNEL_B:	// Channel B
	// 	{
	// 		if(en)
	// 			gpio_set_value(GPIO_BYPASS_B, 1);
	// 		else
	// 			gpio_set_value(GPIO_BYPASS_B, 0);
	// 		break;
	// 	}
	// }
	return;
}

static void mod_duo_enable_audio(bool en)
{
	// if (en) 
	// {
	// 	gpio_set_value(GPIO_CODEC_RESET, 1);
	// } 
	// else 
	// {
	// 	gpio_set_value(GPIO_CODEC_RESET, 0);
	// }
	return;
}

/* 
* CS4245 
*/
static int mod_duo_cs4245_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	mod_duo_set_impedance(CHANNEL_A, INSTRUMENT);
	mod_duo_set_impedance(CHANNEL_B, INSTRUMENT);
	mod_duo_set_bypass(CHANNEL_A, 0);
	mod_duo_set_bypass(CHANNEL_B, 0);

	/* set freq to 0 */
	return snd_soc_dai_set_sysclk(codec_dai, 0, 0, 0);
}

/* 
* CS4245 
*/
static void mod_duo_cs4245_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* set freq to 0 to enable all possible codec sample rates */
	snd_soc_dai_set_sysclk(codec_dai, 0, 0, 0);
}

/* 
* 
*/
static int mod_duo_analog_suspend(struct snd_soc_card *card)
{
	mod_duo_enable_audio(false);
	return 0;
}

/* 
* 
*/
static int mod_duo_analog_resume(struct snd_soc_card *card)
{
	mod_duo_enable_audio(true);
	return 0;
}

/* 
* 
*/
static s32 get_clock_divder_slave(u32 sample_rate, u32 sample_width, u32* bclk_div, u32* mpll, u32* mult_fs)
{
	u32 ret = -EINVAL;
	switch(sample_rate) {
		case 44100:
			*mpll = 1;
			*bclk_div = 512;
			ret = 0;
			break;
		case 48000:
			*mpll = 0;
			*bclk_div = 512;
			ret = 0;
			break;
		case 88200:
			*mpll = 1;
			*bclk_div = 256;
			ret = 0;
			break;
		case 96000:
			*mpll = 0;
			*bclk_div = 256;
			ret = 0;
			break;
		case 176400:
			*mpll = 1;
			*bclk_div = 128;
			ret = 0;
			break;
		case 192000:
			*mpll = 0;
			*bclk_div = 128;
			ret = 0;
			break;
	}
	return ret;
}

/* 
* 
*/
static s32 get_clock_divder_master(u32 sample_rate, u32 sample_width, u32 * mclk_div, u32* mpll, u32* bclk_div, u32* mult_fs)
{
	u32 i, j, ret = -EINVAL;

	for(i=0; i< 100; i++) {
		 if((MCLK_INF[i].samp_rate == sample_rate) &&
		 	((MCLK_INF[i].mult_fs == 256) || (MCLK_INF[i].mult_fs == 128))) {
			  for(j=0; j<ARRAY_SIZE(BCLK_INF); j++) {
					if((BCLK_INF[j].bitpersamp == sample_width) &&
						(BCLK_INF[j].mult_fs == MCLK_INF[i].mult_fs)) {
						 //set mclk and bclk division
						 *mclk_div = MCLK_INF[i].clk_div;
						 *mpll = MCLK_INF[i].mpll;
						 *bclk_div = BCLK_INF[j].clk_div;
						 *mult_fs = MCLK_INF[i].mult_fs;
						 ret = 0;
						 break;
					}
			  }
		 }
		 else if(MCLK_INF[i].samp_rate == 0xffffffff)
		 	break;
	}

	return ret;
}

/* 
* CS4245 
*/
static int mod_duo_cs4245_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned int clk = 24576000;
	unsigned long rate = params_rate(params);
	unsigned ind fmt = 0;
	u32 mclk_div=0, mpll=0, bclk_div=0, mult_fs=0;

	if(!sunxi_i2s_slave) 
	{
		get_clock_divder_master(rate, 32, &mclk_div, &mpll, &bclk_div, &mult_fs);
		printk("[IIS-0] get_clock_divder_master: rate=(%lu), mclk_div=(%d), mpll=(%d), bclk_div=(%d), mult_fs=(%d)\n", rate, mclk_div, mpll, bclk_div, mult_fs);
		fmt = SND_SOC_DAIFMT_I2S |		/* I2S mode */
	    	 SND_SOC_DAIFMT_NB_NF;		/* normal bit clock + frame */
	    	 // SND_SOC_DAIFMT_CBS_CFS;	/* codec clk & FRM slave */
	} 
	else 
	{
		get_clock_divder_slave(rate, 32, &bclk_div, &mpll, &mult_fs);
		printk("[IIS-0] get_clock_divder_slave: rate=(%lu), bclk_div=(%d), mpll=(%d), mult_fs=(%d)\n", rate, bclk_div, mpll, mult_fs);
		fmt = SND_SOC_DAIFMT_I2S |		/* I2S mode */
	    	 SND_SOC_DAIFMT_NB_NF;		/* normal bit clock + frame */
//	    	 SND_SOC_DAIFMT_CBM_CFM;	/* codec clk & FRM master */
	}

	//call sunxi_iis_set_fmt - CODEC DAI
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0)
		return ret;

	//call sunxi_iis_set_fmt - CPU DAI
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0)
		return ret;

	//call sndi2s_set_dai_sysclk - CODEC DAI
	ret = snd_soc_dai_set_sysclk(codec_dai, 0 , clk, 0);
	if (ret < 0)
		return ret;

	//call sunxi_iis_set_sysclk - CPU DAI
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0 , mpll, 0);
	if (ret < 0)
		return ret;

	if(!sunxi_i2s_slave) {
		//call sunxi_iis_set_clkdiv	
		ret = snd_soc_dai_set_clkdiv(cpu_dai, SUNXI_DIV_MCLK, mclk_div);
		if (ret < 0)
			return ret;

		//call sunxi_iis_set_clkdiv	
		ret = snd_soc_dai_set_clkdiv(cpu_dai, SUNXI_DIV_BCLK, bclk_div);
		if (ret < 0)
			return ret;
	} else {
		//call sunxi_iis_set_clkdiv	
		ret = snd_soc_dai_set_clkdiv(cpu_dai, SUNXI_DIV_EXTCLK, bclk_div);
		if (ret < 0)
			return ret;
	}

	// //call sndi2s_set_dai_clkdiv
	// ret = snd_soc_dai_set_clkdiv(codec_dai, 0, mult_fs);
	// if (ret < 0)
	// 	return ret;

	printk("[IIS-0] mod_duo_cs4245_hw_params: codec_dai=(%s), cpu_dai=(%s)\n", codec_dai->name, cpu_dai->name);
	printk("[IIS-0] mod_duo_cs4245_hw_params: channel num=(%d)\n", params_channels(params));
	printk("[IIS-0] mod_duo_cs4245_hw_params: sample rate=(%lu)\n", rate);

	switch (params_format(params)) 
	{
	case SNDRV_PCM_FORMAT_S16_LE:
		printk("[IIS-0] mod_duo_cs4245_hw_params: format 16 bit\n");
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		printk("[IIS-0] mod_duo_cs4245_hw_params: format 20 bit in 3 bytes\n");
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		printk("[IIS-0] mod_duo_cs4245_hw_params: format 24 bit in 4 bytes\n");
		break;
	default:
		printk("[IIS-0] mod_duo_cs4245_hw_params: Unsupported format (%d)\n", (int)params_format(params));
	}

	return 0;
}

/* 
* CS4245 
*/
static struct snd_soc_ops mod_duo_cs4245_ops = {
	.startup = mod_duo_cs4245_startup,
	.shutdown = mod_duo_cs4245_shutdown,
	.hw_params = mod_duo_cs4245_hw_params,
};

/* 
* 
*/
static struct snd_soc_dai_link snd_soc_mod_duo_dai =
{
	.name = "CS4245-I2S",
	.stream_name = "SUNXI-CS4245-I2S",
	.cpu_dai_name = "sunxi-i2s.0",
	.codec_dai_name = "cs4245",
	.platform_name = "sunxi-i2s-pcm-audio.0",
	.codec_name	= "cs4245.0.1",
	.ops = &mod_duo_cs4245_ops,
};

/* 
* 
*/
static struct snd_soc_card snd_soc_mod_duo_soundcard = {
	.name = "MOD-Duo-Sound-Card",
	.owner = THIS_MODULE,
	.dai_link = &snd_soc_mod_duo_dai,
	.num_links = 1,
	.suspend_post = mod_duo_analog_suspend,
	.resume_pre	= mod_duo_analog_resume,
};

static struct platform_device *mod_duo_audio_device;

static int __init mod_duo_audio_init(void)
{
	int ret;

	/* Register analog device */
	mod_duo_audio_device = platform_device_alloc("soc-audio", 0);
	if (!mod_duo_audio_device)
		return -ENOMEM;

	platform_set_drvdata(mod_duo_audio_device, &snd_soc_mod_duo_dai);

	ret = platform_device_add(mod_duo_audio_device);
	if (ret < 0) {
		platform_device_put(mod_duo_audio_device);
		return ret;
	}

	mod_duo_enable_audio(true);
	return 0;
}

static void __exit mod_duo_audio_exit(void)
{
	mod_duo_enable_audio(false);

	platform_device_unregister(mod_duo_audio_device);

	gpio_free(GPIO_CODEC_RESET);
	gpio_free(GPIO_BYPASS_A);
	gpio_free(GPIO_BYPASS_B);
	gpio_free(GPIO_SW1_A);
	gpio_free(GPIO_SW2_A);
	gpio_free(GPIO_SW3_A);
	gpio_free(GPIO_SW4_A);
	gpio_free(GPIO_SW1_B);
	gpio_free(GPIO_SW2_B);
	gpio_free(GPIO_SW3_B);
	gpio_free(GPIO_SW4_B);
}

module_init(mod_duo_audio_init);
module_exit(mod_duo_audio_exit);

/* Module information */
MODULE_AUTHOR("Rafael Guayer <rafael@musicaloperatingdevices.com>");
MODULE_DESCRIPTION("MOD Duo Sound Card Audio Machine Driver");
MODULE_LICENSE("GPL");
