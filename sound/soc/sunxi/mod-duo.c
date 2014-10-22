/*
 * mod-duo.c  --  SoC audio for MOD Duo audio device
 *
 * Copyright (c) 2014 Rafael Guayer <rafael@musicaloperatingdevices.com>
 *
 * based on code from:
 *
 *    Wolfson Microelectronics PLC.@
 *    Openedhand Ltd.
 *    Liam Girdwood <lrg@slimlogic.co.uk>
 *    Richard Purdie <richard@openedhand.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <plat/sys_config.h>
#include <linux/io.h>

#include "i2s/sunxi-i2s.h"
#include "i2s/sunxi-i2sdma.h"
#include "../codecs/cs4245.h"	// Really needed?

// GPIO macros
#define PIN_DIR_OUT		1
#define CHANNEL_A		0
#define CHANNEL_B		1

#define INSTRUMENT 		0
#define LINE 			1
#define MICROPHONE 		2
#define PAD_OFF			3
#define PAD_ON			4

#define BYPASS 			0
#define PROCESS 		1

#define I2C_ADRESS	0b10011000	// 10011xx + R/!W

static int mod_duo_used = 0;

// GPIO Handler
static u32 mod_duo_gpio_handler = 0;

/*
* GPIO Initialization - TODO: Function description.
*/
static int mod_duo_gpio_init(void)
{
	int err;
	script_gpio_set_t info;

	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

	mod_duo_gpio_handler = gpio_request_ex("mod_duo_souncard_para", NULL);
// JFET Switch A Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a1_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a1_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a1_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a2_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a2_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a2_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a3_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a3_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a3_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a4_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a4_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a4_pin");
// JFET Switch B Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b1_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b1_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
 	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b1_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b2_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b2_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
 	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b2_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b3_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b3_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b3_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b4_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b4_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b4_pin");
// Overflow Leds Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl1_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl1_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl1_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl2_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl2_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
  	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl2_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl3_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl3_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
  	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl3_pin");
	err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl4_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl4_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl4_pin");
// Headphone Volume Control Pin Configuration - TODO: Create a separate driver for Headphone Aomplifier (LM4811).
    err = script_parser_fetch("mod_duo_souncard_para", "hp_vol_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"hp_vol_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "hp_vol_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "hp_clk_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"hp_clk_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "hp_clk_pin");
// True Bypass Control Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "bypass_a_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"bypass_a_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "bypass_a_pin");
    err = script_parser_fetch("mod_duo_souncard_para", "bypass_b_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"bypass_b_pin\" gpio handler, already used by others?.", __FUNCTION__);
        return -EBUSY;
    }
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "bypass_b_pin");
	printk("[MOD Duo Machine Driver]GPIOs initialized.\n");
	return 0;
}

/*
* TODO: Function description.
*/
static void mod_duo_gpio_release(void)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	gpio_release(mod_duo_gpio_handler, 2);
	printk("[MOD Duo Machine Driver] GPIOs released.\n");
	return;
}

/*
* TODO: Function description.
*/
static void mod_duo_set_impedance(int channel, int type)
{
//	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	switch(channel)
	{
		case CHANNEL_A:	// Channel A
		{
			switch(type)
			{
				case INSTRUMENT:	// Instrument impedance
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a1_pin"); // 1: Switch is OFF
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a2_pin"); // 0: Switch is ON
					break;

				case LINE:			// Line impedance
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a2_pin");
					break;
				case MICROPHONE:	// Microphone impedance
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a2_pin");
					break;
				case PAD_OFF:		// Gain Stage OFF
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a4_pin");
					break;
				case PAD_ON:			// Gain Stage ON
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a4_pin");
					break;
			}
			break;
		}
		case CHANNEL_B:	// Channel B
		{
			switch(type)
			{
				case INSTRUMENT:	// Instrument impedance
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b1_pin"); // 1: Switch is OFF
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b2_pin"); // 0: Switch is ON
					break;
				case LINE:			// Line impedance
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b2_pin");
					break;
				case MICROPHONE:	// Microphone impedance
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b2_pin");
					break;
				case PAD_OFF:		// Gain Stage OFF
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b4_pin");
					break;
				case PAD_ON:			// Gain Stage ON
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b4_pin");
					break;
			}
			break;
		}
	}
	return;
}

/*
* TODO: Function description.
*/
static void mod_duo_set_bypass(int channel, bool en)
{
//	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	switch(channel)
	{
		case CHANNEL_A:	// Channel A
			if(en == PROCESS)	// Bypass OFF: Pin = HIGH - Processing. Input jack connected to Ccodec.
				gpio_write_one_pin_value(mod_duo_gpio_handler, PROCESS, "bypass_a_pin");
			else	// Bypass ON: Pin = LOW - Not processing. Input jack connected to output jack.
				gpio_write_one_pin_value(mod_duo_gpio_handler, BYPASS, "bypass_a_pin");			
			break;
		case CHANNEL_B:	// Channel B
			if(en == PROCESS)	// Bypass OFF: Pin = HIGH - Processing. Input jack connected to Ccodec.
				gpio_write_one_pin_value(mod_duo_gpio_handler, PROCESS, "bypass_b_pin");
			else	// Bypass ON: Pin = LOW - Not processing. Input jack connected to output jack.
				gpio_write_one_pin_value(mod_duo_gpio_handler, BYPASS, "bypass_b_pin");
			break;
	}
	return;
}

/* 
* TODO: Function description.
* TODO: Check if the _set_fmt and _setsysclk of cpu_dai and codec_dai should really be called here. 
* TODO: I think the functions should be called on mod_duo_audio_init. 
* TODO: This function is beeing called by aplay every time.
* TODO: I think set_fmt and set_sysclk should be called once only on the initialization of the driver.
* TODO: Compare with other machine drivers startup functions. Seems like it is used to update the DAPM external controllers (ALSA Mixer).
* TODO: Could be on snd_soc_dai_link.init? Just asked on #alsa-soc.
* TODO: Following #alsa-soc, try late_probe() (snd_soc_card) or init() (snd_soc_dai_link).
*/
static int mod_duo_startup(struct snd_pcm_substream *substream)
{
	// int ret;
	// struct snd_soc_pcm_runtime *rtd = substream->private_data;
	// struct snd_soc_dai *codec_dai = rtd->codec_dai;
	// struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	// unsigned int fmt = 0;
	// unsigned int mclk = 24576000;	// MOD Duo Soun Card has an 2457600Hz external clock

	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

	// // Initialize the CS4245 Codec Driver
	// fmt = 	SND_SOC_DAIFMT_I2S |	/* I2S mode */
 //    	 	SND_SOC_DAIFMT_CBM_CFS;	// CS4245: DAC master ADC slave (ALSA: codec clk master & frame slave).

	// //call cs4245_set_dai_fmt (CS4245 Codec Driver) - CODEC DAI.
	// ret = snd_soc_dai_set_fmt(codec_dai, fmt);	// Calls snd_soc_dai_driver .ops->set_fmt
	// if (ret < 0)
	// 	return ret;

	// //call cs4245_set_dai_sysclk (CS4245 Codec Driver) - CODEC DAI.
	// ret = snd_soc_dai_set_sysclk(codec_dai, CS4245_MCLK1_SET , mclk, 0);
	// if (ret < 0)
	// 	return ret;

	// // Initialize the I2S Plataform Driver
	// fmt = 	SND_SOC_DAIFMT_CBS_CFS |					// SoC clk & frm slave.
	// 		SND_SOC_DAIFMT_I2S |						// I2S mode
	// 		SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_32BCLK |	// Word Size = 32.
	// 		SND_SOC_DAIFMT_NB_NF;						// normal bit clock + frame.

	// //call sunxi_i2s_set_fmt (I2S Plataform Driver)- CPU DAI.
	// ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	// if (ret < 0)
	// 	return ret;

	return 0;
}

/* 
* TODO: Function description. 
*/
static void mod_duo_shutdown(struct snd_pcm_substream *substream)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	return;
}

/* 
* TODO: Function description.
*/
static int mod_duo_analog_suspend(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	return 0;
}

/* 
* TODO: Function description.
*/
static int mod_duo_analog_resume(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	return 0;
}

/* 
* TODO: Function description.
*/
static int mod_duo_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

	printk("[MOD Duo Machine Driver]mod_duo_hw_params: codec_dai=(%s), cpu_dai=(%s).\n", codec_dai->name, cpu_dai->name);
	printk("MOD Duo Machine Driver]mod_duo_hw_params: channel num=(%d).\n", params_channels(params));
	printk("[MOD Duo Machine Driver]mod_duo_hw_params: sample rate=(%u).\n", params_rate(params));

	switch (params_format(params)) 
	{
	case SNDRV_PCM_FORMAT_S16_LE:
		printk("[MOD Duo Machine Driver]mod_duo_hw_params: format 16 bit.\n");
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		printk("[MOD Duo Machine Driver]mod_duo_hw_params: format 24 bit in 3 bytes.\n");
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		printk("[MOD Duo Machine Driver]mod_duo_hw_params: format 24 bit in 4 bytes.\n");
		break;
	default:
		printk("[MOD Duo Machine Driver]mod_duo_hw_params: Unsupported format (%d).\n", (int)params_format(params));
	}

	return 0;
}

/*
* TODO: Function description.
* Implemented to replace the calls made in mod_duo_startup.
* The mod_duo_startup and then cpu_dai and codec_dai *_set_fmt and *_set_sysclk were called every aplay, and the reconfiguration was redundant.
*/
int mod_duo_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt = 0;
	unsigned int mclk = 24576000;	// MOD Duo Soun Card has an 2457600Hz external clock
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	// Configure the CS4245 Codec Driver for MOD Duo Soundcard
	fmt = 	SND_SOC_DAIFMT_I2S |	/* I2S mode */
    	 	SND_SOC_DAIFMT_CBM_CFS;	// CS4245: DAC master ADC slave (ALSA: codec clk master & frame slave).
	//call cs4245_set_dai_fmt (CS4245 Codec Driver) - CODEC DAI.
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);	// Calls snd_soc_dai_driver .ops->set_fmt
	if (ret < 0)
		return ret;
	//call cs4245_set_dai_sysclk (CS4245 Codec Driver) - CODEC DAI.
	ret = snd_soc_dai_set_sysclk(codec_dai, CS4245_MCLK1_SET , mclk, 0);
	if (ret < 0)
		return ret;
	// Configure the I2S Plataform Driver for MOD Duo Soundcard
	fmt = 	SND_SOC_DAIFMT_CBS_CFS |					// SoC clk & frm slave.
			SND_SOC_DAIFMT_I2S |						// I2S mode
			SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_32BCLK |	// Word Size = 32.
			SND_SOC_DAIFMT_NB_NF;						// normal bit clock + frame.
	//call sunxi_i2s_set_fmt (I2S Plataform Driver)- CPU DAI.
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0)
		return ret;
	return ret;
}

/* 
* TODO: Structure description.
*/
static struct snd_soc_ops mod_duo_ops = {
	.startup = mod_duo_startup,
	.shutdown = mod_duo_shutdown,
	.hw_params = mod_duo_hw_params,
};

/* 
* TODO: Structure description.
*/
static struct snd_soc_dai_link snd_soc_mod_duo_dai =
{
	.name = "MOD-DUO-I2S",
	.stream_name = "MOD-DUO-SUNXI-I2S",
	.cpu_dai_name = "sunxi-i2s.0",
	.codec_dai_name = "cs4245-dai",
	.platform_name = "sunxi-i2s-pcm-audio.0",
	.codec_name	= "cs4245-codec.1-004c",
	.ops = &mod_duo_ops,
	.init = &mod_duo_dai_link_init,
};

/* 
* TODO: Structure description.
*/
static struct snd_soc_card snd_soc_mod_duo_soundcard = {
	.name = "MOD-Duo-Sound-Card",
	.owner = THIS_MODULE,
	.dai_link = &snd_soc_mod_duo_dai,
	.num_links = 1,
	.suspend_post = mod_duo_analog_suspend,
	.resume_pre	= mod_duo_analog_resume,
};

// /*
// * TODO: Function description.
// */
// static int __devinit mod_duo_probe(struct platform_device *pdev)
// {
// 	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

// 	snd_soc_mod_duo_soundcard.dev = &pdev->dev;
// 	return snd_soc_register_card(&snd_soc_mod_duo_soundcard);
// }

// /*
// * TODO: Function description.
// */
// static int __devexit mod_duo_remove(struct platform_device *pdev)
// {
// 	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

// 	snd_soc_unregister_card(&snd_soc_sunxi_sndi2s);
// 	return 0;
// }

/*
* TODO: Structure description.
*/
static struct platform_device *mod_duo_audio_device;
// static struct platform_device mod_duo_audio_device = {
// 	.name = "MOD-Duo-Sound-Card",
// };

// /*
// *
// */
// static struct platform_driver mod_duo_audio_driver = {
// 	.probe = mod_duo_probe,
// 	.remove = __devexit_p(mod_duo_remove),
// 	.driver = {
// 		.name = "MOD-Duo-Sound-Card",
// 		.owner = THIS_MODULE,
// 	},
// };

/*
* TODO: Function description.
* When loading the module, the kernel sends a message saying to use snd_soc_register_card()
* "soc-audio soc-audio.0: ASoC machine MOD-Duo-Sound-Card should use snd_soc_register_card()"
*/
static int __init mod_duo_audio_init(void)
{
	int ret, i2s_used;
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	ret = script_parser_fetch("i2s_para", "i2s_used", &i2s_used, 1);
	if ((ret != 0) || (!i2s_used)){
		printk("[MOD Duo Machine Driver]I2S not configured on script.bin.\n");
		return -ENODEV;
	}
	ret = script_parser_fetch("mod_duo_souncard_para","mod_duo_souncard_used", &mod_duo_used, sizeof(int));
	if ((ret != 0) || (!mod_duo_used)) {
        printk("[MOD Duo Machine Driver]MOD Duo Sound Card not configured on script.bin.\n");
        return -ENODEV;
	}
	/* Register analog device */
	mod_duo_audio_device = platform_device_alloc("soc-audio", 0);	// TODO: Check memory integrity with variable "mod_duo_audio_device".
	if (!mod_duo_audio_device)
		return -ENOMEM;
	// else
	// 	printk("[MOD Duo Machine Driver] Plataform Device Allocated.\n");
	platform_set_drvdata(mod_duo_audio_device, &snd_soc_mod_duo_soundcard);
	ret = platform_device_add(mod_duo_audio_device);
	if (ret < 0) {
		platform_device_put(mod_duo_audio_device);
		return ret;
	}
	if(mod_duo_used) {
		mod_duo_gpio_init();
		mod_duo_set_impedance(CHANNEL_A, PAD_OFF);
		mod_duo_set_impedance(CHANNEL_B, PAD_OFF);
		mod_duo_set_impedance(CHANNEL_A, INSTRUMENT);
		mod_duo_set_impedance(CHANNEL_B, INSTRUMENT);
		mod_duo_set_bypass(CHANNEL_A, PROCESS);
		mod_duo_set_bypass(CHANNEL_B, PROCESS);
	}
	return 0;
}

/*
*
*/
static void __exit mod_duo_audio_exit(void)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

	if(mod_duo_used)
	{
		mod_duo_gpio_release();
	}
	platform_device_unregister(mod_duo_audio_device);
	return;
}

module_init(mod_duo_audio_init);
module_exit(mod_duo_audio_exit);

/* Module information */
MODULE_AUTHOR("Rafael Guayer <rafael@musicaloperatingdevices.com>");
MODULE_DESCRIPTION("MOD Duo Sound Card Audio Machine Driver");
MODULE_LICENSE("GPL");
