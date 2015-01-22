/*
 * mod-duo.c  --  SoC audio for MOD Duo audio device
 *
 * Copyright (c) 2015 Felipe Correa da Silva Sanches <juca@member.fsf.org>
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
#define STAGE_GAIN_OFF		3
#define STAGE_GAIN_ON		4

#define BYPASS 			0
#define PROCESS 		1

#define TURN_SWITCH_ON 0
#define TURN_SWITCH_OFF 1

#define I2C_ADDRESS	0b10011000	// 10011xx + R/!W

static int mod_duo_used = 0;
static u32 mod_duo_gpio_handler = 0;

#define MOD_DUO_GPIO_INIT(name)\
    err = script_parser_fetch("mod_duo_soundcard_para", "(name)", (int *) &info, sizeof (script_gpio_set_t));\
    if (err) {\
        printk(KERN_INFO "%s: can not get \"mod_duo_soundcard_para\" \"(name)\" gpio handler, already used by others?.", __FUNCTION__);\
        return -EBUSY;\
    }\
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "(name)");

/*
* GPIO Initialization
*/
static int mod_duo_gpio_init(void)
{
	int err;
	script_gpio_set_t info;

	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

	mod_duo_gpio_handler = gpio_request_ex("mod_duo_soundcard_para", NULL);

	// JFET Switch A Pin Configuration
	MOD_DUO_GPIO_INIT(jfet_sw_a1_pin)
	MOD_DUO_GPIO_INIT(jfet_sw_a2_pin)
	MOD_DUO_GPIO_INIT(jfet_sw_a3_pin)
	MOD_DUO_GPIO_INIT(jfet_sw_a4_pin)

	// JFET Switch B Pin Configuration
	MOD_DUO_GPIO_INIT(jfet_sw_b1_pin)
	MOD_DUO_GPIO_INIT(jfet_sw_b2_pin)
	MOD_DUO_GPIO_INIT(jfet_sw_b3_pin)
	MOD_DUO_GPIO_INIT(jfet_sw_b4_pin)

	// Overflow Leds Pin Configuration
	MOD_DUO_GPIO_INIT(led_ovfl1_pin)
	MOD_DUO_GPIO_INIT(led_ovfl2_pin)
	MOD_DUO_GPIO_INIT(led_ovfl3_pin)
	MOD_DUO_GPIO_INIT(led_ovfl4_pin)

	// Headphone Volume Control Pin Configuration
	// TODO: Create a separate driver for Headphone Amplifier (LM4811).
	MOD_DUO_GPIO_INIT(hp_vol_pin)
	MOD_DUO_GPIO_INIT(hp_clk_pin)

	// True Bypass Control Pin Configuration
	MOD_DUO_GPIO_INIT(bypass_a_pin)
	MOD_DUO_GPIO_INIT(bypass_b_pin)

	printk("[MOD Duo Machine Driver]GPIOs initialized.\n");
	return 0;
}

static void mod_duo_gpio_release(void)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	gpio_release(mod_duo_gpio_handler, 2);
	printk("[MOD Duo Machine Driver] GPIOs released.\n");
	return;
}

static void mod_duo_set_impedance(int channel, int type)
{
	switch(channel)
	{
		case CHANNEL_A:
		{
			switch(type)
			{
				case INSTRUMENT:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a2_pin");
					break;
				case LINE:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_a1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a2_pin");
					break;
				case MICROPHONE:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_a2_pin");
					break;
				case STAGE_GAIN_OFF:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a4_pin");
					break;
				case STAGE_GAIN_ON:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_a4_pin");
					break;
			}
			break;
		}
		case CHANNEL_B:
		{
			switch(type)
			{
				case INSTRUMENT:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b2_pin");
					break;
				case LINE:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_b1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b2_pin");
					break;
				case MICROPHONE:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_b2_pin");
					break;
				case STAGE_GAIN_OFF:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b4_pin");
					break;
				case STAGE_GAIN_ON:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_b4_pin");
					break;
			}
			break;
		}
	}
	return;
}

static void mod_duo_set_bypass(int channel, bool state)
{
// state == BYPASS:
// 	No audio processing.
//	Input is connected directly to output, bypassing the codec.
//
// state == PROCESS:
// 	INPUT => CODEC => OUTPUT

	switch(channel)
	{
		case CHANNEL_A:
			gpio_write_one_pin_value(mod_duo_gpio_handler, state, "bypass_a_pin"); break;
		case CHANNEL_B:
			gpio_write_one_pin_value(mod_duo_gpio_handler, state, "bypass_b_pin"); break;
	}
	return;
}

/* 
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
	// unsigned int mclk = 24576000;	// MOD Duo Sound Card has an 2457600Hz external clock

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

static void mod_duo_shutdown(struct snd_pcm_substream *substream)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	return;
}

static int mod_duo_analog_suspend(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	return 0;
}

static int mod_duo_analog_resume(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);
	return 0;
}

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
* Implemented to replace the calls made in mod_duo_startup.
* The mod_duo_startup and then cpu_dai and codec_dai *_set_fmt and *_set_sysclk were called every aplay, and the reconfiguration was redundant.
*/
int mod_duo_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt = 0;
	unsigned int mclk = 24576000;	// MOD Duo Sound Card has an 2457600Hz external clock
	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

	// Configure the CS4245 Codec Driver for MOD Duo Sound Card
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


	// Configure the I2S Plataform Driver for MOD Duo Sound Card
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

static struct snd_soc_ops mod_duo_ops = {
	.startup = mod_duo_startup,
	.shutdown = mod_duo_shutdown,
	.hw_params = mod_duo_hw_params,
};

static struct snd_soc_dai_link snd_soc_mod_duo_dai_link =
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

static struct snd_soc_card snd_soc_mod_duo_soundcard = {
	.name = "MOD-Duo-Sound-Card",
	.owner = THIS_MODULE,
	.dai_link = &snd_soc_mod_duo_dai_link,
	.num_links = 1,
	.suspend_post = mod_duo_analog_suspend,
	.resume_pre	= mod_duo_analog_resume,
};

// static int __devinit mod_duo_probe(struct platform_device *pdev)
// {
// 	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

// 	snd_soc_mod_duo_soundcard.dev = &pdev->dev;
// 	return snd_soc_register_card(&snd_soc_mod_duo_soundcard);
// }

// static int __devexit mod_duo_remove(struct platform_device *pdev)
// {
// 	printk("[MOD Duo Machine Driver]Entered %s.\n", __func__);

// 	snd_soc_unregister_card(&snd_soc_sunxi_sndi2s);
// 	return 0;
// }

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
	ret = script_parser_fetch("mod_duo_soundcard_para","mod_duo_soundcard_used", &mod_duo_used, sizeof(int));
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
		mod_duo_set_impedance(CHANNEL_A, STAGE_GAIN_OFF);
		mod_duo_set_impedance(CHANNEL_B, STAGE_GAIN_OFF);
		mod_duo_set_impedance(CHANNEL_A, INSTRUMENT);
		mod_duo_set_impedance(CHANNEL_B, INSTRUMENT);
		mod_duo_set_bypass(CHANNEL_A, PROCESS);
		mod_duo_set_bypass(CHANNEL_B, PROCESS);
	}
	return 0;
}

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

MODULE_AUTHOR("Felipe Sanches <juca@members.fsf.org>, Rafael Guayer <rafael@musicaloperatingdevices.com>");
MODULE_DESCRIPTION("MOD Duo Sound Card Audio Machine Driver");
MODULE_LICENSE("GPL");
