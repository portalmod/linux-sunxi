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
#include "../codecs/cs4245.h"

// GPIO macros
#define PIN_DIR_OUT		1
#define CHANNEL_A		0
#define CHANNEL_B		1

#define INSTRUMENT 		0
#define LINE 			1
#define MICROPHONE 		2

#define GAIN_STAGE_OFF	0
#define GAIN_STAGE_ON	1

#define BYPASS 			0
#define PROCESS 		1

#define TURN_SWITCH_ON	0
#define TURN_SWITCH_OFF 1

//Default headphone volume is 11th step (of a total of 16) which corresponds to a 0dB gain.
//Each step corresponds to 3dB.
static int headphone_volume = 11;

static int input_left_impedance = 0;
static int input_right_impedance = 0;
static int input_left_gain_stage = 0;
static int input_right_gain_stage = 0;
static int left_true_bypass = 0;
static int right_true_bypass = 0;

static int mod_duo_used = 0;
static u32 mod_duo_gpio_handler = 0;

#define MOD_DUO_GPIO_INIT(name)\
    err = script_parser_fetch("mod_duo_soundcard_para", name, (int *) &info, sizeof (script_gpio_set_t));\
    if (err) {\
        printk(KERN_INFO "%s: can not get \"mod_duo_soundcard_para\" \"" name "\" gpio handler, already used by others?.", __FUNCTION__);\
        return -EBUSY;\
    }\
	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, name);

/*
* GPIO Initialization
*/
static int mod_duo_gpio_init(void)
{
	int err;
	script_gpio_set_t info;

	printk("[MOD Duo Machine Driver] %s\n", __func__);

	mod_duo_gpio_handler = gpio_request_ex("mod_duo_soundcard_para", NULL);

	// JFET Switch A Pin Configuration
	MOD_DUO_GPIO_INIT("jfet_sw_a1_pin")
	MOD_DUO_GPIO_INIT("jfet_sw_a2_pin")
	MOD_DUO_GPIO_INIT("jfet_sw_a3_pin")
	MOD_DUO_GPIO_INIT("jfet_sw_a4_pin")

	// JFET Switch B Pin Configuration
	MOD_DUO_GPIO_INIT("jfet_sw_b1_pin")
	MOD_DUO_GPIO_INIT("jfet_sw_b2_pin")
	MOD_DUO_GPIO_INIT("jfet_sw_b3_pin")
	MOD_DUO_GPIO_INIT("jfet_sw_b4_pin")

	// Overflow Leds Pin Configuration
	MOD_DUO_GPIO_INIT("led_ovfl1_pin")
	MOD_DUO_GPIO_INIT("led_ovfl2_pin")
	MOD_DUO_GPIO_INIT("led_ovfl3_pin")
	MOD_DUO_GPIO_INIT("led_ovfl4_pin")

	// Headphone Volume Control Pin Configuration
	// TODO: Create a separate driver for Headphone Amplifier (LM4811).
	MOD_DUO_GPIO_INIT("hp_vol_pin")
	MOD_DUO_GPIO_INIT("hp_clk_pin")

	// True Bypass Control Pin Configuration
	MOD_DUO_GPIO_INIT("bypass_a_pin")
	MOD_DUO_GPIO_INIT("bypass_b_pin")

	printk("[MOD Duo Machine Driver] GPIOs initialized.\n");
	return 0;
}

static void mod_duo_gpio_release(void)
{
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
			}
			input_left_impedance = type;
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
			}
			input_right_impedance = type;
			break;
		}
	}
	return;
}

static void mod_duo_set_gain_stage(int channel, int state)
{
	switch(channel)
	{
		case CHANNEL_A:
		{
			switch(state)
			{
				case GAIN_STAGE_OFF:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a4_pin");
					break;
				case GAIN_STAGE_ON:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_a4_pin");
					break;
			}
			input_left_gain_stage = state;
			break;
		}
		case CHANNEL_B:
		{
			switch(state)
			{
				case GAIN_STAGE_OFF:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b4_pin");
					break;
				case GAIN_STAGE_ON:
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "jfet_sw_b4_pin");
					break;
			}
			input_right_gain_stage = state;
			break;
		}
	}
	return;
}

static void mod_duo_set_true_bypass(int channel, bool state)
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
			gpio_write_one_pin_value(mod_duo_gpio_handler, state, "bypass_a_pin");
			left_true_bypass = state;
			break;
		case CHANNEL_B:
			gpio_write_one_pin_value(mod_duo_gpio_handler, state, "bypass_b_pin");
			right_true_bypass = state;
			break;
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

	printk("[MOD Duo Machine Driver] %s\n", __func__);

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
	printk("[MOD Duo Machine Driver] %s\n", __func__);
	return;
}

static int mod_duo_analog_suspend(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver] %s\n", __func__);
	return 0;
}

static int mod_duo_analog_resume(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver] %s\n", __func__);
	return 0;
}

/* This routine flips the GPIO pins to send the volume adjustment
   message to the actual headphone gain-control chip (LM4811) */
static void set_headphone_volume(int new_volume){
	int steps = new_volume - headphone_volume;
	int i;

	//select volume adjustment direction:
	gpio_write_one_pin_value(mod_duo_gpio_handler, steps > 0 ? TURN_SWITCH_ON : TURN_SWITCH_OFF, "hp_vol_pin");

	for (i=0; i<abs(steps); i++){
		//toggle clock in order to sample the volume pin upon clock's rising edge:
		gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_OFF, "hp_clk_pin");
		gpio_write_one_pin_value(mod_duo_gpio_handler, TURN_SWITCH_ON, "hp_clk_pin");
	}

	headphone_volume = new_volume;
}

static int headphone_info(struct snd_kcontrol *kcontrol,
						  struct snd_ctl_elem_info *uinfo)
{
      uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
      uinfo->count = 1;
      uinfo->value.integer.min = 0;
      uinfo->value.integer.max = 15;
      return 0;
}

static int headphone_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = headphone_volume;
	return 0;
}


static int headphone_put(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	if (headphone_volume != ucontrol->value.integer.value[0]) {
		set_headphone_volume(ucontrol->value.integer.value[0]);
		changed = 1;
	}
	return changed;
}

static struct snd_kcontrol_new headphone_control __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Headphone Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = headphone_info,
	.get = headphone_get,
	.put = headphone_put
};

//----------------------------------------------------------------------

static int input_left_impedance_info(struct snd_kcontrol *kcontrol,
						  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[3] = { "Instrument", "Line", "Mic" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;
	if (uinfo->value.enumerated.item > 2)
		uinfo->value.enumerated.item = 2;
	strcpy(uinfo->value.enumerated.name,
		texts[uinfo->value.enumerated.item]);
	return 0;
}

static int input_left_impedance_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = input_left_impedance;
	return 0;
}


static int input_left_impedance_put(struct snd_kcontrol *kcontrol,
									struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	if (input_left_impedance != ucontrol->value.integer.value[0]) {
		mod_duo_set_impedance(CHANNEL_A, ucontrol->value.integer.value[0]);
		changed = 1;
	}
	return changed;
}

//----------------------------------------------------------------------

static int input_right_impedance_info(struct snd_kcontrol *kcontrol,
						  struct snd_ctl_elem_info *uinfo)
{
	static char *texts[3] = { "Instrument", "Line", "Mic" };

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;
	if (uinfo->value.enumerated.item > 2)
		uinfo->value.enumerated.item = 2;
	strcpy(uinfo->value.enumerated.name,
		texts[uinfo->value.enumerated.item]);
	return 0;
}

static int input_right_impedance_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = input_right_impedance;
	return 0;
}


static int input_right_impedance_put(struct snd_kcontrol *kcontrol,
									struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	if (input_right_impedance != ucontrol->value.integer.value[0]) {
		mod_duo_set_impedance(CHANNEL_B, ucontrol->value.integer.value[0]);
		changed = 1;
	}
	return changed;
}

//----------------------------------------------------------------------

static int input_left_gain_stage_info(struct snd_kcontrol *kcontrol,
						  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
	return 0;
}

static int input_left_gain_stage_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = input_left_gain_stage;
	return 0;
}


static int input_left_gain_stage_put(struct snd_kcontrol *kcontrol,
									struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	if (input_left_gain_stage != ucontrol->value.integer.value[0]) {
		mod_duo_set_gain_stage(CHANNEL_A, ucontrol->value.integer.value[0]);
		changed = 1;
	}
	return changed;
}

//----------------------------------------------------------------------

static int input_right_gain_stage_info(struct snd_kcontrol *kcontrol,
						  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
	return 0;
}

static int input_right_gain_stage_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = input_right_gain_stage;
	return 0;
}


static int input_right_gain_stage_put(struct snd_kcontrol *kcontrol,
									struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	if (input_right_gain_stage != ucontrol->value.integer.value[0]) {
		mod_duo_set_gain_stage(CHANNEL_B, ucontrol->value.integer.value[0]);
		changed = 1;
	}
	return changed;
}

//----------------------------------------------------------------------

static int left_true_bypass_info(struct snd_kcontrol *kcontrol,
						  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
	return 0;
}

static int left_true_bypass_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = left_true_bypass;
	return 0;
}


static int left_true_bypass_put(struct snd_kcontrol *kcontrol,
									struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	if (left_true_bypass != ucontrol->value.integer.value[0]) {
		mod_duo_set_true_bypass(CHANNEL_A, ucontrol->value.integer.value[0]);
		changed = 1;
	}
	return changed;
}

//----------------------------------------------------------------------

static int right_true_bypass_info(struct snd_kcontrol *kcontrol,
						  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
	return 0;
}

static int right_true_bypass_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = right_true_bypass;
	return 0;
}


static int right_true_bypass_put(struct snd_kcontrol *kcontrol,
									struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	if (right_true_bypass != ucontrol->value.integer.value[0]) {
		mod_duo_set_true_bypass(CHANNEL_B, ucontrol->value.integer.value[0]);
		changed = 1;
	}
	return changed;
}

//----------------------------------------------------------------------


static struct snd_kcontrol_new input_left_impedance_control __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Capture Source",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = input_left_impedance_info,
	.get = input_left_impedance_get,
	.put = input_left_impedance_put
};

static struct snd_kcontrol_new input_right_impedance_control __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Capture Source",
	.index = 1,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = input_right_impedance_info,
	.get = input_right_impedance_get,
	.put = input_right_impedance_put
};

static struct snd_kcontrol_new input_left_gain_stage_control __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Left Gain Stage",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = input_left_gain_stage_info,
	.get = input_left_gain_stage_get,
	.put = input_left_gain_stage_put
};

static struct snd_kcontrol_new input_right_gain_stage_control __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Right Gain Stage",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = input_right_gain_stage_info,
	.get = input_right_gain_stage_get,
	.put = input_right_gain_stage_put
};

static struct snd_kcontrol_new left_true_bypass_control __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Left True-Bypass",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = left_true_bypass_info,
	.get = left_true_bypass_get,
	.put = left_true_bypass_put
};

static struct snd_kcontrol_new right_true_bypass_control __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Right True-Bypass",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = right_true_bypass_info,
	.get = right_true_bypass_get,
	.put = right_true_bypass_put
};

//----------------------------------------------------------------------

static int mod_duo_hw_params(struct snd_pcm_substream *substream,
							 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	printk("[MOD Duo Machine Driver] %s\n", __func__);

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
	printk("[MOD Duo Machine Driver] %s\n", __func__);

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

	return ret;
}

static struct snd_soc_ops mod_duo_ops = {
	.startup = mod_duo_startup,
	.shutdown = mod_duo_shutdown,
	.hw_params = mod_duo_hw_params,
};

static struct snd_soc_dai_link mod_duo_dai =
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
	.name = "MOD Duo",
	.owner = THIS_MODULE,
	.dai_link = &mod_duo_dai,
	.num_links = 1,
	.suspend_post = mod_duo_analog_suspend,
	.resume_pre	= mod_duo_analog_resume,
};

static int __devexit mod_duo_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if(mod_duo_used){
		mod_duo_gpio_release();
	}

	snd_soc_unregister_card(card);
	return 0;
}

static int __devinit mod_duo_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_mod_duo_soundcard;
	int ret, i2s_used;

	printk("[MOD Duo Machine Driver] %s\n", __func__);

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

	ret = snd_ctl_add(snd_soc_mod_duo_soundcard.snd_card, snd_ctl_new1(&headphone_control, NULL));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(snd_soc_mod_duo_soundcard.snd_card, snd_ctl_new1(&input_left_impedance_control, NULL));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(snd_soc_mod_duo_soundcard.snd_card, snd_ctl_new1(&input_right_impedance_control, NULL));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(snd_soc_mod_duo_soundcard.snd_card, snd_ctl_new1(&input_left_gain_stage_control, NULL));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(snd_soc_mod_duo_soundcard.snd_card, snd_ctl_new1(&input_right_gain_stage_control, NULL));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(snd_soc_mod_duo_soundcard.snd_card, snd_ctl_new1(&left_true_bypass_control, NULL));
	if (ret < 0)
		return ret;

	ret = snd_ctl_add(snd_soc_mod_duo_soundcard.snd_card, snd_ctl_new1(&right_true_bypass_control, NULL));
	if (ret < 0)
		return ret;

	if(mod_duo_used) {
		mod_duo_gpio_init();
		mod_duo_set_gain_stage(CHANNEL_A, GAIN_STAGE_OFF);
		mod_duo_set_gain_stage(CHANNEL_B, GAIN_STAGE_OFF);
		mod_duo_set_impedance(CHANNEL_A, INSTRUMENT);
		mod_duo_set_impedance(CHANNEL_B, INSTRUMENT);
		mod_duo_set_true_bypass(CHANNEL_A, PROCESS);
		mod_duo_set_true_bypass(CHANNEL_B, PROCESS);
	}

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",	ret);

	return ret;
}

static struct platform_driver mod_duo_audio_driver = {
	.driver		= {
		.name	= "mod-duo",
		.owner	= THIS_MODULE,
	},
	.probe		= mod_duo_audio_probe,
	.remove		= __devexit_p(mod_duo_audio_remove),
};

module_platform_driver(mod_duo_audio_driver);

/* Module information */
MODULE_AUTHOR("Felipe Sanches <juca@members.fsf.org>, Rafael Guayer <rafael@musicaloperatingdevices.com>");
MODULE_DESCRIPTION("MOD Duo Sound Card Audio Machine Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mod-duo");
