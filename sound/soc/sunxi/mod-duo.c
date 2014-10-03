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

// #include <linux/platform_device.h>
// #include <linux/module.h>
// //#include <linux/i2c.h>
// #include <linux/delay.h>
// #include <linux/gpio.h>
// #include <sound/pcm.h>
// #include <sound/soc.h>
// #include <asm/mach-types.h>

// #include "sunxi-i2s.h"
// #include "sunxi-i2sdma.h"
// #include "../codecs/cs4245.h"

// // Includes for Machine Driver
// #include <linux/delay.h>
// #include <linux/gpio.h>
// #include <linux/mutex.h>
// #include <linux/platform_device.h>
// #include <linux/clk.h>
// #include <linux/module.h>
// #include <sound/core.h>
// #include <sound/jack.h>
// #include <sound/pcm.h>
// #include <sound/soc.h>
// #include <sound/soc-dapm.h>
// //#include <mach/sys_config.h>
// #include <linux/io.h>
// #include <sound/pcm_params.h>

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

// Includes for GPIO in Kernel Space - from "dht22-sun5i"
// #include <linux/kernel.h>
// #include <linux/init.h>
// #include <linux/slab.h>
// #include <linux/proc_fs.h>
// #include <asm/uaccess.h> 
// #include <linux/timer.h>
// #include <linux/delay.h>
// #include <asm/delay.h>

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

// GPIO Handlers
// static unsigned * jfet_swa_gpio_handler[4];
// static unsigned * jfet_swb_gpio_handler[4];
// static unsigned * led_ovfl_gpio_handler[4];
// static unsigned * codec_rst_gpio_handler;
// static unsigned * hp_gpio_handler[2];
// static unsigned * bypass_gpio_handler[2];

static u32 mod_duo_gpio_handler = 0;

// Pin Names
// jfet_sw_a1_pin
// jfet_sw_a2_pin
// jfet_sw_a3_pin
// jfet_sw_b4_pin
// jfet_sw_b1_pin
// jfet_sw_b2_pin
// jfet_sw_b3_pin
// jfet_sw_b4_pin
// led_ovfl1_pin
// led_ovfl2_pin
// led_ovfl3_pin
// led_ovfl4_pin
// codec_rst_pin
// hp_vol_pin
// hp_clk_pin
// bypass_a_pin
// bypass_b_pin

/*
* GPIO Initialization - TODO: Function description.
*/
static int mod_duo_gpio_init(void)
{
	int err;
	script_gpio_set_t info;

	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	mod_duo_gpio_handler = gpio_request_ex("mod_duo_souncard_para", NULL);
// JFET Switch A Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a1_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a1_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // jfet_swa_gpio_handler[0] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_a1_pin");
	// gpio_set_one_pin_io_status(jfet_swa_gpio_handler[0], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a1_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a2_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a2_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // jfet_swa_gpio_handler[1] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_a2_pin");
	// gpio_set_one_pin_io_status(jfet_swa_gpio_handler[1], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a2_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a3_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a3_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
	// jfet_swa_gpio_handler[2] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_a3_pin");
	// gpio_set_one_pin_io_status(jfet_swa_gpio_handler[2], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a3_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_a4_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_a4_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
	// jfet_swa_gpio_handler[3] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_a4_pin");
	// gpio_set_one_pin_io_status(jfet_swa_gpio_handler[3], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_a4_pin");

// JFET Switch B Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b1_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b1_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // jfet_swb_gpio_handler[0] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_b1_pin");
	// gpio_set_one_pin_io_status(jfet_swb_gpio_handler[0], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b1_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b2_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b2_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // jfet_swb_gpio_handler[1] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_b2_pin");
	// gpio_set_one_pin_io_status(jfet_swb_gpio_handler[1], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b2_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b3_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b3_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
	// jfet_swb_gpio_handler[2] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_b3_pin");
	// gpio_set_one_pin_io_status(jfet_swb_gpio_handler[2], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b3_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "jfet_sw_b4_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"jfet_sw_b4_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
	// jfet_swb_gpio_handler[3] = gpio_request_ex("mod_duo_souncard_para", "jfet_sw_b4_pin");
	// gpio_set_one_pin_io_status(jfet_swb_gpio_handler[3], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "jfet_sw_b4_pin");

// Overflow Leds Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl1_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl1_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // led_ovfl_gpio_handler[0] = gpio_request_ex("mod_duo_souncard_para", "led_ovfl1_pin");
	// gpio_set_one_pin_io_status(led_ovfl_gpio_handler[0], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl1_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl2_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl2_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // led_ovfl_gpio_handler[1] = gpio_request_ex("mod_duo_souncard_para", "led_ovfl2_pin");
	// gpio_set_one_pin_io_status(led_ovfl_gpio_handler[1], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl2_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl3_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl3_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // led_ovfl_gpio_handler[2] = gpio_request_ex("mod_duo_souncard_para", "led_ovfl3_pin");
	// gpio_set_one_pin_io_status(led_ovfl_gpio_handler[2], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl3_pin");

	err = script_parser_fetch("mod_duo_souncard_para", "led_ovfl4_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"led_ovfl4_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // led_ovfl_gpio_handler[3] = gpio_request_ex("mod_duo_souncard_para", "led_ovfl4_pin");
	// gpio_set_one_pin_io_status(led_ovfl_gpio_handler[3], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "led_ovfl4_pin");

// Headphone Volume Control Pin Configuration - TODO: Create a separate driver for Headphone Aomplifier (LM4811).
    err = script_parser_fetch("mod_duo_souncard_para", "hp_vol_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"hp_vol_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // hp_gpio_handler[0] = gpio_request_ex("mod_duo_souncard_para", "hp_vol_pin");
	// gpio_set_one_pin_io_status(hp_gpio_handler[0], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "hp_vol_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "hp_clk_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"hp_clk_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // hp_gpio_handler[1] = gpio_request_ex("mod_duo_souncard_para", "hp_clk_pin");
	// gpio_set_one_pin_io_status(hp_gpio_handler[1], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "hp_clk_pin");

// True Bypass Control Pin Configuration
    err = script_parser_fetch("mod_duo_souncard_para", "bypass_a_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"bypass_a_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // bypass_gpio_handler[0] = gpio_request_ex("mod_duo_souncard_para", "bypass_a_pin");
	// gpio_set_one_pin_io_status(bypass_gpio_handler[0], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "bypass_a_pin");

    err = script_parser_fetch("mod_duo_souncard_para", "bypass_b_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"bypass_b_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // bypass_gpio_handler[1] = gpio_request_ex("mod_duo_souncard_para", "bypass_b_pin");
	// gpio_set_one_pin_io_status(bypass_gpio_handler[1], PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "bypass_b_pin");

// Codec Reset Pin Configuration
	err = script_parser_fetch("mod_duo_souncard_para", "codec_rst_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (err) {
        printk(KERN_INFO "%s: can not get \"mod_duo_souncard_para\" \"codec_rst_pin\" gpio handler, already used by others?", __FUNCTION__);
        return -EBUSY;
    }
    // codec_rst_gpio_handler[1] = gpio_request_ex("mod_duo_souncard_para", "codec_rst_pin");
	// gpio_set_one_pin_io_status(codec_rst_gpio_handler, PIN_DIR_OUT, NULL);

	gpio_set_one_pin_io_status(mod_duo_gpio_handler, PIN_DIR_OUT, "codec_rst_pin");

	printk("[MOD Duo Machine Driver] GPIOs initialized.\n");

	return 0;
}

/*
* TODO: Function description.
*/
static void mod_duo_gpio_release(void)
{
	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	// gpio_free(GPIO_CODEC_RESET); - TODO: Device Tree implementation for mainline Kernel.
	//gpio_release(jfet_swa_gpio_handler[0], 0);
	// gpio_release(jfet_swa_gpio_handler[1], 0);
	// gpio_release(jfet_swa_gpio_handler[2], 0);
	// gpio_release(jfet_swa_gpio_handler[3], 0);

	// gpio_release(jfet_swb_gpio_handler[0], 0);
	// gpio_release(jfet_swb_gpio_handler[1], 0);
	// gpio_release(jfet_swb_gpio_handler[2], 0);
	// gpio_release(jfet_swb_gpio_handler[3], 0);

	// gpio_release(led_ovfl_gpio_handler[0], 0);
	// gpio_release(led_ovfl_gpio_handler[1], 0);
	// gpio_release(led_ovfl_gpio_handler[2], 0);
	// gpio_release(led_ovfl_gpio_handler[3], 0);

	// gpio_release(codec_rst_gpio_handler, 0);

	// gpio_release(hp_gpio_handler[0], 0);
	// gpio_release(hp_gpio_handler[1], 0);

	// gpio_release(bypass_gpio_handler[0], 0);
	// gpio_release(bypass_gpio_handler[1], 0);

	gpio_release(mod_duo_gpio_handler, 2);

	printk("[MOD Duo Machine Driver] GPIOs released.\n");

	return;
}

/*
* TODO: Function description.
*/
static void mod_duo_set_impedance(int channel, int type)
{
	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	switch(channel)
	{
		case CHANNEL_A:	// Channel A
		{
			switch(type)
			{
				case INSTRUMENT:	// Instrument impedance
				{
					// gpio_set_value(GPIO_SW1_A, 1);	
					// gpio_set_value(GPIO_SW2_A, 1);	
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[0], 1, "jfet_sw_a1_pin"); // 1: Switch is OFF
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[1], 1, "jfet_sw_a2_pin"); // 0: Switch is ON

					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a1_pin"); // 1: Switch is OFF
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a2_pin"); // 0: Switch is ON

					break;
				}
				case LINE:			// Line impedance
				{
					// gpio_set_value(GPIO_SW1_A, 0);
					// gpio_set_value(GPIO_SW2_A, 1);
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[0], 0, "jfet_sw_a1_pin");
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[1], 1, "jfet_sw_a2_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a2_pin");

					break;
				}
				case MICROPHONE:	// Microphone impedance
				{
					// gpio_set_value(GPIO_SW1_A, 1);
					// gpio_set_value(GPIO_SW2_A, 0);
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[0], 1, "jfet_sw_a1_pin");
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[1], 0, "jfet_sw_a2_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a2_pin");

					break;
				}
				case PAD_OFF:		// Gain Stage OFF
				{
					// gpio_set_value(GPIO_SW3_A, 1);
					// gpio_set_value(GPIO_SW4_A, 0);
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[2], 0, "jfet_sw_a3_pin");
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[3], 1, "jfet_sw_a4_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a4_pin");

					break;
				}

				case PAD_ON:			// Gain Stage ON
				{
					// gpio_set_value(GPIO_SW3_A, 0);
					// gpio_set_value(GPIO_SW4_A, 1);
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[2], 1, "jfet_sw_a3_pin");
					// gpio_write_one_pin_value(jfet_swa_gpio_handler[3], 0, "jfet_sw_a4_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_a3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_a4_pin");

					break;
				}

			}
			break;
		}

		case CHANNEL_B:	// Channel B
		{
			switch(type)
			{
				case INSTRUMENT:	// Instrument impedance
				{
					// gpio_set_value(GPIO_SW1_B, 1);	
					// gpio_set_value(GPIO_SW2_B, 1);	
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[0], 1, "jfet_sw_b1_pin"); // 1: Switch is OFF
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[1], 1, "jfet_sw_b2_pin"); // 0: Switch is ON

					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b1_pin"); // 1: Switch is OFF
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b2_pin"); // 0: Switch is ON

					break;
				}
				case LINE:			// Line impedance
				{
					// gpio_set_value(GPIO_SW1_B, 0);
					// gpio_set_value(GPIO_SW2_B, 1);
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[0], 0, "jfet_sw_b1_pin");
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[1], 1, "jfet_sw_b2_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b2_pin");

					break;
				}
				case MICROPHONE:	// Microphone impedance
				{
					// gpio_set_value(GPIO_SW1_B, 1);
					// gpio_set_value(GPIO_SW2_B, 0);
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[0], 1, "jfet_sw_b1_pin");
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[1], 0, "jfet_sw_b2_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b1_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b2_pin");

					break;
				}
				case PAD_OFF:		// Gain Stage OFF
				{
					// gpio_set_value(GPIO_SW3_B, 1);
					// gpio_set_value(GPIO_SW4_B, 0);
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[2], 0, "jfet_sw_b3_pin");
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[3], 1, "jfet_sw_b4_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b4_pin");

					break;
				}

				case PAD_ON:			// Gain Stage ON
				{
					// gpio_set_value(GPIO_SW3_B, 0);
					// gpio_set_value(GPIO_SW4_B, 1);
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[2], 1, "jfet_sw_b3_pin");
					// gpio_write_one_pin_value(jfet_swb_gpio_handler[3], 0, "jfet_sw_b4_pin");

					gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "jfet_sw_b3_pin");
					gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "jfet_sw_b4_pin");

					break;
				}
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
	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	switch(channel)
	{
		case CHANNEL_A:	// Channel A
		{
			if(en == PROCESS)	// Bypass OFF: Pin = HIGH - Processing. Input jack connected to Ccodec.
//				gpio_write_one_pin_value(bypass_gpio_handler[0], PROCESS, "bypass_a_pin");
				gpio_write_one_pin_value(mod_duo_gpio_handler, PROCESS, "bypass_a_pin");
			else	// Bypass ON: Pin = LOW - Not processing. Input jack connected to output jack.
				// gpio_write_one_pin_value(bypass_gpio_handler[0], BYPASS, "bypass_a_pin");
				gpio_write_one_pin_value(mod_duo_gpio_handler, BYPASS, "bypass_a_pin");			
			break;
		}
		case CHANNEL_B:	// Channel B
		{
			if(en == PROCESS)	// Bypass OFF: Pin = HIGH - Processing. Input jack connected to Ccodec.
				// gpio_write_one_pin_value(bypass_gpio_handler[1], PROCESS, "bypass_b_pin");
				gpio_write_one_pin_value(mod_duo_gpio_handler, PROCESS, "bypass_b_pin");
			else	// Bypass ON: Pin = LOW - Not processing. Input jack connected to output jack.
				// gpio_write_one_pin_value(bypass_gpio_handler[1], BYPASS, "bypass_b_pin");
				gpio_write_one_pin_value(mod_duo_gpio_handler, BYPASS, "bypass_b_pin");
			break;
		}
	}
	return;
}

// /*
// * TODO: Function description.
// */
// static void mod_duo_enable_audio(bool en)
// {
// 	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

// 	if (en) 
// 		// gpio_write_one_pin_value(codec_rst_gpio_handler, 1, "codec_rst_pin");
// 		gpio_write_one_pin_value(mod_duo_gpio_handler, 1, "codec_rst_pin");
// 	else 
// 		// gpio_write_one_pin_value(codec_rst_gpio_handler, 0, "codec_rst_pin");
// 		gpio_write_one_pin_value(mod_duo_gpio_handler, 0, "codec_rst_pin");
// 	return;
// }

/* 
* TODO: Function description.
*/
static int mod_duo_startup(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt = 0;
	unsigned int mclk = 24576000;	// MOD Duo Soun Card has an 2457600Hz external clock

	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	// Initialize the CS4245 Codec Driver
	fmt = 	SND_SOC_DAIFMT_I2S |		/* I2S mode */
    	 	SND_SOC_DAIFMT_CBS_CFS;	/* codec clk & FRM slave  */

	//call cs4245_set_dai_fmt (CS4245 Codec Driver) - CODEC DAI.
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);	// Calls snd_soc_dai_driver .ops->set_fmt
	if (ret < 0)
		return ret;

	//call cs4245_set_dai_sysclk (CS4245 Codec Driver) - CODEC DAI.
	ret = snd_soc_dai_set_sysclk(codec_dai, 0 , mclk, 0);
	if (ret < 0)
		return ret;

	// Initialize the I2S Plataform Driver
	fmt = 	SUNXI_IISCTL_MS |		// codec clk & frm slave.
			SND_SOC_DAIFMT_I2S |	// I2S mode
			SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_32BCLK |	// Word Size = 32.
			SND_SOC_DAIFMT_NB_NF;	// normal bit clock + frame.

	//call sunxi_i2s_set_fmt (I2S Plataform Driver)- CPU DAI.
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0)
		return ret;

	mod_duo_set_impedance(CHANNEL_A, INSTRUMENT);
	mod_duo_set_impedance(CHANNEL_B, INSTRUMENT);
	mod_duo_set_bypass(CHANNEL_A, 0);
	mod_duo_set_bypass(CHANNEL_B, 0);
//	mod_duo_enable_audio(true);

	return ret;
}

/* 
* TODO: Function description. 
*/
static void mod_duo_shutdown(struct snd_pcm_substream *substream)
{
	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

//	mod_duo_enable_audio(false);	// CS4245 in reset.
}

/* 
* TODO: Function description.
*/
static int mod_duo_analog_suspend(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

//	mod_duo_enable_audio(false);	// CS4245 in reset.
	return 0;
}

/* 
* TODO: Function description.
*/
static int mod_duo_analog_resume(struct snd_soc_card *card)
{
	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

//	mod_duo_enable_audio(true);		// CS4245 operational.
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

	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	printk("[IIS-0] mod_duo_cs4245_hw_params: codec_dai=(%s), cpu_dai=(%s)\n", codec_dai->name, cpu_dai->name);
	printk("[IIS-0] mod_duo_cs4245_hw_params: channel num=(%d)\n", params_channels(params));
	printk("[IIS-0] mod_duo_cs4245_hw_params: sample rate=(%u)\n", params_rate(params));

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
* 
*/
static struct snd_soc_ops mod_duo_ops = {
	.startup = mod_duo_startup,
	.shutdown = mod_duo_shutdown,
	.hw_params = mod_duo_hw_params,
};

/* 
* 
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
	int ret, i2s_used;

	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	ret = script_parser_fetch("i2s_para", "i2s_used", &i2s_used, 1);
	if (ret != 0 || !i2s_used)
		return -ENODEV;

	/* Register analog device */
	mod_duo_audio_device = platform_device_alloc("soc-audio", 0);
	if (!mod_duo_audio_device)
		return -ENOMEM;
	else
		printk("[MOD Duo Machine Driver] Plataform Device Allocated\n");

//	platform_set_drvdata(mod_duo_audio_device, &snd_soc_mod_duo_dai);
	platform_set_drvdata(mod_duo_audio_device, &snd_soc_mod_duo_soundcard);

	ret = platform_device_add(mod_duo_audio_device);
	if (ret < 0) {
		platform_device_put(mod_duo_audio_device);
		return ret;
	}

	ret = script_parser_fetch("mod_duo_souncard_para","mod_duo_souncard_used", &mod_duo_used, sizeof(int));
	if (ret) {
        	printk("[MOD Duo Machine Driver] MOD Duo Sound Card not used.\n");
	}

	if(mod_duo_used)
	{
		mod_duo_gpio_init();
		mod_duo_set_impedance(CHANNEL_A, PAD_OFF);
		mod_duo_set_impedance(CHANNEL_B, PAD_OFF);
		mod_duo_set_impedance(CHANNEL_A, INSTRUMENT);
		mod_duo_set_impedance(CHANNEL_B, INSTRUMENT);
		mod_duo_set_bypass(CHANNEL_A, 0);
		mod_duo_set_bypass(CHANNEL_B, 0);
		// mod_duo_enable_audio(true);
	}
	return 0;
}

static void __exit mod_duo_audio_exit(void)
{
	printk("[MOD Duo Machine Driver]Entered %s\n", __func__);

	if(mod_duo_used)
	{
//		mod_duo_enable_audio(false);
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
