/*
 * sound\soc\sunxi\i2s\sunxi-i2s.c
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * chenpailin <chenpailin@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

/* Coments for commit
* - Find the clock divisors inside the Plataform Driver, not in the Machine Driver
* - Moved the channels/output enable, sample rate setting and sample format to *_hw_params, as discussed on #alsa-soc in freenode.
* - Finalized sunxi_i2s_set_fmt
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/clock.h>
#include <plat/system.h>
#include <plat/sys_config.h>

#include <mach/hardware.h>
#include <asm/dma.h>
#include <plat/dma_compat.h>

#include "sunxi-i2sdma.h"
#include "sunxi-i2s.h"

static u32 i2s_handler = 0;
static int regsave[8];
// Variables parsed from *.fex script file.
static int i2s_used = 1;	

// TODO: Initialize structure on probe, after register default configuration
//static struct sunxi_i2s_info sunxi_iis;
static struct sunxi_i2s_info sunxi_iis = {
	.slave = 0,
	.samp_fs = 48000,
	.samp_res = 24,
	.samp_format = 0,
	.ws_size = 32,
	.mclk_rate = 512,
	.lrc_pol = 0,
	.bclk_pol = 0,
	.pcm_datamode = 0,
	.pcm_sw = 0,
	.pcm_sync_period = 0,
	.pcm_sync_type = 0,
	.pcm_start_slot = 0,
	.pcm_lsb_first = 0,
	.pcm_ch_num = 1,
};

static struct clk *i2s_apbclk, *i2s_pll2clk, *i2s_pllx8, *i2s_moduleclk;

static struct sunxi_dma_params sunxi_i2s_pcm_stereo_out = {
	.client.name	=	"I2S PCM Stereo out",
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I
	.channel	=	DMACH_NIIS,
#endif
	.dma_addr 	=	SUNXI_IISBASE + SUNXI_IISTXFIFO,
};

static struct sunxi_dma_params sunxi_i2s_pcm_stereo_in = {
	.client.name	=	"I2S PCM Stereo in",
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I
	.channel	=	DMACH_NIIS,
#endif
	.dma_addr 	=	SUNXI_IISBASE + SUNXI_IISRXFIFO,
};

typedef struct __BCLK_SET_INF
{
    __u8        bitpersamp;     // bits per sample - Word Sizes
    __u8        clk_div;        // clock division
    __u16       mult_fs;        // multiplay of sample rate

} __bclk_set_inf;

typedef struct __MCLK_SET_INF
{
    __u32       samp_rate;      // sample rate
    __u16       mult_fs;        // multiply of smaple rate

    __u8        clk_div;        // mpll division
    __u32       mclk;          // select mpll, 24.576MHz/22.5792Mhz

} __mclk_set_inf;

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
    {  8000, 128, 24, 24576000}, {  8000, 192, 16, 24576000}, {  8000, 256, 12, 24576000},
    {  8000, 384,  8, 24576000}, {  8000, 512,  6, 24576000}, {  8000, 768,  4, 24576000},

    // 16k bitrate
    { 16000, 128, 12, 24576000}, { 16000, 192,  8, 24576000}, { 16000, 256,  6, 24576000},
    { 16000, 384,  4, 24576000}, { 16000, 768,  2, 24576000},

    // 32k bitrate
    { 32000, 128,  6, 24576000}, { 32000, 192,  4, 24576000}, { 32000, 384,  2, 24576000},
    { 32000, 768,  1, 24576000},

    // 64k bitrate
    { 64000, 192,  2, 24576000}, { 64000, 384,  1, 24576000},

    //128k bitrate
    {128000, 192,  1, 24576000},

    // 12k bitrate
    { 12000, 128, 16, 24576000}, { 12000, 256, 8, 24576000}, { 12000, 512, 4, 24576000},

    // 24k bitrate
    { 24000, 128,  8, 24576000}, { 24000, 256, 4, 24576000}, { 24000, 512, 2, 24576000},

    // 48K bitrate
    { 48000, 128,  4, 24576000}, { 48000, 256,  2, 24576000}, { 48000, 512, 1, 24576000},

    // 96k bitrate
    { 96000, 128 , 2, 24576000}, { 96000, 256,  1, 24576000},

    //192k bitrate
    {192000, 128,  1, 24576000},

    //11.025k bitrate
    { 11025, 128, 16, 22579200}, { 11205, 256,  8, 22579200}, { 11205, 512,  4, 22579200},

    //22.05k bitrate
    { 22050, 128,  8, 22579200}, { 22050, 256,  4, 22579200},
    { 22050, 512,  2, 22579200},

    //44.1k bitrate
    { 44100, 128,  4, 22579200}, { 44100, 256,  2, 22579200}, { 44100, 512,  1, 22579200},

    //88.2k bitrate
    { 88200, 128,  2, 22579200}, { 88200, 256,  1, 22579200},

    //176.4k bitrate
    {176400, 128, 1, 22579200},

    //end flag 0xffffffff
    {0xffffffff, 0, 0, 24576000},
};

/* 
* TODO: Function description.
*/
//static s32 get_clock_divder(u32 sample_rate, u32 sample_width, u32 * mclk_div, u32* mpll, u32* bclk_div, u32* mult_fs)
static s32 sunxi_i2s_divisor_values(u32 * mclk_div, u32* bclk_div, u32* mclk)
{
	u32 i, j, ret = -EINVAL;

	printk("[I2S]Entered %s\n", __func__);

	for(i=0; i< ARRAY_SIZE(MCLK_INF); i++) {
		 if((MCLK_INF[i].samp_rate == sunxi_iis.samp_fs) && ((MCLK_INF[i].mult_fs == 256) || (MCLK_INF[i].mult_fs == 128))) {
			  for(j=0; j<ARRAY_SIZE(BCLK_INF); j++) {
					if((BCLK_INF[j].bitpersamp == sunxi_iis.ws_size) && (BCLK_INF[j].mult_fs == MCLK_INF[i].mult_fs)) {
						 //set mclk and bclk division
						 *mclk_div = MCLK_INF[i].clk_div;
						 *mclk = MCLK_INF[i].mclk;
						 *bclk_div = BCLK_INF[j].clk_div;
						 sunxi_iis.mclk_rate = MCLK_INF[i].mult_fs;
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

// /*
// *
// */
// static void sunxi_i2s_printk_register_values(void)
// {
// 	int reg_val[8];

// 	reg_val[0] = readl(sunxi_iis.regs + SUNXI_IISCTL);
// 	reg_val[1] = readl(sunxi_iis.regs + SUNXI_IISFAT0);
// 	reg_val[2] = readl(sunxi_iis.regs + SUNXI_IISFAT1);
// 	reg_val[3] = readl(sunxi_iis.regs + SUNXI_IISFCTL);
// 	reg_val[4] = readl(sunxi_iis.regs + SUNXI_IISINT);
// 	reg_val[5] = readl(sunxi_iis.regs + SUNXI_IISCLKD);
// 	reg_val[6] = readl(sunxi_iis.regs + SUNXI_TXCHSEL);
// 	reg_val[7] = readl(sunxi_iis.regs + SUNXI_TXCHMAP);

// 	printk("[I2S]Register Values:\n");
// 	printk("[I2S]SUNXI_IISCTL: 0x%X.\n", reg_val[0]);
// 	printk("[I2S]SUNXI_IISFAT0: 0x%X.\n", reg_val[1]);
// 	printk("[I2S]SUNXI_IISFAT1: 0x%X.\n", reg_val[2]);
// 	printk("[I2S]SUNXI_IISFCTL: 0x%X.\n", reg_val[3]);
// 	printk("[I2S]SUNXI_IISINT: 0x%X.\n", reg_val[4]);
// 	printk("[I2S]SUNXI_IISCLKD: 0x%X.\n", reg_val[5]);
// 	printk("[I2S]SUNXI_TXCHSEL: 0x%X.\n", reg_val[6]);
// 	printk("[I2S]SUNXI_TXCHMAP: 0x%X.\n", reg_val[7]);

// 	return;
// }

/* 
* TODO: Function description.
*/
static void iisregsave(void)
{
	printk("[I2S]Entered %s\n", __func__);

	regsave[0] = readl(sunxi_iis.regs + SUNXI_IISCTL);
	regsave[1] = readl(sunxi_iis.regs + SUNXI_IISFAT0);
	regsave[2] = readl(sunxi_iis.regs + SUNXI_IISFAT1);
	regsave[3] = readl(sunxi_iis.regs + SUNXI_IISFCTL) | (0x3<<24); // TODO: Bit 24- FRX - Write ‘1’ to flush RX FIFO, self clear to ‘0’. Really needed?
	regsave[4] = readl(sunxi_iis.regs + SUNXI_IISINT);
	regsave[5] = readl(sunxi_iis.regs + SUNXI_IISCLKD);
	regsave[6] = readl(sunxi_iis.regs + SUNXI_TXCHSEL);
	regsave[7] = readl(sunxi_iis.regs + SUNXI_TXCHMAP);
}

/* 
* TODO: Function description.
*/
static void iisregrestore(void)
{
	printk("[I2S]Entered %s\n", __func__);

	writel(regsave[0], sunxi_iis.regs + SUNXI_IISCTL);
	writel(regsave[1], sunxi_iis.regs + SUNXI_IISFAT0);
	writel(regsave[2], sunxi_iis.regs + SUNXI_IISFAT1);
	writel(regsave[3], sunxi_iis.regs + SUNXI_IISFCTL);
	writel(regsave[4], sunxi_iis.regs + SUNXI_IISINT);
	writel(regsave[5], sunxi_iis.regs + SUNXI_IISCLKD);
	writel(regsave[6], sunxi_iis.regs + SUNXI_TXCHSEL);
	writel(regsave[7], sunxi_iis.regs + SUNXI_TXCHMAP);
}

/*
* TODO: Function Description
* TODO: Understand this function!
*/
void sunxi_snd_txctrl_i2s(int on)
{
	u32 reg_val;
	
	printk("[I2S]Entered %s\n", __func__);

	//flush TX FIFO
	reg_val = readl(sunxi_iis.regs + SUNXI_IISFCTL);
	reg_val |= SUNXI_IISFCTL_FTX;
	writel(reg_val, sunxi_iis.regs + SUNXI_IISFCTL);

	//clear TX counter
	writel(0, sunxi_iis.regs + SUNXI_IISTXCNT);

	if (on) {
		/* IIS TX ENABLE */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISCTL);
		reg_val |= SUNXI_IISCTL_TXEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISCTL);

		/* enable DMA DRQ mode for play */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISINT);
		reg_val |= SUNXI_IISINT_TXDRQEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISINT);
	} else {
		/* IIS TX DISABLE */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISCTL);
		reg_val &= ~SUNXI_IISCTL_TXEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISCTL);

		/* DISBALE dma DRQ mode */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISINT);
		reg_val &= ~SUNXI_IISINT_TXDRQEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISINT);
	}
	return;
}

//void sunxi_snd_rxctrl_i2s(int on)
void sunxi_snd_rxctrl_i2s(int on)
{
	u32 reg_val;

	printk("[I2S]Entered %s\n", __func__);

	//flush RX FIFO
	reg_val = readl(sunxi_iis.regs + SUNXI_IISFCTL);
	reg_val |= SUNXI_IISFCTL_FRX;
	writel(reg_val, sunxi_iis.regs + SUNXI_IISFCTL);

	//clear RX counter
	writel(0, sunxi_iis.regs + SUNXI_IISRXCNT);

	if (on) {
		/* IIS RX ENABLE */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISCTL);
		reg_val |= SUNXI_IISCTL_RXEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISCTL);

		/* enable DMA DRQ mode for record */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISINT);
		reg_val |= SUNXI_IISINT_RXDRQEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISINT);
	} else {
		/* IIS RX DISABLE */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISCTL);
		reg_val &= ~SUNXI_IISCTL_RXEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISCTL);

		/* DISABLE dma DRQ mode */
		reg_val = readl(sunxi_iis.regs + SUNXI_IISINT);
		reg_val &= ~SUNXI_IISINT_RXDRQEN;
		writel(reg_val, sunxi_iis.regs + SUNXI_IISINT);
	}
	return;
}

/*
* TODO: Function Description
* Saved in snd_soc_dai_ops sunxi_iis_dai_ops.
* Function called internally. The Machine Driver doesn't need to call this function because it is called whenever sunxi_i2s_set_clkdiv is called.
* The master clock in Allwinner SoM depends on the sampling frequency.
*/
static int sunxi_i2s_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id, unsigned int freq, int dir)
{
	u32 reg_val;
	printk("[I2S]Entered %s\n", __func__);
	if(!sunxi_iis.slave)
	{
		switch(clk_id)
		{
			case SUNXI_SET_MCLK:	// Set the master clock frequency.
				// TODO - Check if the master clock is needed when slave mode is selected.
				if (clk_set_rate(i2s_pll2clk, freq))
				{
					pr_err("Try to set the i2s_pll2clk failed!\n");	
					return -EINVAL;
				}
				break;
			case SUNXI_MCLKO_EN:	// Enables the master clock output
				reg_val = readl(sunxi_iis.regs + SUNXI_IISCLKD);
				if(dir == 1)	// Enable 
					reg_val |= (0x1<<7);
				if(dir == 0)	// Disable
					reg_val &= ~(0x1<<7);
				writel(reg_val, sunxi_iis.regs + SUNXI_IISCLKD);
			break;
		}
	}
	return 0;
}

/*
* TODO: Function Description
* Saved in snd_soc_dai_ops sunxi_iis_dai_ops.
*/
static int sunxi_i2s_set_clkdiv(struct snd_soc_dai *cpu_dai, int div_id, int value)
{
	u32 reg_val, reg_bk, ret;
	u32 mclk = 0;
	u32 mclk_div = 0;
	u32 bclk_div = 0;

	// Here i should know the sample rate and the FS multiple.

 	printk("[I2S]Entered %s\n", __func__);

 	switch (div_id) {
		case SUNXI_DIV_MCLK:	// Sets MCLKDIV
			reg_val = readl(sunxi_iis.regs + SUNXI_IISCLKD);
			reg_val &= ~(0xf<<0);
			reg_val |= ((value & 0xf)<<0);
			writel(reg_val, sunxi_iis.regs + SUNXI_IISCLKD);
			break;
		case SUNXI_DIV_BCLK:	// Sets BCLKDIV
			reg_val = readl(sunxi_iis.regs + SUNXI_IISCLKD);
			reg_val &= ~(0x7<<4);
			reg_val |= ((value & 0x7)<<4);
			writel(reg_val, sunxi_iis.regs + SUNXI_IISCLKD);
			break;
		case SUNXI_SAMPLING_FREQ:
			if(!sunxi_iis.slave)
			{
				reg_bk = sunxi_iis.samp_fs;
				sunxi_iis.samp_fs = (u32)value;
				ret = sunxi_i2s_divisor_values(&mclk_div, &bclk_div, &mclk);	// Get the register values
				if(ret != 0)	
				{
					printk("[I2S]Sampling rate frequency not supported.");
					sunxi_iis.samp_fs = reg_bk;
					return ret;
				}
				else
				{
					sunxi_iis.samp_fs = (u32)value;
					sunxi_i2s_set_sysclk(cpu_dai, SUNXI_SET_MCLK, mclk, 0);	// Set the master clock.
					reg_val = readl(sunxi_iis.regs + SUNXI_IISCLKD);
					reg_val &= ~(0x7f<<0);	// Clear MCLK and BCLK
					reg_val |= (((mclk_div & 0xf)<<0) | ((bclk_div & 0x7)<<4));
					writel(reg_val, sunxi_iis.regs + SUNXI_IISCLKD);
				}
			}
			else
				sunxi_iis.samp_fs = (u32)value;
			break;
	}
	return 0;
}

/*
* TODO: Funcition description.
* TODO: Refactor function because the configuration is with wrong scheme. Use a 4bit mask with the configuration option and then the value?
* TODO: Include TX and RX FIFO trigger levels.
* Saved in snd_soc_dai_ops sunxi_iis_dai_ops.
* Configure:
* - Master/Slave.
* - I2S/PCM mode.
* - Signal Inversion.
* - Word Select Size.
* - PCM Registers.
*/
static int sunxi_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	u32 reg_val1;
	u32 reg_val2;

	printk("[I2S]Entered %s\n", __func__);

	// Master/Slave Definition
	reg_val1 = readl(sunxi_iis.regs + SUNXI_IISCTL);
	switch(fmt & SND_SOC_DAIFMT_MASTER_MASK){
		case SND_SOC_DAIFMT_CBS_CFS:   // codec clk & frm slave
			reg_val1 |= SUNXI_IISCTL_MS; // 1: Slave!
			printk("[I2S]sunxi_i2s_set_fmt: Slave.\n");
			break;
		case SND_SOC_DAIFMT_CBM_CFM:   // codec clk & frm master
			reg_val1 &= ~SUNXI_IISCTL_MS; // 0: Master!
			printk("[I2S]sunxi_i2s_set_fmt: Master.\n");
			break;
		default:
			printk("[I2S]sunxi_i2s_set_fmt: Master-Slave Select unknown mode mode\n");
			return -EINVAL;
	}
	writel(reg_val1, sunxi_iis.regs + SUNXI_IISCTL);

	// I2S or PCM mode.
	reg_val1 = readl(sunxi_iis.regs + SUNXI_IISCTL);
	reg_val2 = readl(sunxi_iis.regs + SUNXI_IISFAT0);	// Register Name in User Manual V1.2: DA_FAT0 - Digital Audio Format Register 0
	switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK)
	{
		case SND_SOC_DAIFMT_I2S:        /* I2S mode */
			reg_val1 &= ~SUNXI_IISCTL_PCM;
			reg_val2 &= ~SUNXI_IISFAT0_FMT_RVD;	// Clear FMT (Bit 1:0)
			reg_val2 |= SUNXI_IISFAT0_FMT_I2S;
			printk("[I2S]sunxi_i2s_set_fmt: Set I2S mode\n");
			sunxi_iis.samp_format = SND_SOC_DAIFMT_I2S;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:    /* Right Justified mode */
			reg_val1 &= ~SUNXI_IISCTL_PCM;
			reg_val2 &= ~SUNXI_IISFAT0_FMT_RVD;	// Clear FMT (Bit 1:0)
			reg_val2 |= SUNXI_IISFAT0_FMT_RGT;
			printk("[I2S]sunxi_i2s_set_fmt: Set Right Justified mode\n");
			sunxi_iis.samp_format = SND_SOC_DAIFMT_RIGHT_J;
			break;
		case SND_SOC_DAIFMT_LEFT_J:     /* Left Justified mode */
			reg_val1 &= ~SUNXI_IISCTL_PCM;
			reg_val2 &= ~SUNXI_IISFAT0_FMT_RVD;	// Clear FMT (Bit 1:0)
			reg_val2 |= SUNXI_IISFAT0_FMT_LFT;
			printk("[I2S]sunxi_i2s_set_fmt: Set Left Justified mode\n");
			sunxi_iis.samp_format = SND_SOC_DAIFMT_LEFT_J;
			break;
		case SND_SOC_DAIFMT_DSP_A:      /* L data msb after FRM LRC */
			reg_val1 |= SUNXI_IISCTL_PCM;
			reg_val2 &= ~SUNXI_IISFAT0_FMT_RVD;	// Clear FMT (Bit 1:0)
			reg_val2 &= ~SUNXI_IISFAT0_LRCP;
			sunxi_iis.samp_format = SND_SOC_DAIFMT_DSP_A;
			printk("[I2S]sunxi_i2s_set_fmt: Set L data msb after FRM LRC mode\n");
			break;
		case SND_SOC_DAIFMT_DSP_B:      /* L data msb during FRM LRC */
			reg_val1 |= SUNXI_IISCTL_PCM;
			reg_val2 &= ~SUNXI_IISFAT0_FMT_RVD;	// Clear FMT (Bit 1:0)
			reg_val2 |= SUNXI_IISFAT0_LRCP;
			sunxi_iis.samp_format = SND_SOC_DAIFMT_DSP_B;
			printk("[I2S]sunxi_i2s_set_fmt: Set L data msb during FRM LRC mode\n");
			break;
		default:
			printk("[I2S]sunxi_i2s_set_fmt: Unknown mode\n");
			return -EINVAL;
	}
	writel(reg_val1, sunxi_iis.regs + SUNXI_IISCTL);
	writel(reg_val2, sunxi_iis.regs + SUNXI_IISFAT0);

	// Word select Size
	reg_val1 = readl(sunxi_iis.regs + SUNXI_IISFAT0);
	switch(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_MASK)	// TODO: Refactor, wrong configuration scheme.
	{
		case SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_16BCLK:
			reg_val1 &= ~SUNXI_IISFAT0_WSS_32BCLK; /* clear word select size */
			reg_val1 |= SUNXI_IISFAT0_WSS_16BCLK;
			sunxi_iis.ws_size = 16;
			printk("[I2S]sunxi_i2s_set_fmt: Set word select size = 16.\n");
			break;
		case SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_20BCLK:
			reg_val1 &= ~SUNXI_IISFAT0_WSS_32BCLK; /* clear word select size */
			reg_val1 |= SUNXI_IISFAT0_WSS_20BCLK;
			sunxi_iis.ws_size = 20;
			printk("[I2S]sunxi_i2s_set_fmt: Set word select size = 20.\n");
			break;
		case SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_24BCLK:
			reg_val1 &= ~SUNXI_IISFAT0_WSS_32BCLK; /* clear word select size */
			reg_val1 |= SUNXI_IISFAT0_WSS_24BCLK;
			sunxi_iis.ws_size = 24;
			printk("[I2S]sunxi_i2s_set_fmt: Set word select size = 24.\n");
			break;
		case SND_SOC_DAIFMT_SUNXI_IISFAT0_WSS_32BCLK:
			reg_val1 |= SUNXI_IISFAT0_WSS_32BCLK;
			sunxi_iis.ws_size = 32;
			printk("[I2S]sunxi_i2s_set_fmt: Set word select size = 32.\n");
			break;
		default:
			printk("[I2S]sunxi_i2s_set_fmt: Unknown mode.\n");
			break;
	}
	writel(reg_val1, sunxi_iis.regs + SUNXI_IISFAT0);

	// Signal Inversion
	reg_val1 = readl(sunxi_iis.regs + SUNXI_IISFAT0);
	switch(fmt & SND_SOC_DAIFMT_INV_MASK)
	{
		case SND_SOC_DAIFMT_NB_NF:     /* normal bit clock + frame */
			reg_val1 &= ~SUNXI_IISFAT0_LRCP;
			reg_val1 &= ~SUNXI_IISFAT0_BCP;
			sunxi_iis.bclk_pol = 0;
			sunxi_iis.lrc_pol = 0;
			printk("[I2S]sunxi_i2s_set_fmt: Normal bit clock + frame\n");
			break;
		case SND_SOC_DAIFMT_NB_IF:     /* normal bclk + inverted frame */
			reg_val1 |= SUNXI_IISFAT0_LRCP;
			reg_val1 &= ~SUNXI_IISFAT0_BCP;
			sunxi_iis.bclk_pol = 0;
			sunxi_iis.lrc_pol = 1;
			printk("[I2S]sunxi_i2s_set_fmt: Normal bclk + inverted frame\n");
			break;
		case SND_SOC_DAIFMT_IB_NF:     /* inverted bclk + normal frame */
			reg_val1 &= ~SUNXI_IISFAT0_LRCP;
			reg_val1 |= SUNXI_IISFAT0_BCP;
			sunxi_iis.bclk_pol = 1;
			sunxi_iis.lrc_pol = 0;
			printk("[I2S]sunxi_i2s_set_fmt: Inverted bclk + normal frame\n");
			break;
		case SND_SOC_DAIFMT_IB_IF:     /* inverted bclk + frame */
			reg_val1 |= SUNXI_IISFAT0_LRCP;
			reg_val1 |= SUNXI_IISFAT0_BCP;
			sunxi_iis.bclk_pol = 1;
			sunxi_iis.lrc_pol = 1;
			printk("[I2S]sunxi_i2s_set_fmt: Inverted bclk + frame\n");
			break;
		default:
			printk("[I2S]sunxi_i2s_set_fmt: Unknown mode\n");
			return -EINVAL;
	}
	writel(reg_val1, sunxi_iis.regs + SUNXI_IISFAT0);

	// PCM register setup
	if((sunxi_iis.samp_format == SND_SOC_DAIFMT_DSP_A) || 
		(sunxi_iis.samp_format == SND_SOC_DAIFMT_DSP_B))		// PCM Format.
	{
		// TODO: Wrong configuration scheme. Refactor this code.
		reg_val1 = readl(sunxi_iis.regs + SUNXI_IISFAT1);
		// TX_PDM and RX_PDM
		switch(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_PDM_MASK)
		{
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_PDM_16PCM:
				reg_val1 &= ~(SUNXI_IISFAT1_TXPDM_8ALAW | SUNXI_IISFAT1_RXPDM_8ALAW);	// clear word select size
				reg_val1 |= (SUNXI_IISFAT1_TXPDM_16PCM | SUNXI_IISFAT1_RXPDM_16PCM);
				sunxi_iis.pcm_datamode = 0;
				printk("[I2S]sunxi_i2s_set_fmt: PCM Data Mode 16-bits Linear PCM.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_PDM_8PCM:
				reg_val1 &= ~(SUNXI_IISFAT1_TXPDM_8ALAW | SUNXI_IISFAT1_RXPDM_8ALAW);	// clear word select siz
				reg_val1 |= (SUNXI_IISFAT1_TXPDM_8PCM | SUNXI_IISFAT1_RXPDM_8PCM);
				sunxi_iis.pcm_datamode = 1;
				printk("[I2S]sunxi_i2s_set_fmt: PCM Data Mode 8-bits Linear PCM.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_PDM_8ULAW:
				reg_val1 &= ~(SUNXI_IISFAT1_TXPDM_8ALAW | SUNXI_IISFAT1_RXPDM_8ALAW);	// clear word select siz
				reg_val1 |= (SUNXI_IISFAT1_TXPDM_8ULAW | SUNXI_IISFAT1_RXPDM_8ULAW);
				sunxi_iis.pcm_datamode = 2;
				printk("[I2S]sunxi_i2s_set_fmt: PCM Data Mode 8-bits u-law.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_PDM_8ALAW:
				reg_val1 &= ~(SUNXI_IISFAT1_TXPDM_8ALAW | SUNXI_IISFAT1_RXPDM_8ALAW);	// clear word select siz
				reg_val1 |= (SUNXI_IISFAT1_TXPDM_8ALAW | SUNXI_IISFAT1_RXPDM_8ALAW);
				sunxi_iis.pcm_datamode = 3;
				printk("[I2S]sunxi_i2s_set_fmt: PCM Data Mode 8-bits a-law.\n");
				break;
			default:
				printk("[I2S]sunxi_i2s_set_fmt: unknown mode.\n");
				break;
		}
		// Sync type.
		if(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_SSYNC){
				reg_val1 |= SUNXI_IISFAT1_SSYNC;
				sunxi_iis.pcm_sync_type = 1;
				printk("[I2S]sunxi_i2s_set_fmt: Short Sync Select Short Frame Sync.\n");
		}
		else{
				reg_val1 &= ~(SUNXI_IISFAT1_SSYNC);
				sunxi_iis.pcm_sync_type = 0;
				printk("[I2S]sunxi_i2s_set_fmt: Short Sync Select Long Frame Sync.\n");
		}
		// Slot Width
		if(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_SW){
				reg_val1 |= SUNXI_IISFAT1_SW;
				sunxi_iis.pcm_sw = 1;
				printk("[I2S]sunxi_i2s_set_fmt: Slot Width 16 clocks width.\n");
		}
		else{
				reg_val1 &= ~(SUNXI_IISFAT1_SW);
				sunxi_iis.pcm_sw = 0;
				printk("[I2S]sunxi_i2s_set_fmt: Slot Width 8 clocks width.\n");
		}
		// Slot Index
		switch(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_SI_MASK)
		{
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SI_1ST:
				reg_val1 &= ~(SUNXI_IISFAT1_SI_4TH);
				reg_val1 |= SUNXI_IISFAT1_SI_1ST;
				sunxi_iis.pcm_start_slot = 0;
				printk("[I2S]sunxi_i2s_set_fmt: Slot Index the 1st slot.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SI_2ND:
				reg_val1 &= ~(SUNXI_IISFAT1_SI_4TH);
				reg_val1 |= SUNXI_IISFAT1_SI_2ND;
				sunxi_iis.pcm_start_slot = 1;
				printk("[I2S]sunxi_i2s_set_fmt: Slot Index the 2nd slot.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SI_3RD:
				reg_val1 &= ~(SUNXI_IISFAT1_SI_4TH);
				reg_val1 |= SUNXI_IISFAT1_SI_3RD;
				sunxi_iis.pcm_start_slot = 2;
				printk("[I2S]sunxi_i2s_set_fmt: Slot Index the 3rd slot.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SI_4TH:
				reg_val1 |= SUNXI_IISFAT1_SI_4TH;
				sunxi_iis.pcm_start_slot = 3;
				printk("[I2S]sunxi_i2s_set_fmt: Slot Index the 4th slot.\n");
				break;
			default:
				printk("[I2S]sunxi_i2s_set_fmt: unknown mode.\n");
				break;
		}
		// Sign Extend
		if(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_SEXT){	
			reg_val1 |= SUNXI_IISFAT1_SEXT;
			printk("[I2S]sunxi_i2s_set_fmt: Sign Extend Sign extension at MSB position.\n");
		}
		else{
			reg_val1 &= ~(SUNXI_IISFAT1_SEXT);
			printk("[I2S]sunxi_i2s_set_fmt: Sign Extend Zeros or audio gain padding at LSB position.\n");
		}
		// MSB / LSB First Select
		if(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_MLS){
			reg_val1 |= SUNXI_IISFAT1_MLS;
			sunxi_iis.pcm_lsb_first = 1;
			printk("[I2S]sunxi_i2s_set_fmt: MSB/LSB First Select MSB First.\n");
		}
		else{
			sunxi_iis.pcm_lsb_first = 0;
			printk("[I2S]sunxi_i2s_set_fmt: MSB/LSB First Select LSB First.\n");
		}
		// PCM Out Mute
		if(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_OUTMUTE){
			reg_val1 |= SUNXI_IISFAT1_OUTMUTE;
			printk("[I2S]sunxi_i2s_set_fmt: PCM Out Mute force PCM_OUT to 0.\n");
		}
		else{
			printk("[I2S]sunxi_i2s_set_fmt: PCM Out Mute don't force PCM_OUT to 0.\n");
		}
		// PCM_SYNC_OUT
		if(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_SYNCOUTEN){
			reg_val1 |= SUNXI_IISFAT1_SYNCOUTEN;
			printk("[I2S]sunxi_i2s_set_fmt: PCM Sync Out Suppress PCM_SYNC whilst keeping PCM_CLK running.\n");
		}
		else{
			printk("[I2S]sunxi_i2s_set_fmt: PCM Sync Out Enable PCM_SYNC output in Master mode.\n");
		}
		// PCM_SYNC_PERIOD
		switch(fmt & SND_SOC_DAIFMT_SUNXI_IISFAT1_SYNCLEN_MASK)
		{
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SYNCLEN_16BCLK:
				reg_val1 |= SUNXI_IISFAT1_SYNCLEN_16BCLK;
				sunxi_iis.pcm_sync_period = 16;
				printk("[I2S]sunxi_i2s_set_fmt: PCM SYNC Period Clock Number 16 BCLK period.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SYNCLEN_32BCLK:
				reg_val1 |= SUNXI_IISFAT1_SYNCLEN_32BCLK;
				sunxi_iis.pcm_sync_period = 32;
				printk("[I2S]sunxi_i2s_set_fmt: PCM SYNC Period Clock Number 32 BCLK period.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SYNCLEN_64BCLK:
				reg_val1 |= SUNXI_IISFAT1_SYNCLEN_64BCLK;
				sunxi_iis.pcm_sync_period = 64;
				printk("[I2S]sunxi_i2s_set_fmt: PCM SYNC Period Clock Number 64 BCLK period.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SYNCLEN_128BCLK:
				reg_val1 |= SUNXI_IISFAT1_SYNCLEN_128BCLK;
				sunxi_iis.pcm_sync_period = 128;
				printk("[I2S]sunxi_i2s_set_fmt: PCM SYNC Period Clock Number 128 BCLK period.\n");
				break;
			case SND_SOC_DAIFMT_SUNXI_IISFAT1_SYNCLEN_256BCLK:
				reg_val1 |= SUNXI_IISFAT1_SYNCLEN_256BCLK;
				sunxi_iis.pcm_sync_period = 256;
				printk("[I2S]sunxi_i2s_set_fmt: PCM SYNC Period Clock Number 256 BCLK period.\n");
				break;
			default:
				printk("[I2S]sunxi_i2s_set_fmt: Unknown mode.\n");
				break;
		}
		writel(reg_val1, sunxi_iis.regs + SUNXI_IISFAT1);
	}

//	sunxi_i2s_printk_register_values();

	return 0;
}

/*
* TODO: Function Description.
* Saved in snd_soc_dai_ops sunxi_iis_dai_ops.
* Configure:
* - Channels.
* - Sample Format.
* - Sample Rate.
*/
static int sunxi_i2s_hw_params(struct snd_pcm_substream *substream,
								struct snd_pcm_hw_params *params,
								struct snd_soc_dai *cpu_dai)
{
	u32 reg_val1, reg_val2, reg_val3;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct sunxi_dma_params *dma_data;

	printk("[I2S]Entered %s\n", __func__);

	// Channel Select and Enable
	// TODO: Debug this mono/stereo configurations. Seems like alsa is forcing -c2 even when "aplay -c1" because the wav file is stereo.
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) // Playback.
	{
		printk("[I2S]sunxi_i2s_hw_params: SNDRV_PCM_STREAM_PLAYBACK.\n");
		dma_data = &sunxi_i2s_pcm_stereo_out;
		printk("[I2S]sunxi_i2s_hw_params: dma_data = &sunxi_i2s_pcm_stereo_out.\n");
		reg_val1 = readl(sunxi_iis.regs + SUNXI_IISCTL);
		reg_val2 = readl(sunxi_iis.regs + SUNXI_TXCHSEL);
		reg_val3 = readl(sunxi_iis.regs + SUNXI_TXCHMAP);
		#if defined CONFIG_ARCH_SUN7I || CONFIG_ARCH_SUN4I	// A10 and A20
		reg_val1 &= ~(SUNXI_IISCTL_SDO0EN | SUNXI_IISCTL_SDO1EN | SUNXI_IISCTL_SDO2EN | SUNXI_IISCTL_SDO3EN);	// Disable all output channels.
		reg_val2 &= ~(0x7 << 0);
		reg_val3 &= ~((0x7 << 0) | (0x7 << 4) | (0x7 << 8) | (0x7 << 12) | (0x7 << 16) | (0x7 << 20) | (0x7 << 24) | (0x7 << 28));
		switch (params_channels(params)) 	// Enables the outputs and sets the map of the samples, on crescent order.
		{
			case 1:	// ! channel 
				reg_val1 |= SUNXI_IISCTL_SDO0EN;
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(1); // TX Channel Select 1-ch.
				reg_val3 |= (0x0 << 0);	// TX Channel0 Mapping 1st sample.
				printk("[I2S]sunxi_i2s_hw_params: SDO0 enabled, 1 channel selected.\n");
				break;
			case 2:
				reg_val1 |= SUNXI_IISCTL_SDO0EN;
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(2); // TX Channel Select 2-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4));	// TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample.
				printk("[I2S]sunxi_i2s_hw_params: SDO0 enabled, 2 channels selected.\n");
				break;
			case 3:
				reg_val1 |= (SUNXI_IISCTL_SDO0EN | SUNXI_IISCTL_SDO1EN);
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(3); // TX Channel Select 3-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4) | (0x2 << 8)); // TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample, TX Channel3 Mapping 3rd sample.
				printk("[I2S]sunxi_i2s_hw_params: SDO0 and SDO1 enabled, 3 channels selected.\n");
				break;
			case 4:
				reg_val1 |= (SUNXI_IISCTL_SDO0EN | SUNXI_IISCTL_SDO1EN);
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(4); // TX Channel Select 4-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4) | (0x2 << 8) | (0x3 << 12)); // TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample, TX Channel3 Mapping 3rd sample, TX Channel4 Mapping 4th sample
				printk("[I2S]sunxi_i2s_hw_params: SDO0 and SDO1 enabled, 4 channels selected.\n");
				break;
			case 5:
				reg_val1 |= (SUNXI_IISCTL_SDO0EN | SUNXI_IISCTL_SDO1EN | SUNXI_IISCTL_SDO2EN);
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(5); // TX Channel Select 5-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4) | (0x2 << 8) | (0x3 << 12) | (0x4 << 16)); // TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample, TX Channel3 Mapping 3rd sample, TX Channel4 Mapping 4th sample, TX Channel5 Mapping 5th sample.
				printk("[I2S]sunxi_i2s_hw_params: SDO0, SDO1 and SDO2 enabled, 5 channels selected.\n");
				break;
			case 6:
				reg_val1 |= (SUNXI_IISCTL_SDO0EN | SUNXI_IISCTL_SDO1EN | SUNXI_IISCTL_SDO2EN);
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(6); // TX Channel Select 6-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4) | (0x2 << 8) | (0x3 << 12) | (0x4 << 16) | (0x5 << 20)); // TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample, TX Channel3 Mapping 3rd sample, TX Channel4 Mapping 4th sample, TX Channel5 Mapping 5th sample, TX Channel6 Mapping 6th sample.
				printk("[I2S]sunxi_i2s_hw_params: SDO0, SDO1 and SDO2 enabled, 6 channels selected.\n");
				break;
			case 7:
				reg_val1 |= (SUNXI_IISCTL_SDO0EN | SUNXI_IISCTL_SDO1EN | SUNXI_IISCTL_SDO2EN | SUNXI_IISCTL_SDO3EN);
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(7); // TX Channel Select 7-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4) | (0x2 << 8) | (0x3 << 12) | (0x4 << 16) | (0x5 << 20) | (0x6 << 24)); // TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample, TX Channel3 Mapping 3rd sample, TX Channel4 Mapping 4th sample, TX Channel5 Mapping 5th sample, TX Channel6 Mapping 6th sample, TX Channel7 Mapping 7th sample.
				printk("[I2S]sunxi_i2s_hw_params: SDO0, SDO1, SDO2 and SDO3 enabled, 7 channels selected.\n");
				break;
			case 8:
				reg_val1 |= (SUNXI_IISCTL_SDO0EN | SUNXI_IISCTL_SDO1EN | SUNXI_IISCTL_SDO2EN | SUNXI_IISCTL_SDO3EN);
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(8); // TX Channel Select 8-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4) | (0x2 << 8) | (0x3 << 12) | (0x4 << 16) | (0x5 << 20) | (0x6 << 24) | (0x7 << 28)); // TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample, TX Channel3 Mapping 3rd sample, TX Channel4 Mapping 4th sample, TX Channel5 Mapping 5th sample, TX Channel6 Mapping 6th sample, TX Channel7 Mapping 7th sample, TX Channel8 Mapping 8th sample.
				printk("[I2S]sunxi_i2s_hw_params: SDO0, SDO1, SDO2 and SDO3 enabled, 8 channels selected.\n");
				break;
			default:
				reg_val1 |= SUNXI_IISCTL_SDO0EN;
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(1); // TX Channel Select 1-ch.
				reg_val3 |= (0x0 << 0);	// TX Channel0 Mapping 1st sample.
				printk("[I2S]sunxi_i2s_hw_params: Wrong number of channels. Default: SDO0 enabled, 1 channel selected.\n");
		}
		#elif defined CONFIG_ARCH_SUN5I	// A10s - TODO: How to know if its A10s or A13 if both are sun5i? A13 doesn't have Digital Audio (I2S/PCM)
		reg_val1 &= ~(SUNXI_IISCTL_SDO0EN);	// Disable all output channels.
		reg_val2 &= ~(0x7 << 0);
		reg_val3 &= ~((0x7 << 0) | (0x7 << 4) | (0x7 << 8) | (0x7 << 12) | (0x7 << 16) | (0x7 << 20) | (0x7 << 24) | (0x7 << 28));
		switch (params_channels(params)) 
		{
			case 1:
				reg_val1 |= SUNXI_IISCTL_SDO0EN;
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(1); // TX Channel Select 1-ch.
				reg_val3 |= (0x0 << 0);	// TX Channel0 Mapping 1st sample.
				printk("[I2S] sunxi_i2s_hw_params: SDO0 enabled, 1 channel selected.\n");
				break;				
			case 2:
				reg_val1 |= SUNXI_IISCTL_SDO0EN;
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(2); // TX Channel Select 2-ch.
				reg_val3 |= ((0x0 << 0) | (0x1 << 4));	// TX Channel0 Mapping 1st sample, TX Channel1 Mapping 2nd sample.
				printk("[I2S] sunxi_i2s_hw_params: SDO0 enabled, 2 channels selected.\n");
				break;	
			default:
				reg_val1 |= SUNXI_IISCTL_SDO0EN;
				reg_val2 |= SUNXI_TXCHSEL_CHNUM(1); // TX Channel Select 1-ch.
				reg_val3 |= (0x0 << 0);	// TX Channel0 Mapping 1st sample.
				printk("[I2S]sunxi_i2s_hw_params: Wrong number of channels. Default: SDO0 enabled, 1 channel selected.\n");
		}
		#endif
		writel(reg_val1, sunxi_iis.regs + SUNXI_IISCTL);
		writel(reg_val2, sunxi_iis.regs + SUNXI_TXCHSEL);
		writel(reg_val3, sunxi_iis.regs + SUNXI_TXCHMAP);
	}
	else {	// Capture.
		printk("[I2S] sunxi_i2s_hw_params: SNDRV_PCM_STREAM_CAPTURE.\n");
		dma_data = &sunxi_i2s_pcm_stereo_in;
		printk("[I2S] sunxi_i2s_hw_params: dma_data = &sunxi_i2s_pcm_stereo_in\n");

		reg_val1 = readl(sunxi_iis.regs + SUNXI_RXCHSEL);
		reg_val2 = readl(sunxi_iis.regs + SUNXI_RXCHMAP);
		reg_val1 &= ~(0x7 << 0);
		reg_val2 &= ~((0x7 << 0) | (0x7 << 4));
		switch (params_channels(params)) 
		{
			case 1:
				reg_val1 |= SUNXI_RXCHSEL_CHNUM(1); // RX Channel Select 1-ch.
				reg_val2 |= (0x0 << 0);	// RX Channel0 Mapping 1st sample.
				printk("[I2S] sunxi_i2s_hw_params: SDO0 enabled, 1 channel selected.\n");
				break;				
			case 2:
				reg_val1 |= SUNXI_RXCHSEL_CHNUM(2); // RX Channel Select 2-ch.
				reg_val2 |= ((0x0 << 0) | (0x1 << 4));	// RX Channel0 Mapping 1st sample, RX Channel1 Mapping 2nd sample.
				printk("[I2S] sunxi_i2s_hw_params: SDO0 enabled, 2 channels selected.\n");
				break;	
			default:
				reg_val1 |= SUNXI_RXCHSEL_CHNUM(1); // RX Channel Select 1-ch.
				reg_val2 |= (0x0 << 0);	// RX Channel0 Mapping 1st sample.
				printk("[I2S]sunxi_i2s_hw_params: Wrong number of channels. Default: SDO0 enabled, 1 channel selected.\n");
		}
		writel(reg_val1, sunxi_iis.regs + SUNXI_RXCHSEL);
		writel(reg_val2, sunxi_iis.regs + SUNXI_RXCHMAP);
	}

	// Sample Format. 
	// TODO: Support SNDRV_PCM_FORMAT_S20_3LE and SNDRV_PCM_FMTBIT_S24_3LE formats. Must check the Word Size and change it for 24bits ("3LE"). 	
	reg_val1 = readl(sunxi_iis.regs + SUNXI_IISFAT0);
	reg_val1 &= ~SUNXI_IISFAT0_SR_RVD;		// Clear sample resolution select size
	switch (params_format(params)) 
	{
		case SNDRV_PCM_FORMAT_S16_LE:
			reg_val1 |= SUNXI_IISFAT0_SR_16BIT;
			sunxi_iis.samp_res = 16;
			printk("[I2S] sunxi_i2s_hw_params: format 16 bit\n");
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			reg_val1 |= SUNXI_IISFAT0_SR_24BIT;
			sunxi_iis.samp_res = 24;
			if(sunxi_iis.ws_size != 32)	// If the Word Size is not equal to 32, sets word size to 32.
			{
				reg_val1 = readl(sunxi_iis.regs + SUNXI_IISFAT0);
				reg_val1 |= SUNXI_IISFAT0_WSS_32BCLK;
				writel(reg_val1, sunxi_iis.regs + SUNXI_IISFAT0);
				sunxi_iis.ws_size = 32;
				printk("[I2S] sunxi_i2s_hw_params: Changing word slect size to 32bit.\n");
			}
			printk("[I2S] sunxi_i2s_hw_params: format 24 bit\n");
			break;
		default:
			printk("[I2S] sunxi_i2s_hw_params: Unsupported format (%d). Setting 24 bit format.\n", (int)params_format(params));
			reg_val1 |= SUNXI_IISFAT0_SR_24BIT;
			sunxi_iis.samp_res = 24;
			if(sunxi_iis.ws_size < 24)	// If the Word Size is lower tehen 24bits, sets the default Word Size (32bits).
			{
				reg_val1 = readl(sunxi_iis.regs + SUNXI_IISFAT0);
				reg_val1 |= SUNXI_IISFAT0_WSS_32BCLK;
				writel(reg_val1, sunxi_iis.regs + SUNXI_IISFAT0);
				sunxi_iis.ws_size = 32;
				printk("[I2S] sunxi_i2s_hw_params: Changing word slect size to 32bit.\n");
			}
			break;
	}
	writel(reg_val1, sunxi_iis.regs + SUNXI_IISFAT0);

	// Sample Rate.
	if(sunxi_iis.slave == 0)	// Only master has to configure the clock registers for sample rate setting.
	{
		sunxi_iis.samp_fs = params_rate(params);
		sunxi_i2s_set_clkdiv(cpu_dai, SUNXI_SAMPLING_FREQ, sunxi_iis.samp_fs);
	}

	snd_soc_dai_set_dma_data(rtd->cpu_dai, substream, dma_data);	// TODO: Place this call in a more apropriate place, like trigger, then test. Printk the cpu->dai name to understand "who" is it?

//	sunxi_i2s_printk_register_values();

	return 0;
}

/*
* TODO: Function Description.
* Saved in snd_soc_dai_ops sunxi_iis_dai_ops.
*/
static int sunxi_i2s_trigger(struct snd_pcm_substream *substream, 
                              int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;
//	u32 reg_val;
	#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I	
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct sunxi_dma_params *dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	#endif

	printk("[I2S]Entered %s\n", __func__);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				sunxi_snd_rxctrl_i2s(1);
			} else {
				sunxi_snd_txctrl_i2s(1);
			}
			#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I
				sunxi_dma_started(dma_data);
			#endif
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				sunxi_snd_rxctrl_i2s(0);
			} else {
				sunxi_snd_txctrl_i2s(0);
			}
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

/*
* TODO: Function Description.
* Saved in snd_soc_dai_driver sunxi_iis_dai.
*/
static int sunxi_i2s_dai_probe(struct snd_soc_dai *cpu_dai)
{
	u32 reg_val;

	printk("[I2S]Entered %s\n", __func__);

	// I2S Default Register Configuration
	sunxi_iis.slave = 1,
	sunxi_iis.samp_fs = 48000,
	sunxi_iis.samp_res = 24,
	sunxi_iis.samp_format = SND_SOC_DAIFMT_I2S,
	sunxi_iis.ws_size = 32,
	sunxi_iis.mclk_rate = 512,
	sunxi_iis.lrc_pol = 0,
	sunxi_iis.bclk_pol = 0,
	sunxi_iis.pcm_datamode = 0,
	sunxi_iis.pcm_sw = 0,
	sunxi_iis.pcm_sync_period = 0,
	sunxi_iis.pcm_sync_type = 0,
	sunxi_iis.pcm_start_slot = 0,
	sunxi_iis.pcm_lsb_first = 0,
	sunxi_iis.pcm_ch_num = 2,

	// Digital Audio Register Default Values
	// DIGITAL AUDIO CONTROL REGISTER DEF
	reg_val = SUNXI_IISCTL_MS | SUNXI_IISCTL_GEN;
	writel(reg_val, sunxi_iis.regs + SUNXI_IISCTL);

	// DIGITAL AUDIO FORMAT REGISTER 0
	reg_val = SUNXI_IISFAT0_FMT_I2S | SUNXI_IISFAT0_SR_24BIT | SUNXI_IISFAT0_WSS_32BCLK;
	writel(reg_val, sunxi_iis.regs + SUNXI_IISFAT0);

	// FIFO control register. TODO: Understand how to optimize this parameter.
	reg_val = (1<<0);	// Expanding received sample sign bit at MSB of DA_RXFIFO register. TODO: Check if this configuration works.
	reg_val |= (1<<2);	// Valid data at the LSB of TXFIFO register
	reg_val |= SUNXI_IISFCTL_RXTL(0xf);		//RX FIFO trigger level
	reg_val |= SUNXI_IISFCTL_TXTL(0x40);	//TX FIFO empty trigger level
	writel(reg_val, sunxi_iis.regs + SUNXI_IISFCTL);

	printk("[I2S]I2S default register configuration complete.\n");

	return 0;
}

/*
* TODO: Function Description.
* Saved in snd_soc_dai_driver sunxi_iis_dai.
*/
static int sunxi_i2s_dai_remove(struct snd_soc_dai *dai)
{
	printk("[I2S]Entered %s\n", __func__);

	// DIGITAL AUDIO CONTROL REGISTER
	writel(0, sunxi_iis.regs + SUNXI_IISCTL);

	return 0;
}

/*
* TODO: Function Description.
* Saved in snd_soc_dai_driver sunxi_iis_dai.
*/
static int sunxi_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	u32 reg_val;

	printk("[I2S]Entered %s\n", __func__);

	//Global Disable Digital Audio Interface
	reg_val = readl(sunxi_iis.regs + SUNXI_IISCTL);
	reg_val &= ~SUNXI_IISCTL_GEN;
	writel(reg_val, sunxi_iis.regs + SUNXI_IISCTL);

	iisregsave();

	if(!sunxi_iis.slave) {
		//release the module clock, only for master mode
		clk_disable(i2s_moduleclk);
	}
	clk_disable(i2s_apbclk);

	//printk("[I2S]PLL2 0x01c20008 = %#x\n", *(volatile int*)0xF1C20008);
	// printk("[I2S]SPECIAL CLK 0x01c20068 = %#x, line= %d\n", *(volatile int*)0xF1C20068, __LINE__);
	// printk("[I2S]SPECIAL CLK 0x01c200B8 = %#x, line = %d\n", *(volatile int*)0xF1C200B8, __LINE__);
	// TODO: Understand this printk!

	return 0;
}

/*
* TODO: Function Description.
* Saved in snd_soc_dai_driver sunxi_iis_dai.
*/
static int sunxi_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	u32 reg_val;

	printk("[I2S]Entered %s\n", __func__);

	//enable the module clock
	clk_enable(i2s_apbclk);

	if(!sunxi_iis.slave) {

		//enable the module clock
		clk_enable(i2s_moduleclk);
	}

	iisregrestore();

	//Global Enable Digital Audio Interface
	reg_val = readl(sunxi_iis.regs + SUNXI_IISCTL);
	reg_val |= SUNXI_IISCTL_GEN;
	writel(reg_val, sunxi_iis.regs + SUNXI_IISCTL);

	return 0;
}

#define SUNXI_I2S_RATES (SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT)
static struct snd_soc_dai_ops sunxi_i2s_dai_ops = {
	.set_sysclk = sunxi_i2s_set_sysclk,
	.set_clkdiv = sunxi_i2s_set_clkdiv,
	.set_fmt 	= sunxi_i2s_set_fmt,
	.hw_params 	= sunxi_i2s_hw_params,
	.trigger 	= sunxi_i2s_trigger,
};

static struct snd_soc_dai_driver sunxi_iis_dai = {
	.name 		= "sunxi-i2s-snd-soc-dai-driver",
	.probe 		= sunxi_i2s_dai_probe,
	.remove 	= sunxi_i2s_dai_remove,
	.suspend 	= sunxi_i2s_suspend,
	.resume 	= sunxi_i2s_resume,
	.ops 		= &sunxi_i2s_dai_ops,
	.capture 	= {
		.stream_name = "pcm0c",
		// TODO: Support SNDRV_PCM_FMTBIT_S20_3LE and SNDRV_PCM_FMTBIT_S24_3LE.
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
		.rates = SUNXI_I2S_RATES,
		.rate_min = SNDRV_PCM_RATE_8000,
		.rate_max = SNDRV_PCM_RATE_192000,
		.channels_min = 1,
		.channels_max = 2,
	},
	.playback 	= {
		.stream_name = "pcm0p",
		// TODO: Support SNDRV_PCM_FMTBIT_S20_3LE and SNDRV_PCM_FMTBIT_S24_3LE. Implies in changing the word select size in *_set_fmt.
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
		.rates = SUNXI_I2S_RATES,
		.rate_min = SNDRV_PCM_RATE_8000,
		.rate_max = SNDRV_PCM_RATE_192000,
		.channels_min = 1,
		.channels_max = 2,
	},
	.symmetric_rates = 1,
};

/*
* TODO: Function Description.
* Saved in platform_driver sunxi_i2s_driver.
*/
static int __devinit sunxi_i2s_dev_probe(struct platform_device *pdev)
{
	int ret;

	printk("[I2S]Entered %s\n", __func__);

 	if (i2s_used) {
		sunxi_iis.regs = ioremap(SUNXI_IISBASE, 0x100);
		if (sunxi_iis.regs == NULL)
			return -ENXIO;

		//i2s apbclk
		#if defined CONFIG_ARCH_SUN7I
		i2s_apbclk = clk_get(NULL,"apb_i2s0");
		#else
		i2s_apbclk = clk_get(NULL,"apb_i2s");
		#endif
		if(-1 == clk_enable(i2s_apbclk)){
			printk("i2s_apbclk failed! line = %d\n", __LINE__);
			goto out;
		}
		if(!sunxi_iis.slave) {
			i2s_pllx8 = clk_get(NULL, "audio_pllx8");
			//i2s pll2clk
			i2s_pll2clk = clk_get(NULL, "audio_pll");
			//i2s module clk
			
			#if defined CONFIG_ARCH_SUN7I
			i2s_moduleclk = clk_get(NULL,"i2s0");
			#else
			i2s_moduleclk = clk_get(NULL,"i2s");
			#endif
			if(clk_set_parent(i2s_moduleclk, i2s_pll2clk)){
				printk("[I2S]Try to set parent of i2s_moduleclk to i2s_pll2ck failed! line = %d\n.",__LINE__);
				goto out1;
			}
			if(clk_set_rate(i2s_moduleclk, 24576000/8)){
				printk("[I2S]Set i2s_moduleclk clock freq to 24576000 failed! line = %d\n.", __LINE__);
				goto out1;
			}
			if(-1 == clk_enable(i2s_moduleclk)){
				printk("[I2S]Open i2s_moduleclk failed! line = %d\n.", __LINE__);
				goto out1;
			}
		} 

		// iounmap(sunxi_iis.ioregs);
		ret = snd_soc_register_dai(&pdev->dev, &sunxi_iis_dai);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register DAI\n");
			goto out2;
		}
 	}

	goto out;
	out2:
		if(!sunxi_iis.slave)
			clk_disable(i2s_moduleclk);
	out1:
		clk_disable(i2s_apbclk);
	out:

	return 0;
}

/*
* TODO: Function Description.
* Saved in platform_driver sunxi_i2s_driver.
*/
static int __devexit sunxi_i2s_dev_remove(struct platform_device *pdev)
{
	printk("[I2S]Entered %s\n", __func__);

	if(i2s_used) {
		i2s_used = 0;
		if(!sunxi_iis.slave) {
			if ((NULL == i2s_moduleclk) ||(IS_ERR(i2s_moduleclk))) {
				printk("i2s_moduleclk handle is invalid, just return\n");
				return -EFAULT;
			} else {
				//release the module clock
				clk_disable(i2s_moduleclk);
			}
			if ((NULL == i2s_pllx8) ||(IS_ERR(i2s_pllx8))) {
				printk("i2s_pllx8 handle is invalid, just return\n");
				return -EFAULT;
			} else {
				//release pllx8clk
				clk_put(i2s_pllx8);
			}
			if ((NULL == i2s_pll2clk) ||(IS_ERR(i2s_pll2clk))) {
				printk("i2s_pll2clk handle is invalid, just return\n");
				return -EFAULT;
			} else {
				//release pll2clk
				clk_put(i2s_pll2clk);
			}
		}
		//release apbclk
		clk_put(i2s_apbclk);

		gpio_release(i2s_handler, 0);	// TODO: Check with parameter = 2 (original driver).

		iounmap(sunxi_iis.ioregs);

		snd_soc_unregister_dai(&pdev->dev);
		platform_set_drvdata(pdev, NULL);
	}
	return 0;
}

/*data relating*/
static struct platform_device sunxi_i2s_device = {
	.name = "sunxi-i2s",
};

/*method relating*/
static struct platform_driver sunxi_i2s_driver = {
	.probe = sunxi_i2s_dev_probe,
	.remove = __devexit_p(sunxi_i2s_dev_remove),
	.driver = {
		.name = "sunxi-i2s",
		.owner = THIS_MODULE,
	},
};

static int __init sunxi_i2s_init(void)
{
	int err = 0;
	int i2s_slv = 0;
	int ret;

	printk("[I2S]Entered %s\n", __func__);

	ret = script_parser_fetch("i2s_para","i2s_used", &i2s_used, sizeof(int));
	if (ret) {
        printk("[I2S]sunxi_i2s_init fetch i2s using configuration failed\n");
	}

 	if (i2s_used) {

		i2s_handler = gpio_request_ex("i2s_para", NULL);

		ret = script_parser_fetch("i2s_para","i2s_slave", &i2s_slv, sizeof(int));
		if ((ret == 0) && i2s_slv) {
			sunxi_iis.slave = 1;
			printk("[I2S]sunxi_i2s_init I2S used in slave mode\n");
		} else {
			sunxi_iis.slave = 0;
			printk("[I2S]sunxi_i2s_init I2S used in master mode\n");
		}

		// printk("[I2S]sunxi_i2s_init - Plataform device registering (platform_device_register).\n");
		if((err = platform_device_register(&sunxi_i2s_device)) < 0)
		{
			printk("[I2S]sunxi_i2s_init - Plataform device registering error (platform_device_register).\n");
			return err;
		}
		// else
		// 	printk("[I2S]sunxi_i2s_init - Plataform device registered (platform_device_register).\n");
		
		// printk("[I2S]sunxi_i2s_init - Plataform driver registering (platform_driver_register).\n");
		if ((err = platform_driver_register(&sunxi_i2s_driver)) < 0)
		{
			printk("[I2S]sunxi_i2s_init - Plataform driver registering error (platform_driver_register).\n");
			return err;
		}
		// else
		// 	printk("[I2S]sunxi_i2s_init - Plataform driver registered (platform_driver_register).\n");
	} 
	else {
        printk("[I2S]sunxi-i2s cannot find any using configuration for controllers, return directly!\n");
        return 0;
    }
	return 0;
}
module_init(sunxi_i2s_init);

static void __exit sunxi_i2s_exit(void)
{
	printk("[I2S]Entered %s\n", __func__);

	platform_driver_unregister(&sunxi_i2s_driver);
}
module_exit(sunxi_i2s_exit);

/* Module information */
MODULE_AUTHOR("ALLWINNER");
MODULE_DESCRIPTION("sunxi I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sunxi-i2s");

