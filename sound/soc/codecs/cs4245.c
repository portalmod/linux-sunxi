/*
 * ALSA System on Chip (ASoC) codec driver for
 * Cirrus Logic CS4245 (104dB, 24-bit, 192kHz Stereo Audio CODEC)
 *
 * Authors:
 *   Felipe Correa da Silva Sanches <juca@members.fsf.org>
 *   Rafael Guayer <rafael@musicaloperatingdevices.com>
 *
 * (c)2014,2015 Musical Operating Devices LLC.  This file is licensed
 * under the terms of the GNU General Public License version 2 (or later).
 * This program is licensed "as is" without any warranty of any kind,
 * whether express or implied.
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
#include <sound/tlv.h>
#include <linux/i2c.h>
#include <plat/sys_config.h>
#include "cs4245.h"

//Uncomment the following line to get debugging info
//(log of CODEC register values):
//#define DEBUG_CS4245

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
	0x00, 	/* Padding for ASoC index 0 read */

	0x00, 	/* Register 0x01 - Chip ID
             - READONLY REGISTER */

	0x01,  	/* Register 0x02 - Power Control
             - (Bit 7) Disable Freeze bit
             - (Bit 0) Enable Device Power-Down */

	0x08,  	/* Register 0x03 - DAC Control 1
             - (Bits 7:6) Single-Speed Mode: 4 to 50kHz sample rates
             - (Bits 5:4) Left Justified, up to 24-bit data (default)
             - (Bit 2) Unmute DAC Output
             - (Bit 1) Disable 50/15us de-emphasis filter response
             - (Bit 0) Set DAC Slave Mode on serial audio port 2 */

	0x00,  	/* Register 0x04 - ADC Control
             - (Bits 7:6) Single-Speed Mode: 4 to 50kHz sample rates
             - (Bit 4) Left Justified, up to 24-bit data (default)
             - (Bit 2) Unmute serial audio output of both ADC channels
             - (Bit 1) Enable internal high-pass filter - See "High-Pass Filter and DC Offset Calibration"
             - (Bit 0) Set ADC Slave Mode on serial audio port 1 */

	0x00,  	/* Register 0x05 - MCLK Frequency
             - (Bits 6:4) MCLK1 Divider: ÷1
             - (Bits 2:0) MCLK2 Divider: ÷1 */

	0x40,  	/* Register 0x06 - Signal Selection
             - (Bits 6:5) Auxiliary analog output source: PGA output
             - (Bit 1) Disable internal digital loopback from ADC to DAC (a.k.a "soft-bypass")
             - (Bit 0) Set Synchronous Mode: ADC and DAC operate at synchronous sample rates derived from MCLK1 */

	0x00,  	/* Register 0x07 - PGA Ch B Gain Control
             - (Bits 5:0) Set PGA gain = 0dB for PGA channel B */

	0x00,  	/* Register 0x08 - PGA Ch A Gain Control
             - (Bits 5:0) Set PGA gain = 0dB for PGA channel A */

	0x19,  	/* Register 0x09 - ADC Input Control
             - (Bits 4:3) PGA Soft Ramp and Zero Cross enabled (default)
             - (Bits 2:0) Input source for the PGA and ADC: Line-Level Input Pair 1 */

	0x00,  	/* Register 0x0A - DAC Ch A Volume Control
             - (Bits 7:0) Set DAC attenuation = 0dB for channel A */

	0x00,  	/* Register 0x0B - DAC Ch B Volume Control
             - (Bits 7:0) Set DAC attenuation = 0dB for channel B */

	0xc0,  	/* Register 0x0C - DAC Control 2
             - (Bits 7:6) Soft Ramp and Zero Cross enabled (default)
             - (Bit 5) Do NOT invert DAC output
             - (Bit 0) INT pin is an active low open drain driver using an external pull-up resistor */

	0x00,  	/* Register 0x0D - Interrupt Status
             - (Bits 3:0) These bits indicate occurences of interrupt conditions.
                          By default we leave them all unset. */

	0x00,  	/* Register 0x0E - Interrupt Mask
             - (Bit 3:0) All error conditions are masked by default.
                         This means their occurence will not affect the Interrupt Status register.*/

	0x00,	0x00 /* Registers 0x0F and 0x10 - Interrupt Mode MSB and LSB
             - (Bits 3:0) All conditions set to Rising-Edge Active Mode:
                          The INT pin becomes active on the arrival of the interrupt condition */
};

struct cs4245_private {
	enum snd_soc_control_type control_type;
	unsigned int mclk;
	unsigned int dai_fmt;
	unsigned int mclk1;
	unsigned int mclk2;
	unsigned int dac_dai_fmt;	// TODO: Variable still not used anywhere, but set on _set_fmt.
	unsigned int adc_dai_fmt;	// TODO: Variable still not used anywhere, but set on _set_fmt.
	int dac_slave_mode;			// TODO: Variable still not used anywhere, but set on _set_fmt.
	int adc_slave_mode;			// TODO: Variable still not used anywhere, but set on _set_fmt.
	int async;					// TODO: Variable still not used anywhere.
};

// CS4245 Driver GPIO Handler
static u32 cs4245_gpio_handler = 0;

#ifdef DEBUG_CS4245
static void cs4245_printk_register_values(struct snd_soc_codec *codec)
{
	int reg_val[12];
	reg_val[0] = snd_soc_read(codec, CS4245_CHIP_ID);
	reg_val[1] = snd_soc_read(codec, CS4245_POWER_CTRL);
	reg_val[2] = snd_soc_read(codec, CS4245_DAC_CTRL_1);
	reg_val[3] = snd_soc_read(codec, CS4245_ADC_CTRL);
	reg_val[4] = snd_soc_read(codec, CS4245_MCLK_FREQ);
	reg_val[5] = snd_soc_read(codec, CS4245_SIGNAL_SEL);
	reg_val[6] = snd_soc_read(codec, CS4245_PGA_B_CTRL);
	reg_val[7] = snd_soc_read(codec, CS4245_PGA_A_CTRL);
	reg_val[8] = snd_soc_read(codec, CS4245_ANALOG_IN);
	reg_val[9] = snd_soc_read(codec, CS4245_DAC_A_CTRL);
	reg_val[10] = snd_soc_read(codec, CS4245_DAC_B_CTRL);
	reg_val[11] = snd_soc_read(codec, CS4245_DAC_CTRL_2);
	printk("[CS4245] Register Values:\n");
	printk("[CS4245] CHIP ID: 0x%X.\n", reg_val[0]);
	printk("[CS4245] POWER CTRL: 0x%X.\n", reg_val[1]);
	printk("[CS4245] DAC CTRL 1: 0x%X.\n", reg_val[2]);
	printk("[CS4245] ADC CTRL: 0x%X.\n", reg_val[3]);
	printk("[CS4245] MCLK FREQ: 0x%X.\n", reg_val[4]);
	printk("[CS4245] SIGNAL SEL: 0x%X.\n", reg_val[5]);
	printk("[CS4245] PGA B CTRL: 0x%X.\n", reg_val[6]);
	printk("[CS4245] PGA A CTRL: 0x%X.\n", reg_val[7]);
	printk("[CS4245] ANALOG IN: 0x%X.\n", reg_val[8]);
	printk("[CS4245] DAC A CTRL: 0x%X.\n", reg_val[9]);
	printk("[CS4245] DAC B CTRL: 0x%X.\n", reg_val[10]);
	printk("[CS4245] DAC CTRL 2: 0x%X.\n", reg_val[11]);
	return;
}
#endif

static void cs4245_reset(bool state)
{
	gpio_write_one_pin_value(cs4245_gpio_handler, state, "codec_rst_pin");
}

static int cs4245_reg_is_readable(struct snd_soc_codec *codec, unsigned int index)
{
	return (index >= CS4245_FIRSTREG) && (index <= CS4245_LASTREG);
}

static int cs4245_reg_is_volatile(struct snd_soc_codec *codec, unsigned int index)
{
	/* Unreadable registers are considered volatile */
	if ((index < CS4245_FIRSTREG) || (index > CS4245_LASTREG))
		return 1;

	return index == CS4245_CHIP_ID;
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
 * supported values: 64, 96, 128, 192, 256, 384, 512, 768, and 1024.
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
 * proper value set once the external clock is set (most probably you would do
 * that from a machine's driver 'hw_param' hook.
 *
 * The CS4245 has two master clocks, MCLK1 and MCLK2, one for each serial interface, for asynchronous operation.
 * This driver implements only synchronous operation, where the MCLK1 is used and the second serial interface uses it for clock reference.
 */
static int cs4245_set_dai_sysclk(struct snd_soc_dai *codec_dai,
								int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);
	int value;

	printk("[CS4245] %s : ", __func__);

 	switch (clk_id)
 	{
		case CS4245_MCLK1_SET:
			printk("MCLK1\n");
			cs4245->mclk1 = freq;
			break;

		case CS4245_MCLK2_SET:
			printk("MCLK2\n");
			cs4245->mclk2 = freq;
			break;

		case CS4245_MCLK_ASYNC_SET:
			value = snd_soc_read(codec, CS4245_SIGNAL_SEL);
			if(dir == CS4245_ASYNCH) {
			// When this bit is set, the DAC and ADC may be operated at
			// independent asynchronous sample rates derived from MCLK1 and MCLK2.
				printk("Asynchronous mode set.\n");
				value |= CS4245_ASYNCH;
			} else {
			// When this bit is cleared, the DAC and ADC must operate
			// at synchronous sample rates derived from MCLK1.
				printk("Synchronous mode set.\n");
				value &= ~(CS4245_ASYNCH);
			}
			return snd_soc_write(codec, CS4245_SIGNAL_SEL, value);

		default:
			printk("Invalid dai sysclk id (0x%04X).\n", clk_id);
			return -EINVAL;
	}

	return 0;
}

static int cs4245_set_dai_clkdiv(struct snd_soc_dai *codec_dai, int div_id, int value)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int regval;

	printk("[CS4245] %s : ", __func__);

 	switch (div_id) {
		case CS4245_MCLK1_DIV_SET:
			printk("MCLK1\n");
			regval = snd_soc_read(codec, CS4245_MCLK_FREQ);
			regval &= ~(CS4245_MCLK1_MASK);
			regval |= (value << CS4245_MCLK1_SHIFT);
			return snd_soc_write(codec, CS4245_MCLK_FREQ, regval);

		case CS4245_MCLK2_DIV_SET:
			printk("MCLK2\n");
			regval = snd_soc_read(codec, CS4245_MCLK_FREQ);
			regval &= ~(CS4245_MCLK2_MASK);
			regval |= (value << CS4245_MCLK2_SHIFT);
			return snd_soc_write(codec, CS4245_MCLK_FREQ, regval);

		case CS4245_DAC_FM_SET:
			printk("DAC FM\n");
			regval = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			regval &= ~(CS4245_DAC_FM_MASK);
			regval |= value;
			return snd_soc_write(codec, CS4245_DAC_CTRL_1, regval);

		case CS4245_ADC_FM_SET:
			printk("ADC FM\n");
			regval = snd_soc_read(codec, CS4245_ADC_CTRL);
			regval &= ~(CS4245_ADC_FM_MASK);
			regval |= value;
			return snd_soc_write(codec, CS4245_ADC_CTRL, regval);

		default:
			printk("Invalid clock divisor id (0x%04X).\n", div_id);
			return -EINVAL;
	}

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
 * The CS4245 is designed to allow different formats for each of the two serial interfaces.
 * But the current implementation of this driver only deals with equal formats for both serial interfaces.
 */
 // TODO - Set format on CODEC.
static int cs4245_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);
	int value, ret;

	printk("[CS4245] %s\n", __func__);

	/* set DAI format */
	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) 	// TODO: Implement CS4245_DAC_DIF_RJUST_16 DAC format configuration.
	{
		case SND_SOC_DAIFMT_LEFT_J:		// Sets both DAC and ADC formats.
			// DAC
			value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			value &= ~(CS4245_DAC_DIF_MASK);
			value |= CS4245_DAC_DIF_LJUST;
			ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec dai format.\n");
				return ret;
			}

			cs4245->dac_dai_fmt = SND_SOC_DAIFMT_LEFT_J;

			// ADC
			value = snd_soc_read(codec, CS4245_ADC_CTRL);
			value &= ~(CS4245_ADC_DIF_MASK);
			value |= CS4245_ADC_DIF_LJUST;
			ret = snd_soc_write(codec, CS4245_ADC_CTRL, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec dai format.\n");
				return ret;
			}

			cs4245->adc_dai_fmt = SND_SOC_DAIFMT_LEFT_J;
			break;

		case SND_SOC_DAIFMT_I2S:		// Sets both DAC and ADC formats.
			// DAC
			value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			value &= ~(CS4245_DAC_DIF_MASK);
			value |= CS4245_DAC_DIF_I2S;
			ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec dai format (DAC I2S).\n");
				return ret;
			}

			cs4245->dac_dai_fmt = SND_SOC_DAIFMT_I2S;

			// ADC
			value = snd_soc_read(codec, CS4245_ADC_CTRL);
			value &= ~(CS4245_ADC_DIF_MASK);
			value |= CS4245_ADC_DIF_I2S;
			ret = snd_soc_write(codec, CS4245_ADC_CTRL, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec dai format (ADC I2S).\n");
				return ret;
			}

			cs4245->adc_dai_fmt = SND_SOC_DAIFMT_I2S;
			break;

		case SND_SOC_DAIFMT_RIGHT_J:	// Only DAC supported.
			// DAC
			value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			value &= ~(CS4245_DAC_DIF_MASK);
			value |= CS4245_DAC_DIF_RJUST_24;
			ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec dai format. (DAC RIGHT_J)\n");
				return ret;
			}

			cs4245->dac_dai_fmt = SND_SOC_DAIFMT_RIGHT_J;
			break;

		default:
			printk("[CS4245]Invalid dai format (0x%04X).\n", format);
			return -EINVAL;
 	}

	/* set master/slave audio interface */
	/* The CS4245 has two serial interfaces, I2S1 and I2S2, which can both work as master or slave.
	 * This driver implements the I2S2 interface always as slave, with the LRCK2 signal connected do LRCK1 and SCLK2 signal connected to SCLK1.
	 * ALSA defines (SND_SOC_DAIFMT_*) used in a different way, as the Codec DAC and ADC can be independently master or slave. And as the Codec is always master clock slave (Needs an external clock.).
 	*/
	switch (format & SND_SOC_DAIFMT_MASTER_MASK)
	{
		case SND_SOC_DAIFMT_CBM_CFM:	// CS4245: DAC master ADC master (ALSA: codec clk & FRM master).
			// DAC
			value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			value |= CS4245_DAC_MASTER;
			ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec DAC master/slave format (CBM_CFM).\n");
				return ret;
			}

			cs4245->dac_slave_mode = 0;

			// ADC
			value = snd_soc_read(codec, CS4245_ADC_CTRL);
			value |= CS4245_ADC_MASTER;
			ret = snd_soc_write(codec, CS4245_ADC_CTRL, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec ADC master/slave format (CBM_CFM).\n");
				return ret;
			}

			cs4245->adc_slave_mode = 0;
			break;

		case SND_SOC_DAIFMT_CBS_CFM:	// CS4245: DAC slave ADC master (ALSA: codec clk slave & FRM master).
			// DAC
			value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			value &= ~(CS4245_DAC_MASTER);
			ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec DAC master/slave format (CBS_CFM).\n");
				return ret;
			}

			cs4245->dac_slave_mode = 1;

			// ADC
			value = snd_soc_read(codec, CS4245_ADC_CTRL);
			value |= CS4245_ADC_MASTER;
			ret = snd_soc_write(codec, CS4245_ADC_CTRL, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec ADC master/slave format (CBS_CFM).\n");
				return ret;
			}

			cs4245->adc_slave_mode = 0;
			break;

		case SND_SOC_DAIFMT_CBM_CFS:	// CS4245: DAC master ADC slave (ALSA: codec clk master & frame slave).
			// DAC
			value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			value |= CS4245_DAC_MASTER;
			ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec DAC master/slave format (CBM_CFS).\n");
				return ret;
			}

			cs4245->dac_slave_mode = 0;

			// ADC
			value = snd_soc_read(codec, CS4245_ADC_CTRL);
			value &= ~(CS4245_ADC_MASTER);
			ret = snd_soc_write(codec, CS4245_ADC_CTRL, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec ADC master/slave format (CBM_CFS).\n");
				return ret;
			}

			cs4245->adc_slave_mode = 1;
			break;

		case SND_SOC_DAIFMT_CBS_CFS:	// CS4245: DAC slave ADC slave (ALSA: codec clk & FRM slave).
			// DAC
			value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
			value &= ~(CS4245_DAC_MASTER);
			ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec DAC master/slave format (CBS_CFS).\n");
				return ret;
			}

			cs4245->dac_slave_mode = 1;

			// ADC
			value = snd_soc_read(codec, CS4245_ADC_CTRL);
			value &= ~(CS4245_ADC_MASTER);
			ret = snd_soc_write(codec, CS4245_ADC_CTRL, value);
			if (ret < 0)
			{
				printk("[CS4245]Error setting the codec ADC master/slave format (CBS_CFS).\n");
				return ret;
			}

			cs4245->adc_slave_mode = 1;
			break;

		default:
			printk("[CS4245]Unknown master/slave configuration (format=0x%04X).\n", format);	/* all other modes are unsupported by the hardware */
			return -EINVAL;
 	}

#ifdef DEBUG_CS4245
	cs4245_printk_register_values(codec);
#endif

	return 0;
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
	int value;

	printk("[CS4245] %s\n", __func__);

	value = snd_soc_read(codec, CS4245_DAC_CTRL_1);
	if (mute){
		value |= CS4245_MUTE_DAC;
	} else {
		value &= ~(CS4245_MUTE_DAC);
	}

	return snd_soc_write(codec, CS4245_DAC_CTRL_1, value);
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
#ifdef DEBUG_CS4245
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
#endif

	printk("[CS4245] %s\n", __func__);

#ifdef DEBUG_CS4245
	cs4245_printk_register_values(codec);
#endif

	// TODO: Implement configuration of sample rate, bit resolution and channel selection.
	// params_format(params);
	// params_rate(params);
	// params_channels(params);

	return 0;
}

static int cs4245_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int reg, ret = 0;

	printk("[CS4245]Entered %s\n", __func__);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {	// CAPTURE - ADC
				reg = snd_soc_read(codec, CS4245_ADC_CTRL);
				reg &= ~(CS4245_MUTE_ADC);	// UNMUTE
				ret = snd_soc_write(codec, CS4245_ADC_CTRL, reg);
				if (ret < 0) {
					printk("[CS4245]ADC Control register configuration failed.\n");
					return ret;
				}
			} else {	// PLAYBACK - DAC
				reg = snd_soc_read(codec, CS4245_DAC_CTRL_1);
				reg &= ~(CS4245_MUTE_DAC);	// UNMUTE
				ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, reg);
				if (ret < 0) {
					printk("[CS4245]DAC Control 1 register configuration failed.\n");
					return ret;
				}
			}
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {	// CAPTURE - ADC
				reg = snd_soc_read(codec, CS4245_ADC_CTRL);
				reg |= (CS4245_MUTE_ADC);	// MUTE
				ret = snd_soc_write(codec, CS4245_ADC_CTRL, reg);
				if (ret < 0) {
					printk("[CS4245]ADC Control register configuration failed.\n");
					return ret;
				}
			} else {	// PLAYBACK - DAC
				reg = snd_soc_read(codec, CS4245_DAC_CTRL_1);
				reg |= (CS4245_MUTE_DAC);	// MUTE
				ret = snd_soc_write(codec, CS4245_DAC_CTRL_1, reg);
				if (ret < 0) {
					printk("[CS4245]DAC Control 1 register configuration failed.\n");
					return ret;
				}
			}
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

/**
 * cs4245_probe - ASoC probe function
 * @pdev: platform device
 *
 * This function is called when ASoC has all the pieces it needs to
 * instantiate a sound driver.
 * Called after loading Machine Driver driver/module. Called after/in machine driver *_init (mod_duo_audio_init).
 */
static int cs4245_probe(struct snd_soc_codec *codec)
{
	struct cs4245_private *cs4245 = snd_soc_codec_get_drvdata(codec);
	int ret;

	printk("[CS4245] %s\n", __func__);

	/* Tell ASoC what kind of I/O to use to read the registers.  ASoC will
	 * then do the I2C transactions itself.
	 */
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, cs4245->control_type);
	if (ret < 0) {
		printk("[CS4245]Failed to set cache I/O (ret=%i).\n", ret);
		return ret;
	}

	/*
	 * Disable Power Down.
	 */
	ret = snd_soc_write(codec, CS4245_POWER_CTRL, CS4245_PDN_MIC);
	if (ret < 0) {
		printk("[CS4245]Power Control register configuration failed.\n");
		return ret;
	}

	/* Default configuration of the CODEC registers
	 * - DAC and ADC formats = I2S, 24-bit data.
	 * - DAC Master.
	 * - ADC Slave.
	 * - MCLK1 = 24.576MHz - synchronous operation.
	 * - MCLK ratio = 512.
	 * - Sampling frequency = 48kHz.
	*/
	cs4245->mclk1 = 24576000;
	cs4245->mclk2 = 0;
	cs4245->dac_dai_fmt = SND_SOC_DAIFMT_I2S;
	cs4245->adc_dai_fmt = SND_SOC_DAIFMT_I2S;
	cs4245->dac_slave_mode = 0;
	cs4245->adc_slave_mode = 1;
	cs4245->async = 0;

	/* DAC Control 1 */
	ret = snd_soc_write(codec, CS4245_DAC_CTRL_1,
		CS4245_DAC_FM_SINGLE | CS4245_DAC_DIF_I2S | CS4245_MUTE_DAC | CS4245_DAC_MASTER);
	if (ret < 0) {
		printk("[CS4245]DAC Control 1 register configuration failed.\n");
		return ret;
	}

	/* ADC Control */
	ret = snd_soc_write(codec, CS4245_ADC_CTRL,
		CS4245_ADC_FM_SINGLE | CS4245_ADC_DIF_I2S | CS4245_MUTE_ADC | CS4245_HPF_FREEZE);
	if (ret < 0) {
		printk("[CS4245]ADC Control register configuration failed.\n");
		return ret;
	}

	/* Master Clock Frequency */
	ret = snd_soc_write(codec, CS4245_MCLK_FREQ,
		(CS4245_MCLK_2 << CS4245_MCLK1_SHIFT) | (CS4245_MCLK_2 << CS4245_MCLK2_SHIFT));
	if (ret < 0) {
		printk("[CS4245]Master Clock Frequency register configuration failed.\n");
		return ret;
	}

	/* Signal Selection */
	ret = snd_soc_write(codec, CS4245_SIGNAL_SEL, CS4245_A_OUT_SEL_HIZ | CS4245_LOOP);
	if (ret < 0) {
		printk("[CS4245]Master Signal Selection register configuration failed.\n");
		return ret;
	}

	/* ADC Input Control */
	ret = snd_soc_write(codec, CS4245_ANALOG_IN,
		CS4245_PGA_SOFT | CS4245_PGA_ZERO | CS4245_SEL_INPUT_4);
	if (ret < 0) {
		printk("[CS4245]ADC Input Control register configuration failed.\n");
		return ret;
	}

	/* DAC Control 2 */
	ret = snd_soc_write(codec, CS4245_DAC_CTRL_2,
		CS4245_DAC_SOFT | CS4245_DAC_ZERO);
	if (ret < 0) {
		printk("[CS4245]DAC Control 2 register configuration failed.\n");
		return ret;
	}

	printk("[CS4245]CODEC default register configuration complete.\n");
	return ret;
}

#define cs4245_remove 	NULL
#define cs4245_soc_suspend	NULL
#define cs4245_soc_resume	NULL

static const struct snd_soc_dai_ops cs4245_dai_ops = {
	.set_sysclk	= cs4245_set_dai_sysclk,
	.set_clkdiv = cs4245_set_dai_clkdiv,
	.set_fmt = cs4245_set_dai_fmt,
	.digital_mute = cs4245_dai_mute,
	.hw_params = cs4245_hw_params,
	.trigger = cs4245_trigger,
};

static struct snd_soc_dai_driver cs4245_dai = {
	.name = "cs4245-dai",
	.ops = &cs4245_dai_ops,
	.capture = {
		.stream_name = "pcm0c",
		.formats = CS4245_FORMATS,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 4000,	// TODO: Use ALSA defines.
		.rate_max = 192000,
		.channels_min = 1,
		.channels_max = 2,
	},
	.playback = {
		.stream_name = "pcm0p",
		.formats = CS4245_FORMATS,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 4000,	// TODO: Use ALSA defines.
		.rate_max = 192000,
		.channels_min = 1,
		.channels_max = 2,
	},
	.symmetric_rates = 1,	// TODO - In a generic driver for CS4245 it should be possible to work assynchronously with different frequencies for MCLK1 and MCLK2
};
//EXPORT_SYMBOL(cs4245_dai);

static unsigned char pga_channel_ctrl_encode(unsigned char value){
/* Encoding of the bits in the PGA registers:
      min=xx101000
      max=xx011000
      (values encoded in two's complement with 0.5dB steps)
*/
	unsigned char code;
	if (value < 24){
		code = 0x20 | (value+8);
	} else {
		code = value - 24;
	}

	return code;
}

static unsigned char pga_channel_ctrl_decode(unsigned char code){
	unsigned char value;
	if (code & 0x20){
		value = (code & 0x1F) - 8;
	} else {
		value = code + 24;
	}

	return value;
}

int pga_gain_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int err;
	unsigned char val_a, val_b;
	//with 0.5dB steps we have
	//step #0 = -12dB
	//step #24 = 0dB
	//step #48 = -12dB

	val_a = pga_channel_ctrl_encode(ucontrol->value.integer.value[0]);
	val_b = pga_channel_ctrl_encode(ucontrol->value.integer.value[1]);

	err = snd_soc_update_bits_locked(codec, CS4245_PGA_A_CTRL,
	                                 /* mask: */ 0x3F, val_a);
	if (err < 0)
		return err;

	err = snd_soc_update_bits_locked(codec, CS4245_PGA_B_CTRL,
	                                 /* mask: */ 0x3F, val_b);

	return err;
}

int pga_gain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned char val_a, val_b;

	val_a = pga_channel_ctrl_decode(snd_soc_read(codec, CS4245_PGA_A_CTRL));
	val_b = pga_channel_ctrl_decode(snd_soc_read(codec, CS4245_PGA_B_CTRL));

	ucontrol->value.integer.value[0] = val_a;
	ucontrol->value.integer.value[1] = val_b;

	return 0;
}

static int pga_gain_info(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 48;
    return 0;
}

static const DECLARE_TLV_DB_SCALE(db_scale_dac, -12750, 50, 0); // DAC output attenuation from -127.5dB to 0dB (in 0.5dB steps)
static const DECLARE_TLV_DB_SCALE(db_scale_pga, -1200, 50, 0); // ADC pre-gain/pre-attenuation from -12dB to +12dB (in 0.5dB steps)

static const struct snd_kcontrol_new cs4245_snd_controls[] = {
	SOC_DOUBLE_R_TLV("DAC Volume", CS4245_DAC_B_CTRL, CS4245_DAC_A_CTRL, 0, 0xFF, 1, db_scale_dac),
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = "PGA Gain",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = pga_gain_info,
		.get = pga_gain_get,
		.put = pga_gain_put,
		.tlv.p = db_scale_pga
	},
	SOC_SINGLE("AUX OUT MUX", CS4245_SIGNAL_SEL, 5, 3, 0),
	SOC_SINGLE("LOOPBACK", CS4245_SIGNAL_SEL, 1, 1, 0)
};


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
	.controls = cs4245_snd_controls,
	.num_controls = ARRAY_SIZE(cs4245_snd_controls),
};

/*
 * cs4245_id - I2C device IDs supported by this driver
 */
static const struct i2c_device_id cs4245_id[] = {
	{"cs4245", 1},
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
static int cs4245_i2c_probe(struct i2c_client *i2c_client, const struct i2c_device_id *id)
{
	struct cs4245_private *cs4245;
	int ret;

	/* Verify that we have a CS4245 */
	ret = i2c_smbus_read_byte_data(i2c_client, CS4245_CHIP_ID);
	if (ret < 0) {
		printk("[CS4245]Failed to read i2c at addr 0x%X.\n", i2c_client->addr);
		return ret;
	}

	/* The top four bits of the chip ID should be 1100. */
	if ((ret & 0xF0) != 0xC0) {
		printk("[CS4245]Device at addr %X is not a CS4245. (Unknown chip ID: 0x%X)\n", i2c_client->addr, (ret & 0xF0) >> 4);
		return -ENODEV;
	}

	printk("[CS4245]Found a Cirrus Logic CS4245 codec at i2c address 0x%02X.\n", i2c_client->addr);
	printk("[CS4245]Hardware revision 0x%X.\n", ret & 0xF);

	cs4245 = devm_kzalloc(&i2c_client->dev, sizeof(struct cs4245_private), GFP_KERNEL);
	if (!cs4245) {
		printk("[CS4245]Could not allocate codec (devm_kzalloc).\n");
		return -ENOMEM;
	}

	cs4245->control_type = SND_SOC_I2C;
	i2c_set_clientdata(i2c_client, cs4245);
	ret = snd_soc_register_codec(&i2c_client->dev, &soc_codec_device_cs4245, &cs4245_dai, 1);
	if(ret != 0){
		printk("[CS4245]Could not register snd codec (snd_soc_register_codec).\n");
	} else {
		printk("[CS4245]CODEC registered (snd_soc_register_codec).\n");
	}

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
	printk("[CS4245] %s\n", __func__);

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

/*
* Initial function called when loading the module.
*/
static int __init cs4245_init(void)
{
	int ret = 0;
	int codec_used = 0;
	script_gpio_set_t info;

	printk("[CS4245] %s\n", __func__);

	ret = script_parser_fetch("codec_para", "codec_used", &codec_used, 1);
	if(ret != 0 || !codec_used)
	{
		printk(KERN_ERR "[CS4245]Codec CS4245 is not enabled in script.bin\n");
		return -ENODEV;
	}

    // Codec Reset Pin Configuration
	cs4245_gpio_handler = gpio_request_ex("codec_para", NULL);
	ret = script_parser_fetch("codec_para", "codec_rst_pin", (int *) &info, sizeof (script_gpio_set_t));
    if (ret) {
        printk(KERN_INFO "%s: can not get \"codec_para\" \"codec_rst_pin\" gpio handler, already used by others?\n", __FUNCTION__);
        return -EBUSY;
    }
    gpio_set_one_pin_io_status(cs4245_gpio_handler, 1, "codec_rst_pin");

	cs4245_reset(CODEC_ENABLE);

	ret = i2c_add_driver(&cs4245_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "[CS4245]Failed to register CS4245 I2C driver: %d.\n", ret);
	}

	return ret;
}
module_init(cs4245_init);

static void __exit cs4245_exit(void)
{
	printk("[CS4245] %s\n", __func__);

	cs4245_reset(CODEC_DISABLE);
	gpio_release(cs4245_gpio_handler, 2);
	i2c_del_driver(&cs4245_i2c_driver);
}
module_exit(cs4245_exit);

MODULE_AUTHOR("Felipe Correa da Silva Sanches <juca@members.fsf.org>, Rafael Guayer <rafael@musicaloperatingdevices.com>");
MODULE_DESCRIPTION("Cirrus Logic CS4245 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
