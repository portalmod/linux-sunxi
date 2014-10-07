 /*
 * The codec isn't really big-endian or little-endian, since the I2S
 * interface requires data to be sent serially with the MSbit first.
 * However, to support BE and LE I2S devices, we specify both here.  That
 * way, ALSA will always match the bit patterns.
 */

// CODEC I2C ADRESS - 0x48

#define CS4245_FORMATS (SNDRV_PCM_FMTBIT_S8      | \
			SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
			SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE | \
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
			SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_3BE | \
			SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE)

/* CS4245 registers addresses */
#define CS4245_CHIP_ID		0x01 	/* Chip ID */
#define CS4245_POWER_CTRL	0x02 	/* Power Control */
#define CS4245_DAC_CTRL_1	0x03 	/* DAC Control */
#define CS4245_ADC_CTRL		0x04 	/* ADC Control */
#define CS4245_MCLK_FREQ	0x05 	/* Master Clock Frequency */
#define CS4245_SIGNAL_SEL	0x06 	/* Signal Selection */
#define CS4245_PGA_B_CTRL	0x07 	/* Channel B PGA Control */
#define CS4245_PGA_A_CTRL	0x08 	/* Channel A PGA Control */
#define CS4245_ANALOG_IN	0x09 	/* ADC Input Control */
#define CS4245_DAC_A_CTRL	0x0a 	/* DAC Channel A Volume Control */
#define CS4245_DAC_B_CTRL	0x0b 	/* DAC Channel B Volume Control */
#define CS4245_DAC_CTRL_2	0x0c 	/* DAC Control 2 */
#define CS4245_INT_STATUS	0x0d 	/* Interrupt Status */
#define CS4245_INT_MASK		0x0e 	/* Interrupt Mask */
#define CS4245_INT_MODE_MSB	0x0f 	/* Interrupt Mode MSB */
#define CS4245_INT_MODE_LSB	0x10 	/* Interrupt Mode LSB */

#define CS4245_FIRSTREG		0x01
#define CS4245_LASTREG		0x10
#define CS4245_NUMREGS		(CS4245_LASTREG - CS4245_FIRSTREG + 1)
#define CS4245_I2C_INCR		0x80 	/* TODO - What is this? */

/* Bit masks for the CS4245 registers */

/* Chip ID */
#define CS4245_CHIP_PART_MASK	0xf0 	
#define CS4245_CHIP_REV_MASK	0x0f 

/* Power Control */
#define CS4245_FREEZE		0x80 	/* Freeze (Bit 7) */
#define CS4245_PDN_MIC		0x08 	/* Power-Down MIC (Bit 3) */
#define CS4245_PDN_ADC		0x04 	/* Power-Down ADC (Bit 2) */
#define CS4245_PDN_DAC		0x02 	/* Power-Down DAC (Bit 1) */
#define CS4245_PDN			0x01 	/* Power-Down Device (Bit 0) */
#define CS4245_PWRCTL_PDN_ALL	(CS4245_PDN_MIC | CS4245_PDN_ADC | CS4245_PDN_DAC | CS4245_PDN)


/* DAC Control */
#define CS4245_DAC_FM_MASK		0xc0 	/* DAC Functional Mode (Bits 7:6) */
#define CS4245_DAC_FM_SINGLE	0x00 	/* Single-Speed Mode: 4 to 50 kHz sample rates */
#define CS4245_DAC_FM_DOUBLE	0x40 	/* Double-Speed Mode: 50 to 100 kHz sample rates */
#define CS4245_DAC_FM_QUAD		0x80 	/* Quad-Speed Mode: 100 to 200 kHz sample rates */
#define CS4245_DAC_DIF_MASK		0x30 	/* DAC Digital Interface Format (Bits 5:4) */
#define CS4245_DAC_DIF_LJUST	0x00 	/* Left Justified, up to 24-bit data (default) */
#define CS4245_DAC_DIF_I2S		0x10 	/* I2S, up to 24-bit data */
#define CS4245_DAC_DIF_RJUST_16	0x20 	/* Right-Justified, 16-bit Data */
#define CS4245_DAC_DIF_RJUST_24	0x30 	/* Right-Justified, 24-bit Data */
//#define CS4245_RESERVED_1		0x08 	/* TODO - Realy needed? */
#define CS4245_MUTE_DAC			0x04 	/* Mute DAC (Bit 2) */
#define CS4245_DEEMPH			0x02 	/* De-Emphasis Control (Bit 1) */
#define CS4245_DAC_MASTER		0x01 	/* DAC Master / Slave Mode (Bit 0) */

/* ADC Control */
#define CS4245_ADC_FM_MASK		0xc0 	/* ADC Functional Mode (Bits 7:6) */
#define CS4245_ADC_FM_SINGLE	0x00 	/* Single-Speed Mode: 4 to 50 kHz sample rates */
#define CS4245_ADC_FM_DOUBLE	0x40 	/* Double-Speed Mode: 50 to 100 kHz sample rates */
#define CS4245_ADC_FM_QUAD		0x80 	/* Quad-Speed Mode: 100 to 200 kHz sample rates */
#define CS4245_ADC_DIF_MASK		0x10 	/* ADC Digital Interface Format (Bit 4) */
#define CS4245_ADC_DIF_LJUST	0x00 	/* Left-Justified, up to 24-bit data (default) */
#define CS4245_ADC_DIF_I2S		0x10 	/* I²S, up to 24-bit data */
#define CS4245_MUTE_ADC			0x04 	/* Mute ADC (Bit 2) */
#define CS4245_HPF_FREEZE		0x02 	/* ADC High-Pass Filter Freeze (Bit 1) */
#define CS4245_ADC_MASTER		0x01 	/* ADC Master / Slave Mode (Bit 0) */

/* MCLK Frequency */
#define CS4245_MCLK1_MASK	0x70 	/* Master Clock 1 Frequency (Bits 6:4) */
#define CS4245_MCLK1_SHIFT	4
#define CS4245_MCLK2_MASK	0x07 	/* Master Clock 2 Frequency (Bits 2:0) */
#define CS4245_MCLK2_SHIFT	0
#define CS4245_MCLK_1		0 		/* ÷ 1 */
#define CS4245_MCLK_1_5		1 		/* ÷ 1.5 */
#define CS4245_MCLK_2		2 		/* ÷ 2 */
#define CS4245_MCLK_3		3 		/* ÷ 3 */
#define CS4245_MCLK_4		4 		/* ÷ 4 */

/* Signal Selection */
#define CS4245_A_OUT_SEL_MASK	0x60 	/* Auxiliary Output Source Select (Bits 6:5) */
#define CS4245_A_OUT_SEL_HIZ	0x00 	/* High Impedance */
#define CS4245_A_OUT_SEL_DAC	0x20 	/* DAC Output */
#define CS4245_A_OUT_SEL_PGA	0x40 	/* PGA Output */
#define CS4245_LOOP				0x02 	/* Digital Loopback (Bit 1) */
#define CS4245_ASYNCH			0x01 	/* Asynchronous Mode (Bit 0) */

/* Channel B/A PGA Control */
#define CS4245_PGA_GAIN_MASK	0x3f 	/* Channel B/A PGA Gain (Bits 5:0) */

/*
Gain[5:0] 	|	Setting
101000 		|	-12 dB
000000 		|	0 dB
011000 		|	+12 dB
*/

/* ADC Input Control */
#define CS4245_ADC_SOFT_ZERO_MASK	0x18 	/* PGA Soft Ramp or Zero Cross Enable (Bits 4:3) */
#define CS4245_PGA_SOFT				0x10 	/* Soft Ramp enabled */
#define CS4245_PGA_ZERO				0x08 	/* Zero Cross enabled */
#define CS4245_SEL_MASK				0x07 	/* Analog Input Selection (Bits 2:0) */
#define CS4245_SEL_MIC				0x00 	/* Microphone-Level Inputs (+32 dB Gain Enabled) */
#define CS4245_SEL_INPUT_1			0x01 	/* Line-Level Input Pair 1 */
#define CS4245_SEL_INPUT_2			0x02 	/* Line-Level Input Pair 2*/
#define CS4245_SEL_INPUT_3			0x03 	/* Line-Level Input Pair 3 */
#define CS4245_SEL_INPUT_4			0x04 	/* Line-Level Input Pair 4 */
#define CS4245_SEL_INPUT_5			0x05 	/* Line-Level Input Pair 5 */
#define CS4245_SEL_INPUT_6			0x06 	/* Line-Level Input Pair 6 */

/* DAC Channel A/B Volume Control */
#define CS4245_VOL_MASK		0xff 	/* Volume Control (Bits 7:0) */

/*
	Binary Code	| 	Volume Setting
	00000000 	|	0 dB
	00000001 	|	-0.5 dB
	00101000	|	-20 dB
	00101001 	|	-20.5
	11111110 	|	-127 dB
	11111111 	|	-127.5 dB
 */

/* DAC Control 2 */
#define CS4245_DAC_SOFT_ZERO_MASK 	0xc0	/* DAC Soft Ramp or Zero Cross Enable (Bits 7:6) */
#define CS4245_DAC_SOFT				0x80 	/* Soft Ramp enabled */
#define CS4245_DAC_ZERO				0x40 	/* Zero Cross enabled */
#define CS4245_INVERT_DAC			0x20 	/* Invert DAC Output (Bit 5) */
#define CS4245_INT_ACTIVE_HIGH		0x01 	/* Active High/Low (Bit 0) */

/* Interrupt Status/Mask/Mode */
#define CS4245_ADC_CLK_ERR	0x08 	/* ADC Clock Error (Bit 3) */
#define CS4245_DAC_CLK_ERR	0x04 	/* DAC Clock Error (Bit 2) */
#define CS4245_ADC_OVFL		0x02 	/* ADC Overflow (Bit 1) */
#define CS4245_ADC_UNDRFL	0x01 	/* ADC Underflow (Bit 0) */

/* Codec Reset Pin Defines */
#define CODEC_ENABLE	1
#define CODEC_DISABLE	(!CODEC_ENABLE)