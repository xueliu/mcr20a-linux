/*
* Driver for NXP MCR20A 802.15.4 Wireless-PAN Networking controller
*
* Copyright (C) 2016 Xue Liu <liuxuenetmail@gmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2
* as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/ieee802154.h>
#include <linux/debugfs.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>

#include <linux/device.h>

#include "mcr20a.h"

#define	SPI_COMMAND_BUFFER			3

#define REGISTER_READ			(1 << 7)
#define REGISTER_WRITE			(0 << 7)
#define REGISTER_ACCESS			(0 << 6)
#define PACKET_BUFF_BURST_ACCESS	(1 << 6)
#define PACKET_BUFF_BYTE_ACCESS		(1 << 5)

#define MCR20A_WRITE_REG(x) 		(x)
#define MCR20A_READ_REG(x) 		(REGISTER_READ | (x))
#define MCR20A_BURST_READ_PACKET_BUF 	(0xC0)
#define MCR20A_BURST_WRITE_PACKET_BUF	(0x40)
//#define MCR20A_BYTE_READ_PACKET_BUF(x) 	( 0xE0 | (x) )
//#define MCR20A_BYTE_WRITE_PACKET_BUF(x) ( 0x60 | (x) )

#define MCR20A_CMD_REG		0x80
#define MCR20A_CMD_REG_MASK	0x3f
#define MCR20A_CMD_WRITE	0x40
#define MCR20A_CMD_FB		0x20

/* Number of Interruot Request Status Register */
#define MCR20A_IRQSTS_NUM 2 /* only IRQ_STS1 and IRQ_STS2 */

/* MCR20A CCA Type */
enum {
	MCR20A_CCA_ED,      // energy detect - CCA bit not active, 
			    // not to be used for T and CCCA sequences
	MCR20A_CCA_MODE1,   // energy detect - CCA bit ACTIVE
	MCR20A_CCA_MODE2,   // 802.15.4 compliant signal detect - CCA bit ACTIVE
	MCR20A_CCA_MODE3   //
};

/* mcr20a Modem Operational Modes */
enum {
	MCR20A_STATE_OFF,
	MCR20A_STATE_HIBERNATE,
	MCR20A_STATE_DOZE,
	MCR20A_STATE_IDLE,
	MCR20A_STATE_RECEIVE,
	MCR20A_STATE_CCA_ED_DETECT
};

/* MCR20A XCVSEQ */
enum {
	MCR20A_XCVSEQ_IDLE 	= 0x00,
	MCR20A_XCVSEQ_RX	= 0x01,
	MCR20A_XCVSEQ_TX	= 0x02,
	MCR20A_XCVSEQ_CCA	= 0x03,
	MCR20A_XCVSEQ_TR	= 0x04,
	MCR20A_XCVSEQ_CCCA	= 0x05,
};

/* IEEE-802.15.4 defined constants (2.4 GHz logical channels) */
#define	MCR20A_MIN_CHANNEL		(11)
#define	MCR20A_MAX_CHANNEL		(26)
#define	MCR20A_CHANNEL_SPACING	(5)

/* MCR20A CCA Threshold constans */
#define MCR20A_MIN_CCA_THRESHOLD (0x6EU)
#define MCR20A_MAX_CCA_THRESHOLD (0x00U)

/* version 0C: new value for ACKDELAY targeting 198us (23 May, 2013, Larry Roshak) */
#define MCR20A_OVERWRITE_VERSION (0x0C)

/* MCR20A PLL configurations */
static const u8  PLL_INT[16] =
{	/* 2405 */ 0x0B,	/* 2410 */ 0x0B,	/* 2415 */ 0x0B,
	/* 2420 */ 0x0B,	/* 2425 */ 0x0B,	/* 2430 */ 0x0B,
	/* 2435 */ 0x0C,	/* 2440 */ 0x0C,	/* 2445 */ 0x0C,
	/* 2450 */ 0x0C,	/* 2455 */ 0x0C,	/* 2460 */ 0x0C,
	/* 2465 */ 0x0D,	/* 2470 */ 0x0D,	/* 2475 */ 0x0D,
	/* 2480 */ 0x0D
};

static const u8 PLL_FRAC[16] = 
{	/* 2405 */ 0x28,	/* 2410 */ 0x50,	/* 2415 */ 0x78,
	/* 2420 */ 0xA0,	/* 2425 */ 0xC8,	/* 2430 */ 0xF0,
	/* 2435 */ 0x18,	/* 2440 */ 0x40,	/* 2445 */ 0x68,
	/* 2450 */ 0x90,	/* 2455 */ 0xB8,	/* 2460 */ 0xE0,
	/* 2465 */ 0x08,	/* 2470 */ 0x30,	/* 2475 */ 0x58,
	/* 2480 */ 0x80
};

static const struct reg_sequence mar20a_iar_overwrites[] = {
	{ IAR_MISC_PAD_CTRL,	0x02 },
	{ IAR_VCO_CTRL1,	0xB3 },
	{ IAR_VCO_CTRL2, 	0x07 },
	{ IAR_PA_TUNING, 	0x71 },
	{ IAR_CHF_IBUF, 	0x2F },
	{ IAR_CHF_QBUF,		0x2F },
	{ IAR_CHF_IRIN,		0x24 },
	{ IAR_CHF_QRIN,		0x24 },
	{ IAR_CHF_IL, 		0x24 },
	{ IAR_CHF_QL, 		0x24 },
	{ IAR_CHF_CC1, 		0x32 },
	{ IAR_CHF_CCL, 		0x1D },
	{ IAR_CHF_CC2, 		0x2D },
	{ IAR_CHF_IROUT,	0x24 },
	{ IAR_CHF_QROUT,	0x24 },
	{ IAR_PA_CAL, 		0x28 },
	{ IAR_AGC_THR1, 	0x55 },
	{ IAR_AGC_THR2, 	0x2D },
	{ IAR_ATT_RSSI1, 	0x5F },
	{ IAR_ATT_RSSI2, 	0x8F },
	{ IAR_RSSI_OFFSET, 	0x61 },
	{ IAR_CHF_PMA_GAIN,	0x03 },
	{ IAR_CCA1_THRESH, 	0x50 },
	{ IAR_CORR_NVAL, 	0x13 },
	{ IAR_ACKDELAY, 	0x3D },
};

/* local variables*/
static u8 data = 0;

/* this structure defines the upper and lower bound for dump registers feature*/
typedef struct registerLimits_tag {
	u8 regStart;
	u8 regEnd;
	u8 bIsRegisterDirect;
} registerLimits_t;

const registerLimits_t darIntervals[] = {
	{ .regStart = 0x00, .regEnd = 0x26 },
	{ .regStart = 0x28, .regEnd = 0x3F },
	{ .regStart = 0x00, .regEnd = 0x00 }
};

const registerLimits_t iarIntervals[] = {
	{ .regStart = 0x00, .regEnd = 0x28 },
//	{ .regStart = 0x2A, .regEnd = 0x2E}, /* GPIO config */
	{ .regStart = 0x30, .regEnd = 0x32 },
	{ .regStart = 0x34, .regEnd = 0x45 },
	{ .regStart = 0x47, .regEnd = 0x56 },
	{ .regStart = 0x58, .regEnd = 0x5B },
	{ .regStart = 0x5D, .regEnd = 0x6B },
	{ .regStart = 0x6E, .regEnd = 0x71 },
	{ .regStart = 0x74, .regEnd = 0x75 },
	{ .regStart = 0x78, .regEnd = 0x83 },
	{ .regStart = 0x86, .regEnd = 0x86 },
	{ .regStart = 0x89, .regEnd = 0x8A },
	{ .regStart = 0x8D, .regEnd = 0x8E },
//	{ .regStart = 0x91, .regEnd = 0x97 },
	//	{ .regStart = 0x9A, .regEnd = 0xA1},
	//	{ .regStart = 0xA3, .regEnd = 0xA8},
	//	{ .regStart = 0xAA, .regEnd = 0xAF},
	//	{ .regStart = 0xB2, .regEnd = 0xB6},
	//	{ .regStart = 0xFE, .regEnd = 0xFF},
	{ .regStart = 0x00, .regEnd = 0x00 }
};

//#endif

#define MCR20A_VALID_CHANNELS (0x07FFF800)

struct mcr20a_platform_data {
	int rst_gpio;
};

#define MCR20A_MAX_BUF		(127)

#define printdev(X) (&X->spi->dev)

/* regmap information for Direct Access Register (DAR) access */
#define MCR20A_DAR_WRITE	0x01
#define MCR20A_DAR_READ		0x00
#define MCR20A_DAR_NUMREGS	0x3F

/* regmap information for Indirect Access Register (IAR) access */
#define MCR20A_IAR_ACCESS	0x80
#define MCR20A_IAR_NUMREGS	0xBEFF

/* Read/Write SPI Commands for DAR and IAR registers. */
#define MCR20A_READSHORT(reg)	((reg) << 1)
#define MCR20A_WRITESHORT(reg)	((reg) << 1 | 1)
#define MCR20A_READLONG(reg)	(1 << 15 | (reg) << 5)
#define MCR20A_WRITELONG(reg)	(1 << 15 | (reg) << 5 | 1 << 4)


/* Type definitions for link configuration of instantiable layers  */
//const uint8_t MCR20A_PHY_INDIRECT_QUEUE_SIZE = 12;
#define MCR20A_PHY_INDIRECT_QUEUE_SIZE (12)

static bool
mcr20a_dar_writeable(struct device *dev, unsigned int reg) {
	switch (reg) {
	case DAR_IRQ_STS1:
	case DAR_IRQ_STS2:
	case DAR_IRQ_STS3:
	case DAR_PHY_CTRL1:
	case DAR_PHY_CTRL2:
	case DAR_PHY_CTRL3:
	case DAR_PHY_CTRL4:
	case DAR_SRC_CTRL:
	case DAR_SRC_ADDRS_SUM_LSB:
	case DAR_SRC_ADDRS_SUM_MSB:
	case DAR_T3CMP_LSB:
	case DAR_T3CMP_MSB:
	case DAR_T3CMP_USB:
	case DAR_T2PRIMECMP_LSB:
	case DAR_T2PRIMECMP_MSB:
	case DAR_T1CMP_LSB:
	case DAR_T1CMP_MSB:
	case DAR_T1CMP_USB:
	case DAR_T2CMP_LSB:
	case DAR_T2CMP_MSB:
	case DAR_T2CMP_USB:
	case DAR_T4CMP_LSB:
	case DAR_T4CMP_MSB:
	case DAR_T4CMP_USB:
	case DAR_PLL_INT0:
	case DAR_PLL_FRAC0_LSB:
	case DAR_PLL_FRAC0_MSB:
	case DAR_PA_PWR:
	/* no DAR_ACM */
	case DAR_OVERWRITE_VER:
	case DAR_CLK_OUT_CTRL:
	case DAR_PWR_MODES:
		return true;
	default:
		return false;
	}
}

static bool
mcr20a_dar_readable(struct device *dev, unsigned int reg) {
	bool rc;

	/* all writeable are also readable */
	rc = mcr20a_dar_writeable(dev, reg);
	if (rc) return rc;

	/* readonly regs */
	switch (reg) {
	case DAR_RX_FRM_LEN:
	case DAR_CCA1_ED_FNL:
	case DAR_EVENT_TMR_LSB:
	case DAR_EVENT_TMR_MSB:
	case DAR_EVENT_TMR_USB:
	case DAR_TIMESTAMP_LSB:
	case DAR_TIMESTAMP_MSB:
	case DAR_TIMESTAMP_USB:
	case DAR_SEQ_STATE:
	case DAR_LQI_VALUE:
	case DAR_RSSI_CCA_CONT:
		return true;
	default:
		return false;
	}
}

static bool
mcr20a_dar_volatile(struct device *dev, unsigned int reg) {
	/* can be changed during runtime */
	switch (reg) {
	case DAR_IRQ_STS1:
	case DAR_IRQ_STS2:
	case DAR_IRQ_STS3:
	/* use them in spi_async and regmap so it's volatile */
		return true;
	default:
		return false;
	}
}

static bool
mcr20a_dar_precious(struct device *dev, unsigned int reg) {
	/* don't clear irq line on read */
	switch (reg) {
	case DAR_IRQ_STS1:
	case DAR_IRQ_STS2:
	case DAR_IRQ_STS3:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config mcr20a_dar_regmap = {
	.name			= "mcr20a_dar",
	.reg_bits		= 8,
	.val_bits		= 8,
	.write_flag_mask	= REGISTER_ACCESS | REGISTER_WRITE,
	.read_flag_mask 	= REGISTER_ACCESS | REGISTER_READ,
	.cache_type 		= REGCACHE_RBTREE,
	.writeable_reg		= mcr20a_dar_writeable,
	.readable_reg		= mcr20a_dar_readable,
	.volatile_reg		= mcr20a_dar_volatile,
	.precious_reg		= mcr20a_dar_precious,
	.fast_io		= true,
};

static bool
mcr20a_iar_writeable(struct device *dev, unsigned int reg) {
	switch (reg) {
	case IAR_XTAL_TRIM:
	case IAR_PMC_LP_TRIM:
	case IAR_MACPANID0_LSB:
	case IAR_MACPANID0_MSB:
	case IAR_MACSHORTADDRS0_LSB:
	case IAR_MACSHORTADDRS0_MSB:
	case IAR_MACLONGADDRS0_0:
	case IAR_MACLONGADDRS0_8:
	case IAR_MACLONGADDRS0_16:
	case IAR_MACLONGADDRS0_24:
	case IAR_MACLONGADDRS0_32:
	case IAR_MACLONGADDRS0_40:
	case IAR_MACLONGADDRS0_48:
	case IAR_MACLONGADDRS0_56:
	case IAR_RX_FRAME_FILTER:
	case IAR_PLL_INT1:
	case IAR_PLL_FRAC1_LSB:
	case IAR_PLL_FRAC1_MSB:
	case IAR_MACPANID1_LSB:
	case IAR_MACPANID1_MSB:
	case IAR_MACSHORTADDRS1_LSB:
	case IAR_MACSHORTADDRS1_MSB:
	case IAR_MACLONGADDRS1_0:
	case IAR_MACLONGADDRS1_8:
	case IAR_MACLONGADDRS1_16:
	case IAR_MACLONGADDRS1_24:
	case IAR_MACLONGADDRS1_32:
	case IAR_MACLONGADDRS1_40:
	case IAR_MACLONGADDRS1_48:
	case IAR_MACLONGADDRS1_56:
	case IAR_DUAL_PAN_CTRL:
	case IAR_DUAL_PAN_DWELL:
	case IAR_CCA1_THRESH:
	case IAR_CCA1_ED_OFFSET_COMP:
	case IAR_LQI_OFFSET_COMP:
	case IAR_CCA_CTRL:
	case IAR_CCA2_CORR_PEAKS:
	case IAR_CCA2_CORR_THRESH:
	case IAR_TMR_PRESCALE:
	case IAR_ANT_PAD_CTRL:
	case IAR_MISC_PAD_CTRL:
	case IAR_BSM_CTRL:
	case IAR_RNG:
	case IAR_RX_WTR_MARK:
	case IAR_SOFT_RESET:
	case IAR_TXDELAY:
	case IAR_ACKDELAY:
	case IAR_CORR_NVAL:
	case IAR_ANT_AGC_CTRL:
	case IAR_AGC_THR1:
	case IAR_AGC_THR2:
	case IAR_LPPS_CTRL:
	case IAR_PA_CAL:
	case IAR_ATT_RSSI1:
	case IAR_ATT_RSSI2:
	case IAR_RSSI_OFFSET:
	case IAR_XTAL_CTRL:
	case IAR_CHF_PMA_GAIN:
	case IAR_CHF_IBUF:
	case IAR_CHF_QBUF:
	case IAR_CHF_IRIN:
	case IAR_CHF_QRIN:
	case IAR_CHF_IL:
	case IAR_CHF_QL:
	case IAR_CHF_CC1:
	case IAR_CHF_CCL:
	case IAR_CHF_CC2:
	case IAR_CHF_IROUT:
	case IAR_CHF_QROUT:
	case IAR_PA_TUNING:
	case IAR_VCO_CTRL1:
	case IAR_VCO_CTRL2:
		return true;
	default:
		return false;
	}
}

static bool
mcr20a_iar_readable(struct device *dev, unsigned int reg) {
	bool rc;

	/* all writeable are also readable */
	rc = mcr20a_iar_writeable(dev, reg);
	if (rc) return rc;

	/* readonly regs */
	switch (reg) {
	case IAR_PART_ID:
	case IAR_DUAL_PAN_STS:
	case IAR_RX_BYTE_COUNT:
	case IAR_FILTERFAIL_CODE1:
	case IAR_FILTERFAIL_CODE2:
	case IAR_RSSI:
		return true;
	default:
		return false;
	}
}

static bool
mcr20a_iar_volatile(struct device *dev, unsigned int reg) {
/* can be changed during runtime */
	switch (reg) {
	case IAR_DUAL_PAN_STS:
	case IAR_RX_BYTE_COUNT:
	case IAR_FILTERFAIL_CODE1:
	case IAR_FILTERFAIL_CODE2:
	case IAR_RSSI:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config mcr20a_iar_regmap = {
	.name 			= "mcr20a_iar",
	.reg_bits 		= 16,
	.val_bits 		= 8,
	.write_flag_mask	= REGISTER_ACCESS | REGISTER_WRITE | IAR_INDEX,
	.read_flag_mask 	= REGISTER_ACCESS | REGISTER_READ  | IAR_INDEX,
	.cache_type 		= REGCACHE_RBTREE,
	.writeable_reg 		= mcr20a_iar_writeable,
	.readable_reg 		= mcr20a_iar_readable,
	.volatile_reg 		= mcr20a_iar_volatile,
	.fast_io		= true,
};

struct mcr20a_local {
	struct spi_device *spi;

	struct ieee802154_hw *hw;
	struct mcr20a_platform_data *pdata;
	struct regmap *regmap_dar;
	struct regmap *regmap_iar;

	u8 *buf;

	atomic_t mcr20a_is_awake;

	bool is_tx;

	/* for writing tx buffer */
	struct spi_message tx_buf_msg;
	u8 tx_header[1];
	/* burst buffer write command */
	struct spi_transfer tx_xfer_header;
	u8 tx_len[1];
	/* len of tx packet */
	struct spi_transfer tx_xfer_len;
	/* data of tx packet */
	struct spi_transfer tx_xfer_buf;
	struct sk_buff *tx_skb;

	/* for read length rxfifo */
	struct spi_message reg_msg;
	u8 reg_cmd[1];
	u8 reg_data[MCR20A_IRQSTS_NUM];
	struct spi_transfer reg_xfer_cmd;
	struct spi_transfer reg_xfer_data;

	/* receive handling */
	struct spi_message rx_buf_msg;
	u8 rx_header[1];
	struct spi_transfer rx_xfer_header;
	u8 rx_lqi[1];
	struct spi_transfer rx_xfer_lqi;
	u8 rx_buf[MCR20A_MAX_BUF];
	struct spi_transfer rx_xfer_buf;

	/* isr handling for reading intstat */
	struct spi_message irq_msg;
	u8 irq_header[1];
	u8 irq_data[MCR20A_IRQSTS_NUM];
	struct spi_transfer irq_xfer_data;
	struct spi_transfer irq_xfer_header;
};

static void
mcr20a_write_tx_buf_complete(void *context) {
	struct mcr20a_local *lp = context;
	int ret;
	__le16 fcf = ieee802154_get_fc_from_skb(lp->tx_skb);
	
	dev_dbg(printdev(lp), "write_tx_buf_complete()\n");

	if (ieee802154_is_ackreq(fcf)) {
		regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1,
			DAR_PHY_CTRL1_RXACKRQD, DAR_PHY_CTRL1_RXACKRQD);
		dev_info(printdev(lp), "tx requires ack\n");
	} else {
		regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1,
			DAR_PHY_CTRL1_RXACKRQD, 0x0);
	}
	
	lp->reg_msg.complete	= NULL;
	lp->reg_cmd[0]		= MCR20A_WRITE_REG(DAR_PHY_CTRL1);
	lp->reg_data[0]		= MCR20A_XCVSEQ_TX;
	lp->reg_xfer_data.len	= 1;

	ret = spi_async(lp->spi, &lp->reg_msg);
	if (ret) {
		dev_err(printdev(lp), "failed to set SEQ TX \n");
	}
}

static int
mcr20a_xmit(struct ieee802154_hw *hw, struct sk_buff *skb) {
	struct mcr20a_local *lp = hw->priv;

	dev_dbg(printdev(lp), "mcr20a_xmit()\n");

	lp->tx_skb = skb;

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "mcr20a write: ", DUMP_PREFIX_OFFSET, 16, 1,
		skb->data, skb->len, 0);
#endif

	lp->is_tx = 1;
	
	lp->reg_msg.complete 	= NULL;
	lp->reg_cmd[0]		= MCR20A_WRITE_REG(DAR_PHY_CTRL1);
	lp->reg_data[0]		= MCR20A_XCVSEQ_IDLE;
	lp->reg_xfer_data.len	= 1;

	return spi_async(lp->spi, &lp->reg_msg);
}

static int
mcr20a_ed(struct ieee802154_hw *hw, u8 *level) {
	struct mcr20a_local *lp = hw->priv;
	unsigned int val = 0;
	u8 energy_level;
	int ret;

	pr_info("mcr20a_ed\n");

	ret = regmap_read(lp->regmap_dar, DAR_PHY_CTRL1, &val);
	if (0x0 == (val & DAR_PHY_CTRL1_XCVSEQ_MASK)) {
		/* Change Clear Channel Assessment Type to 00 -  Energy Detect */
		ret = regmap_write(lp->regmap_dar, DAR_PHY_CTRL4, 0x0);
		/* Perform ED */
		ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1, DAR_PHY_CTRL1_XCVSEQ_MASK, MCR20A_XCVSEQ_CCA);
		msleep(4);
		val = 0;
		while ((DAR_IRQSTS1_CCAIRQ & val) != DAR_IRQSTS1_CCAIRQ) {
			ret = regmap_read(lp->regmap_dar, DAR_IRQ_STS1, &val);
			msleep(2);
		}
		/* the energy level scaled in 0x00 - 0xFF */
		ret = regmap_read(lp->regmap_dar, DAR_CCA1_ED_FNL, &val);
		energy_level = (u8)val;

		if (energy_level >= 90) {
			/* ED value is below minimum. Return 0x00. */
			energy_level = 0x00;
		} else if (energy_level <= 26) {
			/* ED value is above maximum. Return 0xFF. */
			energy_level = 0xFF;
		} else {
			/* Energy level (-90 dBm to -26 dBm ) --> varies form 0 to 64 */
			energy_level = (90 - energy_level);
			/* Rescale the energy level values to the 0x00-0xff range (0 to 64 translates in 0 to 255) */
			/* energyLevel * 3.9844 ~= 4 */
			/* Multiply with 4=2^2 by shifting left.
				The multiplication will not overflow beacause energyLevel has values between 0 and 63 */
			energy_level <<= 2;
		}

		*level = energy_level;
		return 0;
	} else {
		/* switch to IDLE at first */
		regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1, DAR_PHY_CTRL1_XCVSEQ_MASK, MCR20A_XCVSEQ_IDLE);
		return 0;
	}
}

static int
mcr20a_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel) {
	struct mcr20a_local *lp = hw->priv;
	int ret;

	dev_info(printdev(lp), "--> mcr20a_set_channell(): %d\n", channel);

	/* freqency = ((PLL_INT+64) + (PLL_FRAC/65536)) * 32 MHz */
	ret = regmap_write(lp->regmap_dar, DAR_PLL_INT0, PLL_INT[channel - 11]);
	if (ret) return ret;
	ret = regmap_write(lp->regmap_dar, DAR_PLL_FRAC0_LSB, 0x00);
	if (ret) return ret;
	ret = regmap_write(lp->regmap_dar, DAR_PLL_FRAC0_MSB,
		PLL_FRAC[channel - 11]);
	if (ret) return ret;

	return 0;
}

static int
mcr20a_start(struct ieee802154_hw *hw) {
	struct mcr20a_local *lp = hw->priv;
	int ret;

	dev_info(printdev(lp), "--> mcr20a_start()\n");

	/* No slotted operation */
	dev_info(printdev(lp), "no slotted operation\n");
	ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1,
		DAR_PHY_CTRL1_SLOTTED, 0x0);

	/* enable irq */
	enable_irq(lp->spi->irq);

	/* Unmask SEQ interrupt */
	ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL2, DAR_PHY_CTRL2_SEQMSK, 0x0);

	/* Start the RX sequence */
	dev_info(printdev(lp), "start the RX sequence\n");
	ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1, DAR_PHY_CTRL1_XCVSEQ_MASK, MCR20A_XCVSEQ_RX);

	return 0;
}

static void
mcr20a_stop(struct ieee802154_hw *hw) {
	struct mcr20a_local *lp = hw->priv;

	dev_info(printdev(lp), "--> mcr20a_stop()\n");
	
	/* stop all running sequence */
	regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1, DAR_PHY_CTRL1_XCVSEQ_MASK, MCR20A_XCVSEQ_IDLE); 

	/* disable irq */
	disable_irq(lp->spi->irq);
	
//	if ((status_and_control_regs[DAR_PHY_CTRL1] & DAR_PHY_CTRL1_XCVSEQ_MASK) != MCR20A_XCVSEQ_IDLE) {
//		/* Abort current SEQ */
//		status_and_control_regs[DAR_PHY_CTRL1] &= ~(DAR_PHY_CTRL1_XCVSEQ_MASK);
//		regmap_write(lp->regmap_dar, DAR_PHY_CTRL1, status_and_control_regs[DAR_PHY_CTRL1]);
//
//		/* Wait for Sequence Idle (if not already) */
//		do {
//			regmap_read(lp->regmap_dar, DAR_SEQ_STATE,  &seq_state);
//		} while ((seq_state & 0x1F) != 0);
//	}
}

static int
mcr20a_set_hw_addr_filt(struct ieee802154_hw *hw,
						struct ieee802154_hw_addr_filt *filt,
						unsigned long changed) {
	struct mcr20a_local *lp = hw->priv;

	dev_info(printdev(lp), "--> mcr20a_set_hw_addr_filt()\n");

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		u16 addr = le16_to_cpu(filt->short_addr);

		dev_info(printdev(lp), "set short addr:%02x\n", filt->short_addr);

//		mcr20a_iar_multi_write(lp, IAR_MACSHORTADDRS0_LSB, sizeof(addr), (u8 *)&addr);
		regmap_write(lp->regmap_iar, IAR_MACSHORTADDRS0_LSB, addr);
		regmap_write(lp->regmap_iar, IAR_MACSHORTADDRS0_MSB, addr >> 8);
	}

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		u16 pan = le16_to_cpu(filt->pan_id);

		dev_info(printdev(lp), "set pan id:%02x\n", filt->pan_id);
//		mcr20a_iar_multi_write(lp, IAR_MACPANID0_LSB, sizeof(pan), (u8 *)&pan);
		
		regmap_write(lp->regmap_iar, IAR_MACPANID0_LSB, pan);
		regmap_write(lp->regmap_iar, IAR_MACPANID0_MSB, pan >> 8);
	}

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		u8 addr[8], i;
		
		memcpy(addr, &filt->ieee_addr, 8);
		dev_info(printdev(lp), "set IEEE addr:%llx\n", filt->ieee_addr);

//		mcr20a_iar_multi_write(lp, IAR_MACLONGADDRS0_0, sizeof(filt->ieee_addr), addr)
		
		for (i = 0; i < 8; i++)
			regmap_write(lp->regmap_iar, IAR_MACLONGADDRS0_0 + i, addr[i]);
	}

	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		dev_info(printdev(lp),
				 "mcr20a_set_hw_addr_filt called for panc change\n");
		if (filt->pan_coord) {
			regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL4, DAR_PHY_CTRL4_PANCORDNTR0, 0x10);
		} else {
			regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL4, DAR_PHY_CTRL4_PANCORDNTR0, 0x00);;
		}
	}

	return 0;
}

/* -30 dBm to 10 dBm */
#define MCR20A_MAX_TX_POWERS 0x14
static const s32 mcr20a_powers[MCR20A_MAX_TX_POWERS + 1] = {
	-3000, -2800, -2600,\
		-2400, -2200, -2000, -1800, -1600, -1400, -1200, -1000,\
		-800, -600, -400, -200, 0, 200, 400, 600, 800, 1000\
};

static int
mcr20a_set_txpower(struct ieee802154_hw *hw, s32 mbm) {
	struct mcr20a_local *lp = hw->priv;
	u32 i;

	dev_info(&lp->spi->dev, "--> mcr20a_set_txpower:%d\n", mbm);

	for (i = 0; i < lp->hw->phy->supported.tx_powers_size; i++) {
		if (lp->hw->phy->supported.tx_powers[i] == mbm)
			return regmap_write(lp->regmap_dar, DAR_PA_PWR, ((i + 8) & 0x1F));
	}

	return -EINVAL;
}

static int
mcr20a_set_lbt(struct ieee802154_hw *hw, bool on) {
	struct mcr20a_local *lp = hw->priv;
	dev_info(printdev(lp), "--> mcr20a_set_lbt()\n");
	return regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1, DAR_PHY_CTRL1_CCABFRTX, on << DAR_PHY_CTRL1_CCABFRTX_SHIFT);
}

#define MCR20A_MAX_ED_LEVELS MCR20A_MIN_CCA_THRESHOLD
static s32 mcr20a_ed_levels[MCR20A_MAX_ED_LEVELS + 1];

static int
mcr20a_set_cca_mode(struct ieee802154_hw *hw,
					const struct wpan_phy_cca *cca) {
	struct mcr20a_local *lp = hw->priv;
	unsigned int cca_mode = 0xff;
	bool cca_mode_and = false;
	int ret;

	dev_info(printdev(lp), "--> mcr20a_set_cca_mode()\n");
	/* mapping 802.15.4 to driver spec */
	switch (cca->mode) {
	case NL802154_CCA_ENERGY:
		cca_mode = MCR20A_CCA_MODE1;
		break;
	case NL802154_CCA_CARRIER:
		cca_mode = MCR20A_CCA_MODE2;
		break;
	case NL802154_CCA_ENERGY_CARRIER:
		switch (cca->opt) {
		case NL802154_CCA_OPT_ENERGY_CARRIER_AND:
			cca_mode = MCR20A_CCA_MODE3;
			cca_mode_and = true;
			break;
		case NL802154_CCA_OPT_ENERGY_CARRIER_OR:
			cca_mode = MCR20A_CCA_MODE3;
			cca_mode_and = false;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL4, DAR_PHY_CTRL4_CCATYPE_MASK, cca_mode << DAR_PHY_CTRL4_CCATYPE_SHIFT);
	if (ret < 0) return ret;

	if (MCR20A_CCA_MODE3 == cca_mode) {
		if (cca_mode_and) {
			ret = regmap_update_bits(lp->regmap_iar, IAR_CCA_CTRL, IAR_CCA_CTRL_CCA3_AND_NOT_OR, 0x08);
		} else {
			ret = regmap_update_bits(lp->regmap_iar, IAR_CCA_CTRL, IAR_CCA_CTRL_CCA3_AND_NOT_OR, 0x00);
		}
		if (ret < 0) return ret;
	}

	return ret;
}

static int
mcr20a_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm) {
	struct mcr20a_local *lp = hw->priv;
	u32 i;

	dev_info(printdev(lp), "--> mcr20a_set_cca_ed_level()\n");

	for (i = 0; i < hw->phy->supported.cca_ed_levels_size; i++) {
		if (hw->phy->supported.cca_ed_levels[i] == mbm)
			return regmap_write(lp->regmap_iar, IAR_CCA1_THRESH, i);
	}

	return 0;
}

static int
mcr20a_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on) {
	struct mcr20a_local *lp = hw->priv;
	int ret;
	u8 rxFrameFltReg = 0x0;
	u8 val;

	dev_info(printdev(lp), "--> mcr20a_set_promiscuous_mode(%d)\n", on);

	if (on) {
		/* FRM_VER[1:0] = b00. 00: Any FrameVersion accepted (0,1,2 & 3) */
		/* All frame types accepted*/
		val |= DAR_PHY_CTRL4_PROMISCUOUS;
		rxFrameFltReg &= ~(IAR_RX_FRAME_FLT_FRM_VER);
		rxFrameFltReg |= (IAR_RX_FRAME_FLT_ACK_FT | IAR_RX_FRAME_FLT_NS_FT);

		ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL4, DAR_PHY_CTRL4_PROMISCUOUS, DAR_PHY_CTRL4_PROMISCUOUS);
		if (ret < 0) return ret;

		ret = regmap_write(lp->regmap_iar, IAR_RX_FRAME_FILTER, rxFrameFltReg);
		if (ret < 0) return ret;
	} else {
		ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL4, DAR_PHY_CTRL4_PROMISCUOUS, 0x0);
		if (ret < 0) return ret;

		ret = regmap_write(lp->regmap_iar, IAR_RX_FRAME_FILTER, IAR_RX_FRAME_FLT_FRM_VER |\
							   IAR_RX_FRAME_FLT_BEACON_FT |\
							   IAR_RX_FRAME_FLT_DATA_FT |\
							   IAR_RX_FRAME_FLT_CMD_FT);
		if (ret < 0) return ret;
	}

	return 0;
}

static const struct ieee802154_ops mcr20a_hw_ops = {
	.owner			= THIS_MODULE,
	.xmit_async		= mcr20a_xmit,
	.ed			= mcr20a_ed,
	.set_channel 		= mcr20a_set_channel,
	.start 			= mcr20a_start,
	.stop			= mcr20a_stop,
	.set_hw_addr_filt 	= mcr20a_set_hw_addr_filt,
	.set_txpower 		= mcr20a_set_txpower,
	.set_lbt 		= mcr20a_set_lbt,
	.set_cca_mode 		= mcr20a_set_cca_mode,
	.set_cca_ed_level	= mcr20a_set_cca_ed_level,
	.set_promiscuous_mode 	= mcr20a_set_promiscuous_mode,
};

//#ifdef CONFIG_IEEE802154_MCR20A_DEBUGFS
static struct dentry *mcr20a_debugfs_root;

static int mcr20a_stats_show(struct seq_file *file, void *offset) {
//	struct mcr20a_local *lp = file->private;

	seq_printf(file, "Show state of MCR20A\n");
	
	return 0;
}

static int mcr20a_dump_dar(struct seq_file *file, void *offset) {
	struct mcr20a_local *lp = file->private;

	u8 rasRegStartAddress;
	u8 rasRegStopAddress;
	unsigned int rasRegValue = 0x0;
	const registerLimits_t *interval = darIntervals;

	seq_printf(file, " -Dumping MCR20A Direct Access Registers... \n");

	while (!(interval->regStart == 0 && interval->regEnd == 0)) {
//		seq_printf(file, " -Access type : direct\n");
		seq_printf(file, " ----------------------------------------  \n");
		rasRegStopAddress 	= (*interval).regEnd;
		rasRegStartAddress 	= (*interval).regStart;
		seq_printf(file, " Address range : 0x%02x - 0x%02x \n", rasRegStartAddress, rasRegStopAddress);
		while (rasRegStartAddress <= rasRegStopAddress) {
//			mcr20a_dar_single_read(lp, rasRegStartAddress, &rasRegValue);
			regmap_read(lp->regmap_dar, rasRegStartAddress, &rasRegValue);
			seq_printf(file, " Address : 0x%02x\tData value : 0x%02x \n", rasRegStartAddress, rasRegValue);
			rasRegStartAddress += 1;
		}
		seq_printf(file, "\n ----------------------------------------  \n");

		interval++;
	}

	return 0;
}

static int mcr20a_dump_iar(struct seq_file *file, void *offset) {
	struct mcr20a_local *lp = file->private;

	u8 rasRegStartAddress;
	u8 rasRegStopAddress;
	unsigned int rasRegValue = 0x0;
	const registerLimits_t *interval = iarIntervals;

	int ret;

	seq_printf(file, " -Dumping MCR20A Indirect Access Registers... \n");
	while (!(interval->regStart == 0 && interval->regEnd == 0)) {
//		seq_printf(file, " -Access type : indirect\n");
		seq_printf(file, " ----------------------------------------  \n");
		rasRegStopAddress 	= (*interval).regEnd;
		rasRegStartAddress 	= (*interval).regStart;
		seq_printf(file, " Address range : 0x%02x - 0x%02x \n", rasRegStartAddress, rasRegStopAddress);
		while (rasRegStartAddress <= rasRegStopAddress) {
//			mcr20a_iar_single_read(lp, rasRegStartAddress, &rasRegValue);
			ret = regmap_read(lp->regmap_iar, rasRegStartAddress, &rasRegValue);
			if (ret) {
				return ret;
			}
			seq_printf(file, " Address : 0x%02x\tData value : 0x%02x \n", rasRegStartAddress, rasRegValue);
			rasRegStartAddress += 1;
		}
		seq_printf(file, "\n ----------------------------------------  \n");

		interval++;
	}

	return 0;
}

static int mcr20a_stats_open(struct inode *inode, struct file *file) {
	return single_open(file, mcr20a_stats_show, inode->i_private);
}

static int mcr20a_dump_iar_open(struct inode *inode, struct file *file) {
	return single_open(file, mcr20a_dump_iar, inode->i_private);
}

static int mcr20a_dump_dar_open(struct inode *inode, struct file *file) {
	return single_open(file, mcr20a_dump_dar, inode->i_private);
}

static const struct file_operations mcr20a_stats_fops = {
	.owner		= THIS_MODULE,
	.open		= mcr20a_stats_open,
	.read		= seq_read,
//	.write		= seq_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations mcr20a_dump_iar_fops = {
	.open		= mcr20a_dump_iar_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations mcr20a_dump_dar_fops = {
	.open		= mcr20a_dump_dar_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mcr20a_debugfs_init(struct mcr20a_local *lp) {
	char debugfs_dir_name[DNAME_INLINE_LEN + 1] = "mcr20a-";
	struct dentry *stats;

	strncat(debugfs_dir_name, dev_name(&lp->spi->dev), DNAME_INLINE_LEN);

	mcr20a_debugfs_root = debugfs_create_dir(debugfs_dir_name, NULL);
	if (!mcr20a_debugfs_root) return -ENOMEM;

	stats = debugfs_create_file("mcr20a_stats", S_IRUGO,
		mcr20a_debugfs_root, lp, &mcr20a_stats_fops);
	if (!stats) return -ENOMEM;

	stats = debugfs_create_u8("data", S_IRWXUGO, mcr20a_debugfs_root, &data);
	if (!stats) return -ENOMEM;

	stats = debugfs_create_file("dump_iar", S_IRWXUGO, mcr20a_debugfs_root, lp,
		&mcr20a_dump_iar_fops);
	if (!stats) return -ENOMEM;

	stats = debugfs_create_file("dump_dar", S_IRWXUGO, mcr20a_debugfs_root, lp,
		&mcr20a_dump_dar_fops);
	if (!stats) return -ENOMEM;

	return 0;
}

static void mcr20a_debugfs_remove(void) {
	debugfs_remove_recursive(mcr20a_debugfs_root);
}

static int
mcr20a_request_rx(struct mcr20a_local *lp) {

	dev_info(printdev(lp), "-->%s \n", __func__);

//	dev_dbg(printdev(lp), "Set SEQ to IDLE\n");

//	regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1, DAR_PHY_CTRL1_XCVSEQ_MASK, MCR20A_XCVSEQ_IDLE);

	dev_dbg(printdev(lp), "Set SEQ to RX\n");

	/* Start the RX sequence */
	regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1,
		DAR_PHY_CTRL1_XCVSEQ_MASK, MCR20A_XCVSEQ_RX);

	return 0;
}

static void
mcr20a_handle_rx_read_buf_complete(void *context) {
	struct mcr20a_local *lp = context;
	u8 len = lp->reg_data[0] & DAR_RX_FRAME_LENGTH_MASK;
	struct sk_buff *skb;

	dev_info(printdev(lp), "-->%s \n", __func__);
	
	mcr20a_request_rx(lp);
	
	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_vdbg(&lp->spi->dev, "corrupted frame received\n");
		len = IEEE802154_MTU;
	}

	len = len - 2;  /* get rid of frame check field */

	skb = dev_alloc_skb(len);
	if (!skb) {
		dev_err(printdev(lp), "failed to allocate skb\n");
		return;
	}

	memcpy(skb_put(skb, len), lp->rx_buf, len);
	ieee802154_rx_irqsafe(lp->hw, skb, lp->rx_lqi[0]);

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "mcr20a rx: ", DUMP_PREFIX_OFFSET, 16, 1,
				   lp->rx_buf, len, 0);
	pr_info("mcr20a rx: lqi: %02hhx \n", lp->rx_lqi[0]);
#endif
}

static void
mcr20a_handle_rx_read_len_complete(void *context) {
	struct mcr20a_local *lp = context;
	u8 len;
	int ret;
	
	dev_info(printdev(lp), "-->%s \n", __func__);

	/* get the length of received frame */
	len = lp->reg_data[0] & DAR_RX_FRAME_LENGTH_MASK;
	dev_dbg(printdev(lp), "frame len : %d\n", len);

	/* prepare to read the rx buf */
	lp->rx_buf_msg.complete = mcr20a_handle_rx_read_buf_complete;
	lp->rx_header[0] = MCR20A_BURST_READ_PACKET_BUF;
	lp->rx_xfer_buf.len = len;

	ret = spi_async(lp->spi, &lp->rx_buf_msg);
	if (ret) {
		dev_err(printdev(lp), "failed to read rx buffer length\n");
	}
}

static int
mcr20a_handle_rx(struct mcr20a_local *lp) {

	dev_info(printdev(lp), "-->%s \n", __func__); 
	
	lp->reg_msg.complete = mcr20a_handle_rx_read_len_complete;
	lp->reg_cmd[0] = MCR20A_READ_REG(DAR_RX_FRM_LEN);
	lp->reg_xfer_data.len	= 1;

	return spi_async(lp->spi, &lp->reg_msg);
}

static int
mcr20a_handle_tx_complete(struct mcr20a_local *lp) {
	dev_info(printdev(lp), "-->%s \n", __func__);
	
	ieee802154_xmit_complete(lp->hw, lp->tx_skb, false);

	return mcr20a_request_rx(lp);
}

static int
mcr20a_handle_tx(struct mcr20a_local *lp) {
	int ret;
	dev_info(printdev(lp), "-->%s \n", __func__);
	
	/* write tx buffer */
	lp->tx_header[0] 	= MCR20A_BURST_WRITE_PACKET_BUF;
	lp->tx_len[0] 		= lp->tx_skb->len + 2;	/* add 2 bytes of FCS */
	lp->tx_xfer_buf.tx_buf 	= lp->tx_skb->data;
	lp->tx_xfer_buf.len 	= lp->tx_skb->len + 1;	/* add 1 byte psduLength */

	ret = spi_async(lp->spi, &lp->tx_buf_msg);
	if (ret) {
		dev_err(printdev(lp), "SPI write Failed for TX buf\n");
		return ret;
	}

	return 0;
}

static void
mcr20a_irq_clean_complete(void *context) {	
	struct mcr20a_local *lp = context;
	u8 seq_state = lp->irq_data[DAR_IRQ_STS1] & DAR_PHY_CTRL1_XCVSEQ_MASK;

	dev_info(printdev(lp), "--> %s\n", __func__);
	
	enable_irq(lp->spi->irq);

	dev_info(printdev(lp), "IRQ STA1 (%02x) STA2 (%02x) \n", lp->irq_data[DAR_IRQ_STS1], lp->irq_data[DAR_IRQ_STS2]);

	switch (seq_state) {
	/* TX IRQ, RX IRQ and SEQ IRQ */
	case (0x03):
		if (lp->is_tx) {
			lp->is_tx = 0;
			dev_info(printdev(lp), "TX is done. NO ACK \n");
			mcr20a_handle_tx_complete(lp);
			return;
		}
//		else {
//			dev_info(printdev(lp), "Unknown state \n");
//			mcr20a_request_rx(lp);
//			return;
//		}
	case (0x05):
			/* rx is starting */
			dev_info(printdev(lp), "RX is starting\n");
			mcr20a_handle_rx(lp);
			return;
		break;
	case (0x07):
		if (lp->is_tx) {
			/* tx is done */
			lp->is_tx = 0;
			dev_info(printdev(lp), "TX is done \n");
			mcr20a_handle_tx_complete(lp);
			return;
		} else {
			/* rx is starting */
			dev_info(printdev(lp), "RX is starting\n");
			mcr20a_handle_rx(lp);
			return;
		}
		break;
	case (0x01):
		if (lp->is_tx) {
			dev_info(printdev(lp), "TX is starting\n");
			mcr20a_handle_tx(lp);
			return;
		}
		else {
			dev_info(printdev(lp), "MCR20A is stop \n");
//			mcr20a_request_rx(lp);
			return;
		}
		break;
	}
}


static void mcr20a_irq_status_complete(void *context) {
	int ret;
	struct mcr20a_local *lp = context;
	
	dev_info(printdev(lp), "--> %s\n", __func__);
	

	regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1,
		DAR_PHY_CTRL1_XCVSEQ_MASK, MCR20A_XCVSEQ_IDLE);

	
	lp->reg_msg.complete 	= mcr20a_irq_clean_complete;
	lp->reg_cmd[0] 		= MCR20A_WRITE_REG(DAR_IRQ_STS1);
	memcpy(lp->reg_data, lp->irq_data, MCR20A_IRQSTS_NUM);
	lp->reg_xfer_data.len	= MCR20A_IRQSTS_NUM;

	ret = spi_async(lp->spi, &lp->reg_msg);
	
	if (ret) {
		dev_err(printdev(lp), "failed to clean irq status\n");
	}
}

static irqreturn_t mcr20a_irq_isr(int irq, void *data) {
	struct mcr20a_local *lp = data;
	int ret;

	disable_irq_nosync(irq);

	lp->irq_header[0] = MCR20A_READ_REG(DAR_IRQ_STS1);
	/* read IRQSTSx */
	ret = spi_async(lp->spi, &lp->irq_msg);
	if (ret) {
		enable_irq(irq);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int mcr20a_get_platform_data(struct spi_device *spi,
		struct mcr20a_platform_data *pdata) {
	int ret = 0;

	if (spi->dev.of_node == NULL) return -EINVAL;

	pdata->rst_gpio = of_get_named_gpio(spi->dev.of_node, "rst_b-gpio", 0);
	dev_info(&spi->dev, "rst_b-gpio: %d \n", pdata->rst_gpio);

	return ret;
}

static void mcr20a_hw_setup(struct mcr20a_local *lp) {
	u8 i;
	struct ieee802154_hw *hw = lp->hw;
	struct wpan_phy *phy = lp->hw->phy;
	
	dev_info(printdev(lp), "--> %s\n", __func__); 

	phy->symbol_duration = 16;
	phy->lifs_period = 40;
	phy->sifs_period = 12;

	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM |
			IEEE802154_HW_AFILT |
			IEEE802154_HW_PROMISCUOUS;

	phy->flags = WPAN_PHY_FLAG_TXPOWER |
		WPAN_PHY_FLAG_CCA_ED_LEVEL |
		WPAN_PHY_FLAG_CCA_MODE;

	phy->supported.lbt = NL802154_SUPPORTED_BOOL_BOTH;

	phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY) |
		BIT(NL802154_CCA_CARRIER) | BIT(NL802154_CCA_ENERGY_CARRIER);
	phy->supported.cca_opts = BIT(NL802154_CCA_OPT_ENERGY_CARRIER_AND) |
		BIT(NL802154_CCA_OPT_ENERGY_CARRIER_OR);

	/* inititial cca_ed_levels */
	for (i = MCR20A_MAX_CCA_THRESHOLD; i < MCR20A_MIN_CCA_THRESHOLD + 1; ++i) {
		mcr20a_ed_levels[i] =  -i * 100;
	}

	phy->supported.cca_ed_levels = mcr20a_ed_levels;
	phy->supported.cca_ed_levels_size = ARRAY_SIZE(mcr20a_ed_levels);

	phy->cca.mode = NL802154_CCA_ENERGY;

	phy->supported.channels[0] = 0x7FFF800;
	phy->current_page = 0;
	/* MCR20A reset defualt value */
	phy->current_channel = 20;
	phy->symbol_duration = 16;
	phy->supported.tx_powers = mcr20a_powers;
	phy->supported.tx_powers_size = ARRAY_SIZE(mcr20a_powers);
	phy->cca_ed_level = phy->supported.cca_ed_levels[75];
	phy->transmit_power = phy->supported.tx_powers[0x0F];
}

static void
mcr20a_setup_tx_spi_messages(struct mcr20a_local *lp) {
	
	dev_info(printdev(lp), "--> %s\n", __func__);
	
	spi_message_init(&lp->tx_buf_msg);
	lp->tx_buf_msg.context = lp;
	lp->tx_buf_msg.complete = mcr20a_write_tx_buf_complete;

	lp->tx_xfer_header.len = 1;
	lp->tx_xfer_header.tx_buf = lp->tx_header;

	lp->tx_xfer_len.len = 1;
	lp->tx_xfer_len.tx_buf = lp->tx_len;

	spi_message_add_tail(&lp->tx_xfer_header, &lp->tx_buf_msg);
	spi_message_add_tail(&lp->tx_xfer_len, &lp->tx_buf_msg);
	spi_message_add_tail(&lp->tx_xfer_buf, &lp->tx_buf_msg);
}

static void
mcr20a_setup_rx_spi_messages(struct mcr20a_local *lp) {
	
	dev_info(printdev(lp), "--> %s\n", __func__); 
	
	spi_message_init(&lp->reg_msg);
	lp->reg_msg.context = lp;

	lp->reg_xfer_cmd.len = 1;
	lp->reg_xfer_cmd.tx_buf = lp->reg_cmd;
	lp->reg_xfer_cmd.rx_buf = lp->reg_cmd;

	lp->reg_xfer_data.rx_buf = lp->reg_data;
	lp->reg_xfer_data.tx_buf = lp->reg_data;
	
	spi_message_add_tail(&lp->reg_xfer_cmd, &lp->reg_msg);
	spi_message_add_tail(&lp->reg_xfer_data, &lp->reg_msg);

	spi_message_init(&lp->rx_buf_msg);
	lp->rx_buf_msg.context = lp;
	lp->rx_buf_msg.complete = mcr20a_handle_rx_read_buf_complete;
	lp->rx_xfer_header.len = 1;
	lp->rx_xfer_header.tx_buf = lp->rx_header;
	lp->rx_xfer_header.rx_buf = lp->rx_header;

	lp->rx_xfer_buf.rx_buf = lp->rx_buf;

	lp->rx_xfer_lqi.len = 1;
	lp->rx_xfer_lqi.rx_buf = lp->rx_lqi;

	spi_message_add_tail(&lp->rx_xfer_header,	&lp->rx_buf_msg);
	spi_message_add_tail(&lp->rx_xfer_buf, 		&lp->rx_buf_msg);
	spi_message_add_tail(&lp->rx_xfer_lqi,		&lp->rx_buf_msg);
}

static void
mcr20a_setup_irq_spi_messages(struct mcr20a_local *lp) {
	
	dev_info(printdev(lp), "--> %s\n", __func__);
	
	spi_message_init(&lp->irq_msg);
	lp->irq_msg.context		= lp;
	lp->irq_msg.complete	= mcr20a_irq_status_complete;
	lp->irq_xfer_header.len	= 1;
	lp->irq_xfer_header.tx_buf = lp->irq_header;
	lp->irq_xfer_header.rx_buf = lp->irq_header;

	lp->irq_xfer_data.len	= MCR20A_IRQSTS_NUM;
	lp->irq_xfer_data.rx_buf = lp->irq_data;

	spi_message_add_tail(&lp->irq_xfer_header, &lp->irq_msg);
	spi_message_add_tail(&lp->irq_xfer_data, &lp->irq_msg);
}

/**
* mcr20a_phy_init() - Initialize the 802.15.4 Radio registers
* @mcr20a_local:  Pointer to mcr20a_local structure 
*  
* Return: 0 or linux error code 
*/
static int
mcr20a_phy_init(struct mcr20a_local *lp) {
	u8 index;
	unsigned int phyReg = 0;
	int ret;

	dev_info(printdev(lp), "--> %s\n", __func__); 

	/* Disable Tristate on COCO MISO for SPI reads */
	ret = regmap_write(lp->regmap_iar, IAR_MISC_PAD_CTRL, 0x02);
	if (ret) goto err_ret;

	/* Clear all PP IRQ bits in IRQSTS1 to avoid unexpected interrupts immediately after init */
	ret = regmap_write(lp->regmap_dar, DAR_IRQ_STS1, 0xEF);
	if (ret) goto err_ret;

	/* Clear all PP IRQ bits in IRQSTS2 */
	ret = regmap_write(lp->regmap_dar, DAR_IRQ_STS2,
		DAR_IRQSTS2_ASM_IRQ | DAR_IRQSTS2_PB_ERR_IRQ | \
		DAR_IRQSTS2_WAKE_IRQ);
	if (ret) goto err_ret;

	/* Disable all timer interrupts */
	ret = regmap_write(lp->regmap_dar, DAR_IRQ_STS3, 0xFF);
	if (ret) goto err_ret;

	/*  PHY_CTRL1 : default HW settings + AUTOACK enabled */
	ret = regmap_update_bits(lp->regmap_dar, DAR_PHY_CTRL1,
		DAR_PHY_CTRL1_AUTOACK, DAR_PHY_CTRL1_AUTOACK);

	/*  PHY_CTRL2 : disable all interrupts */
	ret = regmap_write(lp->regmap_dar, DAR_PHY_CTRL2, 0xFF);
	if (ret) goto err_ret;

	/* PHY_CTRL3 : disable all timers and remaining interrupts */
	ret = regmap_write(lp->regmap_dar, DAR_PHY_CTRL3,
		DAR_PHY_CTRL3_ASM_MSK | DAR_PHY_CTRL3_PB_ERR_MSK |\
		DAR_PHY_CTRL3_WAKE_MSK);
	if (ret) goto err_ret;

	/* SRC_CTRL : enable Acknowledge Frame Pending and Source Address Matching Enable */
	ret = regmap_write(lp->regmap_dar, DAR_SRC_CTRL, DAR_SRC_CTRL_ACK_FRM_PND |\
						   (DAR_SRC_CTRL_INDEX << DAR_SRC_CTRL_INDEX_SHIFT));
	if (ret) goto err_ret;

	/*  RX_FRAME_FILTER */
	/*  FRM_VER[1:0] = b11. Accept FrameVersion 0 and 1 packets, reject all others */
	ret = regmap_write(lp->regmap_iar, IAR_RX_FRAME_FILTER, IAR_RX_FRAME_FLT_FRM_VER |\
						   IAR_RX_FRAME_FLT_BEACON_FT |\
						   IAR_RX_FRAME_FLT_DATA_FT |\
						   IAR_RX_FRAME_FLT_CMD_FT);
	if (ret) goto err_ret;

	/* Overwrites direct registers  */
	dev_info(printdev(lp), "Overwrites version: 0x%02x \n",
		MCR20A_OVERWRITE_VERSION);
	ret = regmap_write(lp->regmap_dar, DAR_OVERWRITE_VER,
		MCR20A_OVERWRITE_VERSION);
	if (ret) {
		goto err_ret;
	}
	
	/* Overwrites indirect registers  */
	dev_info(printdev(lp), "DAR overwrites\n");
	ret = regmap_multi_reg_write(lp->regmap_iar, mar20a_iar_overwrites,
		ARRAY_SIZE(mar20a_iar_overwrites));
	if (ret) {
		goto err_ret;
	}
	
	/* Clear HW indirect queue */
	dev_info(printdev(lp), "Clear HW indirect queue\n");
	for (index = 0; index < MCR20A_PHY_INDIRECT_QUEUE_SIZE; index++) {
		phyReg = (u8)(((index & DAR_SRC_CTRL_INDEX) << DAR_SRC_CTRL_INDEX_SHIFT)
					  | (DAR_SRC_CTRL_SRCADDR_EN)
					  | (DAR_SRC_CTRL_INDEX_DISABLE));
		ret = regmap_write(lp->regmap_dar, DAR_SRC_CTRL, phyReg);
		if (ret) goto err_ret;
		phyReg = 0;
	}

	/* Assign HW Indirect hash table to PAN0 */
	ret = regmap_read(lp->regmap_iar, IAR_DUAL_PAN_CTRL, &phyReg);
	if (ret) goto err_ret;

	/* Clear current lvl */
	phyReg &= ~IAR_DUAL_PAN_CTRL_DUAL_PAN_SAM_LVL_MSK;

	/* Set new lvl */
	phyReg |= MCR20A_PHY_INDIRECT_QUEUE_SIZE << IAR_DUAL_PAN_CTRL_DUAL_PAN_SAM_LVL_SHIFT;
	ret = regmap_write(lp->regmap_iar, IAR_DUAL_PAN_CTRL, phyReg);
	if (ret) goto err_ret;

	/* set CCA threshold to -75 dBm */
	dev_info(printdev(lp), "Set CCA threshold to -75 dBm\n");
	ret = regmap_write(lp->regmap_iar, IAR_CCA1_THRESH, 0x4B);
	if (ret) goto err_ret;

	/* set prescaller to obtain 1 symbol (16us) timebase */
	dev_info(printdev(lp), "Set prescaller to obtain 1 symbol (16us) timebase\n");
	ret = regmap_write(lp->regmap_iar, IAR_TMR_PRESCALE, 0x05);
	if (ret) goto err_ret;

	/* enable autodoze mode. */
	dev_info(printdev(lp), "Enable autodoze mode\n");
	ret = regmap_update_bits(lp->regmap_dar, DAR_PWR_MODES, DAR_PWR_MODES_AUTODOZE, DAR_PWR_MODES_AUTODOZE);
	if (ret) goto err_ret;

	/* disable clk_out */
	dev_info(printdev(lp), "Disable clk_out\n");
	ret = regmap_update_bits(lp->regmap_dar, DAR_CLK_OUT_CTRL, DAR_CLK_OUT_CTRL_EN, 0x0);
	if (ret) goto err_ret;

	return 0;

err_ret:
	return ret;
}

static int
mcr20a_probe(struct spi_device *spi) {
	struct ieee802154_hw *hw;
	struct mcr20a_local *lp;
	struct mcr20a_platform_data *pdata;
	int irq_type;
	int ret = -ENOMEM;

	dev_dbg(&spi->dev, "-->%s\n", __func__);

	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ specified\n");
		return -EINVAL;
	}

	pdata = kmalloc(sizeof(struct mcr20a_platform_data), GFP_KERNEL);
	if (pdata == NULL) {
		dev_crit(&spi->dev, "Could not allocate platform data\n");
		return -ENOMEM;
	}

	/* set mcr20a platform data */
	ret = mcr20a_get_platform_data(spi, pdata);
	if (ret < 0) {
		dev_crit(&spi->dev, "mcr20a_get_platform_data failed.\n");
		return ret;
	}

	/* init reset gpio */
	if (gpio_is_valid(pdata->rst_gpio)) {
		ret = devm_gpio_request_one(&spi->dev, pdata->rst_gpio,
			GPIOF_OUT_INIT_HIGH, "reset");
		if (ret) return ret;
	}

	/* reset mcr20a */
	if (gpio_is_valid(pdata->rst_gpio)) {
		udelay(10);
		gpio_set_value(pdata->rst_gpio, 0);
		udelay(10);
		gpio_set_value(pdata->rst_gpio, 1);
		usleep_range(120, 240);
	}

	/* allocate ieee802154_hw and private data */
	hw = ieee802154_alloc_hw(sizeof(*lp), &mcr20a_hw_ops);
	if (hw == NULL) {
		dev_crit(&spi->dev, "ieee802154_alloc_hw failed\n");
		return -ENOMEM;
	}

	/* init mcr20a local data */
	lp = hw->priv;
	lp->hw = hw;
	lp->spi = spi;
	lp->spi->dev.platform_data = pdata;
	lp->pdata = pdata;

	/* init ieee802154_hw */
	hw->parent = &spi->dev;
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	/* init buf */
	lp->buf = devm_kzalloc(&spi->dev, SPI_COMMAND_BUFFER, GFP_KERNEL);

	if (!lp->buf)
		return -ENOMEM;

	mcr20a_setup_tx_spi_messages(lp);
	mcr20a_setup_rx_spi_messages(lp);
	mcr20a_setup_irq_spi_messages(lp);

	/* setup regmap */
	lp->regmap_dar = devm_regmap_init_spi(spi, &mcr20a_dar_regmap);
	if (IS_ERR(lp->regmap_dar)) {
		ret = PTR_ERR(lp->regmap_dar);
		dev_err(&spi->dev, "Failed to allocate dar map: %d\n",
				ret);
		goto free_dev;
	}

	lp->regmap_iar = devm_regmap_init_spi(spi, &mcr20a_iar_regmap);
	if (IS_ERR(lp->regmap_iar)) {
		ret = PTR_ERR(lp->regmap_iar);
		dev_err(&spi->dev, "Failed to allocate iar map: %d\n", ret);
		goto free_dev;
	}

	mcr20a_hw_setup(lp);

	spi_set_drvdata(spi, lp);

	ret = mcr20a_phy_init(lp);
	if (ret < 0) {
		dev_crit(&spi->dev, "mcr20a_phy_init failed\n");
		goto free_dev;
	}

	irq_type = irq_get_trigger_type(spi->irq);
	if (!irq_type)
		irq_type = IRQF_TRIGGER_FALLING;

	ret = devm_request_irq(&spi->dev, spi->irq, mcr20a_irq_isr,
		irq_type, dev_name(&spi->dev), lp);
	if (ret) {
		dev_err(&spi->dev, "could not request_irq for mcr20a \n");
		ret = -ENODEV;
		goto free_dev;
	}

	/* disable_irq by default and wait for starting hardware */
	disable_irq(spi->irq);

	ret = mcr20a_debugfs_init(lp);
	if (ret) goto free_dev;

	ret = ieee802154_register_hw(hw);
	if (ret) {
		dev_crit(&spi->dev, "ieee802154_register_hw failed\n");
		goto free_debugfs;
	}

	return ret;

free_debugfs:
	mcr20a_debugfs_remove();
free_dev:
	ieee802154_free_hw(lp->hw);

	return ret;
}

static int mcr20a_remove(struct spi_device *spi) {
	struct mcr20a_local *lp = spi_get_drvdata(spi);

	dev_dbg(&spi->dev, "-->%s\n", __func__);

	ieee802154_unregister_hw(lp->hw);
	ieee802154_free_hw(lp->hw);
	mcr20a_debugfs_remove();

	return 0;
}

static const struct of_device_id mcr20a_of_match[] = {
	{ .compatible = "nxp,mcr20a", },
	{ },
};
MODULE_DEVICE_TABLE(of, mcr20a_of_match);

static const struct spi_device_id mcr20a_device_id[] = {
	{ .name = "mcr20a", },
	{ },
};
MODULE_DEVICE_TABLE(spi, mcr20a_device_id);

static struct spi_driver mcr20a_driver = {
	.id_table = mcr20a_device_id,
	.driver = {
		.of_match_table = of_match_ptr(mcr20a_of_match),
		.name	= "mcr20a",
	},
	.probe      = mcr20a_probe,
	.remove     = mcr20a_remove,
};

module_spi_driver(mcr20a_driver);

MODULE_DESCRIPTION("MCR20A Transceiver Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Xue Liu <liuxuenetmail@gmail>");
MODULE_VERSION("0.1");
