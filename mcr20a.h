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
#ifndef _MCR20A_H
#define _MCR20A_H

/* Direct Accress Register */
#define DAR_IRQ_STS1            0x00
#define DAR_IRQ_STS2            0x01
#define DAR_IRQ_STS3            0x02
#define DAR_PHY_CTRL1           0x03
#define DAR_PHY_CTRL2           0x04
#define DAR_PHY_CTRL3           0x05
#define DAR_RX_FRM_LEN          0x06
#define DAR_PHY_CTRL4           0x07
#define DAR_SRC_CTRL            0x08
#define DAR_SRC_ADDRS_SUM_LSB   0x09
#define DAR_SRC_ADDRS_SUM_MSB   0x0A
#define DAR_CCA1_ED_FNL         0x0B
#define DAR_EVENT_TMR_LSB       0x0C
#define DAR_EVENT_TMR_MSB       0x0D
#define DAR_EVENT_TMR_USB       0x0E
#define DAR_TIMESTAMP_LSB       0x0F
#define DAR_TIMESTAMP_MSB       0x10
#define DAR_TIMESTAMP_USB       0x11
#define DAR_T3CMP_LSB           0x12
#define DAR_T3CMP_MSB           0x13
#define DAR_T3CMP_USB           0x14
#define DAR_T2PRIMECMP_LSB      0x15
#define DAR_T2PRIMECMP_MSB      0x16
#define DAR_T1CMP_LSB           0x17
#define DAR_T1CMP_MSB           0x18
#define DAR_T1CMP_USB           0x19
#define DAR_T2CMP_LSB           0x1A
#define DAR_T2CMP_MSB           0x1B
#define DAR_T2CMP_USB           0x1C
#define DAR_T4CMP_LSB           0x1D
#define DAR_T4CMP_MSB           0x1E
#define DAR_T4CMP_USB           0x1F
#define DAR_PLL_INT0            0x20
#define DAR_PLL_FRAC0_LSB       0x21
#define DAR_PLL_FRAC0_MSB       0x22
#define DAR_PA_PWR              0x23
#define DAR_SEQ_STATE           0x24
#define DAR_LQI_VALUE           0x25
#define DAR_RSSI_CCA_CONT       0x26
/*------------------            0x27 */
#define DAR_ASM_CTRL1           0x28
#define DAR_ASM_CTRL2           0x29
#define DAR_ASM_DATA_0          0x2A
#define DAR_ASM_DATA_1          0x2B
#define DAR_ASM_DATA_2          0x2C
#define DAR_ASM_DATA_3          0x2D
#define DAR_ASM_DATA_4          0x2E
#define DAR_ASM_DATA_5          0x2F
#define DAR_ASM_DATA_6          0x30
#define DAR_ASM_DATA_7          0x31
#define DAR_ASM_DATA_8			0x32
#define DAR_ASM_DATA_9          0x33
#define DAR_ASM_DATA_A          0x34
#define DAR_ASM_DATA_B          0x35
#define DAR_ASM_DATA_C          0x36
#define DAR_ASM_DATA_D          0x37
#define DAR_ASM_DATA_E          0x38
#define DAR_ASM_DATA_F          0x39
/*-----------------------       0x3A */
#define DAR_OVERWRITE_VER       0x3B
#define DAR_CLK_OUT_CTRL        0x3C
#define DAR_PWR_MODES           0x3D
#define IAR_INDEX           	0x3E
#define IAR_DATA            	0x3F

/* Indirect Resgister Memory */
#define IAR_PART_ID             0x00
#define IAR_XTAL_TRIM           0x01
#define IAR_PMC_LP_TRIM         0x02
#define IAR_MACPANID0_LSB       0x03
#define IAR_MACPANID0_MSB       0x04
#define IAR_MACSHORTADDRS0_LSB  0x05
#define IAR_MACSHORTADDRS0_MSB  0x06
#define IAR_MACLONGADDRS0_0     0x07
#define IAR_MACLONGADDRS0_8     0x08
#define IAR_MACLONGADDRS0_16    0x09
#define IAR_MACLONGADDRS0_24    0x0A
#define IAR_MACLONGADDRS0_32    0x0B
#define IAR_MACLONGADDRS0_40    0x0C
#define IAR_MACLONGADDRS0_48    0x0D
#define IAR_MACLONGADDRS0_56    0x0E
#define IAR_RX_FRAME_FILTER     0x0F
#define IAR_PLL_INT1            0x10
#define IAR_PLL_FRAC1_LSB       0x11
#define IAR_PLL_FRAC1_MSB       0x12
#define IAR_MACPANID1_LSB       0x13
#define IAR_MACPANID1_MSB       0x14
#define IAR_MACSHORTADDRS1_LSB  0x15
#define IAR_MACSHORTADDRS1_MSB  0x16
#define IAR_MACLONGADDRS1_0     0x17
#define IAR_MACLONGADDRS1_8     0x18
#define IAR_MACLONGADDRS1_16    0x19
#define IAR_MACLONGADDRS1_24    0x1A
#define IAR_MACLONGADDRS1_32    0x1B
#define IAR_MACLONGADDRS1_40    0x1C
#define IAR_MACLONGADDRS1_48    0x1D
#define IAR_MACLONGADDRS1_56    0x1E
#define IAR_DUAL_PAN_CTRL       0x1F
#define IAR_DUAL_PAN_DWELL      0x20
#define IAR_DUAL_PAN_STS        0x21
#define IAR_CCA1_THRESH         0x22
#define IAR_CCA1_ED_OFFSET_COMP 0x23
#define IAR_LQI_OFFSET_COMP     0x24
#define IAR_CCA_CTRL            0x25
#define IAR_CCA2_CORR_PEAKS     0x26
#define IAR_CCA2_CORR_THRESH    0x27
#define IAR_TMR_PRESCALE        0x28
/*--------------------          0x29 */
#define IAR_GPIO_DATA           0x2A
#define IAR_GPIO_DIR            0x2B
#define IAR_GPIO_PUL_EN         0x2C
#define IAR_GPIO_PUL_SEL        0x2D
#define IAR_GPIO_DS             0x2E
/*------------------            0x2F */
#define IAR_ANT_PAD_CTRL        0x30
#define IAR_MISC_PAD_CTRL       0x31
#define IAR_BSM_CTRL            0x32
/*-------------------           0x33 */
#define IAR_RNG                 0x34
#define IAR_RX_BYTE_COUNT       0x35
#define IAR_RX_WTR_MARK         0x36
#define IAR_SOFT_RESET          0x37
#define IAR_TXDELAY             0x38
#define IAR_ACKDELAY            0x39
#define IAR_SEQ_MGR_CTRL        0x3A
#define IAR_SEQ_MGR_STS         0x3B
#define IAR_SEQ_T_STS           0x3C
#define IAR_ABORT_STS           0x3D
#define IAR_CCCA_BUSY_CNT       0x3E
#define IAR_SRC_ADDR_CHECKSUM1  0x3F
#define IAR_SRC_ADDR_CHECKSUM2  0x40
#define IAR_SRC_TBL_VALID1      0x41
#define IAR_SRC_TBL_VALID2      0x42
#define IAR_FILTERFAIL_CODE1    0x43
#define IAR_FILTERFAIL_CODE2    0x44
#define IAR_SLOT_PRELOAD        0x45
/*--------------------          0x46 */
#define IAR_CORR_VT             0x47
#define IAR_SYNC_CTRL           0x48
#define IAR_PN_LSB_0            0x49
#define IAR_PN_LSB_1            0x4A
#define IAR_PN_MSB_0            0x4B
#define IAR_PN_MSB_1            0x4C
#define IAR_CORR_NVAL           0x4D
#define IAR_TX_MODE_CTRL        0x4E
#define IAR_SNF_THR             0x4F
#define IAR_FAD_THR             0x50
#define IAR_ANT_AGC_CTRL        0x51
#define IAR_AGC_THR1            0x52
#define IAR_AGC_THR2            0x53
#define IAR_AGC_HYS             0x54
#define IAR_AFC					0x55
#define IAR_LPPS_CTRL			0x55
/*-------------------           0x56 */
/*-------------------           0x57 */
#define IAR_PHY_STS             0x58
#define IAR_RX_MAX_CORR         0x59
#define IAR_RX_MAX_PREAMBLE     0x5A
#define IAR_RSSI                0x5B
/*-------------------           0x5C */
/*-------------------           0x5D */
#define IAR_PLL_DIG_CTRL        0x5E
#define IAR_VCO_CAL             0x5F
#define IAR_VCO_BEST_DIFF       0x60
#define IAR_VCO_BIAS            0x61
#define IAR_KMOD_CTRL           0x62
#define IAR_KMOD_CAL            0x63
#define IAR_PA_CAL              0x64
#define IAR_PA_PWRCAL           0x65
#define IAR_ATT_RSSI1           0x66
#define IAR_ATT_RSSI2           0x67
#define IAR_RSSI_OFFSET         0x68
#define IAR_RSSI_SLOPE          0x69
#define IAR_RSSI_CAL1           0x6A
#define IAR_RSSI_CAL2           0x6B
/*-------------------           0x6C */
/*-------------------           0x6D */
#define IAR_XTAL_CTRL           0x6E
#define IAR_XTAL_COMP_MIN       0x6F
#define IAR_XTAL_COMP_MAX       0x70
#define IAR_XTAL_GM             0x71
/*-------------------           0x72 */
/*-------------------           0x73 */
#define IAR_LNA_TUNE            0x74
#define IAR_LNA_AGCGAIN         0x75
/*-------------------           0x76 */
/*-------------------           0x77 */
#define IAR_CHF_PMA_GAIN        0x78
#define IAR_CHF_IBUF            0x79
#define IAR_CHF_QBUF            0x7A
#define IAR_CHF_IRIN            0x7B
#define IAR_CHF_QRIN            0x7C
#define IAR_CHF_IL              0x7D
#define IAR_CHF_QL              0x7E
#define IAR_CHF_CC1             0x7F
#define IAR_CHF_CCL             0x80
#define IAR_CHF_CC2             0x81
#define IAR_CHF_IROUT           0x82
#define IAR_CHF_QROUT           0x83
/*-------------------           0x84 */
/*-------------------           0x85 */
#define IAR_RSSI_CTRL           0x86
/*-------------------           0x87 */
/*-------------------           0x88 */
#define IAR_PA_BIAS             0x89
#define IAR_PA_TUNING           0x8A
/*-------------------           0x8B */
/*-------------------           0x8C */
#define IAR_PMC_HP_TRIM         0x8D
#define IAR_VREGA_TRIM          0x8E
/*-------------------           0x8F */
/*-------------------           0x90 */
#define IAR_VCO_CTRL1           0x91
#define IAR_VCO_CTRL2           0x92
/*-------------------           0x93 */
/*-------------------           0x94 */
#define IAR_ANA_SPARE_OUT1      0x95
#define IAR_ANA_SPARE_OUT2      0x96
#define IAR_ANA_SPARE_IN        0x97
#define IAR_MISCELLANEOUS       0x98
/*-------------------           0x99 */
#define IAR_SEQ_MGR_OVRD0       0x9A
#define IAR_SEQ_MGR_OVRD1       0x9B
#define IAR_SEQ_MGR_OVRD2       0x9C
#define IAR_SEQ_MGR_OVRD3       0x9D
#define IAR_SEQ_MGR_OVRD4       0x9E
#define IAR_SEQ_MGR_OVRD5       0x9F
#define IAR_SEQ_MGR_OVRD6       0xA0
#define IAR_SEQ_MGR_OVRD7       0xA1
/*-------------------           0xA2 */
#define IAR_TESTMODE_CTRL       0xA3
#define IAR_DTM_CTRL1           0xA4
#define IAR_DTM_CTRL2           0xA5
#define IAR_ATM_CTRL1           0xA6
#define IAR_ATM_CTRL2           0xA7
#define IAR_ATM_CTRL3           0xA8
/*-------------------           0xA9 */
#define IAR_LIM_FE_TEST_CTRL    0xAA
#define IAR_CHF_TEST_CTRL       0xAB
#define IAR_VCO_TEST_CTRL       0xAC
#define IAR_PLL_TEST_CTRL       0xAD
#define IAR_PA_TEST_CTRL        0xAE
#define IAR_PMC_TEST_CTRL       0xAF
#define IAR_SCAN_DTM_PROTECT_1  0xFE
#define IAR_SCAN_DTM_PROTECT_0  0xFF

/* IRQSTS1 bits */
#define DAR_IRQSTS1_RX_FRM_PEND         BIT(7)
#define DAR_IRQSTS1_PLL_UNLOCK_IRQ      BIT(6)
#define DAR_IRQSTS1_FILTERFAIL_IRQ      BIT(5)
#define DAR_IRQSTS1_RXWTRMRKIRQ         BIT(4)
#define DAR_IRQSTS1_CCAIRQ              BIT(3)
#define DAR_IRQSTS1_RXIRQ               BIT(2)
#define DAR_IRQSTS1_TXIRQ               BIT(1)
#define DAR_IRQSTS1_SEQIRQ              BIT(0)

typedef union {
  u8 byte;
  struct{
    u8 SEQIRQ:1;
    u8 TXIRQ:1;
    u8 RXIRQ:1;
    u8 CCAIRQ:1;
    u8 RXWTRMRKIRQ:1;
    u8 FILTERFAIL_IRQ:1;
    u8 PLL_UNLOCK_IRQ:1;
    u8 RX_FRM_PEND:1;
  }bit;
} DAR_IRQSTS1_TAG;

/* IRQSTS2 bits */
#define DAR_IRQSTS2_CRCVALID            BIT(7)
#define DAR_IRQSTS2_CCA                 BIT(6)
#define DAR_IRQSTS2_SRCADDR             BIT(5)
#define DAR_IRQSTS2_PI                  BIT(4)
#define DAR_IRQSTS2_TMRSTATUS           BIT(3)
#define DAR_IRQSTS2_ASM_IRQ             BIT(2)
#define DAR_IRQSTS2_PB_ERR_IRQ          BIT(1)
#define DAR_IRQSTS2_WAKE_IRQ            BIT(0)

typedef union {
  u8 byte;
  struct{
    u8 WAKE_IRQ:1;
    u8 PB_ERR_IRQ:1;
    u8 ASM_IRQ:1;
    u8 TMRSTATUS:1;
    u8 PI:1;
    u8 SRCADDR:1;
    u8 CCA:1;
    u8 CRCVALID:1;
  }bit;
} DAR_IRQSTS2_TAG;

/* IRQSTS3 bits */
#define DAR_IRQSTS3_TMR4MSK             BIT(7)
#define DAR_IRQSTS3_TMR3MSK             BIT(6)
#define DAR_IRQSTS3_TMR2MSK             BIT(5)
#define DAR_IRQSTS3_TMR1MSK             BIT(4)
#define DAR_IRQSTS3_TMR4IRQ             BIT(3)
#define DAR_IRQSTS3_TMR3IRQ             BIT(2)
#define DAR_IRQSTS3_TMR2IRQ             BIT(1)
#define DAR_IRQSTS3_TMR1IRQ             BIT(0)

/* regIRQSTS3_tag */
typedef union {
  u8 byte;
  struct{
    u8 TMR1IRQ:1;
    u8 TMR2IRQ:1;
    u8 TMR3IRQ:1;
    u8 TMR4IRQ:1;
    u8 TMR1MSK:1;
    u8 TMR2MSK:1;
    u8 TMR3MSK:1;
    u8 TMR4MSK:1;
  }bit;
} DAR_IRQSTS3_TAG;

/* PHY_CTRL1 bits */
#define DAR_PHY_CTRL1_TMRTRIGEN			BIT(7)
#define DAR_PHY_CTRL1_SLOTTED			BIT(6)
#define DAR_PHY_CTRL1_CCABFRTX			BIT(5)
#define DAR_PHY_CTRL1_CCABFRTX_SHIFT	5
#define DAR_PHY_CTRL1_RXACKRQD			BIT(4)
#define DAR_PHY_CTRL1_AUTOACK			BIT(3)
#define DAR_PHY_CTRL1_XCVSEQ_MASK		0x07

typedef union {
  u8 byte;
  struct{
    u8 XCVSEQ:3;
    u8 AUTOACK:1;
    u8 RXACKRQD:1;
    u8 CCABFRTX:1;
    u8 SLOTTED:1;
    u8 TMRTRIGEN:1;
  }bit;
} DAR_PHY_CTRL1_TAG; 

/* PHY_CTRL2 bits */
#define DAR_PHY_CTRL2_CRC_MSK             BIT(7)
#define DAR_PHY_CTRL2_PLL_UNLOCK_MSK      BIT(6)
#define DAR_PHY_CTRL2_FILTERFAIL_MSK      BIT(5)
#define DAR_PHY_CTRL2_RX_WMRK_MSK         BIT(4)
#define DAR_PHY_CTRL2_CCAMSK              BIT(3)
#define DAR_PHY_CTRL2_RXMSK               BIT(2)
#define DAR_PHY_CTRL2_TXMSK               BIT(1)
#define DAR_PHY_CTRL2_SEQMSK              BIT(0)

typedef union {
  u8 byte;
  struct{
    u8 SEQMSK:1;
    u8 TXMSK:1;
    u8 RXMSK:1;
    u8 CCAMSK:1;
    u8 RX_WMRK_MSK:1;
    u8 FILTERFAIL_MSK:1;
    u8 PLL_UNLOCK_MSK:1;
    u8 CRC_MSK:1;
  }bit;
} DAR_PHY_CTRL2_TAG; 

/* PHY_CTRL3 bits */
#define DAR_PHY_CTRL3_TMR4CMP_EN          BIT(7)
#define DAR_PHY_CTRL3_TMR3CMP_EN          BIT(6)
#define DAR_PHY_CTRL3_TMR2CMP_EN          BIT(5)
#define DAR_PHY_CTRL3_TMR1CMP_EN          BIT(4)
#define DAR_PHY_CTRL3_ASM_MSK             BIT(2)
#define DAR_PHY_CTRL3_PB_ERR_MSK          BIT(1)
#define DAR_PHY_CTRL3_WAKE_MSK            BIT(0)

typedef union {
  u8 byte;
  struct{
    u8 WAKE_MSK:1;
    u8 PB_ERR_MSK:1;
    u8 ASM_MSK:1;
    u8 RESERVED:1;
    u8 TMR1CMP_EN:1;
    u8 TMR2CMP_EN:1;
    u8 TMR3CMP_EN:1;
    u8 TMR4CMP_EN:1;
  }bit;
} DAR_PHY_CTRL3_TAG;

/* RX_FRM_LEN bits */
#define DAR_RX_FRAME_LENGTH_MASK		(0x7F)

/* PHY_CTRL4 bits */
#define DAR_PHY_CTRL4_TRCV_MSK			BIT(7)
#define DAR_PHY_CTRL4_TC3TMOUT			BIT(6)
#define DAR_PHY_CTRL4_PANCORDNTR0		BIT(5)
#define DAR_PHY_CTRL4_CCATYPE			(3<<0)
#define DAR_PHY_CTRL4_CCATYPE_SHIFT     (3)
#define DAR_PHY_CTRL4_CCATYPE_MASK		(0x18)
#define DAR_PHY_CTRL4_TMRLOAD			BIT(2)
#define DAR_PHY_CTRL4_PROMISCUOUS		BIT(1)
#define DAR_PHY_CTRL4_TC2PRIME_EN		BIT(0)

typedef union {
  u8 byte;
  struct{
    u8 TC2PRIME_EN:1;
    u8 PROMISCUOUS:1;
    u8 TMRLOAD:1;
    u8 CCATYPE:2;
    u8 PANCORDNTR0:1;
    u8 TC3TMOUT:1;
    u8 TRCV_MSK:1;
  }bit;
} DAR_PHY_CTRL4_TAG;

/* SRC_CTRL bits */
#define DAR_SRC_CTRL_INDEX               (0x0F)
#define DAR_SRC_CTRL_INDEX_SHIFT       (4)
#define DAR_SRC_CTRL_ACK_FRM_PND         BIT(3)
#define DAR_SRC_CTRL_SRCADDR_EN          BIT(2)
#define DAR_SRC_CTRL_INDEX_EN            BIT(1)
#define DAR_SRC_CTRL_INDEX_DISABLE       BIT(0)

typedef union {
  u8 byte;
  struct{
    u8 INDEX_DISABLE:1;
    u8 INDEX_EN:1;
    u8 SRCADDR_EN:1;
    u8 ACK_FRM_PND:1;
    u8 INDEX:4;
  }bit;
} DAR_SRC_CTRL_TAG;

/* DAR_ASM_CTRL1 bits */
#define DAR_ASM_CTRL1_CLEAR               BIT(7)
#define DAR_ASM_CTRL1_START               BIT(6)
#define DAR_ASM_CTRL1_SELFTST             BIT(5)
#define DAR_ASM_CTRL1_CTR                 BIT(4)
#define DAR_ASM_CTRL1_CBC                 BIT(3)
#define DAR_ASM_CTRL1_AES                 BIT(2)
#define DAR_ASM_CTRL1_LOAD_MAC            BIT(1)

/* DAR_ASM_CTRL2 bits */
#define DAR_ASM_CTRL2_DATA_REG_TYPE_SEL			(7)
#define DAR_ASM_CTRL2_DATA_REG_TYPE_SEL_SHIFT	(5)
#define DAR_ASM_CTRL2_TSTPAS					BIT(1)

/* DAR_CLK_OUT_CTRL bits */
#define DAR_CLK_OUT_CTRL_EXTEND           BIT(7)
#define DAR_CLK_OUT_CTRL_HIZ              BIT(6)
#define DAR_CLK_OUT_CTRL_SR               BIT(5)
#define DAR_CLK_OUT_CTRL_DS               BIT(4)
#define DAR_CLK_OUT_CTRL_EN               BIT(3)
#define DAR_CLK_OUT_CTRL_DIV              (7)

/* DAR_PWR_MODES bits */
#define DAR_PWR_MODES_XTAL_READY          BIT(5)
#define DAR_PWR_MODES_XTALEN              BIT(4)
#define DAR_PWR_MODES_ASM_CLK_EN          BIT(3)
#define DAR_PWR_MODES_AUTODOZE            BIT(1)
#define DAR_PWR_MODES_PMC_MODE            BIT(0)

/* RX_FRAME_FILTER bits */
#define IAR_RX_FRAME_FLT_FRM_VER             (0xC0)
#define IAR_RX_FRAME_FLT_FRM_VER_SHIFT			(6)
#define IAR_RX_FRAME_FLT_ACTIVE_PROMISCUOUS  BIT(5)
#define IAR_RX_FRAME_FLT_NS_FT               BIT(4)
#define IAR_RX_FRAME_FLT_CMD_FT              BIT(3)
#define IAR_RX_FRAME_FLT_ACK_FT              BIT(2)
#define IAR_RX_FRAME_FLT_DATA_FT             BIT(1)
#define IAR_RX_FRAME_FLT_BEACON_FT           BIT(0)

typedef union IAR_RX_FRAME_FILTER_TAG{
  u8 byte;
  struct{
    u8 FRAME_FLT_BEACON_FT:1;
    u8 FRAME_FLT_DATA_FT:1;
    u8 FRAME_FLT_ACK_FT:1;
    u8 FRAME_FLT_CMD_FT:1;
    u8 FRAME_FLT_NS_FT:1;
    u8 FRAME_FLT_ACTIVE_PROMISCUOUS:1;
    u8 FRAME_FLT_FRM_VER:2;
  }bit;
} IAR_RX_FRAME_FILTER_TAG; 

/* DUAL_PAN_CTRL bits */
#define IAR_DUAL_PAN_CTRL_DUAL_PAN_SAM_LVL_MSK      (0xF0)
#define IAR_DUAL_PAN_CTRL_DUAL_PAN_SAM_LVL_SHIFT    (4)
#define IAR_DUAL_PAN_CTRL_CURRENT_NETWORK           BIT(3)
#define IAR_DUAL_PAN_CTRL_PANCORDNTR1               BIT(2)
#define IAR_DUAL_PAN_CTRL_DUAL_PAN_AUTO             BIT(1)
#define IAR_DUAL_PAN_CTRL_ACTIVE_NETWORK            BIT(0)

/* DUAL_PAN_STS bits */
#define IAR_DUAL_PAN_STS_RECD_ON_PAN1        BIT(7)
#define IAR_DUAL_PAN_STS_RECD_ON_PAN0        BIT(6)
#define IAR_DUAL_PAN_STS_DUAL_PAN_REMAIN     (0x3F)

/* CCA_CTRL bits */
#define IAR_CCA_CTRL_AGC_FRZ_EN			BIT(6)
#define IAR_CCA_CTRL_CONT_RSSI_EN		BIT(5)
#define IAR_CCA_CTRL_LQI_RSSI_NOT_CORR	BIT(4)
#define IAR_CCA_CTRL_CCA3_AND_NOT_OR	BIT(3)
#define IAR_CCA_CTRL_POWER_COMP_EN_LQI	BIT(2)
#define IAR_CCA_CTRL_POWER_COMP_EN_ED	BIT(1)
#define IAR_CCA_CTRL_POWER_COMP_EN_CCA1	BIT(0)

/* GPIO_DATA bits */
#define IAR_GPIO_DATA_7		BIT(7)
#define IAR_GPIO_DATA_6		BIT(6)
#define IAR_GPIO_DATA_5		BIT(5)
#define IAR_GPIO_DATA_4		BIT(4)
#define IAR_GPIO_DATA_3		BIT(3)
#define IAR_GPIO_DATA_2		BIT(2)
#define IAR_GPIO_DATA_1		BIT(1)
#define IAR_GPIO_DATA_0		BIT(0)

/* GPIO_DIR bits */
#define IAR_GPIO_DIR_7		BIT(7)
#define IAR_GPIO_DIR_6		BIT(6)
#define IAR_GPIO_DIR_5		BIT(5)
#define IAR_GPIO_DIR_4		BIT(4)
#define IAR_GPIO_DIR_3		BIT(3)
#define IAR_GPIO_DIR_2		BIT(2)
#define IAR_GPIO_DIR_1		BIT(1)
#define IAR_GPIO_DIR_0		BIT(0)

/* GPIO_PUL_EN bits */
#define IAR_GPIO_PUL_EN_7      BIT(7)
#define IAR_GPIO_PUL_EN_6      BIT(6)
#define IAR_GPIO_PUL_EN_5      BIT(5)
#define IAR_GPIO_PUL_EN_4      BIT(4)
#define IAR_GPIO_PUL_EN_3      BIT(3)
#define IAR_GPIO_PUL_EN_2      BIT(2)
#define IAR_GPIO_PUL_EN_1      BIT(1)
#define IAR_GPIO_PUL_EN_0      BIT(0)

/* GPIO_PUL_SEL bits */
#define IAR_GPIO_PUL_SEL_7     BIT(7)
#define IAR_GPIO_PUL_SEL_6     BIT(6)
#define IAR_GPIO_PUL_SEL_5     BIT(5)
#define IAR_GPIO_PUL_SEL_4     BIT(4)
#define IAR_GPIO_PUL_SEL_3     BIT(3)
#define IAR_GPIO_PUL_SEL_2     BIT(2)
#define IAR_GPIO_PUL_SEL_1     BIT(1)
#define IAR_GPIO_PUL_SEL_0     BIT(0)

/* GPIO_DS bits */
#define IAR_GPIO_DS_7          BIT(7)
#define IAR_GPIO_DS_6          BIT(6)
#define IAR_GPIO_DS_5          BIT(5)
#define IAR_GPIO_DS_4          BIT(4)
#define IAR_GPIO_DS_3          BIT(3)
#define IAR_GPIO_DS_2          BIT(2)
#define IAR_GPIO_DS_1          BIT(1)
#define IAR_GPIO_DS_0          BIT(0)

/* ANT_PAD_CTRL bits */
#define IAR_ANT_PAD_CTRL_ANTX_POL           (0x0F)
#define IAR_ANT_PAD_CTRL_ANTX_POL_Shift_c   (4)
#define IAR_ANT_PAD_CTRL_ANTX_CTRLMODE      BIT(3)
#define IAR_ANT_PAD_CTRL_ANTX_HZ            BIT(2)
#define IAR_ANT_PAD_CTRL_ANTX_EN            (3)

/* MISC_PAD_CTRL bits */
#define IAR_MISC_PAD_CTRL_MISO_HIZ_EN        BIT(3)
#define IAR_MISC_PAD_CTRL_IRQ_B_OD           BIT(2)
#define IAR_MISC_PAD_CTRL_NON_GPIO_DS        BIT(1)
#define IAR_MISC_PAD_CTRL_ANTX_CURR          (1<<0)

/* ANT_AGC_CTRL bits */
#define IAR_ANT_AGC_CTRL_FAD_EN_SHIFT		(0)
#define IAR_ANT_AGC_CTRL_FAD_EN_MASK		(1<<IAR_ANT_AGC_CTRL_FAD_EN_SHIFT)
#define IAR_ANT_AGC_CTRL_ANTX_SHIFT			(1)
#define IAR_ANT_AGC_CTRL_ANTX_MASK			(1<<IAR_ANT_AGC_CTRL_ANTX_SHIFT)

/* BSM_CTRL bits */
#define cBSM_CTRL_BSM_EN                  (1<<0)

/* SOFT_RESET bits */
#define IAR_SOFT_RESET_SOG_RST            BIT(7)
#define IAR_SOFT_RESET_REGS_RST           BIT(4)
#define IAR_SOFT_RESET_PLL_RST            BIT(3)
#define IAR_SOFT_RESET_TX_RST             BIT(2)
#define IAR_SOFT_RESET_RX_RST             BIT(1)
#define IAR_SOFT_RESET_SEQ_MGR_RST        BIT(0)

/* SEQ_MGR_CTRL bits */
#define IAR_SEQ_MGR_CTRL_SEQ_STATE_CTRL			(3)
#define IAR_cSEQ_MGR_CTRL_SEQ_STATE_CTRL_SHIFT	(6)
#define IAR_cSEQ_MGR_CTRL_NO_RX_RECYCLE			BIT(5)
#define IAR_SEQ_MGR_CTRL_LATCH_PREAMBLE			BIT(4)
#define IAR_SEQ_MGR_CTRL_EVENT_TMR_DO_NOT_LATCH	BIT(3)
#define IAR_SEQ_MGR_CTRL_CLR_NEW_SEQ_INHIBIT	BIT(2)
#define IAR_SEQ_MGR_CTRL_PSM_LOCK_DIS			BIT(1)
#define IAR_SEQ_MGR_CTRL_PLL_ABORT_OVRD			BIT(0)

/* SEQ_MGR_STS bits */
#define IAR_SEQ_MGR_STS_TMR2_SEQ_TRIG_ARMED		BIT(7)
#define IAR_SEQ_MGR_STS_RX_MODE					BIT(6)
#define IAR_SEQ_MGR_STS_RX_TIMEOUT_PENDING		BIT(5)
#define IAR_SEQ_MGR_STS_NEW_SEQ_INHIBIT			BIT(4)
#define IAR_SEQ_MGR_STS_SEQ_IDLE				BIT(3)
#define IAR_cSEQ_MGR_STS_XCVSEQ_ACTUAL			(7)

/* ABORT_STS bits */
#define IAR_ABORT_STS_PLL_ABORTED        		BIT(2)
#define IAR_ABORT_STS_TC3_ABORTED        		BIT(1)
#define IAR_ABORT_STS_SW_ABORTED         		BIT(0)

/* IAR_FILTERFAIL_CODE2 bits */
#define IAR_FILTERFAIL_CODE2_PAN_SEL  BIT(7)
#define IAR_FILTERFAIL_CODE2_9_8      (3)

/* PHY_STS bits */
#define IAR_PHY_STS_PLL_UNLOCK			BIT(7)
#define IAR_PHY_STS_PLL_LOCK_ERR		BIT(6)
#define IAR_PHY_STS_PLL_LOCK            BIT(5)
#define IAR_PHY_STS_CRCVALID			BIT(3)
#define IAR_PHY_STS_FILTERFAIL_FLAG_SEL	BIT(2)
#define IAR_PHY_STS_SFD_DET             BIT(1)
#define IAR_PHY_STS_PREAMBLE_DET        BIT(0)

/* TESTMODE_CTRL bits */
#define IAR_TEST_MODE_CTRL_HOT_ANT            BIT(4)
#define IAR_TEST_MODE_CTRL_IDEAL_RSSI_EN      BIT(3)
#define IAR_TEST_MODE_CTRL_IDEAL_PFC_EN       BIT(2)
#define IAR_TEST_MODE_CTRL_CONTINUOUS_EN      BIT(1)
#define IAR_TEST_MODE_CTRL_FPGA_EN            BIT(0)

/* DTM_CTRL1 bits */
#define IAR_DTM_CTRL1_ATM_LOCKED	BIT(7)
#define IAR_DTM_CTRL1_DTM_EN		BIT(6)
#define IAR_DTM_CTRL1_PAGE5			BIT(5)
#define IAR_DTM_CTRL1_PAGE4			BIT(4)
#define IAR_DTM_CTRL1_PAGE3			BIT(3)
#define IAR_DTM_CTRL1_PAGE2			BIT(2)
#define IAR_DTM_CTRL1_PAGE1			BIT(1)
#define IAR_DTM_CTRL1_PAGE0			BIT(0)

/* TX_MODE_CTRL */
#define IAR_TX_MODE_CTRL_TX_INV		BIT(4)
#define IAR_TX_MODE_CTRL_BT_EN		BIT(3)
#define IAR_TX_MODE_CTRL_DTS2		BIT(2)
#define IAR_TX_MODE_CTRL_DTS1		BIT(1)
#define IAR_TX_MODE_CTRL_DTS0		BIT(0)

#define cTX_MODE_CTRL_DTS_MASK (7)

/* DAR_CLK_OUT_CTRL bits */
#define DAR_CLK_OUT_EXTEND        BIT(7)
#define DAR_CLK_OUT_HIZ           BIT(6)
#define DAR_CLK_OUT_SR            BIT(5)
#define DAR_CLK_OUT_DS            BIT(4)
#define DAR_CLK_OUT_EN            BIT(3)
#define DAR_CLK_OUT_DIV_Mask      (7)

#define CLK_OUT_FREQ_32_MHz      (0)
#define CLK_OUT_FREQ_16_MHz      (1)
#define CLK_OUT_FREQ_8_MHz       (2)
#define CLK_OUT_FREQ_4_MHz       (3)
#define CLK_OUT_FREQ_1_MHz       (4)
#define CLK_OUT_FREQ_250_KHz     (5)
#define CLK_OUT_FREQ_62_5_KHz    (6)
#define CLK_OUT_FREQ_32_78_KHz   (7)
#define CLK_OUT_FREQ_DISABLE     (8)


#endif /* !_MCR20A_H */
