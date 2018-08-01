/*
 * DecaWave DW1000 IEEE 802.15.4 UWB wireless driver
 *
 * Copyright (C) 2018 Michael Brown <mbrown@fensystems.co.uk>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __DW1000_H
#define __DW1000_H

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/timecounter.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/hwmon.h>
#include "timehires.h"
#include "kcompat.h"

/* Supported channel page (UWB only) */
#define DW1000_CHANNEL_PAGE 4

/* Supported channels */
#define DW1000_CHANNELS (BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(7))

/* Default channel */
#define DW1000_CHANNEL_DEFAULT 7

/* Maximum SPI bus speed when PLL is not yet locked */
#define DW1000_SPI_SLOW_HZ 3000000

/* Maximum SPI bus speed when PLL is locked */
#define DW1000_SPI_FAST_HZ 20000000

/* SPI command */
struct dw1000_spi_command {
	/* Register file and command */
	uint8_t file;
	/* Low order 7 bits and extended address indicator */
	uint8_t low;
	/* High order 8 bits */
	uint8_t high;
} __packed;

/* SPI bus header command flags */
#define DW1000_WRITE 0x80
#define DW1000_OFFSET 0x40

/* SPI bus header extended offset present flags */
#define DW1000_EXTENDED 0x80

/* SPI bus header extended offset shift */
#define DW1000_EXTENDED_SHIFT 7

/* SPI transfer set */
struct dw1000_spi_transfers {
	/* SPI command */
	struct dw1000_spi_command command;
	/* SPI bus header transfer */
	struct spi_transfer header;
	/* SPI bus data transfer */
	struct spi_transfer data;
};

/* EUI-64 extended address format */
union dw1000_eui64 {
	/* OTP values */
	uint32_t otp[2];
	/* Raw bytes */
	uint8_t raw[8];
	/* IEEE 802.15.4 EUI-64 address */
	__le64 addr;
};

/* LDO tuning register format */
union dw1000_ldotune {
	/* OTP values */
	uint32_t otp[2];
	/* Raw bytes */
	uint8_t raw[5];
};

/* Timestamp format */
union dw1000_timestamp {
	/* Cycle count */
	__le64 cc;
	/* Raw bytes */
	uint8_t raw[5];
};

/* Timestamp mask */
#define DW1000_TIMESTAMP_MASK 0xffffffffffUL

/* Frame quality register format */
struct dw1000_rx_fqual {
	/* Standard deviation of noise */
	__le16 std_noise;
	/* First path amplitude point 2 */
	__le16 fp_ampl2;
	/* First path amplitude point 3 */
	__le16 fp_ampl3;
	/* Channel impulse response power */
	__le16 cir_pwr;
} __packed;

/* Receive time stamp register format */
union dw1000_rx_time {
	/* Receive time stamp */
	union dw1000_timestamp rx_stamp;
	struct {
		/* Receive time stamp (padding) */
		uint8_t pad[5];
		/* First path index */
		__le16 fp_index;
		/* First path amplitude point 1 */
		__le16 fp_ampl1;
	} __packed;
};

/* Receive time tracking interval */
union dw1000_rx_ttcki {
	/* Tracking interval */
	__le32 tcki;
	/* Raw bytes */
	uint8_t raw[4];
};

/* Receive time tracking offset */
union dw1000_rx_ttcko {
	/* Tracking offset */
	__le32 tofs;
	/* Raw bytes */
	uint8_t raw[3];
};

/* Register files */
#define DW1000_DEV_ID		0x00
#define DW1000_DEV_ID_LEN	4
#define DW1000_EUI		0x01
#define DW1000_EUI_LEN		8
#define DW1000_PANADR		0x03
#define DW1000_PANADR_LEN	4
#define DW1000_SYS_CFG		0x04
#define DW1000_SYS_CFG_LEN	4
#define DW1000_SYS_TIME		0x06
#define DW1000_SYS_TIME_LEN	5
#define DW1000_TX_FCTRL		0x08
#define DW1000_TX_FCTRL_LEN	5
#define DW1000_TX_BUFFER	0x09
#define DW1000_TX_BUFFER_LEN	1024
#define DW1000_DX_TIME		0x0a
#define DW1000_DX_TIME_LEN	5
#define DW1000_RX_FWTO		0x0c
#define DW1000_RX_FWTO_LEN	2
#define DW1000_SYS_CTRL		0x0d
#define DW1000_SYS_CTRL_LEN	4
#define DW1000_SYS_MASK		0x0e
#define DW1000_SYS_MASK_LEN	4
#define DW1000_SYS_STATUS	0x0f
#define DW1000_SYS_STATUS_LEN	4
#define DW1000_RX_FINFO		0x10
#define DW1000_RX_FINFO_LEN	4
#define DW1000_RX_BUFFER	0x11
#define DW1000_RX_BUFFER_LEN	1024
#define DW1000_RX_FQUAL		0x12
#define DW1000_RX_FQUAL_LEN	8
#define DW1000_RX_TTCKI		0x13
#define DW1000_RX_TTCKI_LEN	4
#define DW1000_RX_TTCKO		0x14
#define DW1000_RX_TTCKO_LEN	5
#define DW1000_RX_TIME		0x15
#define DW1000_RX_TIME_LEN	14
#define DW1000_TX_TIME		0x17
#define DW1000_TX_TIME_LEN	10
#define DW1000_TX_ANTD		0x18
#define DW1000_TX_ANTD_LEN	2
#define DW1000_SYS_STATE	0x19
#define DW1000_SYS_STATE_LEN	3
#define DW1000_ACK_RESP_T	0x1a
#define DW1000_ACK_RESP_T_LEN	4
#define DW1000_RX_SNIFF		0x1d
#define DW1000_RX_SNIFF_LEN	4
#define DW1000_TX_POWER		0x1e
#define DW1000_TX_POWER_LEN	4
#define DW1000_CHAN_CTRL	0x1f
#define DW1000_CHAN_CTRL_LEN	4
#define DW1000_USR_SFD		0x21
#define DW1000_USR_SFD_LEN	41
#define DW1000_AGC_CTRL		0x23
#define DW1000_AGC_CTRL_LEN	33
#define DW1000_EXT_SYNC		0x24
#define DW1000_EXT_SYNC_LEN	12
#define DW1000_ACC_MEM		0x25
#define DW1000_ACC_MEM_LEN	4064 /* sic */
#define DW1000_GPIO_CTRL	0x26
#define DW1000_GPIO_CTRL_LEN	44
#define DW1000_DRX_CONF		0x27
#define DW1000_DRX_CONF_LEN	46
#define DW1000_RF_CONF		0x28
#define DW1000_RF_CONF_LEN	58
#define DW1000_TX_CAL		0x2a
#define DW1000_TX_CAL_LEN	52
#define DW1000_FS_CTRL		0x2b
#define DW1000_FS_CTRL_LEN	21
#define DW1000_AON		0x2c
#define DW1000_AON_LEN		12
#define DW1000_OTP_IF		0x2d
#define DW1000_OTP_IF_LEN	18
#define DW1000_LDE_IF		0x2e
#define DW1000_LDE_IF_LEN	0x2806
#define DW1000_DIG_DIAG		0x2f
#define DW1000_DIG_DIAG_LEN	41
#define DW1000_PMSC		0x36
#define DW1000_PMSC_LEN		48

/* Device ID registers */
#define DW1000_MODELVERREV		0x00
#define DW1000_MODELVERREV_MODEL(val)		(((val) >> 8) & 0xff)
#define DW1000_MODELVERREV_VER(val)		(((val) >> 4) & 0x0f)
#define DW1000_MODELVERREV_REV(val)		(((val) >> 0) & 0x0f)
#define DW1000_RIDTAG			0x02
#define DW1000_RIDTAG_MAGIC			0xdeca

/* PAN address registers */
#define DW1000_PANADR_SHORT_ADDR	0x00
#define DW1000_PANADR_PAN_ID		0x02

/* System configuration register */
#define DW1000_SYS_CFG_FFEN			0x00000001UL
#define DW1000_SYS_CFG_FFBC			0x00000002UL
#define DW1000_SYS_CFG_FFAB			0x00000004UL
#define DW1000_SYS_CFG_FFAD			0x00000008UL
#define DW1000_SYS_CFG_FFAA			0x00000010UL
#define DW1000_SYS_CFG_FFAM			0x00000020UL
#define DW1000_SYS_CFG_FFAR			0x00000040UL
#define DW1000_SYS_CFG_FFA4			0x00000080UL
#define DW1000_SYS_CFG_FFA5			0x00000100UL
#define DW1000_SYS_CFG_DIS_DRXB			0x00001000UL
#define DW1000_SYS_CFG_DIS_STXP			0x00040000UL
#define DW1000_SYS_CFG_RXM110K			0x00400000UL
#define DW1000_SYS_CFG_RXAUTR			0x20000000UL

/* Transmit frame control register */
#define DW1000_TX_FCTRL0		0x00
#define DW1000_TX_FCTRL0_TFLEN(n)		((n) << 0)
#define DW1000_TX_FCTRL1		0x01
#define DW1000_TX_FCTRL1_TXBR(n)		((n) << 5)
#define DW1000_TX_FCTRL1_TXBR_MASK		DW1000_TX_FCTRL1_TXBR(0x3)
#define DW1000_TX_FCTRL2		0x02
#define DW1000_TX_FCTRL2_TXPRF(n)		((n) << 0)
#define DW1000_TX_FCTRL2_TXPRF_MASK		DW1000_TX_FCTRL2_TXPRF(0x3)
#define DW1000_TX_FCTRL2_TXPSR(n)		((n) << 2)
#define DW1000_TX_FCTRL2_TXPSR_MASK		DW1000_TX_FCTRL2_TXPSR(0xf)
#define DW1000_TX_FCTRL4		0x04
#define DW1000_TX_FCTRL4_IFSDELAY(n)		(((n) - 6) << 0)
#define DW1000_TX_FCTRL4_IFSDELAY_MASK		DW1000_TX_FCTRL4_IFSDELAY(0xff)

/* System control register */
#define DW1000_SYS_CTRL0		0x00
#define DW1000_SYS_CTRL0_SFCST			0x01
#define DW1000_SYS_CTRL0_TXSTRT			0x02
#define DW1000_SYS_CTRL0_CANSFCS		0x08
#define DW1000_SYS_CTRL0_TRXOFF			0x40
#define DW1000_SYS_CTRL0_WAIT4RESP		0x80
#define DW1000_SYS_CTRL1		0x01
#define DW1000_SYS_CTRL1_RXENAB			0x01
#define DW1000_SYS_CTRL3		0x03
#define DW1000_SYS_CTRL3_HRBPT			0x01

/* System event mask register */
#define DW1000_SYS_MASK_MTXFRS			0x00000080UL
#define DW1000_SYS_MASK_MRXDFR			0x00002000UL
#define DW1000_SYS_MASK_MRXFCG			0x00004000UL
#define DW1000_SYS_MASK_MRXOVRR			0x00100000UL

#define DW1000_SYS_MASK_MACTIVE		( \
	DW1000_SYS_MASK_MTXFRS		| \
	DW1000_SYS_MASK_MRXDFR		| \
	DW1000_SYS_MASK_MRXOVRR 	)

/* System event status register */
#define DW1000_SYS_STATUS_CPLOCK		0x00000002UL
#define DW1000_SYS_STATUS_TXFRB			0x00000010UL
#define DW1000_SYS_STATUS_TXPRS			0x00000020UL
#define DW1000_SYS_STATUS_TXPHS			0x00000040UL
#define DW1000_SYS_STATUS_TXFRS			0x00000080UL
#define DW1000_SYS_STATUS_RXPRD			0x00000100UL
#define DW1000_SYS_STATUS_RXSFDD		0x00000200UL
#define DW1000_SYS_STATUS_LDEDONE		0x00000400UL
#define DW1000_SYS_STATUS_RXPHD			0x00000800UL
#define DW1000_SYS_STATUS_RXPHE			0x00001000UL
#define DW1000_SYS_STATUS_RXDFR			0x00002000UL
#define DW1000_SYS_STATUS_RXFCG			0x00004000UL
#define DW1000_SYS_STATUS_RXFCE			0x00008000UL
#define DW1000_SYS_STATUS_RXRFSL		0x00010000UL
#define DW1000_SYS_STATUS_RXRFTO		0x00020000UL
#define DW1000_SYS_STATUS_LDEERR		0x00040000UL
#define DW1000_SYS_STATUS_RXOVRR		0x00100000UL
#define DW1000_SYS_STATUS_RXPTO			0x00200000UL
#define DW1000_SYS_STATUS_SLP2INIT		0x00800000UL
#define DW1000_SYS_STATUS_RFPLL_LL		0x01000000UL
#define DW1000_SYS_STATUS_CLKPLL_LL		0x02000000UL
#define DW1000_SYS_STATUS_RXSFDTO		0x04000000UL
#define DW1000_SYS_STATUS_HSRBP			0x40000000UL
#define DW1000_SYS_STATUS_ICRBP			0x80000000UL
#define DW1000_SYS_STATUS0		0x00
#define DW1000_SYS_STATUS0_
#define DW1000_SYS_STATUS0_TXFRB		0x10
#define DW1000_SYS_STATUS0_TXPRS		0x20
#define DW1000_SYS_STATUS0_TXPHS		0x40
#define DW1000_SYS_STATUS0_TXFRS		0x80
#define DW1000_SYS_STATUS1		0x01
#define DW1000_SYS_STATUS2		0x02
#define DW1000_SYS_STATUS2_RXOVRR		0x10

/* Clear-on-write RX status bits */
#define DW1000_RX_STATE_CLEAR		( \
	DW1000_SYS_STATUS_RXPRD		| \
	DW1000_SYS_STATUS_RXSFDD 	| \
	DW1000_SYS_STATUS_LDEDONE	| \
	DW1000_SYS_STATUS_RXPHD		| \
	DW1000_SYS_STATUS_RXPHE		| \
	DW1000_SYS_STATUS_RXDFR		| \
	DW1000_SYS_STATUS_RXFCG		| \
	DW1000_SYS_STATUS_RXFCE		| \
	DW1000_SYS_STATUS_RXRFSL 	| \
	DW1000_SYS_STATUS_RXRFTO 	| \
	DW1000_SYS_STATUS_LDEERR 	| \
	DW1000_SYS_STATUS_RXPTO 	| \
	DW1000_SYS_STATUS_RXSFDTO	)

/* RX HSRBP-ICRBP sync condition */
#define DW1000_HSRPB_SYNC(status) 	\
	(!!((status) & DW1000_SYS_STATUS_HSRBP) != \
	 !!((status) & DW1000_SYS_STATUS_ICRBP))

/* System status register */
#define DW1000_RX_STATUS_HSRBP(val)		(((val) >> 30) & 0x1)

/* Receive frame information register */
#define DW1000_RX_FINFO_RXFLEN(val)		(((val) >> 0) & 0x7ff)
#define DW1000_RX_FINFO_RXNSPL(val)		(((val) >> 11) & 0x3)
#define DW1000_RX_FINFO_RXPSR(val)		(((val) >> 18) & 0x3)
#define DW1000_RX_FINFO_RXPACC(val)		(((val) >> 20) & 0xfff)

/* Transmit time stamp registers */
#define DW1000_TX_STAMP			0x00
#define DW1000_TX_RAWST			0x05

/* System state register */
#define DW1000_SYS_STATE_TX		0x00
#define DW1000_SYS_STATE_RX		0x01
#define DW1000_SYS_STATE_RX_IDLE		0x00
#define DW1000_SYS_STATE_PMSC		0x02
#define DW1000_SYS_STATE_PMSC_RX		0x05

/* Channel control register */
#define DW1000_CHAN_CTRL_TX_CHAN(n)		((n) << 0)
#define DW1000_CHAN_CTRL_TX_CHAN_MASK		DW1000_CHAN_CTRL_TX_CHAN(0xf)
#define DW1000_CHAN_CTRL_RX_CHAN(n)		((n) << 4)
#define DW1000_CHAN_CTRL_RX_CHAN_MASK		DW1000_CHAN_CTRL_RX_CHAN(0xf)
#define DW1000_CHAN_CTRL_RXPRF(n)		((n) << 18)
#define DW1000_CHAN_CTRL_RXPRF_MASK		DW1000_CHAN_CTRL_RXPRF(0x3)
#define DW1000_CHAN_CTRL_TX_PCODE(n)		((n) << 22)
#define DW1000_CHAN_CTRL_TX_PCODE_MASK		DW1000_CHAN_CTRL_TX_PCODE(0x1f)
#define DW1000_CHAN_CTRL_RX_PCODE(n)		((n) << 27)
#define DW1000_CHAN_CTRL_RX_PCODE_MASK		DW1000_CHAN_CTRL_RX_PCODE(0x1f)

/* Automatic gain control registers */
#define DW1000_AGC_TUNE1		0x04
#define DW1000_AGC_TUNE2		0x0c
#define DW1000_AGC_TUNE3		0x12
#define DW1000_AGC_STAT1		0x1e
#define DW1000_AGC_STAT1_LEN		3
#define DW1000_AGC_STAT1_EDG1(val)		(((val) >> 6) & 0x1f)
#define DW1000_AGC_STAT1_EDV2(val)		(((val) >> 11) & 0x1ff)

/* GPIO control registers */
#define DW1000_GPIO_MODE		0x00
#define DW1000_GPIO_MODE_MSGP0_RXOKLED		0x00000040UL
#define DW1000_GPIO_MODE_MSGP0_MASK		0x000000c0UL
#define DW1000_GPIO_MODE_MSGP1_SFDLED		0x00000100UL
#define DW1000_GPIO_MODE_MSGP1_MASK		0x00000300UL
#define DW1000_GPIO_MODE_MSGP2_RXLED		0x00000400UL
#define DW1000_GPIO_MODE_MSGP2_MASK		0x00000c00UL
#define DW1000_GPIO_MODE_MSGP3_TXLED		0x00001000UL
#define DW1000_GPIO_MODE_MSGP3_MASK		0x00003000UL

/* Digital receiver configuration registers */
#define DW1000_DRX_TUNE0B		0x02
#define DW1000_DRX_TUNE1A		0x04
#define DW1000_DRX_TUNE1B		0x06
#define DW1000_DRX_TUNE2		0x08
#define DW1000_DRX_TUNE4H		0x26

/* Analog RF configuration registers */
#define DW1000_RF_RXCTRLH		0x0b
#define DW1000_RF_TXCTRL		0x0c
#define DW1000_RF_SENSOR_HACK_A		0x11
#define DW1000_RF_SENSOR_HACK_A1		0x80
#define DW1000_RF_SENSOR_HACK_B		0x12
#define DW1000_RF_SENSOR_HACK_B1		0x0a
#define DW1000_RF_SENSOR_HACK_B2		0x0f
#define DW1000_RF_LDOTUNE		0x30

/* Transmitter calibration registers */
#define DW1000_TC_SARC			0x00
#define DW1000_TC_SARC_CTRL			0x01
#define DW1000_TC_SARL_LVBAT		0x03
#define DW1000_TC_SARL_LTEMP		0x04
#define DW1000_TC_PGDELAY		0x0b

/* Frequency synthesiser control registers */
#define DW1000_FS_PLLCFG		0x07
#define DW1000_FS_PLLTUNE		0x0b
#define DW1000_FS_XTALT			0x0e
#define DW1000_FS_XTALT_XTALT(n)		((n) << 0)
#define DW1000_FS_XTALT_XTALT_MIDPOINT		DW1000_FS_XTALT_XTALT(0x0f)
#define DW1000_FS_XTALT_XTALT_MASK		DW1000_FS_XTALT_XTALT(0x1f)

/* One-time programmable memory interface registers */
#define DW1000_OTP_ADDR			0x04
#define DW1000_OTP_CTRL			0x06
#define DW1000_OTP_CTRL_OTPRDEN			0x0001
#define DW1000_OTP_CTRL_OTPREAD			0x0002
#define DW1000_OTP_CTRL_LDELOAD			0x8000
#define DW1000_OTP_RDAT			0x0a

/* Leading edge detection interface registers */
#define DW1000_LDE_RXANTD		0x1804
#define DW1000_LDE_CFG2			0x1806
#define DW1000_LDE_REPC			0x2804

/* Antenna delay mask */
#define DW1000_ANTD_MASK			0xffff

/* Digital diagnostic registers */
#define DW1000_EVC_CTRL			0x00
#define DW1000_EVC_CTRL_EVC_EN			0x0001
#define DW1000_EVC_CTRL_EVC_CLR			0x0002
#define DW1000_EVC_OVR			0x0e
#define DW1000_EVC_OVR_MASK			0x0fff

/* Power management and system control registers */
#define DW1000_PMSC_CTRL0		0x00
#define DW1000_PMSC_CTRL0_SYSCLKS_MASK		0x00000003UL
#define DW1000_PMSC_CTRL0_SYSCLKS_AUTO		0x00000000UL
#define DW1000_PMSC_CTRL0_SYSCLKS_SLOW		0x00000001UL
#define DW1000_PMSC_CTRL0_RXCLKS_MASK		0x0000000cUL
#define DW1000_PMSC_CTRL0_RXCLKS_FAST		0x00000008UL
#define DW1000_PMSC_CTRL0_TXCLKS_MASK		0x00000030UL
#define DW1000_PMSC_CTRL0_TXCLKS_FAST		0x00000020UL
#define DW1000_PMSC_CTRL0_LDE_HACK_PRE		0x00000300UL
#define DW1000_PMSC_CTRL0_LDE_HACK_POST		0x00000200UL
#define DW1000_PMSC_CTRL0_LDE_HACK_MASK		0x00000300UL
#define DW1000_PMSC_CTRL0_GPCE			0x00010000UL
#define DW1000_PMSC_CTRL0_GPRN			0x00020000UL
#define DW1000_PMSC_CTRL0_GPDCE			0x00040000UL
#define DW1000_PMSC_CTRL0_GPDRN			0x00080000UL
#define DW1000_PMSC_CTRL0_KHZCLKEN		0x00800000UL
#define DW1000_PMSC_CTRL0_RXRESET		0x10000000UL
#define DW1000_PMSC_CTRL0_SOFTRESET_MASK	0xf0000000UL
#define DW1000_PMSC_LEDC		0x28
#define DW1000_PMSC_LEDC_BLINK_TIM(n)		((n) << 0)
#define DW1000_PMSC_LEDC_BLINK_TIM_DEFAULT	DW1000_PMSC_LEDC_BLINK_TIM(0x01)
#define DW1000_PMSC_LEDC_BLINK_TIM_MASK		DW1000_PMSC_LEDC_BLINK_TIM(0xff)
#define DW1000_PMSC_LEDC_BLNKEN			0x00000100UL

/* OTP layout */
#define DW1000_OTP_EUI64		0x000
#define DW1000_OTP_LDOTUNE		0x004
#define DW1000_OTP_VMEAS		0x008
#define DW1000_OTP_VMEAS_3V3(val)		(((val) >> 0) & 0xff)
#define DW1000_OTP_TMEAS		0x009
#define DW1000_OTP_TMEAS_23C(val)		(((val) >> 0) & 0xff)
#define DW1000_OTP_DELAYS		0x01c
#define DW1000_OTP_DELAYS_16M(val)		(((val) >> 0) & 0xffff)
#define DW1000_OTP_DELAYS_64M(val)		(((val) >> 16) & 0xffff)
#define DW1000_OTP_REV_XTALT		0x01e
#define DW1000_OTP_XTALT(val)			(((val) >> 0) & 0x1f)

/* Time required for OTP read to complete */
#define DW1000_OTP_WAIT_MIN_US 150
#define DW1000_OTP_WAIT_MAX_US 500

/* Time required for LDE microcode load to complete */
#define DW1000_LDELOAD_WAIT_MIN_US 150
#define DW1000_LDELOAD_WAIT_MAX_US 500

/* Delay required for SAR ADC read */
#define DW1000_SAR_WAIT_US 10

/* Cycle counter parameters
 *
 * The DW1000 uses a 40-bit counter running at a nominal 63.8976GHz
 * (128x the 499.2MHz UWB base frequency).  The counter is actually
 * incremented in units of 512 at 124.8MHz; higher resolution on the
 * receive timestamps is achieved within the chip via a Leading Edge
 * Detection (LDE) algorithm run for each received packet.
 * (Transmitted packets are synchronised to the 124.8MHz clock
 * anyway.)
 *
 * There is no provision in hardware for varying the reported
 * frequency (e.g. via a fractional tick count scaling register as
 * with most other PTP-supporting network adapters).  We therefore use
 * the cycle counter multiplier for frequency tuning.
 */
#define DW1000_CYCLECOUNTER_SIZE  40
#define DW1000_CYCLECOUNTER_MULT  288692283805801088ULL

/* Maximum multiplier adjustment
 *
 * This represents the maximum change that can be made to the cycle
 * counter multiplier without exceeding the 32-bit range, expressed
 * as parts per million (<<16) of the original multiplier value.
 * Some margin is provided to allow for the various conversion
 * factors used in calculations.
 *
 * This allows adjusting the frequency by +/- 32767ppm
 */
#define DW1000_PTP_MAX_ADJ 2147483646ULL

/* Cycle counter wraparound check interval: well within 17.2074 seconds */
#define DW1000_PTP_WORK_DELAY (3 * HZ)

/* Energy detection calculation constants */
#define DW1000_EDV2_MIN 40
#define DW1000_EDG1_MULT 3400
#define DW1000_EDG1_SHIFT 10

/* SAR ADC conversions */
#define DW1000_SAR_VBAT_MVOLT(sar, sar_3v3) \
	(3300 + ((1000 * ((sar) - (sar_3v3))) / 173))
#define DW1000_SAR_TEMP_MDEGC(sar, sar_23c) \
	(23000 + ((100000 * ((sar) - (sar_23c)))) / 114)

/* RX Channel Impulse Power scaling factor */
#define DW1000_RX_CIR_PWR_SCALE 17

/* Maximum possible link quality indicator value */
#define DW1000_LQI_MAX 0xff

/* Maximum possible value for F/P */
#define DW1000_FPR_MAX 0xff

/* Link quality indicator thresholds
 *
 * The default values DO NOT put any real limitations on the
 * timestamp reception. These thresholds should be changed
 * to more meaningful values via sysfs, case by case basis.
 */
#define DW1000_SNR_THRESHOLD_DEFAULT	1
#define DW1000_FPR_THRESHOLD_DEFAULT	1
#define DW1000_NOISE_THRESHOLD_DEFAULT	256

/* Timestamp repetition threshold
 *
 * Timestamps within this number of cycles will be considered to be
 * erroneous repetitions of earlier timestamps.
 */
#define DW1000_TIMESTAMP_REPETITION_THRESHOLD 0xffff

/* Receive preamble accumulation threshold
 *
 * The receive timestamp will be considered unreliable if fewer than
 * this fraction of the expected number of preamble symbols are
 * accumulated.
 */
#define DW1000_RXPACC_THRESHOLD 8

/* Pulse repetition frequencies */
enum dw1000_prf {
	DW1000_PRF_16M = 0x1,
	DW1000_PRF_64M = 0x2,
	DW1000_PRF_COUNT
};

/* Data rates */
enum dw1000_rate {
	DW1000_RATE_110K = 0x0,
	DW1000_RATE_850K = 0x1,
	DW1000_RATE_6800K = 0x2,
	DW1000_RATE_COUNT
};

/* Transmit preamble symbol repetitions */
enum dw1000_txpsr {
	DW1000_TXPSR_DEFAULT = 0x0,
	DW1000_TXPSR_64 = 0x1,
	DW1000_TXPSR_128 = 0x5,
	DW1000_TXPSR_256 = 0x9,
	DW1000_TXPSR_512 = 0xd,
	DW1000_TXPSR_1024 = 0x2,
	DW1000_TXPSR_2048 = 0xa,
	DW1000_TXPSR_4096 = 0x3,
};

/* Channel configuration */
struct dw1000_channel_config {
	/* Analog transmit control register values */
	uint8_t rf_txctrl[3];
	/* Analog receive control register values */
	uint8_t rf_rxctrlh;
	/* Transmitter calibration pulse generator delay register value */
	uint8_t tc_pgdelay;
	/* Frequency synthesiser PLL configuration register value */
	uint8_t fs_pllcfg[4];
	/* Frequency synthesiser PLL tuning register value */
	uint8_t fs_plltune;
	/* Transmit power control register values */
	uint8_t tx_power[DW1000_PRF_COUNT][4];
	/* Supported preamble codes */
	unsigned long pcodes[DW1000_PRF_COUNT];
};

/* Preamble code configuration */
struct dw1000_pcode_config {
	/* Leading edge detection replication coefficient register value */
	uint16_t lde_repc;
};

/* Pulse repetition frequency configuration */
struct dw1000_prf_config {
	/* Automatic gain control tuning register 1 value */
	uint8_t agc_tune1[2];
	/* Digital receiver tuning register 1A value */
	uint16_t drx_tune1a;
	/* Leading edge detection configuration register 2 value */
	uint8_t lde_cfg2[2];
};

/* Data rate configuration */
struct dw1000_rate_config {
	/* Symbol duration in nanoseconds */
	unsigned int tdsym_ns;
	/* Default transmit preamble symbol repetitions value */
	unsigned int txpsr;
	/* Digital receiver tuning register 0B value */
	uint16_t drx_tune0b;
	/* Digital receiver tuning register 1B value */
	uint16_t drx_tune1b;
	/* Digital receiver tuning register 2 value */
	uint8_t drx_tune2[DW1000_PRF_COUNT][4];
};

/* Preamble symbol repetitions configuration */
struct dw1000_txpsr_config {
	/* Digital receiver tuning register 4H value */
	uint16_t drx_tune4h;
};

/* Fixed configuration */
struct dw1000_fixed_config {
	/* Automatic gain control tuning register 2 value */
	uint8_t agc_tune2[4];
	/* Automatic gain control tuning register 3 value */
	uint8_t agc_tune3[2];
};

/* Register map parameters */
#define DW1000_REG_BITS 16
#define DW1000_VAL_BITS 8

/* Register map descriptor */
struct dw1000_regmap_config {
	/* Offset within DW1000 device structure */
	size_t offset;
	/* Register file */
	unsigned int file;
	/* Register offset shift */
	unsigned int shift;
	/* Register map configuration */
	struct regmap_config config;
};

/* Register map */
struct dw1000_regmap {
	/* Register map */
	struct regmap *regs;
	/* Containing DW1000 device */
	struct dw1000 *dw;
	/* Register map configuration */
	const struct dw1000_regmap_config *config;
};

/* Timestamp info */
struct dw1000_tsinfo {
	/* Raw hardware timestamp */
	cycle_t rawts;
	/* Link Quality Indicator */
	uint16_t lqi;
	/* Signal-to-Noise estimate */
	uint16_t snr;
	/* First Path Quality estimate */
	uint16_t fpr;
	/* Standard Deviation of Noise */
	uint16_t noise;
	/* Preamble Accumulation Count */
	uint16_t rxpacc;
	/* First Path Index */
	uint16_t fp_index;
	/* First Path Amplitude #1 */
	uint16_t fp_ampl1;
	/* First Path Amplitude #2 */
	uint16_t fp_ampl2;
	/* First Path Amplitude #3 */
	uint16_t fp_ampl3;
	/* Channel Impulse Response Power */
	uint32_t cir_pwr;
	/* First Patch Impulse Power */
	uint32_t fp_pwr;
	/* Time tracking offset */
	uint32_t ttcko;
	/* Time tracking interval */
	uint32_t ttcki;
};

/* Transmit descriptor */
struct dw1000_tx {
	/* Socket buffer */
	struct sk_buff *skb;

	/* Length */
	uint8_t len;
	/* Transmit start check */
	uint8_t check;
	/* Timestamp */
	union dw1000_timestamp time;
	/* Timestamp info */
	struct dw1000_tsinfo tsinfo;

	/* Data SPI message */
	struct spi_message data;
	/* Data buffer SPI transfer set */
	struct dw1000_spi_transfers tx_buffer;
	/* RX disable SPI transfer set */
	struct dw1000_spi_transfers sys_ctrl_trxoff;
	/* Frame control SPI transfer set */
	struct dw1000_spi_transfers tx_fctrl;
	/* TX start SPI transfer set */
	struct dw1000_spi_transfers sys_ctrl_txstrt;
	/* TX start check SPI transfer set */
	struct dw1000_spi_transfers sys_ctrl_check;
	/* Data SPI message has completed */
	bool data_complete;
	/* Data SPI message retry count */
	unsigned int retries;

	/* Information SPI message */
	struct spi_message info;
	/* IRQ acknowledgement SPI transfer set */
	struct dw1000_spi_transfers sys_status;
	/* Timestamp SPI transfer set */
	struct dw1000_spi_transfers tx_time;
};

/* Maximum number of transmit start retries
 *
 * The hardware occasionally decides to ignore the TXSTRT instruction
 * for no discernible reason.  Work around this hardware bug by
 * retrying the instruction several times if needed.
 */
#define DW1000_TX_MAX_RETRIES 5

/* Receive descriptor */
struct dw1000_rx {
	/* Received Link Quality */
	uint8_t lqi;
	/* Data frame validity */
	bool frame_valid;
	/* Timestamp validity */
	bool timestamp_valid;

	/* RX status */
	__le32 status;
	/* Frame information */
	__le32 finfo;
	/* Timestamp */
	union dw1000_rx_time time;
	/* Timestamp info */
	struct dw1000_tsinfo tsinfo;
	/* Frame quality */
	struct dw1000_rx_fqual fqual;
	/* Time tracking */
	union dw1000_rx_ttcko ttcko;
	union dw1000_rx_ttcki ttcki;
	/* Overrun count */
	__le16 evc_ovr;

	/* Information SPI message */
	struct spi_message info;
	/* Frame information SPI transfer set */
	struct dw1000_spi_transfers rx_finfo;
	/* Timestamp SPI transfer set */
	struct dw1000_spi_transfers rx_time;
	/* Frame quality transfer set */
	struct dw1000_spi_transfers rx_fqual;
	/* Frame time tracking transfer set */
	struct dw1000_spi_transfers rx_ttcko;
	struct dw1000_spi_transfers rx_ttcki;
	/* System status transfer set */
	struct dw1000_spi_transfers sys_status;
	/* Digital diagnostics transfer set */
	struct dw1000_spi_transfers dig_diag;

	/* Data SPI message */
	struct spi_message data;
	/* Data buffer SPI transfer set */
	struct dw1000_spi_transfers rx_buffer;
	/* Host buffer toggle SPI transfer set */
	struct dw1000_spi_transfers sys_ctrl;

	/* Recovery SPI message */
	struct spi_message rcvr;
	/* Event mask transfer set */
	struct dw1000_spi_transfers rv_mask0;
	/* Status clear transfer set */
	struct dw1000_spi_transfers rv_status;
	/* Event mask transfer set */
	struct dw1000_spi_transfers rv_mask1;
	/* Buffer toggle transfer set */
	struct dw1000_spi_transfers rv_hrbpt;
	/* RX Enable transfer set */
	struct dw1000_spi_transfers rv_rxenab;
};

/* PTP clock */
struct dw1000_ptp {
	/* Time counter access mutex */
	struct mutex mutex;
	/* PTP clock */
	struct ptp_clock *clock;
	/* PTP clock information */
	struct ptp_clock_info info;
	/* High resolution time counter */
	struct hires_counter tc;
};

/* DW1000 device */
struct dw1000 {
	/* SPI device */
	struct spi_device *spi;
	/* Generic device */
	struct device *dev;
	/* IEEE 802.15.4 device */
	struct ieee802154_hw *hw;
	/* WPAN PHY */
	struct wpan_phy *phy;
	/* Reset GPIO */
	int reset_gpio;

	/* Register maps */
	struct dw1000_regmap dev_id;
	struct dw1000_regmap eui;
	struct dw1000_regmap panadr;
	struct dw1000_regmap sys_cfg;
	struct dw1000_regmap sys_time;
	struct dw1000_regmap tx_fctrl;
	struct dw1000_regmap tx_buffer;
	struct dw1000_regmap dx_time;
	struct dw1000_regmap rx_fwto;
	struct dw1000_regmap sys_ctrl;
	struct dw1000_regmap sys_mask;
	struct dw1000_regmap sys_status;
	struct dw1000_regmap rx_finfo;
	struct dw1000_regmap rx_buffer;
	struct dw1000_regmap rx_fqual;
	struct dw1000_regmap rx_ttcki;
	struct dw1000_regmap rx_ttcko;
	struct dw1000_regmap rx_time;
	struct dw1000_regmap tx_time;
	struct dw1000_regmap tx_antd;
	struct dw1000_regmap sys_state;
	struct dw1000_regmap ack_resp_t;
	struct dw1000_regmap rx_sniff;
	struct dw1000_regmap tx_power;
	struct dw1000_regmap chan_ctrl;
	struct dw1000_regmap usr_sfd;
	struct dw1000_regmap agc_ctrl;
	struct dw1000_regmap ext_sync;
	struct dw1000_regmap acc_mem;
	struct dw1000_regmap gpio_ctrl;
	struct dw1000_regmap drx_conf;
	struct dw1000_regmap rf_conf;
	struct dw1000_regmap tx_cal;
	struct dw1000_regmap fs_ctrl;
	struct dw1000_regmap aon;
	struct dw1000_regmap otp_if;
	struct dw1000_regmap lde_if;
	struct dw1000_regmap dig_diag;
	struct dw1000_regmap pmsc;

	/* One-time programmable memory */
	struct regmap *otp;

	/* Calibrated voltage measurement at 3.3V */
	uint8_t vmeas_3v3;
	/* Calibrated temperature measurement at 23 degC */
	uint8_t tmeas_23c;

	/* XTAL Trim */
	uint8_t xtalt;
	/* Channel number */
	unsigned int channel;
	/* Antenna delays */
	uint16_t antd[DW1000_PRF_COUNT];
	/* Preamble codes */
	unsigned int pcode[DW1000_PRF_COUNT];
	/* Transmit power */
	uint8_t txpwr[4];
	/* Pulse repetition frequency */
	enum dw1000_prf prf;
	/* Data rate */
	enum dw1000_rate rate;
	/* Preamble symbol repetitions */
	enum dw1000_txpsr txpsr;
	/* Smart power control enabled */
	bool smart_power;
	/* Signal to Noise Ratio indicator threshold */
	uint16_t snr_threshold;
	/* First Path Energy indicator threshold */
	uint16_t fpr_threshold;
	/* Link noise indicator threshold */
	uint16_t noise_threshold;
	/* Receive overrun state */
	unsigned int rx_overruns;
	/* Last seen RX timestamp */
	uint64_t rx_timestamp[2];
	/* Number of RXs with sync */
	uint32_t rx_sync_cnt;

	/* Transmit lock */
	spinlock_t tx_lock;
	/* Transmit descriptor */
	struct dw1000_tx tx;
	/* Receive descriptor */
	struct dw1000_rx rx;

	/* PTP clock */
	struct dw1000_ptp ptp;
	/* Clock wraparound detection worker */
	struct delayed_work ptp_work;

	/* Hardware monitor */
	struct device *hwmon;
};

/**
 * to_dw1000() - Get DW1000 device from generic device
 *
 * @dev:		Generic device
 * @return:		DW1000 device
 */
static __always_inline struct dw1000 * to_dw1000(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct ieee802154_hw *hw = spi_get_drvdata(spi);
	struct dw1000 *dw = hw->priv;

	return dw;
}

#endif /* __DW1000_H */
