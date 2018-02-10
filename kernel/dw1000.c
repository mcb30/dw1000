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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <net/mac802154.h>
#include "dw1000.h"

static bool dw1000_default_smart_power = true;

/******************************************************************************
 *
 * Register access
 *
 */

/* SPI message */
struct dw1000_message {
	struct spi_message msg;
	struct spi_transfer xfer[2];
	struct dw1000_spi_header header;
};

/**
 * dw1000_message_init() - Initialise SPI message
 *
 * @msg:		SPI message to initialise
 * @file:		Register file
 * @offset:		Offset within register file
 * @return:		SPI transfer to use for message payload
 */
static struct spi_transfer * dw1000_message_init(struct dw1000_message *msg,
						 unsigned int file,
						 unsigned int offset)
{
	struct spi_transfer *xfer;

	/* Construct header */
	spi_message_init(&msg->msg);
	xfer = &msg->xfer[0];
	xfer->tx_buf = &msg->header;
	msg->header.command = file;
	if (likely(offset == 0)) {
		xfer->len = offsetofend(typeof(msg->header), command);
	} else if (offset < DW1000_EXTENDED) {
		msg->header.command |= DW1000_OFFSET;
		msg->header.low = offset;
		xfer->len = offsetofend(typeof(msg->header), low);
	} else {
		msg->header.command |= DW1000_OFFSET;
		msg->header.low = (offset | DW1000_EXTENDED);
		msg->header.high = (offset >> DW1000_EXTENDED_SHIFT);
		xfer->len = offsetofend(typeof(msg->header), high);
	}
	spi_message_add_tail(xfer++, &msg->msg);

	return xfer;
}

/**
 * dw1000_read() - Read register(s)
 *
 * @dw:			DW1000 device
 * @file:		Register file
 * @offset:		Offset within register file
 * @data:		Raw register value
 * @len:		Length of register value
 * @return:		0 on success or -errno
 */
static int dw1000_read(struct dw1000 *dw, unsigned int file,
		       unsigned int offset, void *data, size_t len)
{
	struct dw1000_message msg;
	struct spi_transfer *xfer;

	/* Construct message */
	memset(&msg, 0, sizeof(msg));
	xfer = dw1000_message_init(&msg, file, offset);
	xfer->rx_buf = data;
	xfer->len = len;
	spi_message_add_tail(xfer, &msg.msg);

	return spi_sync(dw->spi, &msg.msg);
}

/**
 * dw1000_write() - Write register(s)
 *
 * @dw:			DW1000 device
 * @file:		Register file
 * @offset:		Offset within register file
 * @data:		Raw register value
 * @len:		Length of register value
 * @return:		0 on success or -errno
 */
static int dw1000_write(struct dw1000 *dw, unsigned int file,
			unsigned int offset, const void *data, size_t len)
{
	struct dw1000_message msg;
	struct spi_transfer *xfer;

	/* Construct message */
	memset(&msg, 0, sizeof(msg));
	xfer = dw1000_message_init(&msg, file, offset);
	msg.header.command |= DW1000_WRITE;
	xfer->tx_buf = data;
	xfer->len = len;
	spi_message_add_tail(xfer, &msg.msg);

	return spi_sync(dw->spi, &msg.msg);
}

/**
 * dw1000_write_async() - Write register(s) asynchronously
 *
 * @dw:			DW1000 device
 * @file:		Register file
 * @offset:		Offset within register file
 * @data:		Raw register value
 * @len:		Length of register value
 * @msg:		SPI message for asynchronous transfer
 * @complete:		Completion callback
 * @return:		0 on success or -errno
 */
static int dw1000_write_async(struct dw1000 *dw, unsigned int file,
			      unsigned int offset, const void *data,
			      size_t len, struct dw1000_message *msg,
			      void (*complete)(struct dw1000 *dw))
{
	struct spi_transfer *xfer;

	/* Construct message */
	memset(msg, 0, sizeof(*msg));
	xfer = dw1000_message_init(msg, file, offset);
	msg->header.command |= DW1000_WRITE;
	xfer->tx_buf = data;
	xfer->len = len;
	spi_message_add_tail(xfer, &msg->msg);

	/* Prepare completion */
	msg->msg.complete = (void (*)(void *))complete;
	msg->msg.context = dw;

	return spi_async(dw->spi, &msg->msg);
}

/******************************************************************************
 *
 * Register map abstraction
 *
 */

/**
 * dw1000_regmap_write() - Write register(s)
 *
 * @context:		DW1000 register map
 * @data:		Buffer containing register offset and raw register value
 * @len:		Length of buffer
 * @return:		0 on success or -errno
 */
static int dw1000_regmap_write(void *context, const void *data, size_t len)
{
	struct dw1000_regmap *map = context;
	const uint16_t *offset;

	/* Extract offset */
	if (len < sizeof(*offset))
		return -EINVAL;
	offset = data;
	data += sizeof(*offset);
	len -= sizeof(*offset);

	return dw1000_write(map->dw, map->config->file, *offset, data, len);
}

/**
 * dw1000_regmap_read() - Read register(s)
 *
 * @context:		DW1000 register map
 * @reg:		Buffer containing register offset
 * @reg_len:		Length of register offset
 * @val:		Buffer to contain raw register value
 * @val_len:		Length of raw register value
 * @return:		0 on success or -errno
 */
static int dw1000_regmap_read(void *context, const void *reg, size_t reg_len,
			      void *val, size_t val_len)
{
	struct dw1000_regmap *map = context;
	const uint16_t *offset;

	/* Extract offset */
	if (reg_len != sizeof(*offset))
		return -EINVAL;
	offset = reg;

	return dw1000_read(map->dw, map->config->file, *offset, val, val_len);
}

/* Register map bus */
static const struct regmap_bus dw1000_regmap_bus = {
	.write = dw1000_regmap_write,
	.read = dw1000_regmap_read,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
};

/* Register map configurations */
#define DW1000_REGMAP(_name, _file, _len, _type, ...)			\
	static const struct dw1000_regmap_config dw1000_ ## _name = {	\
		.offset = offsetof(struct dw1000, _name),		\
		.file =	_file,						\
		.shift = ilog2(sizeof(_type)),				\
		.config = {						\
			.name = #_name,					\
			.reg_bits = DW1000_REG_BITS,			\
			.reg_stride = sizeof(_type),			\
			.val_bits = (8 * sizeof(_type)),		\
			.max_register = ((_len) - sizeof(_type)),	\
			__VA_ARGS__					\
		},							\
	}
DW1000_REGMAP(dev_id, DW1000_DEV_ID, DW1000_DEV_ID_LEN, uint32_t );
DW1000_REGMAP(eui, DW1000_EUI, DW1000_EUI_LEN, uint8_t );
DW1000_REGMAP(panadr, DW1000_PANADR, DW1000_PANADR_LEN, uint16_t );
DW1000_REGMAP(sys_cfg, DW1000_SYS_CFG, DW1000_SYS_CFG_LEN, uint32_t );
DW1000_REGMAP(sys_time, DW1000_SYS_TIME, DW1000_SYS_TIME_LEN, uint8_t );
DW1000_REGMAP(tx_fctrl, DW1000_TX_FCTRL, DW1000_TX_FCTRL_LEN, uint8_t );
DW1000_REGMAP(tx_buffer, DW1000_TX_BUFFER, DW1000_TX_BUFFER_LEN, uint8_t );
DW1000_REGMAP(dx_time, DW1000_DX_TIME, DW1000_DX_TIME_LEN, uint8_t );
DW1000_REGMAP(rx_fwto, DW1000_RX_FWTO, DW1000_RX_FWTO_LEN, uint16_t );
DW1000_REGMAP(sys_ctrl, DW1000_SYS_CTRL, DW1000_SYS_CTRL_LEN, uint32_t );
DW1000_REGMAP(sys_mask, DW1000_SYS_MASK, DW1000_SYS_MASK_LEN, uint8_t );
DW1000_REGMAP(sys_status, DW1000_SYS_STATUS, DW1000_SYS_STATUS_LEN, uint8_t );
DW1000_REGMAP(rx_finfo, DW1000_RX_FINFO, DW1000_RX_FINFO_LEN, uint32_t );
DW1000_REGMAP(rx_buffer, DW1000_RX_BUFFER, DW1000_RX_BUFFER_LEN, uint8_t );
DW1000_REGMAP(rx_fqual, DW1000_RX_FQUAL, DW1000_RX_FQUAL_LEN, uint16_t );
DW1000_REGMAP(rx_ttcki, DW1000_RX_TTCKI, DW1000_RX_TTCKI_LEN, uint32_t );
DW1000_REGMAP(rx_ttcko, DW1000_RX_TTCKO, DW1000_RX_TTCKO_LEN, uint8_t );
DW1000_REGMAP(rx_time, DW1000_RX_TIME, DW1000_RX_TIME_LEN, uint8_t );
DW1000_REGMAP(tx_time, DW1000_TX_TIME, DW1000_TX_TIME_LEN, uint8_t );
DW1000_REGMAP(tx_antd, DW1000_TX_ANTD, DW1000_TX_ANTD_LEN, uint16_t );
DW1000_REGMAP(ack_resp_t, DW1000_ACK_RESP_T, DW1000_ACK_RESP_T_LEN, uint32_t );
DW1000_REGMAP(rx_sniff, DW1000_RX_SNIFF, DW1000_RX_SNIFF_LEN, uint32_t );
DW1000_REGMAP(tx_power, DW1000_TX_POWER, DW1000_TX_POWER_LEN, uint8_t );
DW1000_REGMAP(chan_ctrl, DW1000_CHAN_CTRL, DW1000_CHAN_CTRL_LEN, uint32_t );
DW1000_REGMAP(usr_sfd, DW1000_USR_SFD, DW1000_USR_SFD_LEN, uint8_t );
DW1000_REGMAP(agc_ctrl, DW1000_AGC_CTRL, DW1000_AGC_CTRL_LEN, uint8_t );
DW1000_REGMAP(ext_sync, DW1000_EXT_SYNC, DW1000_EXT_SYNC_LEN, uint32_t );
DW1000_REGMAP(acc_mem, DW1000_ACC_MEM, DW1000_ACC_MEM_LEN, uint32_t );
DW1000_REGMAP(gpio_ctrl, DW1000_GPIO_CTRL, DW1000_GPIO_CTRL_LEN, uint32_t );
DW1000_REGMAP(drx_conf, DW1000_DRX_CONF, DW1000_DRX_CONF_LEN, uint16_t );
DW1000_REGMAP(rf_conf, DW1000_RF_CONF, DW1000_RF_CONF_LEN, uint8_t );
DW1000_REGMAP(tx_cal, DW1000_TX_CAL, DW1000_TX_CAL_LEN, uint8_t );
DW1000_REGMAP(fs_ctrl, DW1000_FS_CTRL, DW1000_FS_CTRL_LEN, uint8_t );
DW1000_REGMAP(aon, DW1000_AON, DW1000_AON_LEN, uint8_t );
DW1000_REGMAP(otp_if, DW1000_OTP_IF, DW1000_OTP_IF_LEN, uint16_t );
DW1000_REGMAP(lde_ctrl, DW1000_LDE_CTRL, DW1000_LDE_CTRL_LEN, uint16_t );
DW1000_REGMAP(dig_diag, DW1000_DIG_DIAG, DW1000_DIG_DIAG_LEN, uint16_t );
DW1000_REGMAP(pmsc, DW1000_PMSC, DW1000_PMSC_LEN, uint32_t );

static const struct dw1000_regmap_config *dw1000_configs[] = {
	&dw1000_dev_id,
	&dw1000_eui,
	&dw1000_panadr,
	&dw1000_sys_cfg,
	&dw1000_sys_time,
	&dw1000_tx_fctrl,
	&dw1000_tx_buffer,
	&dw1000_dx_time,
	&dw1000_rx_fwto,
	&dw1000_sys_ctrl,
	&dw1000_sys_mask,
	&dw1000_sys_status,
	&dw1000_rx_finfo,
	&dw1000_rx_buffer,
	&dw1000_rx_fqual,
	&dw1000_rx_ttcki,
	&dw1000_rx_ttcko,
	&dw1000_rx_time,
	&dw1000_tx_time,
	&dw1000_tx_antd,
	&dw1000_ack_resp_t,
	&dw1000_rx_sniff,
	&dw1000_tx_power,
	&dw1000_chan_ctrl,
	&dw1000_usr_sfd,
	&dw1000_agc_ctrl,
	&dw1000_ext_sync,
	&dw1000_acc_mem,
	&dw1000_gpio_ctrl,
	&dw1000_drx_conf,
	&dw1000_rf_conf,
	&dw1000_tx_cal,
	&dw1000_fs_ctrl,
	&dw1000_aon,
	&dw1000_otp_if,
	&dw1000_lde_ctrl,
	&dw1000_dig_diag,
	&dw1000_pmsc,
};

/**
 * dw1000_regmap_init() - Initialise register maps
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_regmap_init(struct dw1000 *dw)
{
	const struct dw1000_regmap_config *config;
	struct dw1000_regmap *map;
	unsigned int i;
	int rc;

	/* Initialise register maps */
	for (i = 0; i < ARRAY_SIZE(dw1000_configs); i++) {
		config = dw1000_configs[i];
		map = (void *)dw + config->offset;
		map->dw = dw;
		map->config = config;
		map->regs = devm_regmap_init(dw->dev, &dw1000_regmap_bus, map,
					     &config->config);
		if (IS_ERR(map->regs)) {
			rc = PTR_ERR(map->regs);
			dev_err(dw->dev, "could not allocate %s map: %d\n",
				config->config.name, rc);
			return rc;
		}
	}

	return 0;
}

/******************************************************************************
 *
 * Radio configuration
 *
 */

/* Pulse repetition frequencies */
static const unsigned int dw1000_prf[DW1000_PRF_COUNT] = {
	[DW1000_PRF_SLOW] = DW1000_PRF_SLOW_MHZ,
	[DW1000_PRF_FAST] = DW1000_PRF_FAST_MHZ,
};

/* Channel definitions
 *
 * Magic register values are taken directly from the datasheet.
 */
static const struct dw1000_channel dw1000_channels[] = {
	[1] = {
		.rf_txctrl = { 0x40, 0x5c, 0x00 },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc9,
		.fs_pllcfg = { 0x07, 0x04, 0x00, 0x09 },
		.fs_plltune = 0x1e,
		.tx_power = {
			[DW1000_PRF_SLOW] = { 0x75, 0x55, 0x35, 0x15 },
			[DW1000_PRF_FAST] = { 0x67, 0x47, 0x27, 0x07 },
		},
		.preambles = {
			[DW1000_PRF_SLOW] = (BIT(1) | BIT(2)),
			[DW1000_PRF_FAST] = (BIT(9) | BIT(10) | BIT(11) |
					     BIT(12)),
		},
	},
	[2] = {
		.rf_txctrl = { 0xa0, 0x5c, 0x04 },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc2,
		.fs_pllcfg = { 0x08, 0x05, 0x40, 0x08 },
		.fs_plltune = 0x26,
		.tx_power = {
			[DW1000_PRF_SLOW] = { 0x75, 0x55, 0x35, 0x15 },
			[DW1000_PRF_FAST] = { 0x67, 0x47, 0x27, 0x07 },
		},
		.preambles = {
			[DW1000_PRF_SLOW] = (BIT(3) | BIT(4)),
			[DW1000_PRF_FAST] = (BIT(9) | BIT(10) | BIT(11) |
					     BIT(12)),
		},
	},
	[3] = {
		.rf_txctrl = { 0xc0, 0x6c, 0x08 },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc5,
		.fs_pllcfg = { 0x09, 0x10, 0x40, 0x08 },
		.fs_plltune = 0x56,
		.tx_power = {
			[DW1000_PRF_SLOW] = { 0x6f, 0x4f, 0x2f, 0x0f },
			[DW1000_PRF_FAST] = { 0x8b, 0x6b, 0x4b, 0x2b },
		},
		.preambles = {
			[DW1000_PRF_SLOW] = (BIT(5) | BIT(6)),
			[DW1000_PRF_FAST] = (BIT(9) | BIT(10) | BIT(11) |
					     BIT(12)),
		},
	},
	[4] = {
		.rf_txctrl = { 0x80, 0x5c, 0x04 },
		.rf_rxctrlh = 0xbc,
		.tc_pgdelay = 0x95,
		.fs_pllcfg = { 0x08, 0x05, 0x40, 0x08 },
		.fs_plltune = 0x26,
		.tx_power = {
			[DW1000_PRF_SLOW] = { 0x5f, 0x3f, 0x1f, 0x1f },
			[DW1000_PRF_FAST] = { 0x9a, 0x7a, 0x5a, 0x3a },
		},
		.preambles = {
			[DW1000_PRF_SLOW] = (BIT(7) | BIT(8)),
			[DW1000_PRF_FAST] = (BIT(17) | BIT(18) | BIT(19) |
					     BIT(20)),
		},
	},
	[5] = {
		.rf_txctrl = { 0xe0, 0x3f, 0x1e },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc0,
		.fs_pllcfg = { 0x1d, 0x04, 0x00, 0x08 },
		.fs_plltune = 0xbe,
		.tx_power = {
			[DW1000_PRF_SLOW] = { 0x48, 0x28, 0x08, 0x0e },
			[DW1000_PRF_FAST] = { 0x85, 0x65, 0x45, 0x25 },
		},
		.preambles = {
			[DW1000_PRF_SLOW] = (BIT(5) | BIT(6)),
			[DW1000_PRF_FAST] = (BIT(9) | BIT(10) | BIT(11) |
					     BIT(12)),
		},
	},
	[7] = {
		.rf_txctrl = { 0xe0, 0x7d, 0x1e },
		.rf_rxctrlh = 0xbc,
		.tc_pgdelay = 0x93,
		.fs_pllcfg = { 0x1d, 0x04, 0x00, 0x08 },
		.fs_plltune = 0xbe,
		.tx_power = {
			[DW1000_PRF_SLOW] = { 0x92, 0x72, 0x52, 0x32 },
			[DW1000_PRF_FAST] = { 0xd1, 0xb1, 0x71, 0x51 },
		},
		.preambles = {
			[DW1000_PRF_SLOW] = (BIT(7) | BIT(8)),
			[DW1000_PRF_FAST] = (BIT(17) | BIT(18) | BIT(19) |
					     BIT(20)),
		},
	},
};

/**
 * dw1000_preamble() - Calculate preamble code
 *
 * @dw:			DW1000 device
 * @return:		Preamble code
 */
static unsigned int dw1000_preamble(struct dw1000 *dw)
{
	unsigned long supported;

	/* Use explicitly set preamble code if configured and supported */
	supported = dw1000_channels[dw->channel].preambles[dw->prf];
	if (BIT(dw->preamble) & supported)
		return dw->preamble;

	/* Otherwise, use first supported preamble code */
	return (ffs(supported) - 1);
}

/**
 * dw1000_configure_power() - Configure transmit power
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_configure_power(struct dw1000 *dw)
{
	const struct dw1000_channel *channel = &dw1000_channels[dw->channel];
	uint8_t tx_power[sizeof(channel->tx_power[0])];
	uint32_t sys_cfg;
	unsigned int i;
	int rc;

	/* Construct transmit power register value */
	memcpy(tx_power, &channel->tx_power[dw->prf], sizeof(tx_power));
	if (!dw->smart_power) {
		for (i = 1; i < sizeof(tx_power); i++)
			tx_power[i] = tx_power[0];
	}

	/* Set transmit power */
	if ((rc = regmap_raw_write(dw->tx_power.regs, 0, tx_power,
				   sizeof(tx_power))) != 0)
		return rc;

	/* Enable/disable smart transmit power control */
	sys_cfg = (dw->smart_power ? 0 : DW1000_SYS_CFG_DIS_STXP);
	if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0,
				     DW1000_SYS_CFG_DIS_STXP, sys_cfg)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_configure_preamble() - Configure preamble code
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_configure_preamble(struct dw1000 *dw)
{
	unsigned int preamble;
	uint32_t chan_ctrl;
	int rc;

	/* Set preamble code */
	preamble = dw1000_preamble(dw);
	chan_ctrl = (DW1000_CHAN_CTRL_TX_PCODE(preamble) |
		     DW1000_CHAN_CTRL_RX_PCODE(preamble));
	if ((rc = regmap_update_bits(dw->chan_ctrl.regs, 0,
				     (DW1000_CHAN_CTRL_TX_PCODE_MASK |
				      DW1000_CHAN_CTRL_RX_PCODE_MASK),
				     chan_ctrl)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_configure_prf() - Configure pulse repetition frequency
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_configure_prf(struct dw1000 *dw)
{
	static const uint32_t chan_ctrl[DW1000_PRF_COUNT] = {
		[DW1000_PRF_SLOW] = DW1000_CHAN_CTRL_RXPRF_SLOW,
		[DW1000_PRF_FAST] = DW1000_CHAN_CTRL_RXPRF_FAST,
	};
	int rc;

	/* Set receive pulse repetition frequency */
	if ((rc = regmap_update_bits(dw->chan_ctrl.regs, 0,
				     DW1000_CHAN_CTRL_RXPRF_MASK,
				     chan_ctrl[dw->prf])) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_configure_channel() - Configure radio channel
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_configure_channel(struct dw1000 *dw)
{
	const struct dw1000_channel *channel = &dw1000_channels[dw->channel];
	unsigned int chan = dw->channel;
	int rc;

	/* Set magic register values */
	if ((rc = regmap_raw_write(dw->rf_conf.regs, DW1000_RF_CONF_RF_TXCTRL,
				   channel->rf_txctrl,
				   sizeof(channel->rf_txctrl))) != 0)
		return rc;
	if ((rc = regmap_write(dw->rf_conf.regs, DW1000_RF_CONF_RF_RXCTRLH,
			       channel->rf_rxctrlh)) != 0)
		return rc;
	if ((rc = regmap_write(dw->tx_cal.regs, DW1000_TX_CAL_TC_PGDELAY,
			       channel->tc_pgdelay)) != 0)
		return rc;
	if ((rc = regmap_raw_write(dw->fs_ctrl.regs, DW1000_FS_CTRL_FS_PLLCFG,
				   channel->fs_pllcfg,
				   sizeof(channel->fs_pllcfg))) != 0)
		return rc;
	if ((rc = regmap_write(dw->fs_ctrl.regs, DW1000_FS_CTRL_FS_PLLTUNE,
			       channel->fs_plltune)) != 0)
		return rc;

	/* Set channel numbers */
	if ((rc = regmap_update_bits(dw->chan_ctrl.regs, 0,
				     (DW1000_CHAN_CTRL_TX_CHAN_MASK |
				      DW1000_CHAN_CTRL_RX_CHAN_MASK),
				     (DW1000_CHAN_CTRL_TX_CHAN(chan) |
				      DW1000_CHAN_CTRL_RX_CHAN(chan)))) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_configure() - Configure all radio parameters
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_configure(struct dw1000 *dw)
{
	int rc;

	/* Configure channel */
	if ((rc = dw1000_configure_channel(dw)) != 0)
		return rc;

	/* Configure pulse repetition frequency */
	if ((rc = dw1000_configure_prf(dw)) != 0)
		return rc;

	/* Configure preamble */
	if ((rc = dw1000_configure_preamble(dw)) != 0)
		return rc;

	/* Configure transmit power */
	if ((rc = dw1000_configure_power(dw)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_change_smart_power() - Change use of smart power control
 *
 * @dw:			DW1000 device
 * @smart_power:	Smart power control enabled
 * @return:		0 on success or -errno
 */
static int dw1000_change_smart_power(struct dw1000 *dw, bool smart_power)
{
	int rc;

	/* Record smart power control */
	dw->smart_power = smart_power;

	/* Reconfigure transmit power */
	if ((rc = dw1000_configure_power(dw)) != 0)
		return rc;

	dev_dbg(dw->dev, "smart power control %sabled\n",
		(smart_power ? "en" : "dis"));
	return 0;
}

/**
 * dw1000_change_preamble() - Change preamble code
 *
 * @dw:			DW1000 device
 * @preamble:		Preamble code (or 0 for automatic selection)
 * @return:		0 on success or -errno
 */
static int dw1000_change_preamble(struct dw1000 *dw, unsigned int preamble)
{
	unsigned long supported;
	int rc;

	/* Check that preamble code is valid */
	if (preamble) {
		supported = dw1000_channels[dw->channel].preambles[dw->prf];
		if (!(BIT(preamble) & supported))
			return -EINVAL;
	}

	/* Record preamble code */
	dw->preamble = preamble;

	/* Reconfigure preamble code */
	if ((rc = dw1000_configure_preamble(dw)) != 0)
		return rc;

	dev_dbg(dw->dev, "set preamble code %d (requested %d)\n",
		dw1000_preamble(dw), preamble);
	return 0;
}

/**
 * dw1000_change_prf() - Change pulse repetition frequency
 *
 * @dw:			DW1000 device
 * @prf:		Pulse repetition frequency
 * @return:		0 on success or -errno
 */
static int dw1000_change_prf(struct dw1000 *dw, enum dw1000_prf prf)
{
	int rc;

	/* Record pulse repetition frequency */
	dw->prf = prf;

	/* Reconfigure pulse repetition frequency */
	if ((rc = dw1000_configure_prf(dw)) != 0)
		return rc;

	/* Reconfigure preamble code */
	if ((rc = dw1000_configure_preamble(dw)) != 0)
		return rc;

	/* Reconfigure transmit power */
	if ((rc = dw1000_configure_power(dw)) != 0)
		return rc;

	dev_dbg(dw->dev, "set pulse repetition frequency %dMHz\n",
		dw1000_prf[dw->prf]);
	return 0;
}

/**
 * dw1000_change_channel() - Change radio channel
 *
 * @dw:			DW1000 device
 * @channel:		Channel number
 * @return:		0 on success or -errno
 */
static int dw1000_change_channel(struct dw1000 *dw, unsigned int channel)
{
	int rc;

	/* Check that channel is supported */
	if (!(BIT(channel) & DW1000_CHANNELS))
		return -EINVAL;

	/* Record channel */
	dw->channel = channel;

	/* Reconfigure channel */
	if ((rc = dw1000_configure_channel(dw)) != 0)
		return rc;

	/* Reconfigure preamble code */
	if ((rc = dw1000_configure_preamble(dw)) != 0)
		return rc;

	/* Reconfigure transmit power */
	if ((rc = dw1000_configure_power(dw)) != 0)
		return rc;

	dev_dbg(dw->dev, "set channel %d\n", channel);
	return 0;
}

/******************************************************************************
 *
 * IEEE 802.15.4 interface
 *
 */

/**
 * dw1000_start() - Start device operation
 *
 * @hw:			IEEE 802.15.4 device
 * @return:		0 on success or -errno
 */
static int dw1000_start(struct ieee802154_hw *hw)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "started\n");
	return 0;
}

/**
 * dw1000_stop() - Stop device operation
 *
 * @hw:			IEEE 802.15.4 device
 */
static void dw1000_stop(struct ieee802154_hw *hw)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "stopped\n");
}

static int dw1000_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	return -ENOTSUPP;
}

static int dw1000_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "detecting energy\n");
	*level = 0;
	return 0;
}

/**
 * dw1000_set_channel() - Set radio channel
 *
 * @hw:			IEEE 802.15.4 device
 * @page:		Channel page (must be UWB)
 * @channel:		Channel number
 * @return:		0 on success or -errno
 */
static int dw1000_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct dw1000 *dw = hw->priv;
	int rc;

	/* Sanity checks */
	if (page != DW1000_CHANNEL_PAGE)
		return -EINVAL;

	/* Change radio channel */
	if ((rc = dw1000_change_channel(dw, channel)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_set_hw_addr_filt() - Set hardware address filters
 *
 * @hw:			IEEE 802.15.4 device
 * @filt:		Hardware address filters
 * @changed:		Change bitmask
 * @return:		0 on success or -errno
 */
static int dw1000_set_hw_addr_filt(struct ieee802154_hw *hw,
				   struct ieee802154_hw_addr_filt *filt,
				   unsigned long changed)
{
	struct dw1000 *dw = hw->priv;
	unsigned int pan_id;
	unsigned int short_addr;
	int rc;

	/* Set PAN ID */
	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		pan_id = le16_to_cpu(filt->pan_id);
		if ((rc = regmap_write(dw->panadr.regs, DW1000_PANADR_PAN_ID,
				       pan_id)) != 0)
			return rc;
		dev_dbg(dw->dev, "set PAN ID %04x\n", pan_id);
	}

	/* Set short address */
	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		short_addr = le16_to_cpu(filt->short_addr);
		if ((rc = regmap_write(dw->panadr.regs,
				       DW1000_PANADR_SHORT_ADDR,
				       short_addr)) != 0)
			return rc;
		dev_dbg(dw->dev, "set short address %04x\n", short_addr);
	}

	/* Set EUI-64 */
	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		if ((rc = regmap_raw_write(dw->eui.regs, 0, &filt->ieee_addr,
					   sizeof(filt->ieee_addr))) != 0)
			return rc;
		dev_dbg(dw->dev, "set EUI64 %016llx\n",
			le64_to_cpu(filt->ieee_addr));
	}

	/* Set coordinator status */
	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0,
					     DW1000_SYS_CFG_FFBC,
					     (filt->pan_coord ?
					      DW1000_SYS_CFG_FFBC : 0))) != 0)
			return rc;
		dev_dbg(dw->dev, "PAN coordinator role %sabled\n",
			(filt->pan_coord ? "en" : "dis"));
	}

	return 0;
}

static int dw1000_set_cca_mode(struct ieee802154_hw *hw,
			       const struct wpan_phy_cca *cca)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "setting CCA mode\n");
	return 0;
}

static int dw1000_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "setting CCA ED level %d\n", mbm);
	return 0;
}

static int dw1000_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "setting frame retries %d\n", retries);
	return 0;
}

/**
 * dw1000_set_promiscuous_mode() - Enable/disable promiscuous mode
 *
 * @hw:			IEEE 802.15.4 device
 * @on:			Enable/disable promiscuous mode
 * @return:		0 on success or -errno
 */
static int dw1000_set_promiscuous_mode(struct ieee802154_hw *hw, bool on)
{
	struct dw1000 *dw = hw->priv;
	int rc;

	/* Enable/disable frame filtering */
	if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0, DW1000_SYS_CFG_FFEN,
				     (on ? 0 : DW1000_SYS_CFG_FFEN))) != 0)
		return rc;
	dev_dbg(dw->dev, "promiscuous mode %sabled\n", (on ? "en" : "dis"));

	return 0;
}

/* IEEE 802.15.4 operations */
static const struct ieee802154_ops dw1000_ops = {
	.owner = THIS_MODULE,
	.start = dw1000_start,
	.stop = dw1000_stop,
	.xmit_async = dw1000_xmit_async,
	.ed = dw1000_ed,
	.set_channel = dw1000_set_channel,
	.set_hw_addr_filt = dw1000_set_hw_addr_filt,
	.set_cca_mode = dw1000_set_cca_mode,
	.set_cca_ed_level = dw1000_set_cca_ed_level,
	.set_frame_retries = dw1000_set_frame_retries,
	.set_promiscuous_mode = dw1000_set_promiscuous_mode,
};

/******************************************************************************
 *
 * Device attributes
 *
 */

#define DW1000_ATTR_RW(_name, _mode) \
	DEVICE_ATTR(_name, _mode, dw1000_show_##_name, dw1000_store_##_name)

/* Smart power control */
static ssize_t dw1000_show_smart_power(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->smart_power);
}
static ssize_t dw1000_store_smart_power(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct dw1000 *dw = to_dw1000(dev);
	bool smart_power;
	int rc;

	if (strtobool(buf, &smart_power) < 0)
		return -EINVAL;
	if ((rc = dw1000_change_smart_power(dw, smart_power)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(smart_power, 0644);

/* Pulse repetition frequency */
static ssize_t dw1000_show_prf(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_prf[dw->prf]);
}
static ssize_t dw1000_store_prf(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dw1000 *dw = to_dw1000(dev);
	enum dw1000_prf prf;
	int value;
	int rc;

	if ((rc = kstrtoint(buf, 0, &value)) != 0)
		return rc;
	switch (value) {
	case DW1000_PRF_SLOW_MHZ:
		prf = DW1000_PRF_SLOW;
		break;
	case DW1000_PRF_FAST_MHZ:
		prf = DW1000_PRF_FAST;
		break;
	default:
		return -EINVAL;
	}
	if ((rc = dw1000_change_prf(dw, prf)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(prf, 0644);

/* Preamble code */
static ssize_t dw1000_show_preamble(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_preamble(dw));
}
static ssize_t dw1000_store_preamble(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct dw1000 *dw = to_dw1000(dev);
	int preamble;
	int rc;

	if ((rc = kstrtoint(buf, 0, &preamble)) != 0)
		return rc;
	if ((rc = dw1000_change_preamble(dw, preamble)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(preamble, 0644);

/* Attribute list */
static struct attribute *dw1000_attrs[] = {
	&dev_attr_smart_power.attr,
	&dev_attr_prf.attr,
	&dev_attr_preamble.attr,
	NULL
};

/* Attribute group */
static struct attribute_group dw1000_attr_group = {
	.name = "dw1000",
	.attrs = dw1000_attrs,
};

/******************************************************************************
 *
 * Device initialisation
 *
 */

/**
 * dw1000_check_dev_id() - Check device ID
 *
 * The register ID tag portion of the device ID has a fixed value.
 * This can be used to verify that the SPI bus is operational and that
 * the DW1000 device is present and responding to commands.
 *
 * @dw:			DW1000 device
 * @id:			Device ID to fill in
 * @return:		0 on success or -errno
 */
static int dw1000_check_dev_id(struct dw1000 *dw, struct dw1000_dev_id *id)
{
	int rc;

	/* Read device ID */
	if ((rc = dw1000_read(dw, DW1000_DEV_ID, 0, id, sizeof(*id))) != 0) {
		dev_err(dw->dev, "could not read device ID: %d\n", rc);
		return rc;
	}

	/* Check register ID tag */
	if (id->ridtag != cpu_to_le16(DW1000_RIDTAG_MAGIC)) {
		dev_err(dw->dev, "incorrect RID tag %04X (expected %04X)\n",
			le16_to_cpu(id->ridtag), DW1000_RIDTAG_MAGIC);
		return -EINVAL;
	}

	return 0;
}

/**
 * dw1000_reset() - Reset device
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_reset(struct dw1000 *dw)
{
	struct dw1000_dev_id id;
	int rc;

	/* Force slow SPI clock speed */
	dw->spi->max_speed_hz = DW1000_SPI_SLOW_HZ;

	/* Read device ID register */
	if ((rc = dw1000_check_dev_id(dw, &id)) != 0)
		return rc;
	dev_info(dw->dev, "found %04X model %02X version %02X\n",
		 le16_to_cpu(id.ridtag), id.model, id.ver_rev);

	/* Force system clock to 19.2 MHz XTI clock */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     DW1000_PMSC_CTRL0_SYSCLKS_MASK,
				     DW1000_PMSC_CTRL0_SYSCLKS_SLOW)) != 0)
		return rc;

	/* Initiate reset */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     DW1000_PMSC_CTRL0_SOFTRESET_MASK, 0)) != 0)
		return rc;

	/* Clear reset */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     DW1000_PMSC_CTRL0_SOFTRESET_MASK,
				     DW1000_PMSC_CTRL0_SOFTRESET_MASK)) != 0)
		return rc;

	/* Recheck device ID to ensure bus is still operational */
	if ((rc = dw1000_check_dev_id(dw, &id)) != 0)
		return rc;

	/* Switch to full SPI clock speed */
	dw->spi->max_speed_hz = DW1000_SPI_FAST_HZ;

	/* Recheck device ID to ensure bus is still operational */
	if ((rc = dw1000_check_dev_id(dw, &id)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_load_lde() - Load leading edge detection microcode
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_load_lde(struct dw1000 *dw)
{
	struct dw1000_dev_id id;
	int rc;

	/* Force slow SPI clock speed */
	dw->spi->max_speed_hz = DW1000_SPI_SLOW_HZ;

	/* Force system clock to 19.2 MHz XTI clock and prepare for LDE load */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     (DW1000_PMSC_CTRL0_SYSCLKS_MASK |
				      DW1000_PMSC_CTRL0_LDE_HACK_MASK),
				     (DW1000_PMSC_CTRL0_SYSCLKS_SLOW |
				      DW1000_PMSC_CTRL0_LDE_HACK_PRE))) != 0)
		return rc;

	/* Initiate load of LDE microcode */
	if ((rc = regmap_update_bits(dw->otp_if.regs, DW1000_OTP_IF_CTRL,
				     DW1000_OTP_IF_CTRL_LDELOAD,
				     DW1000_OTP_IF_CTRL_LDELOAD)) != 0)
		return rc;

	/* Allow time for microcode load to complete */
	usleep_range(DW1000_LDELOAD_WAIT_MIN_US, DW1000_LDELOAD_WAIT_MAX_US);

	/* Restore system clock to automatic selection */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     (DW1000_PMSC_CTRL0_SYSCLKS_MASK |
				      DW1000_PMSC_CTRL0_LDE_HACK_MASK),
				     (DW1000_PMSC_CTRL0_SYSCLKS_AUTO |
				      DW1000_PMSC_CTRL0_LDE_HACK_POST))) != 0)
		return rc;

	/* Switch to full SPI clock speed */
	dw->spi->max_speed_hz = DW1000_SPI_FAST_HZ;

	    /* Recheck device ID to ensure bus is still operational */
	if ((rc = dw1000_check_dev_id(dw, &id)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_init() - Apply initial configuration
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_init(struct dw1000 *dw)
{
	uint32_t sys_cfg_filters;
	uint32_t sys_cfg_mask;
	uint32_t sys_cfg_val;
	uint32_t gpio_ctrl_mask;
	uint32_t gpio_ctrl_val;
	int rc;

	/* Set system configuration:
	 *
	 * - enable all frame types (when filtering is enabled)
	 * - use double buffering
	 * - use receiver auto-re-enabling
	 *
	 * Note that the overall use of filtering is controlled via
	 * dw1000_set_promiscuous_mode().
	 */
	sys_cfg_filters = (DW1000_SYS_CFG_FFAB | DW1000_SYS_CFG_FFAD |
			   DW1000_SYS_CFG_FFAA | DW1000_SYS_CFG_FFAM |
			   DW1000_SYS_CFG_FFAR | DW1000_SYS_CFG_FFA4 |
			   DW1000_SYS_CFG_FFA5);
	sys_cfg_mask = (sys_cfg_filters | DW1000_SYS_CFG_DIS_DRXB |
			DW1000_SYS_CFG_RXAUTR);
	sys_cfg_val = (sys_cfg_filters | DW1000_SYS_CFG_RXAUTR);
	if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0, sys_cfg_mask,
				     sys_cfg_val)) != 0)
		return rc;

	/* Enable LEDs */
	gpio_ctrl_mask = (DW1000_GPIO_CTRL_MODE_MSGP0_MASK |
			  DW1000_GPIO_CTRL_MODE_MSGP1_MASK |
			  DW1000_GPIO_CTRL_MODE_MSGP2_MASK |
			  DW1000_GPIO_CTRL_MODE_MSGP3_MASK);
	gpio_ctrl_val = (DW1000_GPIO_CTRL_MODE_MSGP0_RXOKLED |
			 DW1000_GPIO_CTRL_MODE_MSGP1_SFDLED |
			 DW1000_GPIO_CTRL_MODE_MSGP2_RXLED |
			 DW1000_GPIO_CTRL_MODE_MSGP3_TXLED);
	if ((rc = regmap_update_bits(dw->gpio_ctrl.regs, DW1000_GPIO_CTRL_MODE,
				     gpio_ctrl_mask, gpio_ctrl_val)) != 0)
		return rc;

	/* Load LDE microcode */
	if ((rc = dw1000_load_lde(dw)) != 0)
		return rc;

	/* Configure radio */
	if ((rc = dw1000_configure(dw)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_probe() - Create device
 *
 * @spi:		SPI device
 * @return:		0 on success or -errno
 */
static int dw1000_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct dw1000 *dw;
	int rc;

	/* Allocate IEEE 802.15.4 device */
	hw = ieee802154_alloc_hw(sizeof(*dw), &dw1000_ops);
	if (!hw) {
		rc = -ENOMEM;
		goto err_alloc_hw;
	}
	dw = hw->priv;
	dw->spi = spi;
	dw->dev = &spi->dev;
	dw->channel = DW1000_CHANNEL_DEFAULT;
	dw->prf = DW1000_PRF_FAST;
	dw->smart_power = dw1000_default_smart_power;
	hw->parent = &spi->dev;

	/* Report capabilities */
	hw->flags = (IEEE802154_HW_TX_OMIT_CKSUM |
		     IEEE802154_HW_FRAME_RETRIES |
		     IEEE802154_HW_AFILT |
		     IEEE802154_HW_PROMISCUOUS |
		     IEEE802154_HW_RX_OMIT_CKSUM |
		     IEEE802154_HW_RX_DROP_BAD_CKSUM);
	hw->phy->supported.channels[DW1000_CHANNEL_PAGE] = DW1000_CHANNELS;
	hw->phy->current_page = DW1000_CHANNEL_PAGE;
	hw->phy->current_channel = DW1000_CHANNEL_DEFAULT;

	/* Initialise register map */
	if ((rc = dw1000_regmap_init(dw)) != 0)
		goto err_regmap_init;

	/* Reset device */
	if ((rc = dw1000_reset(dw)) != 0) {
		dev_err(dw->dev, "reset failed: %d\n", rc);
		goto err_reset;
	}

	/* Initialise device */
	if ((rc = dw1000_init(dw)) != 0) {
		dev_err(dw->dev, "initialisation failed: %d\n", rc);
		goto err_init;
	}

	/* Add attribute group */
	if ((rc = sysfs_create_group(&dw->dev->kobj, &dw1000_attr_group)) != 0)
		goto err_create_group;

	/* Register IEEE 802.15.4 device */
	if ((rc = ieee802154_register_hw(hw)) != 0) {
		dev_err(dw->dev, "could not register: %d\n", rc);
		goto err_register_hw;
	}

	spi_set_drvdata(spi, hw);
	return 0;

	ieee802154_unregister_hw(hw);
 err_register_hw:
	sysfs_remove_group(&dw->dev->kobj, &dw1000_attr_group);
 err_create_group:
 err_init:
 err_reset:
 err_regmap_init:
	ieee802154_free_hw(hw);
 err_alloc_hw:
	return rc;
}

/**
 * dw1000_remove() - Remove device
 *
 * @spi:		SPI device
 * @return:		0 on success or -errno
 */
static int dw1000_remove(struct spi_device *spi)
{
	struct ieee802154_hw *hw = spi_get_drvdata(spi);
	struct dw1000 *dw = hw->priv;

	ieee802154_unregister_hw(hw);
	sysfs_remove_group(&dw->dev->kobj, &dw1000_attr_group);
	ieee802154_free_hw(hw);
	return 0;
}

/******************************************************************************
 *
 * Module interface
 *
 */

enum {
	DW1000,
};

static const struct of_device_id dw1000_of_ids[] = {
	{ .compatible = "decawave,dw1000", .data = (void *)DW1000 },
	{ },
};
MODULE_DEVICE_TABLE(of, dw1000_of_ids);

static const struct spi_device_id dw1000_spi_ids[] = {
	{ "dw1000", DW1000 },
	{ },
};
MODULE_DEVICE_TABLE(spi, dw1000_spi_ids);

static struct spi_driver dw1000_driver = {
	.driver = {
		.name = "dw1000",
		.of_match_table = of_match_ptr(dw1000_of_ids),
	},
	.id_table = dw1000_spi_ids,
	.probe = dw1000_probe,
	.remove = dw1000_remove,
};
module_spi_driver(dw1000_driver);

module_param_named(default_smart_power, dw1000_default_smart_power, bool, 0644);
MODULE_PARM_DESC(default_smart_power, "Default for TX Smart Power Control");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Brown <mbrown@fensystems.co.uk>");
MODULE_DESCRIPTION("DecaWave DW1000 IEEE 802.15.4 driver");
