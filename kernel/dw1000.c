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

/********************************************************************************
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
static int dw1000_read(struct dw1000 *dw, unsigned int file, unsigned int offset,
		       void *data, size_t len)
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
static int dw1000_write(struct dw1000 *dw, unsigned int file, unsigned int offset,
			const void *data, size_t len)
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
			      unsigned int offset, const void *data, size_t len,
			      struct dw1000_message *msg,
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

/********************************************************************************
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

/********************************************************************************
 *
 * Channel configuration
 *
 */

/* Channel definitions
 *
 * Magic register values are taken directly from the datasheet.
 */
static const struct dw1000_channel dw1000_channels[] = {
	{
		.chan = 1,
		.rf_txctrl = { 0x40, 0x5c, 0x00 },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc9,
		.fs_pllcfg = { 0x07, 0x04, 0x00, 0x09 },
		.fs_plltune = 0x1e,
	},
	{
		.chan = 2,
		.rf_txctrl = { 0xa0, 0x5c, 0x04 },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc2,
		.fs_pllcfg = { 0x08, 0x05, 0x40, 0x08 },
		.fs_plltune = 0x26,
	},
	{
		.chan = 3,
		.rf_txctrl = { 0xc0, 0x6c, 0x08 },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc5,
		.fs_pllcfg = { 0x09, 0x10, 0x40, 0x08 },
		.fs_plltune = 0x56,
	},
	{
		.chan = 4,
		.rf_txctrl = { 0x80, 0x5c, 0x04 },
		.rf_rxctrlh = 0xbc,
		.tc_pgdelay = 0x95,
		.fs_pllcfg = { 0x08, 0x05, 0x40, 0x08 },
		.fs_plltune = 0x26,
	},
	{
		.chan = 5,
		.rf_txctrl = { 0xe0, 0x3f, 0x1e },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc0,
		.fs_pllcfg = { 0x1d, 0x04, 0x00, 0x08 },
		.fs_plltune = 0xbe,
	},
	{
		.chan = 7,
		.rf_txctrl = { 0xe0, 0x7d, 0x1e },
		.rf_rxctrlh = 0xbc,
		.tc_pgdelay = 0x93,
		.fs_pllcfg = { 0x1d, 0x04, 0x00, 0x08 },
		.fs_plltune = 0xbe,
	},
};

/**
 * dw1000_channel() - Identify channel configuration
 *
 * @chan:		Channel number
 * @return:		Channel configuration, or NULL if not found
 */
static const struct dw1000_channel * dw1000_channel(unsigned int chan)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dw1000_channels); i++) {
		if (dw1000_channels[i].chan == chan)
			return &dw1000_channels[i];
	}

	return NULL;
}

/********************************************************************************
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
 * @chan:		Channel number
 * @return:		0 on success or -errno
 */
static int dw1000_set_channel(struct ieee802154_hw *hw, u8 page, u8 chan)
{
	struct dw1000 *dw = hw->priv;
	const struct dw1000_channel *channel;
	int rc;

	/* Sanity check */
	if (page != DW1000_CHANNEL_PAGE)
		return -ENOTSUPP;

	/* Identify channel configuration */
	channel = dw1000_channel(chan);
	if (!channel)
		return -ENOTSUPP;
	dw->channel = channel;

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

	dev_dbg(dw->dev, "set channel %d\n", chan);
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
	int rc;

	/* Set PAN ID */
	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		if ((rc = regmap_write(dw->panadr.regs, DW1000_PANADR_PAN_ID,
				       le16_to_cpu(filt->pan_id))) != 0)
			return rc;
		dev_dbg(dw->dev, "set PAN ID %04x\n", le16_to_cpu(filt->pan_id));
	}

	/* Set short address */
	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		if ((rc = regmap_write(dw->panadr.regs, DW1000_PANADR_SHORT_ADDR,
				       le16_to_cpu(filt->short_addr))) != 0)
			return rc;
		dev_dbg(dw->dev, "set short address %04x\n",
			le16_to_cpu(filt->short_addr));
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

static int dw1000_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "setting TX power %d\n", mbm);
	return 0;
}

static int dw1000_set_lbt(struct ieee802154_hw *hw, bool on)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "setting listen before transmit %s\n",
		 (on ? "on" : "off"));
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

static int dw1000_set_csma_params(struct ieee802154_hw *hw, u8 min_be, u8 max_be,
				  u8 retries)
{
	struct dw1000 *dw = hw->priv;

	dev_info(dw->dev, "setting CSMA BE %d-%d retries %d\n",
		 min_be, max_be, retries);
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
	.set_txpower = dw1000_set_txpower,
	.set_lbt = dw1000_set_lbt,
	.set_cca_mode = dw1000_set_cca_mode,
	.set_cca_ed_level = dw1000_set_cca_ed_level,
	.set_csma_params = dw1000_set_csma_params,
	.set_frame_retries = dw1000_set_frame_retries,
	.set_promiscuous_mode = dw1000_set_promiscuous_mode,
};

/********************************************************************************
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

	/* Force slow clock speed */
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

	/* Switch to full clock speed */
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
	hw->parent = &spi->dev;

	/* Report capabilities */
	hw->flags = (IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_LBT |
		     IEEE802154_HW_CSMA_PARAMS | IEEE802154_HW_FRAME_RETRIES |
		     IEEE802154_HW_AFILT | IEEE802154_HW_PROMISCUOUS |
		     IEEE802154_HW_RX_OMIT_CKSUM |
		     IEEE802154_HW_RX_DROP_BAD_CKSUM);
	hw->phy->supported.channels[DW1000_CHANNEL_PAGE] = DW1000_CHANNELS;
	hw->phy->current_page = DW1000_CHANNEL_PAGE;
	hw->phy->current_channel = DW1000_DEFAULT_CHANNEL;

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

	/* Configure for default channel */
	if ((rc = dw1000_set_channel(hw, hw->phy->current_page,
				     hw->phy->current_channel)) != 0)
		goto err_set_channel;

	/* Register IEEE 802.15.4 device */
	if ((rc = ieee802154_register_hw(hw)) != 0) {
		dev_err(dw->dev, "could not register: %d\n", rc);
		goto err_register_hw;
	}

	spi_set_drvdata(spi, hw);
	return 0;

	ieee802154_unregister_hw(hw);
 err_register_hw:
 err_set_channel:
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

	ieee802154_unregister_hw(hw);
	ieee802154_free_hw(hw);
	return 0;
}

/********************************************************************************
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Brown <mbrown@fensystems.co.uk>");
MODULE_DESCRIPTION("DecaWave DW1000 IEEE 802.15.4 driver");
