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

/* Construct register address */
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

/* Read register(s) */
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

/* Write register(s) */
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

/* Write register(s) asynchronously */
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

/* Write register(s) */
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

/* Read register(s) */
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
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
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
DW1000_REGMAP(rf_conf, DW1000_RF_CONF, DW1000_RF_CONF_LEN, uint32_t );
DW1000_REGMAP(tx_cal, DW1000_TX_CAL, DW1000_TX_CAL_LEN, uint8_t );
DW1000_REGMAP(fs_ctrl, DW1000_FS_CTRL, DW1000_FS_CTRL_LEN, uint8_t );
DW1000_REGMAP(aon, DW1000_AON, DW1000_AON_LEN, uint8_t );
DW1000_REGMAP(otp_if, DW1000_OTP_IF, DW1000_OTP_IF_LEN, uint16_t );
DW1000_REGMAP(lde_ctrl, DW1000_LDE_CTRL, DW1000_LDE_CTRL_LEN, uint16_t );
DW1000_REGMAP(dig_diag, DW1000_DIG_DIAG, DW1000_DIG_DIAG_LEN, uint16_t );
DW1000_REGMAP(pmsc, DW1000_PMSC, DW1000_PMSC_LEN, uint16_t );

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

/* Initialise register maps */
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
		map->regs = devm_regmap_init(&dw->spi->dev, &dw1000_regmap_bus,
					     map, &config->config);
		if (IS_ERR(map->regs)) {
			rc = PTR_ERR(map->regs);
			dev_err(&dw->spi->dev, "could not allocate %s map: %d\n",
				config->config.name, rc);
			return rc;
		}
	}

	return 0;
}

/********************************************************************************
 *
 * IEEE 802.15.4 interface
 *
 */

static int dw1000_start(struct ieee802154_hw *hw)
{
	struct dw1000 *dw = hw->priv;

	dev_info(&dw->spi->dev, "started\n");
	return 0;
}

static void dw1000_stop(struct ieee802154_hw *hw)
{
	struct dw1000 *dw = hw->priv;

	dev_info(&dw->spi->dev, "stopped\n");
}

static int dw1000_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct dw1000 *dw = hw->priv;

	dev_info(&dw->spi->dev, "detecting energy\n");
	*level = 0;
	return 0;
}

static int dw1000_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct dw1000 *dw = hw->priv;

	dev_info(&dw->spi->dev, "setting channel %d:%d\n", page, channel);
	return 0;
}

static int dw1000_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	return -ENOTSUPP;
}

static const struct ieee802154_ops dw1000_ops = {
	.owner = THIS_MODULE,
	.start = dw1000_start,
	.stop = dw1000_stop,
	.xmit_async = dw1000_xmit_async,
	.ed = dw1000_ed,
	.set_channel = dw1000_set_channel,
};

/********************************************************************************
 *
 * Device initialisation
 *
 */

static int dw1000_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct dw1000 *dw;
	struct dw1000_dev_id id;
	int rc;

	/* Allocate IEEE 802.15.4 device */
	hw = ieee802154_alloc_hw(sizeof(*dw), &dw1000_ops);
	if (!hw) {
		rc = -ENOMEM;
		goto err_alloc_hw;
	}
	dw = hw->priv;
	dw->spi = spi;
	hw->parent = &spi->dev;

	/* Read device ID register */
	rc = dw1000_read(dw, DW1000_DEV_ID, 0, &id, sizeof(id));
	if (rc) {
		dev_err(&spi->dev, "could not read device ID: %d\n", rc);
		goto err_dev_id_read;
	}
	dev_info(&spi->dev, "found %04X model %02X version %02X\n",
		 le16_to_cpu(id.ridtag), id.model, id.ver_rev);
	if (id.ridtag != cpu_to_le16(DW1000_RIDTAG_MAGIC)) {
		dev_err(&spi->dev, "incorrect RID tag %04X (expected %04X)\n",
			le16_to_cpu(id.ridtag), DW1000_RIDTAG_MAGIC);
		rc = -EINVAL;
		goto err_dev_id_mismatch;
	}

	/* Initialise register map */
	rc = dw1000_regmap_init(dw);
	if (rc)
		goto err_regmap_init;

	/* Register IEEE 802.15.4 device */
	rc = ieee802154_register_hw(hw);
	if (rc) {
		dev_err(&spi->dev, "could not register: %d\n", rc);
		goto err_register_hw;
	}

	spi_set_drvdata(spi, hw);
	return 0;

	ieee802154_unregister_hw(hw);
 err_register_hw:
 err_dev_id_mismatch:
 err_dev_id_read:
 err_regmap_init:
	ieee802154_free_hw(hw);
 err_alloc_hw:
	return rc;
}

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
