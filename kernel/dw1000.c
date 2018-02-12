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
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <net/mac802154.h>
#include "dw1000.h"

/******************************************************************************
 *
 * Enumeration conversions
 *
 */

/* Pulse repetition frequency values */
static const unsigned int dw1000_prfs[] = {
	[DW1000_PRF_16M] = 16,
	[DW1000_PRF_64M] = 64,
};

/* Data rate values */
static const unsigned int dw1000_rates[] = {
	[DW1000_RATE_110K] = 110,
	[DW1000_RATE_850K] = 850,
	[DW1000_RATE_6800K] = 6800,
};

/**
 * Look up pulse repetition frequency
 *
 * @prf:		Pulse repetition frequency in MHz
 * @return:		Pulse repetition frequency enumeration or -errno
 */
static int dw1000_prf(unsigned int prf)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dw1000_prfs); i++) {
		if (dw1000_prfs[i] && (dw1000_prfs[i] == prf))
			return i;
	}
	return -EINVAL;
}

/**
 * Look up data rate
 *
 * @rate:		Data rate in kbps
 * @return:		Data rate enumeration or -errno
 */
static int dw1000_rate(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dw1000_rates); i++) {
		if (dw1000_rates[i] && (dw1000_rates[i] == rate))
			return i;
	}
	return -EINVAL;
}

/******************************************************************************
 *
 * Register access
 *
 */

/**
 * dw1000_init_transfers() - Initialise transfer set
 *
 * @msg:		SPI message
 * @xfers:		SPI transfer set
 * @file:		Register file
 * @offset:		Offset within register file
 */
static void dw1000_init_transfers(struct spi_message *msg,
				  struct dw1000_spi_transfers *xfers,
				  unsigned int file, unsigned int offset)
{
	struct spi_transfer *previous;

	/* Construct transfers */
	xfers->header.tx_buf = &xfers->command;
	xfers->command.file = file;
	if (likely(offset == 0)) {
		xfers->header.len = offsetofend(typeof(xfers->command), file);
	} else if (offset < DW1000_EXTENDED) {
		xfers->command.file |= DW1000_OFFSET;
		xfers->command.low = offset;
		xfers->header.len = offsetofend(typeof(xfers->command), low);
	} else {
		xfers->command.file |= DW1000_OFFSET;
		xfers->command.low = (offset | DW1000_EXTENDED);
		xfers->command.high = (offset >> DW1000_EXTENDED_SHIFT);
		xfers->header.len = offsetofend(typeof(xfers->command), high);
	}

	/* Toggle chip select for this transfer set if required */
	if (!list_empty(&msg->transfers)) {
		previous = list_last_entry(&msg->transfers, struct spi_transfer,
					   transfer_list);
		previous->cs_change = 1;
	}

	/* Add transfers to message */
	spi_message_add_tail(&xfers->header, msg);
	spi_message_add_tail(&xfers->data, msg);
}

/**
 * dw1000_init_read() - Initialise read transfer set
 *
 * @msg:		SPI message
 * @xfers:		SPI transfer set
 * @file:		Register file
 * @offset:		Offset within register file
 * @data:		Data buffer
 * @len:		Length of data
 */
static void dw1000_init_read(struct spi_message *msg,
			     struct dw1000_spi_transfers *xfers,
			     unsigned int file, unsigned int offset,
			     void *data, size_t len)
{
	/* Initialise transfers */
	dw1000_init_transfers(msg, xfers, file, offset);

	/* Construct data transfer */
	xfers->data.rx_buf = data;
	xfers->data.len = len;
}

/**
 * dw1000_init_write() - Initialise write transfer set
 *
 * @msg:		SPI message
 * @xfers:		SPI transfer set
 * @file:		Register file
 * @offset:		Offset within register file
 * @data:		Data buffer
 * @len:		Length of data
 */
static void dw1000_init_write(struct spi_message *msg,
			      struct dw1000_spi_transfers *xfers,
			      unsigned int file, unsigned int offset,
			      const void *data, size_t len)
{
	/* Initialise transfers */
	dw1000_init_transfers(msg, xfers, file, offset);
	xfers->command.file |= DW1000_WRITE;

	/* Construct data transfer */
	xfers->data.tx_buf = data;
	xfers->data.len = len;
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
	struct {
		struct spi_message msg;
		struct dw1000_spi_transfers xfers;
	} read;

	/* Construct message */
	memset(&read, 0, sizeof(read));
	spi_message_init_no_memset(&read.msg);
	dw1000_init_read(&read.msg, &read.xfers, file, offset, data, len);

	return spi_sync(dw->spi, &read.msg);
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
	struct {
		struct spi_message msg;
		struct dw1000_spi_transfers xfers;
	} write;

	/* Construct message */
	memset(&write, 0, sizeof(write));
	spi_message_init_no_memset(&write.msg);
	dw1000_init_write(&write.msg, &write.xfers, file, offset, data, len);

	return spi_sync(dw->spi, &write.msg);
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
DW1000_REGMAP(sys_ctrl, DW1000_SYS_CTRL, DW1000_SYS_CTRL_LEN, uint8_t );
DW1000_REGMAP(sys_mask, DW1000_SYS_MASK, DW1000_SYS_MASK_LEN, uint32_t );
DW1000_REGMAP(sys_status, DW1000_SYS_STATUS, DW1000_SYS_STATUS_LEN, uint32_t );
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
DW1000_REGMAP(lde_if, DW1000_LDE_IF, DW1000_LDE_IF_LEN, uint8_t );
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
	&dw1000_lde_if,
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
 * All magic register values are taken directly from the datasheet.
 *
 */

/* Channel configurations */
static const struct dw1000_channel_config dw1000_channel_configs[] = {
	[1] = {
		.rf_txctrl = { 0x40, 0x5c, 0x00 },
		.rf_rxctrlh = 0xd8,
		.tc_pgdelay = 0xc9,
		.fs_pllcfg = { 0x07, 0x04, 0x00, 0x09 },
		.fs_plltune = 0x1e,
		.tx_power = {
			[DW1000_PRF_16M] = { 0x75, 0x55, 0x35, 0x15 },
			[DW1000_PRF_64M] = { 0x67, 0x47, 0x27, 0x07 },
		},
		.pcodes = {
			[DW1000_PRF_16M] = (BIT(1) | BIT(2)),
			[DW1000_PRF_64M] = (BIT(9) | BIT(10) | BIT(11) |
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
			[DW1000_PRF_16M] = { 0x75, 0x55, 0x35, 0x15 },
			[DW1000_PRF_64M] = { 0x67, 0x47, 0x27, 0x07 },
		},
		.pcodes = {
			[DW1000_PRF_16M] = (BIT(3) | BIT(4)),
			[DW1000_PRF_64M] = (BIT(9) | BIT(10) | BIT(11) |
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
			[DW1000_PRF_16M] = { 0x6f, 0x4f, 0x2f, 0x0f },
			[DW1000_PRF_64M] = { 0x8b, 0x6b, 0x4b, 0x2b },
		},
		.pcodes = {
			[DW1000_PRF_16M] = (BIT(5) | BIT(6)),
			[DW1000_PRF_64M] = (BIT(9) | BIT(10) | BIT(11) |
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
			[DW1000_PRF_16M] = { 0x5f, 0x3f, 0x1f, 0x1f },
			[DW1000_PRF_64M] = { 0x9a, 0x7a, 0x5a, 0x3a },
		},
		.pcodes = {
			[DW1000_PRF_16M] = (BIT(7) | BIT(8)),
			[DW1000_PRF_64M] = (BIT(17) | BIT(18) | BIT(19) |
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
			[DW1000_PRF_16M] = { 0x48, 0x28, 0x08, 0x0e },
			[DW1000_PRF_64M] = { 0x85, 0x65, 0x45, 0x25 },
		},
		.pcodes = {
			[DW1000_PRF_16M] = (BIT(5) | BIT(6)),
			[DW1000_PRF_64M] = (BIT(9) | BIT(10) | BIT(11) |
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
			[DW1000_PRF_16M] = { 0x92, 0x72, 0x52, 0x32 },
			[DW1000_PRF_64M] = { 0xd1, 0xb1, 0x71, 0x51 },
		},
		.pcodes = {
			[DW1000_PRF_16M] = (BIT(7) | BIT(8)),
			[DW1000_PRF_64M] = (BIT(17) | BIT(18) | BIT(19) |
					    BIT(20)),
		},
	},
};

/* Preamble code configurations */
static const struct dw1000_pcode_config dw1000_pcode_configs[] = {
	[1]  = { .lde_repc = 0x5998 },
	[2]  = { .lde_repc = 0x5998 },
	[3]  = { .lde_repc = 0x51ea },
	[4]  = { .lde_repc = 0x428e },
	[5]  = { .lde_repc = 0x451e },
	[6]  = { .lde_repc = 0x2e14 },
	[7]  = { .lde_repc = 0x8000 },
	[8]  = { .lde_repc = 0x51ea },
	[9]  = { .lde_repc = 0x28f4 },
	[10] = { .lde_repc = 0x3332 },
	[11] = { .lde_repc = 0x3ae0 },
	[12] = { .lde_repc = 0x3d70 },
	[13] = { .lde_repc = 0x3ae0 },
	[14] = { .lde_repc = 0x35c2 },
	[15] = { .lde_repc = 0x2b84 },
	[16] = { .lde_repc = 0x35c2 },
	[17] = { .lde_repc = 0x3332 },
	[18] = { .lde_repc = 0x35c2 },
	[19] = { .lde_repc = 0x35c2 },
	[20] = { .lde_repc = 0x47ae },
	[21] = { .lde_repc = 0x3ae0 },
	[22] = { .lde_repc = 0x3850 },
	[23] = { .lde_repc = 0x30a2 },
	[24] = { .lde_repc = 0x3850 },
};

/* Pulse repetition frequency configurations */
static const struct dw1000_prf_config dw1000_prf_configs[] = {
	[DW1000_PRF_16M] = {
		.agc_tune1 = { 0x70, 0x88 },
		.lde_cfg2 = { 0x07, 0x16 },
	},
	[DW1000_PRF_64M] = {
		.agc_tune1 = { 0x9b, 0x88 },
		.lde_cfg2 = { 0x07, 0x06 },
	},
};

/* Data rate configurations */
static const struct dw1000_rate_config dw1000_rate_configs[] = {
	[DW1000_RATE_110K] = {
		.tdsym_ns = 8205,
		.txpsr = 0x3,
		.drx_tune0b = 0x000a,
		.drx_tune1b = 0x0064,
		.drx_tune2 = {
			[DW1000_PRF_16M] = { 0x1d, 0x01, 0x1a, 0x37 },
			[DW1000_PRF_16M] = { 0x96, 0x02, 0x3b, 0x37 },
		},
		.drx_tune4h = 0x0028,
	},
	[DW1000_RATE_850K] = {
		.tdsym_ns = 1025,
		.txpsr = 0x2,
		.drx_tune0b = 0x0001,
		.drx_tune1b = 0x0020,
		.drx_tune2 = {
			[DW1000_PRF_16M] = { 0x9a, 0x00, 0x1a, 0x35 },
			[DW1000_PRF_16M] = { 0x5e, 0x01, 0x3b, 0x35 },
		},
		.drx_tune4h = 0x0028,
	},
	[DW1000_RATE_6800K] = {
		.tdsym_ns = 128,
		.txpsr = 0x1,
		.drx_tune0b = 0x0001,
		.drx_tune1b = 0x0010,
		.drx_tune2 = {
			[DW1000_PRF_16M] = { 0x2d, 0x00, 0x1a, 0x31 },
			[DW1000_PRF_16M] = { 0x6b, 0x00, 0x3b, 0x31 },
		},
		.drx_tune4h = 0x0010,
	},
};

/* Fixed configurations */
static const struct dw1000_fixed_config dw1000_fixed_config = {
	.agc_tune2 = { 0x07, 0xa9, 0x02, 0x25 },
	.agc_tune3 = { 0x35, 0x00 },
};

/* Radio reconfiguration change bitmasks */
enum dw1000_configuration_change {
	DW1000_CONFIGURE_FIXED		= BIT(0),
	DW1000_CONFIGURE_CHANNEL	= BIT(1),
	DW1000_CONFIGURE_PCODE		= BIT(2),
	DW1000_CONFIGURE_PRF		= BIT(3),
	DW1000_CONFIGURE_RATE		= BIT(4),
	DW1000_CONFIGURE_SMART_POWER	= BIT(5),
};

/**
 * dw1000_pcode() - Calculate preamble code
 *
 * @dw:			DW1000 device
 * @return:		Preamble code
 */
static unsigned int dw1000_pcode(struct dw1000 *dw)
{
	unsigned long pcodes;

	/* Use explicitly set preamble code if configured and pcodes */
	pcodes = dw1000_channel_configs[dw->channel].pcodes[dw->prf];
	if (BIT(dw->pcode) & pcodes)
		return dw->pcode;

	/* Otherwise, use first pcodes preamble code */
	return (ffs(pcodes) - 1);
}

/**
 * dw1000_reconfigure() - Reconfigure radio parameters
 *
 * @dw:			DW1000 device
 * @changed:		Changed parameter bitmask
 * @return:		0 on success or -errno
 */
static int dw1000_reconfigure(struct dw1000 *dw, unsigned int changed)
{
	const struct dw1000_channel_config *channel_cfg;
	const struct dw1000_pcode_config *pcode_cfg;
	const struct dw1000_prf_config *prf_cfg;
	const struct dw1000_rate_config *rate_cfg;
	const struct dw1000_fixed_config *fixed_cfg;
	uint8_t tx_power[sizeof(channel_cfg->tx_power[0])];
	uint16_t lde_repc;
	unsigned int channel;
	unsigned int pcode;
	enum dw1000_prf prf;
	enum dw1000_rate rate;
	bool smart_power;
	unsigned int i;
	int mask;
	int value;
	int rc;

	/* Look up current configurations */
	channel = dw->channel;
	pcode = dw1000_pcode(dw);
	prf = dw->prf;
	rate = dw->rate;
	smart_power = dw->smart_power;
	channel_cfg = &dw1000_channel_configs[channel];
	pcode_cfg = &dw1000_pcode_configs[pcode];
	prf_cfg = &dw1000_prf_configs[prf];
	rate_cfg = &dw1000_rate_configs[rate];
	fixed_cfg = &dw1000_fixed_config;

	/* Calculate inter-frame spacing */
	dw->phy->symbol_duration = (rate_cfg->tdsym_ns / 1000);
	if (!dw->phy->symbol_duration)
		dw->phy->symbol_duration = 1;
	dw->phy->lifs_period =
		((IEEE802154_LIFS_PERIOD * rate_cfg->tdsym_ns) / 1000);
	dw->phy->sifs_period =
		((IEEE802154_SIFS_PERIOD * rate_cfg->tdsym_ns) / 1000);

	/* SYS_CFG register */
	if (changed & (DW1000_CONFIGURE_RATE | DW1000_CONFIGURE_SMART_POWER)) {
		mask = (DW1000_SYS_CFG_DIS_STXP | DW1000_SYS_CFG_RXM110K);
		value = ((smart_power ? 0 : DW1000_SYS_CFG_DIS_STXP) |
			 ((rate == DW1000_RATE_110K) ?
			  DW1000_SYS_CFG_RXM110K : 0));
		if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0, mask,
					     value)) != 0)
			return rc;
	}

	/* TX_FCTRL registers */
	if (changed & DW1000_CONFIGURE_RATE) {
		mask = DW1000_TX_FCTRL1_TXBR_MASK;
		value = DW1000_TX_FCTRL1_TXBR(rate);
		if ((rc = regmap_update_bits(dw->tx_fctrl.regs,
					     DW1000_TX_FCTRL1,
					     mask, value)) != 0)
			return rc;
	}
	if (changed & (DW1000_CONFIGURE_PRF | DW1000_CONFIGURE_RATE)) {
		mask = (DW1000_TX_FCTRL2_TXPRF_MASK |
			DW1000_TX_FCTRL2_TXPSR_MASK |
			DW1000_TX_FCTRL2_PE_MASK);
		value = (DW1000_TX_FCTRL2_TXPRF(prf) |
			 DW1000_TX_FCTRL2_TXPSR(rate_cfg->txpsr));
		if ((rc = regmap_update_bits(dw->tx_fctrl.regs,
					     DW1000_TX_FCTRL2,
					     mask, value)) != 0)
			return rc;
	}

	/* TX_POWER register */
	if (changed & (DW1000_CONFIGURE_CHANNEL | DW1000_CONFIGURE_PRF |
		       DW1000_CONFIGURE_SMART_POWER)) {
		memcpy(tx_power, &channel_cfg->tx_power[prf], sizeof(tx_power));
		if (!smart_power) {
			for (i = 1; i < sizeof(tx_power); i++)
				tx_power[i] = tx_power[0];
		}
		if ((rc = regmap_raw_write(dw->tx_power.regs, 0, tx_power,
					   sizeof(tx_power))) != 0)
			return rc;
	}

	/* CHAN_CTRL register */
	if (changed & (DW1000_CONFIGURE_CHANNEL | DW1000_CONFIGURE_PCODE |
		       DW1000_CONFIGURE_PRF)) {
		mask = (DW1000_CHAN_CTRL_TX_CHAN_MASK |
			DW1000_CHAN_CTRL_RX_CHAN_MASK |
			DW1000_CHAN_CTRL_RXPRF_MASK |
			DW1000_CHAN_CTRL_TX_PCODE_MASK |
			DW1000_CHAN_CTRL_RX_PCODE_MASK);
		value = (DW1000_CHAN_CTRL_TX_CHAN(channel) |
			 DW1000_CHAN_CTRL_RX_CHAN(channel) |
			 DW1000_CHAN_CTRL_RXPRF(prf) |
			 DW1000_CHAN_CTRL_TX_PCODE(pcode) |
			 DW1000_CHAN_CTRL_RX_PCODE(pcode));
		if ((rc = regmap_update_bits(dw->chan_ctrl.regs, 0, mask,
					     value)) != 0)
			return rc;
	}

	/* AGC_CTRL registers */
	if (changed & DW1000_CONFIGURE_PRF) {
		if ((rc = regmap_raw_write(dw->agc_ctrl.regs, DW1000_AGC_TUNE1,
					   prf_cfg->agc_tune1,
					   sizeof(prf_cfg->agc_tune1))) != 0)
			return rc;
	}
	if (changed & DW1000_CONFIGURE_FIXED) {
		if ((rc = regmap_raw_write(dw->agc_ctrl.regs, DW1000_AGC_TUNE2,
					   fixed_cfg->agc_tune2,
					   sizeof(fixed_cfg->agc_tune2))) != 0)
			return rc;
		if ((rc = regmap_raw_write(dw->agc_ctrl.regs, DW1000_AGC_TUNE3,
					   fixed_cfg->agc_tune3,
					   sizeof(fixed_cfg->agc_tune3))) != 0)
			return rc;
	}

	/* DRX_CONF registers */
	if (changed & DW1000_CONFIGURE_RATE) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE0B,
				       rate_cfg->drx_tune0b)) != 0)
			return rc;
	}
	if (changed & DW1000_CONFIGURE_PRF) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE1A,
				       prf_cfg->drx_tune1a)) != 0)
			return rc;
	}
	if (changed & DW1000_CONFIGURE_RATE) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE1B,
				       rate_cfg->drx_tune1b)) != 0)
			return rc;
	}
	if (changed & (DW1000_CONFIGURE_PRF | DW1000_CONFIGURE_RATE)) {
		if ((rc = regmap_raw_write(dw->drx_conf.regs, DW1000_DRX_TUNE2,
					   rate_cfg->drx_tune2[prf],
					   sizeof(rate_cfg->
						  drx_tune2[prf]))) != 0)
			return rc;
	}
	if (changed & DW1000_CONFIGURE_RATE) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE4H,
				       rate_cfg->drx_tune4h)) != 0)
			return rc;
	}

	/* RF_CONF registers */
	if (changed & DW1000_CONFIGURE_CHANNEL) {
		if ((rc = regmap_raw_write(dw->rf_conf.regs, DW1000_RF_TXCTRL,
					   channel_cfg->rf_txctrl,
					   sizeof(channel_cfg->rf_txctrl))) !=0)
			return rc;
		if ((rc = regmap_write(dw->rf_conf.regs, DW1000_RF_RXCTRLH,
				       channel_cfg->rf_rxctrlh)) != 0)
			return rc;
	}

	/* TX_CAL registers */
	if (changed & DW1000_CONFIGURE_CHANNEL) {
		if ((rc = regmap_write(dw->tx_cal.regs, DW1000_TC_PGDELAY,
				       channel_cfg->tc_pgdelay)) != 0)
			return rc;
	}

	/* FS_CTRL registers */
	if (changed & DW1000_CONFIGURE_CHANNEL) {
		if ((rc = regmap_raw_write(dw->fs_ctrl.regs, DW1000_FS_PLLCFG,
					   channel_cfg->fs_pllcfg,
					   sizeof(channel_cfg->fs_pllcfg))) !=0)
			return rc;
		if ((rc = regmap_write(dw->fs_ctrl.regs, DW1000_FS_PLLTUNE,
				       channel_cfg->fs_plltune)) != 0)
			return rc;
	}

	/* LDE_IF registers */
	if (changed & DW1000_CONFIGURE_PRF) {
		if ((rc = regmap_raw_write(dw->lde_if.regs, DW1000_LDE_CFG2,
					   prf_cfg->lde_cfg2,
					   sizeof(prf_cfg->lde_cfg2))) != 0)
			return rc;
	}
	if (changed & (DW1000_CONFIGURE_PCODE | DW1000_CONFIGURE_RATE)) {
		lde_repc = pcode_cfg->lde_repc;
		if (rate == DW1000_RATE_110K)
			lde_repc >>= 3;
		cpu_to_le16s(&lde_repc);
		if ((rc = regmap_raw_write(dw->lde_if.regs, DW1000_LDE_REPC,
					   &lde_repc, sizeof(lde_repc))) != 0)
			return rc;
	}

	return 0;
}

/**
 * dw1000_configure_channel() - Configure radio channel
 *
 * @dw:			DW1000 device
 * @channel:		Channel number
 * @return:		0 on success or -errno
 */
static int dw1000_configure_channel(struct dw1000 *dw, unsigned int channel)
{
	int rc;

	/* Check that channel is supported */
	if (!(BIT(channel) & DW1000_CHANNELS))
		return -EINVAL;

	/* Record channel */
	dw->channel = channel;

	/* Reconfigure channel */
	if ((rc = dw1000_reconfigure(dw, DW1000_CONFIGURE_CHANNEL)) != 0)
		return rc;

	dev_dbg(dw->dev, "set channel %d\n", channel);
	return 0;
}

/**
 * dw1000_configure_pcode() - Configure preamble code
 *
 * @dw:			DW1000 device
 * @pcode:		Preamble code (or 0 for automatic selection)
 * @return:		0 on success or -errno
 */
static int dw1000_configure_pcode(struct dw1000 *dw, unsigned int pcode)
{
	unsigned long pcodes;
	int rc;

	/* Check that preamble code is valid */
	if (pcode) {
		pcodes = dw1000_channel_configs[dw->channel].pcodes[dw->prf];
		if (!(BIT(pcode) & pcodes))
			return -EINVAL;
	}

	/* Record preamble code */
	dw->pcode = pcode;

	/* Reconfigure preamble code */
	if ((rc = dw1000_reconfigure(dw, DW1000_CONFIGURE_PCODE)) != 0)
		return rc;

	dev_dbg(dw->dev, "set preamble code %d (requested %d)\n",
		dw1000_pcode(dw), pcode);
	return 0;
}

/**
 * dw1000_configure_prf() - Configure pulse repetition frequency
 *
 * @dw:			DW1000 device
 * @prf:		Pulse repetition frequency
 * @return:		0 on success or -errno
 */
static int dw1000_configure_prf(struct dw1000 *dw, enum dw1000_prf prf)
{
	int rc;

	/* Record pulse repetition frequency */
	dw->prf = prf;

	/* Reconfigure pulse repetition frequency */
	if ((rc = dw1000_reconfigure(dw, DW1000_CONFIGURE_PRF)) != 0)
		return rc;

	dev_dbg(dw->dev, "set pulse repetition frequency %dMHz\n",
		dw1000_prfs[dw->prf]);
	return 0;
}

/**
 * dw1000_configure_rate() - Configure data rate
 *
 * @dw:			DW1000 device
 * @rate:		Data rate
 * @return:		0 on success or -errno
 */
static int dw1000_configure_rate(struct dw1000 *dw, enum dw1000_rate rate)
{
	int rc;

	/* Record data rate */
	dw->rate = rate;

	/* Reconfigure data rate */
	if ((rc = dw1000_reconfigure(dw, DW1000_CONFIGURE_RATE)) != 0)
		return rc;

	dev_dbg(dw->dev, "set data rate %dkbps\n", dw1000_rates[dw->rate]);
	return 0;
}

/**
 * dw1000_configure_smart_power() - Configure use of smart power control
 *
 * @dw:			DW1000 device
 * @smart_power:	Smart power control enabled
 * @return:		0 on success or -errno
 */
static int dw1000_configure_smart_power(struct dw1000 *dw, bool smart_power)
{
	int rc;

	/* Record smart power control */
	dw->smart_power = smart_power;

	/* Reconfigure transmit power */
	if ((rc = dw1000_reconfigure(dw, DW1000_CONFIGURE_SMART_POWER)) != 0)
		return rc;

	dev_dbg(dw->dev, "set smart power control %s\n",
		(smart_power ? "on" : "off"));
	return 0;
}

/******************************************************************************
 *
 * Transmit datapath
 *
 */

/**
 * dw1000_xmit_async() - Start packet transmission
 *
 * @hw:			IEEE 802.15.4 device
 * @skb:		Socket buffer
 * @return:		0 on success or -errno
 */
static int dw1000_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct dw1000 *dw = hw->priv;
	struct dw1000_xmit *tx = &dw->tx;
	int rc;

	/* Sanity check */
	if (tx->skb) {
		dev_err(dw->dev, "concurrent transmit is not supported\n");
		rc = -ENOBUFS;
		goto err_concurrent;
	}

	/* Construct transmission */
	tx->skb = skb;
	tx->tx_buffer.data.tx_buf = skb->data;
	tx->tx_buffer.data.len = skb->len;
	tx->len = DW1000_TX_FCTRL0_TFLEN(skb->len);
	tx->submitted = false;

	/* Submit transmission */
	if ((rc = spi_async(dw->spi, &tx->msg)) != 0)
		goto err_spi;

	return 0;

 err_spi:
	tx->skb = NULL;
 err_concurrent:
	return rc;
}

/**
 * dw1000_xmit_complete() - Handle transmit completion
 *
 * @dw:			DW1000 device
 */
static void dw1000_xmit_complete(struct dw1000 *dw)
{
	struct dw1000_xmit *tx = &dw->tx;
	struct sk_buff *skb;
	size_t sifs_max_len;

	/* Ignore if submission has not yet completed */
	if (!tx->submitted) {
		dev_err(dw->dev, "spurious transmit completion\n");
		return;
	}

	/* Report completion to IEEE 802.15.4 stack */
	skb = tx->skb;
	tx->skb = NULL;
	sifs_max_len = (IEEE802154_MAX_SIFS_FRAME_SIZE - IEEE802154_FCS_LEN);
	ieee802154_xmit_complete(dw->hw, skb, (skb->len <= sifs_max_len));
}

/**
 * dw1000_xmit_complete() - Handle transmit submission completion
 *
 * @context:		DW1000 device
 */
static void dw1000_xmit_submit_complete(void *context)
{
	struct dw1000 *dw = context;
	struct dw1000_xmit *tx = &dw->tx;

	/* Mark as submitted */
	tx->submitted = true;

	/* Complete immediately if submission failed */
	if (tx->msg.status != 0) {
		dev_err(dw->dev, "transmit submission failed: %d\n",
			tx->msg.status);
		dw1000_xmit_complete(dw);
	}
}

/******************************************************************************
 *
 * Interrupt handling
 *
 */

/**
 * dw1000_isr() - Interrupt handler
 *
 * @irq:		Interrupt number
 * @context:		DW1000 device
 * @return:		IRQ status
 */
static irqreturn_t dw1000_isr(int irq, void *context)
{
	struct dw1000 *dw = context;

	/* Disable interrupt and schedule status register handler */
	disable_irq_nosync(irq);
	schedule_work(&dw->irq_work);
	return IRQ_HANDLED;
}

/**
 * dw1000_irq_worker() - Interrupt status register handler
 *
 * @work:		Worker
 */
static void dw1000_irq_worker(struct work_struct *work)
{
	struct dw1000 *dw = container_of(work, struct dw1000, irq_work);
	int status;
	int rc;

	/* Read interrupt status register */
	if ((rc = regmap_read(dw->sys_status.regs, 0, &status)) != 0)
		goto abort;

	/* Handle transmit completion, if applicable */
	if (status & DW1000_IRQ_TXFRS)
		dw1000_xmit_complete(dw);

	/* Acknowledge interrupts */
	if ((rc = regmap_write(dw->sys_status.regs, 0, status)) != 0)
		goto abort;

 abort:
	enable_irq(dw->spi->irq);
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
	int value;
	int rc;

	/* Enable interrupt generation */
	value = DW1000_IRQ_TXFRS;
	if ((rc = regmap_write(dw->sys_mask.regs, 0, value)) != 0)
		return rc;

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
	int rc;

	/* Disable receiver and transmitter */
	if ((rc = regmap_write(dw->sys_ctrl.regs, DW1000_SYS_CTRL0,
			       DW1000_SYS_CTRL0_TRXOFF)) != 0) {
		dev_err(dw->dev, "could not disable TX/RX: %d\n", rc);
	}

	/* Disable further interrupt generation */
	if ((rc = regmap_write(dw->sys_mask.regs, 0, 0)) != 0) {
		dev_err(dw->dev, "could not disable interrupts: %d\n", rc);
	}

	/* Complete any pending interrupt work */
	flush_work(&dw->irq_work);

	dev_info(dw->dev, "stopped\n");
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

	/* Configure radio channel */
	if ((rc = dw1000_configure_channel(dw, channel)) != 0)
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
	.set_promiscuous_mode = dw1000_set_promiscuous_mode,
};

/******************************************************************************
 *
 * Device attributes
 *
 */

#define DW1000_ATTR_RW(_name, _mode) \
	DEVICE_ATTR(_name, _mode, dw1000_show_##_name, dw1000_store_##_name)

/* Preamble code */
static ssize_t dw1000_show_pcode(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_pcode(dw));
}
static ssize_t dw1000_store_pcode(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dw1000 *dw = to_dw1000(dev);
	int pcode;
	int rc;

	if ((rc = kstrtoint(buf, 0, &pcode)) != 0)
		return rc;
	if ((rc = dw1000_configure_pcode(dw, pcode)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(pcode, 0644);

/* Pulse repetition frequency */
static ssize_t dw1000_show_prf(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_prfs[dw->prf]);
}
static ssize_t dw1000_store_prf(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dw1000 *dw = to_dw1000(dev);
	int raw;
	int prf;
	int rc;

	if ((rc = kstrtoint(buf, 0, &raw)) != 0)
		return rc;
	if ((prf = dw1000_prf(raw)) < 0)
		return prf;
	if ((rc = dw1000_configure_prf(dw, prf)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(prf, 0644);

/* Data rate */
static ssize_t dw1000_show_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_rates[dw->rate]);
}
static ssize_t dw1000_store_rate(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct dw1000 *dw = to_dw1000(dev);
	int raw;
	int rate;
	int rc;

	if ((rc = kstrtoint(buf, 0, &raw)) != 0)
		return rc;
	if ((rate = dw1000_rate(raw)) < 0)
		return rate;
	if ((rc = dw1000_configure_rate(dw, rate)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(rate, 0644);

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
	if ((rc = dw1000_configure_smart_power(dw, smart_power)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(smart_power, 0644);

/* Attribute list */
static struct attribute *dw1000_attrs[] = {
	&dev_attr_pcode.attr,
	&dev_attr_prf.attr,
	&dev_attr_rate.attr,
	&dev_attr_smart_power.attr,
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
	if ((rc = regmap_update_bits(dw->otp_if.regs, DW1000_OTP_CTRL,
				     DW1000_OTP_CTRL_LDELOAD,
				     DW1000_OTP_CTRL_LDELOAD)) != 0)
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
	int sys_cfg_filters;
	int mask;
	int value;
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
	mask = (sys_cfg_filters | DW1000_SYS_CFG_DIS_DRXB |
		DW1000_SYS_CFG_RXAUTR);
	value = (sys_cfg_filters | DW1000_SYS_CFG_RXAUTR);
	if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0, mask, value)) != 0)
		return rc;

	/* Handle short inter-frame gaps in hardware */
	value = DW1000_TX_FCTRL4_IFSDELAY(IEEE802154_SIFS_PERIOD);
	if ((rc = regmap_write(dw->tx_fctrl.regs, DW1000_TX_FCTRL4,
			       value)) != 0)
		return rc;

	/* Enable GPIO block and kilohertz clock for LED blinking */
	mask = (DW1000_PMSC_CTRL0_GPCE | DW1000_PMSC_CTRL0_GPRN |
		DW1000_PMSC_CTRL0_GPDCE | DW1000_PMSC_CTRL0_GPDRN |
		DW1000_PMSC_CTRL0_KHZCLKEN);
	value = (DW1000_PMSC_CTRL0_GPCE | DW1000_PMSC_CTRL0_GPRN |
		 DW1000_PMSC_CTRL0_GPDCE | DW1000_PMSC_CTRL0_GPDRN |
		 DW1000_PMSC_CTRL0_KHZCLKEN);
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     mask, value)) != 0)
		return rc;

	/* Configure GPIOs as LED outputs */
	mask = (DW1000_GPIO_MODE_MSGP0_MASK | DW1000_GPIO_MODE_MSGP1_MASK |
		DW1000_GPIO_MODE_MSGP2_MASK | DW1000_GPIO_MODE_MSGP3_MASK);
	value = (DW1000_GPIO_MODE_MSGP0_RXOKLED | DW1000_GPIO_MODE_MSGP1_SFDLED|
		 DW1000_GPIO_MODE_MSGP2_RXLED | DW1000_GPIO_MODE_MSGP3_TXLED);
	if ((rc = regmap_update_bits(dw->gpio_ctrl.regs, DW1000_GPIO_MODE,
				     mask, value)) != 0)
		return rc;

	/* Enable LED blinking */
	mask = (DW1000_PMSC_LEDC_BLINK_TIM_MASK | DW1000_PMSC_LEDC_BLNKEN);
	value = (DW1000_PMSC_LEDC_BLINK_TIM_DEFAULT | DW1000_PMSC_LEDC_BLNKEN);
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_LEDC,
				     mask, value)) != 0)
		return rc;

	/* Load LDE microcode */
	if ((rc = dw1000_load_lde(dw)) != 0)
		return rc;

	/* Configure radio */
	if ((rc = dw1000_reconfigure(dw, -1U)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_prepare() - Prepare predefined transfers
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_prepare(struct dw1000 *dw)
{
	static const uint8_t txstrt = DW1000_SYS_CTRL0_TXSTRT;
	struct dw1000_xmit *tx = &dw->tx;

	/* Prepare transmission */
	memset(tx, 0, sizeof(*tx));
	spi_message_init_no_memset(&tx->msg);
	tx->msg.complete = dw1000_xmit_submit_complete;
	tx->msg.context = dw;
	dw1000_init_write(&tx->msg, &tx->tx_buffer, DW1000_TX_BUFFER, 0,
			  NULL, 0);
	dw1000_init_write(&tx->msg, &tx->tx_fctrl, DW1000_TX_FCTRL,
			  DW1000_TX_FCTRL0, &tx->len, sizeof(tx->len));
	dw1000_init_write(&tx->msg, &tx->sys_ctrl, DW1000_SYS_CTRL,
			  DW1000_SYS_CTRL0, &txstrt, sizeof(txstrt));

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
	dw->hw = hw;
	dw->phy = hw->phy;
	dw->channel = DW1000_CHANNEL_DEFAULT;
	dw->pcode = 0;
	dw->prf = DW1000_PRF_64M;
	dw->rate = DW1000_RATE_6800K;
	dw->smart_power = true;
	INIT_WORK(&dw->irq_work, dw1000_irq_worker);
	hw->parent = &spi->dev;

	/* Report capabilities */
	hw->flags = (IEEE802154_HW_TX_OMIT_CKSUM |
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

	/* Prepare predefined transfers */
	if ((rc = dw1000_prepare(dw)) != 0) {
		dev_err(dw->dev, "preparation failed: %d\n", rc);
		goto err_prepare;
	}

	/* Add attribute group */
	if ((rc = sysfs_create_group(&dw->dev->kobj, &dw1000_attr_group)) != 0){
		dev_err(dw->dev, "could not create attributes: %d\n", rc);
		goto err_create_group;
	}

	/* Hook interrupt */
	if ((rc = devm_request_irq(dw->dev, spi->irq, dw1000_isr, 0,
				   dev_name(dw->dev), dw)) != 0) {
		dev_err(dw->dev, "could not request IRQ %d: %d\n",
			spi->irq, rc);
		goto err_request_irq;
	}

	/* Register IEEE 802.15.4 device */
	if ((rc = ieee802154_register_hw(hw)) != 0) {
		dev_err(dw->dev, "could not register: %d\n", rc);
		goto err_register_hw;
	}

	spi_set_drvdata(spi, hw);
	return 0;

	ieee802154_unregister_hw(hw);
 err_register_hw:
 err_request_irq:
	sysfs_remove_group(&dw->dev->kobj, &dw1000_attr_group);
 err_create_group:
 err_prepare:
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Brown <mbrown@fensystems.co.uk>");
MODULE_DESCRIPTION("DecaWave DW1000 IEEE 802.15.4 driver");
