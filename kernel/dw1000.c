/*
 * DecaWave DW1000 IEEE 802.15.4 UWB wireless driver
 *
 * Copyright (C) 2018 Michael Brown <mbrown@fensystems.co.uk>
 * Copyright (C) 2019 Petri Mattila <petri.mattila@unipart.io>
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <uapi/linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <net/mac802154.h>

/* Enable error injection */
#define DW1000_ERROR_INJECT

/* Enable GPIO controlled LEDs */
#define DW1000_GPIO_LEDS

#include "dw1000.h"
#include "inject.h"


/******************************************************************************
 *
 * Module parameters
 *
 */

static int dw1000_tsinfo_ena = 0;
module_param_named(tsinfo, dw1000_tsinfo_ena, uint, 0444);
MODULE_PARM_DESC(tsinfo, "Timestamp information enable flags");


/******************************************************************************
 *
 * Forward declarations
 *
 */

static int dw1000_reset(struct dw1000 *dw);
static int dw1000_enable(struct dw1000 *dw);
static int dw1000_disable(struct dw1000 *dw);
static int dw1000_poweron(struct dw1000 *dw);
static int dw1000_poweroff(struct dw1000 *dw);

static __always_inline void
dw1000_set_state(struct dw1000 *dw, unsigned state);
static __always_inline void
dw1000_set_state_timeout(struct dw1000 *dw, unsigned state, unsigned us);

static void dw1000_state_timer_handler(struct timer_list *timer);
static void dw1000_adc_timer_handler(struct timer_list *timer);
static void dw1000_ptp_timer_handler(struct timer_list *timer);

static int dw1000_event_thread(void *data);

static void dw1000_enqueue(struct dw1000 *dw, unsigned long work);
static void dw1000_enqueue_wait(struct dw1000 *dw, unsigned long work);
static void dw1000_enqueue_irq(struct dw1000 *dw);

static inline void dw1000_stats_inc(struct dw1000 *dw, int id);


/******************************************************************************
 *
 * Enumeration conversions
 *
 */

/* Statistics items */
#define STATS_ITEM(name)   [DW1000_STATS_ ## name] = #name
static const char * dw1000_stats[] = {
	STATS_ITEM(RX_FRAME),
	STATS_ITEM(TX_FRAME),
	STATS_ITEM(RX_ERROR),
	STATS_ITEM(TX_ERROR),
	STATS_ITEM(IRQ_COUNT),
	STATS_ITEM(SPI_ERROR),
	STATS_ITEM(RX_RESET),
	STATS_ITEM(RX_RESYNC),
	STATS_ITEM(RX_HSRBP),
	STATS_ITEM(RX_OVRR),
	STATS_ITEM(RX_DFR),
	STATS_ITEM(RX_FCG),
	STATS_ITEM(RX_FCE),
	STATS_ITEM(RX_LDEDONE),
	STATS_ITEM(RX_KPI_ERROR),
	STATS_ITEM(RX_STAMP_ERROR),
	STATS_ITEM(RX_FRAME_REP),
	STATS_ITEM(TX_RETRY),
	STATS_ITEM(SNR_REJECT),
	STATS_ITEM(FPR_REJECT),
	STATS_ITEM(NOISE_REJECT),
	STATS_ITEM(HARD_RESET),
};

#ifdef DW1000_ERROR_INJECT
/* Error injection items */
#define INJECT_ITEM(name)   [ERROR_INJECT_ ## name] = #name
const char * dw1000_inject_items[] = {
	INJECT_ITEM(NONE),
	INJECT_ITEM(STATE_DELAY),
	INJECT_ITEM(STATE_SYS_STATUS),
	INJECT_ITEM(TX_DELAY),
	INJECT_ITEM(TX_SPI_DATA),
	INJECT_ITEM(TX_SPI_INFO),
	INJECT_ITEM(RX_DELAY),
	INJECT_ITEM(RX_SYS_STATUS),
	INJECT_ITEM(SPI_RD_ERROR),
	INJECT_ITEM(SPI_WR_ERROR),
	INJECT_ITEM(CONFIG_ERROR),
};
#endif /* DW1000_ERROR_INJECT */

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

/* Transmit preamble symbol repetitions values */
static const unsigned int dw1000_txpsrs[] = {
	[DW1000_TXPSR_DEFAULT] = 0,
	[DW1000_TXPSR_64] = 64,
	[DW1000_TXPSR_128] = 128,
	[DW1000_TXPSR_256] = 256,
	[DW1000_TXPSR_512] = 512,
	[DW1000_TXPSR_1024] = 1024,
	[DW1000_TXPSR_2048] = 2048,
	[DW1000_TXPSR_4096] = 4096,
};

/**
 * Look up pulse repetition frequency
 *
 * @prf:		Pulse repetition frequency in MHz
 * @return:		Pulse repetition frequency enumeration or -errno
 */
static int dw1000_lookup_prf(unsigned int prf)
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
static int dw1000_lookup_rate(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dw1000_rates); i++) {
		if (dw1000_rates[i] && (dw1000_rates[i] == rate))
			return i;
	}
	return -EINVAL;
}

/**
 * Look up preamble symbol repetitions
 *
 * @txpsr:		Preamble symbol repetitions
 * @return:		Preamble symbol repetitions enumeration or -errno
 */
static int dw1000_lookup_txpsr(unsigned int txpsr)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dw1000_txpsrs); i++) {
		if (dw1000_txpsrs[i] == txpsr)
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
		previous = list_last_entry(&msg->transfers,
					   struct spi_transfer, transfer_list);
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
	int rc;
	
	struct {
		struct spi_message msg;
		struct dw1000_spi_transfers xfers;
	} read;

	/* Construct message */
	memset(&read, 0, sizeof(read));
	spi_message_init_no_memset(&read.msg);
	dw1000_init_read(&read.msg, &read.xfers, file, offset, data, len);

	/* SPI transaction */
	rc = spi_sync(dw->spi, &read.msg);

	INJECT_ERROR(dw, SPI_RD_ERROR, rc);

	return rc;
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
	int rc;
	
	struct {
		struct spi_message msg;
		struct dw1000_spi_transfers xfers;
	} write;

	/* Construct message */
	memset(&write, 0, sizeof(write));
	spi_message_init_no_memset(&write.msg);
	dw1000_init_write(&write.msg, &write.xfers, file, offset, data, len);

	/* SPI transaction */
	rc = spi_sync(dw->spi, &write.msg);

	INJECT_ERROR(dw, SPI_WR_ERROR, rc);

	return rc;
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
			.val_bits = (sizeof(_type) * BITS_PER_BYTE),	\
			.max_register = ((_len) - sizeof(_type)),	\
			__VA_ARGS__					\
		},							\
	}
DW1000_REGMAP(dev_id, DW1000_DEV_ID, DW1000_DEV_ID_LEN, uint16_t);
DW1000_REGMAP(eui, DW1000_EUI, DW1000_EUI_LEN, uint8_t);
DW1000_REGMAP(panadr, DW1000_PANADR, DW1000_PANADR_LEN, uint16_t);
DW1000_REGMAP(sys_cfg, DW1000_SYS_CFG, DW1000_SYS_CFG_LEN, uint32_t);
DW1000_REGMAP(sys_time, DW1000_SYS_TIME, DW1000_SYS_TIME_LEN, uint8_t);
DW1000_REGMAP(tx_fctrl, DW1000_TX_FCTRL, DW1000_TX_FCTRL_LEN, uint8_t);
DW1000_REGMAP(tx_buffer, DW1000_TX_BUFFER, DW1000_TX_BUFFER_LEN, uint8_t);
DW1000_REGMAP(dx_time, DW1000_DX_TIME, DW1000_DX_TIME_LEN, uint8_t);
DW1000_REGMAP(rx_fwto, DW1000_RX_FWTO, DW1000_RX_FWTO_LEN, uint16_t);
DW1000_REGMAP(sys_ctrl, DW1000_SYS_CTRL, DW1000_SYS_CTRL_LEN, uint8_t);
DW1000_REGMAP(sys_mask, DW1000_SYS_MASK, DW1000_SYS_MASK_LEN, uint32_t);
DW1000_REGMAP(sys_status, DW1000_SYS_STATUS, DW1000_SYS_STATUS_LEN, uint32_t);
DW1000_REGMAP(rx_finfo, DW1000_RX_FINFO, DW1000_RX_FINFO_LEN, uint32_t);
DW1000_REGMAP(rx_buffer, DW1000_RX_BUFFER, DW1000_RX_BUFFER_LEN, uint8_t);
DW1000_REGMAP(rx_fqual, DW1000_RX_FQUAL, DW1000_RX_FQUAL_LEN, uint16_t);
DW1000_REGMAP(rx_ttcki, DW1000_RX_TTCKI, DW1000_RX_TTCKI_LEN, uint32_t);
DW1000_REGMAP(rx_ttcko, DW1000_RX_TTCKO, DW1000_RX_TTCKO_LEN, uint8_t);
DW1000_REGMAP(rx_time, DW1000_RX_TIME, DW1000_RX_TIME_LEN, uint8_t);
DW1000_REGMAP(tx_time, DW1000_TX_TIME, DW1000_TX_TIME_LEN, uint8_t);
DW1000_REGMAP(tx_antd, DW1000_TX_ANTD, DW1000_TX_ANTD_LEN, uint16_t);
DW1000_REGMAP(sys_state, DW1000_SYS_STATE, DW1000_SYS_STATE_LEN, uint8_t);
DW1000_REGMAP(ack_resp_t, DW1000_ACK_RESP_T, DW1000_ACK_RESP_T_LEN, uint32_t);
DW1000_REGMAP(rx_sniff, DW1000_RX_SNIFF, DW1000_RX_SNIFF_LEN, uint32_t);
DW1000_REGMAP(tx_power, DW1000_TX_POWER, DW1000_TX_POWER_LEN, uint8_t);
DW1000_REGMAP(chan_ctrl, DW1000_CHAN_CTRL, DW1000_CHAN_CTRL_LEN, uint32_t);
DW1000_REGMAP(usr_sfd, DW1000_USR_SFD, DW1000_USR_SFD_LEN, uint8_t);
DW1000_REGMAP(agc_ctrl, DW1000_AGC_CTRL, DW1000_AGC_CTRL_LEN, uint8_t);
DW1000_REGMAP(ext_sync, DW1000_EXT_SYNC, DW1000_EXT_SYNC_LEN, uint32_t);
DW1000_REGMAP(acc_mem, DW1000_ACC_MEM, DW1000_ACC_MEM_LEN, uint32_t);
DW1000_REGMAP(gpio_ctrl, DW1000_GPIO_CTRL, DW1000_GPIO_CTRL_LEN, uint32_t);
DW1000_REGMAP(drx_conf, DW1000_DRX_CONF, DW1000_DRX_CONF_LEN, uint16_t);
DW1000_REGMAP(rf_conf, DW1000_RF_CONF, DW1000_RF_CONF_LEN, uint8_t);
DW1000_REGMAP(tx_cal, DW1000_TX_CAL, DW1000_TX_CAL_LEN, uint8_t);
DW1000_REGMAP(fs_ctrl, DW1000_FS_CTRL, DW1000_FS_CTRL_LEN, uint8_t);
DW1000_REGMAP(aon, DW1000_AON, DW1000_AON_LEN, uint8_t);
DW1000_REGMAP(otp_if, DW1000_OTP_IF, DW1000_OTP_IF_LEN, uint16_t);
DW1000_REGMAP(lde_if, DW1000_LDE_IF, DW1000_LDE_IF_LEN, uint8_t);
DW1000_REGMAP(dig_diag, DW1000_DIG_DIAG, DW1000_DIG_DIAG_LEN, uint16_t);
DW1000_REGMAP(pmsc, DW1000_PMSC, DW1000_PMSC_LEN, uint32_t);

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
	&dw1000_sys_state,
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
 * One-time programmable memory (OTP) access
 *
 */

/**
 * dw1000_otp_read() - Read from one-time programmable memory
 *
 * @dw:			DW1000 device
 * @address:		OTP address
 * @data:		Data to fill in
 * @return:		0 on success or -errno
 */
static int dw1000_otp_read(struct dw1000 *dw,
			   unsigned int address, __le32 *data)
{
	int otp_ctrl;
	int rc;

	/* Set OTP address */
	if ((rc = regmap_write(dw->otp_if.regs, DW1000_OTP_ADDR, address)) != 0)
		return rc;

	/* Initiate read */
	otp_ctrl = (DW1000_OTP_CTRL_OTPRDEN | DW1000_OTP_CTRL_OTPREAD);
	if ((rc = regmap_write(dw->otp_if.regs, DW1000_OTP_CTRL,
			       otp_ctrl)) != 0)
		return rc;

	/* Wait for read to complete */
	usleep_range(DW1000_OTP_WAIT_MIN_US, DW1000_OTP_WAIT_MAX_US);

	/* Check that read has completed */
	if ((rc = regmap_read(dw->otp_if.regs, DW1000_OTP_CTRL,
			      &otp_ctrl)) != 0)
		return rc;
	if (otp_ctrl & DW1000_OTP_CTRL_OTPREAD) {
		dev_err(dw->dev, "OTP read timed out\n");
		return -ETIMEDOUT;
	}

	/* Read data */
	if ((rc = regmap_raw_read(dw->otp_if.regs, DW1000_OTP_RDAT, data,
				  sizeof(*data))) != 0)
		return rc;

	/* Terminate read */
	if ((rc = regmap_write(dw->otp_if.regs, DW1000_OTP_CTRL, 0)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_otp_reg_read() - Read OTP register(s)
 *
 * @context:		DW1000 register map
 * @reg:		Buffer containing register offset
 * @reg_len:		Length of register offset
 * @val:		Buffer to contain raw register value
 * @val_len:		Length of raw register value
 * @return:		0 on success or -errno
 */
static int dw1000_otp_reg_read(void *context, unsigned int reg,
			       unsigned int *val)
{
	struct dw1000 *dw = context;
	__le32 data;
	int rc;

	/* Read register */
	if ((rc = dw1000_otp_read(dw, reg, &data)) != 0)
		return rc;

	/* Convert value */
	*val = le32_to_cpu(data);

	return 0;
}

/* OTP register map bus */
static const struct regmap_bus dw1000_otp_regmap_bus = {
	.reg_read = dw1000_otp_reg_read,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

/* OTP register map configuration */
static const struct regmap_config dw1000_otp_regmap_config = {
	.name = "otp",
	.val_bits = 32,
	.max_register = 0x1f,
};

/**
 * dw1000_otp_regmap_init() - Initialise OTP register map
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_otp_regmap_init(struct dw1000 *dw)
{
	int rc;

	/* Initialise register maps */
	dw->otp = devm_regmap_init(dw->dev, &dw1000_otp_regmap_bus, dw,
				   &dw1000_otp_regmap_config);
	if (IS_ERR(dw->otp)) {
		rc = PTR_ERR(dw->otp);
		dev_err(dw->dev, "could not allocate OTP map: %d\n", rc);
		return rc;
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
			[DW1000_PRF_16M] = (BIT(3) | BIT(4) | BIT(7) | BIT(8)),
			[DW1000_PRF_64M] = (BIT(9) | BIT(10) | BIT(11) |
					    BIT(12) | BIT(17) | BIT(18) |
					    BIT(19) | BIT(20)),
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
			[DW1000_PRF_16M] = (BIT(3) | BIT(4)),
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
			[DW1000_PRF_16M] = (BIT(3) | BIT(4) | BIT(7) | BIT(8)),
			[DW1000_PRF_64M] = (BIT(9) | BIT(10) | BIT(11) |
					    BIT(12) | BIT(17) | BIT(18) |
					    BIT(19) | BIT(20)),
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
		.drx_tune1a = 0x0087,
		.lde_cfg2 = { 0x07, 0x16 },
	},
	[DW1000_PRF_64M] = {
		.agc_tune1 = { 0x9b, 0x88 },
		.drx_tune1a = 0x008d,
		.lde_cfg2 = { 0x07, 0x06 },
	},
};

/* Data rate configurations */
static const struct dw1000_rate_config dw1000_rate_configs[] = {
	[DW1000_RATE_110K] = {
		.tdsym_ns = 8205,
		.txpsr = DW1000_TXPSR_4096,
		.drx_tune0b = 0x000a,
		.drx_tune2 = {
			[DW1000_PRF_16M] = { 0x1d, 0x01, 0x1a, 0x37 },
			[DW1000_PRF_64M] = { 0x96, 0x02, 0x3b, 0x37 },
		},
	},
	[DW1000_RATE_850K] = {
		.tdsym_ns = 1025,
		.txpsr = DW1000_TXPSR_1024,
		.drx_tune0b = 0x0001,
		.drx_tune2 = {
			[DW1000_PRF_16M] = { 0x9a, 0x00, 0x1a, 0x35 },
			[DW1000_PRF_64M] = { 0x5e, 0x01, 0x3b, 0x35 },
		},
	},
	[DW1000_RATE_6800K] = {
		.tdsym_ns = 128,
		.txpsr = DW1000_TXPSR_64,
		.drx_tune0b = 0x0001,
		.drx_tune2 = {
			[DW1000_PRF_16M] = { 0x2d, 0x00, 0x1a, 0x31 },
			[DW1000_PRF_64M] = { 0x6b, 0x00, 0x3b, 0x31 },
		},
	},
};

/* Preamble symbol repetitions configurations */
static const struct dw1000_txpsr_config dw1000_txpsr_configs[] = {
	[DW1000_TXPSR_64]   = { .drx_tune1b = 0x0010, .drx_tune4h = 0x0010 },
	[DW1000_TXPSR_128]  = { .drx_tune1b = 0x0020, .drx_tune4h = 0x0028 },
	[DW1000_TXPSR_256]  = { .drx_tune1b = 0x0020, .drx_tune4h = 0x0028 },
	[DW1000_TXPSR_512]  = { .drx_tune1b = 0x0020, .drx_tune4h = 0x0028 },
	[DW1000_TXPSR_1024] = { .drx_tune1b = 0x0020, .drx_tune4h = 0x0028 },
	[DW1000_TXPSR_2048] = { .drx_tune1b = 0x0064, .drx_tune4h = 0x0028 },
	[DW1000_TXPSR_4096] = { .drx_tune1b = 0x0064, .drx_tune4h = 0x0028 },
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
	DW1000_CONFIGURE_ANTD		= BIT(4),
	DW1000_CONFIGURE_RATE		= BIT(5),
	DW1000_CONFIGURE_TXPSR		= BIT(6),
	DW1000_CONFIGURE_XTAL_TRIM	= BIT(7),
	DW1000_CONFIGURE_TX_POWER	= BIT(8),
	DW1000_CONFIGURE_SMART_POWER	= BIT(9),
	DW1000_CONFIGURE_PAN_ID         = BIT(10),
	DW1000_CONFIGURE_SHORT_ADDR     = BIT(11),
	DW1000_CONFIGURE_IEEE_ADDR      = BIT(12),
	DW1000_CONFIGURE_FRAME_FILTER   = BIT(13),
};

/**
 * dw1000_get_pcode() - Calculate preamble code
 *
 * @dw:			DW1000 device
 * @return:		Preamble code
 */
static unsigned int dw1000_get_pcode(struct dw1000_config *cfg)
{
	unsigned int pcode = cfg->pcode[cfg->prf];
	unsigned long pcodes;

	/* Use explicitly set preamble code if configured */
	pcodes = dw1000_channel_configs[cfg->channel].pcodes[cfg->prf];
	if (BIT(pcode) & pcodes)
		return pcode;

	/* Otherwise, use highest supported preamble code */
	return (fls(pcodes) - 1);
}

/**
 * dw1000_get_txpsr() - Calculate preamble symbol repetitions
 *
 * @dw:			DW1000 device
 * @return:		Preamble symbol repetitions
 */
static unsigned int dw1000_get_txpsr(struct dw1000_config *cfg)
{
	/* Use default for the current data rate */
	if (cfg->txpsr == DW1000_TXPSR_DEFAULT)
		return dw1000_rate_configs[cfg->rate].txpsr;

	return cfg->txpsr;
}

/**
 * dw1000_load_config() - Change radio parameters
 *
 * @dw:			DW1000 device
 * @changed:		Changed parameter bitmask
 * @return:		0 on success or -errno
 */
static int dw1000_load_config(struct dw1000 *dw)
{
	const struct dw1000_channel_config *channel_cfg;
	const struct dw1000_pcode_config *pcode_cfg;
	const struct dw1000_prf_config *prf_cfg;
	const struct dw1000_rate_config *rate_cfg;
	const struct dw1000_txpsr_config *txpsr_cfg;
	const struct dw1000_fixed_config *fixed_cfg;
	struct dw1000_config *cfg = &dw->cfg;
	unsigned int changed;
	unsigned int pcode;
	unsigned int txpsr;
	int value, mask;
	int rc = 0;

	/* Locked while working */
	mutex_lock(&cfg->mutex);

	INJECT_ERROR(dw, CONFIG_ERROR, rc, error);
	
	/* Changed configrations */
	changed = cfg->pending;

	/* Look up current configurations */
	txpsr = dw1000_get_txpsr(cfg);
	pcode = dw1000_get_pcode(cfg);
	pcode_cfg = &dw1000_pcode_configs[pcode];
	prf_cfg = &dw1000_prf_configs[cfg->prf];
	rate_cfg = &dw1000_rate_configs[cfg->rate];
	txpsr_cfg = &dw1000_txpsr_configs[txpsr];
	channel_cfg = &dw1000_channel_configs[cfg->channel];
	fixed_cfg = &dw1000_fixed_config;

	/* Set channel also in the 802.15.4 stack */
	dw->hw->phy->current_channel = cfg->channel;
	
	/* Calculate inter-frame spacing */
	dw->hw->phy->symbol_duration = (rate_cfg->tdsym_ns / 1000);
	if (!dw->hw->phy->symbol_duration)
		dw->hw->phy->symbol_duration = 1;
	dw->hw->phy->lifs_period =
		((IEEE802154_LIFS_PERIOD * rate_cfg->tdsym_ns) / 1000);
	dw->hw->phy->sifs_period =
		((IEEE802154_SIFS_PERIOD * rate_cfg->tdsym_ns) / 1000);

	/* If needed, set TX_POWER to default */
	if (((changed & DW1000_CONFIGURE_TX_POWER) == 0) &&
	    (changed & (DW1000_CONFIGURE_CHANNEL | DW1000_CONFIGURE_PRF |
			DW1000_CONFIGURE_SMART_POWER))) {
		if (cfg->smart_power) {
			cfg->txpwr[0] =	channel_cfg->tx_power[cfg->prf][0];
			cfg->txpwr[1] =	channel_cfg->tx_power[cfg->prf][1];
			cfg->txpwr[2] = channel_cfg->tx_power[cfg->prf][2];
			cfg->txpwr[3] =	channel_cfg->tx_power[cfg->prf][3];
		} else {
			cfg->txpwr[0] = 0;
			cfg->txpwr[1] = channel_cfg->tx_power[cfg->prf][0];
			cfg->txpwr[2] = channel_cfg->tx_power[cfg->prf][0];
			cfg->txpwr[3] = 0;
		}
		changed |= DW1000_CONFIGURE_TX_POWER;
	}

	/* SYS_CFG register */
	if (changed & (DW1000_CONFIGURE_RATE | DW1000_CONFIGURE_SMART_POWER)) {
		mask = (DW1000_SYS_CFG_DIS_STXP | DW1000_SYS_CFG_RXM110K);
		value = ((cfg->smart_power ? 0 : DW1000_SYS_CFG_DIS_STXP) |
			 ((cfg->rate == DW1000_RATE_110K) ?
			  DW1000_SYS_CFG_RXM110K : 0));
		if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0, mask,
					     value)) != 0)
			goto error;
	}

	/* TX_FCTRL registers */
	if (changed & DW1000_CONFIGURE_RATE) {
		mask = DW1000_TX_FCTRL1_TXBR_MASK;
		value = DW1000_TX_FCTRL1_TXBR(cfg->rate);
		if ((rc = regmap_update_bits(dw->tx_fctrl.regs,
					     DW1000_TX_FCTRL1,
					     mask, value)) != 0)
			goto error;
	}
	if (changed & (DW1000_CONFIGURE_PRF | DW1000_CONFIGURE_RATE |
		       DW1000_CONFIGURE_TXPSR)) {
		mask = (DW1000_TX_FCTRL2_TXPRF_MASK |
			DW1000_TX_FCTRL2_TXPSR_MASK);
		value = (DW1000_TX_FCTRL2_TXPRF(cfg->prf) |
			 DW1000_TX_FCTRL2_TXPSR(txpsr));
		if ((rc = regmap_update_bits(dw->tx_fctrl.regs,
					     DW1000_TX_FCTRL2,
					     mask, value)) != 0)
			goto error;
	}

	/* TX_ANTD register */
	if (changed & (DW1000_CONFIGURE_PRF | DW1000_CONFIGURE_ANTD)) {
		if ((rc = regmap_write(dw->tx_antd.regs, 0,
				       cfg->antd[cfg->prf])) != 0)
			goto error;
	}

	/* TX_POWER register */
	if (changed & DW1000_CONFIGURE_TX_POWER) {
		if ((rc = regmap_raw_write(dw->tx_power.regs, 0, cfg->txpwr,
					   sizeof(cfg->txpwr))) != 0)
			goto error;
	}

	/* CHAN_CTRL register */
	if (changed & (DW1000_CONFIGURE_CHANNEL |
		       DW1000_CONFIGURE_PCODE |
		       DW1000_CONFIGURE_PRF)) {
		mask = (DW1000_CHAN_CTRL_TX_CHAN_MASK |
			DW1000_CHAN_CTRL_RX_CHAN_MASK |
			DW1000_CHAN_CTRL_RXPRF_MASK |
			DW1000_CHAN_CTRL_TX_PCODE_MASK |
			DW1000_CHAN_CTRL_RX_PCODE_MASK);
		value = (DW1000_CHAN_CTRL_TX_CHAN(cfg->channel) |
			 DW1000_CHAN_CTRL_RX_CHAN(cfg->channel) |
			 DW1000_CHAN_CTRL_RXPRF(cfg->prf) |
			 DW1000_CHAN_CTRL_TX_PCODE(pcode) |
			 DW1000_CHAN_CTRL_RX_PCODE(pcode));
		if ((rc = regmap_update_bits(dw->chan_ctrl.regs, 0,
					     mask, value)) != 0)
			goto error;
	}

	/* AGC_CTRL registers */
	if (changed & DW1000_CONFIGURE_PRF) {
		if ((rc = regmap_raw_write(dw->agc_ctrl.regs,
					   DW1000_AGC_TUNE1,
					   prf_cfg->agc_tune1,
					   sizeof(prf_cfg->agc_tune1))) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_FIXED) {
		if ((rc = regmap_raw_write(dw->agc_ctrl.regs,
					   DW1000_AGC_TUNE2,
					   fixed_cfg->agc_tune2,
					   sizeof(fixed_cfg->agc_tune2))) != 0)
			goto error;
		if ((rc = regmap_raw_write(dw->agc_ctrl.regs,
					   DW1000_AGC_TUNE3,
					   fixed_cfg->agc_tune3,
					   sizeof(fixed_cfg->agc_tune3))) != 0)
			goto error;
	}

	/* DRX_CONF registers */
	if (changed & DW1000_CONFIGURE_RATE) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE0B,
				       rate_cfg->drx_tune0b)) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_PRF) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE1A,
				       prf_cfg->drx_tune1a)) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_TXPSR) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE1B,
				       txpsr_cfg->drx_tune1b)) != 0)
			goto error;
	}
	if (changed & (DW1000_CONFIGURE_PRF | DW1000_CONFIGURE_RATE)) {
		if ((rc = regmap_raw_write(dw->drx_conf.regs, DW1000_DRX_TUNE2,
				rate_cfg->drx_tune2[cfg->prf],
				sizeof(rate_cfg->drx_tune2[cfg->prf]))) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_RATE) {
		if ((rc = regmap_write(dw->drx_conf.regs, DW1000_DRX_TUNE4H,
				       txpsr_cfg->drx_tune4h)) != 0)
			goto error;
	}

	/* RF_CONF registers */
	if (changed & DW1000_CONFIGURE_CHANNEL) {
		if ((rc = regmap_raw_write(dw->rf_conf.regs, DW1000_RF_TXCTRL,
					   channel_cfg->rf_txctrl,
					   sizeof(channel_cfg->rf_txctrl))) != 0)
			goto error;
		if ((rc = regmap_write(dw->rf_conf.regs, DW1000_RF_RXCTRLH,
				       channel_cfg->rf_rxctrlh)) != 0)
			goto error;
	}

	/* TX_CAL registers */
	if (changed & DW1000_CONFIGURE_CHANNEL) {
		if ((rc = regmap_write(dw->tx_cal.regs, DW1000_TC_PGDELAY,
				       channel_cfg->tc_pgdelay)) != 0)
			goto error;
	}

	/* FS_CTRL registers */
	if (changed & DW1000_CONFIGURE_CHANNEL) {
		if ((rc = regmap_raw_write(dw->fs_ctrl.regs, DW1000_FS_PLLCFG,
					   channel_cfg->fs_pllcfg,
					   sizeof(channel_cfg->fs_pllcfg))) != 0)
			goto error;
		if ((rc = regmap_write(dw->fs_ctrl.regs, DW1000_FS_PLLTUNE,
				       channel_cfg->fs_plltune)) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_XTAL_TRIM) {
		if ((rc = regmap_update_bits(dw->fs_ctrl.regs, DW1000_FS_XTALT,
					     DW1000_FS_XTALT_XTALT_MASK,
					     cfg->xtalt)) != 0)
			goto error;
	}

	/* LDE_IF registers */
	if (changed & (DW1000_CONFIGURE_PRF | DW1000_CONFIGURE_ANTD)) {
		uint16_t lde_rxantd = cpu_to_le16(cfg->antd[cfg->prf]);
		if ((rc = regmap_raw_write(dw->lde_if.regs,
					   DW1000_LDE_RXANTD, &lde_rxantd,
					   sizeof(lde_rxantd))) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_PRF) {
		if ((rc = regmap_raw_write(dw->lde_if.regs,
					   DW1000_LDE_CFG2, prf_cfg->lde_cfg2,
					   sizeof(prf_cfg->lde_cfg2))) != 0)
			goto error;
	}
	if (changed & (DW1000_CONFIGURE_PCODE | DW1000_CONFIGURE_RATE)) {
		uint16_t lde_repc = pcode_cfg->lde_repc;
		if (cfg->rate == DW1000_RATE_110K)
			lde_repc >>= 3;
		cpu_to_le16s(&lde_repc);
		if ((rc = regmap_raw_write(dw->lde_if.regs,
					   DW1000_LDE_REPC, &lde_repc,
					   sizeof(lde_repc))) != 0)
			goto error;
	}

	/* IEEE 802.15.4 stack config */
	if (changed & DW1000_CONFIGURE_PAN_ID) {
		if ((rc = regmap_write(dw->panadr.regs,
				       DW1000_PANADR_PAN_ID,
				       cfg->pan_id)) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_SHORT_ADDR) {
		if ((rc = regmap_write(dw->panadr.regs,
				       DW1000_PANADR_SHORT_ADDR,
				       cfg->short_addr)) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_IEEE_ADDR) {
		if ((rc = regmap_raw_write(dw->eui.regs, 0,
					   &cfg->ieee_addr,
					   sizeof(cfg->ieee_addr))) != 0)
			goto error;
	}
	if (changed & DW1000_CONFIGURE_FRAME_FILTER) {
		if ((rc = regmap_update_bits(dw->sys_cfg.regs, 0,
					     DW1000_SYS_CFG_FF,
					     cfg->frame_filter)) != 0)
			goto error;
	}

	/* Mark all done */
	cfg->pending = 0;
	
 error:
	/* Record return value */
	cfg->rc = rc;

	/* Unlock configuration */
	mutex_unlock(&cfg->mutex);
	
	return rc;
}

/**
 * dw1000_reconfigure() - Reconfigure radio parameters
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static inline int dw1000_reconfigure(struct dw1000 *dw)
{
	struct dw1000_config *cfg = &dw->cfg;

	/* Wait until work done */
	dw1000_enqueue_wait(dw, DW1000_CONFIG_WORK);

	return cfg->rc;
}

/**
 * dw1000_configure_xtalt() - Configure XTAL trim
 *
 * @dw:			DW1000 device
 * @trim:		Trim value
 * @return:		0 on success or -errno
 */
static int dw1000_configure_xtalt(struct dw1000 *dw, unsigned int trim)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Check that trim value is supported */
	if (trim & ~DW1000_FS_XTALT_XTALT_MASK)
		return -EINVAL;

	/* Assign new trim value */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_XTAL_TRIM;
	cfg->xtalt = trim;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set xtal trim %d\n", trim);
	return rc;
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
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Check that channel is supported */
	if (!(BIT(channel) & DW1000_CHANNELS))
		return -EINVAL;

	/* Record channel */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_CHANNEL;
	cfg->channel = channel;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set channel %d\n", channel);
	return rc;
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
	struct dw1000_config *cfg = &dw->cfg;
	unsigned long pcodes;
	int rc;
	
	/* Lock config */
	mutex_lock(&cfg->mutex);

	/* Check that preamble code is valid */
	if (pcode) {
		pcodes = dw1000_channel_configs[cfg->channel].pcodes[cfg->prf];
		if (!(BIT(pcode) & pcodes)) {
			mutex_unlock(&cfg->mutex);
			return -EINVAL;
		}
	}

	/* Record preamble code */
	cfg->pending |= DW1000_CONFIGURE_PCODE;
	cfg->pcode[cfg->prf] = pcode;
	
	/* Unlock configuration */
	mutex_unlock(&cfg->mutex);

	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set preamble code %d\n", pcode);
	return rc;
}

/**
 * dw1000_configure_prf() - Configure pulse repetition frequency
 *
 * @dw:			DW1000 device
 * @prf:		Pulse repetition frequency
 * @return:		0 on success or -errno
 */
static int dw1000_configure_prf(struct dw1000 *dw, unsigned prf)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Record pulse repetition frequency */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_PRF;
	cfg->prf = prf;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set pulse repetition frequency %dMHz\n",
		dw1000_prfs[prf]);
	return rc;
}

/**
 * dw1000_configure_antd() - Configure antenna delay
 *
 * @dw:			DW1000 device
 * @antd:		Antenna delay
 * @return:		0 on success or -errno
 */
static int dw1000_configure_antd(struct dw1000 *dw, unsigned int delay)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Check value range */
	if (delay & ~DW1000_ANTD_MASK)
		return -EINVAL;

	/* Record new antenna delay */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_ANTD;
	cfg->antd[cfg->prf] = delay;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set antenna delay 0x%4x\n", delay);
	return rc;
}

/**
 * dw1000_configure_rate() - Configure data rate
 *
 * @dw:			DW1000 device
 * @rate:		Data rate
 * @return:		0 on success or -errno
 */
static int dw1000_configure_rate(struct dw1000 *dw, unsigned int rate)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Record data rate */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_RATE;
	cfg->rate = rate;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set data rate %dkbps\n", dw1000_rates[rate]);
	return rc;
}

/**
 * dw1000_configure_txpsr() - Configure transmit preamble symbol repetitions
 *
 * @dw:			DW1000 device
 * @txpsr:		Transmit preamble symbol repetitions
 * @return:		0 on success or -errno
 */
static int dw1000_configure_txpsr(struct dw1000 *dw, unsigned int txpsr)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Record transmit preamble symbol repetitions */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_TXPSR;
	cfg->txpsr = txpsr;
	mutex_unlock(&cfg->mutex);

	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set %d preamble symbol repetitions\n",
		dw1000_txpsrs[dw1000_get_txpsr(cfg)]);
	return rc;
}

/**
 * dw1000_configure_tx_power() - Configure tx power control
 *
 * @dw:			DW1000 device
 * @power:		Tx power level
 * @return:		0 on success or -errno
 */
static int dw1000_configure_tx_power(struct dw1000 *dw, uint32_t power)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Lock configuration */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_TX_POWER;

	/* Power levels */
	if (power & 0xffffff00) {
		/* Four byte power level */
		cfg->txpwr[0] = (power >>  0) & 0xff;
		cfg->txpwr[1] = (power >>  8) & 0xff;
		cfg->txpwr[2] = (power >> 16) & 0xff;
		cfg->txpwr[3] = (power >> 24) & 0xff;
	} else {
		/* One byte power level */
		cfg->txpwr[0] = power;
		cfg->txpwr[1] = power;
		cfg->txpwr[2] = power;
		cfg->txpwr[3] = power;
	}
	
	/* Unlock configuration */
	mutex_unlock(&cfg->mutex);

	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);
	
	dev_dbg(dw->dev, "set tx power 0x%08x\n", power);
	return rc;
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
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Record smart power control */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_SMART_POWER;
	cfg->smart_power = smart_power;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set smart power control %s\n",
		(smart_power ? "on" : "off"));
	return rc;
}

/**
 * dw1000_configure_frame_filter() - Configure hardware frame filter
 *
 * @dw:			DW1000 device
 * @filter:		Filter bitmap
 * @return:		0 on success or -errno
 */
static int dw1000_configure_frame_filter(struct dw1000 *dw, unsigned int filter)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Check that filter is supported */
	if ((filter & DW1000_SYS_CFG_FF) != filter)
		return -EINVAL;

	/* Record filter */
	mutex_lock(&cfg->mutex);
	cfg->pending |= DW1000_CONFIGURE_FRAME_FILTER;
	cfg->frame_filter = filter;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "set frame filter 0x%04x\n", filter);
	return rc;
}

/**
 * dw1000_configure_profile() - Configure calibration profile
 *
 * @dw:			DW1000 device
 * @profile:		Calibration profile name
 * @return:		0 on success or -errno
 */
static int dw1000_configure_profile(struct dw1000 *dw, const char *profile)
{
	struct dw1000_config *cfg = &dw->cfg;
	struct dw1000_calib *calib = NULL;
	int rc;
	int i;

	/* Find a matching profile */
	for (i=0; i<DW1000_MAX_CALIBS; i++) {
		if (!strcmp(dw->calib[i].id, profile)) {
			calib = &dw->calib[i];
			break;
		}
	}
	if (!calib) {
		dev_warn(dw->dev, "calibration profile \"%s\" not found\n",
			 profile);
		return -EINVAL;
	}

	dev_info(dw->dev, "loading calibration profile \"%s\"\n", profile);

	/* Lock configuration */
	mutex_lock(&cfg->mutex);

	/* Set changed parameters */
	cfg->pending |=
		DW1000_CONFIGURE_CHANNEL |
		DW1000_CONFIGURE_PRF |
		DW1000_CONFIGURE_ANTD |
		DW1000_CONFIGURE_TX_POWER |
		DW1000_CONFIGURE_SMART_POWER;

	/* Set channel in the 802.15.4 stack */
	dw->hw->phy->current_channel = calib->ch;
	
	/* Record values */
	cfg->channel = calib->ch;
	cfg->prf = calib->prf;
	cfg->antd[cfg->prf] = calib->antd;
	cfg->txpwr[0] = (calib->power >>  0) & 0xff;
	cfg->txpwr[1] = (calib->power >>  8) & 0xff;
	cfg->txpwr[2] = (calib->power >> 16) & 0xff;
	cfg->txpwr[3] = (calib->power >> 24) & 0xff;
	cfg->smart_power = !!(calib->power & 0xff000000);

	/* Unlock configuration */
	mutex_unlock(&cfg->mutex);

	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);
	
	return rc;
}


/******************************************************************************
 *
 * SAR ADC access
 *
 */

/**
 * dw1000_sar_init() - Prepare SAR SPI message
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_sar_init(struct dw1000 *dw)
{
	struct dw1000_sar *sar = &dw->sar;

	/* Initialise SAR descriptor */
	memset(sar, 0, sizeof(*sar));

	/* Set contant values used in SPI transfer */
	sar->ctrl_a1  = DW1000_RF_SENSOR_SARC_A1;
	sar->ctrl_b1  = DW1000_RF_SENSOR_SARC_B1;
	sar->ctrl_b2  = DW1000_RF_SENSOR_SARC_B2;
	sar->ctrl_ena = DW1000_TC_SARC_CTRL;
	sar->ctrl_dis = 0;

	/* Prepare SAR SPI message */
	spi_message_init_no_memset(&sar->spi_msg);
	dw1000_init_write(&sar->spi_msg, &sar->spi_sarc_a1,
			  DW1000_RF_CONF, DW1000_RF_SENSOR_SARC_A,
			  &sar->ctrl_a1, sizeof(sar->ctrl_a1));
	dw1000_init_write(&sar->spi_msg, &sar->spi_sarc_b1,
			  DW1000_RF_CONF, DW1000_RF_SENSOR_SARC_B,
			  &sar->ctrl_b1, sizeof(sar->ctrl_b1));
	dw1000_init_write(&sar->spi_msg, &sar->spi_sarc_b2,
			  DW1000_RF_CONF, DW1000_RF_SENSOR_SARC_B,
			  &sar->ctrl_b2, sizeof(sar->ctrl_b2));
	dw1000_init_write(&sar->spi_msg, &sar->spi_sarc_ena,
			  DW1000_TX_CAL, DW1000_TC_SARC,
			  &sar->ctrl_ena, sizeof(sar->ctrl_ena));

	/* Keep the ADC on for a few us */
	sar->spi_sarc_ena.data.delay_usecs = DW1000_SAR_WAIT_US;

	dw1000_init_write(&sar->spi_msg, &sar->spi_sarc_dis,
			  DW1000_TX_CAL, DW1000_TC_SARC,
			  &sar->ctrl_dis, sizeof(sar->ctrl_dis));
	dw1000_init_read(&sar->spi_msg, &sar->spi_sarl,
			 DW1000_TX_CAL, DW1000_TC_SARL,
			 &sar->adc.raw, sizeof(sar->adc.raw));

	return 0;
}

/**
 * dw1000_sar_read() - Read SAR ADC values
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_sar_read(struct dw1000 *dw)
{
	struct dw1000_sar *sar = &dw->sar;
	int rc;

	/* Send SPI message */
	if ((rc = spi_sync(dw->spi, &sar->spi_msg)) != 0) {
		dev_dbg(dw->dev, "ADC SAR read failed: %d\n", rc);
		return rc;
	}

	return 0;
}

/**
 * dw1000_sar_get_temp(struct dw1000 *dw) - Read SAR temperature reading
 *
 * @dw:			DW1000 device
 * @return:		Temperature in millidegrees Celcius [mC]
 */
static inline int dw1000_sar_get_temp(struct dw1000 *dw)
{
	return (100000 * ((int)dw->sar.adc.temp - (int)dw->tcalib_23c))
		/ 114 + 23000;
}

/**
 * dw1000_sar_get_volt(struct dw1000 *dw) - Read SAR voltage reading
 *
 * @dw:			DW1000 device
 * @return:		Voltage in millivolts [mV]
 */
static inline int dw1000_sar_get_volt(struct dw1000 *dw)
{
	return (1000 * ((int)dw->sar.adc.volt - (int)dw->vcalib_3v3))
		/ 173 + 3300;
}


/******************************************************************************
 *
 * PTP hardware clock
 *
 */

/**
 * dw1000_ptp_cc_read() - Read underlying cycle counter
 *
 * @cc:			Cycle counter
 * @return:		Cycle count
 */
static uint64_t dw1000_ptp_cc_read(const struct hires_counter *tc)
{
	struct dw1000 *dw = container_of(tc, struct dw1000, ptp.tc);
	union dw1000_timestamp time;
	int rc;

	/* The documentation does not state whether or not reads from
	 * the SYS_TIME register are atomic.  At the rated 20MHz SPI
	 * bus speed, reading all 5 bytes will take 2us.  During this
	 * time the SYS_TIME value should have been incremented (in
	 * units of 512) around 250 times.
	 *
	 * We assume that Decawave has taken this problem into account
	 * and caused SYS_TIME reads to return a snapshot of the
	 * complete 40-bit timer at the point that the read was
	 * initiated.
	 *
	 * Use a low-level read to avoid any potential for the regmap
	 * abstraction layer deciding to split the read into multiple
	 * SPI transactions.
	 */
	time.cc = 0;
	if ((rc = dw1000_read(dw, DW1000_SYS_TIME, 0, &time.raw,
			      sizeof(time.raw))) != 0) {
		dev_err(dw->dev, "could not read timestamp: %d\n", rc);
		/* There is no way to indicate failure here */
		return 0;
	}

	return le64_to_cpu(time.cc);
}

/**
 * dw1000_ptp_adjfine() - Adjust frequency of hardware clock
 *
 * @ptp:		PTP clock information
 * @delta:		Frequency offset in ppm + 16bit fraction
 * @return:		0 on success or -errno
 */
static int dw1000_ptp_adjfine(struct ptp_clock_info *ptp_info, long delta)
{
	struct dw1000 *dw = container_of(ptp_info, struct dw1000, ptp.info);
	struct dw1000_ptp *ptp = &dw->ptp;
	uint64_t mult;

	/* Calculate multiplier value */
	mult = (delta < 0) ? -delta : delta;
	/* mult *= DW1000_CYCLECOUNTER_MULT / 65536000000ULL; */
	mult = (mult * (DW1000_CYCLECOUNTER_MULT / 128000000ULL)) >> 9;
	mult = (delta < 0) ? DW1000_CYCLECOUNTER_MULT - mult:
			     DW1000_CYCLECOUNTER_MULT + mult;

	/* Adjust counter multiplier */
	mutex_lock(&ptp->mutex);
	hires_counter_setmult(&ptp->tc, mult);
	mutex_unlock(&ptp->mutex);

	dev_dbg(dw->dev, "adjust frequency %+ld sppm: multiplier adj %llu\n",
		delta, (unsigned long long) mult);
	return 0;
}

/**
 * dw1000_ptp_adjtime() - Adjust time of hardware clock
 *
 * @ptp:		PTP clock information
 * @delta:		Change in nanoseconds
 * @return:		0 on success or -errno
 */
static int dw1000_ptp_adjtime(struct ptp_clock_info *ptp_info, int64_t delta)
{
	struct dw1000 *dw = container_of(ptp_info, struct dw1000, ptp.info);
	struct dw1000_ptp *ptp = &dw->ptp;
	struct timehires hdelta = { delta, 0, };

	/* Adjust timecounter */
	mutex_lock(&ptp->mutex);
	hires_counter_adjtime(&ptp->tc, hdelta);
	mutex_unlock(&ptp->mutex);

	dev_dbg(dw->dev, "adjust time %+lld ns\n", delta);
	return 0;
}

/**
 * dw1000_ptp_gettime() - Get time of hardware clock
 *
 * @ptp:		PTP clock information
 * @ts:			Time to fill in
 * @return:		0 on sucess or -errno
 */
static int dw1000_ptp_gettime(struct ptp_clock_info *ptp_info,
			      struct timespec64 *ts)
{
	struct dw1000 *dw = container_of(ptp_info, struct dw1000, ptp.info);
	struct dw1000_ptp *ptp = &dw->ptp;
	struct timehires time;

	/* Read timecounter */
	mutex_lock(&ptp->mutex);
	time = hires_counter_cyc2time(&ptp->tc, hires_counter_read(&ptp->tc));
	mutex_unlock(&ptp->mutex);

	/* Convert to timespec */
	*ts = ns_to_timespec64(time.tv_nsec);

	return 0;
}

/**
 * dw1000_ptp_settime() - Set time of hardware clock
 *
 * @ptp:		PTP clock information
 * @ts:			Time
 * @return:		0 on sucess or -errno
 */
static int dw1000_ptp_settime(struct ptp_clock_info *ptp_info,
			      const struct timespec64 *ts)
{
	struct dw1000 *dw = container_of(ptp_info, struct dw1000, ptp.info);
	struct dw1000_ptp *ptp = &dw->ptp;
	struct timehires time;

	/* Convert to nanoseconds */
	time.tv_nsec = timespec64_to_ns(ts);
	time.tv_frac = 0;

	/* Reset timecounter */
	mutex_lock(&ptp->mutex);
	hires_counter_settime(&ptp->tc, time);
	mutex_unlock(&ptp->mutex);

	dev_dbg(dw->dev, "set time %llu ns\n", time.tv_nsec);
	return 0;
}

/**
 * dw1000_ptp_enable() - Enable/disable feature
 *
 * @ptp:		PTP clock information
 * @request:		Feature request
 * @on:			Enable/disable feature
 * @return:		0 on success or -errno
 */
static int dw1000_ptp_enable(struct ptp_clock_info *ptp,
			     struct ptp_clock_request *request, int on)
{
	/* No additional features supported */
	return -EOPNOTSUPP;
}

/**
 * dw1000_ptp_sync() - Sync clock source
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_ptp_sync(struct dw1000 *dw)
{
	struct dw1000_ptp *ptp = &dw->ptp;

	/* Synchronise counter. This must be done at least twice 
	 * per wraparound ~17s. We try to do it 5 times.
	 */
	mutex_lock(&ptp->mutex);
	hires_counter_sync(&ptp->tc);
	mod_timer(&ptp->timer, jiffies +
		  msecs_to_jiffies(DW1000_PTP_SYNC_DELAY));
	mutex_unlock(&ptp->mutex);

	return 0;
}

/**
 * dw1000_ptp_reset() - Reset clock source
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_ptp_reset(struct dw1000 *dw)
{
	struct dw1000_ptp *ptp = &dw->ptp;
	struct timehires time;

	/* Counter starting time */
	time.tv_nsec = ktime_to_ns(ktime_get_real());
	time.tv_frac = 0;
	
	/* Reset timecounter */
	mutex_lock(&ptp->mutex);
	hires_counter_settime(&ptp->tc, time);
	mutex_unlock(&ptp->mutex);

	return 0;
}

/**
 * dw1000_ptp_init() - Initialise PTP clock
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_ptp_init(struct dw1000 *dw)
{
	struct dw1000_ptp *ptp = &dw->ptp;
	struct ptp_clock_info *info = &ptp->info;
	struct timehires start;
	int rc;
	
	/* Clear memory */
	memset(ptp, 0, sizeof(*ptp));
	
	/* Initialise mutex */
	mutex_init(&ptp->mutex);

	/* Counter starting time */
	start.tv_nsec = ktime_to_ns(ktime_get_real());
	start.tv_frac = 0;

	/* Initialise time counter */
	hires_counter_init(&ptp->tc, dw1000_ptp_cc_read, start,
			   DW1000_CYCLECOUNTER_SIZE, DW1000_CYCLECOUNTER_MULT);

	/* Setup timer */
	timer_setup(&ptp->timer, dw1000_ptp_timer_handler, 0);

	/* Initialise PTP clock information */
	info->owner = THIS_MODULE;
	info->max_adj = DW1000_PTP_MAX_ADJ;
	snprintf(info->name, sizeof(info->name), "%s", dev_name(dw->dev));
	info->adjfine = dw1000_ptp_adjfine;
	info->adjtime = dw1000_ptp_adjtime;
	info->gettime64 = dw1000_ptp_gettime;
	info->settime64 = dw1000_ptp_settime;
	info->enable = dw1000_ptp_enable;

	/* Register PTP clock */
	ptp->clock = ptp_clock_register(&ptp->info, dw->dev);
	if (IS_ERR(ptp->clock)) {
		rc = PTR_ERR(ptp->clock);
		return rc;
	}

	/* Start timer */
	mod_timer(&ptp->timer, jiffies +
		  msecs_to_jiffies(DW1000_PTP_SYNC_DELAY));

	return 0;
}

/**
 * dw1000_ptp_remove() - Remove PTP clock
 *
 * @dw:			DW1000 device
 */
static void dw1000_ptp_remove(struct dw1000 *dw)
{
	struct dw1000_ptp *ptp = &dw->ptp;

	/* Unregister */
	if (ptp->clock) {
		del_timer_sync(&ptp->timer);
		ptp_clock_unregister(ptp->clock);
	}
	ptp->clock = NULL;
}

	
/**
 * dw1000_ptp_timestamp() - Convert DW1000 timestamp to kernel timestamp
 *
 * @dw:			DW1000 device
 * @time:		DW1000 timestamp
 * @hwtstamps:		Kernel hardware timestamp
 */
static void dw1000_ptp_timestamp(struct dw1000 *dw,
				 const union dw1000_timestamp *time,
				 struct skb_shared_hwtstamps *hwtstamps)
{
	struct dw1000_ptp *ptp = &dw->ptp;
	struct timehires stamp;
	cycle_t cc;

	/* Get 5-byte timestamp */
	cc = le64_to_cpu(time->cc) & DW1000_TIMESTAMP_MASK;

	/* Convert to nanoseconds */
	mutex_lock(&ptp->mutex);
	stamp = hires_counter_cyc2time(&ptp->tc, cc);
	mutex_unlock(&ptp->mutex);

	/* Fill in kernel hardware timestamp */
	memset(hwtstamps, 0, sizeof(*hwtstamps));
	hwtstamps->hwtstamp = ns_to_ktime(stamp.tv_nsec);
#ifdef HAVE_HWTSFRAC
	hwtstamps->hwtsfrac = ns_to_ktime_frac(stamp.tv_frac);
#endif
}

/**
 * dw1000_ptp_tsinfo() - Convert DW1000 tsinfo to kernel timestamp
 *
 * @dw:			DW1000 device
 * @time:		DW1000 timestamp
 * @tsinfo:		DW1000 tsinfo
 * @hwtstamps:		Kernel hardware timestamp
 */
static void dw1000_ptp_tsinfo(struct dw1000 *dw,
			      const union dw1000_timestamp *time,
			      struct dw1000_tsinfo *tsinfo,
			      struct skb_shared_hwtstamps *hwtstamps)
{
	struct dw1000_ptp *ptp = &dw->ptp;
	
	/* Voltage and temperature KPIs */
	if (dw1000_tsinfo_ena & DW1000_TSINFO_SADC) {
		tsinfo->temp = dw1000_sar_get_temp(dw) / 10;
		tsinfo->volt = dw1000_sar_get_volt(dw);
	}

	/* Raw timestamp enabled */
	if (dw1000_tsinfo_ena & DW1000_TSINFO_TIME) {
		cycle_t rawts, cc;

		/* Get 5-byte timestamp */
		cc = le64_to_cpu(time->cc) & DW1000_TIMESTAMP_MASK;

		/* Convert to raw monotonic ticks */
		mutex_lock(&ptp->mutex);
		rawts = hires_counter_cyc2raw(&ptp->tc, cc);
		mutex_unlock(&ptp->mutex);

		/* Assign raw timestamp to tsinfo */
		tsinfo->rawts = rawts;
	}

#ifdef HAVE_HWTSINFO
	/* If any flags enabled, copy the whole structure */
	if (dw1000_tsinfo_ena)
		memcpy(hwtstamps->hwtsinfo, tsinfo, sizeof(*tsinfo));
#endif
}


/******************************************************************************
 *
 * Transmit datapath
 *
 */

/**
 * dw1000_tx_init() - Initialise Tx descriptor
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_tx_init(struct dw1000 *dw)
{
	struct dw1000_tx *tx = &dw->tx;

	/* Initialise transmit descriptor */
	memset(tx, 0, sizeof(*tx));

	/* Init lock */
	spin_lock_init(&tx->lock);
	
	/* Set constant values used in SPI transfer */
	tx->trxoff = DW1000_SYS_CTRL0_TRXOFF;
	tx->txstrt = DW1000_SYS_CTRL0_TXSTRT;
	tx->txmask = cpu_to_le32(DW1000_SYS_MASK_TX);
	tx->rxmask = cpu_to_le32(DW1000_SYS_MASK_RX);
	tx->rxena  = DW1000_SYS_CTRL1_RXENAB;
	tx->txfrs  = DW1000_SYS_STATUS0_TXFRB | DW1000_SYS_STATUS0_TXPRS |
		     DW1000_SYS_STATUS0_TXPHS | DW1000_SYS_STATUS0_TXFRS;

	/* Prepare data SPI message */
	spi_message_init_no_memset(&tx->spi_data);
	dw1000_init_write(&tx->spi_data, &tx->spi_txmask, DW1000_SYS_MASK,
			  0, &tx->txmask, sizeof(tx->txmask));
	dw1000_init_write(&tx->spi_data, &tx->spi_buffer, DW1000_TX_BUFFER, 0,
			  NULL, 0);
	dw1000_init_write(&tx->spi_data, &tx->spi_fctrl, DW1000_TX_FCTRL,
			  DW1000_TX_FCTRL0, &tx->len, sizeof(tx->len));
	dw1000_init_write(&tx->spi_data, &tx->spi_trxoff, DW1000_SYS_CTRL,
			  DW1000_SYS_CTRL0, &tx->trxoff, sizeof(tx->trxoff));
	dw1000_init_write(&tx->spi_data, &tx->spi_txstrt, DW1000_SYS_CTRL,
			  DW1000_SYS_CTRL0, &tx->txstrt, sizeof(tx->txstrt));

	/* Prepare information SPI message */
	spi_message_init_no_memset(&tx->spi_info);
	dw1000_init_write(&tx->spi_info, &tx->spi_status, DW1000_SYS_STATUS,
			  DW1000_SYS_STATUS0, &tx->txfrs, sizeof(tx->txfrs));
	dw1000_init_read(&tx->spi_info, &tx->spi_txtime, DW1000_TX_TIME,
			 DW1000_TX_STAMP, &tx->time.raw, sizeof(tx->time.raw));
	dw1000_init_write(&tx->spi_info, &tx->spi_rxmask, DW1000_SYS_MASK,
			  0, &tx->rxmask, sizeof(tx->rxmask));
	dw1000_init_write(&tx->spi_info, &tx->spi_rxena, DW1000_SYS_CTRL,
                          DW1000_SYS_CTRL1, &tx->rxena, sizeof(tx->rxena));

	return 0;
}

/**
 * dw1000_tx_complete() - Complete transmission
 *
 * @dw:			DW1000 device
 */
static void dw1000_tx_complete(struct dw1000 *dw)
{
	struct dw1000_tx *tx = &dw->tx;
	struct sk_buff *skb;
	unsigned long flags;
	
	/* Release skb */
	spin_lock_irqsave(&tx->lock, flags);
	skb = tx->skb;
	tx->skb = NULL;
	spin_unlock_irqrestore(&tx->lock, flags);
	
	/* Reset tx retries */
	tx->retries = 0;
	
	/* Report completion to IEEE 802.15.4 stack */
	if (skb)
		ieee802154_xmit_complete(dw->hw, skb,
					 (skb->len <= DW1000_MAX_SKB_LEN));
}

/**
 * dw1000_tx_frame_start() - Start packet transmit
 *
 * @hw:			IEEE 802.15.4 device
 * @skb:		Socket buffer
 * @return:		0 on success or -errno
 */
static void dw1000_tx_frame_start(struct dw1000 *dw)
{
	struct dw1000_tx *tx = &dw->tx;
	struct sk_buff *skb;
	unsigned long flags;

	/* Acquire TX lock, get skb */
	spin_lock_irqsave(&tx->lock, flags);
	skb = tx->skb;
	spin_unlock_irqrestore(&tx->lock, flags);

	/* Sanity check */
	if (!skb)
		return;

	/* Transmission in progress */
	dw1000_set_state_timeout(dw, DW1000_STATE_TX, DW1000_TX_TIMEOUT);
	
	/* Update data SPI message */
	tx->spi_buffer.data.tx_buf = skb->data;
	tx->spi_buffer.data.len = skb->len;

	/* Adjust frame length */
	tx->len = DW1000_TX_FCTRL0_TFLEN(skb->len + IEEE802154_FCS_LEN);

	/* Indicate timestamping */
	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	INJECT_ERROR(dw, TX_DELAY);

	/* Data transfer SPI message */
	if ((spi_sync(dw->spi, &tx->spi_data)) != 0) {
		dw1000_enqueue(dw, DW1000_ERROR_WORK);
		goto err_spi;
	}

	INJECT_ERROR(dw, TX_SPI_DATA, err_spi);

	/* Set software timestamp close to TXSTRT */
	skb_tx_timestamp(skb);

	return;
	
 err_spi:
	/* Statistics */
	dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
	dw1000_stats_inc(dw, DW1000_STATS_TX_ERROR);
	
	/* Discard skb */
	dw1000_tx_complete(dw);
}

/**
 * dw1000_tx_frame_sent() - Handle Transmit Frame Sent (TXFRS) event
 *
 * @dw:			DW1000 device
 */
static void dw1000_tx_frame_sent(struct dw1000 *dw)
{
	struct dw1000_tx *tx = &dw->tx;
	struct dw1000_tsinfo *tsi = &tx->tsinfo;
	struct skb_shared_hwtstamps hwtstamps;
	struct sk_buff *skb;
	unsigned long flags;
	int rc;

	/* Get skb */
	spin_lock_irqsave(&tx->lock, flags);
	skb = tx->skb;
	spin_unlock_irqrestore(&tx->lock, flags);

	/* sanity check */
	if (!skb)
		return;
	
	/* Information SPI message */
	if ((rc = spi_sync(dw->spi, &tx->spi_info)) != 0) {
		dw1000_enqueue(dw, DW1000_ERROR_WORK);
		goto err_spi;
	}

	INJECT_ERROR(dw, TX_SPI_INFO, err_spi);

	/* Record hardware timestamp, if applicable */
	if (skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS) {
		dw1000_ptp_timestamp(dw, &tx->time, &hwtstamps);
		dw1000_ptp_tsinfo(dw, &tx->time, tsi, &hwtstamps);
		skb_tstamp_tx(skb, &hwtstamps);
	}

	/* skb completed */
	dw1000_tx_complete(dw);

	/* Statistics */
	dw1000_stats_inc(dw, DW1000_STATS_TX_FRAME);
	
	/* Return to RX */
	dw1000_set_state(dw, DW1000_STATE_RX);
	return;
	
 err_spi:
	/* Statistics */
	dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
	dw1000_stats_inc(dw, DW1000_STATS_TX_ERROR);
	
	/* Discard skb */
	dw1000_tx_complete(dw);
}

/**
 * dw1000_tx_error() - Handle Transmit failure
 *
 * @dw:			DW1000 device
 */
static void dw1000_tx_error(struct dw1000 *dw)
{
	struct dw1000_tx *tx = &dw->tx;

	/* Recovery action */
	dw1000_disable(dw);

	/* Increment retry count */
	tx->retries++;

	/* Retry a few times */
	if (tx->retries < DW1000_TX_MAX_RETRIES) {
		dw1000_tx_frame_start(dw);
		dw1000_stats_inc(dw, DW1000_STATS_TX_RETRY);
	} else {
		dw1000_tx_complete(dw);
		dw1000_stats_inc(dw, DW1000_STATS_TX_ERROR);
	}
}


/******************************************************************************
 *
 * Receive datapath
 *
 */

/**
 * dw1000_rx_init() - Prepare receive SPI messages
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_rx_init(struct dw1000 *dw)
{
	struct dw1000_rx *rx = &dw->rx;

	/* Initialise receive descriptor */
	memset(rx, 0, sizeof(*rx));

	/* Set contant values used in SPI transfers */
	rx->hrbpt = DW1000_SYS_CTRL3_HRBPT;
	rx->mask0 = cpu_to_le32(DW1000_SYS_MASK_IDLE);
	rx->mask1 = cpu_to_le32(DW1000_SYS_MASK_RX);
	rx->clear = cpu_to_le32(DW1000_SYS_STATUS_DBLBUF);
	rx->reset = cpu_to_le32(DW1000_SYS_STATUS_TRX);

	/* Prepare information SPI message */
	spi_message_init(&rx->spi_info);
	dw1000_init_read(&rx->spi_info, &rx->spi_finfo, DW1000_RX_FINFO, 0,
			 &rx->finfo, sizeof(rx->finfo));
	dw1000_init_read(&rx->spi_info, &rx->spi_fqual, DW1000_RX_FQUAL, 0,
			 &rx->fqual, sizeof(rx->fqual));
	dw1000_init_read(&rx->spi_info, &rx->spi_time, DW1000_RX_TIME, 0,
			 &rx->time, sizeof(rx->time));

	/* Include Time Tracking only if enabled */
	if (dw1000_tsinfo_ena & DW1000_TSINFO_TTCK) {
		dw1000_init_read(&rx->spi_info, &rx->spi_ttcki, DW1000_RX_TTCKI,
				 0, &rx->ttcki, sizeof(rx->ttcki.raw));
		dw1000_init_read(&rx->spi_info, &rx->spi_ttcko, DW1000_RX_TTCKO,
				 0, &rx->ttcko, sizeof(rx->ttcko.raw));
	}

	/* Read status last */
	dw1000_init_read(&rx->spi_info, &rx->spi_status, DW1000_SYS_STATUS, 0,
			 &rx->status, sizeof(rx->status));
	

	/* Prepare data SPI message */
	spi_message_init(&rx->spi_data);
	dw1000_init_read(&rx->spi_data, &rx->spi_buffer, DW1000_RX_BUFFER, 0,
			 NULL, 0);
	dw1000_init_write(&rx->spi_data, &rx->spi_clear, DW1000_SYS_STATUS, 0,
			  &rx->clear, sizeof(rx->clear));
	dw1000_init_write(&rx->spi_data, &rx->spi_sys_ctrl, DW1000_SYS_CTRL,
			  DW1000_SYS_CTRL3, &rx->hrbpt, sizeof(rx->hrbpt));

	/* Prepare recovery SPI message */
	spi_message_init(&rx->spi_rcvr);
	dw1000_init_write(&rx->spi_rcvr, &rx->spi_mask0, DW1000_SYS_MASK,
			  0, &rx->mask0, sizeof(rx->mask0));
	dw1000_init_write(&rx->spi_rcvr, &rx->spi_reset, DW1000_SYS_STATUS,
			  0, &rx->reset, sizeof(rx->reset));
	dw1000_init_write(&rx->spi_rcvr, &rx->spi_hrbpt, DW1000_SYS_CTRL,
			  DW1000_SYS_CTRL3, &rx->hrbpt, sizeof(rx->hrbpt));
	dw1000_init_write(&rx->spi_rcvr, &rx->spi_mask1, DW1000_SYS_MASK,
			  0, &rx->mask1, sizeof(rx->mask1));
	dw1000_init_read(&rx->spi_info, &rx->spi_systat, DW1000_SYS_STATUS, 0,
			 &rx->status, sizeof(rx->status));

	return 0;
}

/**
 * dw1000_rx_reset() - Device Rx reset
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_rx_reset(struct dw1000 *dw)
{
	int rc;

	/* Update stats */
	dw1000_stats_inc(dw, DW1000_STATS_RX_RESET);

	/* Initiate reset */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     DW1000_PMSC_CTRL0_RXRESET, 0)) != 0)
		goto err_spi;

	/* Clear reset */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     DW1000_PMSC_CTRL0_RXRESET,
				     DW1000_PMSC_CTRL0_RXRESET)) != 0)
		goto err_spi;

	return 0;

 err_spi:
	dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
	dw1000_enqueue(dw, DW1000_ERROR_WORK);
	return rc;
}

/**
 * dw1000_rx_resync() - Handle minor receiver errors
 *
 * @dw:			DW1000 device
 */
static int dw1000_rx_resync(struct dw1000 *dw)
{
	struct dw1000_rx *rx = &dw->rx;
	unsigned status;
	int rc;

	/* Statistics */
	dw1000_stats_inc(dw, DW1000_STATS_RX_RESYNC);
		
	/* Send recovery SPI message */
	if ((rc = spi_sync(dw->spi, &rx->spi_rcvr)) != 0) {
		dev_err(dw->dev, "RX recovery failed: %d\n", rc);
		goto err_spi;
	}
	/* System status */
	status = le32_to_cpu(rx->status);
	
	/* RX out-of-sync */
	if (!DW1000_HSRPB_SYNC(status)) {
		/* Update statistics */
		dw1000_stats_inc(dw, DW1000_STATS_RX_HSRBP);
		/* Send recovery SPI message */
		if ((rc = spi_sync(dw->spi, &rx->spi_rcvr)) != 0) {
			dev_err(dw->dev, "RX sync recovery failed: %d\n", rc);
			goto err_spi;
		}
	}
	
	return 0;

 err_spi:
	dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
	dw1000_enqueue(dw, DW1000_ERROR_WORK);
	return rc;
}

/**
 * dw1000_rx_recover() - Recover from rx errors, like RXOVRR
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_rx_recover(struct dw1000 *dw)
{
	int rc;

	/* Disable trx */
	if ((rc = dw1000_disable(dw)) != 0)
		return rc;
	/* Soft Rx reset */
	if ((rc = dw1000_rx_reset(dw)) != 0)
		return rc;
	/* Resync and clear status */
	if ((rc = dw1000_rx_resync(dw)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_rx_link_qual() - Check RX quality and calculate KPIs, as
 *			   described in the DW1000 manual section 4.7.
 *
 * @dw:			DW1000 device
 */
static void dw1000_rx_link_qual(struct dw1000 *dw)
{
	struct dw1000_rx *rx = &dw->rx;
	struct dw1000_tsinfo *tsi = &rx->tsinfo;
	uint32_t finfo, status, hsrbp, rxpacc, index;
	uint32_t ampl1, ampl2, ampl3;
	unsigned int power = 0, noise = 0, snr = 0, psr = 0;
	unsigned int fpnrj = 0, fppwr = 0, fpr = 0;
	cycle_t cc;

	/* Clear Timestamp info */
	memset(tsi, 0, sizeof(*tsi));

	/* Assume frame and timestamp are valid */
	rx->frame_valid = true;
	rx->timestamp_valid = true;

	/* Extract status */
	finfo = le32_to_cpu(rx->finfo);
	status = le32_to_cpu(rx->status);

	/* LDE routine failed */
	if (unlikely((status & DW1000_SYS_STATUS_LDEDONE) == 0)) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_LDEDONE);
		goto invalid;
	}
	
	/* Extract buffer id */
	hsrbp = DW1000_RX_STATUS_HSRBP(status);

	/* Get 5-byte timestamp */
	cc = le64_to_cpu(rx->time.rx_stamp.cc) & DW1000_TIMESTAMP_MASK;

	/* Check it is really a new value */
	if (unlikely(((cc - rx->timestamp[hsrbp]) & DW1000_TIMESTAMP_MASK)
		     < DW1000_TIMESTAMP_REPETITION_THRESHOLD)) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_FRAME_REP);
		rx->frame_valid = false;
		goto invalid;
	}
	if (unlikely(((cc - rx->timestamp[hsrbp^1]) & DW1000_TIMESTAMP_MASK)
		     < DW1000_TIMESTAMP_REPETITION_THRESHOLD)) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_FRAME_REP);
		rx->frame_valid = false;
		goto invalid;
	}

	/* Remember timestamp */
	rx->timestamp[hsrbp] = cc;

	/* Signal quality KPIs */
	index = le16_to_cpu(rx->time.fp_index);
	ampl1 = le16_to_cpu(rx->time.fp_ampl1);
	ampl2 = le16_to_cpu(rx->fqual.fp_ampl2);
	ampl3 = le16_to_cpu(rx->fqual.fp_ampl3);
	power = le16_to_cpu(rx->fqual.cir_pwr);
	noise = le16_to_cpu(rx->fqual.std_noise);
	rxpacc = DW1000_RX_FINFO_RXPACC(finfo);

	/* Assign to Timestamp info */
	if (dw1000_tsinfo_ena & DW1000_TSINFO_RXQC) {
		tsi->fp_index = index;
		tsi->fp_ampl1 = ampl1;
		tsi->fp_ampl2 = ampl2;
		tsi->fp_ampl3 = ampl3;
		tsi->cir_pwr  = power;
		tsi->noise    = noise;
		tsi->rxpacc   = rxpacc;
	}

	/* Time tracking KPIs */
	if (dw1000_tsinfo_ena & DW1000_TSINFO_TTCK) {
		tsi->ttcko = le32_to_cpu(rx->ttcko.tofs);
		tsi->ttcki = le32_to_cpu(rx->ttcki.tcki);
	}

	/* Sanity check */
	if (noise == 0 || power == 0 || rxpacc == 0 ||
	    ampl1 == 0 || ampl2 == 0 || ampl3 == 0) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_KPI_ERROR);
		goto invalid;
	}

	/* Get (expected) preamble symbol repetition count */
	psr = dw1000_txpsrs[dw->cfg.txpsr];

	/* Calculate RX signal estimates */
	fpnrj = ampl1*ampl1 + ampl2*ampl2 + ampl3*ampl3;
	fppwr = fpnrj / (rxpacc*rxpacc);

	/* Calculate a simplistic S/N estimate */
	snr = fppwr / noise;

	/* Calculate a simplistic 8bit FP/Energy estimate */
	fpr = (fpnrj / power) >> (DW1000_RX_CIR_PWR_SCALE - 8);

	/* The reported LQI is upper limited */
	rx->lqi = (snr < DW1000_LQI_MAX) ? snr : DW1000_LQI_MAX;

	/* Assign calculated KPIs */
	if (dw1000_tsinfo_ena & DW1000_TSINFO_RXQC) {
		tsi->fp_pwr = fppwr;
		tsi->snr = snr;
		tsi->fpr = fpr;
		tsi->lqi = rx->lqi;
	}

	/* Check background noise */
	if (noise > dw->noise_threshold) {
		dw1000_stats_inc(dw, DW1000_STATS_NOISE_REJECT);
		goto reject;
	}

	/* Check S/N Ratio */
	if (snr < dw->snr_threshold) {
		dw1000_stats_inc(dw, DW1000_STATS_SNR_REJECT);
		goto reject;
	}
	if (fpr < dw->fpr_threshold) {
		dw1000_stats_inc(dw, DW1000_STATS_FPR_REJECT);
		goto reject;
	}

	return;

 invalid:
	/* Statistics */
	dw1000_stats_inc(dw, DW1000_STATS_RX_STAMP_ERROR);
 reject:
	/* Timestamp is invalid */
	rx->timestamp_valid = false;
}


/**
 * dw1000_rx_frame() - Handle Receiver Data Frame event
 *
 * @dw:			DW1000 device
 */
static void dw1000_rx_frame(struct dw1000 *dw)
{
	struct dw1000_rx *rx = &dw->rx;
	struct sk_buff *skb;
	uint32_t finfo, status;
	size_t len;
	int rc;

	/* The double buffering implementation in the DW1000 is
	 * somewhat flawed.  A packet arriving while there are no
	 * receive buffers available will be discarded, but only after
	 * corrupting the RX_FINFO, RX_TIME, and RX_FQUAL registers.
	 *
	 * We therefore read those registers first, then re-read the
	 * SYS_STATUS register.  If overrun is detected then we must
	 * discard the packet: even though the received data is still
	 * available, we cannot know its length.  If overrun is not
	 * detected at this point then we are safe to read the
	 * remaining data even if a subsequent overrun occurs before
	 * our data read is complete.
	 */

	/* Send information SPI message */
	if ((rc = spi_sync(dw->spi, &rx->spi_info)) != 0) {
		dev_err(dw->dev, "RX information message failed: %d\n", rc);
		dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
		goto err_info;
	}
	status = le32_to_cpu(rx->status);

	INJECT_ERROR(dw, RX_SYS_STATUS, status);
	
	/* Double buffering overrun check */
	if (unlikely(status & DW1000_SYS_STATUS_RXOVRR)) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_OVRR);
		goto err_recover;
	}
	/* FCS Error */
	if (unlikely(status & DW1000_SYS_STATUS_RXFCE)) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_FCE);
		goto err_discard;
	}
	/* FCS Good not set */
	if (unlikely(!(status & DW1000_SYS_STATUS_RXFCG))) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_FCG);
		goto err_discard;
	}
	/* Data frame ready not set */
	if (unlikely(!(status & DW1000_SYS_STATUS_RXDFR))) {
		dw1000_stats_inc(dw, DW1000_STATS_RX_DFR);
		goto err_discard;
	}

	/* Estimate link quality */
	dw1000_rx_link_qual(dw);

	/* Frame is invalid */
	if (!rx->frame_valid)
		goto err_discard;

	/* Frame length */
	finfo = le32_to_cpu(rx->finfo);
	len = DW1000_RX_FINFO_RXFLEN(finfo);

	/* Allocate socket buffer */
	skb = dev_alloc_skb(len);
	if (!skb) {
		dev_err(dw->dev, "RX buffer allocation failed\n");
		goto err_alloc;
	}
	skb_put(skb, len);

	/* Record hardware ptp timestamp, if valid */
	if (rx->timestamp_valid)
		dw1000_ptp_timestamp(dw, &rx->time.rx_stamp,
				     skb_hwtstamps(skb));

	/* Record ptp timestamp info */
	dw1000_ptp_tsinfo(dw, &rx->time.rx_stamp,
			  &rx->tsinfo, skb_hwtstamps(skb));

	/* Update data SPI message */
	rx->spi_buffer.data.rx_buf = skb->data;
	rx->spi_buffer.data.len = len;

	INJECT_ERROR(dw, RX_DELAY);

	/* Send data SPI message */
	if ((rc = spi_sync(dw->spi, &rx->spi_data)) != 0) {
		dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
		goto err_data;
	}

	/* Hand off to IEEE 802.15.4 stack */
	ieee802154_rx_irqsafe(dw->hw, skb, rx->lqi);

	/* Statistics */
	dw1000_stats_inc(dw, DW1000_STATS_RX_FRAME);
	
	return;

 err_data:
	dev_kfree_skb_any(skb);
 err_alloc:
 err_info:
	dw1000_stats_inc(dw, DW1000_STATS_RX_ERROR);
	dw1000_enqueue(dw, DW1000_ERROR_WORK);
	return;

 err_discard:
	dw1000_stats_inc(dw, DW1000_STATS_RX_ERROR);
	dw1000_rx_resync(dw);
	return;
	
 err_recover:
	dw1000_stats_inc(dw, DW1000_STATS_RX_ERROR);
	dw1000_rx_recover(dw);
	return;
}


/******************************************************************************
 *
 * IEEE 802.15.4 interface
 *
 */

/**
 * dw1000_ieee802154_start() - Start device operation
 *
 * @hw:			IEEE 802.15.4 device
 * @return:		0 on success or -errno
 */
static int dw1000_ieee802154_start(struct ieee802154_hw *hw)
{
	struct dw1000 *dw = hw->priv;

	dev_dbg(dw->dev, "ieee802154 stack start\n");
	
	dw->stm.ieee802154_enabled = true;
	dw1000_enqueue_wait(dw, DW1000_STACK_WORK);
	
	return 0;
}

/**
 * dw1000_ieee802154_stop() - Stop device operation
 *
 * @hw:			IEEE 802.15.4 device
 */
static void dw1000_ieee802154_stop(struct ieee802154_hw *hw)
{
	struct dw1000 *dw = hw->priv;
	
	dev_dbg(dw->dev, "ieee802154 stack stop\n");

	dw->stm.ieee802154_enabled = false;
	dw1000_enqueue_wait(dw, DW1000_STACK_WORK);
}

/**
 * dw1000_xmit_async() - Accept a packet for transmission
 *
 * @hw:			IEEE 802.15.4 device
 * @skb:		Socket buffer
 * @return:		0 on success or -errno
 */
static int dw1000_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct dw1000 *dw = hw->priv;
	struct dw1000_tx *tx = &dw->tx;
	unsigned long flags;

	/* Acquire TX lock */
	spin_lock_irqsave(&tx->lock, flags);

	/* Sanity check */
	if (tx->skb) {
		spin_unlock_irqrestore(&tx->lock, flags);
		dev_err(dw->dev, "concurrent transmit is not supported\n");
		return -ENOBUFS;
	}
	tx->skb = skb;
	spin_unlock_irqrestore(&tx->lock, flags);

	dw1000_enqueue(dw, DW1000_TX_WORK);
	return 0;
}

/**
 * dw1000_ed() - Detect energy on current radio channel
 *
 * @hw:			IEEE 802.15.4 device
 * @level:		Energy level to fill in (in arbitrary units)
 * @return:		0 on success or -errno
 */
static int dw1000_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct dw1000 *dw = hw->priv;
	__le32 agc_stat1 = 0;
	unsigned int edg1;
	unsigned int edv2;
	int rc;

	/* Read AGC_STAT1 register */
	if ((rc = regmap_raw_read(dw->agc_ctrl.regs, DW1000_AGC_STAT1,
				  &agc_stat1, DW1000_AGC_STAT1_LEN)) != 0)
		return rc;

	/* Calculate energy level as per datasheet.  Given the limited
	 * range available for the answer, we use a logarithmic scale
	 * and some approximations.
	 */
	edg1 = DW1000_AGC_STAT1_EDG1(le32_to_cpu(agc_stat1));
	edv2 = DW1000_AGC_STAT1_EDV2(le32_to_cpu(agc_stat1));
	if (edv2 < DW1000_EDV2_MIN) {
		*level = 0;
	} else {
		*level = (fls(edv2 - DW1000_EDV2_MIN) +
			  ((edg1 * DW1000_EDG1_MULT) >> DW1000_EDG1_SHIFT));
	}

	dev_dbg(dw->dev, "detected energy level %d on channel %d "
		"(EDG1=%d, EDV2=%d)\n",
		*level, dw->hw->phy->current_channel, edg1, edv2);
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
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Lock configuration */
	mutex_lock(&cfg->mutex);
	
	/* Set PAN ID */
	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		cfg->pan_id = le16_to_cpu(filt->pan_id);
		cfg->pending |= DW1000_CONFIGURE_PAN_ID;
		dev_dbg(dw->dev, "set PAN ID %04x\n", cfg->pan_id);
	}

	/* Set short address */
	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		cfg->short_addr = le16_to_cpu(filt->short_addr);
		cfg->pending |= DW1000_CONFIGURE_SHORT_ADDR;
		dev_dbg(dw->dev, "set short address %04x\n",
			cfg->short_addr);
	}

	/* Set EUI-64 */
	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		cfg->ieee_addr = filt->ieee_addr;
		cfg->pending |= DW1000_CONFIGURE_IEEE_ADDR;
		dev_dbg(dw->dev, "set EUI-64 %016llx\n",
			le64_to_cpu(cfg->ieee_addr));
	}

	/* Set coordinator status */
	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		if (filt->pan_coord)
			cfg->frame_filter |= DW1000_SYS_CFG_FFBC;
		else
			cfg->frame_filter &= ~DW1000_SYS_CFG_FFBC;
		cfg->pending |= DW1000_CONFIGURE_FRAME_FILTER;
		dev_dbg(dw->dev, "PAN coordinator role %sabled\n",
			(filt->pan_coord ? "en" : "dis"));
	}

	/* Unlock configuration */
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	return rc;
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
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* Enable/disable frame filtering */
	mutex_lock(&cfg->mutex);
	if (on)
		cfg->frame_filter &= ~DW1000_SYS_CFG_FFEN;
	else
		cfg->frame_filter |= DW1000_SYS_CFG_FFEN;
	cfg->pending |= DW1000_CONFIGURE_FRAME_FILTER;
	mutex_unlock(&cfg->mutex);
	
	/* Reconfigure changes */
	rc = dw1000_reconfigure(dw);

	dev_dbg(dw->dev, "promiscuous mode %sabled\n", (on ? "en" : "dis"));
	return rc;
}

/* IEEE 802.15.4 operations */
static const struct ieee802154_ops dw1000_ops = {
	.owner = THIS_MODULE,
	.start = dw1000_ieee802154_start,
	.stop = dw1000_ieee802154_stop,
	.xmit_async = dw1000_xmit_async,
	.ed = dw1000_ed,
	.set_channel = dw1000_set_channel,
	.set_hw_addr_filt = dw1000_set_hw_addr_filt,
	.set_promiscuous_mode = dw1000_set_promiscuous_mode,
};


/******************************************************************************
 *
 * Hardware monitor
 *
 */

/**
 * dw1000_hwmon_is_visible() - Check attribute visibility
 *
 * @drvdata:		DW1000 device
 * @type:		Sensor type
 * @attr:		Sensor attribute
 * @channel:		Channel number
 * @return:		File mode
 */
static umode_t dw1000_hwmon_is_visible(const void *drvdata,
				       enum hwmon_sensor_types type,
				       uint32_t attr, int channel)
{
	/* All attributes are read-only */
	return 0444;
}

/**
 * dw1000_hwmon_read() - Read attribute
 *
 * @dev:		Hardware monitor device
 * @type:		Sensor type
 * @attr:		Sensor attribute
 * @channel:		Channel number
 * @val:		Value to fill in
 * @return:		0 on success or -errno
 */
static int dw1000_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			     uint32_t attr, int channel, long *val)
{
	struct dw1000 *dw = dev_get_drvdata(dev);
	struct dw1000_sar *sar = &dw->sar;

	/* Convert result */
	switch (type) {
	    case hwmon_in:
		*val = dw1000_sar_get_volt(dw);
		dev_dbg(dw->dev, "hwmon voltage %#x (%#x @3.3V) is %ldmV\n",
			sar->adc.volt, dw->vcalib_3v3, *val);
		break;
	    case hwmon_temp:
		*val = dw1000_sar_get_temp(dw);
		dev_dbg(dw->dev, "hwmon temperature %#x (%#x @23C) is %ldmC\n",
			sar->adc.temp, dw->tcalib_23c, *val);
		break;
	    default:
		return -EINVAL;
	}

	return 0;
}

/* Hardware monitor voltage configuration */
static const uint32_t dw1000_in_config[] = {
	HWMON_T_INPUT,
	0
};

/* Hardware monitor voltage channel */
static const struct hwmon_channel_info dw1000_in = {
	.type = hwmon_in,
	.config = dw1000_in_config,
};

/* Hardware monitor temperature configuration */
static const uint32_t dw1000_temp_config[] = {
	HWMON_T_INPUT,
	0
};

/* Hardware monitor temperature channel */
static const struct hwmon_channel_info dw1000_temp = {
	.type = hwmon_temp,
	.config = dw1000_temp_config,
};

/* Hardware monitor channels */
static const struct hwmon_channel_info *dw1000_hwmon_channel[] = {
	&dw1000_in,
	&dw1000_temp,
	NULL
};

/* Hardware monitor operations */
static const struct hwmon_ops dw1000_hwmon_ops = {
	.is_visible = dw1000_hwmon_is_visible,
	.read = dw1000_hwmon_read,
};

/* Hardware monitor information */
static const struct hwmon_chip_info dw1000_hwmon_chip = {
	.info = dw1000_hwmon_channel,
	.ops = &dw1000_hwmon_ops,
};


/******************************************************************************
 *
 * Device attributes
 *
 */

#define DW1000_ATTR_RW(_name, _mode) \
	DEVICE_ATTR(_name, _mode, dw1000_show_##_name, dw1000_store_##_name)

#define DW1000_ATTR_RO(_name, _mode) \
	DEVICE_ATTR(_name, _mode, dw1000_show_##_name, NULL)

#define DW1000_ATTR_WR(_name, _mode) \
	DEVICE_ATTR(_name, _mode, NULL, dw1000_store_##_name)

/* Calibration profile */
static ssize_t dw1000_show_profile(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	struct dw1000_calib *cb;
	int i, count = 0;

	for (i=0; i<DW1000_MAX_CALIBS; i++) {
		cb = &dw->calib[i];
		if (cb->id[0])
			count += sprintf(buf + count,
					 "%s,%d,%d,0x%04x,0x%08x\n",
					 cb->id, cb->ch, dw1000_prfs[cb->prf],
					 cb->antd, cb->power);
	}
	return count;
}
static ssize_t dw1000_store_profile(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	char name[DW1000_CALIB_ID_LEN];
	int rc, i;

	for (i=0; i<(DW1000_CALIB_ID_LEN-1) && i<count && buf[i] != '\0' &&
		     buf[i] != '\n' && buf[i] != '\t' && buf[i] != ' '; i++)
		name[i] = buf[i];
	name[i] = 0;

	if ((rc = dw1000_configure_profile(dw, name)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(profile, 0644);

/* XTAL trim */
static ssize_t dw1000_show_xtalt(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->cfg.xtalt);
}
static ssize_t dw1000_store_xtalt(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned int trim;
	int rc;

	if ((rc = kstrtouint(buf, 0, &trim)) != 0)
		return rc;
	if ((rc = dw1000_configure_xtalt(dw, trim)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(xtalt, 0644);

/* Channel */
static ssize_t dw1000_show_channel(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->cfg.channel);
}
static ssize_t dw1000_store_channel(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	int channel;
	int rc;

	if ((rc = kstrtoint(buf, 0, &channel)) != 0)
		return rc;
	if ((rc = dw1000_configure_channel(dw, channel)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(channel, 0644);

/* Preamble code */
static ssize_t dw1000_show_pcode(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_get_pcode(&dw->cfg));
}
static ssize_t dw1000_store_pcode(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
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
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_prfs[dw->cfg.prf]);
}
static ssize_t dw1000_store_prf(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	int raw;
	int prf;
	int rc;

	if ((rc = kstrtoint(buf, 0, &raw)) != 0)
		return rc;
	if ((prf = dw1000_lookup_prf(raw)) < 0)
		return prf;
	if ((rc = dw1000_configure_prf(dw, prf)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(prf, 0644);

/* Antenna delay */
static ssize_t dw1000_show_antd(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "0x%4x\n", dw->cfg.antd[dw->cfg.prf]);
}
static ssize_t dw1000_store_antd(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned int delay;
	int rc;

	if ((rc = kstrtouint(buf, 0, &delay)) != 0)
		return rc;
	if ((rc = dw1000_configure_antd(dw, delay)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(antd, 0644);

/* Data rate */
static ssize_t dw1000_show_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw1000_rates[dw->cfg.rate]);
}
static ssize_t dw1000_store_rate(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	int raw, rate;
	int rc;

	if ((rc = kstrtoint(buf, 0, &raw)) != 0)
		return rc;
	if ((rate = dw1000_lookup_rate(raw)) < 0)
		return rate;
	if ((rc = dw1000_configure_rate(dw, rate)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(rate, 0644);

/* Transmit preamble symbol repetitions */
static ssize_t dw1000_show_txpsr(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	struct dw1000_config *cfg = &dw->cfg;

	return sprintf(buf, "%d\n", dw1000_txpsrs[dw1000_get_txpsr(cfg)]);
}
static ssize_t dw1000_store_txpsr(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	int raw;
	int txpsr;
	int rc;

	if ((rc = kstrtoint(buf, 0, &raw)) != 0)
		return rc;
	if ((txpsr = dw1000_lookup_txpsr(raw)) < 0)
		return txpsr;
	if ((rc = dw1000_configure_txpsr(dw, txpsr)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(txpsr, 0644);

/* Transmission power */
static ssize_t dw1000_show_tx_power(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	struct dw1000_config *cfg = &dw->cfg;
	
	return sprintf(buf, "0x%02x%02x%02x%02x\n",
		       cfg->txpwr[3], cfg->txpwr[2],
		       cfg->txpwr[1], cfg->txpwr[0]);
}
static ssize_t dw1000_store_tx_power(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned power;
	int rc;

	if ((rc = kstrtouint(buf, 0, &power)) != 0)
		return rc;
	if ((rc = dw1000_configure_tx_power(dw, power)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(tx_power, 0644);

/* Smart power control */
static ssize_t dw1000_show_smart_power(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->cfg.smart_power);
}
static ssize_t dw1000_store_smart_power(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	bool smart_power;
	int rc;

	if (strtobool(buf, &smart_power) < 0)
		return -EINVAL;
	if ((rc = dw1000_configure_smart_power(dw, smart_power)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(smart_power, 0644);

/* Frame filter */
static ssize_t dw1000_show_frame_filter(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	struct dw1000_config *cfg = &dw->cfg;

	return sprintf(buf, "0x%04x\n", cfg->frame_filter);
}
static ssize_t dw1000_store_frame_filter(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned int filter;
	int rc;
	
	if ((rc = kstrtouint(buf, 0, &filter)) != 0)
		return rc;
	if ((rc = dw1000_configure_frame_filter(dw, filter)) != 0)
		return rc;
	return count;
}
static DW1000_ATTR_RW(frame_filter, 0644);

/* Signal to Noise Ratio indicator threshold */
static ssize_t dw1000_show_snr_threshold(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)

{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->snr_threshold);
}
static ssize_t dw1000_store_snr_threshold(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned int snr_threshold;
	int rc;
	
	if ((rc = kstrtouint(buf, 0, &snr_threshold)) != 0)
		return rc;
	dw->snr_threshold = snr_threshold;
	return count;
}
static DW1000_ATTR_RW(snr_threshold, 0644);

/* First Path to Total Energy Ratio indicator threshold */
static ssize_t dw1000_show_fpr_threshold(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->fpr_threshold);
}
static ssize_t dw1000_store_fpr_threshold(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned int fpr_threshold;
	int rc;
	
	if ((rc = kstrtouint(buf, 0, &fpr_threshold)) != 0)
		return rc;
	dw->fpr_threshold = fpr_threshold;
	return count;
}
static DW1000_ATTR_RW(fpr_threshold, 0644);

/* Noise threshold */
static ssize_t dw1000_show_noise_threshold(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->noise_threshold);
}
static ssize_t dw1000_store_noise_threshold(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned int noise_threshold;
	int rc;
	
	if ((rc = kstrtouint(buf, 0, &noise_threshold)) != 0)
		return rc;
	dw->noise_threshold = noise_threshold;
	return count;
}
static DW1000_ATTR_RW(noise_threshold, 0644);

/* Statistics log interval */
static ssize_t dw1000_show_stats_interval(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct dw1000 *dw = dev_to_dw1000(dev);

	return sprintf(buf, "%d\n", dw->stats.interval);
}
static ssize_t dw1000_store_stats_interval(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	unsigned int interval;
	int rc;
	
	if ((rc = kstrtouint(buf, 0, &interval)) != 0)
		return rc;
	dw->stats.interval = interval;
	if (interval > 0)
		schedule_delayed_work(&dw->stats.work, interval * HZ);
	else
		cancel_delayed_work_sync(&dw->stats.work);
	return count;
}
static DW1000_ATTR_RW(stats_interval, 0644);

#ifdef DW1000_ERROR_INJECT
/* Error injection */
static ssize_t dw1000_store_error_inject(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct dw1000 *dw = dev_to_dw1000(dev);
	const char *col, *end;
	int error = 0;
	int value = 0;
	int cnt, i;
	int rc;

	end = strchrnul(buf, '\n');
	col = strchrnul(buf, ':');
	cnt = min(col,end) - buf;
	for (i=1; i<ERROR_INJECT_COUNT; i++) {
		if (strncasecmp(buf,dw1000_inject_items[i],cnt) == 0) {
			error = i;
			break;
		}
	}
	if (!error)
		return -EINVAL;
	if (*col) {
		if ((rc = kstrtoint(col+1, 0, &value)) != 0)
			return rc;
	}
	
	dw->inject_error = error;
	dw->inject_value = value;
	return count;
}
static DW1000_ATTR_WR(error_inject, 0200);
#endif /* DW1000_ERROR_INJECT */

/* Attribute list */
#define DW1000_SYSFS_ATTR(_name)  &dev_attr_##_name.attr
static struct attribute *dw1000_attrs[] = {
	DW1000_SYSFS_ATTR(profile),
	DW1000_SYSFS_ATTR(xtalt),
	DW1000_SYSFS_ATTR(channel),
	DW1000_SYSFS_ATTR(pcode),
	DW1000_SYSFS_ATTR(prf),
	DW1000_SYSFS_ATTR(antd),
	DW1000_SYSFS_ATTR(rate),
	DW1000_SYSFS_ATTR(txpsr),
	DW1000_SYSFS_ATTR(tx_power),
	DW1000_SYSFS_ATTR(smart_power),
	DW1000_SYSFS_ATTR(frame_filter),
	DW1000_SYSFS_ATTR(snr_threshold),
	DW1000_SYSFS_ATTR(fpr_threshold),
	DW1000_SYSFS_ATTR(noise_threshold),
	DW1000_SYSFS_ATTR(stats_interval),
#ifdef DW1000_ERROR_INJECT
	DW1000_SYSFS_ATTR(error_inject),
#endif /* DW1000_ERROR_INJECT */
	NULL
};

/* Attribute group */
static struct attribute_group dw1000_attr_group = {
	.name = "dw1000",
	.attrs = dw1000_attrs,
};


/******************************************************************************
 *
 * Statistics
 *
 */

/**
 * dw1000_stats_inc() - Increment statistics
 *
 * @dw:			DW1000 device
 * @id:                 Stats enum id
 */
static inline void dw1000_stats_inc(struct dw1000 *dw, int id)
{
	dw->stats.count[id]++;
}

/**
 * dw1000_print_stats() - Print changed statistics
 *
 * @dw:			DW1000 device
 */

static void dw1000_stats_print(struct dw1000 *dw)
{
	char buf1[20], buf2[20];
	int i;

	/* Detect changes */
	if (memcmp(dw->stats.reported, dw->stats.count,
		   sizeof(dw->stats.reported)) != 0) {
		dev_dbg(dw->dev, "STATISTICS:\n");
		for (i=0; i<DW1000_STATS_COUNT; i++) {
			if (dw->stats.count[i] != dw->stats.reported[i]) {
				sprintf(buf1, "%+d", dw->stats.count[i] -
					             dw->stats.reported[i]);
				sprintf(buf2, "%d", dw->stats.count[i]);
				dev_dbg(dw->dev, "  %-16s %10s  [%9s]\n",
					dw1000_stats[i], buf1, buf2);
				dw->stats.reported[i] = dw->stats.count[i];
			}
		}
	}
}

#define DW1000_STATS_DEF(_name,_enum)                                         \
static ssize_t dw1000_show_dw1000_##_name(struct device *dev,                 \
                struct device_attribute *attr, char *buf)                     \
{                                                                             \
        struct dw1000 *dw = dev_to_dw1000(dev);                               \
        return sprintf(buf, "%d", dw->stats.count[DW1000_STATS_##_enum]);     \
}                                                                             \
static ssize_t dw1000_store_dw1000_##_name(struct device *dev,                \
                struct device_attribute *attr,                                \
                const char *buf, size_t count)                                \
{                                                                             \
        struct dw1000 *dw = dev_to_dw1000(dev);                               \
        unsigned int value;                                                   \
        int rc;                                                               \
        if ((rc = kstrtouint(buf, 0, &value)) != 0)                           \
                return rc;                                                    \
        dw->stats.count[DW1000_STATS_##_enum] = value;                        \
        return count;                                                         \
}                                                                             \
static DEVICE_ATTR(dw1000_##_name, 0644,                                      \
                   dw1000_show_dw1000_##_name,				      \
		   dw1000_store_dw1000_##_name);			      \

DW1000_STATS_DEF(rx_frame, RX_FRAME)
DW1000_STATS_DEF(tx_frame, TX_FRAME)
DW1000_STATS_DEF(rx_error, RX_ERROR)
DW1000_STATS_DEF(tx_error, TX_ERROR)
DW1000_STATS_DEF(irq_count, IRQ_COUNT)
DW1000_STATS_DEF(spi_error, SPI_ERROR)
DW1000_STATS_DEF(rx_reset, RX_RESET)
DW1000_STATS_DEF(rx_resync, RX_RESYNC)
DW1000_STATS_DEF(rx_hsrbp, RX_HSRBP)
DW1000_STATS_DEF(rx_ovrr, RX_OVRR)
DW1000_STATS_DEF(rx_dfr, RX_DFR)
DW1000_STATS_DEF(rx_fcg, RX_FCG)
DW1000_STATS_DEF(rx_ldedone, RX_LDEDONE)
DW1000_STATS_DEF(rx_kpi_error, RX_KPI_ERROR)
DW1000_STATS_DEF(rx_stamp_error, RX_STAMP_ERROR)
DW1000_STATS_DEF(rx_frame_rep, RX_FRAME_REP)
DW1000_STATS_DEF(tx_retry, TX_RETRY)
DW1000_STATS_DEF(snr_reject, SNR_REJECT)
DW1000_STATS_DEF(fpr_reject, FPR_REJECT)
DW1000_STATS_DEF(noise_reject, NOISE_REJECT)
DW1000_STATS_DEF(hard_reset, HARD_RESET)

/* Attribute list */
#define DW1000_STATS_ATTR(_name)  &dev_attr_dw1000_##_name.attr
static struct attribute *dw1000_stats_attrs[] = {
	DW1000_STATS_ATTR(rx_frame),
	DW1000_STATS_ATTR(tx_frame),
	DW1000_STATS_ATTR(rx_error),
	DW1000_STATS_ATTR(tx_error),
	DW1000_STATS_ATTR(irq_count),
	DW1000_STATS_ATTR(spi_error),
	DW1000_STATS_ATTR(rx_reset),
	DW1000_STATS_ATTR(rx_resync),
	DW1000_STATS_ATTR(rx_hsrbp),
	DW1000_STATS_ATTR(rx_ovrr),
	DW1000_STATS_ATTR(rx_dfr),
	DW1000_STATS_ATTR(rx_fcg ),
	DW1000_STATS_ATTR(rx_ldedone),
	DW1000_STATS_ATTR(rx_kpi_error),
	DW1000_STATS_ATTR(rx_stamp_error),
	DW1000_STATS_ATTR(rx_frame_rep),
	DW1000_STATS_ATTR(tx_retry),
	DW1000_STATS_ATTR(snr_reject),
	DW1000_STATS_ATTR(fpr_reject),
	DW1000_STATS_ATTR(noise_reject),
	DW1000_STATS_ATTR(hard_reset),
	NULL
};

/* Attribute group */
static struct attribute_group dw1000_stats_attr_group = {
	.name = "statistics",
	.attrs = dw1000_stats_attrs,
};


/******************************************************************************
 *
 * Event handlers
 *
 */

/**
 * dw1000_stats_work() - Statistics work
 *
 * @work:		Worker
 */
static void dw1000_stats_work(struct work_struct *work)
{
	struct dw1000 *dw = container_of(to_delayed_work(work),
					 struct dw1000, stats.work);
	
	/* Print stats */
	dw1000_stats_print(dw);

	/* Reschedule worker */
	if (dw->stats.interval > 0)
		schedule_delayed_work(&dw->stats.work,
				      dw->stats.interval * HZ);
}

/**
 * dw1000_ptp_timer_handler() - PTP Timer event handler
 *
 * @timer:		Timer pointer
 */
static void dw1000_ptp_timer_handler(struct timer_list *timer)
{
	struct dw1000 *dw = container_of(timer, struct dw1000, ptp.timer);

	dw1000_enqueue(dw, DW1000_PTP_WORK);
}

/**
 * dw1000_state_timer_handler() - State timer event handler
 *
 * @timer:		Timer pointer
 */
static void dw1000_state_timer_handler(struct timer_list *timer)
{
	struct dw1000 *dw = container_of(timer, struct dw1000, stm.state_timer);

	dw1000_enqueue(dw, DW1000_TIMER_WORK);
}

/**
 * dw1000_adc_timer_handler() - ADC timer event handler
 *
 * @timer:		Timer pointer
 */
static void dw1000_adc_timer_handler(struct timer_list *timer)
{
	struct dw1000 *dw = container_of(timer, struct dw1000, stm.adc_timer);

	dw1000_enqueue(dw, DW1000_ADC_WORK);
	mod_timer(timer, jiffies +
		  msecs_to_jiffies(DW1000_ADC_READ_DELAY));
}

/**
 * dw1000_irq_handler() - Interrupt handler
 *
 * @irq:		Interrupt number
 * @context:		DW1000 device
 * @return:		IRQ status
 */
static irqreturn_t dw1000_irq_handler(int irq, void *context)
{
	struct dw1000 *dw = context;

	dw1000_enqueue_irq(dw);
	
	return IRQ_HANDLED;
}


/******************************************************************************
 *
 * Device initialisation
 *
 */

/**
 * dw1000_poweron() - Power on the device
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_poweron(struct dw1000 *dw)
{
	/* Power on the device */
	if (gpio_is_valid(dw->power_gpio)) {
		gpio_set_value(dw->power_gpio, 1);
		msleep(DW1000_POWER_ON_DELAY);
	}

	/* Release RESET GPIO */
	if (gpio_is_valid(dw->reset_gpio)) {
		gpio_set_value(dw->reset_gpio, 1);
		msleep(DW1000_HARD_RESET_DELAY);
	}

	return 0;
}

/**
 * dw1000_poweroff() - Power off the device
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_poweroff(struct dw1000 *dw)
{
	/* Assert RESET GPIO */
	if (gpio_is_valid(dw->reset_gpio)) {
		gpio_set_value(dw->reset_gpio, 0);
		msleep(DW1000_HARD_RESET_DELAY);
	}

	/* Power off the device */
	if (gpio_is_valid(dw->power_gpio)) {
		gpio_set_value(dw->power_gpio, 0);
		msleep(DW1000_POWER_OFF_DELAY);
	}

	return 0;
}

/**
 * dw1000_enable() - Enable device
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_enable(struct dw1000 *dw)
{
	unsigned sys_status;
	int rc;

	/* Read system status */
	if ((rc = regmap_read(dw->sys_status.regs, 0, &sys_status)) != 0)
		goto err_spi;

	/* Ensure that HSRBP and ICRBP are in sync */
	if (!DW1000_HSRPB_SYNC(sys_status)) {
		if ((rc = regmap_write(dw->sys_ctrl.regs, DW1000_SYS_CTRL3,
				       DW1000_SYS_CTRL3_HRBPT)) != 0)
			goto err_spi;
	}

	/* Enable RX interrupt generation */
	if ((rc = regmap_write(dw->sys_mask.regs, 0,
			       DW1000_SYS_MASK_RX)) != 0)
		goto err_spi;

	/* Enable receiver */
	if ((rc = regmap_write(dw->sys_ctrl.regs, DW1000_SYS_CTRL1,
			       DW1000_SYS_CTRL1_RXENAB)) != 0)
		goto err_spi;

	/* Set state to RX ready */
	dw1000_set_state(dw, DW1000_STATE_RX);
	return 0;

 err_spi:
	dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
	dw1000_enqueue(dw, DW1000_ERROR_WORK);
	return rc;
}

/**
 * dw1000_disable() - Disable device
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_disable(struct dw1000 *dw)
{
	int rc;

	/*
	 * This is designed not to fail if the device is
	 * unresponsive or dead. The SPI transactions will
	 * succeed nontheless, and in the end the device
	 * should be inactive.
	 */

	/* Disable further interrupt generation */
	if ((rc = regmap_write(dw->sys_mask.regs, 0,
			       DW1000_SYS_MASK_IDLE)) != 0)
		goto err_spi;

	/* Disable receiver and transmitter */
	if ((rc = regmap_write(dw->sys_ctrl.regs, DW1000_SYS_CTRL0,
			       DW1000_SYS_CTRL0_TRXOFF)) != 0)
		goto err_spi;

	/* Clear pending trx interrupts */
	if ((rc = regmap_write(dw->sys_status.regs, 0,
			       DW1000_SYS_STATUS_TRX)) != 0)
		goto err_spi;
	
	/* IDLE state */
	dw1000_set_state(dw, DW1000_STATE_IDLE);
	return 0;

 err_spi:
	dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
	dw1000_enqueue(dw, DW1000_ERROR_WORK);
	return rc;
}

/**
 * dw1000_check_dev_id() - Check device ID
 *
 * The register ID tag portion of the device ID has a fixed value.
 * This can be used to verify that the SPI bus is operational and that
 * the DW1000 device is present and responding to commands.
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_check_dev_id(struct dw1000 *dw)
{
	int ridtag;
	int rc;

	/* Read device ID */
	if ((rc = regmap_read(dw->dev_id.regs, DW1000_RIDTAG, &ridtag)) != 0) {
		dev_err(dw->dev, "could not read device ID: %d\n", rc);
		return rc;
	}

	/* Check register ID tag */
	if (ridtag != DW1000_RIDTAG_MAGIC) {
		dev_err(dw->dev, "incorrect RID tag %04X (expected %04X)\n",
			ridtag, DW1000_RIDTAG_MAGIC);
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
	int rc;

	/* Force slow SPI clock speed */
	dw->spi->max_speed_hz = DW1000_SPI_SLOW_HZ;

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

	/* Check device ID to ensure bus is operational */
	if ((rc = dw1000_check_dev_id(dw)) != 0)
		return rc;

	/* Switch to full SPI clock speed */
	dw->spi->max_speed_hz = DW1000_SPI_FAST_HZ;

	/* Recheck device ID to ensure bus is still operational */
	if ((rc = dw1000_check_dev_id(dw)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_load_xtalt() - Load XTALT calibration value
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_load_xtalt(struct dw1000 *dw)
{
	struct dw1000_config *cfg = &dw->cfg;
	uint32_t value;
	uint8_t xtalt;
	int rc;

	/* Default value for XTALT */
	xtalt = DW1000_FS_XTALT_XTALT_MIDPOINT;

	/* Read XTAL_Trim calibration value from OTP */
	if ((rc = regmap_read(dw->otp, DW1000_OTP_REV_XTALT, &value)) != 0)
		return rc;
	if (DW1000_OTP_XTALT(value) != 0 && DW1000_OTP_XTALT(value) != 0x1f)
		xtalt = DW1000_OTP_XTALT(value);

	/* Read XTALT calibration value from devicetree */
	if (of_property_read_u32(dw->dev->of_node, "decawave,xtalt",
				 &value) == 0) {
		/* Check the value range */
		if ((value & ~DW1000_FS_XTALT_XTALT_MASK) == 0)
			xtalt = value;
		else
			dev_err(dw->dev, "invalid decawave,xtalt "
				"value 0x%02x\n", value);
	}

	cfg->xtalt = xtalt;

	dev_dbg(dw->dev, "set xtal trim 0x%02x\n", xtalt);
	return 0;
}

/**
 * dw1000_load_ldotune() - Load LDOTUNE calibration value
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_load_ldotune(struct dw1000 *dw)
{
	union dw1000_ldotune ldotune;
	unsigned int i;
	int rc;

	/* Read LDOTUNE calibration value from OTP */
	if ((rc = regmap_bulk_read(dw->otp, DW1000_OTP_LDOTUNE, ldotune.otp,
				   ARRAY_SIZE(ldotune.otp))) != 0)
		return rc;
	for (i = 0; i < ARRAY_SIZE(ldotune.otp); i++)
		cpu_to_le32s(ldotune.otp[i]);

	/* Read LDOTUNE calibration value from devicetree */
	rc = of_property_read_variable_u8_array(dw->dev->of_node,
						"decawave,ldotune",
						ldotune.raw,
						sizeof(ldotune.raw),
						sizeof(ldotune.raw));
	
	/* Apply LDOTUNE calibration value, if applicable */
	if (ldotune.raw[0]) {
		dev_dbg(dw->dev, "set ldotune %02x:%02x:%02x:%02x:%02x\n",
			ldotune.raw[4], ldotune.raw[3], ldotune.raw[2],
			ldotune.raw[1], ldotune.raw[0]);
		if ((rc = regmap_raw_write(dw->rf_conf.regs,
					   DW1000_RF_LDOTUNE,
					   &ldotune.raw,
					   sizeof(ldotune.raw))) != 0)
		    return rc;
	}

	return 0;
}

/**
 * dw1000_load_eui64() - Load EUI-64 extended address
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_load_eui64(struct dw1000 *dw)
{
	struct wpan_phy *phy = dw->hw->phy;
	union dw1000_eui64 eui64;
	int i;
	int rc;

	/* Read EUI-64 address from OTP */
	if ((rc = regmap_bulk_read(dw->otp, DW1000_OTP_EUI64, eui64.otp,
				   ARRAY_SIZE(eui64.otp))) != 0)
		return rc;
	for (i = 0; i < ARRAY_SIZE(eui64.otp); i++)
		cpu_to_le32s(eui64.otp[i]);

	/* Read EUI-64 address from devicetree */
	rc = of_property_read_u64(dw->dev->of_node, "decawave,eui64",
				  &eui64.addr);

	/* Use EUI-64 address if valid, otherwise generate random address */
	if (ieee802154_is_valid_extended_unicast_addr(eui64.addr)) {
		phy->perm_extended_addr = eui64.addr;
	} else {
		dev_warn(dw->dev, "has no permanent EUI-64 address\n");
		ieee802154_random_extended_addr(&phy->perm_extended_addr);
	}

	return 0;
}

/**
 * dw1000_load_delays() - Load antenna delays
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_load_delays(struct dw1000 *dw)
{
	struct dw1000_config *cfg = &dw->cfg;
	uint32_t array[2];
	uint32_t delays;
	int rc;

	/* Read delays from OTP */
	if ((rc = regmap_read(dw->otp, DW1000_OTP_DELAYS, &delays)) != 0)
		return rc;
	if (delays != 0 && delays != 0xfffffffful) {
		cfg->antd[DW1000_PRF_16M] = DW1000_OTP_DELAYS_16M(delays);
		cfg->antd[DW1000_PRF_64M] = DW1000_OTP_DELAYS_64M(delays);
	}

	/* Read delays from devicetree */
	if (of_property_read_u32_array(dw->dev->of_node, "decawave,antd",
				       array, ARRAY_SIZE(array)) == 0) {
		if ((array[0] & ~DW1000_ANTD_MASK) ||
		    (array[1] & ~DW1000_ANTD_MASK)) {
			dev_err(dw->dev,
				"invalid decawave,antd values <0x%04x,0x%04x>\n",
				array[0], array[1]);
			return 0;
		}
		cfg->antd[DW1000_PRF_16M] = array[0];
		cfg->antd[DW1000_PRF_64M] = array[1];
	}

	dev_dbg(dw->dev, "set default antenna delays 0x%04x,0x%04x\n",
		cfg->antd[DW1000_PRF_16M], cfg->antd[DW1000_PRF_64M]);
	return 0;
}

/**
 * dw1000_load_sar() - Load SAR ADC calibration values
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_load_sar(struct dw1000 *dw)
{
	int vcalib;
	int tcalib;
	int rc;

	/* Read voltage measurement calibration from OTP */
	if ((rc = regmap_read(dw->otp, DW1000_OTP_VMEAS, &vcalib)) != 0)
		return rc;
	dw->vcalib_3v3 = DW1000_OTP_VMEAS_3V3(vcalib);

	/* Read temperature measurement calibration from OTP */
	if ((rc = regmap_read(dw->otp, DW1000_OTP_TMEAS, &tcalib)) != 0)
		return rc;
	dw->tcalib_23c = DW1000_OTP_TMEAS_23C(tcalib);

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
	if ((rc = dw1000_check_dev_id(dw)) != 0)
		return rc;

	return 0;
}

/**
 * dw1000_load_profiles() - Load configuration profiles
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_load_profiles(struct dw1000 *dw)
{
	struct device_node *node, *np = NULL;
	const char *name, *profile;
	int ch, prf, prfd, antd, power;
	int pid = 0;

	/* Find the calibration node */
	node = of_find_node_by_name(dw->dev->of_node, "decawave,calib");
	if (!node) {
		dev_dbg(dw->dev, "no calibration sets in device tree\n");
		return 0;
	}

	/* Iterate through calibrations */
	while ((pid < DW1000_MAX_CALIBS) && (np = of_get_next_child(node,np))) {
		if (!strcmp(np->name, "calib")) {
			if (of_property_read_string(np, "id", &name) ||
			    of_property_read_u32(np, "ch", &ch) ||
			    of_property_read_u32(np, "prf", &prfd) ||
			    of_property_read_u32(np, "antd", &antd) ||
			    of_property_read_u32(np, "power", &power))
				goto error;

			prf = dw1000_lookup_prf(prfd);
			if (prf < 0)
				goto error;
			if (!(BIT(ch) & DW1000_CHANNELS))
				goto error;
			if (antd & ~DW1000_ANTD_MASK)
				goto error;

			dev_dbg(dw->dev, "profile %s: %d,%d,0x%04x,0x%08x\n",
				name, ch, prfd, antd, power);

			strncpy(dw->calib[pid].id, name, DW1000_CALIB_ID_LEN-1);
			dw->calib[pid].ch = ch;
			dw->calib[pid].prf = prf;
			dw->calib[pid].antd = antd;
			dw->calib[pid].power = power;

			pid++;
			continue;
		}
 error:
		dev_warn(dw->dev, "failure reading \"%s\"\n", np->full_name);
	}

	of_node_put(node);

	/* Default profile */
	if (of_property_read_string(dw->dev->of_node,
				    "decawave,default", &profile) == 0) {
		dw1000_configure_profile(dw,profile);
	}

	return 0;
}

/**
 * dw1000_hwinit() - Apply hardware configuration
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_hwinit(struct dw1000 *dw)
{
	int modelverrev;
	int mask;
	int value;
	int rc;

	/* Log device ID */
	if ((rc = regmap_read(dw->dev_id.regs, DW1000_MODELVERREV,
			      &modelverrev)) != 0)
		return rc;
	dev_info(dw->dev, "DW1000 revision %d.%d.%d detected\n",
		 DW1000_MODELVERREV_MODEL(modelverrev),
		 DW1000_MODELVERREV_VER(modelverrev),
		 DW1000_MODELVERREV_REV(modelverrev));

	/* Set system configuration:
	 *
	 * - frame filter disabled
	 * - use double buffering
	 * - use receiver auto-re-enabling
	 *
	 * Note that the overall use of filtering is controlled via
	 * dw1000_set_promiscuous_mode().
	 */
	mask = (DW1000_SYS_CFG_FFA | DW1000_SYS_CFG_DIS_DRXB |
		DW1000_SYS_CFG_RXAUTR);
	value = (DW1000_SYS_CFG_RXAUTR);
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
	
#ifdef DW1000_GPIO_LEDS
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
#endif
	/* Clear diagnostic counters */
	mask = DW1000_EVC_CTRL_EVC_CLR;
	value = DW1000_EVC_CTRL_EVC_CLR;
	if ((rc = regmap_update_bits(dw->dig_diag.regs, DW1000_EVC_CTRL,
				     mask, value)) != 0)
		return rc;

	/* Enable diagnostic counters */
	mask = DW1000_EVC_CTRL_EVC_EN;
	value = DW1000_EVC_CTRL_EVC_EN;
	if ((rc = regmap_update_bits(dw->dig_diag.regs, DW1000_EVC_CTRL,
				     mask, value)) != 0)
		return rc;

	/* Enable clock PLL detection flags */
	mask = DW1000_EC_CTRL_PLLLDT;
	value = DW1000_EC_CTRL_PLLLDT;
	if ((rc = regmap_update_bits(dw->ext_sync.regs, DW1000_EC_CTRL,
				     mask, value)) != 0)
		return rc;

	/* Force TX and RX clocks to stay enabled at all times.  This
	 * seems to prevent an internal state machine lockup under
	 * heavy load, the symptoms of which are that SYS_STATE_PMSC
	 * reports the overall state as "RX" but SYS_STATE_RX reports
	 * the RX state as "idle".
	 */
	if ((rc = regmap_update_bits(dw->pmsc.regs, DW1000_PMSC_CTRL0,
				     (DW1000_PMSC_CTRL0_RXCLKS_MASK |
				      DW1000_PMSC_CTRL0_TXCLKS_MASK),
				     (DW1000_PMSC_CTRL0_RXCLKS_FAST |
				      DW1000_PMSC_CTRL0_TXCLKS_FAST))) != 0)
		return rc;

	/* Clear startup status bits */
	if ((rc = regmap_write(dw->sys_status.regs, 0,
			       DW1000_SYS_STATUS_RESET)) != 0)
		return rc;

	return 0;

}

/**
 * dw1000_init() - Initialise device
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_init(struct dw1000 *dw)
{
	struct dw1000_config *cfg = &dw->cfg;
	int rc;
	
	/* All config items pending */
	cfg->pending = ~0;

	/* Start device */
	if ((rc = dw1000_reset(dw)) != 0)
		goto error;
	if ((rc = dw1000_hwinit(dw)) != 0)
		goto error;
	if ((rc = dw1000_load_lde(dw)) != 0)
		goto error;
	if ((rc = dw1000_load_config(dw)) != 0)
		goto error;
	if ((rc = dw1000_ptp_reset(dw)) != 0)
		goto error;
	
	dw1000_set_state(dw, DW1000_STATE_IDLE);
	return 0;

 error:
	dw1000_disable(dw);
	dw1000_poweroff(dw);
	dw1000_set_state(dw, DW1000_STATE_DEAD);
 	return rc;
}

/**
 * dw1000_remove() - Remove device
 *
 * @dw:			DW1000 device
 */
static void dw1000_remove(struct dw1000 *dw)
{
	/* Stop device */
	dw1000_disable(dw);
	dw1000_poweroff(dw);
	dw1000_set_state(dw, DW1000_STATE_OFF);
}

/**
 * dw1000_reload() - Reset & reload device
 *
 * @dw:			DW1000 device
 */
static void dw1000_reload(struct dw1000 *dw)
{
	struct dw1000_state *stm = &dw->stm;
	
	/* Power down */
	dw1000_remove(dw);

	/* Number of hard resets */
	dw1000_stats_inc(dw, DW1000_STATS_HARD_RESET);

	/* Limit recovery attempts */
	if (stm->recovery_count++ < DW1000_MAX_RECOVERY) {
		dw1000_set_state(dw, DW1000_STATE_OFF);
		dw1000_enqueue(dw, DW1000_INIT_WORK);
	} else {
		dw1000_set_state(dw, DW1000_STATE_DEAD);
	}
}

/**
 * dw1000_game_over() - Device failure
 *
 * @dw:			DW1000 device
 */
static void dw1000_game_over(struct dw1000 *dw)
{
	/* Kill device */
	dw1000_disable(dw);
	dw1000_poweroff(dw);
	dw1000_set_state(dw, DW1000_STATE_DEAD);

	dev_err(dw->dev, "GAME OVER\n");
}


/******************************************************************************
 *
 * State Machine
 *
 */

/**
 * dw1000_state_init() - Prepare state machine
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_state_init(struct dw1000 *dw)
{
	struct dw1000_state *stm = &dw->stm;
	struct sched_param sched_par = {
		.sched_priority = MAX_RT_PRIO - 2
	};
	
	/* Clear memory */
	memset(stm,0,sizeof(*stm));

	/* Wait queues */
	init_waitqueue_head(&stm->work_wq);

	/* Setup timer for state timeout */
	timer_setup(&stm->state_timer, dw1000_state_timer_handler, 0);
	timer_setup(&stm->adc_timer, dw1000_adc_timer_handler, 0);

	/* Setup background work */
	INIT_DELAYED_WORK(&dw->stats.work, dw1000_stats_work);

	/* Init event handler thread */
	stm->mthread = kthread_create(dw1000_event_thread, dw,
				      "%s", dev_name(dw->dev));
	if (IS_ERR(stm->mthread)) {
		return PTR_ERR(stm->mthread);
	}
	kthread_bind(stm->mthread, 0);
	
	/* Increase thread priority */
        sched_setscheduler(stm->mthread, SCHED_FIFO, &sched_par);
	
	/* Default state */
	stm->mstate = DW1000_STATE_OFF;
	
	return 0;
}

/**
 * dw1000_state_start() - Start state machine
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_state_start(struct dw1000 *dw)
{
	struct dw1000_state *stm = &dw->stm;

	/* Start ADC timer */
	mod_timer(&stm->adc_timer, jiffies +
		  msecs_to_jiffies(DW1000_ADC_READ_DELAY));
	
	/* Initialisation work */
	dw1000_enqueue(dw, DW1000_INIT_WORK);

	/* Start state machine thread */
	wake_up_process(stm->mthread);
	
	dev_dbg(dw->dev, "state machine started\n");
	return 0;
}

/**
 * dw1000_state_stop() - Sop state machine
 *
 * @dw:			DW1000 device
 * @return:		0 on success or -errno
 */
static int dw1000_state_stop(struct dw1000 *dw)
{
	struct dw1000_state *stm = &dw->stm;

	/* Stop state machine thread */
	kthread_stop(stm->mthread);

	/* No IRQs after this point */
	disable_irq(dw->spi->irq);
	
	/* Stop statistics work */
	cancel_delayed_work_sync(&dw->stats.work);
	
	/* Delete timers */
	del_timer_sync(&stm->state_timer);
	del_timer_sync(&stm->adc_timer);
	
	dev_dbg(dw->dev, "state machine stopped\n");
	return 0;
}

static __always_inline void
dw1000_set_state(struct dw1000 *dw, unsigned state)
{
	dw1000_set_state_timeout(dw, state, DW1000_DEFAULT_TIMEOUT);
}

static __always_inline void
dw1000_set_state_timeout(struct dw1000 *dw, unsigned state, unsigned us)
{
	struct dw1000_state *stm = &dw->stm;
	
	stm->mstate = state;
	mod_timer(&stm->state_timer, jiffies + usecs_to_jiffies(us));
}

static void dw1000_enqueue(struct dw1000 *dw, unsigned long work)
{
	struct dw1000_state *stm = &dw->stm;
	unsigned long flags;
	
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work |= work;
	wake_up_locked(&stm->work_wq);
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

static void dw1000_enqueue_wait(struct dw1000 *dw, unsigned long work)
{
	struct dw1000_state *stm = &dw->stm;
	unsigned long flags;
	
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work |= work;
	wake_up_locked(&stm->work_wq);
	wait_event_interruptible_locked_irq(stm->work_wq,
					    !(stm->pending_work & work));
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

static void dw1000_dequeue(struct dw1000 *dw, unsigned long work)
{
	struct dw1000_state *stm = &dw->stm;
	unsigned long flags;
	
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work &= ~work;
	wake_up_locked(&stm->work_wq);
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

static void dw1000_enqueue_irq(struct dw1000 *dw)
{
	struct dw1000_state *stm = &dw->stm;
	unsigned long flags;
	
	dw1000_stats_inc(dw, DW1000_STATS_IRQ_COUNT);
	
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	if (!(stm->pending_work & DW1000_IRQ_WORK)) {
		stm->pending_work |= DW1000_IRQ_WORK;
		disable_irq_nosync(dw->spi->irq);
	}
	wake_up_locked(&stm->work_wq);
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

static void dw1000_wait_pending_work(struct dw1000 *dw)
{
	struct dw1000_state *stm = &dw->stm;
	unsigned long flags;
	
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	if (stm->pending_work & DW1000_IRQ_WORK) {
		stm->pending_work &= ~DW1000_IRQ_WORK;
		enable_irq(dw->spi->irq);
	}
	wait_event_interruptible_locked_irq(stm->work_wq,
					    stm->pending_work ||
					    kthread_should_stop());
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

static unsigned long dw1000_get_pending_work(struct dw1000 *dw)
{
	struct dw1000_state *stm = &dw->stm;
	unsigned long work;
	unsigned long flags;
	
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	work = stm->pending_work;
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);

	return work;
}

static int dw1000_event_thread(void *data)
{
	struct dw1000 *dw = data;
	struct dw1000_state *stm = &dw->stm;
	unsigned long pending_work = 0;
	unsigned sys_status = 0;
	int rc;

	/* State is now OFF */
	dw1000_set_state(dw, DW1000_STATE_OFF);

	/* Run until stopped */
	while (!kthread_should_stop()) {
 again:
		/* Read status if applicable */
		switch (stm->mstate)
		{
		    case DW1000_STATE_IDLE:
		    case DW1000_STATE_RX:
		    case DW1000_STATE_TX:
			/* Read system status */
			if ((rc = regmap_read(dw->sys_status.regs,
					      0, &sys_status)) != 0) {
				dw1000_stats_inc(dw, DW1000_STATS_SPI_ERROR);
				dw1000_enqueue(dw, DW1000_ERROR_WORK);
				goto again_nospi;
			}
			break;
		    default:
			sys_status = 0;
			break;
		}

		INJECT_ERROR(dw, STATE_SYS_STATUS, sys_status);

		/* State independent errors */
		if (sys_status & DW1000_SYS_STATUS_ERROR) {
			if (sys_status & DW1000_SYS_STATUS_RFPLL_LL)
				dev_err(dw->dev, "RFPLL sync lost\n");
			if (sys_status & DW1000_SYS_STATUS_CLKPLL_LL)
				dev_err(dw->dev, "CLKPLL sync lost\n");
			dw1000_game_over(dw);
			goto again_nospi;
		}
		
 again_nospi:
		/* Pending work items */
		pending_work = dw1000_get_pending_work(dw);

		/* SPI/HW errors */
		if (pending_work & DW1000_ERROR_WORK) {
			dev_err(dw->dev, "error detected; resetting\n");
			dw1000_reload(dw);
			dw1000_dequeue(dw, DW1000_ERROR_WORK);
			goto again_nospi;
		}

		INJECT_ERROR(dw, STATE_DELAY);

		/* State dependent handling */
		switch (stm->mstate) {
		case DW1000_STATE_OFF:
			/* Handle pending work */
			if (pending_work) {
				/* Remove reguested */
				if (pending_work & DW1000_KILL_WORK) {
					dw1000_dequeue(dw, DW1000_KILL_WORK);
					goto again_nospi;
				}
				/* Init reguested */
				if (pending_work & DW1000_INIT_WORK) {
					dw1000_poweron(dw);
					dw1000_init(dw);
					dw1000_dequeue(dw, DW1000_INIT_WORK);
					goto again;
				}
			}
			break;
			
		case DW1000_STATE_IDLE:
			/* Handle pending work */
			if (pending_work) {
				/* Remove reguested */
				if (pending_work & DW1000_KILL_WORK) {
					dw1000_remove(dw);
					dw1000_dequeue(dw, DW1000_KILL_WORK);
					goto again;
				}
				/* Reconfigure reguested */
				if (pending_work & DW1000_CONFIG_WORK) {
					dw1000_load_config(dw);
					dw1000_dequeue(dw, DW1000_CONFIG_WORK);
					goto again;
				}
				/* PTP work */
				if (pending_work & DW1000_PTP_WORK) {
					dw1000_ptp_sync(dw);
					dw1000_dequeue(dw, DW1000_PTP_WORK);
					goto again;
				}
				/* ADC work */
				if (pending_work & DW1000_ADC_WORK) {
					dw1000_sar_read(dw);
					dw1000_dequeue(dw, DW1000_ADC_WORK);
					goto again;
				}
				/* Timer expired */
				if (pending_work & DW1000_TIMER_WORK) {
					dw1000_set_state(dw, DW1000_STATE_IDLE);
					dw1000_dequeue(dw, DW1000_TIMER_WORK);
					goto again_nospi;
				}
				/* Stack work */
				if (pending_work & DW1000_STACK_WORK) {
					dw1000_dequeue(dw, DW1000_STACK_WORK);
					goto again_nospi;
				}
			}
			
			/* ieee802154 stack enabled */
			if (stm->ieee802154_enabled == true) {
				dw1000_enable(dw);
				goto again;
			}
			break;
			
		case DW1000_STATE_RX:
			/* Handle receiver activity */
			if (sys_status & DW1000_SYS_STATUS_RXOVRR) {
				dw1000_stats_inc(dw, DW1000_STATS_RX_OVRR);
				dw1000_rx_recover(dw);
				goto again;
			}
			if (sys_status & DW1000_SYS_STATUS_RXDFR) {
				dw1000_rx_frame(dw);
				goto again;
			}
			
			/* Handle pending work */
			if (pending_work) {
				/* Remove reguested */
				if (pending_work & DW1000_KILL_WORK) {
					dw1000_remove(dw);
					dw1000_dequeue(dw, DW1000_KILL_WORK);
					goto again;
				}
				/* Reconfigure requested */
				if (pending_work & DW1000_CONFIG_WORK) {
					dw1000_disable(dw);
					/* Continue in IDLE state */
					goto again;
				}
				/* TX requested */
				if (pending_work & DW1000_TX_WORK) {
					dw1000_tx_frame_start(dw);
					dw1000_dequeue(dw, DW1000_TX_WORK);
					goto again;
				}
				/* PTP work */
				if (pending_work & DW1000_PTP_WORK) {
					dw1000_ptp_sync(dw);
					dw1000_dequeue(dw, DW1000_PTP_WORK);
					goto again;
				}
				/* ADC work */
				if (pending_work & DW1000_ADC_WORK) {
					dw1000_disable(dw);
					/* Continue in IDLE state */
					goto again;
				}
				/* Timer expired */
				if (pending_work & DW1000_TIMER_WORK) {
					dw1000_set_state(dw, DW1000_STATE_RX);
					dw1000_dequeue(dw, DW1000_TIMER_WORK);
					goto again_nospi;
				}
				/* Stack work */
				if (pending_work & DW1000_STACK_WORK) {
					dw1000_dequeue(dw, DW1000_STACK_WORK);
					goto again_nospi;
				}
			}
			
			/* ieee802154 stack disabled */
			if (stm->ieee802154_enabled == false) {
				dw1000_disable(dw);
				goto again;
			}
			break;
			
		case DW1000_STATE_TX:
			/* Handle IRQ activity */
			if (sys_status & DW1000_SYS_STATUS_TXFRS) {
				dw1000_tx_frame_sent(dw);
				goto again;
			}
			
			/* Handle pending work */
			if (pending_work) {
				/* Timer expired */
				if (pending_work & DW1000_TIMER_WORK) {
					dw1000_tx_error(dw);
					dw1000_dequeue(dw, DW1000_TIMER_WORK);
					goto again;
				}
			}
			break;
			
		case DW1000_STATE_DEAD:
			/* Ignore all work */
			if (pending_work) {
				dw1000_dequeue(dw, pending_work);
				goto again_nospi;
			}
			break;
			
		default:
			/* Should not be here */
			dev_err(dw->dev,
				"Internal error: state mismatch (%d)\n",
				stm->mstate);
			dw1000_game_over(dw);
			break;
		}
		
		/* Wait for more work */
		dw1000_wait_pending_work(dw);
	}

	/* Make sure device is off */
	dw1000_remove(dw);
	
	dev_dbg(dw->dev, "thread finished\n");
	return 0;
}


/******************************************************************************
 *
 * Device probe
 *
 */

/**
 * dw1000_spi_probe() - Probe device on SPI
 *
 * @spi:		SPI device
 * @return:		0 on success or -errno
 */
static int dw1000_spi_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct dw1000 *dw;
	struct dw1000_config *cfg;
	int irqf;
	int rc;

	/* Allocate IEEE 802.15.4 device */
	hw = ieee802154_alloc_hw(sizeof(*dw), &dw1000_ops);
	if (!hw) {
		rc = -ENOMEM;
		goto err_alloc_hw;
	}
	hw->parent = &spi->dev;
	spi_set_drvdata(spi, hw);

	dw = hw->priv;
	dw->hw = hw;
	dw->spi = spi;
	dw->dev = &spi->dev;

	dev_dbg(dw->dev, "Loading driver\n");
	
	/* Radio configuration */
	cfg = &dw->cfg;
	cfg->xtalt = DW1000_FS_XTALT_XTALT_MIDPOINT;
	cfg->channel = DW1000_CHANNEL_DEFAULT;
	cfg->prf = DW1000_PRF_64M;
	cfg->rate = DW1000_RATE_6800K;
	cfg->txpsr = DW1000_TXPSR_DEFAULT;
	cfg->smart_power = true;
	cfg->frame_filter = DW1000_SYS_CFG_FFA;
	mutex_init(&cfg->mutex);

	/* Timestamp quality thresholds */
	dw->snr_threshold = DW1000_SNR_THRESHOLD_DEFAULT;
	dw->fpr_threshold = DW1000_FPR_THRESHOLD_DEFAULT;
	dw->noise_threshold = DW1000_NOISE_THRESHOLD_DEFAULT;

	/* Report capabilities */
	hw->flags = (IEEE802154_HW_TX_OMIT_CKSUM |
		     IEEE802154_HW_AFILT |
		     IEEE802154_HW_PROMISCUOUS |
		     IEEE802154_HW_RX_DROP_BAD_CKSUM);
	hw->phy->supported.channels[DW1000_CHANNEL_PAGE] = DW1000_CHANNELS;
	hw->phy->current_page = DW1000_CHANNEL_PAGE;
	hw->phy->current_channel = DW1000_CHANNEL_DEFAULT;

	/* Initialise reset GPIO pin as output */
	dw->reset_gpio = of_get_named_gpio(dw->dev->of_node, "reset-gpio", 0);
	if (gpio_is_valid(dw->reset_gpio)) {
		if ((rc = devm_gpio_request_one(dw->dev, dw->reset_gpio,
						GPIOF_DIR_OUT |
						GPIOF_OPEN_DRAIN |
						GPIOF_INIT_LOW,
						"dw1000-reset")) < 0)
			goto err_req_reset_gpio;
	} else {
		dev_warn(dw->dev, "device does not support GPIO RESET control");
	}

	/* Initialise power GPIO pin as output */
	dw->power_gpio = of_get_named_gpio(dw->dev->of_node, "power-gpio", 0);
	if (gpio_is_valid(dw->power_gpio)) {
		if ((rc = devm_gpio_request_one(dw->dev, dw->power_gpio,
						GPIOF_DIR_OUT |
						GPIOF_INIT_LOW,
						"dw1000-power")) < 0)
			goto err_req_power_gpio;
	} else {
		dev_warn(dw->dev, "device does not support GPIO POWER control");
	}

	/* Hook interrupt */
	irqf = irq_get_trigger_type(spi->irq);
        if (!irqf)
                irqf = IRQF_TRIGGER_HIGH;
	if ((rc = devm_request_irq(dw->dev, spi->irq, dw1000_irq_handler,
				   irqf, dev_name(dw->dev), dw)) != 0) {
		dev_err(dw->dev, "could not request IRQ %d: %d\n", spi->irq, rc);
		goto err_request_irq;
	}

	/* Initialise register maps */
	if ((rc = dw1000_regmap_init(dw)) != 0)
		goto err_regmap_init;
	if ((rc = dw1000_otp_regmap_init(dw)) != 0)
		goto err_otp_regmap_init;

	/* Initialise state descriptor */
	if ((rc = dw1000_state_init(dw)) != 0) {
		dev_err(dw->dev, "STATE initialisation failed: %d\n", rc);
		goto err_state_init;
	}
	/* Initialise sarc descriptor */
	if ((rc = dw1000_sar_init(dw)) != 0) {
		dev_err(dw->dev, "SAR initialisation failed: %d\n", rc);
		goto err_sar_init;
	}
	/* Initialise transmit descriptor */
	if ((rc = dw1000_tx_init(dw)) != 0) {
		dev_err(dw->dev, "TX initialisation failed: %d\n", rc);
		goto err_tx_init;
	}
	/* Initialise receive descriptor */
	if ((rc = dw1000_rx_init(dw)) != 0) {
		dev_err(dw->dev, "RX initialisation failed: %d\n", rc);
		goto err_rx_init;
	}

	/* Turn on power */
	if ((rc = dw1000_poweron(dw)) != 0) {
		dev_err(dw->dev, "Device power on failed: %d\n", rc);
		goto err_power;
	}
	/* Soft reset */
	if ((rc = dw1000_reset(dw)) != 0) {
		dev_err(dw->dev, "Device reset failed: %d\n", rc);
		goto err_reset;
	}
	
	/* Load OTP and DT */
	if ((rc = dw1000_load_xtalt(dw)) != 0) {
		dev_err(dw->dev, "Load OTP/DT XTALT failed: %d\n", rc);
		goto err_load_otp;
	}
	if ((rc = dw1000_load_ldotune(dw)) != 0) {
		dev_err(dw->dev, "Load OTP/DT LDOTUNE failed: %d\n", rc);
		goto err_load_otp;
	}
	if ((rc = dw1000_load_sar(dw)) != 0) {
		dev_err(dw->dev, "Load OTP/DT SAR failed: %d\n", rc);
		goto err_load_otp;
	}
	if ((rc = dw1000_load_eui64(dw)) != 0) {
		dev_err(dw->dev, "Load OTP/DT EUI64 failed: %d\n", rc);
		goto err_load_otp;
	}
	if ((rc = dw1000_load_delays(dw)) != 0) {
		dev_err(dw->dev, "Load OTP/DT delays failed: %d\n", rc);
		goto err_load_otp;
	}
	if ((rc = dw1000_load_profiles(dw)) != 0) {
		dev_err(dw->dev, "Load OTP/DT profiles failed: %d\n", rc);
		goto err_load_otp;
	}
	
	/* Register PTP clock */
	if ((rc = dw1000_ptp_init(dw)) != 0) {
		dev_err(dw->dev, "could not register ptp clock: %d\n", rc);
		goto err_ptp_init;
	}
	
	/* Register hardware monitor */
	dw->hwmon = devm_hwmon_device_register_with_info(dw->dev,
							 "dw1000", dw,
							 &dw1000_hwmon_chip,
							 NULL);
	if (IS_ERR(dw->hwmon)) {
		rc = PTR_ERR(dw->hwmon);
		dev_err(dw->dev, "could not register hwmon: %d\n", rc);
		goto err_register_hwmon;
	}

	/* Add attribute groups */
	if ((rc = sysfs_create_group(&dw->dev->kobj,
				     &dw1000_attr_group)) != 0) {
		dev_err(dw->dev, "could not create sysfs attributes: %d\n", rc);
		goto err_create_group;
	}
	if ((rc = sysfs_merge_group(&dw->dev->kobj,
				    &dw1000_stats_attr_group)) != 0) {
		dev_err(dw->dev, "could not merge sysfs attributes: %d\n", rc);
		goto err_merge_stats_group;
	}

	/* Register IEEE 802.15.4 device */
	if ((rc = ieee802154_register_hw(hw)) != 0) {
		dev_err(dw->dev,
			"could not register IEEE 802.15.4 device: %d\n", rc);
		goto err_register_hw;
	}

	/* Start state machine */
	dw1000_state_start(dw);
	
	return 0;

 err_register_hw:
	sysfs_unmerge_group(&dw->dev->kobj, &dw1000_stats_attr_group);
 err_merge_stats_group:
	sysfs_remove_group(&dw->dev->kobj, &dw1000_attr_group);
 err_create_group:
 err_register_hwmon:
 err_ptp_init:
 err_load_otp:
 err_reset:
 err_power:
 err_rx_init:
 err_tx_init:
 err_sar_init:
 err_state_init:
 err_otp_regmap_init:
 err_regmap_init:
 err_request_irq:
 err_req_power_gpio:
 err_req_reset_gpio:
	ieee802154_free_hw(hw);
 err_alloc_hw:
	return rc;
}

/**
 * dw1000_spi_remove() - Remove SPI device
 *
 * @spi:		SPI device
 * @return:		0 on success or -errno
 */
static int dw1000_spi_remove(struct spi_device *spi)
{
	struct ieee802154_hw *hw = spi_get_drvdata(spi);
	struct dw1000 *dw = hw->priv;

	/* Remove sysfs nodes */
	sysfs_unmerge_group(&dw->dev->kobj, &dw1000_stats_attr_group);
	sysfs_remove_group(&dw->dev->kobj, &dw1000_attr_group);
	
	/* Unregister subsystems */
	ieee802154_unregister_hw(hw);
	dw1000_ptp_remove(dw);
	dw1000_state_stop(dw);
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
	.probe = dw1000_spi_probe,
	.remove = dw1000_spi_remove,
};
module_spi_driver(dw1000_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Brown <mbrown@fensystems.co.uk>");
MODULE_AUTHOR("Petri Mattila <petri.mattila@unipart.io>");
MODULE_DESCRIPTION("DecaWave DW1000 IEEE 802.15.4 driver");
