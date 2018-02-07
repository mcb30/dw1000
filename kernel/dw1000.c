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
 * Register map
 *
 */

static const struct regmap_config dw1000_regmap_dword = {
	.name = "dw1000-dword",
	.reg_bits = DW1000_DWORD_REG_BITS,
	.pad_bits = DW1000_DWORD_PAD_BITS,
	.val_bits = DW1000_DWORD_VAL_BITS,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.write_flag_mask = DW1000_WRITE_FLAG_MASK,
	.read_flag_mask = DW1000_READ_FLAG_MASK,
};

static int dw1000_regmap_init(struct dw1000 *dw)
{
	int rc;

	/* Initialise dword register map */
	dw->regs.dword = devm_regmap_init_spi(dw->spi, &dw1000_regmap_dword);
	if (IS_ERR(dw->regs.dword)) {
		rc = PTR_ERR(dw->regs.dword);
		dev_err(&dw->spi->dev, "could not allocate dword map: %d\n", rc);
		return rc;
	}

	return 0;
}

/********************************************************************************
 *
 * Device initialisation
 *
 */

static int dw1000_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct dw1000 *dw;
	unsigned int id;
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

	/* Initialise register map */
	rc = dw1000_regmap_init(dw);
	if (rc)
		goto err_regmap_init;

	/* Read device ID register */
	rc = regmap_read(dw->regs.dword, DW1000_DEV_ID, &id);
	if (rc) {
		dev_err(&spi->dev, "could not read device ID: %d\n", rc);
		goto err_dev_id;
	}
	dev_info(&spi->dev, "found %08X using IRQ %d\n", id, spi->irq);

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
 err_dev_id:
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
