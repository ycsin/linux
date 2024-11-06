// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C Support for Andes ATCIIC100 two-wire interface (TWI)
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Copyright (C) 2016 Andes Technology Corporation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>

#define SUPPORT_AT24C128_BY_BOARD	0

#define DRV_NAME	"atciic100"
#define EMPTY_STR

/* ID and Revision Register */
#define IDREV	0x00
#define ID_OFF	12
#define ID_MSK	0xfffff
#define ID	0x02021

/* Configuration Register */
#define CFG	0x10
#define FIFOSIZE_MASK	0x3

/* INterrupt Enable Register */
#define INTEN	0x14

/* Status Register */
#define STA	0x18

#define LINESDA	(1 << 14)
#define LINESCL	(1 << 13)
#define GENCALL	(1 << 12)
#define BBUSY	(1 << 11)
#define ACK	(1 << 10)
#define COMPL	(1 << 9)
#define BRECV	(1 << 8)
#define BTRAN	(1 << 7)
#define START	(1 << 6)
#define STOP	(1 << 5)
#define ALOSE	(1 << 4)
#define ADDRHIT	(1 << 3)
#define FIFOHAF	(1 << 2)
#define FIFOFUL	(1 << 1)
#define FIFOETY	(1 << 0)
#define STAW1C	(COMPL | BRECV | BTRAN | START | STOP | ALOSE | ADDRHIT)

/* Address Register */
#define ADR	0x1c

/* Data Register */
#define DAT	0x20

/* Control Register */
#define CTL	0x24
#define PSTART	(1 << 12)
#define PADDR	(1 << 11)
#define PDATA	(1 << 10)
#define PSTOP	(1 << 9)
#define DIR_RX	(1 << 8)
#define DCNTMSK	0xff

/* Command Register */
#define CMD	0x28
#define ISSUE_DATA	0x1
#define RESPOND_ACK	0x2
#define RESPOND_NACK	0x3
#define CLEAR_FIFO	0x4
#define RESET_IIC	0x5

/* Setup Register */
#define SETUP	0x2c
#define MASTER	(1 << 2)
#define ADDR10	(1 << 1)
#define IICEN	(1 << 0)

#define xget (buf[i++] = IIC(DAT))
#define xput (IIC(DAT) = buf[i++])

#define XFER_TIMEOUT	0xf000
#define MODE_PIO	0
#define MODE_IRQ	1
#ifdef CONFIG_ATCIIC_PIO
#define XFER_MODE	MODE_PIO
#else
#define XFER_MODE	MODE_IRQ
#endif

#ifdef CONFIG_ATCIIC100_DEBUG
#define PRINTK printk
#else
#define PRINTK(x...)
#endif

#define IIC(reg) (*(volatile u32 *)(iface->regs_base + reg))

#define xwait(c, b, eq, m)			\
do {						\
int cnt = c;					\
while ((IIC(STA)&b) eq b)			\
if (!(cnt--)) {					\
	PRINTK(KERN_INFO "wait %s error\n", m);	\
	return -EIO;				\
} 						\
} while (0)

#define xfer_wait(c, b, eq, m, act1, act2)	\
while (length--) {				\
	act1;					\
	xwait(c, b, eq, m);			\
	act2;					\
}

struct atciic {
	int			irq;
	spinlock_t		lock;
	char			read_write;
	u8			fifolen;
	u8			mode;
	u8			command;
	u8			*ptr;
	int			readNum;
	int			writeNum;
	int			cur_mode;
	int			addr_hit;
	int			cmpl;
	int			manual_stop;
	int			result;
	struct i2c_adapter	adap;
	struct completion	complete;
	struct i2c_msg		*pmsg;
	int			msg_num;
	int			cur_msg;
	u16			saved_clkdiv;
	u16			saved_control;
	void __iomem		*regs_base;
	struct resource		*res;
};

static u32 xrecev(struct atciic *iface)
{
	u32 status_wait = 0;

	if (iface->readNum >= iface->fifolen) {
		IIC(INTEN) = (COMPL | FIFOFUL);
		status_wait = FIFOFUL;
	} else {
		IIC(INTEN) = COMPL;
		status_wait = COMPL;
	}
	return status_wait;
}

static irqreturn_t atciic_irq(int irq, void *dev_id)
{
	struct atciic *iface = dev_id;
	int i = 0;
	unsigned long flags;
	u32 status = IIC(STA);
	u32 rleft = (IIC(CTL)&0xff);
	char flen = iface->fifolen;
	unsigned char *buf = iface->ptr;

	spin_lock_irqsave(&iface->lock, flags);
	IIC(STA) = status;
	if (status & ADDRHIT)
		iface->addr_hit = 1;

	if (iface->pmsg->flags & I2C_M_RD) {
		if (status & xrecev(iface)) {
			flen = iface->readNum - rleft;
			iface->readNum -= flen;
			while (flen--)
				xget;
			xrecev(iface);
		}
	} else {
		IIC(INTEN) = (COMPL | FIFOETY);
		if (iface->writeNum <= flen)
			flen = iface->writeNum;

		if (status & FIFOETY) {
			iface->writeNum -= flen;
			while (flen--)
				xput;
		}
	}
	iface->ptr += i;
	spin_unlock_irqrestore(&iface->lock, flags);
	if (status & COMPL) {
		IIC(INTEN) = 0;
		iface->cmpl = 1;
		if (iface->addr_hit == 1)
			complete(&iface->complete);
	}
	return IRQ_HANDLED;
}

static int irq_transfer(struct atciic *iface)
{
	if (!wait_for_completion_timeout(&iface->complete, iface->adap.timeout)) {
		iface->result = -EIO;
		PRINTK(KERN_INFO
			"%s(%s) fail, addrhit 0x%x, cmpl 0x%x, %s %d, int 0x%lx, status 0x%lx\n",
			(iface->pmsg->flags & I2C_M_RD) ? "read" : "write",
			iface->mode ? "irq" : "pio",
			iface->addr_hit,
			iface->cmpl,
			(iface->pmsg->flags & I2C_M_RD) ? "readNum" : "writeNum",
			(iface->pmsg->flags & I2C_M_RD) ? iface->readNum : iface->writeNum,
			IIC(INTEN), IIC(STA));
	}
	reinit_completion(&(iface->complete));
	return iface->result;
}

static int pio_transfer(struct atciic *iface)
{
	int length = iface->pmsg->len;
	unsigned char *buf = iface->pmsg->buf;
	int i = 0;

	if (iface->pmsg->flags & I2C_M_RD) {
		xfer_wait(XFER_TIMEOUT, FIFOETY, ==, "fifo empty", EMPTY_STR, xget);
		xwait(XFER_TIMEOUT, COMPL, !=, "cmpl");
	} else {
		xwait(XFER_TIMEOUT, ADDRHIT, !=, "addrhit");
		xfer_wait(XFER_TIMEOUT, FIFOFUL, ==, "fifofull", xput, EMPTY_STR);
		xwait(XFER_TIMEOUT, COMPL, !=, "cmpl");
	}
	return 0;
}

static int atciic_transfer(struct atciic *iface)
{
	if (iface->mode == MODE_PIO)
		return pio_transfer(iface);
	else if (iface->mode == MODE_IRQ)
		return irq_transfer(iface);
	else
		return -EIO;
}

/* Adjust timeout depend on transaction time. */
static void calculate_timeout(struct i2c_adapter *adap, int length)
{
	if (length > 4)
		adap->timeout = 3 * HZ;
	else
		adap->timeout = HZ/8;
}

/* Generic i2c master receive. */
static int atciic_read(struct i2c_adapter *adap, unsigned char *buf, int length)
{
	int rc = 0;
	struct atciic *iface = adap->algo_data;

	iface->readNum = length;
	iface->result = 0;
	IIC(CTL) &= ~0xff;
	IIC(CTL) |= ((length & 0xff) | DIR_RX);
	if (iface->mode == MODE_IRQ)
		IIC(INTEN) = (FIFOFUL | COMPL);
	IIC(CMD) = ISSUE_DATA;
	rc = atciic_transfer(iface);
	return rc;
}

/* Generic i2c master transmit. */
static int atciic_write(struct i2c_adapter *adap, unsigned char *buf, int length)
{
	int rc = 0;
	struct atciic *iface = adap->algo_data;

	iface->result = 0;
	iface->writeNum = length;
	IIC(CTL) &= ~(DIR_RX | DCNTMSK);
	IIC(CTL) |= (length&0xff);
	if (iface->mode == MODE_IRQ)
		IIC(INTEN) = (FIFOETY | COMPL);
	/* Write cycle time issue */
	mdelay(5);
	IIC(CMD) = ISSUE_DATA;
	rc = atciic_transfer(iface);
	return rc;
}

/* Generic i2c master transfer entrypoint. */
static int atciic_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int i, ret;
	u16 rw;
	int retry = adap->retries;
	struct atciic *iface = adap->algo_data;
	struct i2c_msg *msg;

	if ((pmsg->flags & I2C_M_RD) && (pmsg->len == 1))
		retry = 0;
xfer_start:
	msg = pmsg;
	dev_dbg(&adap->dev, "%s: processing %d messages:\n", __func__, num);
	for (i = 0; i < num; i++) {
		if (msg->len > 0x100)
			return -EIO;
		dev_dbg(&adap->dev, " #%d: %sing %d byte%s %s 0x%02x\n", i,
			pmsg->flags & I2C_M_RD ? "read" : "writ",
			pmsg->len, pmsg->len > 1 ? "s" : "",
			pmsg->flags & I2C_M_RD ? "from" : "to",	pmsg->addr);
		iface->addr_hit = 0;
		iface->cmpl = 0;
		iface->pmsg = msg;
		rw = iface->pmsg->flags & I2C_M_RD;
		iface->ptr = msg->buf;
		IIC(INTEN) = 0;
		IIC(ADR) = (msg->addr);
		if (msg->len && msg->buf) {	/* sanity check */
			calculate_timeout(adap, msg->len);

			if (msg->flags & I2C_M_RD)
				ret = atciic_read(adap, msg->buf, msg->len);
			else
				ret = atciic_write(adap, msg->buf, msg->len);

			IIC(STA) = STAW1C;
			if (ret) {
				if (retry--) {
					PRINTK(KERN_INFO "xfer %s(%s) retry\n",
						rw ? "read" : "write",
						iface->mode ? "irq" : "pio");
					goto xfer_start;
				} else
					return ret;
			}
		}
		dev_dbg(&adap->dev, "transfer complete\n");
		msg++;		/* next message */
	}
	return i;
}

/* Return list of supported functionality. */
static u32 atciic_func(struct i2c_adapter *adapter)
{
	return (u32)(I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
		I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE |
		I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
		I2C_FUNC_SMBUS_READ_WORD_DATA |	I2C_FUNC_SMBUS_READ_BLOCK_DATA |
		I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | I2C_FUNC_SMBUS_READ_I2C_BLOCK |
		I2C_FUNC_SMBUS_WRITE_I2C_BLOCK | I2C_FUNC_SMBUS_WRITE_WORD_DATA);
}

static struct i2c_algorithm atciic_algorithm = {
	.master_xfer	= atciic_xfer,
	.functionality	= atciic_func,
};

#if SUPPORT_AT24C128_BY_BOARD
#include <linux/i2c/at24.h>

static struct at24_platform_data at24c128 = {
	.byte_len	= 0x00020000,
	.page_size	= 64,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info ae300_i2c_devices[] __initdata = {
	{
		I2C_BOARD_INFO("24c128", 0x50),
		.platform_data = &at24c128,
	},
};

void __init ae300_add_device_i2c(struct i2c_board_info *devices, int nr_devices)
{
	i2c_register_board_info(0, devices, nr_devices);
}
#endif /* SUPPORT_AT24C128_BY_BOARD */

/* Main initialization routine. */
static int atciic_probe(struct platform_device *pdev)
{
	int (*read_fixup)(void __iomem *addr, unsigned int val,
		unsigned int shift_bits);
	struct i2c_adapter *padap;
	struct atciic *iface;
	int rc, ret;

#if SUPPORT_AT24C128_BY_BOARD
	ae300_add_device_i2c(ae300_i2c_devices, ARRAY_SIZE(ae300_i2c_devices));
#endif

	iface = kzalloc(sizeof(struct atciic), GFP_KERNEL);
	if (!iface) {
		rc = -ENOMEM;
		goto out_error_nomem;
	}
	spin_lock_init(&(iface->lock));
	iface->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (iface->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		rc = -ENOMEM;
		goto out_error_get_res;
	}

	if (!iface->res) {
		rc = -ENOMEM;
		goto out_error_get_res;
	}

	iface->regs_base = devm_ioremap_resource(&pdev->dev, iface->res);

	if (iface->regs_base == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		rc = -ENXIO;
		goto out_error_get_io;
	}

	/* Check ID and Revision Register */
	read_fixup = symbol_get(readl_fixup);
	ret = read_fixup(iface->regs_base, 0x020210, 8);
	symbol_put(readl_fixup);
	if (!ret) {
		dev_err(&pdev->dev, "I2C version NOT match, bitmap not support atciic100\n");
		rc = -ENXIO;
		goto out_error_get_io;
	}

	iface->irq = platform_get_irq(pdev, 0);
	if (iface->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		rc = -ENOENT;
		goto out_error_get_io;
	}

	rc = request_irq(iface->irq, atciic_irq, 0, pdev->name, iface);
	if (rc) {
		dev_err(&pdev->dev, "Can't get IRQ %d !\n", iface->irq);
		rc = -ENODEV;
		goto out_error_req_irq;
	}

	iface->mode = XFER_MODE;
	init_completion(&(iface->complete));
	padap = &iface->adap;
	strscpy(padap->name, pdev->name, sizeof(padap->name));
	padap->algo = &atciic_algorithm;
	padap->class = I2C_CLASS_HWMON;
	padap->dev.parent = &pdev->dev;
	padap->dev.of_node = pdev->dev.of_node;
	padap->algo_data = iface;
	padap->timeout = 3 * HZ;
	padap->retries = 1;
	padap->nr = -1;

	rc = i2c_add_numbered_adapter(padap);
	if (rc < 0) {
		dev_err(&pdev->dev, "Can't add i2c adapter!\n");
		goto out_error_add_adapter;
	}

	platform_set_drvdata(pdev, padap);
	padap = platform_get_drvdata(pdev);
	iface->fifolen = (1 << ((IIC(CFG) & FIFOSIZE_MASK) + 1));
	IIC(SETUP) |= (MASTER | IICEN);

	dev_info(&pdev->dev, "Andes i2c bus driver.\n");
	return 0;

out_error_add_adapter:
	free_irq(iface->irq, iface);
out_error_req_irq:
out_error_get_io:
	release_mem_region(iface->res->start, resource_size(iface->res));
out_error_get_res:
	kfree(iface);
out_error_nomem:
	return rc;
}

static int atciic_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	struct atciic *iface = adapter->algo_data;

	i2c_del_adapter(adapter);
	platform_set_drvdata(pdev, NULL);
	release_mem_region(iface->res->start, resource_size(iface->res));
	free_irq(iface->irq, iface);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id atc_iic_dt_match[] = {
	{ .compatible = "andestech,atciic100" },
	{},
};
MODULE_DEVICE_TABLE(of, atc_iic_dt_match);
#endif

static struct platform_driver atciic_platform_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(atc_iic_dt_match),
	},
	.probe		= atciic_probe,
	.remove		= atciic_remove,
};

/* I2C may be needed to bring up other drivers */
static int __init andes_i2c_init_driver(void)
{
	return platform_driver_register(&atciic_platform_driver);
}
subsys_initcall(andes_i2c_init_driver);

static void __exit andes_i2c_exit_driver(void)
{
	platform_driver_unregister(&atciic_platform_driver);
}
module_exit(andes_i2c_exit_driver);

MODULE_AUTHOR("Rick Chen");
MODULE_AUTHOR("Dylan Jhong");
MODULE_DESCRIPTION("I2C driver for Andes atciic100");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:atciic100");
