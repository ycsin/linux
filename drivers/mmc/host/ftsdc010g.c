// SPDX-License-Identifier: GPL-2.0
/* drivers/mmc/host/ftsdc010.c
 *  Copyright (C) 2021 Andestech
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include "ftsdc010g.h"
#include "../core/core.h"
#include <linux/highmem.h>
#include <linux/kernel.h>

#define DRIVER_NAME "ftsdc010g"
#define REG_READ(addr) readl((host->base + addr))
#define REG_WRITE(data, addr) writel((data), (host->base + addr))

enum dbg_channels {
	dbg_err   = (1 << 0),
	dbg_debug = (1 << 1),
	dbg_info  = (1 << 2),
	dbg_irq   = (1 << 3),
	dbg_sg    = (1 << 4),
	dbg_dma   = (1 << 5),
	dbg_pio   = (1 << 6),
	dbg_fail  = (1 << 7),
	dbg_conf  = (1 << 8),
};

static struct workqueue_struct *mywq;

static const int dbgmap_err   = dbg_fail;
static const int dbgmap_info  = dbg_info | dbg_conf;
static const int dbgmap_debug = dbg_err | dbg_debug | dbg_info | dbg_conf;
#define dbg(host, channels, args...)			\
do {						\
	if (dbgmap_err & channels)			\
		dev_err(&host->pdev->dev, args);	\
	else if (dbgmap_info & channels)		\
		dev_info(&host->pdev->dev, args);	\
	else if (dbgmap_debug & channels)		\
		dev_dbg(&host->pdev->dev, args);	\
} while (0)
static void ftsdc_send_request(struct mmc_host *mmc);
#ifdef CONFIG_MMC_DEBUG
static void dbg_dumpregs(struct ftsdc_host *host, char *prefix)
{
	u32 con, cmdarg, r0, r1, r2, r3, rcmd, dcon, dtimer,
	    dlen, sta, clr, imask, pcon, ccon, bwidth, scon1,
	    scon2, ssta, fea;

	con	= REG_READ(SDC_CMD_REG);
	cmdarg	= REG_READ(SDC_ARGU_REG);
	r0	= REG_READ(SDC_RESPONSE0_REG);
	r1	= REG_READ(SDC_RESPONSE1_REG);
	r2	= REG_READ(SDC_RESPONSE2_REG);
	r3	= REG_READ(SDC_RESPONSE3_REG);
	rcmd	= REG_READ(SDC_RSP_CMD_REG);
	dcon	= REG_READ(SDC_DATA_CTRL_REG);
	dtimer	= REG_READ(SDC_DATA_TIMER_REG);
	dlen	= REG_READ(SDC_DATA_LEN_REG);
	sta	= REG_READ(SDC_STATUS_REG);
	clr	= REG_READ(SDC_CLEAR_REG);
	imask	= REG_READ(SDC_INT_MASK_REG);
	pcon	= REG_READ(SDC_POWER_CTRL_REG);
	ccon	= REG_READ(SDC_CLOCK_CTRL_REG);
	bwidth	= REG_READ(SDC_BUS_WIDTH_REG);
	scon1	= REG_READ(SDC_SDIO_CTRL1_REG);
	scon2	= REG_READ(SDC_SDIO_CTRL2_REG);
	ssta	= REG_READ(SDC_SDIO_STATUS_REG);
	fea	= REG_READ(SDC_FEATURE_REG);

	dbg(host, dbg_debug,
		"%s CON:[%08x] STA:[%08x] INT:[%08x] PWR:[%08x] CLK:[%08x]\n",
		prefix, con, sta, imask, pcon, ccon);
	dbg(host, dbg_debug,
		"%s DCON:[%08x] DTIME:[%08x] DLEN:[%08x] DWIDTH:[%08x]\n",
		prefix, dcon, dtimer, dlen, bwidth);
	dbg(host, dbg_debug,
		"%s R0:[%08x]   R1:[%08x] R2:[%08x]   R3:[%08x]\n",
		prefix, r0, r1, r2, r3);
	dbg(host, dbg_debug,
		"%s SCON1:[%08x]   SCON2:[%08x] SSTA:[%08x]   FEA:[%08x]\n",
		prefix, scon1, scon2, ssta, fea);
}

static void prepare_dbgmsg(struct ftsdc_host *host, struct mmc_command *cmd,
			   int stop)
{
	snprintf(host->dbgmsg_cmd, 300,
		 "#%u%s op:%i arg:0x%08x flags:0x08%x retries:%u",
		 host->ccnt, (stop ? " (STOP)" : ""),
		 cmd->opcode, cmd->arg, cmd->flags, cmd->retries);

	if (cmd->data) {
		snprintf(host->dbgmsg_dat, 300,
			 "#%u bsize:%u blocks:%u bytes:%u",
			 host->dcnt, cmd->data->blksz,
			 cmd->data->blocks,
			 cmd->data->blocks * cmd->data->blksz);
	} else {
		host->dbgmsg_dat[0] = '\0';
	}
}

static void dbg_dumpcmd(struct ftsdc_host *host,
	struct mmc_command *cmd, int fail)
{
	unsigned int dbglvl = fail ? dbg_fail : dbg_debug;

	if (!cmd)
		return;

	if (cmd->error == 0) {
		dbg(host, dbglvl, "CMD[OK] %s R0:0x%08x\n",
			host->dbgmsg_cmd, cmd->resp[0]);
	} else {
		dbg(host, dbglvl, "CMD[ERR %i] %s Status:%s\n",
			cmd->error, host->dbgmsg_cmd, host->status);
	}

	if (!cmd->data)
		return;

	if (cmd->data->error == 0) {
		dbg(host, dbglvl, "DAT[OK] %s\n", host->dbgmsg_dat);
	} else {
		dbg(host, dbglvl, "DAT[ERR %i] %s DCNT:0x%08x\n",
			cmd->data->error, host->dbgmsg_dat,
			REG_READ(SDC_DATA_LEN_REG));
	}
}
#else
static void dbg_dumpcmd(struct ftsdc_host *host,
	struct mmc_command *cmd, int fail)
{
}

static void prepare_dbgmsg(struct ftsdc_host *host,
	struct mmc_command *cmd, int stop)
{
}

static void dbg_dumpregs(struct ftsdc_host *host, char *prefix)
{
}

#endif /* CONFIG_MMC_DEBUG */

static inline bool ftsdc_dmaexist(struct ftsdc_host *host)
{
	return (host->dma.chan != NULL);
}

static inline u32 enable_imask(struct ftsdc_host *host, u32 imask)
{
	u32 newmask;

	newmask = REG_READ(SDC_INT_MASK_REG);
	newmask |= imask;

	REG_WRITE(newmask, SDC_INT_MASK_REG);

	return newmask;
}

static inline u32 disable_imask(struct ftsdc_host *host, u32 imask)
{
	u32 newmask;

	newmask = REG_READ(SDC_INT_MASK_REG);
	newmask &= ~imask;

	REG_WRITE(newmask, SDC_INT_MASK_REG);

	return newmask;
}

static inline void clear_imask(struct ftsdc_host *host)
{
	u32 mask = REG_READ(SDC_INT_MASK_REG);

	/* preserve the SDIO IRQ mask state */
	mask &= (SDC_INT_MASK_REG_SDIO_INTR | SDC_INT_MASK_REG_CARD_CHANGE);
	REG_WRITE(mask, SDC_INT_MASK_REG);
}

static inline void get_data_buffer(struct ftsdc_host *host)
{
	struct scatterlist *sg;

	WARN_ON(host->buf_sgptr >= host->mrq->data->sg_len);
	sg = &host->mrq->data->sg[host->buf_sgptr];
	host->buf_page = 0;
	host->buf_bytes = sg->length;
	host->buf_ptr = host->dodma ?
		(u32 *)(unsigned long)sg->dma_address : sg_virt(sg);
#ifdef CONFIG_HIGHMEM
	if (PageHighMem(sg_page(sg))) {
		host->buf_page = sg_page(sg);
		host->buf_offset = sg->offset;
		if (!host->dodma)
			host->buf_ptr = 0;
	}
#endif
	host->buf_sgptr++;
}

static inline u32 cal_blksz(unsigned int blksz)
{
	u32 blksztwo = 0;

	while (blksz >>= 1)
		blksztwo++;

	return blksztwo;
}

/**
 * ftsdc_enable_irq - enable IRQ, after having disabled it.
 * @host: The device state.
 * @more: True if more IRQs are expected from transfer.
 *
 * Enable the main IRQ if needed after it has been disabled.
 *
 * The IRQ can be one of the following states:
 *	- enable after data read/write
 *	- disable when handle data read/write
 */
static void ftsdc_enable_irq(struct ftsdc_host *host, bool enable)
{
	unsigned long flags;

	local_irq_save(flags);
	host->irq_enabled = enable;
	if (enable)
		enable_irq(host->irq);
	else
		disable_irq(host->irq);
	local_irq_restore(flags);
}

static void do_pio_read(struct ftsdc_host *host)
{
	u32 fifo;
	u32 fifo_words;
	u32 *ptr = 0;
	void *tptr = 0;
	u32 status;
	u32 retry = 0;

	WARN_ON(host->buf_bytes != 0);
	while (host->buf_sgptr < host->mrq->data->sg_len) {
		get_data_buffer(host);
		tptr = host->buf_ptr;
		while (host->buf_bytes) {
			host->page_cnt = host->buf_bytes;
			if (host->buf_page != 0) {
				host->buf_ptr = kmap_atomic(host->buf_page);
				tptr = host->buf_ptr;
				tptr +=  host->buf_offset;
				host->page_cnt = min((u32)(PAGE_SIZE-host->buf_offset),
					host->buf_bytes);
				host->buf_offset = 0;
			}
			host->buf_bytes -= host->page_cnt;
			ptr = tptr;
			while (host->page_cnt) {
				status = REG_READ(SDC_STATUS_REG);
				if (status & SDC_STATUS_REG_FIFO_OVERRUN) {
					fifo = host->fifo_len > host->page_cnt ?
					host->page_cnt : host->fifo_len;
					dbg(host, dbg_pio,
					"pio_read(): fifo:[%02i] buffer:[%03i] dcnt:[%08X]\n",
					fifo, host->page_cnt,
					REG_READ(SDC_DATA_LEN_REG));
					host->page_cnt -= fifo;
					host->buf_count += fifo;
					fifo_words = fifo >> 2;
					while (fifo_words--)
						*ptr++ = REG_READ(SDC_DATA_WINDOW_REG);
					if (fifo & 3) {
						u32 n;
						u32 data;
						u8 *p;

						n = fifo & 3;
						data = REG_READ(SDC_DATA_WINDOW_REG);
						p = (u8 *)ptr;
						while (n--) {
							*p++ = data;
							data >>= 8;
						}
					}
				} else {
					udelay(1);
					if (++retry >= SDC_PIO_RETRY) {
						host->mrq->data->error = -EIO;
						goto err;
					}
				}
			}
			if (host->buf_page != 0) {
				kunmap_atomic(host->buf_ptr);
				host->buf_page++;
			}
		}
	}
err:
	host->buf_active = XFER_NONE;
	host->complete_what = COMPLETION_FINALIZE;
}

static void do_pio_write(struct ftsdc_host *host)
{
	u32 fifo;
	u32 *ptr;
	void *tptr;
	u32 status;
	u32 retry = 0;

	WARN_ON(host->buf_bytes != 0);
	while (host->buf_sgptr < host->mrq->data->sg_len) {
		get_data_buffer(host);
		dbg(host, dbg_pio,
		    "pio_write(): new source: [%i]@[%p]\n",
		    host->buf_bytes, host->buf_ptr);
		while (host->buf_bytes) {
			host->page_cnt = host->buf_bytes;
			tptr = host->buf_ptr;
			if (host->buf_page != 0) {
				host->buf_ptr = kmap_atomic(host->buf_page);
				host->page_cnt = min((u32)(PAGE_SIZE-host->buf_offset),
					host->page_cnt);
				tptr = host->buf_ptr;
				tptr += host->buf_offset;
				host->buf_offset = 0;
			}
			host->buf_bytes -= host->page_cnt;
			ptr = tptr;
			while (host->page_cnt) {
				status = REG_READ(SDC_STATUS_REG);
				if (status & SDC_STATUS_REG_FIFO_UNDERRUN) {
					fifo = host->fifo_len > host->page_cnt ?
					host->page_cnt : host->fifo_len;
					dbg(host, dbg_pio,
					"pio_write(): fifo:[%02i] buffer:[%03i] dcnt:[%08X]\n",
					fifo, host->buf_bytes,
					REG_READ(SDC_DATA_LEN_REG));
					host->page_cnt -= fifo;
					host->buf_count += fifo;
					fifo = (fifo + 3) >> 2;
					while (fifo--) {
						REG_WRITE(*ptr, SDC_DATA_WINDOW_REG);
						ptr++;
					}
				} else {
					udelay(1);
					if (++retry >= SDC_PIO_RETRY) {
						host->mrq->data->error = -EIO;
						goto err;
					}
				}
			}
			if (host->buf_page != 0) {
				kunmap_atomic(host->buf_ptr);
				host->buf_page++;
			}
		}
	}

err:
	host->buf_active = XFER_NONE;
	host->complete_what = COMPLETION_FINALIZE;
}

static void ftsdc_dma_cleanup(struct ftsdc_host *host)
{
	struct mmc_data                 *data = host->mrq->data;

	if (data)
		dma_unmap_sg(host->dma.chan->device->dev,
			data->sg, data->sg_len, mmc_get_dma_dir(data));
}

static int ftsdc_configure_dma(struct ftsdc_host *host)
{
	host->dma.chan = dma_request_chan(&host->pdev->dev,
		"rxtx");
	if (PTR_ERR(host->dma.chan) == -ENODEV) {
		struct ftsdc_mmc_config *pdata = host->pdev->dev.platform_data;
		dma_cap_mask_t mask;

		if (!pdata || !pdata->dma_filter)
			return -ENODEV;

		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		host->dma.chan = dma_request_channel(mask, pdata->dma_filter,
			pdata->dma_slave);
		if (!host->dma.chan)
			host->dma.chan = ERR_PTR(-ENODEV);
	}
	if (IS_ERR(host->dma.chan))
		return PTR_ERR(host->dma.chan);

	dev_info(&host->pdev->dev, "using %s for DMA transfers\n",
		dma_chan_name(host->dma.chan));
	host->dma_conf.src_addr = host->mem->start + SDC_DATA_WINDOW_REG;
	host->dma_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	host->dma_conf.src_maxburst = 4;
	host->dma_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	host->dma_conf.dst_maxburst = 4;

	return 0;
}


static void ftsdc_dma_complete(void *arg,
	const struct dmaengine_result *result)
{
	struct ftsdc_host	*host = arg;
	struct mmc_data		*data = host->mrq->data;

	host->dma_finish = true;
	data->error = result->result;
	host->buf_active = XFER_NONE;
	host->complete_what = COMPLETION_FINALIZE;
	ftsdc_dma_cleanup(host);
	ftsdc_enable_irq(host, true);
	tasklet_schedule(&host->pio_tasklet);
}

static u32 ftsdc_prepare_data_dma(struct ftsdc_host *host,
	struct mmc_data *data)
{
	struct dma_chan			*chan;
	struct dma_async_tx_descriptor	*desc;
	enum dma_transfer_direction	slave_dirn;
	unsigned int			sglen;

	chan = host->dma.chan;
	if (!chan)
		return -ENODEV;

	if (data->flags & MMC_DATA_READ) {
		host->dma_conf.direction = slave_dirn = DMA_DEV_TO_MEM;
		host->dma_conf.src_addr = host->mem->start
			+ SDC_DATA_WINDOW_REG;
	} else {
		host->dma_conf.direction = slave_dirn = DMA_MEM_TO_DEV;
		host->dma_conf.dst_addr = host->mem->start
			+ SDC_DATA_WINDOW_REG;
	}
	sglen = dma_map_sg(chan->device->dev, data->sg,
			data->sg_len, mmc_get_dma_dir(data));
	dmaengine_slave_config(chan, &host->dma_conf);
	desc = dmaengine_prep_slave_sg(chan,
			data->sg, sglen, slave_dirn,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		goto unmap_exit;
	host->dma.data_desc = desc;
	desc->callback_result = ftsdc_dma_complete;
	desc->callback_param = host;

	return 0;

unmap_exit:
	dma_unmap_sg(chan->device->dev, data->sg, data->sg_len,
		     mmc_get_dma_dir(data));

	return -ENOMEM;
}


static void
ftsdc_submit_data_dma(struct ftsdc_host *host)
{
	struct dma_chan	*chan = host->dma.chan;
	struct dma_async_tx_descriptor	*desc = host->dma.data_desc;

	if (chan) {
		dmaengine_submit(desc);
		dma_async_issue_pending(chan);
	}
}

static void ftsdc_work(struct work_struct *work)
{
	struct ftsdc_host *host =
		container_of(work, struct ftsdc_host, work);
	struct mmc_data *data = host->mrq->data;

	ftsdc_enable_irq(host, false);
	if (host->dodma) {
		get_data_buffer(host);
		ftsdc_prepare_data_dma(host, data);
		ftsdc_submit_data_dma(host);
	} else {
		if (host->buf_active == XFER_WRITE)
			do_pio_write(host);

		if (host->buf_active == XFER_READ)
			do_pio_read(host);
		ftsdc_enable_irq(host, true);
		tasklet_schedule(&host->pio_tasklet);
	}
}

static void finalize_request(struct ftsdc_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd;
	u32 con;
	int debug_as_failure = 0;

	if (host->complete_what != COMPLETION_FINALIZE)
		return;

	if (!mrq)
		return;

	cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	if (cmd->data && (cmd->error == 0) &&
	    (cmd->data->error == 0)) {
		if (host->dodma && (!host->dma_finish)) {
			dbg(host, dbg_dma, "DMA Missing (%d)!\n",
			    host->dma_finish);
			return;
		}
		host->dodma = false;
	}

	/* Read response from controller. */
	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[3] = REG_READ(SDC_RESPONSE0_REG);
		cmd->resp[2] = REG_READ(SDC_RESPONSE1_REG);
		cmd->resp[1] = REG_READ(SDC_RESPONSE2_REG);
		cmd->resp[0] = REG_READ(SDC_RESPONSE3_REG);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = REG_READ(SDC_RESPONSE0_REG);
	}

	if (cmd->error)
		debug_as_failure = 1;

	if (cmd->data && cmd->data->error)
		debug_as_failure = 1;

	dbg_dumpcmd(host, cmd, debug_as_failure);

	clear_imask(host);
	con = REG_READ(SDC_STATUS_REG);
	con &= ~SDC_CLEAR_REG_SDIO_INTR;
	REG_WRITE(con, SDC_CLEAR_REG);

	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	if (cmd->data && cmd->data->stop && (!host->cmd_is_stop)) {
		host->cmd_is_stop = 1;
		ftsdc_send_request(host->mmc);
		return;
	}

	/* If we have no data transfer we are finished here */
	if (!mrq->data)
		goto request_done;

	/* Calculate the amount of bytes transfer if there was no error */
	if (mrq->data->error == 0) {
		mrq->data->bytes_xfered =
			(mrq->data->blocks * mrq->data->blksz);
	} else {
		mrq->data->bytes_xfered = 0;
	}

request_done:
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;

	host->last_opcode = mrq->cmd->opcode;
	mmc_request_done(host->mmc, mrq);
}

static void pio_tasklet(unsigned long data)
{
	struct ftsdc_host *host = (struct ftsdc_host *) data;

	if (host->complete_what == COMPLETION_XFER_PROGRESS) {
		queue_work(mywq, (struct work_struct *)&host->work);
		return;
	}

	if (host->complete_what == COMPLETION_FINALIZE) {
		clear_imask(host);
		if (host->buf_active != XFER_NONE) {
			dbg(host, dbg_err, "unfinished %s buf_count:[%u] buf_bytes:[%u]\n",
			(host->buf_active == XFER_READ) ? "read" : "write",
			host->buf_count, host->buf_bytes);

			if (host->mrq->data)
				host->mrq->data->error = -EINVAL;
		}
		finalize_request(host);
	}
}

static void ftsdc_send_command(struct ftsdc_host *host,
					struct mmc_command *cmd)
{
	u32 ccon = 0;
	u32 newmask = 0;
	u32 scon;

	if (cmd->data) {
		host->complete_what = COMPLETION_XFER_PROGRESS;
		newmask |= SDC_INT_MASK_REG_RSP_TIMEOUT;
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		host->complete_what = COMPLETION_RSPFIN;
		newmask |= SDC_INT_MASK_REG_RSP_TIMEOUT;
	} else {
		host->complete_what = COMPLETION_CMDSENT;
		newmask |= SDC_INT_MASK_REG_CMD_SEND;
	}

	ccon |= cmd->opcode & SDC_CMD_REG_INDEX;
	ccon |= SDC_CMD_REG_CMD_EN;

	if (cmd->flags & MMC_RSP_PRESENT) {
		ccon |= SDC_CMD_REG_NEED_RSP;
		newmask |= SDC_INT_MASK_REG_RSP_CRC_OK |
			SDC_INT_MASK_REG_RSP_CRC_FAIL;
	}

	if (cmd->flags & MMC_RSP_136)
		ccon |= SDC_CMD_REG_LONG_RSP;

	/* applicatiion specific cmd must follow an MMC_APP_CMD. The
	 * value will be updated in finalize_request function
	 */
	if (host->last_opcode == MMC_APP_CMD)
		ccon |= SDC_CMD_REG_APP_CMD;

	enable_imask(host, newmask);
	REG_WRITE(cmd->arg, SDC_ARGU_REG);

	scon = REG_READ(SDC_SDIO_CTRL1_REG);
	if (host->mmc->card != NULL && mmc_card_sdio(host->mmc->card))
		scon |= SDC_SDIO_CTRL1_REG_SDIO_ENABLE;
	else
		scon &= ~SDC_SDIO_CTRL1_REG_SDIO_ENABLE;
	REG_WRITE(scon, SDC_SDIO_CTRL1_REG);

	dbg_dumpregs(host, "");
	dbg(host, dbg_debug, "CON[%x]\n", ccon);

	REG_WRITE(ccon, SDC_CMD_REG);
}

static int ftsdc_setup_data(struct ftsdc_host *host, struct mmc_data *data)
{
	u32 dcon, newmask = 0;

	/* configure data transfer parameter */
	if (!data)
		return 0;
	if (host->mmc->card && host->mmc->card->type == (unsigned int)MMC_TYPE_SD) {
		if (((data->blksz - 1) & data->blksz) != 0) {
			pr_warn("%s: can't do non-power-of 2 (blksz %d)\n",
				__func__, data->blksz);
			return -EINVAL;
		}
	}

	if (data->blksz <= 2) {
		/* We cannot deal with unaligned blocks with more than
		 * one block being transferred
		 */
		if (data->blocks > 1) {
			pr_warn("%s: can't do non-word sized (blksz %d)\n",
				__func__, data->blksz);
			return -EINVAL;
		}
	}

	/* data length */
	dcon = data->blksz * data->blocks;
	REG_WRITE(dcon, SDC_DATA_LEN_REG);

	/* write data control */
	dcon = 0;
	dcon = cal_blksz(data->blksz);

	/* enable UNDERFUN will trigger interrupt immediatedly
	 * So setup it when rsp is received successfully
	 */
	if (data->flags & MMC_DATA_WRITE) {
		dcon |= SDC_DATA_CTRL_REG_DATA_WRITE;
	} else {
		dcon &= ~SDC_DATA_CTRL_REG_DATA_WRITE;
		newmask |= SDC_INT_MASK_REG_FIFO_OVERRUN;
	}

	/* always reset fifo since last transfer may fail */
	dcon |= SDC_DATA_CTRL_REG_FIFO_RST;

	/* enable data transfer which will be pended until cmd is send */
	dcon |= SDC_DATA_CTRL_REG_DATA_EN;
	if (ftsdc_dmaexist(host) &&
			((data->blksz * data->blocks) & 0xf) == 0) {
		newmask &= ~SDC_INT_MASK_REG_FIFO_OVERRUN;
		dcon |= SDC_DATA_CTRL_REG_DMA_EN;
		dcon |= SDC_DMA_TYPE_4;
		host->dodma = true;

	}
	REG_WRITE(dcon, SDC_DATA_CTRL_REG);
	/* add to IMASK register */
	newmask |= SDC_INT_MASK_REG_DATA_CRC_FAIL |
			SDC_INT_MASK_REG_DATA_TIMEOUT;

	enable_imask(host, newmask);
	/* handle sdio */
	dcon = SDC_SDIO_CTRL1_REG_READ_WAIT_ENABLE
		& REG_READ(SDC_SDIO_CTRL1_REG);
	dcon |= data->blksz | data->blocks << 15;
	if (data->blocks > 1)
		dcon |= SDC_SDIO_CTRL1_REG_SDIO_BLK_MODE;
	REG_WRITE(dcon, SDC_SDIO_CTRL1_REG);

	return 0;
}

#define BOTH_DIR (MMC_DATA_WRITE | MMC_DATA_READ)

static int ftsdc_prepare_buffer(struct ftsdc_host *host, struct mmc_data *data)
{
	int rw = (data->flags & MMC_DATA_WRITE) ? 1 : 0;

	if ((!host->mrq) || (!host->mrq->data))
		return -EINVAL;

	WARN_ON((data->flags & BOTH_DIR) == BOTH_DIR);
	host->buf_sgptr = 0;
	host->buf_bytes = 0;
	host->buf_count = 0;
	host->buf_active = rw ? XFER_WRITE : XFER_READ;
	if (host->dodma)
		host->dma_finish = false;

	return 0;
}

static irqreturn_t ftsdc_irq(int irq, void *dev_id)
{
	struct ftsdc_host *host = dev_id;
	struct mmc_command *cmd;
	u32 mci_status;
	u32 mci_clear;
	u32 mci_imsk;
	unsigned long iflags;

	mci_status = REG_READ(SDC_STATUS_REG);
	mci_imsk = REG_READ(SDC_INT_MASK_REG);

	dbg(host, dbg_debug, "irq: status:0x%08x, mask : %08x\n",
		mci_status, mci_imsk);

	if (mci_status & SDC_STATUS_REG_SDIO_INTR) {
		if (mci_imsk & SDC_INT_MASK_REG_SDIO_INTR) {
			mci_clear = SDC_CLEAR_REG_SDIO_INTR;
			REG_WRITE(mci_clear, SDC_CLEAR_REG);

			mmc_signal_sdio_irq(host->mmc);
			return IRQ_HANDLED;
		}
	}
	spin_lock_irqsave(&host->complete_lock, iflags);
	mci_clear = 0;
	if (mci_status & SDC_STATUS_REG_CARD_CHANGE) {
		if ((mci_status & SDC_STATUS_REG_CARD_DETECT)
			== SDC_CARD_INSERT) {
			host->status = "card insert";
		} else {
			host->status = "card remove";
		}
		mmc_detect_change(host->mmc, msecs_to_jiffies(500));
		mci_clear |= SDC_CLEAR_REG_CARD_CHANGE;
		dbg(host, dbg_irq, "%s\n", host->status);

		if (host->complete_what != COMPLETION_NONE) {
			host->mrq->cmd->error = -EIO;
			goto close_transfer;
		}

		goto irq_out;
	}

	if ((host->complete_what == COMPLETION_NONE) ||
	    (host->complete_what == COMPLETION_FINALIZE)) {
		host->status = "nothing to complete";
		mci_clear = -1u;
		goto irq_out;
	}

	if (!host->mrq) {
		host->status = "no active mrq";
		clear_imask(host);
		goto irq_out;
	}

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;

	if (!cmd) {
		host->status = "no active cmd";
		clear_imask(host);
		goto irq_out;
	}

	if (mci_status & SDC_STATUS_REG_CMD_SEND) {
		mci_clear |= SDC_CLEAR_REG_CMD_SEND;

		if (host->complete_what == COMPLETION_CMDSENT) {
			host->status = "ok: command sent";
			goto close_transfer;
		} else {
			host->status = "error: command sent(status not match)";
			cmd->error = -EINVAL;
			goto fail_transfer;
		}
	}

	/* handle error status */
	if (mci_status & SDC_STATUS_REG_RSP_TIMEOUT) {
		dbg(host, dbg_err, "CMDSTAT: error RSP TIMEOUT\n");
		mci_clear |= SDC_CLEAR_REG_RSP_TIMEOUT;
		cmd->error = -ETIMEDOUT;
		host->status = "error: response timeout";
		goto fail_transfer;
	}

	if (mci_status & SDC_STATUS_REG_RSP_CRC_FAIL) {
		mci_clear |= SDC_CLEAR_REG_RSP_CRC_FAIL;
		/* This is weird hack */
		if (cmd->flags & MMC_RSP_CRC) {
			dbg(host, dbg_err, "CMDSTAT: error RSP CRC\n");
			cmd->error = -EILSEQ;
			host->status = "error: RSP CRC failed";
			goto fail_transfer;
		} else {
			host->status = "R3 or R4 type command";
			goto close_transfer;
		}
	}

	if (mci_status & SDC_STATUS_REG_RSP_CRC_OK) {
		mci_clear |= SDC_CLEAR_REG_RSP_CRC_OK;

		if (host->complete_what == COMPLETION_XFER_PROGRESS) {
			REG_WRITE(mci_clear, SDC_CLEAR_REG);

			host->status = "RSP recv OK";
			if (!cmd->data)
				goto close_transfer;

			if (host->dodma) {
				tasklet_schedule(&host->pio_tasklet);
				host->status = "dma access";
				goto irq_out;
			}

			if (host->buf_active == XFER_WRITE)
				enable_imask(host,
				SDC_INT_MASK_REG_FIFO_UNDERRUN);
		} else if (host->complete_what == COMPLETION_RSPFIN) {
			goto close_transfer;
		}
	}

	/* handler data transfer */
	if (mci_status & SDC_STATUS_REG_DATA_TIMEOUT) {
		dbg(host, dbg_err, "CMDSTAT: error DATA TIMEOUT\n");
		mci_clear |= SDC_CLEAR_REG_DATA_TIMEOUT;
		cmd->error = -ETIMEDOUT;
		host->status = "error: data timeout";
		goto fail_transfer;
	}

	if (mci_status & SDC_STATUS_REG_DATA_CRC_FAIL) {
		dbg(host, dbg_err, "CMDSTAT: error DATA CRC\n");
		mci_clear |= SDC_CLEAR_REG_DATA_CRC_FAIL;
		cmd->error = -EILSEQ;
		host->status = "error: data CRC fail";
		goto fail_transfer;
	}

	if ((mci_status & SDC_STATUS_REG_FIFO_UNDERRUN) ||
		mci_status & SDC_STATUS_REG_FIFO_OVERRUN) {

		disable_imask(host, SDC_INT_MASK_REG_FIFO_OVERRUN |
				SDC_INT_MASK_REG_FIFO_UNDERRUN);

		if (!host->dodma) {
			if (host->buf_active == XFER_WRITE) {
				tasklet_schedule(&host->pio_tasklet);
				host->status = "pio tx";
			} else if (host->buf_active == XFER_READ) {

				tasklet_schedule(&host->pio_tasklet);
				host->status = "pio rx";
			}
		}
	}

	goto irq_out;

fail_transfer:
	host->buf_active = XFER_NONE;

close_transfer:
	host->complete_what = COMPLETION_FINALIZE;

	clear_imask(host);
	tasklet_schedule(&host->pio_tasklet);

irq_out:
	REG_WRITE(mci_clear, SDC_CLEAR_REG);

	dbg(host, dbg_debug, "irq: %s\n", host->status);
	spin_unlock_irqrestore(&host->complete_lock, iflags);
	return IRQ_HANDLED;
}

static void ftsdc_send_request(struct mmc_host *mmc)
{
	struct ftsdc_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	host->ccnt++;
	prepare_dbgmsg(host, cmd, host->cmd_is_stop);
	dbg(host, dbg_debug, "%s\n", host->dbgmsg_cmd);

	if (cmd->data) {
		int res = ftsdc_setup_data(host, cmd->data);

		host->dcnt++;

		if (res) {
			dbg(host, dbg_err, "setup data error %d\n", res);
			cmd->error = res;
			cmd->data->error = res;

			mmc_request_done(mmc, mrq);
			return;
		}

		res = ftsdc_prepare_buffer(host, cmd->data);

		if (res) {
			dbg(host, dbg_err, "data prepare error %d\n", res);
			cmd->error = res;
			cmd->data->error = res;

			mmc_request_done(mmc, mrq);
			return;
		}
	}

	/* Send command */
	ftsdc_send_command(host, cmd);
}

static int ftsdc_get_cd(struct mmc_host *mmc)
{
	struct ftsdc_host *host = mmc_priv(mmc);

	u32 con = REG_READ(SDC_STATUS_REG);
		dbg(host, dbg_debug, "get_cd status:%.8x\n\n", con);

	return (con & SDC_STATUS_REG_CARD_DETECT) ? 0 : 1;
}

static void ftsdc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ftsdc_host *host = mmc_priv(mmc);

	host->status = "mmc request";
	host->cmd_is_stop = 0;
	host->mrq = mrq;
	if (ftsdc_get_cd(mmc) == 0) {
		dbg(host, dbg_err, "%s: no medium present\n", __func__);
		host->mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
	} else {
		ftsdc_send_request(mmc);
	}
	dbg(host, dbg_debug, "send request\n");
}

static void ftsdc_set_clk(struct ftsdc_host *host, struct mmc_ios *ios)
{
	u32 clk_div = 0;
	u32 con;
	struct ftsdc_mmc_config *pdata = host->pdev->dev.platform_data;
	u32 freq = pdata->max_freq;

	dbg(host, dbg_debug, "request clk : %u\n", ios->clock);
	con = REG_READ(SDC_CLOCK_CTRL_REG);
	if (ios->clock == 0) {
		host->real_rate = 0;
		con |= SDC_CLOCK_CTRL_REG_CLK_DIS;
	} else {
		clk_div = (freq / (ios->clock << 1)) - 1;
		host->real_rate = freq / ((clk_div+1)<<1);
		if (host->real_rate > ios->clock) {
			++clk_div;
			host->real_rate = freq / ((clk_div+1)<<1);
		}
		if (clk_div > 127)
			dbg(host, dbg_err, "%s: no match clock rate, %u\n",
			__func__, ios->clock);

		con = (con & ~SDC_CLOCK_CTRL_REG_CLK_DIV)
			| (clk_div & SDC_CLOCK_CTRL_REG_CLK_DIV);
		con &= ~SDC_CLOCK_CTRL_REG_CLK_DIS;
	}

	REG_WRITE(con, SDC_CLOCK_CTRL_REG);
}

static void ftsdc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ftsdc_host *host = mmc_priv(mmc);
	u32 con;

	con = REG_READ(SDC_POWER_CTRL_REG);
	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		con |= SDC_POWER_CTRL_REG_POWER_ON;
		break;
	case MMC_POWER_OFF:
	default:
		con &= ~SDC_POWER_CTRL_REG_POWER_ON;
		break;
	}

	REG_WRITE(con, SDC_POWER_CTRL_REG);

	ftsdc_set_clk(host, ios);

	if ((ios->power_mode == MMC_POWER_ON) ||
	    (ios->power_mode == MMC_POWER_UP)) {
		dbg(host, dbg_debug, "running at %ukHz (requested: %ukHz).\n",
			host->real_rate/1000, ios->clock/1000);
	} else {
		dbg(host, dbg_debug, "powered down.\n");
	}

	host->bus_width = ios->bus_width;
	/* write bus configure */
	con = REG_READ(SDC_BUS_WIDTH_REG);

	con &= ~(SDC_BUS_WIDTH_REG_SINGLE_BUS |
			SDC_BUS_WIDTH_REG_WIDE_4_BUS |
			SDC_BUS_WIDTH_REG_WIDE_8_BUS);
	if (host->bus_width == MMC_BUS_WIDTH_1)
		con |= SDC_BUS_WIDTH_REG_SINGLE_BUS;
	else if (host->bus_width == MMC_BUS_WIDTH_4)
		con |= SDC_BUS_WIDTH_REG_WIDE_4_BUS;
	else if (host->bus_width == MMC_BUS_WIDTH_8)
		con |= SDC_BUS_WIDTH_REG_WIDE_8_BUS;
	else
		dbg(host, dbg_err, "set_ios: can't support bus mode");
	REG_WRITE(con, SDC_BUS_WIDTH_REG);

	/*set rsp and data timeout */
	con = -1;
	REG_WRITE(con, SDC_DATA_TIMER_REG);
	if (ios->power_mode == MMC_POWER_UP)
		mmc_delay(250);
}

static int ftsdc_get_ro(struct mmc_host *mmc)
{
	struct ftsdc_host *host = mmc_priv(mmc);
	u32 con = REG_READ(SDC_STATUS_REG);

	dbg(host, dbg_debug, "get_ro status:%.8x\n", con);

	return (con & SDC_STATUS_REG_CARD_LOCK) ? 1 : 0;
}


static void ftsdc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct ftsdc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 con;
#ifdef CONFIG_MMC_DEBUG
	u32 ena;
#endif

	local_irq_save(flags);

	con = REG_READ(SDC_INT_MASK_REG);
#ifdef CONFIG_MMC_DEBUG
	ena = (con & SDC_STATUS_REG_SDIO_INTR) ? 1:0;
	if (ena == enable)
		dbg(host, dbg_debug, "ena %.8x %.8x\n", ena, enable);
#endif

	con = enable ? (con | SDC_STATUS_REG_SDIO_INTR)
		: (con & ~SDC_STATUS_REG_SDIO_INTR);
	REG_WRITE(con, SDC_INT_MASK_REG);

#ifdef CONFIG_MMC_DEBUG
	//check and ensure data out to SD host controller
	ena = (REG_READ(SDC_INT_MASK_REG) & SDC_STATUS_REG_SDIO_INTR) ? 1:0;
	if (ena != enable)
		dbg(host, dbg_debug, "ena %.8x %.8x\n", ena, enable);
#endif

	local_irq_restore(flags);
}

static struct mmc_host_ops ftsdc_ops = {
	.request	= ftsdc_request,
	.set_ios	= ftsdc_set_ios,
	.get_ro		= ftsdc_get_ro,
	.get_cd		= ftsdc_get_cd,
	.enable_sdio_irq = ftsdc_enable_sdio_irq,
};

#ifdef CONFIG_DEBUG_FS

static int ftsdc_state_show(struct seq_file *seq, void *v)
{
	struct ftsdc_host *host = seq->private;

	seq_printf(seq, "Register base = 0x%08x\n",
		(u32)((unsigned long)host->base));
	seq_printf(seq, "Clock rate = %u\n", host->real_rate);
	seq_printf(seq, "host status = %s\n", host->status);
	seq_printf(seq, "IRQ = %d\n", host->irq);
	seq_printf(seq, "IRQ enabled = %d\n", host->irq_enabled);
	seq_printf(seq, "complete what = %d\n", host->complete_what);
	seq_printf(seq, "dma support = %d\n", ftsdc_dmaexist(host));
	seq_printf(seq, "use dma = %d\n", host->dodma);

	return 0;
}

static int ftsdc_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, ftsdc_state_show, inode->i_private);
}

static const struct file_operations ftsdc_fops_state = {
	.owner		= THIS_MODULE,
	.open		= ftsdc_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define DBG_REG(_r) { .addr = SDC_## _r ## _REG, .name = #_r }

static struct ftsdc_reg {
	unsigned short	addr;
	unsigned char	*name;
} debug_regs[] = {
	DBG_REG(CMD),
	DBG_REG(ARGU),
	DBG_REG(RESPONSE0),
	DBG_REG(RESPONSE1),
	DBG_REG(RESPONSE2),
	DBG_REG(RESPONSE3),
	DBG_REG(RSP_CMD),
	DBG_REG(DATA_CTRL),
	DBG_REG(DATA_TIMER),
	DBG_REG(DATA_LEN),
	DBG_REG(STATUS),
	DBG_REG(CLEAR),
	DBG_REG(INT_MASK),
	DBG_REG(POWER_CTRL),
	DBG_REG(CLOCK_CTRL),
	DBG_REG(BUS_WIDTH),
	DBG_REG(SDIO_CTRL1),
	DBG_REG(SDIO_CTRL2),
	DBG_REG(SDIO_STATUS),
	DBG_REG(FEATURE),
	DBG_REG(REVISION),
	{}
};

static int ftsdc_regs_show(struct seq_file *seq, void *v)
{
	struct ftsdc_host *host = seq->private;
	struct ftsdc_reg *rptr = debug_regs;

	for (; rptr->name; rptr++)
		seq_printf(seq, "SDI%s\t=0x%08x\n", rptr->name,
			   REG_READ(rptr->addr));

	return 0;
}

static int ftsdc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, ftsdc_regs_show, inode->i_private);
}

static const struct file_operations ftsdc_fops_regs = {
	.owner		= THIS_MODULE,
	.open		= ftsdc_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void ftsdc_debugfs_attach(struct ftsdc_host *host)
{
	struct device *dev = &host->pdev->dev;

	host->debug_root = debugfs_create_dir(dev_name(dev), NULL);
	if (IS_ERR(host->debug_root)) {
		dev_err(dev, "failed to create debugfs root\n");
		return;
	}

	host->debug_state = debugfs_create_file("state", 0444,
						host->debug_root, host,
						&ftsdc_fops_state);

	if (IS_ERR(host->debug_state))
		dev_err(dev, "failed to create debug state file\n");

	host->debug_regs = debugfs_create_file("regs", 0444,
					       host->debug_root, host,
					       &ftsdc_fops_regs);

	if (IS_ERR(host->debug_regs))
		dev_err(dev, "failed to create debug regs file\n");
}

static void ftsdc_debugfs_remove(struct ftsdc_host *host)
{
	debugfs_remove(host->debug_regs);
	debugfs_remove(host->debug_state);
	debugfs_remove(host->debug_root);
}

#else
static inline void ftsdc_debugfs_attach(struct ftsdc_host *host) { }
static inline void ftsdc_debugfs_remove(struct ftsdc_host *host) { }

#endif /* CONFIG_DEBUG_FS */

enum {
	MMC_CTLR_VERSION_1 = 0,
	MMC_CTLR_VERSION_2,
};


static struct platform_device_id ftsdc_mmc_devtype[] = {
	{
		.name	= "ag101p",
		.driver_data = MMC_CTLR_VERSION_1,
	}, {
		.name	= "ae3xx",
		.driver_data = MMC_CTLR_VERSION_2,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, ftsdc_mmc_devtype);

static const struct of_device_id ftsdc_mmc_dt_ids[] = {
	{
		.compatible = "andestech,atfsdc010g",
		.data = &ftsdc_mmc_devtype[MMC_CTLR_VERSION_2],
	},
	{},
};
MODULE_DEVICE_TABLE(of, ftsdc_mmc_dt_ids);


static struct ftsdc_mmc_config
	*mmc_parse_pdata(struct platform_device *pdev)
{
	struct device_node *np;
	struct ftsdc_mmc_config *pdata = pdev->dev.platform_data;
	const struct of_device_id *match =
		of_match_device(of_match_ptr(ftsdc_mmc_dt_ids), &pdev->dev);
	u32 data;

	np = pdev->dev.of_node;
	if (!np)
		return pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	pdev->dev.platform_data = (void *)pdata;

	if (!pdata) {
		dev_err(&pdev->dev, "Failed to allocate memory for struct ftsdc_mmc_config\n");
		goto nodata;
	}

	if (match)
		pdev->id_entry = match->data;

	if (of_property_read_u32(np, "max-frequency", &pdata->max_freq))
		dev_info(&pdev->dev, "'max-frequency' property not specified, defaulting to 25MHz\n");

	of_property_read_u32(np, "bus-width", &data);
	switch (data) {
	case 1:
	case 4:
	case 8:
		pdata->wires = data;
		break;
	default:
		pdata->wires = 1;
	}
nodata:
	return pdata;
}

static int __init ftsdc_probe(struct platform_device *pdev)
{
	int (*read_fixup)(void __iomem *addr, unsigned int val,
		unsigned int shift_bits);
	struct ftsdc_host *host;
	struct mmc_host	*mmc;
	struct ftsdc_mmc_config *pdata = NULL;
	struct resource *r, *mem = NULL;
	int ret = -ENOMEM;
	u32 con;
	int irq = 0;
	size_t mem_size;

	pdata = mmc_parse_pdata(pdev);
	if (pdata == NULL) {
		dev_err(&pdev->dev, "Couldn't get platform data\n");
		return -ENOENT;
	}
	ret = -ENODEV;
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);

	if (!r || irq < 0)
		goto probe_out;

	ret = -EBUSY;
	mem_size = resource_size(r);
	mem = request_mem_region(r->start, mem_size, pdev->name);

	if (!mem) {
		dev_err(&pdev->dev,
			"failed to get io memory region resource.\n");
		goto probe_out;
	}
	ret = -ENOMEM;
	mmc = mmc_alloc_host(sizeof(struct ftsdc_host), &pdev->dev);
	if (!mmc)
		goto probe_out;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;
	mywq = create_workqueue("atcsdc_queue");
	if (mywq == NULL)
		goto probe_free_host;

	spin_lock_init(&host->complete_lock);
	tasklet_init(&host->pio_tasklet, pio_tasklet, (unsigned long) host);
	init_completion(&host->dma_complete);
	INIT_WORK(&host->work, ftsdc_work);
	host->complete_what = COMPLETION_NONE;
	host->buf_active = XFER_NONE;
	host->mem = mem;
#ifdef CONFIG_MMC_FTSDC_DMA
	ret = ftsdc_configure_dma(host);
	if (ret)
		goto probe_free_host;
#endif
	host->base = (void __iomem *) ioremap(mem->start, mem_size);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto probe_free_mem_region;
	}

	/* Check revision register */
	read_fixup = symbol_get(readl_fixup);
	ret = read_fixup(host->base + SDC_REVISION_REG, 0x00030107, 0);
	symbol_put(readl_fixup);
	if (!ret) {
		dev_err(&pdev->dev,
			"bitmap revision mismatch(ftsdc)\n");
		ret = -ENXIO;
		goto probe_free_mem_region;
	}

	host->irq = irq;

	ret = request_irq(host->irq, ftsdc_irq, 0, DRIVER_NAME, host);
	if (ret) {
		dev_err(&pdev->dev, "failed to request mci interrupt.\n");
		ret = -ENOENT;
		goto probe_free_mem_region;
	}
	host->irq_enabled = true;
	/* enable card change interruption */
	con = REG_READ(SDC_INT_MASK_REG);
	con |= SDC_INT_MASK_REG_CARD_CHANGE;
	REG_WRITE(con, SDC_INT_MASK_REG);

	con = REG_READ(SDC_BUS_WIDTH_REG);
	mmc->ops = &ftsdc_ops;
	mmc->ocr_avail	= MMC_VDD_32_33 | MMC_VDD_33_34;

	if (con & SDC_WIDE_4_BUS_SUPPORT)
		mmc->caps |= MMC_CAP_4_BIT_DATA;
	else if (con & SDC_WIDE_8_BUS_SUPPORT)
		mmc->caps |= MMC_CAP_8_BIT_DATA;

#ifndef A320D_BUILDIN_SDC
	mmc->caps |= MMC_CAP_SDIO_IRQ;
#endif
	mmc->f_min = pdata->max_freq / (2 * 128);
	mmc->f_max = pdata->max_freq / 2;
	/* limit SDIO mode max size */
	mmc->max_req_size = 128 * 1024 * 1024 - 1;
	mmc->max_blk_size = 2047;
	mmc->max_req_size = (mmc->max_req_size + 1) / (mmc->max_blk_size + 1);
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_blk_count = (1<<17)-1;
	/* set fifo length and default threshold half */
	con = REG_READ(SDC_FEATURE_REG);
	host->fifo_len = (con & SDC_FEATURE_REG_FIFO_DEPTH) * sizeof(u32);
	dbg(host, dbg_debug,
	    "probe: mapped mci_base:%p irq:%u.\n",
	    host->base, host->irq);
	dbg_dumpregs(host, "");
	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host.\n");
		goto probe_free_irq;
	}
	ftsdc_debugfs_attach(host);
	platform_set_drvdata(pdev, mmc);
	dev_info(&pdev->dev, "%s - using %s SDIO IRQ\n", mmc_hostname(mmc),
		 mmc->caps & MMC_CAP_SDIO_IRQ ? "hw" : "sw");
	return 0;

 probe_free_irq:
	free_irq(host->irq, host);

 probe_free_mem_region:
	release_mem_region(host->mem->start, resource_size(host->mem));
	destroy_workqueue(mywq);

 probe_free_host:
	if (!IS_ERR(host->dma.chan))
		dma_release_channel(host->dma.chan);
	mmc_free_host(mmc);

 probe_out:
	return ret;
}

static void ftsdc_shutdown(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct ftsdc_host *host = mmc_priv(mmc);

	flush_workqueue(mywq);
	destroy_workqueue(mywq);

	ftsdc_debugfs_remove(host);
	mmc_remove_host(mmc);
}

static int __exit ftsdc_remove(struct platform_device *pdev)
{
	struct mmc_host		*mmc  = platform_get_drvdata(pdev);
	struct ftsdc_host	*host = mmc_priv(mmc);

	ftsdc_shutdown(pdev);
	tasklet_disable(&host->pio_tasklet);
	if (!IS_ERR(host->dma.chan))
		dma_release_channel(host->dma.chan);
	free_irq(host->irq, host);
	iounmap(host->base);
	release_mem_region(host->mem->start, resource_size(host->mem));
	mmc_free_host(mmc);

	return 0;
}

#ifdef CONFIG_PM
static int ftsdc_free_dma(struct ftsdc_host *host)
{
	if (!IS_ERR(host->dma.chan))
		dma_release_channel(host->dma.chan);

	return 0;
}

static int ftsdc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct ftsdc_host *host = mmc_priv(mmc);
	int ret = 0;

	if (mmc)
		ftsdc_free_dma(host);

	return ret;

}

static int ftsdc_resume(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	int ret = 0;

	if (mmc) {
		struct ftsdc_host *host = mmc_priv(mmc);

		ftsdc_configure_dma(host);
	}
	return ret;
}

#else
#define ftsdc_suspend NULL
#define ftsdc_resume NULL
#endif

static struct platform_driver ftsdc_driver = {
	.driver	= {
		.name	= "ftsdc010g",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ftsdc_mmc_dt_ids),
	},
	.remove		= __exit_p(ftsdc_remove),
	.shutdown	= ftsdc_shutdown,
	.suspend	= ftsdc_suspend,
	.resume		= ftsdc_resume,
};

module_platform_driver_probe(ftsdc_driver, ftsdc_probe);
MODULE_DESCRIPTION("Andestech Leopard MMC/SD Card Interface driver");
MODULE_AUTHOR("Rick Chen <rick@andestech.com>");
MODULE_LICENSE("GPL v2");
