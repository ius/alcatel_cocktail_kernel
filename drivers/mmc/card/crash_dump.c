/*
 *  linux/arch/arm/mach-msm/crash_dump.c
 *  
 *  Save crash log to mmc, just for Cocktail
 *
 *  Copyright 2011 Shenglei HUANG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/err.h>
#include <linux/genhd.h>
#include <linux/device.h>
#include <linux/bio.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/syslog.h>
#include <linux/bug.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/core.h>
#include <linux/scatterlist.h>

#include <asm/delay.h>
#include <mach/dma.h>

struct gendisk *target_disk=NULL;
struct mmc_card *target_card=NULL;
struct block_device *target_bdev=NULL;
struct hd_struct *target_part=NULL;

struct sg_table sgt;

//128K for Cocktail
#define DUMP_BUF_LEN (1 << CONFIG_LOG_BUF_SHIFT)
extern int log_buf_copy(char *dest, int idx, int len);

static DEFINE_MUTEX(reopen_mutex);

/* static void test_thread(struct work_struct *work); */
/* static DECLARE_DELAYED_WORK(crash_task, test_thread); */

/*
 * Fill in the mmc_request structure given a set of transfer parameters.
 */
static void mmc_prepare_mrq(struct mmc_card *card, struct mmc_request *mrq,
				 struct scatterlist *sg, unsigned sg_len,
				 unsigned dev_addr, unsigned blocks,
				 unsigned blksz, int write)
{
    BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

    if (blocks > 1) {
	mrq->cmd->opcode = write ?
	    MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
    } else {
	mrq->cmd->opcode = write ?
	    MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
    }

    mrq->cmd->arg = dev_addr;
    if (!mmc_card_blockaddr(card))
	mrq->cmd->arg <<= 9;

    mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

    if (blocks == 1)
	mrq->stop = NULL;
    else {
	mrq->stop->opcode = MMC_STOP_TRANSMISSION;
	mrq->stop->arg = 0;
	mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
    }

    mrq->data->blksz = blksz;
    mrq->data->blocks = blocks;
    mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
    mrq->data->sg = sg;
    mrq->data->sg_len = sg_len;

    mmc_set_data_timeout(mrq->data, card);
}

/*
 * Really process the request
 */
static void mmc_start_request(struct mmc_host *host, struct mmc_request *mrq)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned int i, sz;
	struct scatterlist *sg;
#endif
        WARN_ON(!host->claimed);

	mrq->cmd->error = 0;
	mrq->cmd->mrq = mrq;
	if (mrq->data) {
		BUG_ON(mrq->data->blksz > host->max_blk_size);
		BUG_ON(mrq->data->blocks > host->max_blk_count);
		BUG_ON(mrq->data->blocks * mrq->data->blksz >
			host->max_req_size);

#ifdef CONFIG_MMC_DEBUG
		sz = 0;
		for_each_sg(mrq->data->sg, sg, mrq->data->sg_len, i)
			sz += sg->length;
		BUG_ON(sz != mrq->data->blocks * mrq->data->blksz);
#endif

		mrq->cmd->data = mrq->data;
		mrq->data->error = 0;
		mrq->data->mrq = mrq;
		if (mrq->stop) {
			mrq->data->stop = mrq->stop;
			mrq->stop->error = 0;
			mrq->stop->mrq = mrq;
		}

	}
	host->ops->request(host, mrq);
}

/*
 * Wait for the card to finish the busy state
 */
static int mmc_wait_busy(struct mmc_card *card)
{
	int busy;
	struct mmc_command cmd;
	struct mmc_request mrq;


	busy = 0;
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

		memset(&mrq, 0, sizeof(struct mmc_request));

		memset(cmd.resp, 0, sizeof(cmd.resp));
		cmd.retries = 0;

		mrq.cmd = &cmd;
		cmd.data = NULL;

		mmc_start_request(card->host, &mrq);

		if (cmd.error)
		    return cmd.error;

		if (!busy && !(cmd.resp[0] & R1_READY_FOR_DATA)) {
			busy = 1;
			printk(KERN_ERR "%s: Warning: Host did not "
				"wait for busy state to end.\n",
				mmc_hostname(card->host));
		}
	} while (!(cmd.resp[0] & R1_READY_FOR_DATA));

	return cmd.error;
}

static void crash_dump_init_sg_list(int page_nr, char *buffer)
{
    int buf_idx = 0;
    int i;
    struct scatterlist *sgl = sgt.sgl;
    struct page *mpage = NULL;

    for (i=0; i<page_nr; i++)
    {
	mpage = sg_page(sgl);
	memset(page_address(mpage), 0, PAGE_SIZE);
	if (!buffer){
	    log_buf_copy(page_address(mpage), buf_idx, PAGE_SIZE);
	    buf_idx += PAGE_SIZE;
	}
	else{
	    memcpy(page_address(mpage), buffer, PAGE_SIZE);
	    buffer += PAGE_SIZE;
	}
	
	sgl = sg_next(sgl);
    }
}    
	
/*
 *  write buffer to EMMC
 */
static unsigned int crash_dump_backup(struct mmc_card *card,  unsigned int offset, char *buffer)
{
    struct mmc_command cmd;
    struct mmc_request mrq;
    struct mmc_command stop;
    struct mmc_data data;
    struct scatterlist *sg;
    unsigned int sg_len = 0;
    unsigned int wr_blk_size = 0;
    unsigned int blocks = 0 ;
    unsigned int ret = 0;
    
    mmc_claim_host(card->host);

    memset(&mrq, 0, sizeof(struct mmc_request));
    memset(&cmd, 0, sizeof(struct mmc_command));
    memset(&data, 0, sizeof(struct mmc_data));
    memset(&stop, 0, sizeof(struct mmc_command));
    
    mrq.cmd = &cmd;
    mrq.data = &data;
    mrq.stop = &stop;
    
    wr_blk_size = 1 << card->csd.write_blkbits;
    sg_len = DUMP_BUF_LEN / PAGE_SIZE;
    blocks = DUMP_BUF_LEN / wr_blk_size;

    sg = sgt.sgl;

    crash_dump_init_sg_list(sg_len, buffer);
    
    mmc_prepare_mrq(card, &mrq, sg, sg_len, offset, blocks, wr_blk_size, 1);

    //mmc_start_request(card->host, &mrq);
    mmc_wait_for_req(card->host, &mrq);

    if (cmd.error)
	return cmd.error;
    if (data.error)
	return data.error;

    ret = mmc_wait_busy(card);
    if (ret)
	return ret;

    mmc_release_host(card->host);
    
    return 0;
}

/* static int crash_dump_stop_mmc(struct mmc_card *card) */
/* { */
/*     struct gendisk *disk = target_disk; */
/*     /\* struct mmc_command cmd; *\/ */
/*     /\* struct mmc_request mrq; *\/ */
/*     //int ret=-1; */

/*     /\* if (!disk->queue){ *\/ */
/*     /\* 	printk(KERN_ERR "crash_dump: host queue is not initialized!\n"); *\/ */
/*     /\* 	return ret; *\/ */
/*     /\* } *\/ */
/*     printk(KERN_ERR "crash_dump: try stop mmc card queue\n"); */

/*     /\* spin_lock_irq(disk->queue->queue_lock); *\/ */
/*     /\* queue_flag_set(QUEUE_FLAG_PLUGGED, disk->queue); *\/ */
/*     /\* spin_unlock_irq(disk->queue->queue_lock); *\/ */
    
/*     printk(KERN_ERR "crash_dump: mmc card queue stoped.\n"); */
/*     /\* memset((struct mmc_command *)&cmd, 0, sizeof(struct mmc_command)); *\/ */
/*     /\* memset(&mrq, 0, sizeof(struct mmc_request)); *\/ */
	
/*     /\* cmd.opcode = MMC_STOP_TRANSMISSION; *\/ */
/*     /\* cmd.arg = card->rca << 16 | 1; *\/ */
/*     /\* cmd.flags = MMC_RSP_R1B | MMC_CMD_AC; *\/ */

/*     /\* memset(cmd.resp, 0, sizeof(cmd.resp)); *\/ */
/*     /\* cmd.retries = 5; *\/ */
    
/*     /\* mrq.cmd = &cmd; *\/ */
/*     /\* cmd.data = NULL; *\/ */

/*     /\* mmc_start_request(card->host, &mrq); *\/ */

/*     /\* if (cmd.error) { *\/ */
/*     /\* 	printk(KERN_ERR "crash_dump: unable to stop current transmisson!\n"); *\/ */
/*     /\* 	return cmd.error; *\/ */
/*     /\* } *\/ */

/*     /\* ret = mmc_wait_busy(card); *\/ */
/*     /\* if (ret) *\/ */
/*     /\* 	return ret; *\/ */

/*     return 0; */
/* } */

extern char * smem_log_dump(void);

int crash_dump_panic(void)
{
    struct mmc_card *card = target_card;
    unsigned long target_addr;
    int ret= 0;
    char *buf = NULL;

    if (!mutex_trylock(&reopen_mutex))
    {
	printk(KERN_ERR "crash_dump: process have launched!\n");
	return -EBUSY;
    }

    if (target_part == NULL)
    {
    	printk(KERN_ERR "crash_dump: get target partition fail!\n");
    	return -1;
    }

    // target sector address on EMMC, 
    target_addr = target_part->start_sect + target_part->nr_sects/2;
    
    /* ret = crash_dump_stop_mmc(card); */
    /* if (ret <0) */
    /* 	return ret; */

    ret = crash_dump_backup(card, target_addr, NULL);

    //we may also need to dump share memory log
    target_addr =  target_part->start_sect + target_part->nr_sects/2 + DUMP_BUF_LEN / 512;
    buf = smem_log_dump();
    ret = crash_dump_backup(card, target_addr, buf);
    
    return ret;
}
EXPORT_SYMBOL(crash_dump_panic);

/* static void test_thread(struct work_struct *work) */
/* { */
/*     printk(KERN_ERR "Entering crash_test thread!\n"); */
    
/*     *(int *)0 = 0; */
    
/*     return; */
/* } */

/* /\* */
/*  * linux_crash_read_proc() */
/*  * This interface is just for test */
/*  *\/ */
/* static int linux_crash_read_proc( */
/* 	char *page, char **start, off_t off, int count, int *eof, void *data) */
/* { */
/*     printk(KERN_ERR "Testing crash dump\n"); */

/*     schedule_delayed_work(&crash_task, HZ); */

/*     return 0; */
/* } */

void crash_dump_init(struct gendisk *disk, struct mmc_card *card)
{
    struct hd_struct *part = disk->part_tbl->part[13];
    sector_t part_start = 0;
    sector_t part_nr = 0;
    int partno = 0;
    int page_nr = 0;
    int i, ret = 0;
    /* struct proc_dir_entry *d_entry; */

    struct scatterlist * sgl;
    struct page *mpage = NULL;

    page_nr = DUMP_BUF_LEN / PAGE_SIZE;

    memset(&sgt, 0, sizeof(struct sg_table));
    ret = sg_alloc_table(&sgt, page_nr, GFP_ATOMIC);
    if (ret){
	printk(KERN_ERR "crash_dump: allocate sg table fail!\n");
	return;
    }
    sgl = sgt.sgl;

    // Allocate memory for dump 
    
    for (i=0; i<page_nr; i++)
    {
	mpage = alloc_pages( GFP_ATOMIC, 0);
	if (mpage ==NULL){
	    printk(KERN_ERR "crash_dump: allocate dump buffer fail!\n");
	    return;
	}
	memset(page_address(mpage), 0, PAGE_SIZE);
	sg_set_page(sgl, mpage, PAGE_SIZE, 0);

	sgl = sg_next(sgl);
    }


    if (part == NULL)
    {
    	printk(KERN_ERR "crash dump:Get partition information fail!\n");
    	return;
    }
    part_start = part->start_sect;
    part_nr = part->nr_sects;
    partno = part->partno;

    printk(KERN_ERR "crash dump: \n");
    printk(KERN_ERR "     Partition number = %d \n", partno);
    printk(KERN_ERR "     Partition start  = %d \n", (int)part_start);
    printk(KERN_ERR "     Partition size   = %d KB\n", (int)part_nr/2);
    printk(KERN_ERR "     Card block size   = %d B\n", (int)card->csd.write_blkbits);

    target_disk = disk;
    target_card = card;
    //TODO: Get correct partition by ID or NAME
    target_bdev = bdget_disk(target_disk,13);
    target_part = part;

    // Only root can read/write this file
    /* d_entry = create_proc_entry("linux_crash", S_IRUSR , NULL); */
    /* if (d_entry) { */
    /* 	d_entry->read_proc = linux_crash_read_proc; */
    /* } */
    
    return;
}
EXPORT_SYMBOL(crash_dump_init);
