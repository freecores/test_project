/* -*- linux-c -*-
 *
 * OpenCores MMC Controller driver
 *
 * 
 * Command 51 and Command 2 hardcoded
 * 
 * 
 * 
 * Copyright (C) 2009 ORSoC, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/highmem.h>
#include <asm/board.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mmc.h>
#include <asm/system.h>
#include <linux/mmc/card.h>
#include "mmc_ocores.h"

#define DRIVER_NAME "mmc-ocores"

#define NR_SG	1




struct ocores_host {
	struct mmc_host		*mmc;
	spinlock_t		lock;
	struct resource		*res;
	void __iomem		*base;
	unsigned int		cmdat;
	unsigned int		power_mode;
	struct ocores_platform_data *pdata;
	unsigned int		word_cnt;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	int irq_cmd;
	int irq_dat;
	unsigned int flags;
	struct tasklet_struct finish_cmd;
	struct tasklet_struct finish_data;
	struct {
		unsigned int normal_int_status;
		unsigned int error_int_status;
		unsigned int data_int_status;
	} registers;
	int clock;
	
	/* DMA buffer used for transmitting */
	
	
	unsigned int total_length;
	unsigned int dma_len;
	int transfer_index;
	
	int free_tx_bd;
	int free_rx_bd;
	/* Latest in the scatterlist that has been enabled for transfer, but not freed */
	int in_use_index;
};



static void ocores_tasklet_finish_cmd(unsigned long param);
static void ocores_tasklet_finish_data(unsigned long param);

static inline void CMD_IRQ_ON(struct ocores_host *host, u32 mask)
{
	u32 val = readl(host->base  + SD_NOMAL_INT_SIGNAL_ENABLE);	
	val |= mask;
	writel (val, host->base  + SD_NOMAL_INT_SIGNAL_ENABLE);	
}

static inline void CMD_IRQ_OFF(struct ocores_host *host, u32 mask)
{
	u32 val = readl(host->base  + SD_NOMAL_INT_SIGNAL_ENABLE);
	val  &= ~mask;
	writel (val, host->base  + SD_NOMAL_INT_SIGNAL_ENABLE);
}

static inline void DAT_IRQ_ON(struct ocores_host *host, u32 mask)
{

	
	u32 val = readl(host->base  + SD_BD_ISER);	
	val |= mask;
	writel (mask, host->base  + SD_BD_ISER);
		
	
	
}

static inline void DAT_IRQ_OFF(struct ocores_host *host, u32 mask)
{
	u32 val = readl(host->base  + SD_BD_ISER);
	val  &= ~mask;
	writel (val, host->base  + SD_BD_ISER);
}


static void ocores_pre_dma(struct ocores_host *host)
{
	unsigned int i,j;
	unsigned int off_scal;
	struct scatterlist *sg;
	struct mmc_command *cmd;
	struct mmc_data *data;
	unsigned long flags;
	
	off_scal=512;	
	if (host->mmc->card!= NULL){
				
		if (mmc_card_blockaddr(host->mmc->card))
			off_scal=1;
		else
			off_scal=512;	
	} 
	pr_debug("Pre block_offset %d\n", off_scal);
	
	cmd = host->cmd;
	if (!cmd) {
		pr_debug("no command\n");
		return;
	}
	data = cmd->data;
	if (!data) {
		pr_debug("no data\n");
		return;
	}
	
	/* Setup the next transfer */	
		
	sg = &data->sg[host->transfer_index++];
		

	if (data->flags & MMC_DATA_READ)
		host->dma_len = dma_map_sg(mmc_dev(host->mmc), sg, data->sg_len,  DMA_FROM_DEVICE);
	else
		host->dma_len = dma_map_sg(mmc_dev(host->mmc), sg, data->sg_len,  DMA_TO_DEVICE);
	pr_debug(KERN_ALERT "Dma address = %d, sg_dma_len %d, length = %d, sg_dma_len %d\n", sg_dma_address(&data->sg[0]),  sg_dma_len(&data->sg[0]), sg->length, host->dma_len); 
	printk(KERN_ALERT "Dma address = %d, sg_dma_len %d, length = %d, sg_dma_len %d\n", sg_dma_address(&data->sg[0]),  sg_dma_len(&data->sg[0]), sg->length, host->dma_len);
	
	for (i = 0; i < host->dma_len; i++) {
		unsigned int length = sg_dma_len(&data->sg[i]);
		
		if (length >= 512)
			length /=512;
		else
			length = 1;			
		
		//XXX:512 SD 1.0 card only.
		if (data->flags & MMC_DATA_READ){
			for (j = 0; j< length;j++) {								
				writel((sg_dma_address(&data->sg[i])+ 512*j), host->base + BD_RX);
				wmb();
				writel(cmd->arg+off_scal*j, host->base + BD_RX);
			}
			DAT_IRQ_ON (host,(TRE|CMDE|FIFOE|MRC|TRS));			
		}
		else{						
						 
			for (j = 0; j< length;j++) {
												
				writel((sg_dma_address(&data->sg[i])+ 512*j), host->base + BD_TX);
				wmb();
				writel(cmd->arg+off_scal*j, host->base + BD_TX);				
								
			}
			
		
			DAT_IRQ_ON (host,(TRE|CMDE|FIFOE|MRC|TRS));
			pr_debug("pre dma write done\n");
		}	
					
		
	} 

		
	
}



static void ocores_start_cmd(struct ocores_host *host, struct mmc_command *cmd)
{
	unsigned int cmd_arg, cmd_command=0;

	struct mmc_data *data = cmd->data;
	//WARN_ON(host->cmd != NULL);
	host->cmd = cmd;

	//XXX:opcode == 51 not supported by hardware, hack here	    
	if (data && ( cmd->opcode != 51)&& ( cmd->opcode != 12)) {
       		if ( data->blksz & 0x3 ) {
			pr_debug("Unsupported block size\n");
			cmd->error = -EINVAL;
			mmc_request_done(host->mmc, host->mrq);
			return;
		} 
		
		data->bytes_xfered = 0;
		host->transfer_index = 0;
		host->in_use_index = 0;		
		
		if (data->flags & MMC_DATA_READ){  //Handle a read
			pr_debug(KERN_ALERT "%s: Data read dat Len %u\n", __FUNCTION__,host->total_length);
			
			host->total_length = 0;
			ocores_pre_dma(host); //Set up BD
			
		}
		else if (data->flags & MMC_DATA_WRITE){ //Handle write
				//cmdr |= AT91_MCI_TRCMD_START;
				
			host->total_length = data->sg->length; 
			pr_debug(KERN_ALERT "%s: Data write dat Len %u\n", __FUNCTION__,host->total_length);
			
			ocores_pre_dma(host); //Set up BD
		
		}
					
		
	}
	else{
			//Set up command
		cmd_arg = cmd->arg;
		cmd_command |= cmd->opcode << 8;
		cmd_command |= host->word_cnt << 6;

		if ( mmc_resp_type(cmd) == MMC_RSP_CRC  )
			cmd_command |= CRCE;
		if ( mmc_resp_type(cmd) == MMC_RSP_OPCODE  )
			cmd_command |= CICE;

		switch (mmc_resp_type(cmd)) {
		case MMC_RSP_NONE:
			cmd_command |= MMCOC_RSP_NONE;
			break;
		case MMC_RSP_R1:
			cmd_command |= MMCOC_RSP_48;
			break;
		case MMC_RSP_R1B:
			cmd_command |= MMCOC_RSP_48;
			break;
		case MMC_RSP_R2:
			cmd_command |= MMCOC_RSP_136;
			break;
		case MMC_RSP_R3:
			cmd_command |= MMCOC_RSP_48;
			break;
		default:
			printk(KERN_INFO "mmc_ocores: unhandled response type %02x\n",
			       mmc_resp_type(cmd));
		}
		 //Send Command
		CMD_IRQ_ON (host,(ECC|EEI));
		writel(cmd_command, host->base + SD_COMMAND);
		wmb();
		writel(cmd_arg, host->base + SD_ARG);	
		//printk(KERN_ALERT "%s: cmd_arg = %08x\n", __FUNCTION__, cmd_arg);
		//printk(KERN_ALERT "%s: cmd_command   = %08x\n", __FUNCTION__, cmd_command);
		
	}
	
	
}

static void ocores_process_next(struct ocores_host *host)
{
	host->word_cnt=0;
	
	if (!(host->flags & FL_SENT_COMMAND)) {
		host->flags |= FL_SENT_COMMAND;
		ocores_start_cmd(host, host->mrq->cmd);
	}
	else if ((!(host->flags & FL_SENT_STOP)) && host->mrq->stop) {
		host->flags |= FL_SENT_STOP;
		ocores_start_cmd(host, host->mrq->stop);
	}
}

static void ocores_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ocores_host *host = mmc_priv(mmc);

	
	host->mrq = mrq;
	host->flags = 0;
	ocores_process_next(host);


	//printk(KERN_ALERT "%s: exit\n", __FUNCTION__);
}



static int ocores_get_ro(struct mmc_host *mmc)
{
	/* struct ocores_host *host = mmc_priv(mmc); */

	printk(KERN_ALERT "%s: enter\n", __FUNCTION__);


	/* /\* Host doesn't support read only detection so assume writeable *\/ */
	return 0;
}

static void ocores_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ocores_host *host = mmc_priv(mmc);
	int clk_div, cmd_timeout;
	/* struct ocores_host *host = mmc_priv(mmc); */
       
	printk(KERN_ALERT "%s: clock = 0x%08x\n", __FUNCTION__, ios->clock);
	printk(KERN_ALERT "%s: vdd = 0x%04x\n", __FUNCTION__, ios->vdd);
	printk(KERN_ALERT "%s: bus_mode = 0x%02x\n", __FUNCTION__, ios->bus_mode);
	printk(KERN_ALERT "%s: power_mode = 0x%02x\n", __FUNCTION__, ios->power_mode);
	printk(KERN_ALERT "%s: bus_width = 0x%02x\n", __FUNCTION__, ios->bus_width);
	printk(KERN_ALERT "%s: timing = 0x%02x\n", __FUNCTION__, ios->timing);

	//Set clock divider and timeout registers
    	printk(KERN_ALERT "%s: host->base = %p\n", __FUNCTION__, host->base);
	if (ios->clock == 0) {
		//ocores_mci_write (host, SD_SOFTWARE_RST, SD_DISABLE);
	}
	else if (ios->clock != host->clock)
	{       
		host->clock = ios->clock;
		writel(SD_DISABLE, host->base + SD_SOFTWARE_RST);
		clk_div = ((SYS_CLK / ios->clock)-2 )/ 2;
		cmd_timeout = ((SYS_CLK/ios->clock) * 512);

		printk(KERN_ALERT " clk_div = 0x%02x\n", clk_div);
		printk(KERN_ALERT " cmd_timeout = 0x%02x\n", cmd_timeout);

		writel (clk_div, host->base  + SD_CLOCK_DIVIDER);
		writel (cmd_timeout, host->base  + SD_TIMEOUT);


		writel(SD_ENABLE, host->base + SD_SOFTWARE_RST);
	}
         
}



static irqreturn_t ocores_irq_cmd(int irq, void *devid)
{
	 struct ocores_host *host = (struct ocores_host *) devid;

	 disable_irq(host->irq_cmd);

	 	//printk(KERN_ALERT "%s: IRQ START***** Normal In  = %08x\n", __FUNCTION__, readl(host->base + SD_NORMAL_INT_STATUS));

	 host->registers.normal_int_status  = readl(host->base + SD_NORMAL_INT_STATUS);
	 rmb();
	 host->registers.error_int_status  = readl(host->base + SD_ERROR_INT_STATUS);

	 writel(0,host->base + SD_NORMAL_INT_STATUS);
	 writel(0,host->base + SD_ERROR_INT_STATUS);

	 //printk(KERN_ALERT "%s: IRQ END***** Error In  = %08x\n", __FUNCTION__, readl(host->base + SD_ERROR_INT_STATUS));
	 tasklet_schedule(&host->finish_cmd);
	 CMD_IRQ_OFF (host,(ECC|EEI));
	 enable_irq(host->irq_cmd);

	 return IRQ_HANDLED;
}

static irqreturn_t ocores_irq_dat(int irq, void *devid)
{
	 struct ocores_host *host = (struct ocores_host *) devid;

	 //disable_irq(host->irq_dat);
	 
	 host->registers.data_int_status  = readl(host->base + SD_BD_ISR);	
         
	 writel(0,host->base + SD_BD_ISR);	
	 tasklet_schedule(&host->finish_data);
	
	 //enable_irq(host->irq_dat);

	 return IRQ_HANDLED;
	
	
	

}

static const struct mmc_host_ops ocores_ops = {
	.request		= ocores_request,
	.get_ro			= ocores_get_ro,
	.set_ios		= ocores_set_ios,
	/* .enable_sdio_irq	= ocores_enable_sdio_irq, */
};

static int ocores_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct ocores_host *host = NULL;
	struct resource *r;
	int ret;
        
       
	printk(KERN_ALERT "%s: enter\n", __FUNCTION__);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	printk(KERN_ALERT "%s: resource %x, %x\n", __FUNCTION__, r->start, r->end);

	r = request_mem_region(r->start, r->end - r->start, DRIVER_NAME);
	if (!r) {
		return -EBUSY;
	}

	mmc = mmc_alloc_host(sizeof(struct ocores_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}


	mmc->ops = &ocores_ops;

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA;
	mmc->f_min = 700000;  //SYS_CLK  60; 0.7 Mhz
	mmc->f_max = 6166666;  //SYS_CLK;   4.166 666 mhz

	mmc->max_blk_count =  4;//8; //XXX: 8
	mmc->max_hw_segs = 2;
	mmc->max_blk_size = MMCOC_MAX_BLOCK_SIZE;
	//mmc->max_seg_size = mmc->max_blk_count * mmc->max_blk_size;
	
	
	mmc->max_seg_size = PAGE_SIZE;
	mmc->max_req_size = mmc->max_seg_size;
	mmc->max_phys_segs = 1; //BD size

	host = mmc_priv(mmc);
	 
	 /*XXX:  */
        host->clock = 0;
        
	host->mmc = mmc;
	host->cmdat = 0;
	host->registers.normal_int_status =0;
	
	printk("Free FBD \n");
	host->free_tx_bd=0;
	host->free_rx_bd=0;
	printk("Free EBD \n");
	
	
	
	tasklet_init(&host->finish_cmd, ocores_tasklet_finish_cmd,
		     (unsigned long) host);
		     
	tasklet_init(&host->finish_data, ocores_tasklet_finish_data,
		     (unsigned long) host);
    		     

	spin_lock_init(&host->lock);
	host->res = r;

	host->base = ioremap(r->start, r->end - r->start +1);
	/* host->base = (void *)r->start; */
	if (!host->base) {
		ret = -ENOMEM;
		goto out;
	}
	host->free_tx_bd=readl( host->base + BD_STATUS );	
	host->free_tx_bd=host->free_tx_bd & 0x00ff;
	printk("Free TX BD = %d\n", host->free_tx_bd);
	host->free_rx_bd=readl( host->base + BD_STATUS );	
	host->free_rx_bd=(host->free_rx_bd & 0xff00)>>8;
	printk("Free TX BD = %d\n", host->free_rx_bd);
	
	host->pdata = pdev->dev.platform_data;
	mmc->ocr_avail = host->pdata->ocr_mask;

	host->irq_cmd = platform_get_irq_byname(pdev, "cmd_irq");
	ret = request_irq(host->irq_cmd, ocores_irq_cmd, IRQF_DISABLED, DRIVER_NAME, host);
	printk(KERN_ALERT "%s: IRQ dat resource  %x\n", __FUNCTION__, host->irq_cmd );
	printk(KERN_ALERT "%s: RET cmd irq %x\n", __FUNCTION__, ret);
	if (ret)
		goto out;
	disable_irq(host->irq_cmd);

	host->irq_dat = platform_get_irq_byname(pdev, "dat_irq");
	ret = request_irq(host->irq_dat, ocores_irq_dat, IRQF_DISABLED, DRIVER_NAME, host);
	printk(KERN_ALERT "%s: IRQ dat resource  %x\n", __FUNCTION__, host->irq_dat );
	printk(KERN_ALERT "%s: RET Dat irq  %x\n", __FUNCTION__, ret);
	if (ret)
		goto out;
	disable_irq(host->irq_dat);



	enable_irq(host->irq_cmd);
   	enable_irq(host->irq_dat);
	printk(KERN_ALERT "%s: host->base = %p\n", __FUNCTION__, host->base);
	printk(KERN_ALERT "%s: SD_BLOCK = %08x\n", __FUNCTION__, readl(host->base + SD_BLOCK));
	printk(KERN_ALERT "%s: host->pdata->ocr_mask = %08x\n", __FUNCTION__, host->pdata->ocr_mask);
	
	mmc_add_host(mmc);

	printk(KERN_ALERT "%s: exit\n", __FUNCTION__);

	return 0;

 out:
  	printk(KERN_ALERT "%s: ERROR REQUESTINING RESOURCES\n", __FUNCTION__);
	if (mmc) {
		mmc_free_host(mmc);
	}
	release_resource(r);

	return ret;
}

static void ocores_tasklet_finish_cmd(unsigned long param)
{
	struct ocores_host *host = (struct ocores_host *) param;
	
	struct mmc_data *data;
	struct scatterlist *sg;	
	data = host->cmd->data;
	sg = &data->sg[0];
	
	//printk(KERN_ALERT " CMD TASKLET RUNNS************\n");
	//Check For Transmissions errors
	if ((host->registers.normal_int_status & EI) == EI)
	{
		printk(KERN_ALERT "TRANSMISSION ERROR DETECTED");
		switch ( host->registers.error_int_status )
     	 	{
		case (CTE):
			pr_debug("Card took too long to respond\n");
			host->mrq->cmd->error = -ETIMEDOUT ;
			if (host->mrq->stop)
				host->mrq->stop->error = -ETIMEDOUT ;
     	 		break;
		case (CCRC  ):
			pr_debug(" CRC  problem with the received or sent data\n");
			host->mrq->cmd->error = -EILSEQ;
			if (host->mrq->stop)	
				host->mrq->stop->error = -EILSEQ ;
     	 		break;
		case (CIE  ):
			pr_debug("Index problem with the received or sent data\n");
			host->mrq->cmd->error = -EILSEQ;
			if (host->mrq->stop)
				host->mrq->stop->error = -EILSEQ ;
     	 		break;
		}
		
		mmc_request_done(host->mmc, host->mrq);
	}
	else
	{         
		if ( mmc_resp_type(host->mrq->cmd) == MMC_RSP_R2      )   //Long response
		{
			printk(KERN_ALERT "Long Response, Word Cnt %d, RESP  * = %08x\n ",host->word_cnt,readl(host->base + SD_RESP1));
			
			if (host->mrq->cmd->opcode == 2) { //XXX: Hack until supported long response
				host->mrq->cmd->resp[0]  =  readl(host->base + SD_RESP1);
				host->mrq->cmd->resp[1]  =  0x01302331;
				host->mrq->cmd->resp[2]  =  0x195abdef;
				host->mrq->cmd->resp[3]  =  0x195abdef;
				mmc_request_done(host->mmc, host->mrq);			
			} //XXX: Hack until supported long response
			else if(host->mrq->cmd->opcode == 51)
			{
				host->mrq->cmd->resp[0] = 0x900;
				host->mrq->cmd->resp[1] = 0xaaa;
				host->mrq->cmd->resp[2] = 0xbbb;
				host->mrq->cmd->resp[3] = 0xccc;
				host->mrq->data->bytes_xfered =8;
				dma_unmap_sg(mmc_dev(host->mmc), sg, data->sg_len, DMA_FROM_DEVICE);
				mmc_request_done(host->mmc, host->mrq);	
			} 
			
		
			
			else {
				host->word_cnt+=1;
				switch(host->word_cnt-1)
				{
				case (0):
					host->mrq->cmd->resp[0]  =  readl(host->base + SD_RESP1);
					ocores_start_cmd(host, host->mrq->cmd);
					break;
				case (1):
					host->mrq->cmd->resp[1]  =  readl(host->base + SD_RESP1);
					ocores_start_cmd(host, host->mrq->cmd);
					break;
				case (2):
					host->mrq->cmd->resp[2]  =  readl(host->base + SD_RESP1);
					ocores_start_cmd(host, host->mrq->cmd);
					break;
				case (3):
					host->mrq->cmd->resp[3]  =  readl(host->base + SD_RESP1);
					mmc_request_done(host->mmc, host->mrq);
					break;
				}
			}
		}
		else  //Short response
		{
			host->mrq->cmd->error = 0 ;
			
			if (host->mrq->stop)
				host->mrq->stop->resp[0] = readl(host->base + SD_RESP1);
			else
				host->mrq->cmd->resp[0] = readl(host->base + SD_RESP1);
			//printk(KERN_ALERT "Short Response CMD RSP * = %08x\n", host->mrq->cmd->resp[0]);
			mmc_request_done(host->mmc, host->mrq);
		}
	}
	
}
static void ocores_tasklet_finish_data(unsigned long param)
{
	struct ocores_host *host = (struct ocores_host *) param;
	
	
	struct mmc_data *data;
	struct scatterlist *sg;
	int free_bd,i;
	int *adr;
	data = host->cmd->data;
	sg = &data->sg[0]; //XXX:O Change to dynamic later?
	
	
	
	//IF read operation
	
	if ((host->registers.data_int_status & TRS) == TRS){ 			           
		if (data->flags & MMC_DATA_READ){
			free_bd=readl( host->base + BD_STATUS );
			free_bd=(free_bd&0xff00)>>8;
			//printk(KERN_ALERT " DATA READ TASKLET RUNNS*** Free BD %d\n", free_bd);
			if (free_bd == host->free_rx_bd) {
				dma_unmap_sg(mmc_dev(host->mmc), sg, sg->length, DMA_FROM_DEVICE);
				host->mrq->cmd->resp[0]  =  readl(host->base + SD_RESP1);
				
				DAT_IRQ_OFF (host,(TRE|CMDE|FIFOE|MRC|TRS));
				data->bytes_xfered = sg->length;		
									
				if (host->mrq->stop) 	
					host->mrq->stop->resp[0] = readl(host->base + SD_RESP1);
				
				
				mmc_request_done(host->mmc, host->mrq);
						
			}
		} else if (data->flags & MMC_DATA_WRITE){
			free_bd=readl( host->base + BD_STATUS );
			free_bd=(free_bd&0x00FF);
			//printk(KERN_ALERT " DATA WRITE TASKLET RUNNS*** Free BD %d\n", free_bd);
			if (free_bd == host->free_tx_bd) {
				dma_unmap_sg(mmc_dev(host->mmc), sg, sg->length, DMA_TO_DEVICE);
				host->mrq->cmd->resp[0]  =  readl(host->base + SD_RESP1);
				
				DAT_IRQ_OFF (host,(TRE|CMDE|FIFOE|MRC|TRS));
				data->bytes_xfered = sg->length;	
				
								
				if (host->mrq->stop) 		
					host->mrq->stop->resp[0] = readl(host->base + SD_RESP1);
				
				
				mmc_request_done(host->mmc, host->mrq);
							
			}
			
		}
	  	
	}		
	else {  printk(KERN_ALERT "DATA TRANS ERROR %d\n", host->registers.data_int_status);
				
				data->error = -ETIMEDOUT;
			if ((host->registers.data_int_status & MRC) == MRC)
				data->error = -ETIMEDOUT;
			if ((host->registers.data_int_status &  CMDE) ==  CMDE)		
				data->error = -EILSEQ;
			data->bytes_xfered =0;
		
		host->mrq->cmd->resp[0]  =  readl(host->base + SD_RESP1);
		dma_unmap_sg(mmc_dev(host->mmc), sg, data->sg_len, DMA_FROM_DEVICE);
		DAT_IRQ_OFF (host,(TRE|FIFOE|MRC|TRS));	
		mmc_request_done(host->mmc, host->mrq);	
		
	
	}
	
	
	
	
	
	
	

}
static int ocores_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct ocores_host *host;
	struct resource *r;
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	
	printk(KERN_ALERT "%s: enter\n", __FUNCTION__);

	platform_set_drvdata(pdev, NULL);
	
	tasklet_kill(&host->finish_cmd);
	tasklet_kill(&host->finish_data);
	
	free_irq(host->irq_cmd, host);
	free_irq(host->irq_dat, host);
	
	iounmap(host->base);

	release_mem_region(r->start, r->end - r->start + 1);
	if (mmc) {
		struct ocores_host *host = mmc_priv(mmc);

		mmc_remove_host(mmc);

		release_resource(host->res);

		mmc_free_host(mmc);
	}

	printk(KERN_ALERT "%s: exit\n", __FUNCTION__);

	return 0;
}

#ifdef CONFIG_PM
static int ocores_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	printk(KERN_ALERT "%s: enter\n", __FUNCTION__);

	if (mmc) {
		ret = mmc_suspend_host(mmc, state);
	}

	printk(KERN_ALERT "%s: exit\n", __FUNCTION__);

	return ret;
}

static int ocores_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	printk(KERN_ALERT "%s: enter\n", __FUNCTION__);

	if (mmc) {
		ret = mmc_resume_host(mmc);
	}

	printk(KERN_ALERT "%s: exit\n", __FUNCTION__);

	return ret;
}
#else
#define ocores_suspend	NULL
#define ocores_resume	NULL
#endif

static struct platform_driver ocores_driver = {
	.probe		= ocores_probe,
	.remove		= ocores_remove,
	.suspend	= ocores_suspend,
	.resume		= ocores_resume,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init ocores_init(void)
{
	printk(KERN_ALERT "%s: registering ocores platform_driver\n", __FUNCTION__);

	return platform_driver_register(&ocores_driver);
}

static void __exit ocores_exit(void)
{
	printk(KERN_ALERT "%s: unregistering ocores platform_driver\n", __FUNCTION__);

	platform_driver_unregister(&ocores_driver);
}

module_init(ocores_init);
module_exit(ocores_exit);

MODULE_DESCRIPTION("OpenCores Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");
