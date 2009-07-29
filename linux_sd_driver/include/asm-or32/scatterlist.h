#ifndef __ASM_OR32_SCATTERLIST_H
#define __ASM_OR32_SCATTERLIST_H

struct scatterlist {
#ifdef CONFIG_DEBUG_SG
	unsigned long sg_magic;
#endif
	dma_addr_t  address;    /* Location data is to be transferred to */
	unsigned int length;

	/* The following is i386 highmem junk - not used by us */
	unsigned long page_link; /* Location for highmem page, if any */
	unsigned int offset;/* for highmem, page offset */

};

#define sg_dma_address(sg)	((sg)->address)
#define sg_dma_len(sg)		((sg)->length)
/* i386 junk */

#define ISA_DMA_THRESHOLD (0x1fffffff)

#endif /* !(__ASM_OR32_SCATTERLIST_H) */
