/* $Id: dma.h,v 1.2 2001/05/09 12:17:42 johana Exp $ */

#ifndef _ASM_DMA_H
#define _ASM_DMA_H

#define MAX_DMA_ADDRESS PAGE_OFFSET

#ifdef CONFIG_PCI
extern int isa_dma_bridge_buggy;
#else
#define isa_dma_bridge_buggy    (0)
#endif

#endif /* _ASM_DMA_H */