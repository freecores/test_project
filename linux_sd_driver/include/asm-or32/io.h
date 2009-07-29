#ifndef _OR32_IO_H
#define _OR32_IO_H

#include <asm/page.h>   /* for __va, __pa */


/*
 * Change virtual addresses to physical addresses and vv.
 */

static inline unsigned long virt_to_phys(volatile void * address)
{
	return __pa(address);
}

static inline void * phys_to_virt(unsigned long address)
{
	return __va(address);
}

extern void * __ioremap(unsigned long offset, unsigned long size, unsigned long flags);

extern inline void * ioremap(unsigned long offset, unsigned long size)
{
	return __ioremap(offset, size, 0);
}

/* #define _PAGE_CI       0x002 */
extern inline void * ioremap_nocache(unsigned long offset, unsigned long size)
{
	return __ioremap(offset, size, 0x002);
}

extern void iounmap(void *addr);

#define page_to_phys(page)	((page - mem_map) << PAGE_SHIFT)

/*
 * IO bus memory addresses are also 1:1 with the physical address
 */
#define virt_to_bus virt_to_phys
#define bus_to_virt phys_to_virt

/*
 * readX/writeX() are used to access memory mapped devices. On some
 * architectures the memory mapped IO stuff needs to be accessed
 * differently. On the or32 architecture, we just read/write the
 * memory location directly.
 */
#define readb(addr) (*(volatile unsigned char *) (addr))
#define readw(addr) (*(volatile unsigned short *) (addr))
#define readl(addr) (*(volatile unsigned int *) (addr))

#define writeb(b,addr) ((*(volatile unsigned char *) (addr)) = (b))
#define writew(b,addr) ((*(volatile unsigned short *) (addr)) = (b))
#define writel(b,addr) ((*(volatile unsigned int *) (addr)) = (b))
#define __raw_writel writel /*RGD*/

#define memset_io(a,b,c)	memset((void *)(a),(b),(c))
#define memcpy_fromio(a,b,c)	memcpy((a),(void *)(b),(c))
#define memcpy_toio(a,b,c)	memcpy((void *)(a),(b),(c))

/*
 * Again, or32 does not require mem IO specific function.
 */

#define eth_io_copy_and_sum(a,b,c,d)	eth_copy_and_sum((a),(void *)(b),(c),(d))

#define IO_BASE			0x0
#define IO_SPACE_LIMIT 		0xffffffff

#define inb(port)		(*(volatile unsigned char *) (port+IO_BASE))
#define outb(value,port)	((*(volatile unsigned char *) (port+IO_BASE)) = (value))	

#define inb_p(port)             inb((port))
#define outb_p(val, port)       outb((val), (port))

/*
 * Convert a physical pointer to a virtual kernel pointer for /dev/mem
 * access
 */
#define xlate_dev_mem_ptr(p)	__va(p)

/*
 * Convert a virtual cached pointer to an uncached pointer
 */
#define xlate_dev_kmem_ptr(p)	p


/* Create a virtual mapping cookie for an IO port range */
extern void __iomem *ioport_map(unsigned long port, unsigned int nr);
extern void ioport_unmap(void __iomem *);

#endif
