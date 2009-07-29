/*
 *  linux/arch/or32/kernel/setup.c
 *
 *  or32 version
 *    author(s): Matjaz Breskvar (phoenix@bsemi.com)
 *
 *  derived from cris, i386, m68k, ppc, sh ports.
 *
 *  changes:
 *  18. 11. 2003: Matjaz Breskvar (phoenix@bsemi.com)
 *    initial port to or32 architecture
 *
 */
 
/*
 * This file handles the architecture-dependent parts of initialization
 */
 
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/user.h>
#include <linux/a.out.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/seq_file.h>
#include <linux/serial.h>
 
#include <asm/board.h>
#include <asm/segment.h>
#include <asm/system.h>
#include <asm/smp.h>
#include <asm/pgtable.h>
#include <asm/types.h>
#include <asm/setup.h>
#include <asm/io.h>

/*
 * Debugging stuff
 */

int __phx_mmu__ = 0;
int __phx_warn__ = 0;
int __phx_debug__ = 0;
int __phx_signal__ = 0;
 
/*
 * Setup options
 */
 
extern int root_mountflags;
extern char _stext, _etext, _edata, _end;
extern int __init setup_early_serial8250_console(char *cmdline);
#ifdef CONFIG_BLK_DEV_INITRD
extern unsigned long initrd_start, initrd_end;
extern char __initrd_start, __initrd_end;
extern char __initramfs_start, __initramfs_end;
#endif
 
#include <asm/machdep.h>

extern void config_BSP(char *command, int len);

void (*mach_tick)(void) = NULL;
int (*mach_set_clock_mmss) (unsigned long) = NULL;
void (*mach_sched_init) (void) = NULL;
unsigned long (*mach_gettimeoffset)(void) = NULL;
void (*mach_gettod)(int *year, int *mon, int *day, int *hour, int *min, int *sec) = NULL;
int (*mach_hwclk)(int, struct hwclk_time*) = NULL;
void (*mach_mksound)( unsigned int count, unsigned int ticks) = NULL;
void (*mach_reset)( void ) = NULL;
void (*mach_debug_init)(void) = NULL;


unsigned long fb_mem_start;
static char command_line[COMMAND_LINE_SIZE] = "root=/dev/ram console=uart,mmio,0x90000000";
/*static char command_line[COMMAND_LINE_SIZE] = "root=/dev/ram console=ttyS0";*/
extern const unsigned long text_start, edata; /* set by the linker script */ 
       
static unsigned long __init setup_memory(void)
{
	unsigned long bootmap_size, start_pfn, max_low_pfn;


#define PFN_UP(x)       (((x) + PAGE_SIZE-1) >> PAGE_SHIFT)
#define PFN_DOWN(x)     ((x) >> PAGE_SHIFT)
#define PFN_PHYS(x)     ((x) << PAGE_SHIFT)

#ifndef CONFIG_FB_OC_SHMEM_SIZE
#define CONFIG_FB_OC_SHMEM_SIZE 0
#endif /* CONFIG_FB_OC_SHMEM_SIZE */


	/* min_low_pfn points to the start of DRAM, start_pfn points
	 * to the first DRAM pages after the kernel, and max_low_pfn
	 * to the end of DRAM. Partial pages are not useful, so round it 
	 * down.
	 */ 
	start_pfn   = PFN_UP(__pa(&_end));
	max_low_pfn = PFN_DOWN(CONFIG_OR32_MEMORY_SIZE
			       -CONFIG_FB_OC_SHMEM_SIZE
			       -CONFIG_OR32_RESERVED_MEM_SIZE);
	min_low_pfn = PAGE_OFFSET >> PAGE_SHIFT;

#undef CONFIG_FB_OC_SHMEM_SIZE

	/*
	 * set the beginning of frame buffer
	 */
	fb_mem_start = PFN_PHYS(max_low_pfn);

	/* 
	 * initialize the boot-time allocator (with low memory only)
	 */ 
	bootmap_size = init_bootmem(start_pfn, max_low_pfn);
	free_bootmem(PFN_PHYS(start_pfn), PFN_PHYS(max_low_pfn - start_pfn));
	reserve_bootmem(PFN_PHYS(start_pfn), bootmap_size);

	return(max_low_pfn);
}

static inline unsigned long extract_value_bits(unsigned long reg, 
					       short bit_nr, short width)
{
	return((reg >> bit_nr) & (0 << width));
}

static inline unsigned long extract_value(unsigned long reg, 
					  unsigned long mask)
{
	while (!(mask & 0x1)) {
		reg  = reg  >> 1;
		mask = mask >> 1;
	}
	return(mask & reg);
}

void __init detect_timer(void) 
{
	unsigned long cfg;

	cfg = mfspr(SPR_TTMR);
	printk("period 0x%x\n", SPR_TTMR & SPR_TTMR_PERIOD);
}

void __init detect_unit_config(unsigned long upr, unsigned long mask,
			       char *text, void (*func)(void))
{
        if (text != NULL)
		printk("%s", text);

	if ( upr & mask ) {
		if (func != NULL)
			func();
		else
			printk("present\n");
	}
	else
		printk("N/A\n");
}


void __init detect_soc_generic(unsigned long upr)
{
	detect_unit_config(upr, SPR_UPR_DCP,  "  dCACHE: ", NULL);
	detect_unit_config(upr, SPR_UPR_ICP,  "  iCACHE: ", NULL);
	detect_unit_config(upr, SPR_UPR_DMP,  "  dMMU\t: ", NULL);
	detect_unit_config(upr, SPR_UPR_IMP,  "  iMMU\t: ", NULL);
	detect_unit_config(upr, SPR_UPR_DUP,  "  debug : ", NULL);
	detect_unit_config(upr, SPR_UPR_PCUP, "  PerfC : ", NULL);
	detect_unit_config(upr, SPR_UPR_PMP,  "  PM    : ", NULL);
	detect_unit_config(upr, SPR_UPR_PICP, "  PIC   : ", NULL);
	detect_unit_config(upr, SPR_UPR_TTP,  "  TIMER : ", detect_timer);
	detect_unit_config(upr, SPR_UPR_CUST, "  CUs   : ", NULL);

/* add amount configured memory 
 */
}

void __init detect_soc_rev_0(unsigned long upr)
{
	
	printk("  WARNING, using default values !\n"
               "           (some early revision 0 processors don't have unit present register\n"
	       "            populated with all avaliable units)\n");

	printk("  dCACHE: assumed %4d Kb size, %2d bytes/line, %d way(s)", 
	       CONFIG_OR32_DC_SIZE, CONFIG_OR32_DC_LINE, 1);
	detect_unit_config(upr, SPR_UPR_DCP,  ", detected: ", NULL);

	printk("  iCACHE: assumed %4d Kb size, %2d bytes/line, %d way(s)", 
	       CONFIG_OR32_IC_SIZE, CONFIG_OR32_IC_LINE, 1);
	detect_unit_config(upr, SPR_UPR_ICP,  ", detected: ", NULL);

	printk("  dMMU\t: assumed %4d entries, %d way(s)", 
	       CONFIG_OR32_DTLB_ENTRIES, 1);
	detect_unit_config(upr, SPR_UPR_DMP,  ", detected: ", NULL);

	printk("  iMMU\t: assumed %4d entries, %d way(s)", 
	       CONFIG_OR32_ITLB_ENTRIES, 1);
	detect_unit_config(upr, SPR_UPR_IMP,  ", detected: ", NULL);
	detect_unit_config(upr, SPR_UPR_DUP,  "  debug : unknown (guess yes), detected: ", NULL);
	detect_unit_config(upr, SPR_UPR_PCUP, "  PerfC : unknown (guess no ), detected: ", NULL);
	detect_unit_config(upr, SPR_UPR_PMP,  "  PM    : unknown (guess yes), detected: ", NULL);
	detect_unit_config(upr, SPR_UPR_PICP, "  PIC   : unknown (guess yes), detected: ", NULL);
	detect_unit_config(upr, SPR_UPR_TTP,  "  TIMER : unknown (guess yes), detected: ", NULL);
	detect_unit_config(upr, SPR_UPR_CUST, "  CUs   : unknown (guess no ), detected: ", NULL);

/* add amount configured memory 
 */
}

unsigned long su_asm(void);

void __init detect_soc(void)
{
	unsigned long upr, cfg, version, revision;

	upr = mfspr(SPR_UPR);
	if (upr &  SPR_UPR_UP)
		printk("Detecting Processor units:\n");
	else {
		printk("Unit Present Register not avaliable\n");
		return;
	}
	       
	cfg = mfspr(SPR_VR);
	version=extract_value(cfg, SPR_VR_VER);
	revision=extract_value(cfg, SPR_VR_REV);

#ifndef CONFIG_OR32_ANONYMOUS
	printk("  CPU\t: or32/OpenRISC-%lx, revision %lx, @%d MHz, %s\n", 
	       version, revision,
	       CONFIG_OR32_SYS_CLK,
	       (upr & (unsigned long)SPR_UPR_SRP) ? 
	       "with shadow registers" : "with no shadow registers");

	if ((version == 0x1200) & (revision == 0)) {
		detect_soc_rev_0(upr);
	} else {
		detect_soc_generic(upr);
	}
#endif /* CONFIG_OR32_ANONYMOUS */

	printk("  Signed 0x%lx\n", su_asm());
}


void __init setup_arch(char **cmdline_p)
{
	unsigned long max_low_pfn;

	config_BSP(&command_line[0], COMMAND_LINE_SIZE);

	/* detect System on Chip parameters */
	detect_soc();

	/* process 1's initial memory region is the kernel code/data */
	init_mm.start_code = (unsigned long) &_stext;
	init_mm.end_code =   (unsigned long) &_etext;
	init_mm.end_data =   (unsigned long) &_edata;
	init_mm.brk =        (unsigned long) &_end;

#ifdef CONFIG_BLK_DEV_INITRD
	initrd_start = (unsigned long)&__initrd_start;
	initrd_end = (unsigned long)&__initrd_end;
#endif

        /* setup bootmem allocator */
	max_low_pfn = setup_memory();
		
	/* paging_init() sets up the MMU and marks all pages as reserved */
	paging_init();
	
#ifdef CONFIG_SERIAL_8250_CONSOLE
	//	early_serial_console_init(command_line); RGD
	setup_early_serial8250_console(command_line);
#endif

#if defined(CONFIG_VT) && defined(CONFIG_DUMMY_CONSOLE)
	if(!conswitchp)
        	conswitchp = &dummy_con;
#endif

	*cmdline_p = command_line;
	
	/* Save command line copy for /proc/cmdline RGD removed 2.6.21*/
	//memcpy(saved_command_line, command_line, COMMAND_LINE_SIZE);
	//saved_command_line[COMMAND_LINE_SIZE-1] = '\0';

	/* fire up 8051 */
//	printk("Starting 8051...\n");
//	oc8051_init();

#ifndef CONFIG_OR32_ANONYMOUS
	printk("Linux/or32 port 2003-2005 OpenCores <phoenix@bsemi.com>\n");
#endif /* CONFIG_OR32_ANONYMOUS */
}

static int show_cpuinfo(struct seq_file *m, void *v)
{

#ifndef CONFIG_OR32_ANONYMOUS
	return seq_printf(m,
			  "cpu\t\t: or32\n"
			  "cpu revision\t: %lu\n"
			  "cpu model\t: %s\n"
			  "icache size\t: %d kB\n"
			  "dcache size\t: %d kB\n"
			  "immu\t\t: %s\n"
			  "dmmu\t\t: %s\n"
			  "fpu\t\t: %s\n"
			  "ethernet\t: %s Mbps\n"
			  "bogomips\t: %lu.%02lu\n",
			  
			  0UL,
			  "Marvin",
			  8,
			  8,
			  "64 entries, 1 way",
			  "64 entries, 1 way",
			  "no",
			  "10/100",
			  (loops_per_jiffy * HZ) / 500000,
			  ((loops_per_jiffy * HZ) / 5000) % 100);
#else /* CONFIG_OR32_ANONYMOUS */
	return seq_printf(m,
			  "cpu revision\t: %lu\n"
			  "icache size\t: %d kB\n"
			  "dcache size\t: %d kB\n"
			  "immu\t\t: %s\n"
			  "dmmu\t\t: %s\n"
			  "ethernet\t: %s Mbps\n",
			  
			  0UL,
			  8,
			  8,
			  "64 entries, 1 way",
			  "64 entries, 1 way",
			  "10/100");
#endif /* CONFIG_OR32_ANONYMOUS */
			  
			  
}

static void *c_start(struct seq_file *m, loff_t *pos)
{
	/* We only got one CPU... */
	return *pos < 1 ? (void *)1 : NULL;
}
 
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
}

struct seq_operations cpuinfo_op = {
	start:  c_start,
	next:   c_next,
	stop:   c_stop,
	show:   show_cpuinfo,
};

void arch_gettod(int *year, int *mon, int *day, int *hour,
		                  int *min, int *sec)
{
   
	if (mach_gettod)
		mach_gettod(year, mon, day, hour, min, sec);
	else
		*year = *mon = *day = *hour = *min = *sec = 0;
}

/*RGD this awful hack is because our compiler does
 *support the "weak" attribute correctly at this time
 *once we do (support weak) this should be removed!!
 */
extern const void __start_notes(){}
extern const void __stop_notes(){}
