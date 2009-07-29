/*
 *  linux/arch/or32/kernel/irq.c
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

#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/kernel_stat.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/init.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/page.h>
#include <asm/machdep.h>
#include <asm/or32-hf.h>

/* table for system interrupt handlers */
static or_irq_handler_t irq_list[NR_IRQS]; /*RGD*/

static const char *default_names[NR_IRQS] = {
	"int0", "int1", "int2", "int3",	"int4", "int5", "int6", "int7"
	"int8", "int9", "int10", "int11", "int12", "int13", "int14", "int15"
	"int16", "int17", "int18", "int19", "int20", "int21", "int22", "int23"
	"int24", "int25", "int26", "int27", "int28", "int29", "int30", "int31"
};

int pic_enable_irq(unsigned int irq)
{
	/* Enable in the IMR */
	mtspr(SPR_PICMR, mfspr(SPR_PICMR) | (0x00000001L << irq));

	return 0;
}

int pic_disable_irq(unsigned int irq)
{
	/* Disable in the IMR */
	mtspr(SPR_PICMR, mfspr(SPR_PICMR) & ~(0x00000001L << irq));

	return 0;
}

int pic_init(void)
{
	/* turn off all interrupts */
	mtspr(SPR_PICMR, 0);
	return 0;
}

int pic_do_irq(struct pt_regs *fp)
{
	int irq;
	int mask;

	unsigned long pend = mfspr(SPR_PICSR) & 0xfffffffc;

	if (pend & 0x0000ffff) {
		if (pend & 0x000000ff) {
			if (pend & 0x0000000f) {
				mask = 0x00000001;
				irq = 0;
			} else {
				mask = 0x00000010;
				irq = 4;
			}
		} else {
			if (pend & 0x00000f00) {
				mask = 0x00000100;
				irq = 8;
			} else {
				mask = 0x00001000;
				irq = 12;
			}
		}
	} else if(pend & 0xffff0000) {
		if (pend & 0x00ff0000) {
			if (pend & 0x000f0000) {
				mask = 0x00010000;
				irq = 16;
			} else {
				mask = 0x00100000;
				irq = 20;
			}
		} else {
			if (pend & 0x0f000000) {
				mask = 0x01000000;
				irq = 24;
			} else {
				mask = 0x10000000;
				irq = 28;
			}
		}
	} else {
		return -1;
	}

	while (! (mask & pend)) {
		mask <<=1;
		irq++;
	}

//	mtspr(SPR_PICSR, mfspr(SPR_PICSR) & ~mask);
	return irq;
}

void init_IRQ(void)
{
	int i;

	for (i = 0; i < NR_IRQS; i++) {
		irq_list[i].handler = NULL;
		irq_list[i].flags   = IRQ_FLG_STD;
		irq_list[i].dev_id  = NULL;
		irq_list[i].devname = default_names[i];
	}

	pic_init();
	mtspr(SPR_SR, mfspr(SPR_SR) | SPR_SR_IEE);
}


int show_interrupts(struct seq_file *p, void *v)
{
	phx_warn("NOT implemented yet");
#if 0
	int i = *(loff_t *) v;
	struct irqaction * action;
	unsigned long flags;
	
	if (i < NR_IRQS) {
                local_irq_save(flags);
		action = irq_action[i];
                if (!action) 
                        goto skip;
		seq_printf(p, "%2d: %10u %c %s",
			   i, kstat_cpu(0).irqs[i],
			   (action->flags & SA_INTERRUPT) ? '+' : ' ',
			   action->name);
                for (action = action->next; action; action = action->next) {
                        seq_printf(p, ",%s %s",
				   (action->flags & SA_INTERRUPT) ? " +" : "",
				   action->name);
                }
		seq_putc(p, '\n');
	skip:
                local_irq_restore(flags);
        }
#endif
        return 0;
}

asmlinkage void do_IRQ(struct pt_regs *regs)
{
	int irq;
	int cpu;
	unsigned long flags;

//	printk("x");
	irq_enter();
	local_irq_save(flags);

	check_stack(NULL, __FILE__, __FUNCTION__, __LINE__);
	cpu = smp_processor_id();

	while((irq = pic_do_irq(regs)) >= 0) {
//		printk("*");
//		mtspr(SPR_PICMR, mfspr(SPR_PICMR) & ~(1UL << irq));

		if (irq_list[irq].handler)
			irq_list[irq].handler(irq, irq_list[irq].dev_id, regs);
		else
			panic("No interrupt handler for autovector %d\n", irq);
//		mtspr(SPR_PICMR, mfspr(SPR_PICMR) | (1UL << irq));
		mtspr(SPR_PICSR, mfspr(SPR_PICSR) & ~(1UL << irq));

	}
	local_irq_restore(flags);
	
	irq_exit();
}

int request_irq(unsigned int irq,
		irqreturn_t (*handler)(int, void *), /*RGD removed pt_reg*/
		unsigned long flags, const char *devname, void *dev_id)
{
	if (irq >= NR_IRQS) {
		printk("%s: Incorrect IRQ %d from %s\n", __FUNCTION__, irq, devname);
		return -ENXIO;
	}

	if (!(irq_list[irq].flags & IRQ_FLG_STD)) {
		if (irq_list[irq].flags & IRQ_FLG_LOCK) {
			printk("%s: IRQ %d from %s is not replaceable\n",
			       __FUNCTION__, irq, irq_list[irq].devname);
			return -EBUSY;
		}
		if (flags & IRQ_FLG_REPLACE) {
			printk("%s: %s can't replace IRQ %d from %s\n",
			       __FUNCTION__, devname, irq, irq_list[irq].devname);
			return -EBUSY;
		}
	}
	irq_list[irq].handler = handler;
	irq_list[irq].flags   = flags;
	irq_list[irq].dev_id  = dev_id;
	irq_list[irq].devname = devname;

	pic_enable_irq(irq);
	
	return 0;
}

void free_irq(unsigned int irq, void *dev_id)
{
	if (irq >= NR_IRQS) {
		printk("%s: Incorrect IRQ %d\n", __FUNCTION__, irq);
		return;
	}

	pic_disable_irq(irq);

	irq_list[irq].handler = NULL;
	irq_list[irq].flags   = IRQ_FLG_STD;
	irq_list[irq].dev_id  = NULL;
	irq_list[irq].devname = default_names[irq];
}

unsigned long probe_irq_on (void)
{
	return 0;
}

int probe_irq_off (unsigned long irqs)
{
	return 0;
}

void enable_irq(unsigned int irq)
{
	if (irq >= NR_IRQS) {
		printk("%s: Incorrect IRQ %d\n", __FUNCTION__, irq);
		return;
	}
	pic_enable_irq(irq);
}

void disable_irq(unsigned int irq)
{
	if (irq >= NR_IRQS) {
		printk("%s: Incorrect IRQ %d\n", __FUNCTION__, irq);
		return;
	}
	pic_disable_irq(irq);
}

int get_irq_list(char *buf)
{
	int i, len = 0;

	/* autovector interrupts */
	for (i = 0; i < NR_IRQS; i++) {
		if (irq_list[i].handler) {
			if (irq_list[i].flags & IRQ_FLG_LOCK)
				len += sprintf(buf+len, "L ");
			else
				len += sprintf(buf+len, "  ");
			if (irq_list[i].flags & IRQ_FLG_PRI_HI)
				len += sprintf(buf+len, "H ");
			else
				len += sprintf(buf+len, "L ");
			len += sprintf(buf+len, "%s\n", irq_list[i].devname);
		}
	}

	return len;
}

#if 0
void dump(struct pt_regs *fp)
{
	unsigned long	*sp;
	unsigned char	*tp;
	int		i;

	printk("\nCURRENT PROCESS:\n\n");
	printk("COMM=%s PID=%d\n", current->comm, current->pid);
	if (current->mm) {
		printk("TEXT=%08x-%08x DATA=%08x-%08x BSS=%08x-%08x\n",
			(int) current->mm->start_code,
			(int) current->mm->end_code,
			(int) current->mm->start_data,
			(int) current->mm->end_data,
			(int) current->mm->end_data,
			(int) current->mm->brk);
		printk("USER-STACK=%08x  KERNEL-STACK=%08x\n\n",
			(int) current->mm->start_stack,
			(int) current->kernel_stack_page);
	}
	printk("PC: %08lx  Status: %08lx\n",
	       fp->pc, fp->sr);
	printk("R0 : %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx\n",
	       	0L,             fp->sp,      fp->gprs[0], fp->gprs[1], 
		fp->gprs[2], fp->gprs[3], fp->gprs[4], fp->gprs[5]);
	printk("R8 : %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx\n",
	       	fp->gprs[6], fp->gprs[7], fp->gprs[8], fp->gprs[9], 
		fp->gprs[10], fp->gprs[11], fp->gprs[12], fp->gprs[13]);
	printk("R16: %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx\n",
	       	fp->gprs[14], fp->gprs[15], fp->gprs[16], fp->gprs[17], 
		fp->gprs[18], fp->gprs[19], fp->gprs[20], fp->gprs[21]);
	printk("R24: %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx  %08lx\n",
	       	fp->gprs[22], fp->gprs[23], fp->gprs[24], fp->gprs[25], 
		fp->gprs[26], fp->gprs[27], fp->gprs[28], fp->gprs[29]);

	printk("\nUSP: %08lx   TRAPFRAME: %08x\n",
		fp->sp, (unsigned int) fp);

	printk("\nCODE:");
	tp = ((unsigned char *) fp->pc) - 0x20;
	for (sp = (unsigned long *) tp, i = 0; (i < 0x40);  i += 4) {
		if ((i % 0x10) == 0)
			printk("\n%08x: ", (int) (tp + i));
		printk("%08x ", (int) *sp++);
	}
	printk("\n");

	printk("\nKERNEL STACK:");
	tp = ((unsigned char *) fp) - 0x40;
	for (sp = (unsigned long *) tp, i = 0; (i < 0xc0); i += 4) {
		if ((i % 0x10) == 0)
			printk("\n%08x: ", (int) (tp + i));
		printk("%08x ", (int) *sp++);
	}
	printk("\n");
	if (STACK_MAGIC != *(unsigned long *)current->kernel_stack_page)
                printk("(Possibly corrupted stack page??)\n");
	printk("\n");

	printk("\nUSER STACK:");
	tp = (unsigned char *) (fp->sp - 0x10);
	for (sp = (unsigned long *) tp, i = 0; (i < 0x80); i += 4) {
		if ((i % 0x10) == 0)
			printk("\n%08x: ", (int) (tp + i));
		printk("%08x ", (int) *sp++);
	}
	printk("\n\n");
}
#endif /* 0 */

void init_irq_proc(void)
{
	phx_warn("TODO");
}
