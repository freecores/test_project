/*
 *  linux/arch/or32/kernel/process.c
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
 * This file handles the architecture-dependent parts of process handling..
 * Based on m86k.
 */

#define __KERNEL_SYSCALLS__
#include <stdarg.h>

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/user.h>
#include <linux/elfcore.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/init_task.h>
#include <linux/mqueue.h>
#include <linux/fs.h>

#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <asm/spr_defs.h>
#include <asm/or32-hf.h>


#include <linux/smp.h>

/*
 * Initial task structure. Make this a per-architecture thing,
 * because different architectures tend to have different
 * alignment requirements and potentially different initial
 * setup.
 */

static struct fs_struct init_fs = INIT_FS;
static struct files_struct init_files = INIT_FILES;
static struct signal_struct init_signals = INIT_SIGNALS(init_signals);
static struct sighand_struct init_sighand = INIT_SIGHAND(init_sighand);
struct mm_struct init_mm = INIT_MM(init_mm);

EXPORT_SYMBOL(init_mm);

/*
 * Initial thread structure.
 *
 * We need to make sure that this is 8192-byte aligned due to the
 * way process stacks are handled. This is done by having a special
 * "init_task" linker map entry..
 */
union thread_union init_thread_union 
	__attribute__((__section__(".data.init_task"))) =
		{ INIT_THREAD_INFO(init_task) };

/*
 * Pointer to Current thread info structure.
 *
 * Used at user space -> kernel transitions. 
 */
struct thread_info *current_thread_info_set[NR_CPUS] = {&init_thread_info, };

/*
 * Initial task structure.
 *
 * All other task structs will be allocated on slabs in fork.c
 */
struct task_struct init_task = INIT_TASK(init_task);

EXPORT_SYMBOL(init_task);

/*
 * The hlt_counter, disable_hlt and enable_hlt is just here as a hook if
 * there would ever be a halt sequence (for power save when idle) with
 * some largish delay when halting or resuming *and* a driver that can't
 * afford that delay.  The hlt_counter would then be checked before
 * executing the halt sequence, and the driver marks the unhaltable
 * region by enable_hlt/disable_hlt.
 */

static int hlt_counter=0;

void disable_hlt(void)
{
	hlt_counter++;
}

EXPORT_SYMBOL(disable_hlt);

void enable_hlt(void)
{
	hlt_counter--;
}

EXPORT_SYMBOL(enable_hlt);
 
void machine_restart(void)
{
	printk("*** MACHINE RESTART ***\n");
	__asm__("l.nop 1");
}

EXPORT_SYMBOL(machine_restart);

/*
 * Similar to machine_power_off, but don't shut off power.  Add code
 * here to freeze the system for e.g. post-mortem debug purpose when
 * possible.  This halt has nothing to do with the idle halt.
 */

void machine_halt(void)
{
	printk("*** MACHINE HALT ***\n");
	__asm__("l.nop 1");
}

EXPORT_SYMBOL(machine_halt);

/* If or when software power-off is implemented, add code here.  */

void machine_power_off(void)
{
	printk("*** MACHINE POWER OFF ***\n");
	__asm__("l.nop 1");
}

EXPORT_SYMBOL(machine_power_off);

void (*pm_power_off)(void) = machine_power_off;
EXPORT_SYMBOL(pm_power_off);

/*
 * When a process does an "exec", machine state like FPU and debug
 * registers need to be reset.  This is a hook function for that.
 * Currently we don't have any such state to reset, so this is empty.
 */

void flush_thread(void)
{
}

void show_regs(struct pt_regs *regs)
{
	extern void show_registers(struct pt_regs *regs);

	/* __PHX__ cleanup this mess */
	show_registers(regs);
}

asmlinkage int sys_fork(int r3, int r4, int r5, int r6, int r7, struct pt_regs *regs)
{
	return do_fork(SIGCHLD, regs->sp, regs, 0, NULL, NULL);
}

asmlinkage int sys_clone(int r3, int r4, int r5, int r6, int r7, struct pt_regs *regs)
{
	unsigned long clone_flags = (unsigned long)r3;
	return do_fork(clone_flags, regs->sp, regs, 0, NULL, NULL);
}

asmlinkage int sys_vfork(int r3, int r4, int r5, int r6, int r7, struct pt_regs *regs)
{
	return do_fork(CLONE_VFORK | CLONE_VM | SIGCHLD, regs->gprs[1], regs, 0, NULL, NULL);
}

unsigned long thread_saved_pc(struct task_struct *t)
{
	return (unsigned long)user_regs(t->stack)->pc;
}

void release_thread(struct task_struct *dead_task)
{
}

int copy_thread(int nr, unsigned long clone_flags, unsigned long usp,
		unsigned long unused, struct task_struct *p, struct pt_regs *regs)
{
        struct pt_regs *childregs, *kregs;
        extern void ret_from_fork(void);
	unsigned long sp = (unsigned long)p->stack + THREAD_SIZE;
        unsigned long childframe;
        struct thread_info *tmp;
	p->set_child_tid = p->clear_child_tid = NULL;

        /* Copy registers */
	sp -= sizeof(struct pt_regs);
        childregs = (struct pt_regs *) sp;

        *childregs = *regs;
        
	if ((childregs->sr & SPR_SR_SM) == 1) {
                /* for kernel thread, set `current' and stackptr in new task */
                childregs->sp = sp + sizeof(struct pt_regs);
                childregs->gprs[8] = (unsigned long)p->stack;
                /* __PHX__ :: i think this thread.regs is not needed */
                p->thread.regs = NULL;  /* no user register state */
        } else
                p->thread.regs = childregs;

        childregs->gprs[9] = 0;  /* Result from fork() */
//	sp -= STACK_FRAME_OVERHEAD;
	childframe = sp;

        /*
         * The way this works is that at some point in the future
         * some task will call _switch to switch to the new task.
         * That will pop off the stack frame created below and start
         * the new task running at ret_from_fork.  The new task will
         * do some house keeping and then return from the fork or clone
         * system call, using the stack frame created above.
         */
	sp -= sizeof(struct pt_regs);
        kregs = (struct pt_regs *) sp;
       
//      sp -= STACK_FRAME_OVERHEAD;
        tmp = (struct thread_info*)p->stack;
        tmp->ksp = sp;
	
	kregs->sr = regs->sr | SPR_SR_SM;
	kregs->sp = sp + sizeof(struct pt_regs);
	kregs->gprs[1] = (unsigned long)p;              /* for schedule_tail */
	kregs->gprs[8] = (unsigned long)p->stack; /* current           */
        kregs->pc = (unsigned long)ret_from_fork;
        p->thread.last_syscall = -1;
        return 0;
}


/*
 * Set up a thread for executing a new program
 */
void start_thread(struct pt_regs *regs, unsigned long pc, unsigned long sp)
{
	phx_warn("NIP: %lx, SP: %lx", pc, sp);

	set_fs(USER_DS);
	memset(regs->gprs, 0, sizeof(regs->gprs));

	regs->pc = pc;
	regs->sr = regs->sr & ~ SPR_SR_SM;
	regs->sp = sp;
}

/* Fill in the fpu structure for a core dump.  */
int dump_fpu(struct pt_regs *regs, elf_fpregset_t *fpu)
{
	phx_warn("FPU :: TODO");
	return 0;
} 

void _switch_to(struct task_struct *old, 
		struct task_struct *new,
		struct task_struct **last)
{
	extern struct thread_info *_switch(struct thread_info *old_ti,
					   struct thread_info *new_ti);
	struct thread_info *new_ti, *old_ti;
	long flags;

	local_irq_save(flags);
	check_stack(NULL, __FILE__, __FUNCTION__, __LINE__);

	/* current_set is an array of saved current pointers
	 * (one for each cpu). we need them at user->kernel transition,
	 * while we save them at kernel->user transition
	 */
	new_ti = new->stack;
	old_ti = old->stack;

	current_thread_info_set[smp_processor_id()] = new_ti;
	*last = (_switch(old_ti, new_ti))->task;

	check_stack(NULL, __FILE__, __FUNCTION__, __LINE__);
	local_irq_restore(flags);
} 

/*
 * fill in the user structure for a core dump..
 */
void dump_thread(struct pt_regs *regs, struct user *dump)
{
	phx_warn("TODO");
}

/*
 * sys_execve() executes a new program.
 */
asmlinkage int sys_execve(char *name, char **argv, char **envp,
			  int r6, int r7, struct pt_regs *regs)
{
	int error;
	char * filename;

	filename = getname(name);
	error = PTR_ERR(filename);

	if (IS_ERR(filename))
	  goto out;

	error = do_execve(filename, argv, envp, regs);
	putname(filename);
 
out:	
	return error;
}

unsigned long get_wchan(struct task_struct *p)
{
	phx_warn("TODO");

	return 0;
}

int kernel_execve(const char *filename, char *const argv[], char *const 
envp[])
{
  register long __res asm("r11") = __NR_execve;
  register long __a asm("r3") = (long)(filename);
  register long __b asm("r4") = (long)(argv);
  register long __c asm("r5") = (long)(envp);
  __asm__ volatile ("l.sys 1" : "=r" (__res)
           : "r" (__res), "r" (__a), "r" (__b), "r" (__c));
  __asm__ volatile("l.nop");
  return __res;
}
