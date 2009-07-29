/*
 *  linux/arch/or32/kernel/idle.c
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
 * Idle daemon for or32.  Idle daemon will handle any action
 * that needs to be taken when the system becomes idle.
 *
 * Based on:
 * arch/ppc/kernel/idle.c
 * Written by Cort Dougan (cort@cs.nmt.edu)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>

#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <asm/mmu.h>
#include <asm/cache.h>

void (*powersave)(void) = NULL;

void cpu_idle(void)
{
	int spr_sr;

	for (;;) {
		
// __PHX__ TODO: make an config option or something
#if 1
		if (!(SPR_SR_TEE & (spr_sr = mfspr(SPR_SR)))) {
			printk("idled: tick timer disabled, enabling...\n");
			printk("idled: SR 0x%lx, ESR 0x%lx, EPCR 0x%lx, EEAR 0x%lx\n",
			       mfspr(SPR_SR), mfspr(SPR_ESR_BASE), 
			       mfspr(SPR_EPCR_BASE), mfspr(SPR_EEAR_BASE));
			
			mtspr(SPR_SR, spr_sr | SPR_SR_TEE);
			// __PHX__ TODO: add trace
		}
#endif

		if (!need_resched()) {
			if (powersave != NULL) {
				powersave();
			}
		}
#if 0
		if (need_resched())
			schedule();

#else
	schedule();
#endif

	}
}
