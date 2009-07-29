/*
 *  linux/arch/or32/kernel/time.c
 *
 *  or32 version
 *
 *  Copied/hacked from:
 *
 *  linux/arch/m68knommu/kernel/time.c
 *
 *  Copyright (C) 1991, 1992, 1995  Linus Torvalds
 *
 * This file contains the m68k-specific time handling details.
 * Most of the stuff is located in the machine specific files.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/profile.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/interrupt.h>

#include <asm/machdep.h>
#include <asm/segment.h>
#include <asm/io.h>
#include <asm/or32-hf.h>

#define TICK_SIZE (tick_nsec / 1000)

extern unsigned long wall_jiffies;


static inline int set_rtc_mmss(unsigned long nowtime)
{
  if (mach_set_clock_mmss)
    return mach_set_clock_mmss (nowtime);
  return -1;
}

/* last time the RTC clock got updated */
static long last_rtc_update;

/* 
 * timer_interrupt() needs to keep up the real-time clock,
 * as well as call the "do_timer()" routine every clocktick
 */
void do_timer_interrupt(struct pt_regs *regs)
{
       /* may need to kick the hardware timer */
       if (mach_tick)
	 mach_tick();

       do_timer(1); /*RGD*/

#ifndef CONFIG_SMP
       update_process_times(user_mode(regs));
#endif
       /*profile_tick(CPU_PROFILING); RGD may be broken*/

       /*
	* If we have an externally synchronized Linux clock, then update
	* CMOS clock accordingly every ~11 minutes. Set_rtc_mmss() has to be
	* called as close as possible to 500 ms before the new second starts.
	*/
#if 0 /*RGD*/
       if (time_state != TIME_BAD && xtime.tv_sec > last_rtc_update + 660 &&
	   (xtime.tv_nsec / 1000) > 500000 - ((unsigned)TICK_SIZE >> 1) &&
	   (xtime.tv_nsec / 1000) < 500000 + ((unsigned)TICK_SIZE >> 1)) {
	       if (set_rtc_mmss(xtime.tv_sec) == 0)
		       last_rtc_update = xtime.tv_sec;
	       else
		       last_rtc_update = xtime.tv_sec - 600; /* do it again in 60 s */
       }
#endif /*RGD*/
      	
       /*RGD from FRV*/
        /*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. Set_rtc_mmss() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 */
	if (ntp_synced() &&
	    xtime.tv_sec > last_rtc_update + 660 &&
	    (xtime.tv_nsec / 1000) >= 500000 - ((unsigned) TICK_SIZE) / 2 &&
	    (xtime.tv_nsec / 1000) <= 500000 + ((unsigned) TICK_SIZE) / 2
	    ) {
		if (set_rtc_mmss(xtime.tv_sec) == 0)
			last_rtc_update = xtime.tv_sec;
		else
			last_rtc_update = xtime.tv_sec - 600; /* do it again in 60 s */
	}
	/*RGD from FRV*/
}


/*
 * This get called at every clock tick...
 *
 */
irqreturn_t timer_interrupt(struct pt_regs * regs)
{
  	check_stack(regs, __FILE__, __FUNCTION__, __LINE__);

        /*
         * Here we are in the timer irq handler. We just have irqs locally
         * disabled but we don't know if the timer_bh is running on the other
         * CPU. We need to avoid to SMP race with it. NOTE: we don' t need
         * the irq version of write_lock because as just said we have irq
         * locally disabled. -arca
         */
        write_seqlock(&xtime_lock);
        do_timer_interrupt(regs);
        write_sequnlock(&xtime_lock);
 
        return IRQ_HANDLED;
}

/*
 * This version of gettimeofday has near microsecond resolution.
 *
 * Note: Division is quite slow on OR32 and do_gettimeofday is called
 *       rather often. Maybe we should do some kind of approximation here
 *       (a naive approximation would be to divide by 1024).  
 */
void do_gettimeofday(struct timeval *tv)
{
	unsigned long flags;
	signed long usec, sec;
	local_irq_save(flags);
	local_irq_disable();
	usec = mach_gettimeoffset ? mach_gettimeoffset() : 0;
#if 0 /*RGD*/
	{
		unsigned long lost = jiffies - wall_jiffies;
		if (lost)
		  usec += lost * (1000000 / HZ);
	}
#endif /*RGD*/	
	/*
	 * If time_adjust is negative then NTP is slowing the clock
	 * so make sure not to go into next possible interval.
	 * Better to lose some accuracy than have time go backwards..
	 */
	if (unlikely(time_adjust < 0) && usec > tickadj)
	  usec = tickadj;
	
	sec = xtime.tv_sec;
	usec += xtime.tv_nsec / 1000;
	local_irq_restore(flags);
	
	while (usec >= 1000000) {
		usec -= 1000000;
		sec++;
	}
	
	tv->tv_sec = sec;
	tv->tv_usec = usec;
}

EXPORT_SYMBOL(do_gettimeofday);

int do_settimeofday(struct timespec *tv)
{
	time_t wtm_sec, sec = tv->tv_sec;
	long wtm_nsec, nsec = tv->tv_nsec;

	if ((unsigned long)tv->tv_nsec >= NSEC_PER_SEC)
		return -EINVAL;

	write_seqlock_irq(&xtime_lock);
	/*
	 * This is revolting. We need to set the xtime.tv_usec
	 * correctly. However, the value in this location is
	 * is value at the last tick.
	 * Discover what correction gettimeofday
	 * would have done, and then undo it!
	 */
	if (mach_gettimeoffset)
		nsec -= (mach_gettimeoffset() * 1000);

	wtm_sec  = wall_to_monotonic.tv_sec + (xtime.tv_sec - sec);
	wtm_nsec = wall_to_monotonic.tv_nsec + (xtime.tv_nsec - nsec);

	set_normalized_timespec(&xtime, sec, nsec);
	set_normalized_timespec(&wall_to_monotonic, wtm_sec, wtm_nsec);

	time_adjust = 0;		/* stop active adjtime() */
	time_status |= STA_UNSYNC;
	time_maxerror = NTP_PHASE_LIMIT;
	time_esterror = NTP_PHASE_LIMIT;
	write_sequnlock_irq(&xtime_lock);
	clock_was_set();
	return 0;
}

EXPORT_SYMBOL(do_settimeofday);



/*
 * Scheduler clock - returns current time in nanosec units.
 */
unsigned long long sched_clock(void)
{
	return (unsigned long long)jiffies * (1000000000 / HZ);
}


void __init time_init(void)
{
	unsigned int year, mon, day, hour, min, sec;

	extern void arch_gettod(int *year, int *mon, int *day, int *hour,
				int *min, int *sec);

	arch_gettod (&year, &mon, &day, &hour, &min, &sec);

	if ((year += 1900) < 1970)
		year += 100;
	xtime.tv_sec = mktime(year, mon, day, hour, min, sec);
	xtime.tv_nsec = 0;

	
	if (mach_sched_init)
		mach_sched_init();
}

