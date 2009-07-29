/*
 *  linux/arch/or32/board/config.c
 *
 *  or32 version
 *    author(s): Simon Srot (srot@opencores.org)
 *
 *  For more information about OpenRISC processors, licensing and
 *  design services you may contact Beyond Semiconductor at
 *  sales@bsemi.com or visit website http://www.bsemi.com.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 * Based on m68knommu/platform/xx/config.c
 */

#include <stdarg.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/serial_8250.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/mmc.h>
#include <asm/serial.h>

// extern void register_console(void (*proc)(const char *));

/* Tick timer period */
unsigned long tick_period = SYS_TICK_PER;

void BSP_sched_init(void)
{
	/* Set counter period, enable timer and interrupt */
	mtspr(SPR_TTMR, SPR_TTMR_IE | SPR_TTMR_RT | (SYS_TICK_PER & SPR_TTMR_PERIOD));
}

void BSP_tick(void)
{
	mtspr(SPR_TTMR, SPR_TTMR_IE | SPR_TTMR_RT | (SYS_TICK_PER & SPR_TTMR_PERIOD));
}

unsigned long BSP_gettimeoffset(void)
{
	unsigned long count, result;

	count = mfspr(SPR_TTCR);
	result = count / CONFIG_OR32_SYS_CLK;
#if 0
	printk("gettimeofday offset :: cnt %d, sys_tick_per %d, result %d\n",
	       count, CONFIG_OR32_SYS_CLK, result);
#endif
	return(result);

}

void BSP_gettod (int *yearp, int *monp, int *dayp,
		   int *hourp, int *minp, int *secp)
{
}

int BSP_hwclk(int op, struct hwclk_time *t)
{
	if (!op) {
		/* read */
	} else {
		/* write */
	}
	return 0;
}

int BSP_set_clock_mmss (unsigned long nowtime)
{
#if 0
	short real_seconds = nowtime % 60, real_minutes = (nowtime / 60) % 60;

	tod->second1 = real_seconds / 10;
	tod->second2 = real_seconds % 10;
	tod->minute1 = real_minutes / 10;
	tod->minute2 = real_minutes % 10;
#endif
	return 0;
}

void BSP_reset (void)
{
        local_irq_disable();
}

void config_BSP(char *command, int len)
{
	mach_sched_init      = BSP_sched_init;
	mach_tick            = BSP_tick;
	mach_gettimeoffset   = BSP_gettimeoffset;
	mach_gettod          = BSP_gettod;
	mach_hwclk           = NULL;
	mach_set_clock_mmss  = NULL;
	mach_mksound         = NULL;
	mach_reset           = BSP_reset;
	mach_debug_init      = NULL;
}

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.mapbase	= 0x90000000,
		.irq		= 2,
		.uartclk	= SYS_CLK, /*BASE_BAUD,*/
		.regshift	= 0,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF,
	},
	{ },
};

static struct platform_device serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

static struct resource ocores_mmc_resources[] = {
	[0] = {
		.start	= 0xa0000000,
		.end	= 0xa0000100,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name   = "cmd_irq",
		.start	= 3,
		.end	= 3,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name   = "cmd_err_irq",
		.start	= 4,
		.end	= 4,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.name   = "dat_irq",
		.start	= 5,
		.end	= 5,
		.flags	= IORESOURCE_IRQ,
	},
	

};

static struct platform_device ocores_mmc_device = {
	.name		= "mmc-ocores",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ocores_mmc_resources),
	.resource	= ocores_mmc_resources,
};

static struct ocores_platform_data ocores_platform_data = {
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
};

static void __init ocores_set_mmc_info(struct ocores_platform_data *info)
{
	ocores_mmc_device.dev.platform_data = info;
}

static int __init config_or32(void)
{
	platform_device_register(&serial_device);
	platform_device_register(&ocores_mmc_device);
	ocores_set_mmc_info(&ocores_platform_data);

	return 0;
}

arch_initcall(config_or32);
