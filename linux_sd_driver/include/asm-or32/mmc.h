#ifndef ASMOR32_MMC_H
#define ASMOR32_MMC_H

#include <linux/mmc/host.h>
#include <linux/interrupt.h>

struct device;
struct mmc_host;

struct ocores_platform_data {
	unsigned int ocr_mask; /* available voltages */
};

#endif
