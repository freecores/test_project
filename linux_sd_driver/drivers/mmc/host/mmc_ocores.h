/* -*- linux-c -*-
 *
 * OpenCores MMC Controller driver
 *
 * Copyright (C) 2009 ORSoC, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#define MMCOC_REG_BASE MMC_BASE_ADD
#define MMCOC_MAX_BLOCK_SIZE 512

#define FL_SENT_COMMAND	(1 << 0)
#define FL_SENT_STOP	(1 << 1)

#define WORD_0 0x00
#define WORD_1 0x40
#define WORD_2 0x80
#define WORD_3 0xC0

#define SD_ARG 0x00
#define SD_COMMAND 0x04
#define SD_STATUS 0x08
#define SD_RESP1 0x0c
#define SD_CTRL 0x1c
#define SD_BLOCK 0x20
#define SD_POWER 0x24
#define SD_SOFTWARE_RST 0x28
#define SD_TIMEOUT 0x2c
#define SD_NORMAL_INT_STATUS 0x30
#define SD_ERROR_INT_STATUS 0x34
#define SD_NORMAL_INT_STATUS_ENABLE 0x38
#define SD_ERROR_INT_STATUS_ENABLE 0x3c

#define SD_NOMAL_INT_SIGNAL_ENABLE  0x38
#define SD_ERROR_INT_SIGNAL_ENABLE  0x3c
#define SD_CAPABILITY  0x48
#define SD_CLOCK_DIVIDER  0x4c
#define BD_STATUS 0x50
#define SD_BD_ISR 0x54
#define SD_BD_ISER 0x58
#define BD_RX 0x60
#define BD_TX 0x80

//Normal Interupt signal enable register
#define ECC 0x1 //Interupt on CommandComplete 
#define EEI 0x8000 //Interupt on CommandError

//Data Interupt

#define TRE 0x20
#define CMDE 0x10
#define FIFOE 0x04
#define MRC 0x02
#define TRS 0x01

#define SD_ENABLE 0
#define SD_DISABLE 1

/* Commands */
#define CMD2   0x200
#define CMD3   0x300
#define CMD7   0x700
#define CMD8   0x800
#define CMD9   0x900
#define CMD16  0x1000
#define CMD17  0x1100
#define CMD32  0x2000
#define CMD33  0x2100
#define CMD38  0x2600
#define ACMD41 0x2900
#define ACMD6  0x600
#define CMD55  0x3700

/* CMD ARG */
/* CMD8 */
#define VHS  0x100 //2.7-3.6V
#define CHECK_PATTERN 0xAA
/* ACMD41 */
#define BUSY 0x80000000
#define HCS 0x40000000
#define VOLTAGE_MASK 0xFFFFFF

/* CMD7 */
#define READY_FOR_DATA 0x100
#define CARD_STATUS_STB  0x600

//BIT MASK INTERNAL REGISTERS

/* Command setting register*/
#define CICE    0x10
#define CRCE    0x08
#define MMCOC_RSP_48  0x2
#define MMCOC_RSP_136 0x1
#define MMCOC_RSP_NONE 0x0




/* Normal interupt status */
#define CC 0x1 //Command Complete
#define EI 0x8000 //Error interrupt bit set

/* Error interupt status */
#define CTE 0x1 //Command timeout
#define CCRC 0x2 //Command CRC Error
#define CIE  0x8 //Command Index Error

//
#define CID_MID_MASK 0x7F8000
#define CID_OID_MASK 0x7FFF
#define CID_B1 0x7F800000
#define CID_B2 0x7F8000
#define CID_B3 0x7F80
#define CID_B4 0x7F

#define RCA_RCA_MASK 0xFFFF0000

#define ocores_mci_write(host, reg, val)	__raw_writel((val), (host)->base + (reg))
