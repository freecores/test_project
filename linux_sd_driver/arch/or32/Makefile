# BK Id: %F% %I% %G% %U% %#%
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies. Remember to do have actions
# for "archclean" and "archdep" for cleaning up and making dependencies for
# this architecture
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1994 by Linus Torvalds

# Changes for or32 by Matjaz Breskvar
# Based on:
# arch/i386/Makefile

LDFLAGS         :=
OBJCOPYFLAGS    := -O binary -R .note -R .comment -S
LDFLAGS_vmlinux :=
LIBGCC 		 = $(shell $(CC) $(CFLAGS) -print-file-name=libgcc.a)

CFLAGS		+= -pipe
CFLAGS		+= -fsigned-char -pipe -ffixed-r10 \
#		   -ffixed-r16 -ffixed-r17 -ffixed-r18 -ffixed-r19 \
#		   -ffixed-r20 -ffixed-r21 -ffixed-r22 -ffixed-r23 \
#		   -ffixed-r24 -ffixed-r25 -ffixed-r26 -ffixed-r27 \
#		   -ffixed-r28 -ffixed-r29 -ffixed-r30 -ffixed-r31 \
                   -Wno-uninitialized
AFLAGS		+= 

head-y 		:= arch/or32/kernel/head.o


core-y		+= arch/or32/lib/ \
		   arch/or32/kernel/ \
		   arch/or32/mm/ \
		   arch/or32/drivers/ \
		   arch/or32/board/ \
		   arch/or32/support/
libs-y		+= $(LIBGCC)

all: vmlinux

#prepare: include/asm-$(ARCH)/offsets.h
CLEAN_FILES +=  include/asm-$(ARCH)/offsets.h
 
#arch/$(ARCH)/kernel/asm-offsets.s: include/asm include/linux/version.h \
#				   include/config/MARKER

#include/asm-$(ARCH)/offsets.h: arch/$(ARCH)/kernel/asm-offsets.s
#	$(call filechk,gen-asm-offsets)
