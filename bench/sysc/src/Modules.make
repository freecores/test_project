# Make file for SystemC modules not associated directly with Verilator
#
# Copyright (C) 2009 Embecosm Limited
#
# Contributor Jeremy Bennett <jeremy.bennett@embecosm.com>
# 
# This file is part of the cycle accurate model of the OpenRISC 1000 based
# system-on-chip, ORPSoC, built using Verilator.
# 
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 3 of the License, or (at your option)
# any later version.
# 
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>. */

# The C/C++ parts of this program are commented throughout in a fashion
# suitable for processing with Doxygen.

# Tools and flags
ARFLAGS = rcs
#CXXFLAGS += $(OPT_FAST) $(OPT_SLOW) $(OPT) $(PROF_FLAGS)
CXX ?= g++
PROF_OPTS ?= -fbranch-probabilities -fvpt -funroll-loops -fpeel-loops -ftracer
OPT_ALL ?= $(OPT_SLOW) $(OPT_FAST) $(OPT)

# Sub-directories
SYSC_INC_DIR = ../include
INCDIRS   = -I$(SYSTEMC)/include -I$(SYSC_INC_DIR)

# Local objects
OBJS = Or1200MonitorSC.o \
       ResetSC.o \
	UartSC.o
LIB  = libmodules.a

# -----------------------------------------------------------------------------
# Rule to make dependency files
%.d: %.cpp
	@set -e; rm -f $@; \
		$(CXX) -MM $(CPPFLAGS) $(INCDIRS) $< > $@.$$$$; \
		sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$

# Rule to make object files
%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(INCDIRS) $(CXXFLAGS) -c  $<


# -----------------------------------------------------------------------------
# Make the library
$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $+


# -----------------------------------------------------------------------------
# Tidy up
.PHONY: clean
clean:
	$(RM) *.d *.d.*
	$(RM) *.o *.gcno *.gcda
	$(RM) $(LIB) 


# -----------------------------------------------------------------------------
# More modest tidy up for branch profiling
.PHONY: prof-clean
prof-clean:
	$(RM) *.o
	$(RM) $(LIB)


# -----------------------------------------------------------------------------
# Include the dependency files
include $(OBJS:.o=.d)
