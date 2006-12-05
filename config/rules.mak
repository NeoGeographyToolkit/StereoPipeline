# __BEGIN_LICENSE__
#
# Copyright (C) 2006 United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration
# (NASA).  All Rights Reserved.
# 
# This software is distributed under the NASA Open Source Agreement
# (NOSA), version 1.3.  The NOSA has been approved by the Open Source
# Initiative.  See the file COPYING at the top of the distribution
# directory tree for the complete NOSA document.
# 
# THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
# KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
# LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
# SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
# A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
# THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
# DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
#
# __END_LICENSE__

########################################################################
# tests (using cxxtest)
########################################################################

SUFFIXES = .cxx

CXXTEST_DIR :=   $(top_srcdir)/thirdparty/cxxtest
CXXTEST_GEN :=   $(CXXTEST_DIR)/cxxtestgen.pl
CXXTEST_ARGS :=  --error-printer

TESTS_SRCS := $(TESTS_H:.h=.cxx)
TESTS := $(TESTS_SRCS:.cxx=.test)

# We do some magic here to build inside the tests directory so that we
# can avoid having to put the current directory in the include path.
%.cxx: %.h 
	cd tests ; ../$(CXXTEST_GEN) $(CXXTEST_ARGS) -o $(patsubst tests/%,%,$@) $(patsubst tests/%,%,$<)

CXXCOMPILE = $(CXX) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) \
	$(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CXXFLAGS) $(CXXFLAGS)
DEPDIR = .deps

# This is what I used to do here, and we probably ought to be explicitly 
# falling back to it in a non-GNU environment....
#$(CXX) $(CC_OPTIMIZE_DEBUG) $(CFLAGS) $(CPPFLAGS) $(AM_CPPFLAGS) -I$(CXXTEST_DIR) $(INCLUDES) -o $@ $< $(TESTS_FLAGS) $(LDFLAGS) $(AM_LDFLAGS)

%.test: %.cxx 
	if test ! -d "$(DEPDIR)/tests" ; then mkdir -p "$(DEPDIR)/tests" ; fi
	if $(CXXCOMPILE) -I$(CXXTEST_DIR) -MT $@ -MD -MP -MF "$(DEPDIR)/$*.Tpo" -o $@ $<; \
	then mv -f "$(DEPDIR)/$*.Tpo" "$(DEPDIR)/$*.Po"; else rm -f "$(DEPDIR)/$*.Tpo"; exit 1; fi

ifneq  ($(strip $(wildcard .deps/tests/*.Po)),)
include $(wildcard .deps/tests/*.Po)
@ENDIF@

########################################################################
# extra hooks
########################################################################

clean-local:
	rm -f $(TESTS_SRCS)

distclean-local:
	rm -rf *~ $(TESTS) $(DEPDIR)
