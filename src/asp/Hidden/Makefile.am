# __BEGIN_LICENSE__
#  Copyright (c) 2009-2013, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__


########################################################################
# sources
########################################################################

bin_PROGRAMS = 
bin_SCRIPTS =
libexec_SCRIPTS =
libexec_PROGRAMS = # Auxiliary C++ executables

if MAKE_APP_CSV_FILTER
  csv_filter_LDADD    = $(APP_CSV_FILTER_LIBS)
  csv_filter_SOURCES  = csv_filter.cc
  bin_PROGRAMS         += csv_filter
endif

if MAKE_APP_S2P_PPRC
  s2p_pprc_LDADD    = $(APP_S2P_PPRC_LIBS)
  s2p_pprc_SOURCES  = s2p_pprc.cc
  bin_PROGRAMS     += s2p_pprc
endif

if MAKE_APP_S2P_FLTR
  s2p_fltr_LDADD    = $(APP_S2P_FLTR_LIBS)
  s2p_fltr_SOURCES  = s2p_fltr.cc
  bin_PROGRAMS     += s2p_fltr
endif

# Scripts
##############################################################################

bin_SCRIPTS +=
all_scripts = $(bin_SCRIPTS)
all_scripts += $(libexec_SCRIPTS)
CLEANFILES = $(all_scripts)

# $@ = target file name
# $< = name of first prerequisite
# %  = matches any non-empty substring
$(all_scripts): %: %.in
	sed 's/\[@\]ASP_VERSION\[@\]/$(VERSION)/g' < $(srcdir)/$< > $@
	chmod +x $@


##############################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

SUBDIRS = .

includedir = $(prefix)/include/asp/Hidden

include $(top_srcdir)/config/rules.mak
