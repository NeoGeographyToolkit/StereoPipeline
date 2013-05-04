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


TESTS = #TestPEDR TestNURBS

#nodist_TestNURBS_SOURCES = TestNURBS.cxx
#nodist_TestPEDR_SOURCES  = TestPEDR.cxx

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@ -I$(top_srcdir)/thirdparty/cxxtest
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_VW_LIBS@

check_PROGRAMS = $(TESTS)

EXTRA_DIST = $(TESTS:%=%.h)

CLEANFILES = $(TESTS:%=%.cxx)

include $(top_srcdir)/config/rules.mak
