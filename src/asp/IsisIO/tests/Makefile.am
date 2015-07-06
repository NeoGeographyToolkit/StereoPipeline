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

if MAKE_MODULE_ISISIO

TestIsisCameraModel_SOURCES       = TestIsisCameraModel.cxx
TestEphemerisEquations_SOURCES    = TestEphemerisEquations.cxx

TESTS = TestIsisCameraModel TestEphemerisEquations

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_ISISIO_LIBS@

check_PROGRAMS = $(TESTS)
#CLEANFILES = log.txt

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am
