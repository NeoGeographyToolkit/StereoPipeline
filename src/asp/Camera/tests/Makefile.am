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

if MAKE_MODULE_CAMERA

TestRPCStereoModel_SOURCES  = TestRPCStereoModel.cxx
TestDGCameraModel_SOURCES  = TestDGCameraModel.cxx
TestSpotCameraModel_SOURCES  = TestSpotCameraModel.cxx

TESTS = TestDGCameraModel TestRPCStereoModel TestSpotCameraModel

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_CAMERA_LIBS@

check_PROGRAMS = $(TESTS)
EXTRA_DIST = wv_test1.xml wv_test2.xml wv_mvp_1.xml wv_mvp_2.xml

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am
