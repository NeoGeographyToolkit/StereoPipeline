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


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_CORE

if HAVE_PKG_VW_BUNDLEADJUSTMENT

ba_headers = BundleAdjustUtils.h
ba_sources = BundleAdjustUtils.cc

endif

include_HEADERS = StereoSettings.h        \
                  MedianFilter.h OrthoRasterizer.h           \
                  SoftwareRenderer.h $(ba_headers) Macros.h    \
                  Common.h Common.tcc ThreadedEdgeMask.h                   \
                  InterestPointMatching.h FileUtils.h \
                  DemDisparity.h LocalHomography.h AffineEpipolar.h        \
                  Point2Grid.h PointUtils.h PhotometricOutlier.h


libaspCore_la_SOURCES = Common.cc MedianFilter.cc   \
                  SoftwareRenderer.cc StereoSettings.cc $(ba_sources)    \
                  InterestPointMatching.cc DemDisparity.cc               \
                  LocalHomography.cc AffineEpipolar.cc Point2Grid.cc     \
                  OrthoRasterizer.cc PointUtils.cc PhotometricOutlier.cc \
                  FileUtils.cc

libaspCore_la_LIBADD = @MODULE_CORE_LIBS@

lib_LTLIBRARIES = libaspCore.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = . tests

includedir = $(prefix)/include/asp/Core

include $(top_srcdir)/config/rules.mak
