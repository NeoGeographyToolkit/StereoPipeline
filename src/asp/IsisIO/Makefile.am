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

if MAKE_MODULE_ISISIO

include_HEADERS = BaseEquation.h Equation.h PolyEquation.h            \
		  RPNEquation.h DiskImageResourceIsis.h               \
		  IsisCameraModel.h            \
		  IsisInterface.h IsisInterfaceFrame.h                \
		  IsisInterfaceLineScan.h IsisInterfaceMapFrame.h     \
		  IsisInterfaceMapLineScan.h

libaspIsisIO_la_SOURCES = DiskImageResourceIsis.cc Equation.cc        \
		  PolyEquation.cc RPNEquation.cc IsisInterface.cc     \
		  IsisInterfaceFrame.cc IsisInterfaceLineScan.cc      \
		  IsisInterfaceMapFrame.cc IsisInterfaceMapLineScan.cc

libaspIsisIO_la_LIBADD = @MODULE_ISISIO_LIBS@

lib_LTLIBRARIES = libaspIsisIO.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = . tests

includedir = $(prefix)/include/asp/IsisIO

include $(top_srcdir)/config/rules.mak