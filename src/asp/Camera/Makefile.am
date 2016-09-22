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

if MAKE_MODULE_CAMERA

include_HEADERS = RPCModel.h RPCModelGen.h RPCStereoModel.h                   \
		  LinescanDGModel.h  LinescanDGModel.tcc                      \
                  LinescanSpotModel.h LinescanASTERModel.h                    \
                  AdjustedLinescanDGModel.h RPC_XML.h                          \
                  SPOT_XML.h ASTER_XML.h XMLBase.h 

libaspCamera_la_SOURCES = RPCModel.cc XMLBase.cc RPC_XML.cc                    \
                          SPOT_XML.cc ASTER_XML.cc                            \
                          RPCStereoModel.cc RPCModelGen.cc                    \
                          LinescanSpotModel.cc LinescanASTERModel.cc

libaspCamera_la_LIBADD = @MODULE_CAMERA_LIBS@

lib_LTLIBRARIES = libaspCamera.la

SUBDIRS = . tests



endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

includedir = $(prefix)/include/asp/Camera

include $(top_srcdir)/config/rules.mak
