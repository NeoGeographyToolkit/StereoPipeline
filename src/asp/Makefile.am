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


#########################################
# sources
#########################################

core_subdirs = Core
core_headers = Core.h

camera_subdirs = Camera

sessions_subdirs = Sessions

isisio_subdirs = IsisIO
isisio_headers = IsisIO.h

spiceio_subdirs = SpiceIO
spiceio_headers = SpiceIO.h

gui_subdirs = GUI
python_subdirs = Python
tools_subdirs = Tools
wv_correct_subdirs = WVCorrect
icebridge_subdirs = IceBridge
hidden_subdirs = Hidden

include_HEADERS = $(core_headers) $(spiceio_headers)            \
	  $(isisio_headers) asp_config.h

SUBDIRS = $(core_subdirs) $(spiceio_subdirs) $(isisio_subdirs) \
	  $(camera_subdirs) $(sessions_subdirs) $(gui_subdirs) $(python_subdirs) \
	  $(tools_subdirs) $(wv_correct_subdirs) $(icebridge_subdirs) $(hidden_subdirs)
CLEANFILES = asp_config.h.pre.in~

#########################################
# general
#########################################

includedir = $(prefix)/include/asp

include $(top_srcdir)/config/rules.mak
