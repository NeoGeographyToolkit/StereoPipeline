# __BEGIN_LICENSE__
#  Copyright (c) 2006-2013, United States Government as represented by the
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

if MAKE_MODULE_GUI

# 'make clean' should get rid of any QT meta-object compiled files.
CLEANFILES = *.moc.cc

include_HEADERS =               \
  GuiUtilities.h                \
  MainWidget.h                  \
  MainWindow.h 

BUILT_SOURCES =                 \
  GuiUtilities.moc.cc           \
  MainWidget.moc.cc             \
  MainWindow.moc.cc

libaspGUI_la_SOURCES =          \
  $(BUILT_SOURCES)              \
  GuiUtilities.cc               \
  MainWidget.cc                 \
  MainWindow.cc

libaspGUI_la_LIBADD = @MODULE_GUI_LIBS@

lib_LTLIBRARIES = libaspGUI.la

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

includedir = $(prefix)/include/asp/GUI

include $(top_srcdir)/config/rules.mak
