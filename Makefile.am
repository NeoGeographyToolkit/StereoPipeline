# __BEGIN_LICENSE__
#  Copyright (c) 2009-2020, United States Government as represented by the
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

ACLOCAL_AMFLAGS = -I m4 -I thirdparty/m4

SUBDIRS = src data

EXTRA_DIST = \
	config/rules.mak \
	config/relicense \
	data \
	docs \
	stereo.default.example

dist_doc_DATA = AUTHORS.rst COPYING INSTALLGUIDE.rst NEWS.rst README.rst THIRDPARTYLICENSES
exampledir = $(docdir)/examples
dist_example_DATA = stereo.default.example

########################################################################
# special top-level rules
########################################################################

#install-data-hook:
#	$(INSTALL) src/config.h $(prefix)/include/

dist-hook:
	rm -rf `find $(distdir) -name '.svn'`
if DIST_LICENSE
	cp $(DIST_LICENSE) $(distdir)/COPYING
endif
if DIST_LICENSE_SUMMARY
	find $(distdir) -type f | config/relicense $(DIST_LICENSE_SUMMARY) -
endif
if DIST_CONFIG_OPTIONS_DEFAULT
	cp $(DIST_CONFIG_OPTIONS_DEFAULT) $(distdir)/config/config.options.default
endif

doxygen:
	cd src && doxygen

########################################################################
# general
########################################################################

include $(top_srcdir)/config/rules.mak
