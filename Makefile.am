# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

ACLOCAL_AMFLAGS = -I m4 -I thirdparty/m4

SUBDIRS = src data

EXTRA_DIST = \
	INSTALLGUIDE \
	config/rules.mak \
	config/relicense \
	data \
	docs \
	stereo.default.example

dist_doc_DATA = AUTHORS COPYING README NEWS THIRDPARTYLICENSES
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
