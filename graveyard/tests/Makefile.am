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
