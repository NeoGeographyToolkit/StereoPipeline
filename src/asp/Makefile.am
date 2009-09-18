
#########################################
# sources
#########################################

core_subdirs = Core
core_headers = Core.h

sessions_subdirs = Sessions
sessions_headers = Sessions.h

isisio_subdirs = IsisIO
isisio_headers = IsisIO.h

spiceio_subdirs = SpiceIO
spiceio_headers = SpiceIO.h

tools_subdirs = Tools

include_HEADERS = $(core_headers) $(spiceio_headers)            \
	  $(isisio_headers) $(sessions_headers) asp_config.h

SUBDIRS = $(core_subdirs) $(spiceio_subdirs) $(isisio_subdirs) \
	  $(sessions_subdirs) $(tools_subdirs)

CLEANFILES = asp_config.h.pre.in~

#########################################
# general
#########################################

includedir = $(prefix)/include/asp

include $(top_srcdir)/config/rules.mak