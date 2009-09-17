
#########################################
# sources
#########################################

core_subdirs = Core
core_headers = Core.h

sessions_subdirs = Sessions
sessions_headers = Sessions.h

tools_subdirs = Tools

include_HEADERS = $(core_headers) $(sessions_headers) asp_config.h

SUBDIRS = $(core_subdirs) $(sessions_subdirs) $(tools_subdirs)

CLEANFILES = asp_config.h.pre.in~

#########################################
# general
#########################################

includedir = $(prefix)/include/asp

include $(top_srcdir)/config/rules.mak