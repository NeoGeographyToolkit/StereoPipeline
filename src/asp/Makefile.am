# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


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

mpi_subdirs = MPI

photometry_subdirs = PhotometryTK

controlnet_subdirs = ControlNetTK

include_HEADERS = $(core_headers) $(spiceio_headers)            \
	  $(isisio_headers) $(sessions_headers) asp_config.h

SUBDIRS = $(core_subdirs) $(spiceio_subdirs) $(isisio_subdirs) \
	  $(sessions_subdirs) $(tools_subdirs) $(mpi_subdirs)  \
	  $(photometry_subdirs) $(controlnet_subdirs)

CLEANFILES = asp_config.h.pre.in~

#########################################
# general
#########################################

includedir = $(prefix)/include/asp

include $(top_srcdir)/config/rules.mak