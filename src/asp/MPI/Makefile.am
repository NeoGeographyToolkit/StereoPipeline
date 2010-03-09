# __BEGIN_LICENSE__
# Copyright (C) 2006-2009 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_MPI

include_HEADERS =

#libaspMPI_la_SOURCES =

#libaspMPI_la_LIBADD = @MODULE_MPI_LIBS@

#lib_LTLIBRARIES = libaspMPI.la

endif

if ENABLE_EXCEPTIONS

MPI_LOCAL_LIBS = @MODULE_MPI_LIBS@

isis_mpi_adjust_SOURCES = isis_mpi_adjust.cc
isis_mpi_adjust_LDADD   = @PKG_VW_CAMERA_LIBS@ $(MPI_LOCAL_LIBS)

bin_PROGRAMS = isis_mpi_adjust

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = .

includedir = $(prefix)/include/asp/MPI

include $(top_srcdir)/config/rules.mak