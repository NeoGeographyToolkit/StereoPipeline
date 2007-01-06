# __BEGIN_LICENSE__
#
# Copyright (C) 2006 United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration
# (NASA).  All Rights Reserved.
# 
# This software is distributed under the NASA Open Source Agreement
# (NOSA), version 1.3.  The NOSA has been approved by the Open Source
# Initiative.  See the file COPYING at the top of the distribution
# directory tree for the complete NOSA document.
# 
# THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
# KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
# LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
# SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
# A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
# THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
# DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
#
# __END_LICENSE__

########################################################################
# sources
########################################################################

bin_PROGRAMS = stereo rmax_stereo nurbs

generic_stereo_sources = file_lib.cc mask.cc nff_terrain.cc	\
	SurfaceNURBS.cc SIFT.cc DEM.cc

stereo_SOURCES = $(generic_stereo_sources) stereo.cc 

rmax_stereo_SOURCES = $(generic_stereo_sources) rmax_stereo.cc 

if HAVE_PKG_GDAL
bin_PROGRAMS += hrsc_stereo moc_stereo ctx_stereo

moc_stereo_SOURCES = $(generic_stereo_sources) MOC/Ephemeris.cc		\
	  MOLAReader.cc Spice.cc TabulatedDataReader.cc MOC/Metadata.cc	\
	  MOC/MOLA.cc moc_stereo.cc

hrsc_stereo_SOURCES = $(generic_stereo_sources) HRSC/HRSC.cc Spice.cc	\
	   TabulatedDataReader.cc hrsc_stereo.cc

ctx_stereo_SOURCES = $(generic_stereo_sources) Spice.cc							\
	TabulatedDataReader.cc ctx_stereo.cc MRO/DiskImageResourceDDD.cc	\
	MRO/CTXEphemeris.cc MRO/CTXMetadata.cc
endif

nurbs_SOURCES = SurfaceNURBS.cc nurbs.cc

SUBDIRS = .

AM_CPPFLAGS = @CPPFLAGS@ @MODULE_STEREOPIPELINE_CPPFLAGS@
AM_LDFLAGS = @LDFLAGS@ @MODULE_STEREOPIPELINE_LDFLAGS@

##############################################################################
#                             Tests	                                         #
##############################################################################

include $(top_srcdir)/config/rules.mak




