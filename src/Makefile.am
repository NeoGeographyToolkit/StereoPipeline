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

bin_PROGRAMS = stereo disparitydebug point2mesh point2dem # ctximage # ctxblend # stereo rmax2cahvor # ctx2kml ctxblend # nurbs # stereodefault

moc_sources = MOC/StereoSessionMOC.cc MOC/Ephemeris.cc MOLAReader.cc	\
	  	 MOC/Metadata.cc MOC/MOLA.cc

hrsc_sources = HRSC/StereoSessionHRSC.cc HRSC/HRSC.cc

apollo_sources = apollo/StereoSessionApolloMetric.cc

generic_stereo_sources = file_lib.cc StereoSession.cc										\
		 StereoSessionKeypoint.cc StereoSessionPinhole.cc SIFT.cc Spice.cc	\
		 TabulatedDataReader.cc $(moc_sources) $(hrsc_sources)							\
		 $(apollo_sources)

stereo_SOURCES = $(generic_stereo_sources) stereo.cc 

bin_PROGRAMS += # ctx_stereo

ctx_stereo_SOURCES = $(generic_stereo_sources) Spice.cc							\
	TabulatedDataReader.cc ctx_stereo.cc MRO/DiskImageResourceDDD.cc	\
	MRO/CTXEphemeris.cc MRO/CTXMetadata.cc

# nurbs_SOURCES = nurbs.cc

ctx2kml_SOURCES = ctx2kml.cc MRO/DiskImageResourceDDD.cc
ctxblend_SOURCES = ctxblend.cc MRO/DiskImageResourceDDD.cc
ctximage_SOURCES = ctximage.cc MRO/DiskImageResourceDDD.cc

disparitydebug_SOURCES = disparitydebug.cc

point2mesh_SOURCES = point2mesh.cc nff_terrain.cc
point2dem_SOURCES = point2dem.cc DEM.cc

rmax2cahvor_SOURCES = rmax2cahvor.cc

stereodefault_SOURCES = stereodefault.cc file_lib.cc

SUBDIRS = .

AM_CPPFLAGS = @CPPFLAGS@ @MODULE_STEREOPIPELINE_CPPFLAGS@
AM_LDFLAGS = @LDFLAGS@ @MODULE_STEREOPIPELINE_LDFLAGS@

##############################################################################
#                             Tests	                                         #
##############################################################################

include $(top_srcdir)/config/rules.mak




