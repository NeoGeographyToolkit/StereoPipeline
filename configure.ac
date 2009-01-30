# __BEGIN_LICENSE__
#
# Copyright (C) 2006 United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration
# (NASA).  All Rights Reserved.
#
# Copyright 2006 Carnegie Mellon University. All rights reserved.
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

##################################################
# A few starting bits
##################################################

dnl TODO BEFORE RELEASE: update package version
AC_INIT([NASA Ames Stereo Pipeline], [2.1], [asp@nx.arc.nasa.gov], [StereoPipeline])
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE([nostdinc])
AX_CONFIG_HEADER_PREFIX([src/asp_config.h], [ASP_])

dnl TODO BEFORE RELEASE: update libtool version
dnl if abi changes in any way:
dnl     current++
dnl     revision=0
dnl     if new abi is backward-compatible:
dnl         age += 1
dnl     else:
dnl         age = 0
dnl else:
dnl     revision++
dnl (see http://sources.redhat.com/autobook/autobook/autobook_91.html)
dnl
dnl LIBTOOL_VERSION=current:revision:age
LIBTOOL_VERSION=1:0:0
AC_SUBST(LIBTOOL_VERSION)

# This should be first, so everything else can use it
AX_ARG_ENABLE(verbose, no, [none], [verbose configuration output])

# load the config.options
if test -f config.options; then
  if test "yes" = "$ENABLE_VERBOSE" ; then
    AC_MSG_NOTICE([sourcing config.options])
    cat config.options
  fi
  source config.options
fi



##################################################
# Find the necessary programs
##################################################

# Make sure these are set so AC_PROG_CC doesn't initialize them with
# its defaults, since we take care of setting debug and optimization
# levels ourselves.
CFLAGS="$CFLAGS"
CXXFLAGS="$CXXFLAGS"

# Find the tools
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AX_PROG_AR
AC_PROG_LIBTOOL
AC_PROG_MAKE_SET
AC_LANG(C++)
AX_COMMON_OPTIONS



##################################################
# package checks
##################################################

AX_PKG_PTHREADS

AX_PKG_BOOST
AX_PKG_BOOST_CHECK_VERSION([ASP_])

AC_MSG_NOTICE([Stereo will be built using Boost version $BOOST_VERSION])
AX_PKG_BOOST_LIB(PROGRAM_OPTIONS, [-lboost_program_options], [boost/program_options.hpp])

AX_PKG_ONE_OF(BOOST_FILESYSTEM,
    BOOST_FILESYSTEM_PRE_1_35,  [AX_PKG_BOOST_LIB(FILESYSTEM_PRE_1_35,  [-lboost_filesystem], [boost/filesystem/path.hpp])],
    BOOST_FILESYSTEM_POST_1_35, [AX_PKG_BOOST_LIB(FILESYSTEM_POST_1_35, [-lboost_filesystem -lboost_system], [boost/filesystem/path.hpp])])

AX_PKG_BOOST_LIB(THREAD, [-lboost_thread], [boost/thread/thread.hpp])

# This provides an easy way for users to override the threads options
# if they are replacing Boost threads with something else for their platform.
AX_PKG(THREADS, [BOOST_THREAD], [], [])

AX_PKG_ONE_OF(LAPACK,
  APPLE_LAPACK,
    [AX_PKG_APPLE(LAPACK, [], [-framework vecLib])],
  CLAPACK,
    [AX_PKG(CLAPACK, [], [-lclapack -lblas -lf2c], [])],
  SLAPACK,
    [AX_PKG(SLAPACK, [], [-llapack -lblas], [])],
  FLAPACK,
    [AX_PKG(FLAPACK, [], [-llapack -lblas -lgfortran], [])],
  STANDALONE_LAPACK_BLAS,
    [AX_PKG(STANDALONE_BLAS, [], [-lblas], [])
     AX_PKG(STANDALONE_LAPACK, [], [-llapack], [])
     AX_GROUP_PKG(STANDALONE_LAPACK_AND_BLAS, [STANDALONE_LAPACK STANDALONE_BLAS])],
  STANDALONE_FLAPACK_FBLAS,
    [AX_PKG(STANDALONE_F2C, [], [-lf2c], [])
     AX_PKG(STANDALONE_FBLAS, [STANDALONE_F2C], [-lblas], [])
     AX_PKG(STANDALONE_FLAPACK, [STANDALONE_F2C], [-llapack], [])
     AX_GROUP_PKG(STANDALONE_FLAPACK_FBLAS, [STANDALONE_FLAPACK STANDALONE_FBLAS STANDALONE_F2C])])

AX_PKG(VW,                [BOOST], [-lvw],            [vw/Image/ImageView.h vw/Math/LinearAlgebra.h])
AX_PKG(VW_STEREO,         [BOOST], [-lvwStereo],      [vw/Stereo/OptimizedCorrelator.h])
AX_PKG(VW_CAMERA,         [BOOST], [-lvwCamera],      [vw/Camera/CAHVModel.h])
AX_PKG(VW_CARTOGRAPHY,    [BOOST], [-lvwCartography], [vw/Cartography/GeoReference.h])
AX_PKG(VW_INTEREST_POINT, [BOOST], [-lvwInterestPoint])

AX_PKG(CUDA,              [],      [-lcuda -lcudart], [cuda.h])

AX_PKG_ONE_OF(GL,
  APPLE_GL,
    [AX_PKG_APPLE([GL], [], [-framework OpenGL -framework GLUT -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib])],
  LINUX_GL,
    [AX_PKG(X11, [], [-lXext -lX11 -lXmu -lXi -lXt], [])
     AX_PKG(LINUX_GL, [X11], [-lGL -lGLU -lglut], [GL/gl.h GL/glu.h GL/glut.h], [])])

AX_PKG(OPENSCENEGRAPH, [GL], [-losg -losgDB -lOpenThreads -losgViewer -losgText -losgGA -losgFX -losgTerrain -losgUtil], [osg/ref_ptr])

AX_PKG_ONE_OF(MBA,
    MBA11, [AX_PKG(MBA11, [BOOST], [-lMBA-1.1],         [MBA.h])],
    MBA10, [AX_PKG(MBA10, [BOOST], [-lMBA_1.0_gpl_opt], [MBA.h])])

AX_PKG(SPICE, [], [-lcspice], [SpiceUsr.h SpiceZfc.h])

# Qt with AutoTroll.
AT_WITH_QT([+gui +sql +opengl], [], [], [], [HAVE_PKG_QT_INCLUDE=no])

if test x"$HAVE_PKG_QT_INCLUDE" != x"no"; then
    AT_REQUIRE_QT_VERSION([4.0], [], [HAVE_PKG_QT_INCLUDE=no])
fi

# We seed QT_INCLUDE's cppflags with the ones qmake figured out
PKG_QT_INCLUDE_CPPFLAGS="$PKG_QT_INCLUDE_CPPFLAGS $QT_CPPFLAGS"
AX_PKG(QT_INCLUDE, [], [], [QVector])

AX_PKG_ONE_OF(QT,
    APPLE_QT, [AX_PKG_APPLE(QT, [GL QT_INCLUDE], [$QT_LIBS])],
    LINUX_QT,
     [AX_PKG(QT_LIBS,    [],              [$QT_LIBS], [])
      AX_PKG(QT_OPENGL,  [GL QT_INCLUDE], [],         [QtOpenGL])
      AX_PKG(QT_GUI,     [QT_INCLUDE],    [],         [QLabel])
      AX_PKG(QT_SQL,     [QT_INCLUDE],    [],         [QtSql])
      AX_GROUP_PKG(LINUX_QT, [QT_INCLUDE QT_GUI QT_OPENGL QT_SQL QT_LIBS])])

AX_PKG_ONE_OF(QWT,
    APPLE_QWT, [AX_PKG_APPLE(QWT, [QT_INCLUDE], [-lqwt.5])],
    PLAIN_QWT, [AX_PKG(PLAIN_QWT, [QT_INCLUDE], [-lqwt])],
    SUFFX_QWT, [AX_PKG(SUFFX_QWT, [QT_INCLUDE], [-lqwt-qt4])])

AX_PKG_ONE_OF(GSL,
    GSL_HASBLAS,  [AX_PKG(GSL_HASBLAS,  [LAPACK], [-lgsl])],
    GSL_NEEDBLAS, [AX_PKG(GSL_NEEDBLAS, [LAPACK], [-lgsl -lgslcblas])])

if test x"$host_vendor" = "xapple"; then
    AX_PKG(GEOS,    [],             [-lgeos-3.0.0], [])
    AX_PKG(SUPERLU, [],             [-lsuperlu_3.0], [])
    AX_PKG(XERCESC, [],             [-lxerces-c.28], [])
else
    AX_PKG(GEOS,    [],             [-lgeos])
    AX_PKG(SUPERLU, [],             [-lsuperlu])
    AX_PKG(XERCESC, [],             [-lxerces-c])
fi

dnl We need to link ISIS3RDPARTY against LAPACK because, on some systems,
dnl the ISIS compilation does not resolve certain BLAS symbols that its
dnl dependent 3rd party libraries depend on (for whatever retarded reason).
dnl Thus, we link against LAPACK here because it defines these missing
dnl BLAS symbols in the event that we need them.
AX_PKG(ISIS3RDPARTY, [SUPERLU LAPACK GSL QWT GEOS SPICE XERCESC], [])
AX_PKG(ISIS, [QT ISIS3RDPARTY], [-lisis3], [SpecialPixel.h])

# These are here (instead of inside the PKG macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(HAVE_PKG_SPICE, [test "$HAVE_PKG_SPICE" = "yes"])
AM_CONDITIONAL(HAVE_PKG_ISIS, [test "$HAVE_PKG_ISIS" = "yes"])

##################################################
# module definitions
##################################################

AX_GROUP_PKG(BOOST_COMMON, [BOOST BOOST_PROGRAM_OPTIONS])
AX_GROUP_PKG(BOOST_ALL,    [BOOST_COMMON BOOST_FILESYSTEM])
AX_GROUP_PKG(VW_ALL,       [VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT])

AX_APP(STEREO,         [src], yes, [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(STEREOGUI,      [src], no,  [BOOST_ALL VW_ALL LAPACK MBA QT],  [ISIS SPICE])
AX_APP(ORTHOPROJECT,   [src], yes, [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(BUNDLEADJUST,   [src], no,  [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(ORBITVIZ,       [src], no,  [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(DISPARITYDEBUG, [src], yes, [BOOST_COMMON VW VW_STEREO])
AX_APP(POINT2MESH,     [src], yes, [BOOST_COMMON VW OPENSCENEGRAPH])
AX_APP(POINT2MESH2,    [src], yes, [BOOST_COMMON VW OPENSCENEGRAPH])
AX_APP(POINT2DEM,      [src], yes, [BOOST_COMMON VW VW_CARTOGRAPHY])
AX_APP(NURBS,          [src], no,  [BOOST_COMMON VW MBA])
AX_APP(CTXIMAGE,       [src], no,  [BOOST_COMMON VW VW_CARTOGRAPHY])
AX_APP(RMAX2CAHVOR,    [src], no,  [BOOST_COMMON VW VW_CAMERA BOOST_FILESYSTEM])
AX_APP(RMAXADJUST,     [src], no,  [BOOST_ALL VW VW_CAMERA VW_STEREO VW_INTEREST_POINT LAPACK])
AX_APP(BUNDLEVIS,      [src], no,  [BOOST_COMMON OPENSCENEGRAPH VW VW_CAMERA])
AX_APP(ISISADJUST,     [src], yes, [BOOST_ALL VW_ALL ISIS])
AX_APP(RESULTS,        [src], yes, [BOOST_ALL VW_ALL ISIS])
AX_APP(CUDATEST,       [src], no,  [VW CUDA])

# These are here (instead of inside the APP macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(MAKE_APP_STEREO, [test "$MAKE_APP_STEREO" = "yes"])
AM_CONDITIONAL(MAKE_APP_STEREOGUI, [test "$MAKE_APP_STEREOGUI" = "yes"])
AM_CONDITIONAL(MAKE_APP_ORTHOPROJECT, [test "$MAKE_APP_ORTHOPROJECT" = "yes"])
AM_CONDITIONAL(MAKE_APP_BUNDLEADJUST, [test "$MAKE_APP_BUNDLEADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_ORBITVIZ, [test "$MAKE_APP_ORBITVIZ" = "yes"])
AM_CONDITIONAL(MAKE_APP_DISPARITYDEBUG, [test "$MAKE_APP_DISPARITYDEBUG" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2MESH, [test "$MAKE_APP_POINT2MESH" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2MESH2, [test "$MAKE_APP_POINT2MESH2" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2DEM, [test "$MAKE_APP_POINT2DEM" = "yes"])
AM_CONDITIONAL(MAKE_APP_NURBS, [test "$MAKE_APP_NURBS" = "yes"])
AM_CONDITIONAL(MAKE_APP_CTXIMAGE, [test "$MAKE_APP_CTXIMAGE" = "yes"])
AM_CONDITIONAL(MAKE_APP_RMAX2CAHVOR, [test "$MAKE_APP_RMAX2CAHVOR" = "yes"])
AM_CONDITIONAL(MAKE_APP_RMAXADJUST, [test "$MAKE_APP_RMAXADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_BUNDLEVIS, [test "$MAKE_APP_BUNDLEVIS" = "yes"])
AM_CONDITIONAL(MAKE_APP_RESULTS, [test "$MAKE_APP_RESULTS" = "yes"])
AM_CONDITIONAL(MAKE_APP_CUDATEST, [test "$MAKE_APP_CUDATEST" = "yes"])



##################################################
# final processing
##################################################

ASP_CPPFLAGS="-I\${top_srcdir}/src $OTHER_CPPFLAGS"
ASP_LDFLAGS="$OTHER_LDFLAGS"

if test "yes" = "$ENABLE_VERBOSE" ; then
  AC_MSG_NOTICE([using CPPFLAGS=$CPPFLAGS])
  AC_MSG_NOTICE([using ASP_CPPFLAGS=$ASP_CPPFLAGS])
  AC_MSG_NOTICE([using CFLAGS=$CFLAGS])
  AC_MSG_NOTICE([using CXXFLAGS=$CXXFLAGS])
  AC_MSG_NOTICE([using LDFLAGS=$LDFLAGS])
  AC_MSG_NOTICE([using ASP_LDFLAGS=$ASP_LDFLAGS])
  AC_MSG_NOTICE([using ASP_ENABLE_EXCEPTIONS=$ENABLE_EXCEPTIONS])
fi

AC_SUBST(ASP_CPPFLAGS)
AC_SUBST(ASP_LDFLAGS)



##################################################
# makefiles to process
##################################################

AC_CONFIG_FILES([                       \
  Makefile                              \
  src/Makefile                          \
  src/tests/Makefile                    \
])

AC_OUTPUT
