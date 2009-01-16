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
AM_INIT_AUTOMAKE
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

# We set up the include paths ourselves, so we ask autoconf to get out of the way
DEFAULT_INCLUDES=
AC_SUBST(DEFAULT_INCLUDES)



##################################################
# Compilation options
##################################################

AX_ARG_ENABLE(exceptions,   yes, [am-yes cpp-bool], [enable the C++ exception mechanism])
AX_ARG_ENABLE(debug,         no, [none],            [generate debugging symbols])
AX_ARG_ENABLE(optimize,       3, [none],            [compiler optimization level])
AX_ARG_ENABLE(lib64,       auto, [none],            [force /lib64 instead of /lib])

##################################################
# Handle options
##################################################

# For 64-bit machines, we'll generally want to autodetect if lib64 exists,
# and use it. Sometimes the user will want to force use of lib64 or lib
# instead of the autodetection. Thus we have this command line option.
if test x"$USE_LIB64" = "xauto"; then
  if test -d "/lib64" -o -d "/usr/lib64"; then
    USE_LIB64="yes"
  else
    USE_LIB64="no"
  fi
fi

# Sometimes we have /foo/lib64 and /foo/lib confusion on 64-bit machines,
# so we'll use possibly both if one doesn't appear for a certain
# library path.
if test x"$USE_LIB64" = "xyes"; then
  AX_LIBDIR="lib64"
  AX_OTHER_LIBDIR="lib"
else
  AX_LIBDIR="lib"
  AX_OTHER_LIBDIR="lib64"
fi

# These are good if they're supported
AX_TRY_CPPFLAGS([-Wl,-no-undefined], [LDFLAGS="$LDFLAGS -Wl,-no-undefined"])
AX_TRY_CPPFLAGS([-Wl,-as-needed],    [LDFLAGS="$LDFLAGS -Wl,-as-needed"])

# Debugging
case "$ENABLE_DEBUG" in
    yes|1) AX_CFLAGS="$AX_CFLAGS -g -DDEBUG" ;;
    2)     AX_CFLAGS="$AX_CFLAGS -g -DDEBUG -D_GLIBCXX_DEBUG" ;;
    no)    AX_CFLAGS="$AX_CFLAGS -DNDEBUG" ;;
    *)     AC_MSG_ERROR([Unknown debug option: "$ENABLE_DEBUG"]) ;;
esac

# Optimization
case "$ENABLE_OPTIMIZE" in
    yes)     AX_CFLAGS="$AX_CFLAGS -O3" ;;
    3|2|1)   AX_CFLAGS="$AX_CFLAGS -O$ENABLE_OPTIMIZE" ;;
    coreduo) AX_CFLAGS="$AX_CFLAGS -O4 -march=prescott -mtune=prescott -funroll-loops -msse -msse2 -msse3 -mfpmath=sse" ;;
    sse3)    AX_CFLAGS="$AX_CFLAGS -O4 -funroll-loops -msse -msse2 -msse3 -mfpmath=sse" ;;
    no|0)    AC_MSG_WARN([*** The Vision Workbench may not work properly with optimization disabled! ***])
             AX_CFLAGS="$AX_CFLAGS -O0" ;;
    *)       AC_MSG_ERROR([Unknown optimize option: "$ENABLE_OPTIMIZE"]) ;;
esac

CFLAGS="$CFLAGS $AX_CFLAGS"
CXXFLAGS="$CXXFLAGS $AX_CFLAGS"

# These need to be here because automake-1.6 is dumb and does not properly
# process AM_CONDITIONALs unless the first argument is a simple string.
AM_CONDITIONAL(ENABLE_EXCEPTIONS, [test x"$ENABLE_EXCEPTIONS" = x"yes"])


##################################################
# installation options
##################################################

if test ${prefix} = NONE; then
  if test ! -z ${PREFIX} ; then
    prefix="${PREFIX}"
    AC_MSG_NOTICE([using installation prefix ${prefix}])
  fi
fi



##################################################
# distribution options
##################################################

AX_ARG_WITH(dist-license,                       [], [mk am-set], [special distribution license file to be included as the COPYING file in the distribution])
AX_ARG_WITH(dist-license-summary,               [], [mk am-set], [special distribution license summary file to be included in the headers of source files in the distribution])
AX_ARG_WITH(dist-config-options-default,        [], [mk am-set], [special distribution config.options.default file])



##################################################
# handle distribution options
##################################################

if test ! -z "$DIST_LICENSE" ; then
  AC_MSG_NOTICE([using distribution license file $DIST_LICENSE])
fi

if test ! -z "$DIST_LICENSE_SUMMARY" ; then
  AC_MSG_NOTICE([using distribution license summary file $DIST_LICENSE_SUMMARY])
fi

if test ! -z "$DIST_CONFIG_OPTIONS_DEFAULT" ; then
  if test "yes" = "$DIST_CONFIG_OPTIONS_DEFAULT" ; then
    DIST_CONFIG_OPTIONS_DEFAULT="config/config.options.default"
  fi
  AC_MSG_NOTICE([using distribution config.options.default file: $DIST_CONFIG_OPTIONS_DEFAULT])
fi

# These need to be here because automake-1.6 is dumb and does not properly
# process AM_CONDITIONALs unless the first argument is a simple string.
AM_CONDITIONAL(DIST_LICENSE, [test ! -z "$DIST_LICENSE"])
AM_CONDITIONAL(DIST_LICENSE_SUMMARY, [test ! -z "$DIST_LICENSE_SUMMARY"])
AM_CONDITIONAL(DIST_CONFIG_OPTIONS_DEFAULT, [test ! -z "$DIST_CONFIG_OPTIONS_DEFAULT"])



##################################################
# package checks
##################################################

PKG_PATHS_FROM_OPT="$PKG_PATHS"
AX_ARG_WITH(pkg-paths, [], [none], [additional search path(s) for packages])
PKG_PATHS="$PKG_PATHS $PKG_PATHS_FROM_OPT ${HOME} ${HOME}/local /sw /opt /opt/local /usr/local /usr/X11R6 /usr"

if test "yes" = "$ENABLE_VERBOSE"; then
  AC_MSG_NOTICE([using PKG_PATHS=$PKG_PATHS])
fi

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

AX_PKG(OPENSCENEGRAPH, [], [-losg -losgDB -lOpenThreads -losgViewer -losgText -losgGA -losgFX -losgTerrain -losgUtil], [osg/ref_ptr])

AX_PKG_ONE_OF(MBA,
    MBA10, [AX_PKG(MBA10, [BOOST], [-lMBA_1.0_gpl_opt], [MBA.h])],
    MBA11, [AX_PKG(MBA11, [BOOST], [-lMBA-1.1],         [MBA.h])])


AX_PKG(SPICE, [], [-lcspice], [SpiceUsr.h SpiceZfc.h])

# Qt with AutoTroll.
AT_WITH_QT([+gui +sql +opengl], [], [], [], [HAVE_PKG_QT_INCLUDE=no])

if test x"$HAVE_PKG_QT_INCLUDE" != x"no"; then
    AT_REQUIRE_QT_VERSION([4.0], [], [HAVE_PKG_QT_INCLUDE=no])
fi

# We seed QT_INCLUDE's cppflags with the ones qmake figured out
ADD_QT_INCLUDE_CPPFLAGS="$ADD_QT_INCLUDE_CPPFLAGS $QT_CPPFLAGS"
AX_PKG(QT_INCLUDE, [], [], [QVector])

AX_PKG_ONE_OF(QT,
    APPLE_QT, [AX_PKG_APPLE(QT, [QT_INCLUDE], [$QT_LIBS])],
    LINUX_QT,
     [AX_PKG(QT_LIBS,              [], [$QT_LIBS], [])
      AX_PKG(QT_OPENGL,  [QT_INCLUDE], [],         [QtOpenGL])
      AX_PKG(QT_GUI,     [QT_INCLUDE], [],         [QLabel])
      AX_PKG(QT_SQL,     [QT_INCLUDE], [],         [QtSql])
      AX_GROUP_PKG(LINUX_QT, [QT_INCLUDE QT_GUI QT_OPENGL QT_SQL QT_LIBS])])

AX_PKG_ONE_OF(MBA,
  MBA10, [AX_PKG(MBA10, [BOOST], [-lMBA_1.0_gpl_opt], [MBA.h])],
  MBA11, [AX_PKG(MBA11, [BOOST], [-lMBA-1.1],         [MBA.h])])

AX_PKG_ONE_OF(QWT,
    APPLE_QWT, [AX_PKG_APPLE(QWT, [QT_INCLUDE], [-lqwt.5])],
    PLAIN_QWT, [AX_PKG(PLAIN_QWT, [QT_INCLUDE], [-lqwt])],
    SUFFX_QWT, [AX_PKG(SUFFX_QWT, [QT_INCLUDE], [-lqwt-qt4])])

AX_PKG_ONE_OF(GSL,
    GSL_DEFINED,   [AX_PKG(GSL_DEFINED,   [LAPACK], [-lgsl -lgslcblas])],
    GSL_UNDEFINED, [AX_PKG(GSL_UNDEFINED, [LAPACK], [-Wl,-no-as-needed,-lgsl,-lgslcblas,-as-needed])])

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
AM_CONDITIONAL(HAVE_PKG_ISIS, [test "$HAVE_PKG_ISIS" = "yes"])

##################################################
# module definitions
##################################################

AX_APP(STEREO, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT])
AX_APP(STEREOGUI, [src], no, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY])
AX_APP(ORTHOPROJECT, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT])
AX_APP(BUNDLEADJUST, [src], no, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT])
AX_APP(ORBITVIZ, [src], no, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT])
AX_APP(DISPARITYDEBUG, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS VW VW_STEREO])
AX_APP(POINT2MESH, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS VW OPENSCENEGRAPH])
AX_APP(POINT2MESH2, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS OPENSCENEGRAPH VW])
AX_APP(POINT2DEM, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS VW VW_CARTOGRAPHY])
AX_APP(NURBS, [src], no, [BOOST BOOST_PROGRAM_OPTIONS VW MBA])
AX_APP(CTXIMAGE, [src], no, [BOOST BOOST_PROGRAM_OPTIONS VW VW_CARTOGRAPHY])
AX_APP(RMAX2CAHVOR, [src], no, [BOOST VW VW_CAMERA BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM])
AX_APP(RMAXADJUST, [src], no, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_STEREO VW_INTEREST_POINT LAPACK])
AX_APP(BUNDLEVIS, [src], no, [BOOST BOOST_PROGRAM_OPTIONS OPENSCENEGRAPH VW VW_CAMERA])
AX_APP(ISISADJUST, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_STEREO VW_CARTOGRAPHY VW_INTEREST_POINT LAPACK ISIS])
AX_APP(RESULTS, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_STEREO VW_CARTOGRAPHY VW_INTEREST_POINT LAPACK ISIS])
AX_APP(CUDATEST, [src], no, [VW CUDA])

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
# library options
##################################################

AX_ARG_WITH(num-threads, [4], [cpp-int], [set the default number of processing threads for multi-threaded operations])
AC_MSG_NOTICE([VW will use $NUM_THREADS THREADS by default])

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
