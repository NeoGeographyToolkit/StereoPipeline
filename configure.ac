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

AC_INIT([NASA Ames Stereo Pipeline], [2.1], [asp@nx.arc.nasa.gov], [StereoPipeline])
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE
AX_CONFIG_HEADER_PREFIX([src/asp_config.h], [ASP_])

LIBTOOL_CURRENT=1
LIBTOOL_REVISION=0
LIBTOOL_AGE=0
LIBTOOL_VERSION=$LIBTOOL_CURRENT:$LIBTOOL_REVISION:$LIBTOOL_AGE
AC_SUBST(LIBTOOL_VERSION)


##################################################
#	configuration options
##################################################

# This should be first, so everything else can use it
AC_ARG_ENABLE(verbose,
  AC_HELP_STRING([--enable-verbose], [verbose configuration output @<:@no@:>@]),
  [ ENABLE_VERBOSE=$enableval ],
  [ if test -z $"ENABLE_VERBOSE"; then ENABLE_VERBOSE=no; fi ]
)

if test -f config.options; then
  if test "yes" = "$ENABLE_VERBOSE" ; then
    AC_MSG_NOTICE([sourcing config.options])
    cat config.options
  fi
  source config.options
fi

AC_ARG_WITH(paths,
  AC_HELP_STRING([--with-paths=PATHS], [additional search path(s) for packages]),
  [ PKG_PATHS="$withval $PKG_PATHS" ]
)
PKG_PATHS="${PKG_PATHS} ${HOME} ${HOME}/local /sw /opt /opt/local /usr/local /usr/X11R6 /usr"


##################################################
#	compilation options
##################################################

AC_ARG_ENABLE(debug,
  AC_HELP_STRING([--enable-debug], [generate debugging symbols @<:@no@:>@]),
  [ ENABLE_DEBUG=$enableval ],
  [ if test -z "$ENABLE_DEBUG"; then ENABLE_DEBUG=no; fi ]
)

if test "yes" = "$ENABLE_DEBUG"; then
   AX_CFLAGS="$AX_CFLAGS -g -DDEBUG"
else
   AX_CFLAGS="$AX_CFLAGS -DNDEBUG"
fi

# For 64-bit machines, we'll generally want to autodetect if lib64 exists,
# and use it. Sometimes the user will want to force use of lib64 or lib
# instead of the autodetection. Thus we have this command line option.
AC_ARG_ENABLE(lib64,
  AC_HELP_STRING([--enable-lib64], [force configure to use the lib64 directory (yes) or the lib directory (no) to find libraries @<:@autodetect@:>@]),
  [ USE_LIB64=$enableval ],
  [ if test -z "$USE_LIB64"; then USE_LIB64="autodetect"; fi ]
)
if test x"$USE_LIB64" = "xautodetect"; then
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
AC_ARG_ENABLE(optimize,
  AC_HELP_STRING([--enable-optimize@<:@=LEVEL@:>@], [compiler optimization level @<:@yes@:>@]),
  [ ENABLE_OPTIMIZE=$enableval ],
  [ if test -z "$ENABLE_OPTIMIZE"; then ENABLE_OPTIMIZE=yes; fi ]
)

if test "yes" = "$ENABLE_OPTIMIZE" || test "3" = "$ENABLE_OPTIMIZE"; then
   AX_CFLAGS="$AX_CFLAGS -O3"
elif test "2" = "$ENABLE_OPTIMIZE"; then
   AX_CFLAGS="$AX_CFLAGS -O2"
elif test "1" = "$ENABLE_OPTIMIZE"; then
   AX_CFLAGS="$AX_CFLAGS -O1"
elif test "coreduo" = "$ENABLE_OPTIMIZE"; then
   AX_CFLAGS="$AX_CFLAGS -O4 -march=prescott -mtune=prescott -funroll-loops -msse -msse2 -msse3 -mfpmath=sse"
elif test "sse3" = "$ENABLE_OPTIMIZE"; then
   AX_CFLAGS="$AX_CFLAGS -O4 -funroll-loops -msse -msse2 -msse3 -mfpmath=sse"
else
   echo 'WARNING: *** The Vision Workbench may not work properly with optimization disabled! ***'
   AX_CFLAGS="$AX_CFLAGS -O0"
fi

CFLAGS="$CFLAGS -I\${top_srcdir}/src $AX_CFLAGS"
CXXFLAGS="$CXXFLAGS -I\${top_srcdir}/src $AX_CFLAGS"


AC_ARG_ENABLE(exceptions,
  AC_HELP_STRING([--enable-exceptions@<:@=yes@:>@], [enable the C++ exception mechanism]),
  [ ENABLE_EXCEPTIONS=$enableval ],
  [ if test -z "$ENABLE_EXCEPTIONS"; then ENABLE_EXCEPTIONS=yes; fi ]
)

if test "$ENABLE_EXCEPTIONS" = "yes"; then
   no_exceptions=0
else
   no_exceptions=1
fi

AM_CONDITIONAL(ENABLE_EXCEPTIONS, [test "$ENABLE_EXCEPTIONS" = "yes"])

AC_DEFINE_UNQUOTED(NO_EXCEPTIONS,
                   [$no_exceptions],
                   [Define to 1 to disable C++ exceptions.])

# We set up the include paths ourselves, so
# we ask autoconf to get out of the way.
DEFAULT_INCLUDES=
AC_SUBST(DEFAULT_INCLUDES)


##################################################
#	installation options
##################################################

if test ${prefix} = NONE; then
  if test ! -z ${PREFIX} ; then
    prefix=${PREFIX}
    AC_MSG_NOTICE([using installation prefix ${prefix}])
  fi
fi


##################################################
#	distribution options
##################################################

AC_ARG_WITH(dist-license,
  AC_HELP_STRING([--with-dist-license=LICENSE], [special distribution license file to be included as the COPYING file in the distribution]),
  [DIST_LICENSE=$withval]
)
AC_SUBST(DIST_LICENSE)
AM_CONDITIONAL(DIST_LICENSE, [test ! -z "$DIST_LICENSE"])

if test ! -z "$DIST_LICENSE" ; then
  AC_MSG_NOTICE([using distribution license file $DIST_LICENSE])
fi

AC_ARG_WITH(dist-license-summary,
  AC_HELP_STRING([--with-dist-license-summary=SUMMARY], [special distribution license summary file to be included in the headers of source files in the distribution]),
  [DIST_LICENSE_SUMMARY=$withval]
)
AC_SUBST(DIST_LICENSE_SUMMARY)
AM_CONDITIONAL(DIST_LICENSE_SUMMARY, [test ! -z "$DIST_LICENSE_SUMMARY"])

if test ! -z "$DIST_LICENSE_SUMMARY" ; then
  AC_MSG_NOTICE([using distribution license summary file $DIST_LICENSE_SUMMARY])
fi

AC_ARG_WITH(dist-config-options-default,
  AC_HELP_STRING([--with-dist-config-options-default@<:@=FILE@:>@], [special distribution config.options.default file]),
  [DIST_CONFIG_OPTIONS_DEFAULT=$withval]
)
AC_SUBST(DIST_CONFIG_OPTIONS_DEFAULT)
AM_CONDITIONAL(DIST_CONFIG_OPTIONS_DEFAULT, [test ! -z "$DIST_CONFIG_OPTIONS_DEFAULT"])

if test ! -z "$DIST_CONFIG_OPTIONS_DEFAULT" ; then
  if test "yes" = "$DIST_CONFIG_OPTIONS_DEFAULT" ; then
    DIST_CONFIG_OPTIONS_DEFAULT="config/config.options.default"
  fi
  AC_MSG_NOTICE([using distribution config.options.default file: $DIST_CONFIG_OPTIONS_DEFAULT])
fi

AC_ARG_WITH(dist-remove-modules,
  AC_HELP_STRING([--with-dist-remove-modules="MODULE @<:@MODULE...@:>@"], [remove certain modules from the distribution]),
  [DIST_REMOVE_MODULES=$withval], [DIST_REMOVE_MODULES="Geometry InterestPoint Python"]
)
AC_SUBST(DIST_REMOVE_MODULES)
AM_CONDITIONAL(DIST_REMOVE_MODULES, [test ! -z "$DIST_REMOVE_MODULES"])


##################################################
#	library options
##################################################

if test -z $NUM_THREADS ; then NUM_THREADS=4 ; fi
AH_TEMPLATE([NUM_THREADS],
            [Define to the default number of threads that the Vision
             Workbench should spawn for multi-threaded operations.])
AC_ARG_WITH(num-threads,
  AC_HELP_STRING([--with-num-threads[=4]], [set the default number of processing threads for multi-threaded operations]),
  [NUM_THREADS=$withval])
AC_DEFINE_UNQUOTED([NUM_THREADS],[$NUM_THREADS])

##################################################
#	package checks
##################################################

# Make sure these are set so AC_PROG_CC doesn't initialize them with
# its defaults, since we take care of setting debug and optimization
# levels ourselves.
CFLAGS="$CFLAGS"
CXXFLAGS="$CXXFLAGS"

AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AX_PROG_AR
AC_PROG_LIBTOOL
AC_PROG_MAKE_SET

AC_LANG(C++)

# Qt with AutoTroll.
AT_WITH_QT([+gui +sql +opengl])

if test "yes" = "$ENABLE_VERBOSE"; then
  AC_MSG_NOTICE([using PKG_PATHS=$PKG_PATHS])
fi

AX_PKG_PTHREADS

AX_PKG_BOOST
AX_PKG_BOOST_CHECK_VERSION([ASP_])

AC_MSG_NOTICE([Stereo will be built using Boost version $BOOST_VERSION])
AX_PKG_BOOST_LIB(PROGRAM_OPTIONS, [-lboost_program_options], [boost/program_options.hpp])
AX_PKG_BOOST_LIB(FILESYSTEM_PRE_1_35, [-lboost_filesystem], [boost/filesystem/path.hpp])
if test "$HAVE_PKG_BOOST_FILESYSTEM_PRE_1_35" = "no"; then
  echo "Checking for BOOST_FILESYSTEM 1.35 or later..."
  AX_PKG_BOOST_LIB(FILESYSTEM, [-lboost_filesystem -lboost_system], [boost/filesystem/path.hpp])
else
  AX_PKG(BOOST_FILESYSTEM, [BOOST_FILESYSTEM_PRE_1_35], [], [])
fi
AX_PKG_BOOST_LIB(THREAD, [-lboost_thread], [boost/thread/thread.hpp])

# This provides an easy way for users to override the threads options
# if they are replacing Boost threads with something else for their platform.
AX_PKG(THREADS, [BOOST_THREAD], [], [])

AX_PKG_LAPACK

AX_PKG(VW,                [BOOST], [-lvw],            [vw/Image/ImageView.h vw/Math/LinearAlgebra.h])
AX_PKG(VW_STEREO,         [BOOST], [-lvwStereo],      [vw/Stereo/OptimizedCorrelator.h])
AX_PKG(VW_CAMERA,         [BOOST], [-lvwCamera],      [vw/Camera/CAHVModel.h])
AX_PKG(VW_CARTOGRAPHY,    [BOOST], [-lvwCartography], [vw/Cartography/GeoReference.h])
AX_PKG(VW_INTEREST_POINT, [BOOST], [-lvwInterestPoint])

AX_PKG(OPENSCENEGRAPH, [], [-losg -losgDB -lOpenThreads -losgViewer -losgText -losgGA -losgFX -losgTerrain -losgUtil], [osg/ref_ptr])
AX_PKG(MBA10, [], [-lMBA_1.0_gpl_nopt], [MBA.h])

if test "$HAVE_PKG_MBA10" = "no"; then
  echo "Checking for MBA 1.1"
  AX_PKG(MBA, [], [-lMBA-1.1], [MBA.h])
else
  AX_PKG(MBA, [MBA10], [], [])
fi

AX_PKG(SPICE, [], [-lcspice], [SpiceUsr.h SpiceZfc.h])

dnl ISIS includes QT headers without the proper qt module prefix, so
dnl we need to add this check here to force the build system to add an
dnl extra include path at the top level of the QTCore tree.
dnl We also seed QT_INCLUDE's cppflags with the ones qmake figured out
ADD_QT_INCLUDE_CPPFLAGS="$ADD_QT_INCLUDE_CPPFLAGS $QT_CPPFLAGS"
AX_PKG(QT_INCLUDE, [],           [],          [QVector])

if test x"$host_vendor" = "xapple"; then
    AX_PKG(QWT,     [QT_INCLUDE],   [-lqwt.5])
    AX_PKG(GEOS,    [],             [-lgeos-3.0.0], [])
    AX_PKG(GSL,     [LAPACK],       [-lgsl -lgsl.0 -lgslcblas.0 ])
    AX_PKG(SUPERLU, [],             [-lsuperlu_3.0], [])
    AX_PKG(XERCESC, [],             [-lxerces-c.28], [])
    AX_PKG(QT,      [QT_INCLUDE],   [],              [])
else
    AX_PKG(QWT,     [QT_INCLUDE],   [-lqwt])
    AX_PKG(GEOS,    [],             [-lgeos])
    AX_PKG(GSL,     [LAPACK],       [-lgsl -lgslcblas])
    AX_PKG(SUPERLU, [],             [-lsuperlu])
    AX_PKG(XERCESC, [],             [-lxerces-c])

    AX_PKG(QT_CORE, [QT_INCLUDE],   [-lQtCore],  [QVector])
    AX_PKG(QT_GUI,  [QT_INCLUDE],   [-lQtGui],   [QLabel])
    AX_PKG(QT_SQL,  [QT_INCLUDE],   [-lQtSql],   [QtSql])
    AX_PKG(QT,      [QT_INCLUDE QT_CORE QT_GUI QT_SQL], [], [])
fi

dnl We need to link ISIS3RDPARTY against LAPACK because, on some systems,
dnl the ISIS compilation does not resolve certain BLAS symbols that its
dnl dependent 3rd party libraries depend on (for whatever retarded reason).
dnl Thus, we link against LAPACK here because it defines these missing
dnl BLAS symbols in the event that we need them.
AX_PKG(ISIS3RDPARTY, [SUPERLU LAPACK GSL QWT GEOS SPICE XERCESC], [])
AX_PKG(ISIS, [QT ISIS3RDPARTY], [-lisis3])

# These are here (instead of inside the PKG macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(HAVE_PKG_ISIS, [test "$HAVE_PKG_ISIS" = "yes"])

##################################################
#	module definitions
##################################################

AX_APP(STEREO, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT_CORE QT_GUI])
AX_APP(STEREOGUI, [src], no, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY])
AX_APP(ORTHOPROJECT, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT_CORE QT_GUI])
AX_APP(BUNDLEADJUST, [src], no, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT_CORE QT_GUI])
AX_APP(ORBITVIZ, [src], no, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE], [ISIS ISIS3RDPARTY QT_CORE QT_GUI])
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

##################################################
#	final processing
##################################################
ASP_CPPFLAGS="$OTHER_CPPFLAGS"
ASP_LDFLAGS="$OTHER_LDFLAGS"

if test "yes" = "$ENABLE_VERBOSE" ; then
  AC_MSG_NOTICE([using CPPFLAGS=$CPPFLAGS $ASP_CPPFLAGS])
  AC_MSG_NOTICE([using CFLAGS=$CFLAGS])
  AC_MSG_NOTICE([using CXXFLAGS=$CXXFLAGS])
  AC_MSG_NOTICE([using LDFLAGS=$LDFLAGS $ASP_LDFLAGS])
  AC_MSG_NOTICE([using ASP_NO_EXCEPTIONS=$no_exceptions])
fi

AC_SUBST(ASP_CPPFLAGS)
AC_SUBST(ASP_LDFLAGS)


##################################################
#	makefiles to process
##################################################

AC_CONFIG_FILES([                       \
  Makefile                              \
  src/Makefile                          \
  src/tests/Makefile                    \
])

# This hideous hack makes it possible to use GNU make's conditionals.
ENDIF=endif
AC_SUBST(ENDIF)

AC_OUTPUT
