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

CPPFLAGS="$CPPFLAGS -I\${top_srcdir}/src -I/Users/mbroxton/local/qt-4.3.2/include/ -I/Users/mbroxton/local/qt-4.3.2/include/QtCore"

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

CFLAGS="$CFLAGS $AX_CFLAGS"
CXXFLAGS="$CXXFLAGS $AX_CFLAGS"

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

if test "yes" = "$ENABLE_VERBOSE"; then
  AC_MSG_NOTICE([using PKG_PATHS=$PKG_PATHS])
fi

AX_PKG_PTHREADS

AX_PKG_BOOST
AX_PKG_BOOST_LIB(PROGRAM_OPTIONS, [-lboost_program_options], [boost/program_options.hpp])
AX_PKG_BOOST_LIB(FILESYSTEM, [-lboost_filesystem], [boost/filesystem/path.hpp])
AX_PKG_BOOST_LIB(THREAD, [-lboost_thread], [boost/thread/thread.hpp])

AX_PKG_LAPACK

# This provides an easy way for users to override the threads options 
# if they are replacing Boost threads with something else for their platform.
AX_PKG(THREADS, [BOOST_THREAD], [], [])

AX_PKG(VW, [], [-lvw], [vw/Image/ImageView.h vw/Math/LinearAlgebra.h])
AX_PKG(VW_STEREO, [], [-lvwStereo], [vw/Stereo/OptimizedCorrelator.h])
AX_PKG(VW_CAMERA, [], [-lvwCamera], [vw/Camera/CAHVModel.h])
AX_PKG(VW_CARTOGRAPHY, [], [-lvwCartography], [vw/Cartography/GeoReference.h])
AX_PKG(VW_INTEREST_POINT, [], [-lvwInterestPoint])

AX_PKG(OPENSCENEGRAPH, [], [-losg -losgDB -lOpenThreads], [osg/ref_ptr])
AX_PKG(MBA, [], [-lMBA_1.0_gpl_nopt], [MBA.h])
AX_PKG(SPICE, [], [-lcspice -lcsupport], [SpiceUsr.h SpiceZfc.h])
#AX_PKG(QT, [], [], [QTCore.h])
#AX_PKG(ISIS, [], [-lisis3])
#AX_PKG(ISIS3RDPARTY, [], [-lgeos -lxerces-c -lgsl -lgslcblas -liconv -lqwt])

##################################################
#	module definitions
##################################################

AX_APP(STEREO, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE])
#AX_APP(ISISSTEREO, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT LAPACK MBA SPICE QT ISIS ISIS3RDPARTY])
AX_APP(DISPARITYDEBUG, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS VW VW_STEREO])
AX_APP(POINT2MESH, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS VW OPENSCENEGRAPH])
AX_APP(POINT2DEM, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS VW VW_CARTOGRAPHY])
AX_APP(NURBS, [src], yes, [BOOST BOOST_PROGRAM_OPTIONS VW MBA])
AX_APP(CTXIMAGE, [src], no, [BOOST BOOST_PROGRAM_OPTIONS VW VW_CARTOGRAPHY])

##################################################
#	final processing	    
##################################################

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
])

# This hideous hack makes it possible to use GNU make's conditionals.
ENDIF=endif
AC_SUBST(ENDIF)

AC_OUTPUT
