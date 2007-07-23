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


AC_INIT([NASA Ames Stereo Pipeline], [2.0], [asp@nx.arc.nasa.gov], [StereoPipeline])
AC_CONFIG_AUX_DIR(config)
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE
AX_CONFIG_HEADER_PREFIX([src/asp_config.h], [ASP_])

##################################################
#	configuration options
##################################################

# This should be first, so everything else can use it
AC_ARG_ENABLE(verbose,
  AC_HELP_STRING([--enable-verbose], [verbose configuration output @<:@no@:>@]),           
  [ ENABLE_VERBOSE=$enableval ],
  [ if test -z $"ENABLE_VERBOSE"; then ENABLE_VERBOSE=no; fi ]
)

if ! test -f config.options && test -f config/config.options.default; then
  if test "yes" = "$ENABLE_VERBOSE" ; then
    AC_MSG_NOTICE([creating default config.options file])
  fi
  cp config/config.options.default config.options
fi

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

CPPFLAGS="$CPPFLAGS -I\${top_srcdir}/src"

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
   AX_CFLAGS="$AX_CFLAGS -O4 -march=prescott -mtune=prescott -funroll-loops -fexpensive-optimizations -msse -msse2 -msse3 -mfpmath=sse"
elif test "sse3" = "$ENABLE_OPTIMIZE"; then
   AX_CFLAGS="$AX_CFLAGS -O4 -funroll-loops -fexpensive-optimizations -msse -msse2 -msse3 -mfpmath=sse"
else
   AX_CFLAGS="$AX_CFLAGS -O0"
fi

CFLAGS="$CFLAGS $AX_CFLAGS"
CXXFLAGS="$CXXFLAGS $AX_CFLAGS"

if test "yes" = "$ENABLE_VERBOSE" ; then
  AC_MSG_NOTICE([using CPPFLAGS=$CPPFLAGS])
  AC_MSG_NOTICE([using CFLAGS=$CFLAGS])
  AC_MSG_NOTICE([using CXXFLAGS=$CXXFLAGS])
fi

# We set up the include paths ourselves, so 
# we ask autoconf to get out of the way.
DEFAULT_INCLUDES=
AC_SUBST(DEFAULT_INCLUDES)

##################################################
#	installation options
##################################################

AC_ARG_ENABLE(custom-arch-name,
  AC_HELP_STRING([--enable-custom-arch-name=NAME], [custom architecture name, used for generating the default intallation prefix]),
  [ENABLE_CUSTOM_ARCH_NAME=$enableval]
)

if test ${prefix} = NONE; then
  if test ! -z ${ENABLE_CUSTOM_ARCH_NAME} ; then
    prefix=`pwd`/build/${ENABLE_CUSTOM_ARCH_NAME}
  else 
    prefix=`pwd`/build/${host}
  fi
fi

AC_MSG_NOTICE([using installation prefix ${prefix}])


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


##################################################
#	package checks	    
##################################################

AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
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
AX_PKG_BOOST_LIB(REGEX, [-lboost_regex], [boost/regex/config.hpp])

if test `uname -s` == Darwin; then
AX_PKG_LAPACK
fi

AX_PKG(M, [], [-lm], [math.h])
AX_PKG(Z, [], [-lz], [zlib.h])
AX_PKG(PNG, [], [-lpng], [png.h])
AX_PKG(TIFF, [], [-ltiff], [tiff.h])
AX_PKG(JPEG, [], [-ljpeg], [jpeglib.h])

AX_PKG(VW, [], [-lvw], [vw/Image/ImageView.h])
AX_PKG(VW_MATH, [], [-lvwMath], [vw/Math/LinearAlgebra.h])
AX_PKG(VW_STEREO, [], [-lvwStereo], [vw/Stereo/OptimizedCorrelator.h])
AX_PKG(VW_CAMERA, [], [-lvwCamera], [vw/Camera/CAHVModel.h])
AX_PKG(VW_CARTOGRAPHY, [], [-lvwCartography], [vw/Cartography/GeoReference.h])
AX_PKG(VW_INTEREST_POINT, [], [-lvwInterestPoint])

AX_PKG(GDAL, [], [-lgdal], [gdal.h])
AM_CONDITIONAL(HAVE_PKG_GDAL, [test "$HAVE_PKG_GDAL" = "yes"])

AX_PKG(OPENSCENEGRAPH, [], [-losg -losgDB -lOpenThreads], [osg/ref_ptr])
AX_PKG(MBA, [], [-lMBA_1.0_gpl_nopt], [MBA.h])
AX_PKG(SPICE, [], [-lcspice -lcsupport], [SpiceUsr.h SpiceZfc.h])
AX_PKG(SIFT, [], [-llowesift], [key.h])

##################################################
#	module definitions
##################################################

if test `uname -s` == Darwin; then
AX_MODULE(STEREOPIPELINE, src, yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM BOOST_THREAD BOOST_REGEX VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_MATH LAPACK Z PNG TIFF JPEG MBA SPICE], [SIFT VW_INTEREST_POINT GDAL OPENSCENEGRAPH])
else 
AX_MODULE(STEREOPIPELINE, src, yes, [BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM BOOST_THREAD BOOST_REGEX VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_MATH Z PNG TIFF JPEG MBA SPICE], [SIFT VW_INTEREST_POINT GDAL OPENSCENEGRAPH])
fi

if test ${MAKE_MODULE_STEREOPIPELINE} != "yes"; then
  AC_MSG_ERROR([cannot build the Stereo Pipeline!])
fi

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
