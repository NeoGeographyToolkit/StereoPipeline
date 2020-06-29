dnl __BEGIN_LICENSE__
dnl  Copyright (c) 2009-2013, United States Government as represented by the
dnl  Administrator of the National Aeronautics and Space Administration. All
dnl  rights reserved.
dnl
dnl  The NGT platform is licensed under the Apache License, Version 2.0 (the
dnl  "License"); you may not use this file except in compliance with the
dnl  License. You may obtain a copy of the License at
dnl  http://www.apache.org/licenses/LICENSE-2.0
dnl
dnl  Unless required by applicable law or agreed to in writing, software
dnl  distributed under the License is distributed on an "AS IS" BASIS,
dnl  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
dnl  See the License for the specific language governing permissions and
dnl  limitations under the License.
dnl __END_LICENSE__


AC_DEFUN([AX_COMMON_OPTIONS], [

##################################################
# Compilation options
##################################################

AX_ARG_ENABLE(exceptions,   yes, [am-yes cpp-bool], [enable the C++ exception mechanism])
AX_ARG_ENABLE(debug,         no, [none],            [generate debugging symbols])
AX_ARG_ENABLE(optimize,       3, [none],            [compiler optimization level])
AX_ARG_ENABLE(profile,       no, [none],            [generate profiling data])
AX_ARG_ENABLE(arch-libs,     no, [none],            [force /lib64 (=64) or /lib32 (=32) instead of /lib])
AX_ARG_ENABLE(ccache,        no, [none],            [try to use ccache, if available])
AX_ARG_ENABLE(multi-arch,    [], [none],            [build multi-arch (universal) binaries])
AX_ARG_ENABLE(no-undefined,  no, [none],            [set -Wl,-no-undefined (might break linking)])
AX_ARG_ENABLE(rpath,         no, [none],            [set RPATH/RUNPATH on generated binaries])
AX_ARG_ENABLE(as-needed,     no, [none],            [set -Wl,-as-needed (might break linking)])
AX_ARG_ENABLE(google-tcmalloc, yes, [none],            [Try to use google perftools' tcmalloc])
AX_ARG_ENABLE(google-profiler,  no, [none],            [Try to use google perftools' cpu profiler])


##################################################
# Handle options
##################################################

# Sometimes we have /foo/lib64 and /foo/lib confusion on 64-bit machines,
# so we'll use possibly both if one doesn't appear for a certain
# library path.
AX_OTHER_LIBDIR="lib"
if test x"$ENABLE_ARCH_LIBS" = "x64"; then
  AX_LIBDIR="lib64"
elif test x"$ENABLE_ARCH_LIBS" = "x32"; then
  AX_LIBDIR="lib32"
else
  AX_LIBDIR="lib"
  AX_OTHER_LIBDIR=""
fi

if test x"$ENABLE_GOOGLE_TCMALLOC" != x"yes"; then
  HAVE_PKG_TCMALLOC=no:disabled
fi
if test x"$ENABLE_GOOGLE_PROFILER" != x"yes"; then
  HAVE_PKG_GOOGLE_PROFILER=no:disabled
fi

AX_PKG(TCMALLOC,        [], [-ltcmalloc], [google/tcmalloc.h])
AX_PKG(GOOGLE_PROFILER, [], [-lprofiler], [google/profiler.h])

if test x"$HAVE_PKG_TCMALLOC" = x"yes"; then
  LIBS="$LIBS $PKG_TCMALLOC_LIBS"
fi

if test x"$HAVE_PKG_GOOGLE_PROFILER" = x"yes"; then
  LIBS="$LIBS $PKG_GOOGLE_PROFILER_LIBS"
fi

# Pass apple gcc options to build a universal binary
for arch in $ENABLE_MULTI_ARCH; do
    AX_CFLAGS="$AX_CFLAGS -arch $arch"
done

if test x"$ENABLE_CCACHE" = x"yes"; then
    AC_CHECK_PROGS(CCACHE, ccache, false)
    if test x"$CCACHE" != "xfalse"; then
        CC="$CCACHE $CC"
        CXX="$CCACHE $CXX"
    fi
fi

# These are good if they're supported
if test x"$ENABLE_NO_UNDEFINED" = "xyes"; then
    AX_TRY_CPPFLAGS([-Wl,-no-undefined], [OTHER_LDFLAGS="$OTHER_LDFLAGS -Wl,-no-undefined"])
fi

if test x"$ENABLE_AS_NEEDED" = "xyes"; then
    AX_TRY_CPPFLAGS([-Wl,-as-needed],    [OTHER_LDFLAGS="$OTHER_LDFLAGS -Wl,-as-needed"])
fi

# Debugging
case "$ENABLE_DEBUG" in
    yes|1) AX_CFLAGS="$AX_CFLAGS -g -DDEBUG" ;;
    2)     AX_CFLAGS="$AX_CFLAGS -g -DDEBUG -D_GLIBCXX_DEBUG" ;;
    no)    AX_CFLAGS="$AX_CFLAGS -DNDEBUG" ;;
    ignore) ;;
    *)     AC_MSG_ERROR([Unknown debug option: "$ENABLE_DEBUG"]) ;;
esac

# Optimization
case "$ENABLE_OPTIMIZE" in
    yes)     AX_CFLAGS="$AX_CFLAGS -O3" ;;
    3|2|1)   AX_CFLAGS="$AX_CFLAGS -O$ENABLE_OPTIMIZE" ;;
    no|0)    AC_MSG_WARN([*** The Vision Workbench may not work properly with optimization disabled! ***])
             AX_CFLAGS="$AX_CFLAGS -O0" ;;
    ignore)  ;;
    *)       AC_MSG_ERROR([Unknown optimize option: "$ENABLE_OPTIMIZE"]) ;;
esac

if test x"$ENABLE_PROFILE" = "xyes"; then
    AX_TRY_CPPFLAGS([-pg], [AX_CFLAGS="$AX_CFLAGS -pg"], [AC_MSG_ERROR([Cannot enable profiling: compiler doesn't seem to support it])])
fi

AX_TRY_CPPFLAGS([-Wno-missing-field-initializers], [
  CFLAGS="-Wno-missing-field-initializers $CFLAGS "
  CXXFLAGS="-Wno-missing-field-initializers $CXXFLAGS"
])

CFLAGS="-Wall -Wextra -Wno-unused-parameter $AX_CFLAGS $CFLAGS"
CXXFLAGS="-Wall -Wextra -Wno-unused-parameter $AX_CFLAGS $CXXFLAGS"

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

AX_ARG_ENABLE(pkg-paths-default, [${HOME} ${HOME}/local /sw /opt /opt/local /usr/local /usr/X11R6 /usr /usr/local/cuda], [none], [Whether to use a built-in search path])
if test x"$ENABLE_PKG_PATHS_DEFAULT" = "xno"; then
  ENABLE_PKG_PATHS_DEFAULT=
fi

PKG_PATHS_FROM_FILE="$PKG_PATHS"
PKG_PATHS=""
AX_ARG_WITH(pkg-paths, [], [none], [additional search path(s) for packages])
PKG_PATHS="$PKG_PATHS $PKG_PATHS_FROM_FILE $ENABLE_PKG_PATHS_DEFAULT"

AX_LOG([using PKG_PATHS=$PKG_PATHS])



##################################################
# library options
##################################################

AX_ARG_WITH(num-threads, [4], [cpp-int], [set the default number of processing threads for multi-threaded operations])
AC_MSG_NOTICE([VW will use $NUM_THREADS THREADS by default])


])
