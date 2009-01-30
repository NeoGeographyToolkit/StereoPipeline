AC_DEFUN([AX_COMMON_OPTIONS], [

##################################################
# Compilation options
##################################################

AX_ARG_ENABLE(exceptions,   yes, [am-yes cpp-bool], [enable the C++ exception mechanism])
AX_ARG_ENABLE(debug,         no, [none],            [generate debugging symbols])
AX_ARG_ENABLE(optimize,       3, [none],            [compiler optimization level])
AX_ARG_ENABLE(lib64,       auto, [none],            [force /lib64 instead of /lib])
AX_ARG_ENABLE(proper-libs,  yes, [none],            [useful linker options])



##################################################
# Handle options
##################################################

# For 64-bit machines, we'll generally want to autodetect if lib64 exists,
# and use it. Sometimes the user will want to force use of lib64 or lib
# instead of the autodetection. Thus we have this command line option.
if test x"$ENABLE_LIB64" = "xauto"; then
  if test -d "/lib64" -o -d "/usr/lib64"; then
    ENABLE_LIB64="yes"
  else
    ENABLE_LIB64="no"
  fi
fi

# Sometimes we have /foo/lib64 and /foo/lib confusion on 64-bit machines,
# so we'll use possibly both if one doesn't appear for a certain
# library path.
if test x"$ENABLE_LIB64" = "xyes"; then
  AX_LIBDIR="lib64"
  AX_OTHER_LIBDIR="lib"
else
  AX_LIBDIR="lib"
  AX_OTHER_LIBDIR="lib64"
fi

if test x"$ENABLE_PROPER_LIBS" = "xyes"; then
# These are good if they're supported
    AX_TRY_CPPFLAGS([-Wl,-no-undefined], [OTHER_LDFLAGS="$OTHER_LDFLAGS -Wl,-no-undefined"])
    AX_TRY_CPPFLAGS([-Wl,-as-needed],    [OTHER_LDFLAGS="$OTHER_LDFLAGS -Wl,-as-needed"])
fi

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
    coreduo) AX_CFLAGS="$AX_CFLAGS -O4 -march=prescott -mtune=prescott -funroll-loops -msse -msse2 -msse3 -mfpmath=sse -ftree-vectorize" ;;
    sse3)    AX_CFLAGS="$AX_CFLAGS -O4 -funroll-loops -msse -msse2 -msse3 -mfpmath=sse -ftree-vectorize" ;;
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

AX_ARG_ENABLE(pkg-paths-default, [${HOME} ${HOME}/local /sw /opt /opt/local /usr/local /usr/X11R6 /usr], [none], [Whether to use a built-in search path])
if test x"$ENABLE_PKG_PATHS_DEFAULT" = "xno"; then
  ENABLE_PKG_PATHS_DEFAULT=
fi

PKG_PATHS_FROM_FILE="$PKG_PATHS"
PKG_PATHS=""
AX_ARG_WITH(pkg-paths, [], [none], [additional search path(s) for packages])
PKG_PATHS="$PKG_PATHS $PKG_PATHS_FROM_FILE $ENABLE_PKG_PATHS_DEFAULT"

if test "yes" = "$ENABLE_VERBOSE"; then
  AC_MSG_NOTICE([using PKG_PATHS=$PKG_PATHS])
fi



##################################################
# library options
##################################################

AX_ARG_WITH(num-threads, [4], [cpp-int], [set the default number of processing threads for multi-threaded operations])
AC_MSG_NOTICE([VW will use $NUM_THREADS THREADS by default])


])
