dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006, 2007 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl Here's a new version of AX_PKG_BOOST_LIB designed to find the
dnl multithreaded boost libraries and boost libraries that are just weirdly
dnl named in general. Boost libraries follow a weird naming convention
dnl that makes our old logic not work. You can't just add -mt to the old
dnl library you're looking for, because the -compiler part comes first.
dnl IE, the non-multithreaded library would be named libboost_X-gcc41.so,
dnl and the multithreaded library would be named libboost_X-gcc41-mt.so.
dnl
dnl For that reason, we've added an environment variable:
dnl BOOST_LIBRARIES_SUFFIX. The function here tries to find a version of
dnl Boost with the string in said variable somewhere inside the Boost
dnl library names, but after the initial name of the library (specified
dnl as the second parameter to this function). A blank value will give
dnl normal behavior.
# Usage: AX_PKG_BOOST_LIB(<name>, <libraries>, <header>, [<ldflags>])
AC_DEFUN([AX_PKG_BOOST_LIB],
[
  AC_MSG_CHECKING(for package BOOST_$1)

  AC_LANG_ASSERT(C++)

  # Skip testing if the user has overridden
  if test -z ${HAVE_PKG_BOOST_$1}; then

    HAVE_PKG_BOOST_$1=no

    # Check for general Boost presence
    if test "x${HAVE_PKG_BOOST}" = "xyes" ; then
      # Check for required headers
      AX_FIND_FILES([$3],[${PKG_BOOST_INCDIR}])
      if test ! -z "$ax_find_files_path" ; then
        # Check for required libraries with no suffix, aside from the one
        # given by environment variable.
        AX_FIND_FILES([`echo $2 | sed "s/-l\([[^[:space:]]]*\)/lib\1${BOOST_LIBRARIES_SUFFIX}.*/g"`],[$PKG_BOOST_LIBDIR])
        if test ! -z "$ax_find_files_path" ; then
          HAVE_PKG_BOOST_$1="yes"
          ax_pkg_boost_lib=`echo $2 | sed "s/\(-l[[^[:space:]]]*\)/\1${BOOST_LIBRARIES_SUFFIX}/g"`
          PKG_BOOST_$1_LIBS="$PKG_BOOST_LIBS $ax_pkg_boost_lib $4"
        else
          # Check for required libraries with some suffix. We have to check
          # for both a suffix before ${BOOST_LIBRARIES_SUFFIX} (pre-suffix)
          # and a suffix after (for example) the -mt (post-suffix), because
          # boost likes to stick the name of the compiler before the -mt.
          # Extremely annoying.

          ax_pkg_boost_lib=`echo $2 | awk '{print [$]1}' | sed 's/-l\([[^[:space:]-]]*\).*/lib\1/g'`
          ax_pkg_boost_file=`ls ${PKG_BOOST_LIBDIR}/${ax_pkg_boost_lib}-*${BOOST_LIBRARIES_SUFFIX}${BOOST_LIBRARIES_SUFFIX:+*} | head -n 1 | sed "s,^${PKG_BOOST_LIBDIR}/\(.*\),\1,"`

          # The pre-suffix.
          ax_pkg_boost_presuffix=`echo ${ax_pkg_boost_file} | sed "s/${ax_pkg_boost_lib}\([[^.]]*\)${BOOST_LIBRARIES_SUFFIX}.*/\1/"`

          # The post-suffix.
          ax_pkg_boost_postsuffix=`echo ${ax_pkg_boost_file} | sed "s/${ax_pkg_boost_lib}${ax_pkg_boost_presuffix}${BOOST_LIBRARIES_SUFFIX}\([[^.]]*\).*/\1/"`

          AX_FIND_FILES([`echo $2 | sed "s/-l\([[^[:space:]]]*\)/lib\1${ax_pkg_boost_presuffix}${BOOST_LIBRARIES_SUFFIX}${ax_pkg_boost_postsuffix}.*/g"`],[$PKG_BOOST_LIBDIR])
          if test ! -z $ax_find_files_path ; then
            HAVE_PKG_BOOST_$1="yes"
            PKG_BOOST_$1_LIBS=`echo $2 | sed "s/[[^ ]]*/&${ax_pkg_boost_presuffix}${BOOST_LIBRARIES_SUFFIX}${ax_pkg_boost_postsuffx}/g"`
            PKG_BOOST_$1_LIBS="$PKG_BOOST_LIBS $PKG_BOOST_$1_LIBS $4"
          fi
        fi
      fi
    fi
  fi

  # Check to make sure we found a working one
  ax_pkg_old_cppflags="$CPPFLAGS"
  ax_pkg_old_libs="$LIBS"

  echo > conftest.h
  for header in $3 ; do
    echo "#include <$header>" >> conftest.h
  done

  CPPFLAGS="$CPPFLAGS $PKG_BOOST_CPPFLAGS"
  LIBS="$LIBS $PKG_BOOST_$1_LIBS"

  AC_LINK_IFELSE(
    AC_LANG_PROGRAM([#include "conftest.h"],[]),
    [ HAVE_PKG_BOOST_$1=yes ], [ HAVE_PKG_BOOST_$1=no ] )

  CPPFLAGS="$ax_pkg_old_cppflags"
  LIBS="$ax_pkg_old_libs"

  AC_MSG_RESULT([$HAVE_PKG_BOOST_]$1)

  if test "${HAVE_PKG_BOOST_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_BOOST_$1],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the BOOST_$1 package is available.])

  AC_SUBST(HAVE_PKG_BOOST_$1)
  AC_SUBST(PKG_BOOST_$1_LIBS)

  AX_LOG([HAVE_PKG_BOOST_$1=${HAVE_PKG_BOOST_$1}])
  AX_LOG([PKG_BOOST_$1_LIBS=$PKG_BOOST_$1_LIBS])
  AX_LOG([CPPFLAGS=$CPPFLAGS])
  AX_LOG([LDFLAGS=$LDFLAGS])
])
