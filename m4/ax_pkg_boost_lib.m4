# Usage: AX_PKG_BOOST_LIB(<name>, <libraries>, <header>)
AC_DEFUN([AX_PKG_BOOST_LIB],
[
  AC_MSG_CHECKING(for package BOOST_$1)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  AC_LANG_ASSERT(C++)

  # Skip testing if the user has overridden
  if test -z "${HAVE_PKG_BOOST_$1}"; then

    HAVE_PKG_BOOST_$1=no

    # Check for general Boost presence
    if test "x${HAVE_PKG_BOOST}" = "xyes" ; then
      # Check for required headers
      AX_FIND_FILES([$3],[${PKG_BOOST_INCDIR}])
      if test ! -z "$ax_find_files_path" ; then
        # Check for required libraries with no suffix
        AX_FIND_FILES([`echo $2 | sed 's/-l\([[^[:space:]]]*\)/lib\1.*/g'`],[$PKG_BOOST_LIBDIR])
        if test ! -z "$ax_find_files_path" ; then
          HAVE_PKG_BOOST_$1="yes"
          PKG_BOOST_$1_LIBS="$2"
        else
          # Check for required libraries with some suffix
          ax_pkg_boost_lib=`echo $2 | awk '{print [$]1}' | sed 's/-l\([[^[:space:]-]]*\).*/lib\1/g'`
          ax_pkg_boost_lib_ext=`ls ${PKG_BOOST_LIBDIR}/${ax_pkg_boost_lib}-* | head -n 1 | sed "s,^${PKG_BOOST_LIBDIR}/${ax_pkg_boost_lib}\(-[[^.]]*\).*,\1,"`
          if test ! -z "$ax_pkg_boost_lib_ext" ; then
            AX_FIND_FILES([`echo $2 | sed "s/-l\([[^[:space:]]]*\)/lib\1${ax_pkg_boost_lib_ext}.*/g"`],[$PKG_BOOST_LIBDIR])
            if test ! -z "$ax_find_files_path" ; then
              HAVE_PKG_BOOST_$1="yes"
              PKG_BOOST_$1_LIBS=`echo $2 | sed "s/[[^ ]]*/&${ax_pkg_boost_lib_ext}/g"`
            fi
          fi
        fi
      fi
    fi
  fi

  ax_pkg_old_vw_cppflags=$ASP_CPPFLAGS
  ax_pkg_old_vw_ldflags=$ASP_LDFLAGS
  ax_pkg_old_cppflags=$CPPFLAGS
  ax_pkg_old_ldflags=$LDFLAGS
  ax_pkg_old_libs=$LIBS
  while true ; do
    echo > conftest.h
    for header in $3 ; do
      echo "#include <$header>" >> conftest.h
    done
    # First see if the current paths are sufficient
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_CHECKING([whether current paths are sufficient...])
    fi
    CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
    LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
    LIBS="$PKG_BOOST_$1_LIBS $ax_pkg_old_libs"
    AC_LINK_IFELSE( AC_LANG_PROGRAM([#include "conftest.h"],[]), [ax_result=yes], [ax_result=no] )
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_RESULT([$ax_result])
    fi
    if test "$ax_result" = "yes" ; then break ; fi
    # Try it with just the include path
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_CHECKING([whether adding the include path is sufficient...])
    fi
    ASP_CPPFLAGS="-I${PKG_BOOST_INCDIR} $ASP_CPPFLAGS"
    CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
    AC_LINK_IFELSE( AC_LANG_PROGRAM([#include "conftest.h"],[]), [ax_result=yes], [ax_result=no] )
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_RESULT([$ax_result])
    fi
    if test "$ax_result" = "yes" ; then break ; fi
    # Finally, try it with the linker path
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_CHECKING([whether adding the include and linker paths works...])
    fi
    ASP_LDFLAGS="-L${PKG_BOOST_LIBDIR} $ASP_LDFLAGS"
    LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
    AC_LINK_IFELSE( AC_LANG_PROGRAM([#include "conftest.h"],[]), [ax_result=yes], [ax_result=no] )
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_RESULT([$ax_result])
    fi
    if test "$ax_result" = "yes" ; then break ; fi
    # The detected version of boost seems to be invalid!
    HAVE_PKG_BOOST_$1="no"
    ASP_CPPFLAGS="$ax_pkg_old_vw_cppflags"
    ASP_LDFLAGS="$ax_pkg_old_vw_ldflags"
    break
  done

  CPPFLAGS="$ax_pkg_old_cppflags"
  LDFLAGS="$ax_pkg_old_ldflags"
  LIBS="$ax_pkg_old_libs"

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

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([HAVE_PKG_BOOST_$1 = ${HAVE_PKG_BOOST_$1}])
    AC_MSG_NOTICE([PKG_BOOST_$1_LIBS= $PKG_BOOST_$1_LIBS])
  else
    AC_MSG_RESULT([${HAVE_PKG_BOOST_$1}])
  fi
])
