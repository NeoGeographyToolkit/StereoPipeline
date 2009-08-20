dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006, 2007 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


# Usage: AX_PKG_BOOST
AC_DEFUN([AX_PKG_BOOST],
[
  AC_ARG_WITH(boost,
    AC_HELP_STRING([--with-boost], [enable searching for the boost package @<:@auto@:>@]),
    [ HAVE_PKG_BOOST=$withval ]
  )

  AC_MSG_CHECKING(for package BOOST)

  AC_LANG_ASSERT(C++)

  if test -n "${HAVE_PKG_BOOST}" && test "${HAVE_PKG_BOOST}" != "yes" && test "${HAVE_PKG_BOOST}" != "no" && test x"${HAVE_PKG_BOOST#no:}" == "x$HAVE_PKG_BOOST"; then
    PKG_PATHS_BOOST="${HAVE_PKG_BOOST}"
  else
    PKG_PATHS_BOOST="${PKG_PATHS}"
  fi

  # Skip testing if the user has overridden
  if test "no" = "$HAVE_PKG_BOOST"; then
    AC_MSG_RESULT([no (disabled by user)])
  elif test x"${HAVE_PKG_BOOST#no:}" != "x$HAVE_PKG_BOOST"; then # read as: if has_prefix(HAVE_PKG_BOOST, "no:")
    dnl { and } break AC_MSG_RESULT
    reason="${HAVE_PKG_BOOST#no:}"
    AC_MSG_RESULT([no ($reason)])
    HAVE_PKG_BOOST=no
  else

    PKG_BOOST_CPPFLAGS=
    PKG_BOOST_LIBS=
    HAVE_PKG_BOOST=no

    ax_pkg_old_cppflags="$CPPFLAGS"
    ax_pkg_old_ldflags="$LDFLAGS"

    for ax_boost_base_path in $PKG_PATHS_BOOST; do
      # First look for a system-style installation
      AX_LOG([Checking for a boost in ${ax_boost_base_path}])

      if test -f "${ax_boost_base_path}/include/boost/version.hpp" ; then
        AX_LOG([Found a system-style boost])
        PKG_BOOST_INCDIR="${ax_boost_base_path}/include"
        PKG_BOOST_LIBDIR="${ax_boost_base_path}/${AX_LIBDIR}"
        HAVE_PKG_BOOST="yes"
      else
        # Next look for a default-style installation
        for ax_boost_inc_path in `ls -d ${ax_boost_base_path}/include/boost-* 2> /dev/null` ; do
          AX_LOG([Checking for default-style boost in ${ax_boost_inc_path}])
          if test -f "${ax_boost_inc_path}/boost/version.hpp"; then
            AX_LOG([Found a default-style boost in ${ax_boost_inc_path}])
            # At the moment we greedily accept the first one we find, regardless of version
            PKG_BOOST_INCDIR="${ax_boost_inc_path}"
            PKG_BOOST_LIBDIR="${ax_boost_base_path}/${AX_LIBDIR}"
            HAVE_PKG_BOOST="yes"
          fi
        done
      fi

      if test x"${HAVE_PKG_BOOST}" = "xyes"; then

        HAVE_PKG_BOOST="no"

        # In case it's not in lib64 despite specifying lib64...
        if test ! -d $PKG_BOOST_LIBDIR -a x"${AX_OTHER_LIBDIR}" != "x"; then
          PKG_BOOST_LIBDIR="${ax_boost_base_path}/${AX_OTHER_LIBDIR}"
        fi

        CPPFLAGS="$ax_pkg_old_cppflags -I${PKG_BOOST_INCDIR}"
        LDFLAGS="$ax_pkg_old_ldflags -L${PKG_BOOST_LIBDIR}"

        echo "#include <boost/version.hpp>" > conftest.h

        dnl check for the header
        dnl otherwise, check next path
        AC_LINK_IFELSE(
          AC_LANG_PROGRAM([#include "conftest.h"],[]),
          [ HAVE_PKG_BOOST=yes; break; ])

        HAVE_PKG_BOOST="no"
        unset PKG_BOOST_INCDIR
        unset PKG_BOOST_LIBDIR

      fi

    done

    CPPFLAGS="$ax_pkg_old_cppflags"
    LDFLAGS="$ax_pkg_old_ldflags"

    AC_MSG_RESULT([$HAVE_PKG_BOOST])

  fi

  if test "${HAVE_PKG_BOOST}" = "yes" ; then
    ax_have_pkg_bool=1
    PKG_BOOST_CPPFLAGS="-I${PKG_BOOST_INCDIR}"
    PKG_BOOST_LIBS="-L${PKG_BOOST_LIBDIR}"
    OTHER_CPPFLAGS="${OTHER_CPPFLAGS} ${PKG_BOOST_CPPFLAGS}"
    OTHER_LDFLAGS="${OTHER_LDFLAGS} ${PKG_BOOST_LIBS}"
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_BOOST],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the BOOST package is available.])

  AC_SUBST(PKG_BOOST_CPPFLAGS)
  AC_SUBST(PKG_BOOST_LIBS)
  AC_SUBST(HAVE_PKG_BOOST)

  AX_LOG(HAVE_PKG_BOOST=${HAVE_PKG_BOOST})
  AX_LOG(PKG_BOOST_CPPFLAGS=$PKG_BOOST_CPPFLAGS)
  AX_LOG(PKG_BOOST_LIBS=$PKG_BOOST_LIBS)
  AX_LOG(CPPFLAGS=$CPPFLAGS)
  AX_LOG(LDFLAGS=$LDFLAGS)
  AX_LOG(OTHER_CPPFLAGS=$OTHER_CPPFLAGS)
  AX_LOG(OTHER_LDFLAGS=$OTHER_LDFLAGS)
])
