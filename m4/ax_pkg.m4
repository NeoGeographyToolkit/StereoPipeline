dnl Usage: AX_PKG(<name>, <dependencies>, <libraries>, <headers>[, <relative include path>, <required-functions>])
AC_DEFUN([AX_PKG],
[
  AC_ARG_WITH(m4_tolower([[$1]]),
    AC_HELP_STRING([--with-]m4_tolower([[$1]]), [enable searching for the $1 package @<:@auto@:>@]),
    [ HAVE_PKG_$1=$withval ]
  )

  if test x"$ENABLE_VERBOSE" = "xyes"; then
    if test -z "$6"; then
      AC_MSG_CHECKING([for package $1 in current paths])
    else
      AC_MSG_CHECKING([for package $1 in current paths with functions ($6)])
    fi
  else
    if test -z "$6"; then
      AC_MSG_CHECKING([for package $1])
    else
      AC_MSG_CHECKING([for package $1 with functions ($6)])
    fi
  fi

  AC_LANG_ASSERT(C++)

  # We can skip searching if we're already at "no"
  if test "no" = "$HAVE_PKG_$1"; then
    AC_MSG_RESULT([no (disabled by user)])

  else
    if test -z "${PKG_$1_LDFLAGS}"; then
        PKG_$1_LIBS="$3"
    else
        PKG_$1_LIBS="${PKG_$1_LDFLAGS}"
    fi

    # Test for and inherit from dependencies
    for x in $2; do
      ax_pkg_have_dep=HAVE_PKG_${x}
      if test "${!ax_pkg_have_dep}" = "yes"; then
        ax_pkg_dep_cxxflags="PKG_${x}_CPPFLAGS"
        ax_pkg_dep_libs="PKG_${x}_LIBS"
        PKG_$1_CPPFLAGS="$PKG_$1_CPPFLAGS ${!ax_pkg_dep_cxxflags}"
        PKG_$1_LIBS="$PKG_$1_LIBS ${!ax_pkg_dep_libs}"
        unset ax_pkg_dep_cxxflags
        unset ax_pkg_dep_libs
      else
        unset PKG_$1_CPPFLAGS
        unset PKG_$1_LIBS
        HAVE_PKG_$1="no"
        break
      fi
    done

    if test "x$HAVE_PKG_$1" = "xno" ; then
      AC_MSG_RESULT([no (needs $x)])

    # We skip the search if the user has been explicit about "yes"
    elif test "x$HAVE_PKG_$1" = "xyes" ; then
      AC_MSG_RESULT([yes (using user-supplied flags)])

    # Otherwise we look for a path that contains the needed headers and libraries
    else

      if test "x$ENABLE_VERBOSE" = "yes"; then
        AC_MSG_RESULT([searching...])
      fi

      if test -n "${PKG_$1_LDFLAGS}"; then
        PKG_PATHS_$1=""
      elif test -n "${HAVE_PKG_$1}" && test "${HAVE_PKG_$1}" != "yes" && test "${HAVE_PKG_$1}" != "no"; then
        PKG_PATHS_$1=${HAVE_PKG_$1}
      else
        PKG_PATHS_$1="default ${PKG_PATHS}"
      fi

      HAVE_PKG_$1=no

      ax_pkg_old_libs="$LIBS"
      ax_pkg_old_cppflags="$CPPFLAGS"
      ax_pkg_old_ldflags="$LDFLAGS"
      ax_pkg_old_other_cppflags="$OTHER_CPPFLAGS"
      ax_pkg_old_other_ldflags="$OTHER_LDFLAGS"

      LIBS="$PKG_$1_LIBS $LIBS"
      for path in $PKG_PATHS_$1; do
        echo ["SEARCH: Checking $path for $3"] >&AS_MESSAGE_LOG_FD

        CPPFLAGS="$PKG_$1_CPPFLAGS $ax_pkg_old_cppflags"
        LDFLAGS="$ax_pkg_old_ldflags"
        OTHER_CPPFLAGS="$ax_pkg_old_other_cppflags"
        OTHER_LDFLAGS="$ax_pkg_old_other_ldflags"

        echo > conftest.h
        for header in $4 ; do
          echo "#include <$header>" >> conftest.h
        done
        TRY_ADD_CPPFLAGS=""
        TRY_ADD_LDFLAGS=""

        if test x"$ENABLE_VERBOSE" = "xyes"; then
          AC_MSG_CHECKING([for package $1 in $path])
        fi

        if test "$path" != "default"; then
          # ISIS is really stupid, and they use /foo/inc as their include file
          # location instead of /foo/include. So we check for that. This sees 
          # about any other idiot libraries that use the same design as well.
          AX_INCLUDE_DIR=include
          if ! test -d $path/${AX_INCLUDE_DIR}; then
            if test -d $path/inc; then
              AX_INCLUDE_DIR=inc
            fi
          fi

          if test -z "$5"; then
            TRY_ADD_CPPFLAGS="$PKG_$1_CPPFLAGS -I$path/${AX_INCLUDE_DIR}"
          else
            TRY_ADD_CPPFLAGS="$PKG_$1_CPPFLAGS -I$path/${AX_INCLUDE_DIR}/$5"
          fi

          if test -d $path/${AX_LIBDIR}; then
              TRY_ADD_LDFLAGS="-L$path/${AX_LIBDIR}"
          elif test x"${AX_LIBDIR}" = "xlib64"; then
              TRY_ADD_LDFLAGS="-L$path/${AX_OTHER_LIBDIR}"
          fi

          CPPFLAGS="$CPPFLAGS $TRY_ADD_CPPFLAGS"
          LDFLAGS="$LDFLAGS $TRY_ADD_LDFLAGS"
        else
            # search in current paths
            CPPFLAGS="$CPPFLAGS $OTHER_CPPFLAGS"
            LDFLAGS="$LDFLAGS $OTHER_LDFLAGS"
        fi

        dnl check for the headers and libs. if found, keep going.
        dnl otherwise, check next path
        AC_LINK_IFELSE(
          AC_LANG_PROGRAM([#include "conftest.h"],[]),
          [ HAVE_PKG_$1=yes ], [continue] )

        m4_ifval([$6],
            AX_CHECK_FUNCTIONS([$6], [$LDFLAGS $LIBS], [], [ HAVE_PKG_$1=no; echo "package $1 did not have function $func" >&AS_MESSAGE_LOG_FD ])
        )

        if test x"$HAVE_PKG_$1" = x"yes"; then
            AC_MSG_RESULT([yes])
            break
        fi

        TRY_ADD_CPPFLAGS=""
        TRY_ADD_LDFLAGS=""
        if test x"$ENABLE_VERBOSE" = "xyes"; then
          AC_MSG_RESULT([no])
        fi
      done

      PKG_$1_CPPFLAGS="$PKG_$1_CPPFLAGS $TRY_ADD_CPPFLAGS"
      PKG_$1_LIBS="$PKG_$1_LIBS $TRY_ADD_LDFLAGS"

      OTHER_CPPFLAGS="$OTHER_CPPFLAGS $TRY_ADD_CPPFLAGS"
      OTHER_LDFLAGS="$OTHER_LDFLAGS $TRY_ADD_LDFLAGS"
      CPPFLAGS="$ax_pkg_old_cppflags"
      LDFLAGS="$ax_pkg_old_ldflags"
      LIBS="$ax_pkg_old_libs"

      if test "x$HAVE_PKG_$1" = "xno" -a "x$ENABLE_VERBOSE" != "xyes"; then
        AC_MSG_RESULT([no (not found)])
      fi

    fi

  fi
  if test "${HAVE_PKG_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
    PKG_$1_CPPFLAGS=
    PKG_$1_LIBS=
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_$1],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 package is available.])

  AC_SUBST(PKG_$1_CPPFLAGS)
  AC_SUBST(PKG_$1_LIBS)
  AC_SUBST(HAVE_PKG_$1)

  if test x"$ENABLE_VERBOSE" == "xyes"; then
    AC_MSG_NOTICE([HAVE_PKG_$1 = ${HAVE_PKG_$1}])
    AC_MSG_NOTICE([PKG_$1_CPPFLAGS= $PKG_$1_CPPFLAGS])
    AC_MSG_NOTICE([PKG_$1_LIBS= $PKG_$1_LIBS])
    AC_MSG_NOTICE([CPPFLAGS= $CPPFLAGS])
    AC_MSG_NOTICE([LDFLAGS= $LDFLAGS])
    AC_MSG_NOTICE([OTHER_CPPFLAGS= $OTHER_CPPFLAGS])
    AC_MSG_NOTICE([OTHER_LDFLAGS= $OTHER_LDFLAGS])
  fi
])
