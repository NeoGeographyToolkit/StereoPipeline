dnl Usage: AX_PKG(<name>, <dependencies>, <libraries>, <headers>[, <additional cxxflags>])
AC_DEFUN([AX_PKG],
[
  AC_ARG_WITH(translit($1,`A-Z',`a-z'),
    AC_HELP_STRING([--with-]translit($1,`A-Z',`a-z'), [enable searching for the $1 package @<:@auto@:>@]),
    [ HAVE_PKG_$1=$withval ]
  )

 AC_ARG_WITH(translit($1,`A-Z',`a-z')[-cflags],
   AC_HELP_STRING([--with-]translit($1,`A-Z',`a-z')[-cppflags], [Add to $1_CPPFLAGS @<:@auto@:>@]),
   [ ADD_$1_CPPFLAGS=$withval ])
 AC_ARG_WITH(translit($1,`A-Z',`a-z')[-libs],
   AC_HELP_STRING([--with-]translit($1,`A-Z',`a-z')[-libs], [Override $1_LIBS and skip search @<:@auto@:>@]),
   [ FORCE_$1_LDFLAGS=$withval ])

  if test x"$ENABLE_VERBOSE" = "xyes"; then
    AC_MSG_CHECKING([for package $1 in current paths])
  else
    AC_MSG_CHECKING([for package $1])
  fi

  AC_LANG_ASSERT(C++)

  # We can skip searching if we're already at "no"
  if test "no" = "$HAVE_PKG_$1"; then
    AC_MSG_RESULT([no (disabled by user)])

  else
    # Test for and inherit libraries from dependencies
    if test -z "${FORCE_$1_LDFLAGS}"; then
        PKG_$1_LIBS="$3"
    else
        PKG_$1_LIBS="${FORCE_$1_LDFLAGS}"
    fi

    for x in $2; do
      ax_pkg_have_dep=HAVE_PKG_${x}
      if test "${!ax_pkg_have_dep}" = "yes"; then
        ax_pkg_dep_libs="PKG_${x}_LIBS"
        PKG_$1_LIBS="$PKG_$1_LIBS ${!ax_pkg_dep_libs}"
        unset ax_pkg_dep_libs
      else
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

      if test -n "${FORCE_$1_LDFLAGS}"; then
        PKG_PATHS_$1=""
      elif test -n "${HAVE_PKG_$1}" && test "${HAVE_PKG_$1}" != "yes" && test "${HAVE_PKG_$1}" != "no"; then
        PKG_PATHS_$1=${HAVE_PKG_$1}
      else
        PKG_PATHS_$1=${PKG_PATHS}
      fi

      HAVE_PKG_$1=no

      # This is a gross hack that causes the AC_LINK_IFELSE macro use libtool to 
      # link files rather that g++ alone.  This in important for detecting 
      # packages like the Vision Workbench which has many dependencies that 
      # themselves have *.la files.
      OLD_CXX=$CXX
      if test "$host_vendor" = apple; then
        # Apple has lazy link-time dependencies and a different name for libtool,
        # so we turn off this hack on the mac platform.
        CXX=$CXX
      else
        CXX="libtool --mode=link --tag CXX $CXX"
      fi

      ax_pkg_old_libs=$LIBS
      LIBS="$PKG_$1_LIBS $LIBS"
      for path in none $PKG_PATHS_$1; do
        ax_pkg_old_cppflags=$CPPFLAGS
        ax_pkg_old_ldflags=$LDFLAGS
        ax_pkg_old_vw_cppflags=$ASP_CPPFLAGS
        ax_pkg_old_vw_ldflags=$ASP_LDFLAGS
        echo > conftest.h
        for header in $4 ; do
          echo "#include <$header>" >> conftest.h
        done
        CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
        LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
        if test "$path" != "none"; then
          if test x"$ENABLE_VERBOSE" = "xyes"; then
            AC_MSG_CHECKING([for package $1 in $path])
          fi

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
            ASP_CPPFLAGS="-I$path/${AX_INCLUDE_DIR} $ASP_CPPFLAGS"
          else
            ASP_CPPFLAGS="$ADD_$1_CPPFLAGS $5 $ASP_CPPFLAGS"
          fi
          CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
          AC_LINK_IFELSE(
            AC_LANG_PROGRAM([#include "conftest.h"],[]),
            [ HAVE_PKG_$1=yes ; AC_MSG_RESULT([yes]) ; break ] )
          ASP_LDFLAGS="-L$path/lib $ASP_LDFLAGS"
          LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
        fi
        AC_LINK_IFELSE(
          AC_LANG_PROGRAM([#include "conftest.h"],[]),
          [ HAVE_PKG_$1=yes ; AC_MSG_RESULT([yes]) ; break ] )
        if test x"$ENABLE_VERBOSE" = "xyes"; then
          AC_MSG_RESULT([no])
        fi
        CPPFLAGS=$ax_pkg_old_cppflags
        LDFLAGS=$ax_pkg_old_ldflags
        ASP_CPPFLAGS=$ax_pkg_old_vw_cppflags
        ASP_LDFLAGS=$ax_pkg_old_vw_ldflags
      done
      CPPFLAGS=$ax_pkg_old_cppflags
      LDFLAGS=$ax_pkg_old_ldflags
      LIBS=$ax_pkg_old_libs

      if test "x$HAVE_PKG_$1" = "xno" -a "x$ENABLE_VERBOSE" != "xyes"; then
        AC_MSG_RESULT([no (not found)])
      fi

      CXX=$OLD_CXX

    fi

  fi
  if test "${HAVE_PKG_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
    PKG_$1_LIBS=
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_$1],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 package is available.])

  AC_SUBST(PKG_$1_LIBS)
  AC_SUBST(HAVE_PKG_$1)

  if test x"$ENABLE_VERBOSE" == "xyes"; then
    AC_MSG_NOTICE([HAVE_PKG_$1 = ${HAVE_PKG_$1}])
    AC_MSG_NOTICE([PKG_$1_LIBS= $PKG_$1_LIBS])
    AC_MSG_NOTICE([ASP_CPPFLAGS= $ASP_CPPFLAGS])
    AC_MSG_NOTICE([ASP_LDFLAGS= $ASP_LDFLAGS])
  fi
])
