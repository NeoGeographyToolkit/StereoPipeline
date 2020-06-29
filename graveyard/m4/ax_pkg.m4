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


m4_ifdef([_AX_FIXUPS], [], [m4_include([m4/fixups.m4])])

dnl Usage: AX_PKG(<name>, <dependencies>, <libraries>, <headers>[, <relative include path>, <relative lib path>, <required-functions>, <optional-deps>])
AC_DEFUN([AX_PKG],
[

  m4_divert_once([INIT_PREPARE], [dnl
  AC_ARG_WITH(my_tolower([$1]),
    AS_HELP_STRING([--with-]my_tolower([$1]), [look for the $1 package]),
    [ HAVE_PKG_$1=$withval ]
  )])

  ADD_$1_CPPFLAGS="$PKG_$1_CPPFLAGS"
  PKG_$1_CPPFLAGS=""

  ADD_$1_LDFLAGS="$PKG_$1_LDFLAGS"
  PKG_$1_LDFLAGS=""

  if test -z "$ADD_$1_CPPFLAGS"; then :; else
    AX_LOG([APPEND: ADD_]$1[_CPPFLAGS=$ADD_]$1[_CPPFLAGS])
  fi
  if test -z "$ADD_$1_LDFLAGS"; then :; else
    AX_LOG([APPEND: ADD_]$1[_LDFLAGS=$ADD_]$1[_LDFLAGS])
  fi

  m4_ifval([$7],
    [AC_MSG_CHECKING([for package $1 with functions ($7)])],
    [AC_MSG_CHECKING([for package $1])])

  AC_LANG_ASSERT(C++)

  # We can skip searching if we're already at "no"
  if test "no" = "$HAVE_PKG_$1"; then
    AC_MSG_RESULT([no (disabled by user)])
  elif test x"${HAVE_PKG_$1#no:}" != "x$HAVE_PKG_$1"; then # read as: if has_prefix(HAVE_PKG_$1, "no:")
    dnl { and } break AC_MSG_RESULT
    reason="${HAVE_PKG_$1#no:}"
    AC_MSG_RESULT([no ($reason)])
    HAVE_PKG_$1=no
  else
    if test -z "${PKG_$1_LIBS}"; then
        PKG_$1_LIBS="$3"
    else
        AX_LOG([OVERRIDE: ]$1[ libs (]$3[) with $PKG_]$1[_LIBS])
    fi

    if test ! -z "${PKG_$1_MORE_LIBS}"; then
        AX_LOG([APPEND: ]$1[ libs ($PKG_]$1[_LIBS) with $PKG_]$1[_MORE_LIBS])
        PKG_$1_LIBS="$PKG_$1_LIBS $PKG_$1_MORE_LIBS"
    fi

    AS_VAR_PUSHDEF([missing], [ax_pkg_]$1[_missing])
    AX_LOAD_DEPS([$1], [$2], [missing])
    AS_IF([test -n "$missing"], [AC_MSG_RESULT([no ([missing] $missing)]); HAVE_PKG_$1=no])
    AS_VAR_POPDEF([missing])

    dnl load some optional deps
    AX_LOAD_DEPS([$1], [$8])

    if test "x$HAVE_PKG_$1" = "xno" ; then
      # printed the result already
      :
    # We skip the search if the user has been explicit about "yes"
    elif test "x$HAVE_PKG_$1" = "xyes" ; then
      AC_MSG_RESULT([yes (using user-supplied flags)])

    # Otherwise we look for a path that contains the needed headers and libraries
    else

      if test -n "${HAVE_PKG_$1}" && test "${HAVE_PKG_$1}" != "yes" && test "${HAVE_PKG_$1}" != "no"; then
        PKG_PATHS_$1="${HAVE_PKG_$1}"
      else
        PKG_PATHS_$1="default ${PKG_PATHS}"
      fi

      HAVE_PKG_$1=no

      ax_pkg_old_libs="$LIBS"
      ax_pkg_old_cppflags="$CPPFLAGS"
      ax_pkg_old_ldflags="$LDFLAGS"

      LIBS="$PKG_$1_LIBS $LIBS"
      for path in $PKG_PATHS_$1; do
        AX_LOG([SEARCH: Checking $path for $PKG_]$1[_LIBS])

        CPPFLAGS="$PKG_$1_CPPFLAGS $ax_pkg_old_cppflags"
        LDFLAGS="$ax_pkg_old_ldflags"

        echo > conftest.h
        for header in $4 ; do
          echo "#include <$header>" >> conftest.h
        done

        TRY_ADD_CPPFLAGS="$ADD_$1_CPPFLAGS"
        TRY_ADD_LDFLAGS="$ADD_$1_LDFLAGS"

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

          m4_ifval([$5],
            [TRY_ADD_CPPFLAGS="$TRY_ADD_CPPFLAGS -I$path/${AX_INCLUDE_DIR}/]$5["],
            [TRY_ADD_CPPFLAGS="$TRY_ADD_CPPFLAGS -I$path/${AX_INCLUDE_DIR}"])

          if test -d $path/${AX_LIBDIR}; then
            m4_ifval([$6],
              [TRY_ADD_LDFLAGS="$TRY_ADD_LDFLAGS -L$path/${AX_LIBDIR}/]$6["],
              [TRY_ADD_LDFLAGS="$TRY_ADD_LDFLAGS -L$path/${AX_LIBDIR}"])
          elif test x"${AX_OTHER_LIBDIR}" != "x"; then
            m4_ifval([$6],
              [TRY_ADD_LDFLAGS="$TRY_ADD_LDFLAGS -L$path/${AX_OTHER_LIBDIR}/]$6["],
              [TRY_ADD_LDFLAGS="$TRY_ADD_LDFLAGS -L$path/${AX_OTHER_LIBDIR}"])
          fi
        fi

        CPPFLAGS="$CPPFLAGS $TRY_ADD_CPPFLAGS"
        LDFLAGS="$LDFLAGS $TRY_ADD_LDFLAGS"

        dnl check for the headers and libs. if found, keep going.
        dnl otherwise, check next path
        AC_LINK_IFELSE(
          [AC_LANG_PROGRAM([#include "conftest.h"],[])],
          [ HAVE_PKG_$1=yes ], [continue] )

        m4_ifval([$7],
            AX_CHECK_FUNCTIONS([$7], [$LDFLAGS $LIBS], [], [ HAVE_PKG_$1=no; echo "package $1 did not have function $func" >&AS_MESSAGE_LOG_FD ])
        )

        if test x"$HAVE_PKG_$1" = x"yes"; then
            AC_MSG_RESULT([yes])
            break
        fi

        TRY_ADD_CPPFLAGS=""
        TRY_ADD_LDFLAGS=""
      done

      if test "x$HAVE_PKG_$1" = "xyes" ; then
          # Append to CPPFLAGS, since that's the order we detected in
          PKG_$1_CPPFLAGS="$PKG_$1_CPPFLAGS $TRY_ADD_CPPFLAGS"
          # Prepend to LIBS, because dependencies need to be listed after all users
          PKG_$1_LIBS="$TRY_ADD_LDFLAGS $PKG_$1_LIBS"

          # But append the LDFLAGS here, so we don't break detection order
          OTHER_CPPFLAGS="$OTHER_CPPFLAGS $TRY_ADD_CPPFLAGS"
          OTHER_LDFLAGS="$OTHER_LDFLAGS $TRY_ADD_LDFLAGS"
      else
        AC_MSG_RESULT([no (not found)])
      fi

      CPPFLAGS="$ax_pkg_old_cppflags"
      LDFLAGS="$ax_pkg_old_ldflags"
      LIBS="$ax_pkg_old_libs"

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

  AX_LOG([HAVE_PKG_]$1[ = ${HAVE_PKG_]$1[}])
  AX_LOG([PKG_]$1[_CPPFLAGS= $PKG_]$1[_CPPFLAGS])
  AX_LOG([PKG_]$1[_LIBS= $PKG_]$1[_LIBS])
  AX_LOG([CPPFLAGS= $CPPFLAGS])
  AX_LOG([LDFLAGS= $LDFLAGS])
  AX_LOG([OTHER_CPPFLAGS= $OTHER_CPPFLAGS])
  AX_LOG([OTHER_LDFLAGS= $OTHER_LDFLAGS])
])
