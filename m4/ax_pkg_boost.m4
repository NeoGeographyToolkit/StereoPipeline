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

# Usage: AX_PKG_BOOST(<required boost libs> [,boost_lib, how-to-detect-lib]*)
AC_DEFUN([AX_PKG_BOOST],
[
  m4_divert_once([INIT_PREPARE], [dnl
  AC_ARG_WITH(boost,
    AS_HELP_STRING([--with-boost], [look for the boost package]),
    [ HAVE_PKG_BOOST=$withval ]
  )])

  AC_LANG_ASSERT(C++)

  if test -n "${HAVE_PKG_BOOST}" && test "${HAVE_PKG_BOOST}" != "yes" && test "${HAVE_PKG_BOOST}" != "no" && test x"${HAVE_PKG_BOOST#no:}" = "x$HAVE_PKG_BOOST"; then
    PKG_PATHS_BOOST="${HAVE_PKG_BOOST}"
  else
    PKG_PATHS_BOOST="${PKG_PATHS}"
  fi

  # Skip testing if the user has overridden
  if test "no" = "$HAVE_PKG_BOOST"; then
    HAVE_PKG_BOOST="no:disabled by user"
  elif test x"${HAVE_PKG_BOOST#no:}" != "x$HAVE_PKG_BOOST"; then # read as: if has_prefix(HAVE_PKG_BOOST, "no:")
    :
  else

    HAVE_PKG_BOOST=no

    for ax_boost_base_path in $PKG_PATHS_BOOST; do
      boost_base_path_glob="${ax_boost_base_path}/include/boost-*"
      for ax_boost_inc_path in ${ax_boost_base_path}/include `echo ${boost_base_path_glob} | xargs -n1 | sort -r` ; do
        AX_LOG([Checking for a boost in ${ax_boost_inc_path}])
        if test -f "${ax_boost_inc_path}/boost/version.hpp"; then
          AC_MSG_NOTICE([Checking if Boost at ${ax_boost_inc_path}/boost is OK])

          AX_EXTRACT_CPP_SYMBOL([BOOST_VERSION], [#include <boost/version.hpp>], [BOOST_VERSION=$output],
            [AX_LOG([Couldn't get boost version for $ax_boost_inc_path/boost]); continue;],
            ["-I${ax_boost_inc_path}"])

          AX_LOG([Found boost includes at ${ax_boost_inc_path}, version(${BOOST_VERSION})])

          PKG_BOOST_INCDIR="${ax_boost_inc_path}"
          PKG_BOOST_LIBDIR="${ax_boost_base_path}/${AX_LIBDIR}"

          # In case it's not in lib64 despite specifying lib64...
          if test ! -d $PKG_BOOST_LIBDIR -a x"${AX_OTHER_LIBDIR}" != "x"; then
            PKG_BOOST_LIBDIR="${ax_boost_base_path}/${AX_OTHER_LIBDIR}"
          fi

          m4_for([idx], 3, $#, 2, [m4_do(m4_argn(idx, $@)) ])

          missing=
          AC_FOREACH([required], $1,
          [AS_IF([test x"$HAVE_PKG_]required[" != "xyes"], [missing="${missing} required"])
          ])

          AS_IF([test -n "$missing"], [AC_MSG_NOTICE([${ax_boost_inc_path}/boost is missing these required libraries: $missing]); continue;],
          [
          HAVE_PKG_BOOST="yes"
          break 2
          ])
        fi
      done
    done

  fi

  AC_MSG_CHECKING(for package BOOST)

  if test "${HAVE_PKG_BOOST}" = "yes" ; then
    ax_have_pkg_bool=1
    OTHER_CPPFLAGS="${OTHER_CPPFLAGS} -isystem${PKG_BOOST_INCDIR}"
    OTHER_LDFLAGS="${OTHER_LDFLAGS} -L${PKG_BOOST_LIBDIR}"
    AC_MSG_RESULT([yes])
  elif test x"${HAVE_PKG_BOOST#no:}" != "x$HAVE_PKG_BOOST"; then # read as: if has_prefix(HAVE_PKG_BOOST, "no:")
    dnl { and } break AC_MSG_RESULT
    reason="${HAVE_PKG_BOOST#no:}"
    AC_MSG_RESULT([no ($reason)])
  else
    HAVE_PKG_BOOST=no
    ax_have_pkg_bool=0
    AC_MSG_RESULT([no])
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
