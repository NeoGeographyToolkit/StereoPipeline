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

m4_pattern_allow([^PKG_BOOST_SAME_SUFFIX])

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
# Usage: AX_PKG_BOOST_LIB(<name>, <libraries>, <header>, <function>[, <dep-libs>, <other-headers>])
AC_DEFUN([AX_PKG_BOOST_LIB],
[
  AC_MSG_CHECKING(for package BOOST_$1)

  AC_LANG_ASSERT(C++)

  # Skip testing if the user has overridden
  if test -z ${HAVE_PKG_BOOST_$1}; then

    HAVE_PKG_BOOST_$1=no

    module=no eval BOOST_LIB_SUFFIX=$shrext_cmds

    # Check for general Boost presence
    if test -n "$PKG_BOOST_INCDIR" -a -n "$PKG_BOOST_LIBDIR"; then

    m4_ifval([$2], [
        boost_name="$2"
        boost_name="${boost_name#-l}"
        # The SAME_SUFFIX macro is sticky from the previously-searched-for boost lib.
        # They should all have the same suffix.
        if test -n "${PKG_BOOST_SAME_SUFFIX}"; then
          AX_LOG([Reusing suffix ${PKG_BOOST_SAME_SUFFIX}])
          libglob="${PKG_BOOST_LIBDIR}/lib${boost_name}${PKG_BOOST_SAME_SUFFIX}"
          boost_want_suffix=
        else
          AX_LOG([No known suffix. Figuring one out.])
          libglob="${PKG_BOOST_LIBDIR}/lib${boost_name}*${BOOST_LIB_SUFFIX:-.so}*"
          # If they ask for a suffix, look for it first
          if test -n "${BOOST_LIBRARIES_SUFFIX}"; then
            boost_want_suffix="${PKG_BOOST_LIBDIR}/lib${boost_name}${BOOST_LIBRARIES_SUFFIX}${BOOST_LIB_SUFFIX:-.so}*"
          else
            boost_want_suffix=
          fi
        fi
      ])

      ax_pkg_old_cppflags="$CPPFLAGS"
      ax_pkg_old_libs="$LIBS"

      m4_ifval([$2], [
        dnl first try the requested suffix, then try in descending filename
        dnl length, descending asciibetically.  For boost, longer filenames
        dnl mean they probably have more information (maybe version?). In this
        dnl scheme, we prefer longer filenames to shorter ones, and greater
        dnl version numbers to lesser ones.
        for boost_libpath in $boost_want_suffix `for i in ${libglob}; do echo "${#i} $i"; done | sort -nr | cut -d ' ' -f 2`; do
      ], [
        for boost_libpath in boost_header_only; do
      ])

        AX_LOG([Checking to see if $boost_libpath works])
        m4_ifval([$2], [
          dnl If the glob doesn't expand, we'll get the same path back. Check for that.
          AS_IF([test ! -f ${boost_libpath}], [continue])
          dnl for libboost_library_name-blah-pants.so.4.3.5
          dnl    stem is libboost_library_name-blah-pants
          boost_lib_stem="`basename ${boost_libpath%${BOOST_LIB_SUFFIX:-.so}*}`"
          dnl post is .so.4.3.5
          boost_lib_post="${boost_libpath#*${boost_lib_stem}}"
          dnl and suffix is -blah-pants.so.4.3.5
          PKG_BOOST_SAME_SUFFIX="${boost_lib_stem#lib${boost_name}}${boost_lib_post}"
          PKG_BOOST_$1_LIBS="-L${PKG_BOOST_LIBDIR} -l${boost_lib_stem#lib} $5"

          if test ! -z "${PKG_BOOST_$1_MORE_LIBS}"; then
            AX_LOG([APPEND: BOOST_]$1[ libs ($PKG_BOOST_]$1[_LIBS) with $PKG_BOOST_]$1[_MORE_LIBS])
            PKG_BOOST_$1_LIBS="$PKG_BOOST_$1_LIBS $PKG_BOOST_$1_MORE_LIBS"
          fi

          AX_LOG([Using suffix ${PKG_BOOST_SAME_SUFFIX}])
        ])

        PKG_BOOST_$1_CPPFLAGS="-isystem${PKG_BOOST_INCDIR}"

        # Check to make sure we found a working one
        CPPFLAGS="$ax_pkg_old_cppflags $PKG_BOOST_$1_CPPFLAGS"
        LIBS="$ax_pkg_old_libs $PKG_BOOST_$1_LIBS"

        AC_LINK_IFELSE(
          [AC_LANG_PROGRAM(m4_foreach_w([header], [$6 $3], [@%:@include <header>
]), [[$4]])], [ HAVE_PKG_BOOST_$1=yes; break; ], [ continue ] )
      done

      CPPFLAGS="$ax_pkg_old_cppflags"
      LIBS="$ax_pkg_old_libs"

    fi # PKG_BOOST_INCDIR/LIBDIR set.
  fi # User override

  AC_MSG_RESULT([$HAVE_PKG_BOOST_]$1)

  if test "${HAVE_PKG_BOOST_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    PKG_BOOST_SAME_SUFFIX=
    PKG_BOOST_$1_CPPFLAGS=
    PKG_BOOST_$1_LIBS=
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
