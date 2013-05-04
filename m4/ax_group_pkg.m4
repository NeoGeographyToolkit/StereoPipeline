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

dnl AX_GROUP_PKG(<pkg> [, <dep1> .. <depn>])
dnl A stripped-down AX_PKG that copies the vars from one pkg to another without
dnl re-checking. As long as all deps were found, pkg will be marked found,
dnl too. note: no commas after the first one separating pkg from dep1!
dnl
dnl Also, to help other macros: if there are no deps, pkg is marked NOT FOUND.

AC_DEFUN([AX_GROUP_PKG],
[AC_MSG_CHECKING([for package $1])
  m4_pushdef([pkg], my_toupper([$1]))
  AS_VAR_PUSHDEF([have_pkg], [HAVE_PKG_]pkg)
  AS_VAR_PUSHDEF([pkg_c], [PKG_]pkg[_CPPFLAGS])
  AS_VAR_PUSHDEF([pkg_l], [PKG_]pkg[_LIBS])
  AS_VAR_PUSHDEF([missing], [ax_group_pkg_missing_deps])
  AS_VAR_PUSHDEF([bool], [ax_group_pkg_have_pkg_bool])

  m4_divert_once([INIT_PREPARE], [dnl
  AC_ARG_WITH(my_tolower([$1]),
    AS_HELP_STRING([--with-]my_tolower([$1]), [look for the $1 package]),
    [ have_pkg=$withval ]
  )])

  missing=""
  m4_if([$2], [], [have_pkg=[no_deps]], AX_LOAD_DEPS(pkg, my_toupper([$2]), missing))

  AS_IF([test x"$have_pkg" = "xno"],      [AS_VAR_SET([bool], 0); AC_MSG_RESULT([no (disabled by user)])],
    [AS_IF([test x"$have_pkg" = "xno_deps"], [AS_VAR_SET([bool], 0); AC_MSG_RESULT([no])],
      [AS_IF([test -z "$missing"],             [AS_VAR_SET([bool], 1); AC_MSG_RESULT([yes])],
        [bool=0; AC_MSG_RESULT([no ([missing] $missing)]) ])])])

  AS_IF( [test x"$bool" = "x1"], [have_pkg=yes], [pkg_c=""; pkg_l=""; have_pkg=no])

  AC_DEFINE_UNQUOTED(have_pkg, $bool, [Define to 1 if the pkg package is available])
  AC_SUBST(pkg_c)
  AC_SUBST(pkg_l)
  AC_SUBST(have_pkg)

  AS_VAR_POPDEF([bool])
  AS_VAR_POPDEF([missing])
  AS_VAR_POPDEF([pkg_l])
  AS_VAR_POPDEF([pkg_c])
  AS_VAR_POPDEF([have_pkg])
  m4_popdef([pkg])
])

AC_DEFUN([AX_DISABLE_PKG], [AX_GROUP_PKG([$1])])
