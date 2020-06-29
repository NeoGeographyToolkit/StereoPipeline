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

# Usage: AX_MODULE(<name>, <directory>, <library>, <default>, <prerequisites>, <required dependencies>[, <optional dependencies>])
AC_DEFUN([AX_MODULE],
[
  m4_ifdef([_AX_MODULE_PREPARE], [],
  [
    abspath() {
        if test ${1#/} = [$]1; then
            echo "$PWD/[$]1"
        else
            echo [$]1
        fi
    }

    var_uniq() {
        ASP_ECHO_N(["[$]*"]) | sed 's/ \+/\n/g' | sed -n 'G; s/\n/&&/; /^\(@<:@^\n@:>@*\n\).*\n\1/d; s/\n//; h; P' | tr '\n' ' '
    }

    get_rpath() {
        for i in "[$]@"; do
            case [$i] in
                -L*)
                v="`abspath ${i#-L}`";
                if test -n "$v"; then
                   ASP_ECHO_N([" -R$v"])
                fi
                ;;
            esac
        done
        echo
    }
    m4_define([_AX_MODULE_PREPARE], [1])
  ])

  # Silently ignore modules that don't exist in this distribution
  if test -d "$srcdir/$2" ; then

    HAVE_PKG_$1_SRC=yes

    m4_divert_once([INIT_PREPARE], [dnl
      # Silently ignore modules that don't exist in this distribution
      # I'm diverting the output, so i need to do this twice.
      if test -d "$srcdir/$2" ; then

        if test -n "$ENABLE_MODULE_$1"; then
            WANT_MODULE_$1="$ENABLE_MODULE_$1"
        fi
      fi

      AC_ARG_ENABLE([module-]my_tolower([$1]),
        AS_HELP_STRING([--enable-module-]my_tolower([$1]), [enable the $1 module @<:@$4@:>@]),
        [ ENABLE_MODULE_$1=$enableval; WANT_MODULE_$1=$enableval; ],
        [ if test x"$ENABLE_MODULE_$1" = x; then ENABLE_MODULE_$1=`ASP_ECHO_N([$4]) | tr [A-Z] [a-z]` ; fi ]
      )])

    AC_MSG_CHECKING([whether to build module $1])
    ax_module_enable=$ENABLE_MODULE_$1

    # Create a variable to store missing
    AS_VAR_PUSHDEF([missing], [ax_module_]$1[_missing])

    # Load args 5 and 6 as required deps, and capture missing deps in missing var.
    # If missing is populated, bail out. Then load the optional deps
    AS_IF([test x"$ax_module_enable" != "xyes"], [AC_MSG_RESULT([no (disabled)])],
      [AX_LOAD_DEPS([$1], [$5 $6], [missing]) # Load required deps
       AS_IF([test -n "$missing"], [AC_MSG_RESULT([no ([missing] $missing)]); ax_module_enable=no],
         [AX_LOAD_DEPS([$1], [$7]) # Load optional deps
          MODULE_$1_CPPFLAGS="$PKG_$1_CPPFLAGS"
          if test x"$ENABLE_RPATH" = "xyes"; then
            PKG_$1_LIBS="$PKG_$1_LIBS `var_uniq \`get_rpath ${PKG_$1_LIBS}\``"
          fi
          MODULE_$1_LIBS="$PKG_$1_LIBS"
          m4_ifval([$3], [PKG_$1_LIBS="\$(top_builddir)/$2/$3 $PKG_$1_LIBS"])
          AC_MSG_RESULT([yes])])])

    AS_VAR_POPDEF([missing])

  else
    HAVE_PKG_$1_SRC=no
    ax_module_enable=no
    MODULE_$1_LIBS=
    PKG_$1_LIBS=
    MODULE_$1_CPPFLAGS=
    PKG_$1_CPPFLAGS=
  fi

  AC_SUBST(MODULE_$1_CPPFLAGS)
  AC_SUBST(PKG_$1_CPPFLAGS)
  AC_SUBST(MODULE_$1_LIBS)
  AC_SUBST(PKG_$1_LIBS)

  HAVE_PKG_$1=${ax_module_enable}
  MAKE_MODULE_$1=${ax_module_enable}
  AC_SUBST(MAKE_MODULE_$1)

  if test -n "$WANT_MODULE_$1"; then
      if test x"$MAKE_MODULE_$1" != x"$WANT_MODULE_$1"; then
          AC_MSG_ERROR([You said ENABLE_MODULE_]$1[=$WANT_MODULE_]$1[, but I decided $MAKE_MODULE_]$1)
      fi
  fi

  if test "${HAVE_PKG_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED(HAVE_PKG_$1,
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 module is available.])

  if test "$HAVE_PKG_$1_SRC" = "yes" ; then
    AX_LOG([MAKE_MODULE_]$1[ = $MAKE_MODULE_]$1)
    AX_LOG([HAVE_PKG_]$1[ = $HAVE_PKG_]$1)
    AX_LOG([MODULE_]$1[_CPPFLAGS = $MODULE_]$1[_CPPFLAGS])
    AX_LOG([PKG_]$1[_CPPFLAGS = $PKG_]$1[_CPPFLAGS])
    AX_LOG([MODULE_]$1[_LIBS = $MODULE_]$1[_LIBS])
    AX_LOG([PKG_]$1[_LIBS = $PKG_]$1[_LIBS])
  fi

  #  We're putting these in configure.ac manually by now, for
  #  backwards compatability with older versions of automake.
  #  AM_CONDITIONAL([MAKE_MODULE_$1], [test "$MAKE_MODULE_$1" = "yes"])
])
