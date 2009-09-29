dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006, 2007 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__

# Usage: AX_APP(<name>, <directory>, <default>, <required dependencies>[, <optional dependencies>])
AC_DEFUN([AX_APP],
[
  m4_ifdef([_AX_APP_PREPARE], [],
  [
    abspath() {
        if test ${1#/} = [$]1; then
            echo "$PWD/[$]1"
        else
            echo [$]1
        fi
    }

    var_uniq() {
        echo -n "[$]*" | sed 's/ \+/\n/g' | sed -n 'G; s/\n/&&/; /^\(@<:@^\n@:>@*\n\).*\n\1/d; s/\n//; h; P' | tr '\n' ' '
    }

    get_rpath() {
        for i in "[$]@"; do
            case [$i] in
                -L*) v="`abspath ${i#-L}`"; if test -n "$v"; then echo -n " -R$v"; fi;;
            esac
        done
        echo
    }
    m4_define([_AX_APP_PREPARE], [1])
  ])

  # Silently ignore apps that don't exist in this distribution
  if test -d "$srcdir/$2" ; then

    HAVE_PKG_$1_SRC=yes

    if test -n "$ENABLE_APP_$1"; then
        WANT_APP_$1="$ENABLE_APP_$1"
    fi

    AC_DIVERT_PUSH(AX_DIVERSION_PROCESS_OPTIONS)dnl
    AC_ARG_ENABLE([app-]m4_tolower([[$1]]),
      AC_HELP_STRING([--enable-app-]m4_tolower([[$1]]), [enable the $1 app @<:@$3@:>@]),
      [ ENABLE_APP_$1=$enableval; WANT_APP_$1=$enableval; ],
      [ if test "x$ENABLE_APP_$1" = x; then ENABLE_APP_$1=`/bin/echo -n $3 | tr [A-Z] [a-z]` ; fi ]
    )
    AC_DIVERT_POP()dnl

    AC_MSG_CHECKING([whether to build app $1])
    ax_app_enable=$ENABLE_APP_$1

    # Create a variable to store missing
    AS_VAR_PUSHDEF([missing], [ax_app_]$1[_missing])

    # Load args 5 and 6 as required deps, and capture missing deps in missing var.
    # If missing is populated, bail out. Then load the optional deps
    AS_IF([test x"$ax_app_enable" != "xyes"], [AC_MSG_RESULT([no (disabled)])],
      [AX_LOAD_DEPS([$1], [$4], [missing]) # Load required deps
       AS_IF([test -n "$missing"], [AC_MSG_RESULT([no ([missing] $missing)]); ax_app_enable=no],
         [AX_LOAD_DEPS([$1], [$5]) # Load optional deps
          APP_$1_CPPFLAGS="$PKG_$1_CPPFLAGS"
          if test x"$ENABLE_RPATH" = "xyes"; then
            PKG_$1_LIBS="$PKG_$1_LIBS `var_uniq \`get_rpath ${PKG_$1_LIBS}\``"
          fi
          APP_$1_LIBS="$PKG_$1_LIBS"
          AC_MSG_RESULT([yes])])])

    AS_VAR_POPDEF([missing])

  else
    HAVE_PKG_$1_SRC=no
    ax_app_enable=no
    APP_$1_LIBS=
    PKG_$1_LIBS=
    APP_$1_CPPFLAGS=
    PKG_$1_CPPFLAGS=
  fi

  AC_SUBST(APP_$1_CPPFLAGS)
  AC_SUBST(PKG_$1_CPPFLAGS)
  AC_SUBST(APP_$1_LIBS)
  AC_SUBST(PKG_$1_LIBS)

  HAVE_PKG_$1=${ax_app_enable}
  MAKE_APP_$1=${ax_app_enable}
  AC_SUBST(MAKE_APP_$1)

  if test -n "$WANT_APP_$1"; then
      if test x"$MAKE_APP_$1" != x"$WANT_APP_$1"; then
          AC_MSG_ERROR([You said ENABLE_APP_]$1[=$WANT_APP_]$1[, but I decided $MAKE_APP_]$1)
      fi
  fi

  if test "${HAVE_PKG_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED(HAVE_PKG_$1,
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 app is available.])

  if test "$HAVE_PKG_$1_SRC" = "yes" ; then
    AX_LOG([MAKE_APP_]$1[ = $MAKE_APP_]$1)
    AX_LOG([HAVE_PKG_]$1[ = $HAVE_PKG_]$1)
    AX_LOG([APP_]$1[_CPPFLAGS = $APP_]$1[_CPPFLAGS])
    AX_LOG([PKG_]$1[_CPPFLAGS = $PKG_]$1[_CPPFLAGS])
    AX_LOG([APP_]$1[_LIBS = $APP_]$1[_LIBS])
    AX_LOG([PKG_]$1[_LIBS = $PKG_]$1[_LIBS])
  fi

  #  We're putting these in configure.ac manually by now, for
  #  backwards compatability with older versions of automake.
  #  AM_CONDITIONAL([MAKE_APP_$1], [test "$MAKE_APP_$1" = "yes"])
])
