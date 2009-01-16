# Usage: AX_APP(<name>, <directory>, <default>, <required dependencies>[, <optional dependencies>])
AC_DEFUN([AX_APP],
[
  # Silently ignore modules that don't exist in this distribution
  if test -d $2 ; then

    HAVE_PKG_$1_SRC=yes

    AC_ARG_ENABLE([app-]m4_tolower([[$1]]),
      AC_HELP_STRING([--enable-app-]m4_tolower([[$1]]), [enable the $1 app @<:@$3@:>@]),
      [ ENABLE_APP_$1=$enableval ],
      [ if test "x$ENABLE_APP_$1" = x; then ENABLE_APP_$1=`/bin/echo -n $3 | tr [A-Z] [a-z]` ; fi ]
    )

    AC_MSG_CHECKING([whether to build app $1])
    ax_app_enable=$ENABLE_APP_$1

    if test "$ax_app_enable" != "yes" ; then
      AC_MSG_RESULT([no (disabled)])
    fi

    ax_libs=""

    # Check for required dependencies
    if test "$ax_app_enable" = "yes" ; then
      for ax_dependency in $4 ; do
        ax_dependency_have="HAVE_PKG_${ax_dependency}"
        if test x"${!ax_dependency_have}" = "xyes"; then
          ax_dep_libs="PKG_${ax_dependency}_LIBS"
          ax_libs="${ax_libs} ${!ax_dep_libs}"
        else
          AC_MSG_RESULT([no])
          AC_MSG_NOTICE([warning: unable to build requested app $1 (needs ${ax_dependency})!])
          ax_app_enable=no;
          break;
        fi
      done
    fi

    if test "$ax_app_enable" = "yes" ; then
      # Check for optional dependencies
      for ax_dependency in $5 ; do
        ax_dependency_have="HAVE_PKG_${ax_dependency}"
        if test x"${!ax_dependency_have}" = "xyes"; then
          ax_dep_libs="PKG_${ax_dependency}_LIBS"
          ax_libs="${ax_libs} ${!ax_dep_libs}"
        fi
      done

      # Set up the variables
      APP_$1_LIBS=$ax_libs
      PKG_$1_LIBS=$ax_libs
      AC_MSG_RESULT([yes])
    fi

  else
    HAVE_PKG_$1_SRC=no
    ax_app_enable=no
    APP_$1_LIBS=
    PKG_$1_LIBS=
  fi

  AC_SUBST(APP_$1_LIBS)
  AC_SUBST(PKG_$1_LIBS)

  HAVE_PKG_$1=${ax_app_enable}
  MAKE_APP_$1=${ax_app_enable}
  AC_SUBST(MAKE_APP_$1)

  if test "${HAVE_PKG_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED(HAVE_PKG_$1,
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 app is available.])

  if test "$ENABLE_VERBOSE" = "yes" && test "$HAVE_PKG_$1_SRC" = "yes" ; then
    AC_MSG_NOTICE(MAKE_APP_$1 = ${MAKE_APP_$1})
    AC_MSG_NOTICE(HAVE_PKG_$1 = ${HAVE_PKG_$1})
    AC_MSG_NOTICE(APP_$1_LIBS = ${APP_$1_LIBS})
    AC_MSG_NOTICE(PKG_$1_LIBS = ${PKG_$1_LIBS})
  fi

  AM_CONDITIONAL([MAKE_APP_$1], [test "$MAKE_APP_$1" = "yes"])
])
