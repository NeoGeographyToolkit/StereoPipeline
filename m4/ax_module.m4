# Usage: AX_MODULE(<name>, <directory>, <library>, <default>, <prerequisites>, <required dependencies>[, <optional dependencies>])
AC_DEFUN([AX_MODULE],
[
  # Silently ignore modules that don't exist in this distribution
  if test -d $2 ; then

    HAVE_PKG_$1_SRC=yes

    AC_ARG_ENABLE([module-]m4_tolower([[$1]]),
      AC_HELP_STRING([--enable-module-]m4_tolower([[$1]]), [enable the $1 module @<:@$4@:>@]),
      [ ENABLE_MODULE_$1=$enableval ],
      [ if test x"$ENABLE_MODULE_$1" = x; then ENABLE_MODULE_$1=`/bin/echo -n $4 | tr [A-Z] [a-z]` ; fi ]
    )

    AC_MSG_CHECKING([whether to build module $1])
    ax_module_enable=$ENABLE_MODULE_$1

    if test "$ax_module_enable" != "yes" ; then
      AC_MSG_RESULT([no (disabled)])
    fi

    ax_libs=""

    # Check for prerequisites
    if test "$ax_module_enable" = "yes" ; then
      for ax_dependency in $5 ; do
        ax_dependency_have="HAVE_PKG_${ax_dependency}"
        if test x"${!ax_dependency_have}" = "xyes"; then
          ax_dep_libs="PKG_${ax_dependency}_LIBS"
          ax_libs="${ax_libs} ${!ax_dep_libs}"
        else
          AC_MSG_RESULT([no])
          AC_MSG_NOTICE([warning: unable to build requested module $1 (needs ${ax_dependency})!])
          ax_module_enable=no;
          break;
        fi
      done
    fi

    # Check for required dependencies
    if test "$ax_module_enable" = "yes" ; then
      for ax_dependency in $6 ; do
        ax_dependency_have="HAVE_PKG_${ax_dependency}"
        if test x"${!ax_dependency_have}" = "xyes"; then
          ax_dep_libs="PKG_${ax_dependency}_LIBS"
          ax_libs="${ax_libs} ${!ax_dep_libs}"
        else
          AC_MSG_RESULT([no])
          AC_MSG_NOTICE([warning: unable to build requested module $1 (needs ${ax_dependency})!])
          ax_module_enable=no;
          break;
        fi
      done
    fi

    if test "$ax_module_enable" = "yes" ; then
      # Check for optional dependencies
      for ax_dependency in $7 ; do
        ax_dependency_have="HAVE_PKG_${ax_dependency}"
        if test x"${!ax_dependency_have}" = "xyes"; then
          ax_dep_libs="PKG_${ax_dependency}_LIBS"
          ax_libs="${ax_libs} ${!ax_dep_libs}"
        fi
      done

      # Set up the variables
      MODULE_$1_LIBS=$ax_libs

      if test -z "$3"; then
        PKG_$1_LIBS="$ax_libs"
      else
        PKG_$1_LIBS="$ax_libs \$(top_srcdir)/$2/$3"
      fi

      AC_MSG_RESULT([yes])
    fi

  else
    HAVE_PKG_$1_SRC=no
    ax_module_enable=no
    MODULE_$1_LIBS=
    PKG_$1_LIBS=
  fi

  AC_SUBST(MODULE_$1_LIBS)
  AC_SUBST(PKG_$1_LIBS)

  HAVE_PKG_$1=${ax_module_enable}
  MAKE_MODULE_$1=${ax_module_enable}
  AC_SUBST(MAKE_MODULE_$1)

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
    AX_LOG([MODULE_]$1[_LIBS = $MODULE_]$1[_LIBS])
    AX_LOG([PKG_]$1[_LIBS = $PKG_]$1[_LIBS])
  fi

  #  We're putting these in configure.ac manually by now, for
  #  backwards compatability with older versions of automake.
  #  AM_CONDITIONAL([MAKE_MODULE_$1], [test "$MAKE_MODULE_$1" = "yes"])
])
