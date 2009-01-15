dnl usage: AX_MODULE_PYTHON(<directory>, <default>)
AC_DEFUN([AX_MODULE_PYTHON],
[
  # Silently ignore modules that don't exist in this distribution
  if test -d $1 ; then

    HAVE_PKG_PYTHON_SRC=yes

    AC_ARG_ENABLE([module-python],
      AC_HELP_STRING([--enable-module-python], [enable the python bindings @<:@$2@:>@]),
      [ ENABLE_MODULE_PYTHON=$enableval ],
      [ if test x"$ENABLE_MODULE_PYTHON" = x""; then ENABLE_MODULE_PYTHON=`/bin/echo -n $2 | tr [A-Z] [a-z]` ; fi ]
    )

    AC_MSG_CHECKING([whether to build module Python])
    ax_module_enable=$ENABLE_MODULE_PYTHON

    if test "$ax_module_enable" != "yes" ; then
      AC_MSG_RESULT([no (disabled)])
    else

      ax_module_enable=no

      if test "$HAVE_PYTHON" != "yes"; then
        AC_MSG_RESULT([no (missing python)])
      elif test "$HAVE_SWIG" != "yes"; then
        AC_MSG_RESULT([no (missing swig)])
      else
        AC_MSG_RESULT([yes])

        AC_PYTHON_MODULE([numpy])

        AC_MSG_CHECKING([whether to build python bindings])

        if test "$HAVE_PYMOD_NUMPY" != "yes"; then
          AC_MSG_RESULT([no (missing numpy)])
        else
          AC_MSG_RESULT([yes])
          numpy_include=`$PYTHON -c 'import numpy; print numpy.get_include();'`
          NUMPY_CPPFLAGS="-I$numpy_include"
          ax_module_enable=yes
        fi
      fi
    fi

  else
    HAVE_PKG_PYTHON_SRC=no
    ax_module_enable=no
  fi

  AC_SUBST([NUMPY_CPPFLAGS])

  HAVE_PKG_PYTHON=${ax_module_enable}
  MAKE_MODULE_PYTHON=${ax_module_enable}
  AC_SUBST(MAKE_MODULE_PYTHON)

  if test "${HAVE_PKG_PYTHON}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi

  AC_DEFINE_UNQUOTED(HAVE_PKG_PYTHON,
                     [$ax_have_pkg_bool],
                     [Define to 1 if the PYTHON module is available.])

  if test "$ENABLE_VERBOSE" = "yes" && test "$HAVE_PKG_PYTHON_SRC" = "yes" ; then
    AC_MSG_NOTICE(MAKE_MODULE_PYTHON = ${MAKE_MODULE_PYTHON})
    AC_MSG_NOTICE(HAVE_PKG_PYTHON = ${HAVE_PKG_PYTHON})
  fi
])
