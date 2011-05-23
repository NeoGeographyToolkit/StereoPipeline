dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl Usage: AX_CHECK_FUNCTION_ATTRIBUTE([attr])

AC_DEFUN([AX_CHECK_FUNCTION_ATTRIBUTE],
[
    AS_VAR_PUSHDEF([ac_var], ac_cv_func_attribute_[[$1]])dnl
    AC_CACHE_CHECK([whether the current compiler supports $1], ac_var,
    [
        AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
          int f(int i) __attribute__(($1));
        ]], [])], [AS_VAR_SET(ac_var,yes)], [AS_VAR_SET(ac_var,no)])
    ])
    AS_IF([test x"$ac_var" = "xyes"],
      [AC_DEFINE([COMPILER_HAS_ATTRIBUTE_]AS_TR_CPP($1), [1], [does the compiler support function __attribute__(($1))?])],
      [AC_DEFINE([COMPILER_HAS_ATTRIBUTE_]AS_TR_CPP($1), [0], [does the compiler support function __attribute__(($1))?])])
    AS_VAR_POPDEF([ac_var])dnl
])
