dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl Usage: AX_TRY_CPPFLAGS(flags,[action-if-yes,[action-if-no]])
dnl
dnl The default action-if-yes is to append the flags to CPPFLAGS

AC_DEFUN([AX_TRY_CPPFLAGS],
[
    AS_VAR_PUSHDEF([ac_var], ac_cv_cppflags_[[$1]])dnl
    AC_CACHE_CHECK([whether the current compiler supports $1], ac_var,
    [
        save_CPPFLAGS="$CPPFLAGS"
        CPPFLAGS="$CPPFLAGS $1"
        AC_LINK_IFELSE([AC_LANG_PROGRAM()], [AS_VAR_SET(ac_var,yes)], [AS_VAR_SET(ac_var,no)])
        CPPFLAGS="$save_CPPFLAGS"
    ])
    AS_IF([test AS_VAR_GET(ac_var) = yes], [m4_default([$2], [CPPFLAGS="$CPPFLAGS $1"])], [$3])
    AS_VAR_POPDEF([ac_var])dnl
])
