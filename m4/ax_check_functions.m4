dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl Usage: AX_CHECK_FUNCTIONS(<function-list>, <libs>, [<if-all-found>, <for-each-not-found>, <checking?>])
dnl
dnl this is like AC_CHECK_LIB, but with less side effects, and an explicit lib
dnl argument. if checking is defined, outputs the checks to user.
AC_DEFUN([AX_CHECK_FUNCTIONS],
[
  check_function_LDFLAGS="$LDFLAGS"
  check_function_LIBS="$LIBS"

  check_function_missing_libs=no
  check_function_TRY_LIBS="$2"

  LDFLAGS=""
  LIBS="$check_function_TRY_LIBS"

  for func in $1; do
    m4_ifval([$5], [AC_MSG_CHECKING([$5])])
    echo ["Checking for $func in $LIBS"] >&AS_MESSAGE_LOG_FD

    AC_LINK_IFELSE(
      [AC_LANG_CALL([],[$func])],
      [check_function_got_it=yes],
      [
        check_function_missing_libs=yes
        check_function_got_it=no
        m4_ifval([$4], [$4], [:])
        echo ["Could not find $func in $LIBS"] >&AS_MESSAGE_LOG_FD

      ]
    )

    m4_ifval([$5], [AC_MSG_RESULT([$check_function_got_it])])
  done

  m4_ifval([$3], [
      if test x"$check_function_missing_libs" = x"no"; then
        $3
      fi
  ])

  LDFLAGS="$check_function_LDFLAGS"
  LIBS="$check_function_LIBS"
])
