# This checks for various introspection functions
AC_DEFUN([AX_CHECK_INTROSPECTION],
[
    AC_CHECK_HEADERS(execinfo.h cxxabi.h typeinfo dlfcn.h)

    AC_CHECK_FUNCS([__cxa_current_exception_type __cxa_demangle backtrace])

    old_LIBS="$LIBS"
    LIBS=""
    AC_SEARCH_LIBS(dladdr, dl, [AC_DEFINE(HAVE_DLADDR,1,Define if you have dladdr())])
    LDFLAGS="$LDFLAGS $LIBS"
    LIBS="$old_LIBS"
])
