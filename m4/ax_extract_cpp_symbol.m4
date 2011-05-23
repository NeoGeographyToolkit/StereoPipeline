dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl Usage: AX_EXTRACT_CPP_SYMBOL([symbol], [headers], [ifyes], [ifno], [cppflags])
dnl         It the variable $output will contain the extracted value
dnl example: AX_EXTRACT_CPP_SYMBOL([BOOST_VERSION], [#include <boost/version.hpp>], [BOOST_VERSION=$output])
AC_DEFUN([AX_EXTRACT_CPP_SYMBOL],
[
    AC_REQUIRE_CPP()dnl
    AC_CHECK_PROGS([SED], [sed gsed])
AC_LANG_CONFTEST(
    [AC_LANG_SOURCE([$2
#define __ac_extract_cpp_symbol_delimiter "__ac_extract_cpp_symbol_delimiter"
__ac_extract_cpp_symbol_delimiter $1 __ac_extract_cpp_symbol_delimiter])])

old_CPPFLAGS="$CPPFLAGS"
CPPFLAGS="$5 $CPPFLAGS"
AS_VAR_PUSHDEF([output], [ax_extract_cpp_symbol_$1])dnl
if (eval "$ac_cpp conftest.$ac_ext") >conftest.out 2>&AS_MESSAGE_LOG_FD; then
    output="`${SED} -n -e 's/^.*"__ac_extract_cpp_symbol_delimiter" \(.*\) "__ac_extract_cpp_symbol_delimiter".*$/\1/p' conftest.out 2>/dev/null`"
    if test x"${output}" != x"$1"; then
        ifelse([$3], , :, [$3])
    ifelse([$4], , , [else
    $4
])
    fi
    ifelse([$4], , , [else
    $4
])dnl
fi
CPPFLAGS="$old_CPPFLAGS"
AS_VAR_POPDEF([output])
rm -rf conftest*
])
