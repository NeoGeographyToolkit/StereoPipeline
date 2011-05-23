dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


# Usage: AX_PKG_BOOST_CHECK_VERSION([config-prefix])
# config-prefix should be the same as was used above
AC_DEFUN([AX_PKG_BOOST_CHECK_VERSION],
[
  if test -z "$BOOST_VERSION"; then
    AC_MSG_ERROR([Boost version check program did not define BOOST_VERSION])
  fi

  AC_DEFINE_UNQUOTED([BOOST_VERSION],
                     [$BOOST_VERSION],
                     [The version of Boost with which the software built.])

  AH_VERBATIM([_OTHER_CHECK_BOOST_VERSION],
[// Check to make sure the user is using the same version of Boost
// headers that the software was built with.
#include <boost/version.hpp>
#if BOOST_VERSION != $1BOOST_VERSION
#error You are using a different version of Boost than you used to build $1!
#endif
])
])
