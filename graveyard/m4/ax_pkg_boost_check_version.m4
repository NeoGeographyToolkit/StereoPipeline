dnl __BEGIN_LICENSE__
dnl  Copyright (c) 2009-2013, United States Government as represented by the
dnl  Administrator of the National Aeronautics and Space Administration. All
dnl  rights reserved.
dnl
dnl  The NGT platform is licensed under the Apache License, Version 2.0 (the
dnl  "License"); you may not use this file except in compliance with the
dnl  License. You may obtain a copy of the License at
dnl  http://www.apache.org/licenses/LICENSE-2.0
dnl
dnl  Unless required by applicable law or agreed to in writing, software
dnl  distributed under the License is distributed on an "AS IS" BASIS,
dnl  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
dnl  See the License for the specific language governing permissions and
dnl  limitations under the License.
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
