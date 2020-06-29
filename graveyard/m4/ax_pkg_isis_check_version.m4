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


# Usage AX_PKG_ISIS_CHECK_VERSION()
# This checks for ISIS3.3.0+ style versioning
AC_DEFUN([AX_PKG_ISIS_CHECK_VERSION],
[
  if test x"$HAVE_PKG_ISIS" != "xyes"; then
    AC_MSG_ERROR([ISIS not found before hand, unable to find version now])
  fi

  echo "ISISROOT = $ISISROOT" >&AS_MESSAGE_LOG_FD
  echo "PKG_PATHS_ISIS = $PKG_PATHS_ISIS" >&AS_MESSAGE_LOG_FD

  if test -s "$ISISROOT"/version; then
    # Found ISIS 3.3.0+ style versioning
    ISIS_VERSION_PRE=`tr '\n' ' ' < $ISISROOT/version`
  elif test -s "$PKG_PATHS_ISIS"/version; then
    # Found ISIS 3.3.0+ style versioning
    ISIS_VERSION_PRE=`tr '\n' ' ' < $PKG_PATHS_ISIS/version`
  elif grep "std::string version" < "$ISISROOT"/inc/Constants.h > /dev/null; then
    # Found ISIS < 3.3.0 style versioning
    ISIS_VERSION_PRE=`grep "std::string version" < $ISISROOT/inc/Constants.h | sed 's/.*"\(.*\)".*/\1/'`
  elif grep "std::string version" < "$PKG_PATHS_ISIS"/inc/Constants.h > /dev/null; then
    # Found ISIS < 3.3.0 style versioning
    ISIS_VERSION_PRE=`grep "std::string version" < $PKG_PATHS_ISIS/inc/Constants.h | sed 's/.*"\(.*\)".*/\1/'`
  else
    AC_MSG_ERROR([Unable to detect ISIS version])
  fi

  # Trim whitespaces leading an trailing
  ISIS_VERSION=`echo $ISIS_VERSION_PRE | sed 's/^[ \t]*//;s/[ \t]*$//'`

  echo "ISIS_VERSION_PRE = \"$ISIS_VERSION_PRE\"" >&AS_MESSAGE_LOG_FD
  echo "ISIS_VERSION = \"$ISIS_VERSION\"" >&AS_MESSAGE_LOG_FD

  AC_DEFINE_UNQUOTED([ISIS_VERSION],
                     ["$ISIS_VERSION"],
                     [The version of ISIS that this software was built against.])
])