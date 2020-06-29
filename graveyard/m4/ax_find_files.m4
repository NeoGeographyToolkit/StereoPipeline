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


dnl Usage: AX_FIND_FILES(<filenames>, <search paths>)
dnl Looks to see if all the given filenames (relative paths) are accessible
dnl from one of the given base paths.  Returns the path or the empty string
dnl in ${ax_find_files_path}.
AC_DEFUN([AX_FIND_FILES],
[
  ax_find_files_path="";
  for path in $2; do
    ax_find_files_passed=yes
    for filename in $1; do
      pathname="$path/$filename"
      AX_LOG([Find Files: Looking for ${filename} in ${path}])
      ax_find_files_paths=`ls $pathname 2>/dev/null`
      if test ! -z "$ax_find_files_paths" ; then
        AX_LOG([Find Files: Found ${ax_find_files_paths}])
      else
        AX_LOG([Find Files: Not Found])
        ax_find_files_passed=no
        break
      fi
    done
    if test "$ax_find_files_passed" = "yes"; then
      ax_find_files_path="$path"
      break
    fi
  done
])
