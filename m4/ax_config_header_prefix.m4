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


dnl Usage: AX_CONFIG_HEADER_PREFIX(<filename>, <prefix>)
dnl Generates a configuration header file, adding a prefix to all symbols.
dnl This is a two-step process.  First we generate the usual header file
dnl with a filename ending in ".pre".  Then we process that file, adding
dnl the prefix to all symbolx, and copy it into the final location if it
dnl has changed.
AC_DEFUN([AX_CONFIG_HEADER_PREFIX],
[
  AC_CONFIG_HEADERS([$1.pre],
  [
    echo "/* $1.  Generated from $1.pre by config.status.  */" > "$1.new"
    echo "#ifndef __$2_CONFIG_H__" >> "$1.new"
    echo "#define __$2_CONFIG_H__" >> "$1.new"
    sed -e 's/#define /#define $2/' -e 's/#undef /#undef $2/' < "$1.pre" >> "$1.new"
    if git show > /dev/null; then
       echo >> "$1.new"
       echo "/* Commit ID of this build */" >> "$1.new"
       echo "#define $2COMMIT_ID \""`git show --abbrev-commit | head -1 | sed 's/commit //'`"\"" >> "$1.new"
       echo >> "$1.new"
    fi

    echo "#endif // __$2_CONFIG_H__" >> "$1.new"
    if test -f "$1" && diff "$1" "$1.new" > /dev/null ; then
      echo "config.status: $1 is unchanged"
    else
      echo "config.status: creating $1"
      cp "$1.new" "$1"
    fi
    rm -f "$1.new"
  ])
])
