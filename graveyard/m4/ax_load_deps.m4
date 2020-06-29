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


m4_ifdef([_AX_FIXUPS], [], [m4_include([m4/fixups.m4])])

dnl AX_LOAD_DEPS(<pkg>, <deps>[, <missing>])
dnl copies deps into the pkg CPPFLAGS/LIBS
dnl missing could be a shell var name in which to store
dnl the names of the missing deps. missing is NOT cleared

AC_DEFUN([AX_LOAD_DEPS],
 [m4_pushdef([missing], m4_default([$3], [ax_load_deps_missing_deps]))

  m4_foreach_w(dep, [$2],
   [AS_IF([test x"$HAVE_PKG_]dep[" != "xyes"], [missing="$missing dep"],
      [PKG_$1_CPPFLAGS="$PKG_]dep[_CPPFLAGS $PKG_$1_CPPFLAGS"
       PKG_$1_LIBS="$PKG_$1_LIBS $PKG_]dep[_LIBS"])
   ])

  m4_popdef([missing])
])
