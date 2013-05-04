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


dnl Usage: AX_CHECK_FUNCTION_ATTRIBUTE([attr])

AC_DEFUN([AX_CHECK_FUNCTION_ATTRIBUTE],
[
    AS_VAR_PUSHDEF([ac_var], ac_cv_func_attribute_[[$1]])dnl
    AC_CACHE_CHECK([whether the current compiler supports $1], ac_var,
    [
        AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
          int f(int i) __attribute__(($1));
        ]], [])], [AS_VAR_SET(ac_var,yes)], [AS_VAR_SET(ac_var,no)])
    ])
    AS_IF([test x"$ac_var" = "xyes"],
      [AC_DEFINE([COMPILER_HAS_ATTRIBUTE_]AS_TR_CPP($1), [1], [does the compiler support function __attribute__(($1))?])],
      [AC_DEFINE([COMPILER_HAS_ATTRIBUTE_]AS_TR_CPP($1), [0], [does the compiler support function __attribute__(($1))?])])
    AS_VAR_POPDEF([ac_var])dnl
])
