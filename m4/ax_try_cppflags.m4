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


dnl Usage: AX_TRY_CPPFLAGS(flags,[action-if-yes,[action-if-no]])
dnl
dnl The default action-if-yes is to append the flags to CPPFLAGS

AC_DEFUN([AX_TRY_CPPFLAGS],
[
    AS_VAR_PUSHDEF([ac_var], ac_cv_cppflags_[[$1]])dnl
    AC_CACHE_CHECK([whether the current compiler supports $1], ac_var,
    [
        save_CPPFLAGS="$CPPFLAGS"
        CPPFLAGS="$CPPFLAGS $1"
        AC_LINK_IFELSE([AC_LANG_PROGRAM()], [AS_VAR_SET(ac_var,yes)], [AS_VAR_SET(ac_var,no)])
        CPPFLAGS="$save_CPPFLAGS"
    ])
    AS_IF([test AS_VAR_GET(ac_var) = yes], [m4_default([$2], [CPPFLAGS="$CPPFLAGS $1"])], [$3])
    AS_VAR_POPDEF([ac_var])dnl
])
