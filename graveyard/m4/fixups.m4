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


dnl caller must protect themselves from multiple inclusion
dnl m4_ifdef([_AX_FIXUPS], [], [m4_exclude([fixups.m4])])
dnl ^--- this code isn't quite right; the correct code is "include", not "exclude".
dnl automake 1.9.? will break on includes inside comments.

dnl 2.65 added an additional level of quoting for upper and lower
m4_pushdef([AAA],[---])dnl
m4_if(m4_tolower([aaa]), [---],
 [m4_define([my_tolower], [m4_tolower([[$1]])])],
 [m4_copy([m4_tolower], [my_tolower])])
m4_popdef([AAA])

m4_pushdef([AAA],[---])dnl
m4_if(m4_toupper([aaa]), [---],
 [m4_define([my_toupper], [m4_toupper([[$1]])])],
 [m4_copy([m4_toupper], [my_toupper])])
m4_popdef([AAA])

dnl this was introduced in autoconf 2.6, i think.
m4_ifset([m4_foreach_w], [],
[m4_define([m4_foreach_w],[m4_foreach([$1], m4_split(m4_normalize([$2])), [$3])])])

m4_ifset([m4_argn], [],
[m4_define([m4_argn],
[m4_car(m4_shiftn($1, $@))])])

m4_ifset([AS_ECHO_N], [m4_copy([AS_ECHO_N], [ASP_ECHO_N])],
[m4_define([ASP_ECHO_N],
  [AS_REQUIRE([_AS_ECHO_N_PREPARE])dnl
  echo $ECHO_N $1"$ECHO_C"])])

m4_define([_AX_FIXUPS], [1])
