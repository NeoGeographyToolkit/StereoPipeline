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

dnl AX_ARG_(ENABLE|WITH)(<arg>, <default>, <tags>, <description>)
dnl possible tags are:
dnl    always -> set autoconf macro       to value
dnl        mk -> set Makefile variable    to value
dnl   cpp-int -> set config.h value       to value
dnl  cpp-bool -> set config.h value       to int(bool(value == yes))
dnl    am-yes -> set automake conditional to bool(value == yes)
dnl    am-set -> set automake conditional to bool(-n value)
dnl
dnl variable in question is ether ENABLE_ARG or ARG (the "with" is dropped)

dnl AX_ARG(<type>, <arg>, <default>, <tags>, <description>)
AC_DEFUN([AX_ARG],
[
  m4_pushdef([lower], my_tolower([$2]))dnl lower-case arg name
  m4_pushdef([type],  my_toupper([$1]))dnl WITH or ENABLE
  AS_VAR_PUSHDEF([value], m4_if(type, [WITH], [], type[_])[]m4_toupper([[$2]]))dnl


  m4_divert_once([INIT_PREPARE], [dnl
  dnl switch between AC_ARG_(WITH|ENABLE), since I don't think you can indirect
  m4_if(type, [WITH],
    dnl WITH case.
    [AC_ARG_WITH(lower,
    AS_HELP_STRING([--with]-lower, [$5][ ]m4_ifval([$3],[@<:@$3@:>@])), dnl append default value to the desc
    [ value=[$withval] ],                                dnl if passed in, set value
    [ AS_VAR_SET_IF([value], [], [value="[$3]"]) ])],    dnl else set to default if not already set

    dnl ENABLE case. same as above except if default is yes, change enable -> disable
    [AC_ARG_ENABLE(lower,
    AS_HELP_STRING([--][]m4_if(yes, [$3], disable, enable)-lower, [$5][ ]m4_ifval([$3],[@<:@$3@:>@])),
    [ value=[$enableval] ],
    [ AS_VAR_SET_IF([value], [], [value="[$3]"]) ])])])

  dnl if the tag is none, generate no configure code here
  m4_bmatch([$4], [none], [], [dnl

    dnl check for am-yes tag. var name is shell-escaped
    m4_bmatch([$4], [am-yes], [dnl
      AM_CONDITIONAL(AS_TR_SH(value), [test x"$value" = x"yes"])dnl
    ])dnl

    dnl check for am-set tag. var name is shell-escaped
    m4_bmatch([$4], [am-set], [dnl
      AM_CONDITIONAL(AS_TR_SH(value), [test ! -z "$value"])dnl
    ])dnl

    dnl check for cpp-bool tag.
    m4_bmatch([$4], [cpp-bool], [dnl
      AS_VAR_PUSHDEF([int], my_tolower(value))
      AS_IF([test x"$value" = x"yes"], int=1, int=0)
      AC_DEFINE_UNQUOTED(AS_TR_CPP(value), [$int], [$5])
      AS_VAR_POPDEF([int])dnl
    ])dnl

    dnl check for cpp-int tag.
    m4_bmatch([$4], [cpp-int], [dnl
      AS_VAR_PUSHDEF([int], my_tolower(value))dnl
      int=$value dnl
      AC_DEFINE_UNQUOTED(AS_TR_CPP(value), [$int], [$5])dnl
      AS_VAR_POPDEF([int])dnl
    ])dnl

    m4_bmatch([$4], [mk], [dnl
      AC_SUBST(AS_TR_SH(value))dnl
    ])dnl

  ])dnl

  dnl clean up after myself
  AS_VAR_POPDEF([value])dnl
  m4_popdef([type])dnl
  m4_popdef([lower])dnl
])

AC_DEFUN([AX_ARG_ENABLE], [AX_ARG([enable], [$1], [$2], [$3], [$4])])
AC_DEFUN([AX_ARG_WITH],   [AX_ARG([with],   [$1], [$2], [$3], [$4])])
