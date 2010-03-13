dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2010 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
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


m4_define([_AX_FIXUPS], [1])
