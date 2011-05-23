dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
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
