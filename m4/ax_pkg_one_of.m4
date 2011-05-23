dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl AX_PKG_ONE_OF(<name>, <alt1-name>, <alt1-check>, ... [<altn-name>, <altn-check>])
dnl Declare a package to be fulfilled if one of its alts are found.
dnl alt-name is a package name, and alt-check is code that checks for the package
dnl alt-check should define HAVE_PKG_ALTNAME
AC_DEFUN([AX_PKG_ONE_OF],
[m4_case($#,
  0, [m4_fatal([$0: too few arguments: $#])],
  2, [m4_ifset([$2], [m4_fatal([$0: too few arguments: [$1] [$2] $#])], [$0([$1])])],
  1, [AX_GROUP_PKG([$1])],
  [AS_IF([test ! -z "$PKG_$1_CPPFLAGS"], [PKG_$2_CPPFLAGS="$PKG_$2_CPPFLAGS $PKG_$1_CPPFLAGS"]) # push cppflags to child
   AS_IF([test ! -z "$PKG_$1_LDFLAGS"],  [PKG_$2_LDFLAGS="$PKG_$2_LDFLAGS $PKG_$1_LDFLAGS"])    # push ldflags to child
   AS_IF([test -z "$HAVE_PKG_$2"], [HAVE_PKG_$2="$HAVE_PKG_$1"]) # push have_pkg to child if it doesn't have one
   $3
   AS_IF([test x"$HAVE_PKG_$2" = "xyes"], [AX_GROUP_PKG([$1], [$2])],
    [$0([$1], m4_shiftn(3, $@))])]
)])
