dnl AX_PKG_ONE_OF(<name>, <alt1-name>, <alt1-check>, ... [<altn-name>, <altn-check>])
dnl Declare a package to be fulfilled if one of its alts are found.
dnl alt-name is a package name, and alt-check is code that checks for the package
dnl alt-check should define HAVE_PKG_ALTNAME
AC_DEFUN([AX_PKG_ONE_OF],
[m4_case($#,
  0, [m4_fatal([$0: too few arguments: $#])],
  2, [m4_ifset([$2], [m4_fatal([$0: too few arguments: [$1] [$2] $#])], [$0([$1])])],
  1, [AX_GROUP_PKG([$1])],
  [$3
   AS_VAR_PUSHDEF([has_dep], [HAVE_PKG_]$2)
   AS_IF([test x"$has_dep" = "xyes"], [AX_GROUP_PKG([$1], [$2])],
    [$0([$1], m4_shiftn(3, $@))])
   AS_VAR_POPDEF([has_dep])]
)])
