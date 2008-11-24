dnl Usage: AX_USE_LIBTOOL_LINKTEST([enable])
dnl This is a gross hack that causes the AC_LINK_IFELSE macro use libtool to
dnl link files rather that g++ alone. This in important for detecting packages
dnl like the Vision Workbench which has many dependencies that themselves have
dnl *.la files. if enable is non-empty, it is enabled. if enable is empty, it
dnl is disabled. NOT REENTRANT!
AC_DEFUN([AX_USE_LIBTOOL_LINKTEST], [
  if test -z "$1"; then
    if test -n "$LIBTOOL_LINKTEST_OLD_CXX" ; then
      CXX="$LIBTOOL_LINKTEST_OLD_CXX"
      LIBTOOL_LINKTEST_OLD_CXX=""
    fi
  else
    LIBTOOL_LINKTEST_OLD_CXX="$CXX"
    if test "$host_vendor" = apple; then
      # Apple has lazy link-time dependencies and a different name for libtool,
      # so we turn off this hack on the mac platform.
      CXX=$CXX
    else
      CXX="libtool --mode=link --tag CXX $CXX"
    fi
  fi
])
