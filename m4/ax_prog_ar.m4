dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl Usage: AX_PROG_AR
dnl Provides a workaround for Mac OS X's unusual ar behavior, so that
dnl it's possible to build static convenience libraries using libtool
dnl on that platform.  Basically, if we're on a Mac we introduce a
dnl wrapper shell script for ar that detects when we're creating an
dnl empty library and creates it by hand.  In all other cases it just
dnl relays the arguments to the user's AR.
AC_DEFUN([AX_PROG_AR],
[
  AC_REQUIRE([AM_AUX_DIR_EXPAND])
  AC_MSG_CHECKING([whether to use ar.sh wrapper script])
  if test $host_vendor = "apple"; then
    AC_MSG_RESULT([yes])
    if test -z "$AR" ; then
      ax_ar_prog=ar;
    else
      ax_ar_prog=$AR;
    fi
    AR=$am_aux_dir/ar.sh
    cat > $AR <<_AX_EOF
#!/bin/sh
if test -z "[\${3}]" ; then
  echo '!<arch>' > [\${2}]
else
  $ax_ar_prog [\${*}]
fi
_AX_EOF
    chmod +x $am_aux_dir/ar.sh
  else
    AC_MSG_RESULT([no])
  fi
])
