# Usage: AX_PKG_LAPACK
#
# TODO: Add support for other sources of LAPACK and BLAS, such as
# ATLAS and the Intel math libraries.  For people who don't have any
# of these third party libraries installed, we need to fall back to
# compiling BLAS and LAPACK ourselves.
AC_DEFUN([AX_PKG_LAPACK],
[

  # If we are running MacOS X, we can use Apple's vecLib framework to
  # provide us with LAPACK and BLAS routines.
  if test $host_vendor = apple; then
    AC_MSG_CHECKING(for package LAPACK)
    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_RESULT([])
    fi

    HAVE_PKG_LAPACK="yes"
    PKG_LAPACK_LIBS="$OTHER_LDFLAGS -framework vecLib"

    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_RESULT([found])
    fi
    if test "${HAVE_PKG_LAPACK}" = "yes" ; then
      ax_have_pkg_bool=1
    else
      ax_have_pkg_bool=0
    fi
    AC_DEFINE_UNQUOTED([HAVE_PKG_LAPACK],
                       [$ax_have_pkg_bool],
                       [Define to 1 if the LAPACK package is available.])

    AC_SUBST(HAVE_PKG_LAPACK)

    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_NOTICE([HAVE_PKG_LAPACK = ${HAVE_PKG_LAPACK}])
      AC_MSG_NOTICE([PKG_LAPACK_LIBS = ${PKG_LAPACK_LIBS}])
      AC_MSG_NOTICE([OTHER_CPPFLAGS = ${OTHER_CPPFLAGS}])
      AC_MSG_NOTICE([OTHER_LDFLAGS = ${OTHER_LDFLAGS}])
      AC_MSG_NOTICE([CPPFLAGS= $CPPFLAGS])
      AC_MSG_NOTICE([LDFLAGS= $LDFLAGS])
    else
      AC_MSG_RESULT([${HAVE_PKG_LAPACK}])
    fi

  # For all other platforms, we search for static LAPACK libraries
  # in the conventional manner.
  else
    # First check for CLAPACK
    AX_PKG(CLAPACK, [], [-lclapack -lblas -lf2c], [])
    if test "$HAVE_PKG_CLAPACK" = "no"; then
      # Otherwise check for standard LAPACK
      AC_MSG_NOTICE(["CLAPACK not found, trying standard LAPACK."])
      AX_PKG(SLAPACK, [], [-llapack -lblas], [])

      if test "$HAVE_PKG_SLAPACK" = "no"; then
        # Some newer boxes require -lgfortran, so try that too
        AC_MSG_NOTICE(["trying standard LAPACK with -lgfortran."])
        AX_PKG(FLAPACK, [], [-llapack -lblas -lgfortran], [])

        if test "$HAVE_PKG_FLAPACK" = "no"; then
          # On some systems, BLAS and LAPACK are installed in different places
          AC_MSG_NOTICE(["trying to find BLAS and LAPACK seperately."])
          AX_PKG(STANDALONE_BLAS, [], [-lblas], [])
          AX_PKG(STANDALONE_LAPACK, [], [-llapack], [])
          AX_PKG(STANDALONE_LAPACK_AND_BLAS, [STANDALONE_LAPACK STANDALONE_BLAS], [], [])

          if test "$HAVE_PKG_STANDALONE_LAPACK_AND_BLAS" = "no"; then
            # On some systems, F2C, FBLAS and FLAPACK are installed in different places
            AC_MSG_NOTICE(["trying to find F2C, FBLAS, and FLAPACK seperately."])
            AX_PKG(STANDALONE_F2C, [], [-lf2c], [])
            AX_PKG(STANDALONE_FBLAS, [STANDALONE_F2C], [-lblas], [])
            AX_PKG(STANDALONE_FLAPACK, [STANDALONE_F2C], [-llapack], [])
            AX_PKG(LAPACK, [STANDALONE_FLAPACK STANDALONE_FBLAS STANDALONE_F2C], [], [])
          else
            AX_PKG(LAPACK, [STANDALONE_LAPACK_AND_BLAS], [], [])
          fi # FBLAS and FLAPACK
        else
          AX_PKG(LAPACK, [FLAPACK], [], [])
        fi # BLAS and LAPACK
      else
        AX_PKG(LAPACK, [SLAPACK], [], [])
      fi # fortan LAPACK
    else
      AX_PKG(LAPACK, [CLAPACK], [], [])
    fi
  fi
])
