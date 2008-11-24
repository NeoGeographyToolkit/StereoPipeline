# Usage: AX_PKG_GL
#
# TODO: Add support for other sources of GL and BLAS, such as
# ATLAS and the Intel math libraries.  For people who don't have any
# of these third party libraries installed, we need to fall back to
# compiling BLAS and GL ourselves.
AC_DEFUN([AX_PKG_GL],
[

  # If we are running MacOS X, we can use Apple's vecLib framework to
  # provide us with GL and BLAS routines.
  if test "$host_vendor" = apple; then
    AC_MSG_CHECKING(for package GL)
    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_RESULT([])
    fi

    HAVE_PKG_GL="yes"
    PKG_GL_LIBS="$ASP_LDFLAGS -framework OpenGL -framework GLUT"

    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_RESULT([found])
    fi
    if test "${HAVE_PKG_GL}" = "yes" ; then
      ax_have_pkg_bool=1
    else
      ax_have_pkg_bool=0
    fi
    AC_DEFINE_UNQUOTED([HAVE_PKG_GL],
                       [$ax_have_pkg_bool],
                       [Define to 1 if the GL package is available.])

    AC_SUBST(HAVE_PKG_GL)

    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_NOTICE([HAVE_PKG_GL = ${HAVE_PKG_GL}])
      AC_MSG_NOTICE([PKG_GL_LIBS = ${PKG_GL_LIBS}])
      AC_MSG_NOTICE([ASP_CPPFLAGS = ${ASP_CPPFLAGS}])
      AC_MSG_NOTICE([ASP_LDFLAGS = ${ASP_LDFLAGS}])
    else
      AC_MSG_RESULT([${HAVE_PKG_GL}])
    fi  

  # For all other platforms, we search for static GL libraries
  # in the conventional manner.
  else
    AX_PKG(GL, [X11], [-lGL -lGLU -lglut], [GL/gl.h GL/glu.h GL/glut.h])
  fi

])
