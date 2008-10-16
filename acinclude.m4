dnl __BEGIN_LICENSE__
dnl 
dnl Copyright (C) 2006 United States Government as represented by the
dnl Administrator of the National Aeronautics and Space Administration
dnl (NASA).  All Rights Reserved.
dnl 
dnl Copyright 2006 Carnegie Mellon University. All rights reserved.
dnl 
dnl This software is distributed under the NASA Open Source Agreement
dnl (NOSA), version 1.3.  The NOSA has been approved by the Open Source
dnl Initiative.  See the file COPYING at the top of the distribution
dnl directory tree for the complete NOSA document.
dnl 
dnl THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
dnl KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
dnl LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
dnl SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
dnl A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
dnl THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
dnl DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
dnl 
dnl __END_LICENSE__

dnl Usage: AX_CONFIG_HEADER_PREFIX(<filename>, <prefix>)
dnl Generates a configuration header file, adding a prefix to all symbols.
dnl This is a two-step process.  First we generate the usual header file 
dnl with a filename ending in ".pre".  Then we process that file, adding 
dnl the prefix to all symbolx, and copy it into the final location if it 
dnl has changed.

AC_DEFUN([AX_CONFIG_HEADER_PREFIX],
[
  AM_CONFIG_HEADER([$1.pre],
  [
    echo "/* $1.  Generated from $1.pre by config.status.  */" > "$1.new"
    echo "#ifndef __$2_CONFIG_H__" >> "$1.new"
    echo "#define __$2_CONFIG_H__" >> "$1.new"
    sed -e 's/#define /#define $2/' -e 's/#undef /#undef $2/' < "$1.pre" >> "$1.new"
    echo "#endif // __$2_CONFIG_H__" >> "$1.new"
    if test -f "$1" && diff "$1" "$1.new" > /dev/null ; then
      echo "config.status: $1 is unchanged"
    else
      echo "config.status: creating $1"
      cp "$1.new" "$1"
    fi
    rm -f "$1.new"
  ])
])

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
  if test "$host_vendor" = "apple"; then
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

dnl Usage: AX_FIND_FILES(<filenames>, <search paths>)
dnl Looks to see if all the given filenames (relative paths) are accessible 
dnl from one of the given base paths.  Returns the path or the empty string 
dnl in ${ax_find_files_path}.
AC_DEFUN([AX_FIND_FILES],
[
  ax_find_files_path="";
  for path in $2; do
    ax_find_files_passed=yes
    for filename in $1; do
      pathname="$path/$filename"
      if test "$ENABLE_VERBOSE" = "yes"; then
        AC_MSG_CHECKING([for ${pathname}])
      fi
      ax_find_files_paths=`ls $pathname 2>/dev/null`
      if test ! -z "$ax_find_files_paths" ; then
        if test "$ENABLE_VERBOSE" = "yes"; then
          AC_MSG_RESULT([found])
        fi
      else
        if test "$ENABLE_VERBOSE" = "yes"; then
          AC_MSG_RESULT([not found])
        fi
        ax_find_files_passed=no
        break
      fi
    done
    if test "$ax_find_files_passed" = "yes"; then
      ax_find_files_path="$path"
      break
    fi
  done
])


dnl Usage: AX_PKG(<name>, <dependencies>, <libraries>, <headers>[, <additional cxxflags>])
AC_DEFUN([AX_PKG],
[
  AC_ARG_WITH(translit($1,`A-Z',`a-z'),
    AC_HELP_STRING([--with-]translit($1,`A-Z',`a-z'), [enable searching for the $1 package @<:@auto@:>@]),
    [ HAVE_PKG_$1=$withval ]
  )

 AC_ARG_WITH(translit($1,`A-Z',`a-z')[-cflags],
   AC_HELP_STRING([--with-]translit($1,`A-Z',`a-z')[-cppflags], [Add to $1_CPPFLAGS @<:@auto@:>@]),
   [ ADD_$1_CPPFLAGS=$withval ])
 AC_ARG_WITH(translit($1,`A-Z',`a-z')[-libs],
   AC_HELP_STRING([--with-]translit($1,`A-Z',`a-z')[-libs], [Override $1_LIBS and skip search @<:@auto@:>@]),
   [ FORCE_$1_LDFLAGS=$withval ])

  if test x"$ENABLE_VERBOSE" = "xyes"; then
    AC_MSG_CHECKING([for package $1 in current paths])
  else
    AC_MSG_CHECKING([for package $1])
  fi

  AC_LANG_ASSERT(C++)

  # We can skip searching if we're already at "no"
  if test "no" = "$HAVE_PKG_$1"; then
    AC_MSG_RESULT([no (disabled by user)])

  else
    # Test for and inherit libraries from dependencies
    if test -z "${FORCE_$1_LDFLAGS}"; then
        PKG_$1_LIBS="$3"
    else
        PKG_$1_LIBS="${FORCE_$1_LDFLAGS}"
    fi

    for x in $2; do
      ax_pkg_have_dep=HAVE_PKG_${x}
      if test "${!ax_pkg_have_dep}" = "yes"; then
        ax_pkg_dep_libs="PKG_${x}_LIBS"
        PKG_$1_LIBS="$PKG_$1_LIBS ${!ax_pkg_dep_libs}"
        unset ax_pkg_dep_libs
      else
        unset PKG_$1_LIBS
        HAVE_PKG_$1="no"
        break
      fi
    done

    if test "x$HAVE_PKG_$1" = "xno" ; then
      AC_MSG_RESULT([no (needs $x)])

    # We skip the search if the user has been explicit about "yes"
    elif test "x$HAVE_PKG_$1" = "xyes" ; then
      AC_MSG_RESULT([yes (using user-supplied flags)])

    # Otherwise we look for a path that contains the needed headers and libraries
    else

      if test "x$ENABLE_VERBOSE" = "yes"; then
        AC_MSG_RESULT([searching...])
      fi

      if test -n "${FORCE_$1_LDFLAGS}"; then
        PKG_PATHS_$1=""
      elif test -n "${HAVE_PKG_$1}" && test "${HAVE_PKG_$1}" != "yes" && test "${HAVE_PKG_$1}" != "no"; then
        PKG_PATHS_$1=${HAVE_PKG_$1}
      else
        PKG_PATHS_$1=${PKG_PATHS}
      fi

      HAVE_PKG_$1=no

      # This is a gross hack that causes the AC_LINK_IFELSE macro use libtool to 
      # link files rather that g++ alone.  This in important for detecting 
      # packages like the Vision Workbench which has many dependencies that 
      # themselves have *.la files.
      OLD_CXX=$CXX
      if test "$host_vendor" = apple; then
        # Apple has lazy link-time dependencies and a different name for libtool,
        # so we turn off this hack on the mac platform.
        CXX=$CXX
      else
        CXX="libtool --mode=link --tag CXX $CXX"
      fi

      ax_pkg_old_libs=$LIBS
      LIBS="$PKG_$1_LIBS $LIBS"
      for path in none $PKG_PATHS_$1; do
        ax_pkg_old_cppflags=$CPPFLAGS
        ax_pkg_old_ldflags=$LDFLAGS
        ax_pkg_old_vw_cppflags=$ASP_CPPFLAGS
        ax_pkg_old_vw_ldflags=$ASP_LDFLAGS
        echo > conftest.h
        for header in $4 ; do
          echo "#include <$header>" >> conftest.h
        done
        CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
        LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
        if test "$path" != "none"; then
          if test x"$ENABLE_VERBOSE" = "xyes"; then
            AC_MSG_CHECKING([for package $1 in $path])
          fi

          # ISIS is really stupid, and they use /foo/inc as their include file
          # location instead of /foo/include. So we check for that. This sees 
          # about any other idiot libraries that use the same design as well.
          AX_INCLUDE_DIR=include
          if ! test -d $path/${AX_INCLUDE_DIR}; then
            if test -d $path/inc; then
              AX_INCLUDE_DIR=inc
            fi
          fi

          if test -z "$5"; then
            ASP_CPPFLAGS="-I$path/${AX_INCLUDE_DIR} $ASP_CPPFLAGS"
          else
            ASP_CPPFLAGS="$ADD_$1_CPPFLAGS $5 $ASP_CPPFLAGS"
          fi
          CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
          AC_LINK_IFELSE(
            AC_LANG_PROGRAM([#include "conftest.h"],[]),
            [ HAVE_PKG_$1=yes ; AC_MSG_RESULT([yes]) ; break ] )
          ASP_LDFLAGS="-L$path/lib $ASP_LDFLAGS"
          LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
        fi
        AC_LINK_IFELSE(
          AC_LANG_PROGRAM([#include "conftest.h"],[]),
          [ HAVE_PKG_$1=yes ; AC_MSG_RESULT([yes]) ; break ] )
        if test x"$ENABLE_VERBOSE" = "xyes"; then
          AC_MSG_RESULT([no])
        fi
        CPPFLAGS=$ax_pkg_old_cppflags
        LDFLAGS=$ax_pkg_old_ldflags
        ASP_CPPFLAGS=$ax_pkg_old_vw_cppflags
        ASP_LDFLAGS=$ax_pkg_old_vw_ldflags
      done
      CPPFLAGS=$ax_pkg_old_cppflags
      LDFLAGS=$ax_pkg_old_ldflags
      LIBS=$ax_pkg_old_libs

      if test "x$HAVE_PKG_$1" = "xno" -a "x$ENABLE_VERBOSE" != "xyes"; then
        AC_MSG_RESULT([no (not found)])
      fi

      CXX=$OLD_CXX

    fi

  fi
  if test "${HAVE_PKG_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
    PKG_$1_LIBS=
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_$1],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 package is available.])

  AC_SUBST(PKG_$1_LIBS)
  AC_SUBST(HAVE_PKG_$1)

  if test x"$ENABLE_VERBOSE" == "xyes"; then
    AC_MSG_NOTICE([HAVE_PKG_$1 = ${HAVE_PKG_$1}])
    AC_MSG_NOTICE([PKG_$1_LIBS= $PKG_$1_LIBS])
    AC_MSG_NOTICE([ASP_CPPFLAGS= $ASP_CPPFLAGS])
    AC_MSG_NOTICE([ASP_LDFLAGS= $ASP_LDFLAGS])
  fi
])


# Usage: AX_PKG_BOOST
AC_DEFUN([AX_PKG_BOOST],
[
  AC_MSG_CHECKING(for package BOOST)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  AC_LANG_ASSERT(C++)

  if test -n "${HAVE_PKG_BOOST}" && test "${HAVE_PKG_BOOST}" != "yes" && test "${HAVE_PKG_BOOST}" != "no"; then
    PKG_PATHS_BOOST=${HAVE_PKG_BOOST}
    unset HAVE_PKG_BOOST
  else
    PKG_PATHS_BOOST=${PKG_PATHS}
  fi

  # Skip testing if the user has overridden
  if test -z "${HAVE_PKG_BOOST}"; then

    PKG_BOOST_LIBS=
    HAVE_PKG_BOOST=no

    for ax_boost_base_path in $PKG_PATHS_BOOST; do
      # First look for a system-style installation
      if test "$ENABLE_VERBOSE" = "yes"; then
        AC_MSG_CHECKING([for system-style boost in ${ax_boost_base_path}])
      fi
      if test -d "${ax_boost_base_path}/include/boost" ; then
        PKG_BOOST_INCDIR="${ax_boost_base_path}/include"
        PKG_BOOST_LIBDIR="${ax_boost_base_path}/lib"
        HAVE_PKG_BOOST="yes"
        if test "$ENABLE_VERBOSE" = "yes"; then
          AC_MSG_RESULT([found])
        fi
        break
      else
        if test "$ENABLE_VERBOSE" = "yes"; then
          AC_MSG_RESULT([not found])
        fi
      fi
      # Next look for a default-style installation
      if test "$ENABLE_VERBOSE" = "yes"; then
        AC_MSG_CHECKING([for default-style boost in ${ax_boost_base_path}])
      fi
      for ax_boost_inc_path in `ls -d ${ax_boost_base_path}/include/boost-* 2> /dev/null` ; do
        # At the moment we greedily accept the first one we find, regardless of version
        PKG_BOOST_INCDIR="${ax_boost_inc_path}"
        PKG_BOOST_LIBDIR="${ax_boost_base_path}/lib"
        HAVE_PKG_BOOST="yes"
        if test "$ENABLE_VERBOSE" = "yes"; then
          AC_MSG_RESULT([found])
        fi
        break 2
      done
      if test "$ENABLE_VERBOSE" = "yes"; then
        AC_MSG_RESULT([not found])
      fi
    done
  fi

  if test "${HAVE_PKG_BOOST}" = "yes" ; then
    ax_pkg_old_vw_cppflags=$ASP_CPPFLAGS
    ax_pkg_old_vw_ldflags=$ASP_LDFLAGS
    ax_pkg_old_cppflags=$CPPFLAGS
    ax_pkg_old_ldflags=$LDFLAGS
    ax_pkg_old_libs=$LIBS
    while true ; do
      # First see if the current paths are sufficient
      if test "x${ENABLE_VERBOSE}" = "xyes" ; then
	AC_MSG_CHECKING([whether current paths are sufficient...])
      fi
      AC_LINK_IFELSE( AC_LANG_PROGRAM([#include <boost/version.hpp>],[]), [ax_result=yes], [ax_result=no] )
      if test "x${ENABLE_VERBOSE}" = "xyes" ; then
	AC_MSG_RESULT([$ax_result])
      fi
      if test "$ax_result" = "yes" ; then break ; fi
      # Try it with just the include path
      ASP_CPPFLAGS="-I${PKG_BOOST_INCDIR} $ASP_CPPFLAGS"
      CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
      if test "x${ENABLE_VERBOSE}" = "xyes" ; then
	AC_MSG_CHECKING([whether adding the include path is sufficient...])
      fi
      AC_LINK_IFELSE( AC_LANG_PROGRAM([#include <boost/version.hpp>],[]), [ax_result=yes], [ax_result=no] )
      if test "x${ENABLE_VERBOSE}" = "xyes" ; then
	AC_MSG_RESULT([$ax_result])
      fi
      if test "$ax_result" = "yes" ; then break ; fi
      # Finally, try it with the linker path
      ASP_LDFLAGS="-L${PKG_BOOST_LIBDIR} $ASP_LDFLAGS"
      LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
      if test "x${ENABLE_VERBOSE}" = "xyes" ; then
	AC_MSG_CHECKING([whether adding the include and linker paths works...])
      fi
      AC_LINK_IFELSE( AC_LANG_PROGRAM([#include <boost/version.hpp>],[]), [ax_result=yes], [ax_result=no] )
      if test "x${ENABLE_VERBOSE}" = "xyes" ; then
	AC_MSG_RESULT([$ax_result])
      fi
      if test "$ax_result" = "yes" ; then break ; fi
      # The detected version of boost seems to be invalid!
      HAVE_PKG_BOOST="no"
      ASP_CPPFLAGS="$ax_pkg_old_vw_cppflags"
      ASP_LDFLAGS="$ax_pkg_old_vw_ldflags"
      unset PKG_BOOST_INCDIR
      unset PKG_BOOST_LIBDIR
      break
    done
  fi
  CPPFLAGS="$ax_pkg_old_cppflags"
  LDFLAGS="$ax_pkg_old_ldflags"

  if test "${HAVE_PKG_BOOST}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_BOOST],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the BOOST package is available.])

  AC_SUBST(HAVE_PKG_BOOST)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([HAVE_PKG_BOOST= $HAVE_PKG_BOOST])
    AC_MSG_NOTICE([ASP_CPPFLAGS= $ASP_CPPFLAGS])
    AC_MSG_NOTICE([ASP_LDFLAGS= $ASP_LDFLAGS])
  else
    AC_MSG_RESULT([$HAVE_PKG_BOOST])
  fi

])


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
  if test "$host_vendor" = apple; then
    AC_MSG_CHECKING(for package LAPACK)
    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_RESULT([])
    fi

    HAVE_PKG_LAPACK="yes"
    PKG_LAPACK_LIBS="$ASP_LDFLAGS -framework vecLib"

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
      AC_MSG_NOTICE([ASP_CPPFLAGS = ${ASP_CPPFLAGS}])
      AC_MSG_NOTICE([ASP_LDFLAGS = ${ASP_LDFLAGS}])
    else
      AC_MSG_RESULT([${HAVE_PKG_LAPACK}])
    fi  

  # For all other platforms, we search for static LAPACK libraries
  # in the conventional manner.
  else
    # First check for CLAPACK
    AX_PKG(LAPACK, [], [-lclapack -lblas -lf2c], [])
    if test "$HAVE_PKG_LAPACK" = "no"; then
      unset HAVE_PKG_LAPACK
      # Otherwise check for standard LAPACK
      AC_MSG_NOTICE(["CLAPACK not found, trying standard LAPACK."])
      unset HAVE_PKG_LAPACK
      AX_PKG(LAPACK, [], [-llapack -lblas], [])
    fi
    if test "$HAVE_PKG_LAPACK" = "no"; then
      unset HAVE_PKG_LAPACK
      # Some newer boxes require -lgfortran, so try that too
      AC_MSG_NOTICE(["trying standard LAPACK with -lgfortran."])
      unset HAVE_PKG_LAPACK
      AX_PKG(LAPACK, [], [-llapack -lblas -lgfortran], [])
    fi
  fi

])

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


# Usage: AX_PKG_BOOST_LIB(<name>, <libraries>, <header>)
AC_DEFUN([AX_PKG_BOOST_LIB],
[
  AC_MSG_CHECKING(for package BOOST_$1)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  AC_LANG_ASSERT(C++)

  # Skip testing if the user has overridden
  if test -z "${HAVE_PKG_BOOST_$1}"; then

    HAVE_PKG_BOOST_$1=no

    # Check for general Boost presence
    if test "x${HAVE_PKG_BOOST}" = "xyes" ; then
      # Check for required headers
      AX_FIND_FILES([$3],[${PKG_BOOST_INCDIR}])
      if test ! -z "$ax_find_files_path" ; then
        # Check for required libraries with no suffix
        AX_FIND_FILES([`echo $2 | sed 's/-l\([[^[:space:]]]*\)/lib\1.*/g'`],[$PKG_BOOST_LIBDIR])
        if test ! -z "$ax_find_files_path" ; then
          HAVE_PKG_BOOST_$1="yes"
          PKG_BOOST_$1_LIBS="$2"
        else
          # Check for required libraries with some suffix
          ax_pkg_boost_lib=`echo $2 | awk '{print [$]1}' | sed 's/-l\([[^[:space:]-]]*\).*/lib\1/g'`
          ax_pkg_boost_lib_ext=`ls ${PKG_BOOST_LIBDIR}/${ax_pkg_boost_lib}-* | head -n 1 | sed "s,^${PKG_BOOST_LIBDIR}/${ax_pkg_boost_lib}\(-[[^.]]*\).*,\1,"`
          if test ! -z "$ax_pkg_boost_lib_ext" ; then
            AX_FIND_FILES([`echo $2 | sed "s/-l\([[^[:space:]]]*\)/lib\1${ax_pkg_boost_lib_ext}.*/g"`],[$PKG_BOOST_LIBDIR])
            if test ! -z "$ax_find_files_path" ; then
              HAVE_PKG_BOOST_$1="yes"
              PKG_BOOST_$1_LIBS=`echo $2 | sed "s/[[^ ]]*/&${ax_pkg_boost_lib_ext}/g"`
            fi
          fi
        fi
      fi
    fi
  fi

  ax_pkg_old_vw_cppflags=$ASP_CPPFLAGS
  ax_pkg_old_vw_ldflags=$ASP_LDFLAGS
  ax_pkg_old_cppflags=$CPPFLAGS
  ax_pkg_old_ldflags=$LDFLAGS
  ax_pkg_old_libs=$LIBS
  while true ; do
    echo > conftest.h
    for header in $3 ; do
      echo "#include <$header>" >> conftest.h
    done
    # First see if the current paths are sufficient
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_CHECKING([whether current paths are sufficient...])
    fi
    CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
    LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
    LIBS="$PKG_BOOST_$1_LIBS $ax_pkg_old_libs"
    AC_LINK_IFELSE( AC_LANG_PROGRAM([#include "conftest.h"],[]), [ax_result=yes], [ax_result=no] )
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_RESULT([$ax_result])
    fi
    if test "$ax_result" = "yes" ; then break ; fi
    # Try it with just the include path
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_CHECKING([whether adding the include path is sufficient...])
    fi
    ASP_CPPFLAGS="-I${PKG_BOOST_INCDIR} $ASP_CPPFLAGS"
    CPPFLAGS="$ax_pkg_old_cppflags $ASP_CPPFLAGS"
    AC_LINK_IFELSE( AC_LANG_PROGRAM([#include "conftest.h"],[]), [ax_result=yes], [ax_result=no] )
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_RESULT([$ax_result])
    fi
    if test "$ax_result" = "yes" ; then break ; fi
    # Finally, try it with the linker path
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_CHECKING([whether adding the include and linker paths works...])
    fi
    ASP_LDFLAGS="-L${PKG_BOOST_LIBDIR} $ASP_LDFLAGS"
    LDFLAGS="$ax_pkg_old_ldflags $ASP_LDFLAGS"
    AC_LINK_IFELSE( AC_LANG_PROGRAM([#include "conftest.h"],[]), [ax_result=yes], [ax_result=no] )
    if test "x${ENABLE_VERBOSE}" = "xyes" ; then
      AC_MSG_RESULT([$ax_result])
    fi
    if test "$ax_result" = "yes" ; then break ; fi
    # The detected version of boost seems to be invalid!
    HAVE_PKG_BOOST_$1="no"
    ASP_CPPFLAGS="$ax_pkg_old_vw_cppflags"
    ASP_LDFLAGS="$ax_pkg_old_vw_ldflags"
    break
  done

  CPPFLAGS="$ax_pkg_old_cppflags"
  LDFLAGS="$ax_pkg_old_ldflags"
  LIBS="$ax_pkg_old_libs"

  if test "${HAVE_PKG_BOOST_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_BOOST_$1],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the BOOST_$1 package is available.])

  AC_SUBST(HAVE_PKG_BOOST_$1)
  AC_SUBST(PKG_BOOST_$1_LIBS)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([HAVE_PKG_BOOST_$1 = ${HAVE_PKG_BOOST_$1}])
    AC_MSG_NOTICE([PKG_BOOST_$1_LIBS= $PKG_BOOST_$1_LIBS])
  else
    AC_MSG_RESULT([${HAVE_PKG_BOOST_$1}])
  fi
])

dnl Usage: AX_PKG_PTHREADS
AC_DEFUN([AX_PKG_PTHREADS],
[
  AC_MSG_CHECKING(for package PTHREADS)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  AC_LANG_PUSH(C)
  HAVE_PKG_PTHREADS=no

  ax_pkg_pthreads_cppflags_options="none -pthread"
  ax_pkg_pthreads_ldflags_options="-lpthread none"

  for ax_pkg_pthreads_ldflags in $ax_pkg_pthreads_ldflags_options; do
    if test "$ax_pkg_pthreads_ldflags" = "none" ; then
      PKG_PTHREADS_LDFLAGS=""
    else
      PKG_PTHREADS_LDFLAGS=$ax_pkg_pthreads_ldflags
    fi
    for ax_pkg_pthreads_cppflags in $ax_pkg_pthreads_cppflags_options; do
      if test "$ax_pkg_pthreads_cppflags" = "none" ; then
        PKG_PTHREADS_CPPFLAGS=""
      else
        PKG_PTHREADS_CPPFLAGS=$ax_pkg_pthreads_cppflags
      fi

      ax_pkg_pthreads_save_CFLAGS="$CFLAGS"
      ax_pkg_pthreads_save_LDFLAGS="$LDFLAGS"
      CFLAGS="$CFLAGS $PKG_PTHREADS_CPPFLAGS"
      LDFLAGS="$PKG_PTHREADS_LDFLAGS $LDFLAGS"

      if test "$ENABLE_VERBOSE" = "yes" ; then
        AC_MSG_CHECKING([whether pthreads work with flags: \"$CFLAGS\" : \"$LDFLAGS\"])
      fi

      AC_TRY_LINK([#include <pthread.h>],
                  [pthread_t th; pthread_create(0,0,0,0);],
                  [HAVE_PKG_PTHREADS=yes])

      CFLAGS="$ax_pkg_pthreads_save_CFLAGS"
      LDFLAGS="$ax_pkg_pthreads_save_LDFLAGS"

      if test "$ENABLE_VERBOSE" = "yes" ; then
        AC_MSG_RESULT($HAVE_PKG_PTHREADS)
      fi

      if test "$HAVE_PKG_PTHREADS" = "yes"; then
        break 2;
      fi
    done
  done

  AC_LANG_POP(C)

  if test "$HAVE_PKG_PTHREADS" = "yes" ; then
    CFLAGS="$CFLAGS $PKG_PTHREADS_CPPFLAGS"
    CXXFLAGS="$CXXFLAGS $PKG_PTHREADS_CPPFLAGS"
    PKG_PTHREADS_LIBS="$PKG_PTHREADS_LDFLAGS"
  fi

  if test "${HAVE_PKG_PTHREADS}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_PTHREADS],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the PTHREADS package is available.])

  AC_SUBST(HAVE_PKG_PTHREADS)
  AC_SUBST(PKG_PTHREADS_LIBS)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([HAVE_PKG_PTHREADS = ${HAVE_PKG_PTHREADS}])
    AC_MSG_NOTICE([PKG_PTHREADS_LIBS = ${PKG_PTHREADS_LIBS}])
    AC_MSG_NOTICE([CFLAGS= $CFLAGS])
    AC_MSG_NOTICE([CXXFLAGS= $CXXFLAGS])
  else
    AC_MSG_RESULT([${HAVE_PKG_PTHREADS}])
  fi
])


# Usage: AX_APP(<name>, <directory>, <default>, <required dependencies>[, <optional dependencies>])
AC_DEFUN([AX_APP],
[
  # Silently ignore modules that don't exist in this distribution
  if test -d $2 ; then

    HAVE_PKG_$1_SRC=yes

    AC_ARG_ENABLE([app-]translit($1,`A-Z',`a-z'),
      AC_HELP_STRING([--enable-app-]translit($1,`A-Z',`a-z'), [enable the $1 app @<:@$3@:>@]), 
      [ ENABLE_APP_$1=$enableval ],
      [ if test "x$ENABLE_APP_$1" = x; then ENABLE_APP_$1=`/bin/echo -n $3 | tr [A-Z] [a-z]` ; fi ]
    )

    AC_MSG_CHECKING([whether to build app $1])
    ax_app_enable=$ENABLE_APP_$1

    if test "$ax_app_enable" != "yes" ; then
      AC_MSG_RESULT([no (disabled)])
    fi

    ax_libs=""

    # Check for required dependencies
    if test "$ax_app_enable" = "yes" ; then
      for ax_dependency in $4 ; do
        ax_dependency_have="HAVE_PKG_${ax_dependency}"
        if test "x${!ax_dependency_have}" = "xyes"; then
          ax_dep_libs="PKG_${ax_dependency}_LIBS"
          ax_libs="${ax_libs} ${!ax_dep_libs}"
        else
          AC_MSG_RESULT([no])
          AC_MSG_NOTICE([warning: unable to build requested app $1 (needs ${ax_dependency})!])
          ax_app_enable=no;
          break;
        fi
      done
    fi
    
    if test "$ax_app_enable" = "yes" ; then
      # Check for optional dependencies
      for ax_dependency in $5 ; do
        ax_dependency_have="HAVE_PKG_${ax_dependency}"
        if test "x${!ax_dependency_have}" = "xyes"; then
          ax_dep_libs="PKG_${ax_dependency}_LIBS"
          ax_libs="${ax_libs} ${!ax_dep_libs}"
        fi
      done

      # Set up the variables
      APP_$1_LIBS=$ax_libs
      PKG_$1_LIBS=$ax_libs
      AC_MSG_RESULT([yes])
    fi
  
  else
    HAVE_PKG_$1_SRC=no
    ax_app_enable=no
    APP_$1_LIBS=
    PKG_$1_LIBS=
  fi

  AC_SUBST(APP_$1_LIBS)
  AC_SUBST(PKG_$1_LIBS)

  HAVE_PKG_$1=${ax_app_enable}
  MAKE_APP_$1=${ax_app_enable}
  AC_SUBST(MAKE_APP_$1)

  if test "${HAVE_PKG_$1}" = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED(HAVE_PKG_$1,
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 app is available.])

  if test "$ENABLE_VERBOSE" = "yes" && test "$HAVE_PKG_$1_SRC" = "yes" ; then
    AC_MSG_NOTICE(MAKE_APP_$1 = ${MAKE_APP_$1})
    AC_MSG_NOTICE(HAVE_PKG_$1 = ${HAVE_PKG_$1})
    AC_MSG_NOTICE(APP_$1_LIBS = ${APP_$1_LIBS})
    AC_MSG_NOTICE(PKG_$1_LIBS = ${PKG_$1_LIBS})
  fi

  AM_CONDITIONAL([MAKE_APP_$1], [test "$MAKE_APP_$1" = "yes"])
])

# Include Autotroll for building QT components w/out using qmake as 
# the primary build tool.
m4_include([config/autotroll.m4])
