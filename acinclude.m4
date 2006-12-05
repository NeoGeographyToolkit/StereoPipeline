dnl __BEGIN_LICENSE__
dnl
dnl Copyright (C) 2006 United States Government as represented by the
dnl Administrator of the National Aeronautics and Space Administration
dnl (NASA).  All Rights Reserved.
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


dnl Usage: AX_PKG(<name>, <dependencies>, <libraries>, <headers>[, <relative include path>])
AC_DEFUN([AX_PKG],
[
  AC_MSG_CHECKING(for package $1)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  HAVE_PKG_$1=yes

  # Test for and inherit flags from dependencies
  for x in $2; do
    ax_pkg_have_dep=HAVE_PKG_${x}
    if test "${!ax_pkg_have_dep}" == "yes"; then
      ax_pkg_dep_cppflags=PKG_${x}_CPPFLAGS
      ax_pkg_dep_ldflags=PKG_${x}_LDFLAGS
      PKG_$1_CPPFLAGS="${PKG_$1_CPPFLAGS} ${!ax_pkg_dep_cppflags}"
      PKG_$1_LDFLAGS="${PKG_$1_LDFLAGS} ${!ax_pkg_dep_ldflags}"
      unset ax_pkg_dep_cppflags ax_pkg_dep_ldflags
    else
      unset PKG_$1_CPPFLAGS
      unset PKG_$1_LDFLAGS
      HAVE_PKG_$1=no
      break
    fi
  done

  # Test for and configure flags for header and library files
  if test "$HAVE_PKG_$1" = "yes" ; then
    unset ax_pkg_headers ax_pkg_libs
    if test ! -z "$3"; then
      ax_pkg_libs=`echo $3 | sed 's/-l\([[^[:space:]]]*\)/lib\/lib\1.*/g'`
    fi
    if test ! -z "$4"; then
      if test ! -z "$5"; then
        ax_pkg_headers=`for x in $4; do echo -n "include/$5/${x} "; done`
      else
        ax_pkg_headers=`for x in $4; do echo -n "include/${x} "; done`
      fi
    fi
    ax_pkg_files="$ax_pkg_headers $ax_pkg_libs"
    if test ! -z "$3" || test ! -z "$4" ; then
      AX_FIND_FILES([$ax_pkg_files], [$PKG_PATHS])
      if test -z "$ax_find_files_path"; then
        HAVE_PKG_$1=no
      else
        PKG_$1_LDFLAGS="${PKG_$1_LDFLAGS} -L$ax_find_files_path/lib $3"
        if test -z "$5"; then
          PKG_$1_CPPFLAGS="${PKG_$1_CPPFLAGS} -I$ax_find_files_path/include"
        else
          PKG_$1_CPPFLAGS="${PKG_$1_CPPFLAGS} -I${ax_find_files_path}/include/$5"
        fi
      fi
    fi
  fi

  if test "$HAVE_PKG_$1" != "yes" ; then
    PKG_$1_CPPFLAGS=
    PKG_$1_LDFLAGS=
  fi

  if test ${HAVE_PKG_$1} = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_$1],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 package is available.])

  AC_SUBST(PKG_$1_CPPFLAGS)
  AC_SUBST(PKG_$1_LDFLAGS)
  AC_SUBST(HAVE_PKG_$1)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([PKG_$1_CPPFLAGS = ${PKG_$1_CPPFLAGS}])
    AC_MSG_NOTICE([PKG_$1_LDFLAGS = ${PKG_$1_LDFLAGS}])
    AC_MSG_NOTICE([HAVE_PKG_$1 = ${HAVE_PKG_$1}])
  else
    AC_MSG_RESULT([${HAVE_PKG_$1}])
  fi
])


# Usage: AX_PKG_BOOST
AC_DEFUN([AX_PKG_BOOST],
[
  AC_MSG_CHECKING(for package BOOST)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  PKG_BOOST_CPPFLAGS=
  PKG_BOOST_LDFLAGS=
  HAVE_PKG_BOOST=no

  for ax_boost_base_path in $PKG_PATHS; do
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

  if test "$HAVE_PKG_BOOST" = "yes" ; then
    PKG_BOOST_CPPFLAGS="-I${PKG_BOOST_INCDIR}"
    PKG_BOOST_LDFLAGS="-L${PKG_BOOST_LIBDIR}"
  fi 

  if test ${HAVE_PKG_BOOST} = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_BOOST],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the BOOST package is available.])

  AC_SUBST(PKG_BOOST_CPPFLAGS)
  AC_SUBST(PKG_BOOST_LDFLAGS)
  AC_SUBST(HAVE_PKG_BOOST)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([PKG_BOOST_CPPFLAGS = ${PKG_BOOST_CPPFLAGS}])
    AC_MSG_NOTICE([PKG_BOOST_LDFLAGS = ${PKG_BOOST_LDFLAGS}])
    AC_MSG_NOTICE([HAVE_PKG_BOOST = ${HAVE_PKG_BOOST}])
  else
    AC_MSG_RESULT([${HAVE_PKG_BOOST}])
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
  AC_MSG_CHECKING(for package LAPACK)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  PKG_LAPACK_CPPFLAGS=
  PKG_LAPACK_LDFLAGS=
  HAVE_PKG_LAPACK=no

	# If we are running MacOS X, we can use Apple's vecLib framework to
  # provide us with LAPACK and BLAS routines.
  if test $host_vendor == apple; then
		HAVE_PKG_LAPACK="yes"
		
		# This workaround sidesteps a bug in libtool that prevents the
		# -framework directive on a mac from being used when it appears in
		# the inherited_linker_flags of a *.la file.  Instead we force the
		# framework to appear in linker lines throughout the Vision
		# Workbench build system.  This allows binary apps to link when
		# necessary, and it is a harmless extra option in all other cases.
    #
    # Someday when libtool gets its act together, we should go for the
    # more conservative, commented out line that follows and remove
    # the uncommented line below.
    LDFLAGS="$LDFLAGS -framework vecLib"
    #	  PKG_LAPACK_LDFLAGS="-framework vecLib"

    if test "$ENABLE_VERBOSE" = "yes"; then
      AC_MSG_RESULT([found])
    fi
	fi

  if test ${HAVE_PKG_LAPACK} = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_LAPACK],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the LAPACK package is available.])

  AC_SUBST(PKG_LAPACK_CPPFLAGS)
  AC_SUBST(PKG_LAPACK_LDFLAGS)
  AC_SUBST(HAVE_PKG_LAPACK)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([PKG_LAPACK_CPPFLAGS = ${PKG_LAPACK_CPPFLAGS}])
    AC_MSG_NOTICE([PKG_LAPACK_LDFLAGS = ${PKG_LAPACK_LDFLAGS}])
    AC_MSG_NOTICE([HAVE_PKG_LAPACK = ${HAVE_PKG_LAPACK}])
  else
    AC_MSG_RESULT([${HAVE_PKG_LAPACK}])
  fi
])

# Usage: AX_PKG_BOOST_LIB(<name>, <dependencies>, <libraries>)
AC_DEFUN([AX_PKG_BOOST_LIB],
[
  AC_MSG_CHECKING(for package BOOST_$1)
  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_RESULT([])
  fi

  PKG_BOOST_$1_CPPFLAGS=
  PKG_BOOST_$1_LDFLAGS=
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
        PKG_BOOST_$1_LDFLAGS="$2"
      else
        # Check for required libraries with some suffix
				ax_pkg_boost_lib=`echo $2 | awk '{print [$]1}' | sed 's/-l\([[^[:space:]-]]*\).*/lib\1/g'`
				ax_pkg_boost_lib_ext=`ls ${PKG_BOOST_LIBDIR}/${ax_pkg_boost_lib}-* | head -n 1 | sed "s,^${PKG_BOOST_LIBDIR}/${ax_pkg_boost_lib}\(-[[^.]]*\).*,\1,"`
        if test ! -z "$ax_pkg_boost_lib_ext" ; then
          AX_FIND_FILES([`echo $2 | sed "s/-l\([[^[:space:]]]*\)/lib\1${ax_pkg_boost_lib_ext}.*/g"`],[$PKG_BOOST_LIBDIR])
          if test ! -z $ax_find_files_path ; then
            HAVE_PKG_BOOST_$1="yes"
            PKG_BOOST_$1_LDFLAGS=`echo $2 | sed "s/[[^ ]]*/&${ax_pkg_boost_lib_ext}/g"`
          fi
        fi
      fi
    fi
  fi

  if test ${HAVE_PKG_BOOST_$1} = "yes" ; then
    PKG_BOOST_$1_CPPFLAGS=$PKG_BOOST_CPPFLAGS
    PKG_BOOST_$1_LDFLAGS="$PKG_BOOST_LDFLAGS $PKG_BOOST_$1_LDFLAGS"
  fi

  if test ${HAVE_PKG_BOOST_$1} = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_BOOST_$1],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the BOOST_$1 package is available.])

  AC_SUBST(PKG_BOOST_$1_CPPFLAGS)
  AC_SUBST(PKG_BOOST_$1_LDFLAGS)
  AC_SUBST(HAVE_PKG_BOOST_$1)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([PKG_BOOST_$1_CPPFLAGS = ${PKG_BOOST_$1_CPPFLAGS}])
    AC_MSG_NOTICE([PKG_BOOST_$1_LDFLAGS = ${PKG_BOOST_$1_LDFLAGS}])
    AC_MSG_NOTICE([HAVE_PKG_BOOST_$1 = ${HAVE_PKG_BOOST_$1}])
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

  AC_LANG_SAVE
  AC_LANG_C
  HAVE_PKG_PTHREADS=no

  ax_pkg_pthreads_cppflags_options="-pthread none"
  ax_pkg_pthreads_ldflags_options="none -lpthread"

  for ax_pkg_pthreads_ldflags in $ax_pkg_pthreads_ldflags_options; do
    if test "$ax_pkg_pthreads_ldflags" == "none" ; then
      PKG_PTHREADS_LDFLAGS=""
    else
      PKG_PTHREADS_LDFLAGS=$ax_pkg_pthreads_ldflags
    fi
    for ax_pkg_pthreads_cppflags in $ax_pkg_pthreads_cppflags_options; do
      if test "$ax_pkg_pthreads_cppflags" == "none" ; then
        PKG_PTHREADS_CPPFLAGS=""
      else
        PKG_PTHREADS_CPPFLAGS=$ax_pkg_pthreads_cppflags
      fi

      ax_pkg_pthreads_save_CFLAGS="$CFLAGS"
      ax_pkg_pthreads_save_LDFLAGS="$LDFLAGS"
      CFLAGS="$CFLAGS $PKG_PTHREADS_CPPFLAGS"
      LDFLAGS="$PKG_PTHREADS_LDFLAGS $LDFLAGS"

      if test "$ENABLE_VERBOSE" = "yes" ; then
        AC_MSG_CHECKING([whether pthreads work with flags: \"$CFLAGS $LDFLAGS\"])
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

  AC_LANG_RESTORE

  if test "$HAVE_PKG_PTHREADS" = "yes" ; then
    PKG_PTHREADS_LDFLAGS="$PKG_PTHREADS_CPPFLAGS $PKG_PTHREADS_LDFLAGS"
  else
    PKG_PTHREADS_CPPFLAGS=""
    PKG_PTHREADS_LDFLAGS=""
  fi

  if test ${HAVE_PKG_PTHREADS} = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED([HAVE_PKG_PTHREADS],
                     [$ax_have_pkg_bool],
                     [Define to 1 if the PTHREADS package is available.])

  AC_SUBST(PKG_PTHREADS_CPPFLAGS)
  AC_SUBST(PKG_PTHREADS_LDFLAGS)
  AC_SUBST(HAVE_PKG_PTHREADS)

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE([PKG_PTHREADS_CPPFLAGS = ${PKG_PTHREADS_CPPFLAGS}])
    AC_MSG_NOTICE([PKG_PTHREADS_LDFLAGS = ${PKG_PTHREADS_LDFLAGS}])
    AC_MSG_NOTICE([HAVE_PKG_PTHREADS = ${HAVE_PKG_PTHREADS}])
  else
    AC_MSG_RESULT([${HAVE_PKG_PTHREADS}])
  fi
])


# Usage: AX_MODULE(<name>, <directory>, <default>, <required dependencies>[, <optional dependencies>])
AC_DEFUN([AX_MODULE],
[

  AC_ARG_ENABLE([module-]translit($1,`A-Z',`a-z'),
    AC_HELP_STRING([--enable-module-]translit($1,`A-Z',`a-z'), [enable the $2 module @<:@$3@:>@]), 
    [ ENABLE_MODULE_$1=$enableval ],
    [ if test x$ENABLE_MODULE_$1 = x; then ENABLE_MODULE_$1=$3 ; fi ]
  )

  AC_MSG_CHECKING([whether to build module $2])
  ax_module_enable=$ENABLE_MODULE_$1

  if test $ax_module_enable != "yes" ; then
    AC_MSG_RESULT([no])
  fi

  ax_cppflags=""
  ax_ldflags=""

  # Check for necessary dependencies
  if test $ax_module_enable = "yes" ; then
    for ax_dependency in $4 ; do
      ax_dependency_have="HAVE_PKG_${ax_dependency}"
      if test x${!ax_dependency_have} = "xyes"; then
        ax_dep_cppflags="PKG_${ax_dependency}_CPPFLAGS"
        ax_dep_ldflags="PKG_${ax_dependency}_LDFLAGS"
        ax_cppflags="${ax_cppflags} ${!ax_dep_cppflags}"
        ax_ldflags="${ax_ldflags} ${!ax_dep_ldflags}"
      else
        AC_MSG_RESULT([no])
	AC_MSG_NOTICE([warning: unable to build requested module $2 (no ${ax_dependency})!])
        ax_module_enable=no;
        break;
      fi
    done
  fi

  if test $ax_module_enable = "yes" ; then
    # Check for optional dependencies
    for ax_dependency in $5 ; do
      ax_dependency_have="HAVE_PKG_${ax_dependency}"
      if test x${!ax_dependency_have} = "xyes"; then
        ax_dep_cppflags="PKG_${ax_dependency}_CPPFLAGS"
        ax_dep_ldflags="PKG_${ax_dependency}_LDFLAGS"
        ax_cppflags="${ax_cppflags} ${!ax_dep_cppflags}"
        ax_ldflags="${ax_ldflags} ${!ax_dep_ldflags}"
      fi
    done

    # Set up the variables
    MODULE_$1_CPPFLAGS=$ax_cppflags
    MODULE_$1_LDFLAGS=$ax_ldflags
    PKG_$1_CPPFLAGS=$ax_cppflags
    PKG_$1_LDFLAGS=$ax_ldflags
    AC_MSG_RESULT([yes])
  fi
    
  AC_SUBST(MODULE_$1_CPPFLAGS)
  AC_SUBST(MODULE_$1_LDFLAGS)
  AC_SUBST(PKG_$1_CPPFLAGS)
  AC_SUBST(PKG_$1_LDFLAGS)

  HAVE_PKG_$1=${ax_module_enable}
  MAKE_MODULE_$1=${ax_module_enable}
  AC_SUBST(MAKE_MODULE_$1)

  if test ${HAVE_PKG_$1} = "yes" ; then
    ax_have_pkg_bool=1
  else
    ax_have_pkg_bool=0
  fi
  AC_DEFINE_UNQUOTED(HAVE_PKG_$1,
                     [$ax_have_pkg_bool],
                     [Define to 1 if the $1 module is available.])

  if test "$ENABLE_VERBOSE" = "yes"; then
    AC_MSG_NOTICE(MODULE_$1_CPPFLAGS = ${MODULE_$1_CPPFLAGS})
    AC_MSG_NOTICE(MODULE_$1_LDFLAGS = ${MODULE_$1_LDFLAGS})
    AC_MSG_NOTICE(MAKE_MODULE_$1 = ${MAKE_MODULE_$1})
    AC_MSG_NOTICE(PKG_$1_CPPFLAGS = ${PKG_$1_CPPFLAGS})
    AC_MSG_NOTICE(PKG_$1_LDFLAGS = ${PKG_$1_LDFLAGS})
    AC_MSG_NOTICE(HAVE_PKG_$1 = ${HAVE_PKG_$1})
  fi

#  We're putting these in configure.ac manually by now, for 
#  backwards compatability with older versions of automake.
#  AM_CONDITIONAL([MAKE_MODULE_$1], [test "$MAKE_MODULE_$1" = "yes"])
])
