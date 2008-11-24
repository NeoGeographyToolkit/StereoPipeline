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
