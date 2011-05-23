dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


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
      AX_LOG([Find Files: Looking for ${filename} in ${path}])
      ax_find_files_paths=`ls $pathname 2>/dev/null`
      if test ! -z "$ax_find_files_paths" ; then
        AX_LOG([Find Files: Found ${ax_find_files_paths}])
      else
        AX_LOG([Find Files: Not Found])
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
