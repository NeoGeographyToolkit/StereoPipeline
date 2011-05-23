dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006-2011 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__


dnl AX_INSTEAD_IF(RUN-IF-NONEMPTY, MESSAGE)
dnl if RUN-IF-NONEMPTY is nonempty, print the MESSAGE as a warning
dnl else, print the MESSAGE as an error
AC_DEFUN([AX_INSTEAD_IF],
[
    m4_ifval([$1],
        AC_MSG_WARN([$2]); [$1],
        AC_MSG_ERROR([$2])
    )
])

