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

