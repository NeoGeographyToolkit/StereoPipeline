dnl __BEGIN_LICENSE__
dnl  Copyright (c) 2009-2013, United States Government as represented by the
dnl  Administrator of the National Aeronautics and Space Administration. All
dnl  rights reserved.
dnl
dnl  The NGT platform is licensed under the Apache License, Version 2.0 (the
dnl  "License"); you may not use this file except in compliance with the
dnl  License. You may obtain a copy of the License at
dnl  http://www.apache.org/licenses/LICENSE-2.0
dnl
dnl  Unless required by applicable law or agreed to in writing, software
dnl  distributed under the License is distributed on an "AS IS" BASIS,
dnl  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
dnl  See the License for the specific language governing permissions and
dnl  limitations under the License.
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

