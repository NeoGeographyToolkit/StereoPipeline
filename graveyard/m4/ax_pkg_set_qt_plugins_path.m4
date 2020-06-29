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

# Create a qt.conf in Tools/.libs so that stereo_gui can 
# start properly. 
# Usage: AX_PKG_SET_QT_PLUGINS_PATH]([base_system_path])
AC_DEFUN([AX_PKG_SET_QT_PLUGINS_PATH],
[
  case $host_os in
  	*linux*)
		TOOL_LIBS_DIR=$(pwd)/src/asp/Tools/.libs
		mkdir -p $TOOL_LIBS_DIR
		QT_CONFIG_FILE=$TOOL_LIBS_DIR/qt.conf
		echo '@<:@Paths@:>@' > $QT_CONFIG_FILE
		echo 'Plugins='$1'/plugins' >> $QT_CONFIG_FILE
		echo Created Qt config file: $QT_CONFIG_FILE 
		echo with the contents:
		cat $QT_CONFIG_FILE
	   ;;
	*) ;;
esac
])
