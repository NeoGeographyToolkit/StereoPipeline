# __BEGIN_LICENSE__
#  Copyright (c) 2009-2013, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__


########################################################################
# sources
########################################################################

bin_PROGRAMS =
bin_SCRIPTS =
libexec_SCRIPTS = asp_cmd_utils.py asp_file_utils.py asp_geo_utils.py  asp_alg_utils.py \
		asp_image_utils.py asp_string_utils.py asp_system_utils.py

all_scripts = $(libexec_SCRIPTS)
CLEANFILES =

##############################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

SUBDIRS = .

includedir = $(prefix)/include/asp/Python

include $(top_srcdir)/config/rules.mak
