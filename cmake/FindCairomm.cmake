
# TODO:  This was copied from here: https://github.com/alacarte-maps/alacarte/tree/master/cmake
#        Which uses the Affero General Public License.
#        If we end up using this make sure that is ok!


# - Try to find CAIROMM
# Once done this will define
#
#  CAIROMM_ROOT_DIR - Set this variable to the root installation of CAIROMM
#  CAIROMM_FOUND - system has CAIROMM
#  CAIROMM_INCLUDE_DIR - the CAIROMM include directory
#  CAIROMM_LIBRARIES - Link these to use CAIROMM
#
#  Copyright (c) 2008 Joshua L. Blocher <verbalshadow at gmail dot com>
#  Copyright (c) 2012 Dmitry Baryshnikov <polimax at mail dot ru>
#  Copyright (c) 2013 Michael Pavlyshko <pavlushko at tut dot by>
#
#  Distributed under the OSI-approved BSD License
#

if (NOT WIN32)
    find_package(PkgConfig)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(_CAIROMM cairomm-1.0)
    endif (PKG_CONFIG_FOUND)
endif (NOT WIN32)

SET(_CAIROMM_ROOT_HINTS
    $ENV{CAIROMM}
    ${CMAKE_FIND_ROOT_PATH}
    ${CAIROMM_ROOT_DIR}
) 

SET(_CAIROMM_ROOT_PATHS
    ${CMAKE_FIND_ROOT_PATH}
    $ENV{CAIROMM}/src
    /usr
    /usr/local
)

SET(_CAIROMM_ROOT_HINTS_AND_PATHS
    HINTS ${_CAIROMM_ROOT_HINTS}
    PATHS ${_CAIROMM_ROOT_PATHS}
)

FIND_PATH(CAIROMM_INCLUDE_DIR
    NAMES
        cairomm/cairomm.h
    HINTS
        ${_CAIROMM_INCLUDEDIR}
        ${_CAIROMM_ROOT_HINTS_AND_PATHS}
    PATH_SUFFIXES
        include
        "include/cairomm-1.0"
)

FIND_LIBRARY(CAIROMM_LIBRARY
    NAMES
        cairomm
        cairomm-1.0
    ${_CAIROMM_ROOT_HINTS_AND_PATHS}
    PATH_SUFFIXES
        "lib"
        "local/lib"
)

SET(CAIROMM_LIBRARIES ${CAIROMM_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CAIROMM "Could NOT find CAIROMM, try to set the path to CAIROMM root folder in the system variable CAIROMM"
    CAIROMM_LIBRARIES
    CAIROMM_INCLUDE_DIR
)

MARK_AS_ADVANCED(CAIROMM_INCLUDE_DIR CAIROMM_LIBRARY)
