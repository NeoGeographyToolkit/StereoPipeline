
# TODO:  This was copied from here: https://github.com/alacarte-maps/alacarte/tree/master/cmake
#        Which uses the Affero General Public License.
#        If we end up using this make sure that is ok!


# - Try to find the CAIRO library
# Once done this will define
#
# CAIRO_ROOT_DIR - Set this variable to the root installation of CAIRO
#
# Read-Only variables:
# CAIRO_FOUND - system has the CAIRO library
# CAIRO_INCLUDE_DIR - the CAIRO include directory
# CAIRO_LIBRARIES - The libraries needed to use CAIRO
# CAIRO_VERSION - This is set to $major.$minor.$revision (eg. 0.9.8)
#
# Copyright (c) 2008 Joshua L. Blocher <verbalshadow at gmail dot com>
# Copyright (c) 2012 Dmitry Baryshnikov <polimax at mail dot ru>
# Copyright (c) 2013 Michael Pavlyshko <pavlushko at tut dot by>
#
# Distributed under the OSI-approved BSD License
#

if (NOT WIN32)
    find_package(PkgConfig)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(_CAIRO cairo)

        SET(CAIRO_VERSION ${_CAIRO_VERSION})
        STRING (REGEX REPLACE "([0-9]+).([0-9]+).([0-9]+)" "\\1" num "${CAIRO_VERSION}")
        MATH (EXPR CAIRO_VERSION_V "${num}")
        STRING (REGEX REPLACE "([0-9]+).([0-9]+).([0-9]+)" "\\2" num "${CAIRO_VERSION}")
        MATH (EXPR CAIRO_VERSION_MAJOR "${num}")
        STRING (REGEX REPLACE "([0-9]+).([0-9]+).([0-9]+)" "\\3" num "${CAIRO_VERSION}")
        MATH (EXPR CAIRO_VERSION_MINOR "${num}")
    endif (PKG_CONFIG_FOUND)
endif (NOT WIN32)

SET(_CAIRO_ROOT_HINTS
      $ENV{CAIRO}
      ${CMAKE_FIND_ROOT_PATH}
      ${CAIRO_ROOT_DIR}
)

SET(_CAIRO_ROOT_PATHS
    ${CMAKE_FIND_ROOT_PATH}
    $ENV{CAIRO}/src
    /usr
    /usr/local
)

SET(_CAIRO_ROOT_HINTS_AND_PATHS
  HINTS ${_CAIRO_ROOT_HINTS}
  PATHS ${_CAIRO_ROOT_PATHS}
)

FIND_PATH(CAIRO_INCLUDE_DIR
    NAMES
        cairo.h
    HINTS
        ${_CAIRO_INCLUDEDIR}
        ${_CAIRO_ROOT_HINTS_AND_PATHS}
    PATH_SUFFIXES
        include
        "include/cairo"
)

FIND_LIBRARY(CAIRO_LIBRARY
    NAMES
        cairo
    HINTS
        ${_CAIRO_LIBDIR}
        ${_CAIRO_ROOT_HINTS_AND_PATHS}
    PATH_SUFFIXES
        "lib"
        "local/lib"
)

MARK_AS_ADVANCED(CAIRO_LIBRARY)
SET(CAIRO_LIBRARIES ${CAIRO_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CAIRO "Could NOT find CAIRO, try to set the path to CAIRO root folder in the system variable CAIRO"
    CAIRO_LIBRARIES
    CAIRO_INCLUDE_DIR
)

MARK_AS_ADVANCED(CAIRO_INCLUDE_DIR CAIRO_LIBRARIES)
