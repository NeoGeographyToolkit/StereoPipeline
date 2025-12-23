include(Utilities) # Contains functions

# Verify that we have a new enough compiler.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0.0")
    message(FATAL_ERROR "GCC version >= 5.0.0 is required!")
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.1.0")
    message(FATAL_ERROR "Clang version >= 8.1.0 is required!")
  endif()
else()
  message(FATAL_ERROR "Unknown compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()


#===============================================================================
# Project information

project (StereoPipeline)

# The line below is used by the docs/ so don't change it
# without testing the result.  Versioning is based on https://semver.org
set(PACKAGE_VERSION "3.6.0")
#set(PACKAGE_VERSION "3.7.0-alpha")
set(PACKAGE_NAME "NASA Ames Stereo Pipeline")
set(PACKAGE_STRING "${PACKAGE_NAME} ${PACKAGE_VERSION}")
set(PACKAGE_BUGREPORT "oleg.alexandrov@nasa.gov, scott.t.mcmichael@nasa.gov")

# Fetch the Git hash and store in ASP_COMMIT_ID
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE ASP_COMMIT_ID
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Find the build date in UTC. Will agree with the daily build date.
# That because the build is started late at night in PST, which is
# next morning in UTC, and by the time it is published it is next day
# in PST, but midday or early evening in UTC.
string(TIMESTAMP ASP_BUILD_DATE "%Y-%m-%d" UTC)

#===============================================================================

# Set the install path if not specified by the user.
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set (CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "default install path" FORCE)
endif()
message("Cmake install directory set to: ${CMAKE_INSTALL_PREFIX}")

enable_testing() # Needed for unit tests to work

# Add command to build the instructions pdf file
add_custom_target(workbook make
                  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/docs/book")

# Add command to build doxygen files
add_custom_target(doxygen doxygen
                  COMMAND ln -s ${CMAKE_SOURCE_DIR}/docs/doxygen/index.html ${CMAKE_SOURCE_DIR}/docs/doxygen-output.html
                  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

# Add external dependencies which still need to be built

# Build GTest
include_directories(../thirdparty/gtest/include)
#include_directories(../thirdparty/gtest/)
add_library(gtest      ../thirdparty/gtest/src/gtest-all.cc)
add_library(gtest_main ../thirdparty/gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest) 

target_compile_definitions(gtest PRIVATE GTEST_USE_OWN_TR1_TUPLE=1)
target_compile_definitions(gtest_main PRIVATE GTEST_USE_OWN_TR1_TUPLE=1)

# Add all of our code
add_subdirectory(asp)






