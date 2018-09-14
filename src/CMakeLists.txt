
include(Utilities) # Contains functions

#===============================================================================
# Project information

project (StereoPipeline)

set(PACKAGE_VERSION "2.6.1")
set(PACKAGE_NAME "Stereo Pipeline")
set(PACKAGE_STRING "${PACKAGE_NAME} ${PACKAGE_VERSION}")
set(PACKAGE_BUGREPORT "scott.t.mcmichael@nasa.gov")

#===============================================================================

# Set the install path if not specified by the user.
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "default install path" FORCE )
endif()
message("Cmake install directory set to: ${CMAKE_INSTALL_PREFIX}")

enable_testing() # Needed for unit tests to work


# Add command to build the instructions pdf file
# - TODO: This will need more work!
add_custom_target(workbook pdflatex asp_book.tex
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




# Add all of our code
add_subdirectory(asp)






