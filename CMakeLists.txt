if(APPLE)
    # Needed for conda build on OSX with conda
    cmake_policy(SET CMP0025 NEW) 
endif(APPLE)
    
cmake_minimum_required (VERSION 3.15)

# Make it possible to append to these from the command line
set(CMAKE_CXX_FLAGS "" CACHE STRING "")
set(CMAKE_C_FLAGS "" CACHE STRING "")

# cmake fails without the -L flag in CMAKE_CXX_FLAGS, oddly enough.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -DNDEBUG")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_EXE_LINKER_FLAGS "-lm -pthread -lpthread")

if(APPLE)
    # A workaround for the clang included with conda build
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mlinker-version=305")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mlinker-version=305")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mlinker-version=305")
else(APPLE)
    # On Linux need to link to rt and libgcc_s
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt -lgcc_s")
endif(APPLE)

# Tell cmake to look in the /cmake folder.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Make sure we do a release type
set(CMAKE_BUILD_TYPE "Release")
message("Setting CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")

project(StereoPipeline)

add_subdirectory(src)

install(FILES "AUTHORS.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "INSTALLGUIDE.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "LICENSE" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "NEWS.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "README.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "stereo.default.example" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "THIRDPARTYLICENSES.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY "examples" DESTINATION ${CMAKE_INSTALL_PREFIX})

# Install the plugins directory and the files in it
install(DIRECTORY DESTINATION ${CMAKE_INSTALL_PREFIX}/plugins/stereo)
install(FILES "plugins/stereo/plugin_list.txt" DESTINATION ${CMAKE_INSTALL_PREFIX}/plugins/stereo)

# Make a directory for wv_correct data and copy that data to it
install(DIRECTORY DESTINATION ${CMAKE_INSTALL_PREFIX}/share/wv_correct)
install(FILES "src/asp/WVCorrect/ms_correction_lookup.txt" DESTINATION ${CMAKE_INSTALL_PREFIX}/share/wv_correct)
install(FILES "src/asp/WVCorrect/WV02_BAND3_CCD_CORR.tif" DESTINATION ${CMAKE_INSTALL_PREFIX}/share/wv_correct)

# Install the default CASP-GO params
install(FILES "src/asp/Gotcha/CASP-GO_params.xml" DESTINATION ${CMAKE_INSTALL_PREFIX}/share)
