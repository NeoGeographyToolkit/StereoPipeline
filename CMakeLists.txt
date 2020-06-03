cmake_minimum_required (VERSION 3.2)

# Tell cmake to look in the /cmake folder.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

## Add precompiled header tool
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/CMakePCHCompiler)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -std=c++11 -lm -pthread -lpthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -lm -pthread -lpthread")

add_subdirectory(src)

install(FILES "AUTHORS.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "INSTALLGUIDE.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "LICENSE" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "NEWS.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "README.rst" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "stereo.default.example" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES "THIRDPARTYLICENSES" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY "examples" DESTINATION ${CMAKE_INSTALL_PREFIX})
