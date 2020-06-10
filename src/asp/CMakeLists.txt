# Add path for custom modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}" )

include(Utilities) # Contains functions

#===============================================================================
# Configuration options

# --- Directory paths ---
set(BINARYBUILDER_INSTALL_DIR "" CACHE FILEPATH "Path to Binary Builder installation directory")
set(VISIONWORKBENCH_INSTALL_DIR "" CACHE FILEPATH "Path to Binary Builder installation directory")
set(ASP_ENABLE_SSE "1" CACHE BOOL "Allow SSE optimizations.")

if ("${BINARYBUILDER_INSTALL_DIR}" STREQUAL "")
  message(FATAL_ERROR "You need to set BINARYBUILDER_INSTALL_DIR")
endif()

# Make an abbreviation for this long name
set(bbdir ${BINARYBUILDER_INSTALL_DIR})

if ("${ISIS_DEPS_DIR}" STREQUAL "")
  message(FATAL_ERROR "You need to set ISIS_DEPS_DIR")
endif()

# Put the search for Boost early on, to ensure that the Conda version
# is found before we point to the BB folder.
# TODO: Make into a function!
# Define boost requirements
message("--------------------------------------------------------------")
message("Searching for boost...")
set(BOOST_MIN_VERSION "1.65")
set(REQUIRED_BOOST_LIBS program_options system filesystem regex date_time thread iostreams)
#set(Boost_DEBUG 1) # Uncomment for useful finding boost debug output
#if(BINARYBUILDER_INSTALL_DIR) # Look in BinaryBuilder directory
if(ISIS_DEPS_DIR) # Look in BinaryBuilder directory
  set(BOOST_ROOT       "${ISIS_DEPS_DIR}")
  set(BOOST_LIBRARYDIR "${ISIS_DEPS_DIR}/lib")
  set(BOOST_INCLUDEDIR "${ISIS_DEPS_DIR}/include")

  #DBoost_INCLUDE_DIR=$HOME/projects/BinaryBuilder/build_asp/install/include/boost-1_67

  #set(BOOST_ROOT       "${BINARYBUILDER_INSTALL_DIR}")
  #set(BOOST_LIBRARYDIR "${BINARYBUILDER_INSTALL_DIR}/lib")
  #set(BOOST_INCLUDEDIR "${BINARYBUILDER_INSTALL_DIR}/include/boost-1_67")

  set(Boost_DEBUG ON)
  set(Boost_DETAILED_FAILURE_MSG ON)
  #set(CMAKE_CXX_COMPILER_ARCHITECTURE_ID "x64")
  #set(Boost_NO_SYSTEM_PATHS ON) #This seems to confuse boost
endif()  
# Now search for Boost using the available information
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS ${REQUIRED_BOOST_LIBS})

if(Boost_FOUND)
    #set(Boost_LIBRARY_DIR ${BINARYBUILDER_INSTALL_DIR}/lib) # Override this value
    message("Successfully found Boost version ${Boost_VERSION}.")
    message("Boost include dir = ${Boost_INCLUDE_DIR}")
    message("Boost library dir = ${Boost_LIBRARY_DIRS}")
    message("Boost libraries   = ${Boost_LIBRARIES}")
    include_directories(${Boost_INCLUDE_DIR})
    link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)
message("--------------------------------------------------------------")

# Look for libraries in both of these BB locations.
#link_directories(${BINARYBUILDER_INSTALL_DIR}/lib)
#link_directories(${BINARYBUILDER_INSTALL_DIR}/lib64)

# If VW path is provided, use that instead of the BB install dir.
if (NOT VISIONWORKBENCH_INSTALL_DIR)
  set(VISIONWORKBENCH_INSTALL_DIR ${BINARYBUILDER_INSTALL_DIR})
endif()

# Add BB folders to CMAKE paths
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} ${BINARYBUILDER_INSTALL_DIR})
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${BINARYBUILDER_INSTALL_DIR})
SET(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH}  ${BINARYBUILDER_INSTALL_DIR})

# Libraries use paths like "asp/blah/blah.h" so we just need to add this one include path
include_directories( ../ )

# --- Custom options ---
option(BUILD_SHARED_LIBS "Produce shared libraries." TRUE)

# Use full length RPATHS in the installed files
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Make sure installed files can find installed libraries
SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} ${CMAKE_INSTALL_PREFIX}/lib ${ISIS_DEPS_DIR}/lib ${ISIS_DEPS_DIR}/lib64)

# --- Fixed options ---
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)

if (ASP_ENABLE_SSE)
    message(STATUS, "Enabling SSE")
else() # If disabled, set up for compatibility with older systems.
    message(STATUS, "Disaling SSE")
    set(CXXFLAGS "${CXXFLAGS} -mno-sse4.1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-sse4.1")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(ASP_HAVE_PKG_ICEBRIDGE 1)

if (APPLE)
  set(ASP_OSX_BUILD 1)
endif()

# Use CCache if it is available.
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  #message("Using ccache tool...")
  #set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  #set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)


if (UNIX AND NOT APPLE)
  execute_process(COMMAND ${CMAKE_C_COMPILER} -fuse-ld=gold -Wl,--version ERROR_QUIET OUTPUT_VARIABLE ld_version)
  if ("${ld_version}" MATCHES "GNU gold")
    message("Using gold linker...")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold -Wl,--disable-new-dtags")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold -Wl,--disable-new-dtags")
  endif()
endif()


#==================================================================================
# Find dependencies

set(REQUIRED     True )
set(NOT_REQUIRED False)

#TODO: Replace with FindVisionWorkbench module?
# Set up VisionWorkbench
message("--------------------------------------------------------------")
message(" Vision Workbench installation directory: ${VISIONWORKBENCH_INSTALL_DIR}")
set(VISIONWORKBENCH_INCLUDE_DIR ${VISIONWORKBENCH_INSTALL_DIR}/include)
set(VISIONWORKBENCH_LIBRARY_DIR ${VISIONWORKBENCH_INSTALL_DIR}/lib)
set(ASP_HAVE_PACKAGE_VW 1)

set(VISIONWORKBENCH_LIBRARIES)
set(VW_LIBNAMES VwBundleAdjustment  VwCore      VwImage          VwMosaic
                VwCamera            VwFileIO    VwInterestPoint  VwStereo
                VwCartography       VwGeometry  VwMath)

set(ext ".so")
if (APPLE)
  set(ext ".dylib")
endif()

foreach(n ${VW_LIBNAMES})
  set(f ${VISIONWORKBENCH_LIBRARY_DIR}/lib${n}${ext})
  set(VISIONWORKBENCH_LIBRARIES ${VISIONWORKBENCH_LIBRARIES} ${f})
endforeach(n)

include_directories(${VISIONWORKBENCH_INCLUDE_DIR})
#link_directories(${VISIONWORKBENCH_LIBRARY_DIR})


# TODO: Add DLL to our main Boost installation.
#include_directories("/home6/oalexan1/projects/usgscsm/boost_1_70_0/boost")
#include_directories("/home/smcmich1/repo/CSM-CameraModel/install/include")
#include_directories("/home/smcmich1/repo/CSM-Swig/install/include")
#set(CSM_LIBRARIES "/home/smcmich1/repo/CSM-Swig/install/lib/libfixturecsm.so")

#include_directories("/home/smcmich1/repo/csm-3.0.3.1/linux64/include")
#set(CSM_LIBRARIES "/home/smcmich1/repo/csm-3.0.3.1/linux64/lib/libcsmapi.so.3")

message("--------------------------------------------------------------")
message("Searching for QT...")
# QT is complicated to use and should go through find_package
set(QT_QMAKE_EXECUTABLE  ${ISIS_DEPS_DIR}/bin/qmake)
set(QT_MOC_EXECUTABLE    ${ISIS_DEPS_DIR}/bin/moc)
set(QT_RCC_EXECUTABLE    ${ISIS_DEPS_DIR}/bin/rcc)
set(QT_UIC_EXECUTABLE    ${ISIS_DEPS_DIR}/bin/uic)

set(REQUIRED_QT_COMPONENTS  Core                Concurrent
                            Gui                 Multimedia
                            MultimediaWidgets   Network
                            OpenGL              PrintSupport
                            Qml                 Quick
                            Script              ScriptTools
                            Sql                 Svg
                            Test                Widgets
                            Xml                 XmlPatterns)
find_package(Qt5 5.9.6 COMPONENTS ${REQUIRED_QT_COMPONENTS} REQUIRED)

if(Qt5_FOUND)
    # We need this to be able to include headers produced by uic in our
    # code (CMAKE_BINARY_DIR holds a path to the build directory, while
    # INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
    INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} )
    set (ASP_HAVE_PKG_QT 1)
    message("QT found: ${Qt5Core_INCLUDE_DIRS}")
endif(Qt5_FOUND)
message("--------------------------------------------------------------")

find_external_library(QWT ${ISIS_DEPS_DIR} "" "qwt" REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Dependencies for VW

if (APPLE) # Use the system version
  find_package(LAPACK)
  find_package(BLAS)
else() # Use our BB compiled version
  find_external_library(LAPACK ${ISIS_DEPS_DIR} "" "lapack;blas" REQUIRED)
endif()
if(LAPACK_FOUND)
  set(VW_HAVE_PKG_FLAPACK 1) # --> This specifies that it is a Fortran derived version
                             #      How to check this if the user provides Fortran?
endif()

find_external_library(FLANN ${ISIS_DEPS_DIR} "" "flann_cpp" REQUIRED)
find_external_library(Z ${ISIS_DEPS_DIR} "" "z" REQUIRED)
find_external_library(PROJ4 ${ISIS_DEPS_DIR} "" "proj"      REQUIRED)

find_external_library(GDAL ${bbdir} "" "gdal"      REQUIRED)
set(ILMBASE_LIB_NAMES Half Iex Imath IlmThread)
find_external_library(ILMBASE ${ISIS_DEPS_DIR} "" "${ILMBASE_LIB_NAMES}" REQUIRED)
find_external_library(OPENEXR ${ISIS_DEPS_DIR} "OpenEXR" "IlmImf;IlmThread" REQUIRED)

# Dependencies for ASP and ISIS
find_external_library(SPICE ${ISIS_DEPS_DIR} "cspice" "cspice" REQUIRED)
find_external_library(GEOID ${bbdir} "" "egm2008" REQUIRED)
find_external_library(XERCESC ${ISIS_DEPS_DIR} "" "xerces-c" REQUIRED)
find_external_library(PROTOBUF ${ISIS_DEPS_DIR} "" "protobuf" REQUIRED)
find_external_library(EMBREE ${ISIS_DEPS_DIR} "" "embree" REQUIRED)
find_external_library(EIGEN ${ISIS_DEPS_DIR} "eigen3" "" REQUIRED)
find_external_library(CERES ${ISIS_DEPS_DIR} "ceres" "ceres" REQUIRED)
find_external_library(LIBPOINTMATCHER ${bbdir} "" "pointmatcher" REQUIRED)
find_external_library(FASTGLOBALREGISTRATION ${bbdir} "FastGlobalRegistration" "FastGlobalRegistrationLib" REQUIRED)
find_external_library(LIBLAS ${bbdir} "" "las" REQUIRED)
find_external_library(LASZIP ${bbdir} "laszip" "laszip" REQUIRED)
find_external_library(GFLAGS ${ISIS_DEPS_DIR} "gflags" "gflags" REQUIRED)
find_external_library(GLOG ${ISIS_DEPS_DIR} "glog" "glog" REQUIRED)
find_external_library(ARMADILLO ${ISIS_DEPS_DIR} "" "armadillo" REQUIRED)

find_external_library(ISIS ${ISIS_DEPS_DIR} "isis" "isis3" REQUIRED)
include_directories("${ISIS_DEPS_DIR}/include/isis")
include_directories("${bbdir}/include/isis") # temporary, remove when ISIS folks put it above

if(ASP_HAVE_PKG_ISIS)
  # Pull the ISIS version. It is on the first line before the first space.
  file(STRINGS "${ISIS_DEPS_DIR}/version" MY_STRING LIMIT_COUNT 1)
  string(REPLACE " " ";" MY_LIST ${MY_STRING}) # split by space and store into MY_LIST
  list(GET MY_LIST 0 ASP_ISIS_VERSION)
  set(ASP_HAVE_PKG_ISISIO 1)
  message(STATUS "Found ISIS version: ${ASP_ISIS_VERSION}")
endif()

find_external_library(CSM ${bbdir} "libusgscsm" "csmapi" REQUIRED)
#include_directories("${bbdir}/include")
#set(CSM_LIBRARIES "/home6/oalexan1/projects/usgscsm/build/csm/libcsmapi.so.3")

set(PCL_LIB_NAMES pcl_common pcl_features pcl_filters pcl_io_ply pcl_io
                  pcl_kdtree pcl_keypoints pcl_ml pcl_octree pcl_recognition
                  pcl_registration pcl_sample_consensus pcl_search pcl_segmentation
                  pcl_stereo pcl_surface pcl_tracking)
find_external_library(PCL ${ISIS_DEPS_DIR} "pcl" "${PCL_LIB_NAMES}" REQUIRED)

# TODO: Which do we actually need!
#find_external_library(JPEG ${bbdir} "" "jpeg" NOT_REQUIRED)
#find_external_library(PNG ${bbdir} "" "png;png16" NOT_REQUIRED)
#find_external_library(TIFF ${bbdir} "" "tiff" NOT_REQUIRED)


set(OPENCV_LIB_NAMES opencv_calib3d     opencv_reg
                     opencv_core        opencv_shape
                     opencv_features2d  opencv_stitching
                     opencv_flann       opencv_superres
                     opencv_hdf         opencv_surface_matching
                     opencv_highgui     opencv_videoio
                     opencv_imgcodecs   opencv_video
                     opencv_imgproc     opencv_xfeatures2d
                     opencv_ml          opencv_ximgproc
                     opencv_objdetect   opencv_xobjdetect
                     opencv_photo       opencv_xphoto)
find_external_library(OPENCV ${ISIS_DEPS_DIR} "" "${OPENCV_LIB_NAMES}" NOT_REQUIRED)

# For convenience, list some libraries that VW and ISIS depend on.
set(VW_3RD_PARTY_LIBS   ${Z_LIBRARIES} ${OPENCV_LIBRARIES} ${ILMBASE_LIBRARIES} ${OPENEXR_LIBRARIES}
                        ${GDAL_LIBRARIES} ${LAPACK_LIBRARIES} ${Boost_LIBRARIES}  ${PROJ4_LIBRARIES}
                        ${FLANN_LIBRARIES} )
set(ISIS_3RD_PARTY_LIBS Qt5::Core  ${EMBREE_LIBRARIES} ${PROTOBUF_LIBRARIES} ${QWT_LIBRARIES} ${ARMADILLO_LIBRARIES})

message("--------------------------------------------------------------")

# Now that we have found all our external dependencies, generate a config.h file
include("GenerateConfig.cmake")

#TODO: Needed?
#set(HAVE_PKG_HDF)
#set(HAVE_PKG_LIBKML)

# TODO: Do we need this?
#set(ENABLE_EXCEPTIONS True)



message("--------------------------------------------------------------")
#message("Listing all include directories...")
#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(STATUS "dir='${dir}'")
#endforeach()
#message("--------------------------------------------------------------")

# TODO: Do we need these?
# Add all the header files at the top level to the install command
get_all_source_files("." MAIN_HEADER_FILES)
foreach(f ${MAIN_HEADER_FILES})
  INSTALL(FILES ${f} DESTINATION include/asp)
endforeach()

#=========================================================================
# Define each of the libaries

# --- ASP_CORE ------------------------------------------------------------
get_all_source_files( "Core"       ASP_CORE_SRC_FILES)
get_all_source_files( "Core/tests" ASP_CORE_TEST_FILES)
set(ASP_CORE_LIB_DEPENDENCIES ${VW_3RD_PARTY_LIBS} ${VISIONWORKBENCH_LIBRARIES} ${LIBLAS_LIBRARIES} ${LASZIP_LIBRARIES})

# --- ASP_SPICEIO ------------------------------------------------------------
get_all_source_files( "SpiceIO"       ASP_SPICEIO_SRC_FILES)
get_all_source_files( "SpiceIO/tests" ASP_SPICEIO_TEST_FILES)
set(ASP_SPICEIO_LIB_DEPENDENCIES AspCore ${SPICE_LIBRARIES})

# --- ASP_ISISIO ------------------------------------------------------------
get_all_source_files( "IsisIO"       ASP_ISISIO_SRC_FILES)
get_all_source_files( "IsisIO/tests" ASP_ISISIO_TEST_FILES)
set(ASP_ISISIO_LIB_DEPENDENCIES  AspCore ${ISIS_3RD_PARTY_LIBS} ${ISIS_LIBRARIES}
                                 ${VW_3RD_PARTY_LIBS} ${VISIONWORKBENCH_LIBRARIES})

# --- ASP_CAMERA ------------------------------------------------------------
get_all_source_files( "Camera"       ASP_CAMERA_SRC_FILES)
get_all_source_files( "Camera/tests" ASP_CAMERA_TEST_FILES)
set(ASP_CAMERA_LIB_DEPENDENCIES AspCore ${XERCESC_LIBRARIES} ${CSM_LIBRARIES} ${CMAKE_DL_LIBS})

# --- ASP_SESSIONS ------------------------------------------------------------
## This code is more complicated and is specified in the lower level file
get_all_source_files( "Sessions"       ASP_SESSIONS_SRC_FILES)
get_all_source_files( "Sessions/tests" ASP_SESSIONS_TEST_FILES)
set(ASP_SESSIONS_LIB_DEPENDENCIES AspCore AspSpiceIO AspIsisIO AspCamera)

# --- ASP_GUI ------------------------------------------------------------
get_all_source_files( "GUI"       ASP_GUI_SRC_FILES)
get_all_source_files( "GUI/tests" ASP_GUI_TEST_FILES)
set(ASP_GUI_LIB_DEPENDENCIES AspCore Qt5::Core Qt5::Gui Qt5::Widgets ${QWT_LIBRARIES})


#=========================================================================
# Add all of the library subdirectories

add_subdirectory(Core)
add_subdirectory(SpiceIO)
if(ASP_HAVE_PKG_ISISIO)
  add_subdirectory(IsisIO)
endif()
add_subdirectory(Camera)
add_subdirectory(Sessions)
add_subdirectory(GUI)

# Add the non-library subdirectories

add_subdirectory(Python)
add_subdirectory(Tools)
add_subdirectory(WVCorrect)
add_subdirectory(Hidden)
add_subdirectory(IceBridge)











