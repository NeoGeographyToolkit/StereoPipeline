# Add path for custom modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}" )

include(Utilities) # Contains functions

# Configuration options

# Directory paths
set(VISIONWORKBENCH_INSTALL_DIR "" CACHE FILEPATH "Path to the user-built and installed VisionWorkbench")
set(ASP_DEPS_DIR "" CACHE FILEPATH "Path to the conda environment that has the ASP dependencies")

if ("${ASP_DEPS_DIR}" STREQUAL "")
  message(FATAL_ERROR "You need to set ASP_DEPS_DIR")
endif()

# Where to search for dependencies
set(CMAKE_PREFIX_PATH "${ASP_DEPS_DIR}")

# This needed for some libs represented as -lmylib
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${ASP_DEPS_DIR}/lib")

find_package(Threads REQUIRED)

# Put the search for Boost early on, to ensure that the Conda version
# is found before we point to the BB folder.
message("Searching for Boost.")
set(REQUIRED_BOOST_LIBS program_options system filesystem regex date_time thread iostreams)
if (ASP_DEPS_DIR) # Look in that directory
  set(Boost_ROOT       "${ASP_DEPS_DIR}")
  set(Boost_LIBRARYDIR "${ASP_DEPS_DIR}/lib")
  set(Boost_INCLUDEDIR "${ASP_DEPS_DIR}/include")
  set(Boost_NO_SYSTEM_PATHS ON) # Do not search anywhere else
  set(Boost_DEBUG ON)
  set(Boost_DETAILED_FAILURE_MSG ON)
  #set(CMAKE_CXX_COMPILER_ARCHITECTURE_ID "x64")
endif()  
# Now search for Boost using the available information
find_package(Boost REQUIRED COMPONENTS ${REQUIRED_BOOST_LIBS})

if(Boost_FOUND)
    message("Successfully found Boost.")
    message("Boost include dir = ${Boost_INCLUDE_DIR}")
    message("Boost library dir = ${Boost_LIBRARY_DIRS}")
    message("Boost libraries   = ${Boost_LIBRARIES}")
    include_directories(${Boost_INCLUDE_DIR})
    link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

# If VW was not built and installed separately, it should have been
# built and installed in ASP_DEPS_DIR.
if (NOT VISIONWORKBENCH_INSTALL_DIR)
  set(VISIONWORKBENCH_INSTALL_DIR ${ASP_DEPS_DIR})
endif()

# Custom options
option(BUILD_SHARED_LIBS "Produce shared libraries." TRUE)

# Use full length RPATHS in the installed files
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Make sure installed files can find installed libraries
if (APPLE)
  SET(CMAKE_INSTALL_RPATH "@loader_path/../lib" ${CMAKE_INSTALL_RPATH})
elseif (UNIX)
  SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib" ${CMAKE_INSTALL_RPATH})
endif()
SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} ${ASP_DEPS_DIR}/lib)

# Fixed options
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)

# Enable/disable SSE optimizations
if (CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(VW_ENABLE_SSE 0)
else()
  set(VW_ENABLE_SSE 1)
endif()
if (VW_ENABLE_SSE)
  set(CXXFLAGS "${CXXFLAGS} -msse4.1")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
  message(STATUS "Enabling SSE")
else()
  if (CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    message(STATUS "SSE optimizations are not available on Mac Arm.")
  else()
    # Older Intel CPUs do not support SSE4.1
    message(STATUS, "Disabling SSE.")
    set(CXXFLAGS "${CXXFLAGS} -mno-sse4.1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-sse4.1")
  endif()
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

# Find dependencies

set(REQUIRED     True )
set(NOT_REQUIRED False)

#TODO: Replace with FindVisionWorkbench module?
# Set up VisionWorkbench
message("Vision Workbench installation directory: ${VISIONWORKBENCH_INSTALL_DIR}")
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

message("Searching for Qt.")
# QT is complicated to use and should go through find_package
set(QT_QMAKE_EXECUTABLE  ${ASP_DEPS_DIR}/bin/qmake)
set(QT_MOC_EXECUTABLE    ${ASP_DEPS_DIR}/bin/moc)
set(QT_RCC_EXECUTABLE    ${ASP_DEPS_DIR}/bin/rcc)
set(QT_UIC_EXECUTABLE    ${ASP_DEPS_DIR}/bin/uic)

set(REQUIRED_QT_COMPONENTS  Core                Concurrent
                            Gui                 Multimedia
                            MultimediaWidgets   Network
                            OpenGL              PrintSupport
                            Qml                 Quick
                            Script              ScriptTools
                            Sql                 Svg
                            Test                Widgets
                            Xml                 XmlPatterns)
find_package(Qt5 COMPONENTS ${REQUIRED_QT_COMPONENTS} REQUIRED)

if(Qt5_FOUND)
    # We need this to be able to include headers produced by uic in our
    # code (CMAKE_BINARY_DIR holds a path to the build directory, while
    # INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
    set (ASP_HAVE_PKG_QT 1)
    message("Qt found: ${Qt5Core_INCLUDE_DIRS}")
endif(Qt5_FOUND)

find_external_library(QWT ${ASP_DEPS_DIR} "" "qwt" REQUIRED)

# TODO(oalexan1): Figure out why clang does not find OpenMP
# Also try to to understand why OpenMP cannot be found
# without all the "set" logic below.
if (APPLE)
    set(OpenMP_C_LIB_NAMES "gomp")
    set(OPENMP_INCLUDES "${ASP_DEPS_DIR}/include")
    set(OpenMP_C_FLAGS "-fopenmp")
    set(OpenMP_gomp_LIBRARY "${ASP_DEPS_DIR}/lib/libgomp${CMAKE_SHARED_LIBRARY_SUFFIX}")
    set(OpenMP_CXX_FLAGS "-fopenmp")
    set(OpenMP_CXX_LIB_NAMES "gomp")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_gomp_LIBRARY}")
else()
    find_package(OpenMP REQUIRED)
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      
find_external_library(LAPACK ${ASP_DEPS_DIR} "" "lapack;blas" REQUIRED)
set(VW_HAVE_PKG_FLAPACK 1) # This specifies that it is a Fortran derived version

# Required for GLog
add_definitions(-DGLOG_USE_GLOG_EXPORT)

find_external_library(FLANN   ${ASP_DEPS_DIR} "" "flann_cpp" REQUIRED)
find_external_library(LZ4     ${ASP_DEPS_DIR} "" "lz4"       REQUIRED)
find_external_library(Z       ${ASP_DEPS_DIR} "" "z"         REQUIRED)
find_external_library(PROJ4   ${ASP_DEPS_DIR} "" "proj"      REQUIRED)
find_external_library(JPEG    ${ASP_DEPS_DIR} "" "jpeg"      REQUIRED)
find_external_library(PNG     ${ASP_DEPS_DIR} "" "png;png16" REQUIRED)
find_external_library(TIFF    ${ASP_DEPS_DIR} "" "tiff"      REQUIRED)
find_external_library(GEOTIFF ${ASP_DEPS_DIR} "" "geotiff"   REQUIRED)
find_external_library(GDAL    ${ASP_DEPS_DIR} "" "gdal"      REQUIRED)
# set(ILMBASE_LIB_NAMES Half Iex Imath IlmThread)
# find_external_library(ILMBASE ${ASP_DEPS_DIR} "" "${ILMBASE_LIB_NAMES}" REQUIRED)
# find_external_library(OPENEXR ${ASP_DEPS_DIR} "OpenEXR" "IlmImf;IlmThread" REQUIRED)

find_external_library(SPICE ${ASP_DEPS_DIR} "cspice" "cspice" REQUIRED)
# find_external_library(GEOID ${ASP_DEPS_DIR} "" "egm2008" REQUIRED)
find_external_library(XERCESC ${ASP_DEPS_DIR} "" "xerces-c" REQUIRED)
find_external_library(PROTOBUF ${ASP_DEPS_DIR} "" "protobuf" REQUIRED)
# find_external_library(EMBREE ${ASP_DEPS_DIR} "" "embree3" REQUIRED)

find_external_library(EIGEN ${ASP_DEPS_DIR} "eigen3" "" REQUIRED)
find_external_library(CERES ${ASP_DEPS_DIR} "ceres" "ceres" REQUIRED)
# find_external_library(LIBNABO ${ASP_DEPS_DIR} "nabo" "nabo" REQUIRED)
# find_external_library(LIBPOINTMATCHER ${ASP_DEPS_DIR} "" "pointmatcher" REQUIRED)
# find_external_library(FASTGLOBALREGISTRATION ${ASP_DEPS_DIR} "FastGlobalRegistration" "FastGlobalRegistrationLib" REQUIRED)
find_external_library(GFLAGS ${ASP_DEPS_DIR} "gflags" "gflags" REQUIRED)
find_external_library(GLOG ${ASP_DEPS_DIR} "glog" "glog" REQUIRED)
# find_external_library(ARMADILLO ${ASP_DEPS_DIR} "" "armadillo" REQUIRED)
# find_external_library(ISIS ${ASP_DEPS_DIR} "isis" "isis" REQUIRED)

# if(ASP_HAVE_PKG_ISIS)
#   # Pull the ISIS version. It is on the first line before the first space.
#   file(STRINGS "${ASP_DEPS_DIR}/isis_version.txt" ISIS_VERSION_LINE LIMIT_COUNT 1)
#   # Split by space, save to list
#   string(REPLACE " " ";" ISIS_VERSION_LIST ${ISIS_VERSION_LINE}) 
#   # Get the first element
#   list(GET ISIS_VERSION_LIST 0 ISIS_VERSION)
#   set(ASP_HAVE_PKG_ISISIO 1)
#   message(STATUS "Found ISIS version: ${ISIS_VERSION}")
    
  # Some ISIS headers include Qt headers in a way that requires the include below
  include_directories("${ASP_DEPS_DIR}/include/qt")
  include_directories("${ASP_DEPS_DIR}/include/qt/QtCore")
  include_directories("${ASP_DEPS_DIR}/include/qt/QtXml")
# endif()

find_external_library(CSM ${ASP_DEPS_DIR} "csm" "csmapi" REQUIRED)

# Link to libusgscsm. This makes it disappear from the list of CSM
# plugins, presumably because it is found at link time now,
# but things still work. This is is necessary since we need to peek into
# usgscsm by bypassing the csm interface.
# Have to use a global variable to peek in the subdir where this lib is stored.
set(LIB_SUBDIR "csmplugins/")
find_external_library(USGSCSM ${ASP_DEPS_DIR} "usgscsm" "usgscsm" REQUIRED)
set(LIB_SUBDIR "") # No longer needed

# ALE
find_external_library(ALE ${ASP_DEPS_DIR} "ale" "ale" REQUIRED)

# # PCL
# # The PCL include directory is include/pcl-x.xx
# file(GLOB PCL_DIRS "${ASP_DEPS_DIR}/include/pcl-*") # all matches
# list(GET PCL_DIRS 0 PCL_DIR) # first entry
# if (NOT PCL_DIR) # must exist
#   message(FATAL_ERROR "Could not find the PCL include directory.")
# endif()
# get_filename_component(PCL_INC_DIR ${PCL_DIR} NAME) # extract pcl-x.xx
# set(PCL_LIB_NAMES pcl_common pcl_features pcl_filters pcl_io_ply pcl_io
#                   pcl_kdtree pcl_keypoints pcl_ml pcl_octree pcl_recognition
#                   pcl_registration pcl_sample_consensus pcl_search pcl_segmentation
#                   pcl_stereo pcl_surface pcl_tracking)
# find_external_library(PCL ${ASP_DEPS_DIR} "${PCL_INC_DIR}" "${PCL_LIB_NAMES}" REQUIRED)

# OpenCV 
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
                     opencv_photo       opencv_xphoto
                     opencv_stereo)
find_external_library(OPENCV ${ASP_DEPS_DIR} "opencv4" "${OPENCV_LIB_NAMES}" REQUIRED)

# PDAL
set(PDAL_LIB_NAMES pdal_plugin_kernel_fauxplugin
                   pdal_plugin_reader_hdf pdal_plugin_reader_pgpointcloud
                   pdal_plugin_reader_tiledb pdal_plugin_writer_pgpointcloud
                   pdal_plugin_writer_tiledb pdalcpp)
find_external_library(PDAL ${ASP_DEPS_DIR} "pdal" "${PDAL_LIB_NAMES}" REQUIRED)

set(TBB_LIB_NAMES tbb tbbmalloc tbbmalloc_proxy)
find_external_library(TBB ${ASP_DEPS_DIR} "" "${TBB_LIB_NAMES}" REQUIRED)

# This is an experiment, to be continued
# Use only a small portion of VTK, and only in one stand-alone
# tool. That one is a giant library. Thse are installed using
# a custom conda package which only has the minimum needed libs.
#set(VTK_LIB_NAMES    vtkCommonDataModel-9.1 vtkCommonTransforms-9.1
#                     vtkCommonMath-9.1      vtkkissfft-9.1
#                     vtkCommonCore-9.1      vtksys-9.1
#                     vtkCommonMisc-9.1      vtkCommonSystem-9.1
#                     vtkloguru-9.1)
#find_external_library(VTK ${ASP_DEPS_DIR} "vtk-9.1" "${VTK_LIB_NAMES}" REQUIRED)

# For convenience, list some libraries that VW and ISIS depend on.
set(VW_3RD_PARTY_LIBS ${Z_LIBRARIES} ${OPENCV_LIBRARIES} 
    #${ILMBASE_LIBRARIES} ${OPENEXR_LIBRARIES} 
    ${GDAL_LIBRARIES} ${LAPACK_LIBRARIES} ${Boost_LIBRARIES}  
    ${PROJ4_LIBRARIES} ${GEOTIFF_LIBRARIES}  ${JPEG_LIBRARIES} ${TIFF_LIBRARIES} 
    ${PNG_LIBRARIES} ${FLANN_LIBRARIES} ${LZ4_LIBRARIES})
set(ISIS_3RD_PARTY_LIBS Qt5::Core 
    #${EMBREE_LIBRARIES} 
    ${PROTOBUF_LIBRARIES} 
    ${QWT_LIBRARIES} 
    #${ARMADILLO_LIBRARIES} 
    #${PCL_LIBRARIES} 
    ${TBB_LIBRARIES})

# # These are needed for rig_calibrator
# # MVE
# set(MVE_LIB_NAMES mveUtil mveCore mveOgl mveDmrecon)
# find_external_library(MVE ${ASP_DEPS_DIR} "mve" "${MVE_LIB_NAMES}" REQUIRED)
# # Rayint only has headers, no libraries
# find_external_library(RAYINT ${ASP_DEPS_DIR} "rayint" "" REQUIRED)
# # texture reconstruction
# find_external_library(TEXRECON ${ASP_DEPS_DIR} "" "texture_reconstruction" REQUIRED)

# Libraries use paths like "asp/src/Core/Common.h" so we just need to
# add this one include path. These must take precedence over anything
# in the conda or system directories. That is why this is at the bottom.
include_directories(BEFORE ../)

# Now that we have found all our external dependencies, generate a config.h file
include("GenerateConfig.cmake")

# Add all the header files at the top level to the install command
# Note: This does not install the headers in subdirectories.
# Separate code exists in add_library_wrapper() for taking
# care of that.
get_all_source_files("." MAIN_HEADER_FILES)
foreach(f ${MAIN_HEADER_FILES})
  INSTALL(FILES ${f} DESTINATION include/asp)
endforeach()

# CERES and its dependencies
set(SOLVER_LIBRARIES ${CERES_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})
    
# Define each of the libaries

# ASP Core
get_all_source_files( "Core"       ASP_CORE_SRC_FILES)
get_all_source_files( "Core/tests" ASP_CORE_TEST_FILES)
set(ASP_CORE_LIB_DEPENDENCIES ${VW_3RD_PARTY_LIBS} ${VISIONWORKBENCH_LIBRARIES}
    ${PDAL_LIBRARIES} ${OpenMP_CXX_LIBRARIES} ${CMAKE_DL_LIBS})

# # ASP IsisIO
# get_all_source_files( "IsisIO"       ASP_ISISIO_SRC_FILES)
# get_all_source_files( "IsisIO/tests" ASP_ISISIO_TEST_FILES)
# set(ASP_ISISIO_LIB_DEPENDENCIES AspCore ${ISIS_3RD_PARTY_LIBS} ${ISIS_LIBRARIES}
#                                 ${VW_3RD_PARTY_LIBS} ${VISIONWORKBENCH_LIBRARIES})

# ASP OpenMVG
get_all_source_files( "OpenMVG" ASP_OPENMVG_SRC_FILES)
get_all_source_files( "OpenMVG/tests" ASP_OPENMVG_TEST_FILES)
set(ASP_OPENMVG_LIB_DEPENDENCIES "")

# # ASP Rig
# get_all_source_files("Rig"       ASP_RIG_SRC_FILES)
# get_all_source_files("Rig/tests" ASP_RIG_TEST_FILES)
# set(ASP_RIG_LIB_DEPENDENCIES AspCore AspOpenMvg AspPclIO tbb ${OPENCV_LIBRARIES}
#     ${SOLVER_LIBRARIES} ${MVE_LIBRARIES} ${RAYINT_LIBRARIES} ${TEXRECON_LIBRARIES})

# ASP Camera
get_all_source_files( "Camera"       ASP_CAMERA_SRC_FILES)
get_all_source_files( "Camera/tests" ASP_CAMERA_TEST_FILES)
set(ASP_CAMERA_LIB_DEPENDENCIES AspCore 
    # AspIsisIO AspRig 
    ${XERCESC_LIBRARIES}
    ${CSM_LIBRARIES} ${USGSCSM_LIBRARIES} ${ALE_LIBRARIES})

# ASP Sessions
## This code is more complicated and is specified in the lower level file
get_all_source_files( "Sessions"       ASP_SESSIONS_SRC_FILES)
get_all_source_files( "Sessions/tests" ASP_SESSIONS_TEST_FILES)
set(ASP_SESSIONS_LIB_DEPENDENCIES AspCore 
    #AspIsisIO 
    AspCamera)

# # ASP SfS
# get_all_source_files("SfS"       ASP_SfS_SRC_FILES)
# get_all_source_files("SfS/tests" ASP_SfS_TEST_FILES)
# set(ASP_SfS_LIB_DEPENDENCIES AspCore AspCamera ${SOLVER_LIBRARIES})

# # ASP GUI
# get_all_source_files( "GUI"       ASP_GUI_SRC_FILES)
# get_all_source_files( "GUI/tests" ASP_GUI_TEST_FILES)
# set(ASP_GUI_LIB_DEPENDENCIES AspCore AspCamera Qt5::Core Qt5::Gui Qt5::Widgets ${QWT_LIBRARIES})

# # ASP Gotcha
# get_all_source_files( "Gotcha"       ASP_GOTCHA_SRC_FILES)
# get_all_source_files( "Gotcha/tests" ASP_GOTCHA_TEST_FILES)
# set(ASP_GOTCHA_LIB_DEPENDENCIES ${VISIONWORKBENCH_LIBRARIES} ${OPENCV_LIBRARIES} 
#     ${Boost_LIBRARIES})

# # ASP PcAlign
# get_all_source_files("PcAlign" ASP_PC_ALIGN_SRC_FILES)
# get_all_source_files("PcAlign/tests" ASP_PC_ALIGN_TEST_FILES)
# set(ASP_PC_ALIGN_LIB_DEPENDENCIES ${SOLVER_LIBRARIES} ${LIBPOINTMATCHER_LIBRARIES}
#     ${LIBNABO_LIBRARIES} ${FASTGLOBALREGISTRATION_LIBRARIES} AspSessions)

# # ASP PclIO
# get_all_source_files("PclIO" ASP_PCLIO_SRC_FILES)
# get_all_source_files( "PclIO/tests" ASP_PCLIO_TEST_FILES)
# set(ASP_PCLIO_LIB_DEPENDENCIES 
#     pcl_common pcl_io_ply pcl_io ${VISIONWORKBENCH_LIBRARIES} AspCore)

# Add all of the library subdirectories

add_subdirectory(Core)
# if(ASP_HAVE_PKG_ISISIO)
#     add_subdirectory(IsisIO)
# endif()
# add_subdirectory(Rig)
add_subdirectory(OpenMVG)
add_subdirectory(Camera)

add_subdirectory(Sessions)
# add_subdirectory(GUI)
# add_subdirectory(Gotcha)
#add_subdirectory(SfS)

# add_subdirectory(PclIO)
# add_subdirectory(PcAlign)

# Add the non-library subdirectories
add_subdirectory(Python)
add_subdirectory(Tools)
add_subdirectory(WVCorrect)
add_subdirectory(IceBridge)

