# Add path for custom modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}" )

include(Utilities) # Contains functions

# Configuration options

# Directory paths
set(VISIONWORKBENCH_INSTALL_DIR "" CACHE FILEPATH "Path to the user-built and installed VisionWorkbench")
set(ASP_DEPS_DIR "" CACHE FILEPATH "Path to the conda environment that has the ASP dependencies")

set(ASP_ENABLE_SSE "1" CACHE BOOL "Allow SSE optimizations.")

if ("${ASP_DEPS_DIR}" STREQUAL "")
  message(FATAL_ERROR "You need to set ASP_DEPS_DIR")
endif()

# Where to search for dependencies
set(CMAKE_PREFIX_PATH "${ASP_DEPS_DIR}")

find_package(Threads REQUIRED)

# Put the search for Boost early on, to ensure that the Conda version
# is found before we point to the BB folder.
message("Searching for Boost.")
set(REQUIRED_BOOST_LIBS program_options system filesystem regex date_time thread iostreams)
if (ASP_DEPS_DIR) # Look in that directory
  set(BOOST_ROOT       "${ASP_DEPS_DIR}")
  set(BOOST_LIBRARYDIR "${ASP_DEPS_DIR}/lib")
  set(BOOST_INCLUDEDIR "${ASP_DEPS_DIR}/include")
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
SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} ${CMAKE_INSTALL_PREFIX}/lib ${ASP_DEPS_DIR}/lib)

# Fixed options
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
#link_directories(${VISIONWORKBENCH_LIBRARY_DIR})

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
find_package(Qt5 5.9.6 COMPONENTS ${REQUIRED_QT_COMPONENTS} REQUIRED)

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

find_external_library(FLANN   ${ASP_DEPS_DIR} "" "flann_cpp" REQUIRED)
find_external_library(LZ4     ${ASP_DEPS_DIR} "" "lz4"       REQUIRED)
find_external_library(Z       ${ASP_DEPS_DIR} "" "z"         REQUIRED)
find_external_library(PROJ4   ${ASP_DEPS_DIR} "" "proj"      REQUIRED)
find_external_library(JPEG    ${ASP_DEPS_DIR} "" "jpeg"      REQUIRED)
find_external_library(PNG     ${ASP_DEPS_DIR} "" "png;png16" REQUIRED)
find_external_library(TIFF    ${ASP_DEPS_DIR} "" "tiff"      REQUIRED)
find_external_library(GEOTIFF ${ASP_DEPS_DIR} "" "geotiff"   REQUIRED)
find_external_library(GDAL    ${ASP_DEPS_DIR} "" "gdal"      REQUIRED)
set(ILMBASE_LIB_NAMES Half Iex Imath IlmThread)
find_external_library(ILMBASE ${ASP_DEPS_DIR} "" "${ILMBASE_LIB_NAMES}" REQUIRED)
find_external_library(OPENEXR ${ASP_DEPS_DIR} "OpenEXR" "IlmImf;IlmThread" REQUIRED)

find_external_library(SPICE ${ASP_DEPS_DIR} "cspice" "cspice" REQUIRED)
find_external_library(GEOID ${ASP_DEPS_DIR} "" "egm2008" REQUIRED)
find_external_library(XERCESC ${ASP_DEPS_DIR} "" "xerces-c" REQUIRED)
find_external_library(PROTOBUF ${ASP_DEPS_DIR} "" "protobuf" REQUIRED)
find_external_library(EMBREE ${ASP_DEPS_DIR} "" "embree3" REQUIRED)

find_external_library(EIGEN ${ASP_DEPS_DIR} "eigen3" "" REQUIRED)
find_external_library(CERES ${ASP_DEPS_DIR} "ceres" "ceres" REQUIRED)
find_external_library(LIBNABO ${ASP_DEPS_DIR} "nabo" "nabo" REQUIRED)
find_external_library(LIBPOINTMATCHER ${ASP_DEPS_DIR} "" "pointmatcher" REQUIRED)
find_external_library(FASTGLOBALREGISTRATION ${ASP_DEPS_DIR} "FastGlobalRegistration" "FastGlobalRegistrationLib" REQUIRED)
find_external_library(GFLAGS ${ASP_DEPS_DIR} "gflags" "gflags" REQUIRED)
find_external_library(GLOG ${ASP_DEPS_DIR} "glog" "glog" REQUIRED)
find_external_library(ARMADILLO ${ASP_DEPS_DIR} "" "armadillo" REQUIRED)
find_external_library(ISIS ${ASP_DEPS_DIR} "isis" "isis" REQUIRED)

if(ASP_HAVE_PKG_ISIS)
  # Pull the ISIS version. It is on the first line before the first space.
  file(STRINGS "${ASP_DEPS_DIR}/version" MY_STRING LIMIT_COUNT 1)
  string(REPLACE " " ";" MY_LIST ${MY_STRING}) # split by space and store into MY_LIST
  list(GET MY_LIST 0 ASP_ISIS_VERSION)
  set(ASP_HAVE_PKG_ISISIO 1)
  message(STATUS "Found ISIS version: ${ASP_ISIS_VERSION}")
endif()

find_external_library(CSM ${ASP_DEPS_DIR} "csm" "csmapi" REQUIRED)

# Link to libusgscsm. This makes it disappear from the list of CSM
# plugins, presubably because it is found at link time now,
# but things still work. This is is necessary since we need to peek into
# usgscsm by bypassing the csm interface.
find_external_library(USGSCSM ${ASP_DEPS_DIR} "usgscsm" "usgscsm" REQUIRED)

# Link to ale. This is needed to be able to export the CSM camera state.
find_external_library(ALE ${ASP_DEPS_DIR} "ale" "ale" REQUIRED)

# include_directories("${ASP_DEPS_DIR}/include")

# PCL
# TODO(oalexan1): Having to specify the version here always breaks.
set(PCL_LIB_NAMES pcl_common pcl_features pcl_filters pcl_io_ply pcl_io
                  pcl_kdtree pcl_keypoints pcl_ml pcl_octree pcl_recognition
                  pcl_registration pcl_sample_consensus pcl_search pcl_segmentation
                  pcl_stereo pcl_surface pcl_tracking)
find_external_library(PCL ${ASP_DEPS_DIR} "pcl-1.13" "${PCL_LIB_NAMES}" REQUIRED)

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
set(VW_3RD_PARTY_LIBS ${Z_LIBRARIES} ${OPENCV_LIBRARIES} ${ILMBASE_LIBRARIES} ${OPENEXR_LIBRARIES}
    ${GDAL_LIBRARIES} ${LAPACK_LIBRARIES} ${Boost_LIBRARIES}  ${PROJ4_LIBRARIES}
    ${GEOTIFF_LIBRARIES}  ${JPEG_LIBRARIES} ${TIFF_LIBRARIES} ${PNG_LIBRARIES}
    ${FLANN_LIBRARIES} ${LZ4_LIBRARIES})
set(ISIS_3RD_PARTY_LIBS Qt5::Core ${EMBREE_LIBRARIES} ${PROTOBUF_LIBRARIES} ${QWT_LIBRARIES} ${ARMADILLO_LIBRARIES})

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

# Define each of the libaries

# ASP_CORE
get_all_source_files( "Core"       ASP_CORE_SRC_FILES)
get_all_source_files( "Core/tests" ASP_CORE_TEST_FILES)
set(ASP_CORE_LIB_DEPENDENCIES ${VW_3RD_PARTY_LIBS} ${VISIONWORKBENCH_LIBRARIES}
    ${PDAL_LIBRARIES} ${OpenMP_CXX_LIBRARIES} ${CMAKE_DL_LIBS})

# ASP_SPICEIO
get_all_source_files( "SpiceIO"       ASP_SPICEIO_SRC_FILES)
get_all_source_files( "SpiceIO/tests" ASP_SPICEIO_TEST_FILES)
set(ASP_SPICEIO_LIB_DEPENDENCIES AspCore ${SPICE_LIBRARIES})

# ASP_ISISIO
get_all_source_files( "IsisIO"       ASP_ISISIO_SRC_FILES)
get_all_source_files( "IsisIO/tests" ASP_ISISIO_TEST_FILES)
set(ASP_ISISIO_LIB_DEPENDENCIES  AspCore ${ISIS_3RD_PARTY_LIBS} ${ISIS_LIBRARIES}
                                 ${VW_3RD_PARTY_LIBS} ${VISIONWORKBENCH_LIBRARIES})
# ASP_CAMERA
get_all_source_files( "Camera"       ASP_CAMERA_SRC_FILES)
get_all_source_files( "Camera/tests" ASP_CAMERA_TEST_FILES)
set(ASP_CAMERA_LIB_DEPENDENCIES AspCore ${XERCESC_LIBRARIES}
    ${CSM_LIBRARIES} ${USGSCSM_LIBRARIES} ${ALE_LIBRARIES})

# ASP_SESSIONS
## This code is more complicated and is specified in the lower level file
get_all_source_files( "Sessions"       ASP_SESSIONS_SRC_FILES)
get_all_source_files( "Sessions/tests" ASP_SESSIONS_TEST_FILES)
set(ASP_SESSIONS_LIB_DEPENDENCIES AspCore AspSpiceIO AspIsisIO AspCamera)

# ASP_GUI
get_all_source_files( "GUI"       ASP_GUI_SRC_FILES)
get_all_source_files( "GUI/tests" ASP_GUI_TEST_FILES)
set(ASP_GUI_LIB_DEPENDENCIES AspCore Qt5::Core Qt5::Gui Qt5::Widgets ${QWT_LIBRARIES})

# ASP_GOTCHA
get_all_source_files( "Gotcha"       ASP_GOTCHA_SRC_FILES)
get_all_source_files( "Gotcha/tests" ASP_GOTCHA_TEST_FILES)
set(ASP_GOTCHA_LIB_DEPENDENCIES ${VISIONWORKBENCH_LIBRARIES} ${OPENCV_LIBRARIES} ${Boost_LIBRARIES})

# ASP_PclIO
get_all_source_files("PclIO" ASP_PCLIO_SRC_FILES)
get_all_source_files( "PclIO/tests" ASP_PCLIO_TEST_FILES)
set(ASP_PCLIO_LIB_DEPENDENCIES  pcl_common pcl_io_ply pcl_io
    AspCore ${VISIONWORKBENCH_LIBRARIES})

# CERES and its dependencies
set(SOLVER_LIBRARIES ${CERES_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})
    
# Add all of the library subdirectories

add_subdirectory(Core)
add_subdirectory(SpiceIO)
if(ASP_HAVE_PKG_ISISIO)
    add_subdirectory(IsisIO)
endif()
add_subdirectory(Camera)

add_subdirectory(Sessions)
add_subdirectory(GUI)
add_subdirectory(Gotcha)

# Add the non-library subdirectories
add_subdirectory(Python)
add_subdirectory(Tools)
add_subdirectory(WVCorrect)
add_subdirectory(IceBridge)

add_subdirectory(PclIO)
