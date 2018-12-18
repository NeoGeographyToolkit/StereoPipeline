
include(Utilities) # Contains functions

# Add path for custom modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}" )

#===============================================================================
# Configuration options

# --- Directory paths ---
set(BINARYBUILDER_INSTALL_DIR "" CACHE FILEPATH "Path to Binary Builder installation directory")
set(VISIONWORKBENCH_INSTALL_DIR "" CACHE FILEPATH "Path to Binary Builder installation directory")
set(ASP_ENABLE_SSE "1" CACHE BOOL "Allow SSE optimizations.")


if("${BINARYBUILDER_INSTALL_DIR}" STREQUAL "")
  message(SEND_ERROR "You need to set BINARYBUILDER_INSTALL_DIR: ${BINARYBUILDER_INSTALL_DIR}")
endif()

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
SET(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} ${CMAKE_INSTALL_PREFIX}/lib)

# --- Fixed options ---
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON )
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

if (ASP_ENABLE_SSE)
else() # If disabled, set up for compatibility with older systems.
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


message("--------------------------------------------------------------")
message("Searching for boost...")

# TODO: Make into a function!
# Define boost requirements
set(BOOST_MIN_VERSION "1.59.0")
set(REQUIRED_BOOST_LIBS program_options system filesystem regex date_time thread iostreams)
#set(Boost_DEBUG 1) # Uncomment for useful finding boost debug output
if(BINARYBUILDER_INSTALL_DIR) # Look in BinaryBuilder directory
  set(BOOST_ROOT       "${BINARYBUILDER_INSTALL_DIR}/include/boost-1_59")
  set(BOOST_LIBRARYDIR "${BINARYBUILDER_INSTALL_DIR}/lib")
  set(BOOST_INCLUDEDIR "${BINARYBUILDER_INSTALL_DIR}/include/boost-1_59/boost")
  set(Boost_NO_SYSTEM_PATHS ON) #This does not seem to work
endif()  
# Now search for Boost using the available information
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS ${REQUIRED_BOOST_LIBS})

if(Boost_FOUND)
    message("Successfully found Boost version ${Boost_VERSION}!")
    message("---Boost Include Dir = ${Boost_INCLUDE_DIR}")
    #set(Boost_LIBRARY_DIR ${BINARYBUILDER_INSTALL_DIR}/lib) # Override this value
    message("---Boost Library Dir = ${Boost_LIBRARY_DIRS}")
    message("---Boost libraries   = ${Boost_LIBRARIES}")
    include_directories(${Boost_INCLUDE_DIR})
    #include_directories(${Boost_INCLUDE_DIRS})
    #link_directories(${Boost_LIBRARY_DIR})
    #link_directories(${Boost_LIBRARY_DIRS})
    set(ASP_HAVE_PKG_BOOST 1)
endif(Boost_FOUND)


# TODO: Add DLL to our main Boost installation.
include_directories("/home/smcmich1/repo/dll/include")
#include_directories("/home/smcmich1/repo/CSM-CameraModel/install/include")
#include_directories("/home/smcmich1/repo/CSM-Swig/install/include")
#set(CSM_LIBRARIES "/home/smcmich1/repo/CSM-Swig/install/lib/libfixturecsm.so")

#include_directories("/home/smcmich1/repo/csm-3.0.3.1/linux64/include")
#set(CSM_LIBRARIES "/home/smcmich1/repo/csm-3.0.3.1/linux64/lib/libcsmapi.so.3")

include_directories("/home/smcmich1/repo/CSM-CameraModel/install/include")
set(CSM_LIBRARIES "/home/smcmich1/repo/CSM-CameraModel/install/lib/libcsmapi.so.3")


message("--------------------------------------------------------------")
message("Searching for QT...")
# QT is complicated to use and should go through find_package
set(QT_QMAKE_EXECUTABLE  ${BINARYBUILDER_INSTALL_DIR}/bin/qmake)
set(QT_MOC_EXECUTABLE    ${BINARYBUILDER_INSTALL_DIR}/bin/moc)
set(QT_RCC_EXECUTABLE    ${BINARYBUILDER_INSTALL_DIR}/bin/rcc)
set(QT_UIC_EXECUTABLE    ${BINARYBUILDER_INSTALL_DIR}/bin/uic)

set(REQUIRED_QT_COMPONENTS  Core                Concurrent
                            Gui                 Multimedia
                            MultimediaWidgets   Network
                            OpenGL              PrintSupport
                            Qml                 Quick
                            Script              ScriptTools
                            Sql                 Svg
                            Test                Widgets
                            Xml                 XmlPatterns)
find_package(Qt5 5.6.3 COMPONENTS ${REQUIRED_QT_COMPONENTS} REQUIRED)

if(Qt5_FOUND)

    #message("QT_USE_FILE = ${QT_USE_FILE}")
    #include(${QT_USE_FILE}) # Required after using QT find command
    #message("QT include dirs  " ${QT_INCLUDES})

    ## By default only QtCore and QtGui modules are enabled
    ## other modules must be enabled like this:
    #SET( QT_USE_QT3SUPPORT TRUE )   
    #SET( QT_USE_QTXML TRUE )

    # We need this to be able to include headers produced by uic in our
    # code (CMAKE_BINARY_DIR holds a path to the build directory, while
    # INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
    INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} )
    set (ASP_HAVE_PKG_QT 1)
    message("QT found: ${Qt5Core_INCLUDE_DIRS}")
endif(Qt5_FOUND)
message("--------------------------------------------------------------")

find_external_library(QWT "" "qwt" REQUIRED)

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
  find_external_library(LAPACK "" "lapack;blas" REQUIRED)
endif()
if(LAPACK_FOUND)
  set(VW_HAVE_PKG_FLAPACK 1) # --> This specifies that it is a Fortran derived version
                             #      How to check this if the user provides Fortran?
endif()

find_external_library(FLANN "" "flann_cpp" REQUIRED)
find_external_library(Z "" "z" REQUIRED)
find_external_library(PROJ4 "" "proj"      REQUIRED)

find_external_library(GDAL  "" "gdal"      REQUIRED)
set(ILMBASE_LIB_NAMES Half Iex Imath IlmThread)
find_external_library(ILMBASE "" "${ILMBASE_LIB_NAMES}" REQUIRED)
find_external_library(OPENEXR "OpenEXR" "IlmImf;IlmThread" REQUIRED)

# Dependencies for ASP and ISIS
find_external_library(NAIF "naif" "" REQUIRED)
find_external_library(SPICE "" "cspice" REQUIRED)
find_external_library(GEOID "" "egm2008" REQUIRED)
find_external_library(XERCESC "" "xerces-c" REQUIRED)
find_external_library(PROTOBUF "" "protobuf" REQUIRED)
find_external_library(EMBREE "" "embree" REQUIRED)
find_external_library(EIGEN "eigen3" "" REQUIRED)
find_external_library(CERES "ceres" "ceres" REQUIRED)
find_external_library(LIBPOINTMATCHER "" "pointmatcher" REQUIRED)
find_external_library(FASTGLOBALREGISTRATION "FastGlobalRegistration" "FastGlobalRegistrationLib" REQUIRED)
find_external_library(LIBLAS "" "las" REQUIRED)
find_external_library(LASZIP "laszip" "laszip" REQUIRED)
find_external_library(GFLAGS "gflags" "gflags" REQUIRED)
find_external_library(GLOG "glog" "glog" REQUIRED)
find_external_library(ARMADILLO "" "armadillo" REQUIRED)

find_external_library(ISIS "" "isis3" NOT_REQUIRED)

if(ASP_HAVE_PKG_ISIS)
  set(ASP_HAVE_PKG_ISISIO 1)
  set(ASP_ISIS_VERSION "3.5.2") # TODO: Autodetect
endif()

set(PCL_LIB_NAMES pcl_common pcl_features pcl_filters pcl_io_ply pcl_io
                  pcl_kdtree pcl_keypoints pcl_ml pcl_octree pcl_recognition
                  pcl_registration pcl_sample_consensus pcl_search pcl_segmentation
                  pcl_stereo pcl_surface pcl_tracking)
find_external_library(PCL "pcl" "${PCL_LIB_NAMES}" REQUIRED)


set(OPENSCENEGRAPH_LIB_NAMES osgAnimation osgDB osgFX osgGA osgManipulator
                             osgParticle osgPresentation osgShadow osgSim OpenThreads
                             osg osgTerrain osgText osgUtil osgViewer osgVolume osgWidget)
find_external_library(OPENSCENEGRAPH "" "${OPENSCENEGRAPH_LIB_NAMES}" REQUIRED)

# TODO: Which do we actually need!
#find_external_library(JPEG "" "jpeg" NOT_REQUIRED)
#find_external_library(PNG "" "png;png16" NOT_REQUIRED)
#find_external_library(TIFF "" "tiff" NOT_REQUIRED)


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
find_external_library(OPENCV "" "${OPENCV_LIB_NAMES}" NOT_REQUIRED)

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











