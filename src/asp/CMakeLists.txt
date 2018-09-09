
include(Utilities) # Contains functions

# Add path for custom modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}" )

#===============================================================================
# Configuration options


# --- Directory paths ---
set(BINARYBUILDER_INSTALL_DIR "" CACHE FILEPATH "Path to Binary Builder installation directory")
set(VISIONWORKBENCH_INSTALL_DIR "" CACHE FILEPATH "Path to Binary Builder installation directory")


if(NOT ${BINARYBUILDER_INSTALL_DIR})
  message(SEND_ERROR "You need to set BINARYBUILDER_INSTALL_DIR: ${BINARYBUILDER_INSTALL_DIR}")
endif()

# If VW path is provided, use that instead of the BB install dir.
if (${VISIONWORKBENCH_INSTALL_DIR} STREQUAL "")
  set(VISIONWORKBENCH_INSTALL_DIR ${BINARYBUILDER_INSTALL_DIR})
endif()


# Add BB folders to CMAKE paths
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "${BINARYBUILDER_INSTALL_DIR}")
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "${BINARYBUILDER_INSTALL_DIR}")



# Libraries use paths like "asp/blah/blah.h" so we just need to add this one include path
include_directories( ../ )

# --- Custom options ---
option(BUILD_SHARED_LIBS "Produce shared libraries." TRUE)

# --- Fixed options ---
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON )
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#==================================================================================
# Find dependencies

#TODO: Replace with FindVisionWorkbench module
# Set up VisionWorkbench
message("--------------------------------------------------------------")
message(" Vision Workbench installation directory: ${VISIONWORKBENCH_INSTALL_DIR}")
set(VISIONWORKBENCH_INCLUDE_DIR ${VISIONWORKBENCH_INSTALL_DIR}/include)
set(VISIONWORKBENCH_LIBRARY_DIR ${VISIONWORKBENCH_INSTALL_DIR}/lib)

# TODO: Just list these manually?
file(GLOB TEMP "${VISIONWORKBENCH_INSTALL_DIR}/lib/libvw*.so")
set(VISIONWORKBENCH_LIBRARIES)
foreach(f ${TEMP})
  get_filename_component(FILENAME ${f} NAME)
  set(VISIONWORKBENCH_LIBRARIES ${VISIONWORKBENCH_LIBRARIES} ${FILENAME})
endforeach(f)

include_directories(${VISIONWORKBENCH_INCLUDE_DIR})
link_directories(${VISIONWORKBENCH_LIBRARY_DIR})


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
    link_directories(${Boost_LIBRARY_DIR})
    #link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)
message("--------------------------------------------------------------")


message("--------------------------------------------------------------")
message("Searching for QT...")
# TODO: Make sure this finds the correct install!
set (QT_QMAKE_EXECUTABLE  ${BASESYSTEM_INSTALL_DIR}/bin/qmake)
set (QT_MOC_EXECUTABLE    ${BASESYSTEM_INSTALL_DIR}/bin/moc)
set (QT_RCC_EXECUTABLE    ${BASESYSTEM_INSTALL_DIR}/bin/rcc)
set (QT_UIC_EXECUTABLE    ${BASESYSTEM_INSTALL_DIR}/bin/uic)
#SET(CMAKE_PREFIX_PATH ${BASESYSTEM_INSTALL_DIR} ${CMAKE_PREFIX_PATH}) 
set(REQUIRED_QT_LIBS QtConcurrent QtCore QtGui QtNetwork QtSql
                     QtSvg QtWidgets QtXml QtXmlPatterns QtPrintSupport
                     QtTest QtPositioning QtQml QtQuick QtOpenGL
                     QtMultimedia QtMultimediaWidgets QtDBus)
find_package(Qt5 5.6.3 REQUIRED ${REQUIRED_QT_LIBS})

if(Qt5_FOUND)
    include(${QT_USE_FILE}) # Required after using QT find command
    message("QT include dirs  " ${QT_INCLUDES})

    # TODO: Fix!
    ## By default only QtCore and QtGui modules are enabled
    ## other modules must be enabled like this:
    #SET( QT_USE_QT3SUPPORT TRUE )   
    #SET( QT_USE_QTXML TRUE )

    # We need this to be able to include headers produced by uic in our
    # code (CMAKE_BINARY_DIR holds a path to the build directory, while
    # INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
    INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} )

endif(Qt5_FOUND)
message("--------------------------------------------------------------")

# Get ISIS libraries
set(ISIS_FOUND)
set(ISIS_LIBRARIES ${BASESYSTEM_INSTALL_DIR}/lib/libisis3.so)
set(ISIS_INCLUDE_DIRS ${BASESYSTEM_INSTALL_DIR}/include)

# Get NAIF includes (Needed for reading/writing SPICE data)
set(NAIF_FOUND True)
set(NAIF_INCLUDE_DIRS ${BASESYSTEM_INSTALL_DIR}/include/naif/)
include_directories("${NAIF_INCLUDE_DIRS}")

set(PROTOBUF_FOUND)
set(PROTOBUF_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include)
set(PROTOBUF_LIBRARY     ${BASESYSTEM_INSTALL_DIR}/lib/libprotobuf.so)
include_directories(${PROTOBUF_INCLUDE_DIR})

#find_package(Protobuf REQUIRED)
#if(PROTOBUF_FOUND)
#  message("Found PROTOBUF " ${PROTOBUF_LIBRARY})
#endif()


set(EIGEN_FOUND)
set(EIGEN_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include/eigen3)
#set(EIGEN_LIBRARY     ${BASESYSTEM_INSTALL_DIR}/lib/libprotobuf.so)
include_directories(${EIGEN_INCLUDE_DIR})


set(LIBPOINTMATCHER_FOUND)
set(LIBPOINTMATCHER_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include/)
set(LIBPOINTMATCHER_LIBRARY     ${BASESYSTEM_INSTALL_DIR}/lib/libpointmatcher.so)
include_directories(${LIBPOINTMATCHER_INCLUDE_DIR})

set(LIBLAS_FOUND)
set(LIBLAS_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include/)
set(LIBLAS_LIBRARY     ${BASESYSTEM_INSTALL_DIR}/lib/liblas.so)
include_directories(${LIBLAS_INCLUDE_DIR})

# Look for more required packages
set(LAPACK_FOUND)
set(LAPACK_LIBRARIES   ${BASESYSTEM_INSTALL_DIR}/lib/liblapack.so
                       ${BASESYSTEM_INSTALL_DIR}/lib/libblas.so)
set(LAPACK_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include/)

set(OPENSCENEGRAPH_LIB_NAMES osgAnimation.so osgDB.so osgFX.so osgGA.so osgManipulator.so
                             osgParticle.so osgPresentation.so osgShadow.so osgSim.so 
                             osg osgTerrain.so osgText.so osgUtil.so osgViewer.so osgVolume.so osgWidget.so)
find_external_library(OPENSCENEGRAPH "" "${OPENSCENEGRAPH_LIB_NAMES}" REQUIRED)
                                     
if(OPENSCENEGRAPH_FOUND)
  message("Found OpenSceneGraph " ${OPENSCENEGRAPH_LIBRARIES})
  message("Include dirs: " ${OPENSCENEGRAPH_INCLUDE_DIRS})
  include_directories("${OPENSCENEGRAPH_INCLUDE_DIRS}")
endif()

# TODO: Which do we actually need!
#find_external_library(FLANN "" "libflann_cpp.so" NOT_REQUIRED)
#find_external_library(JPEG "" "libjpeg.so" NOT_REQUIRED)
#find_external_library(PNG "" "libpng.so;libpng16.so" NOT_REQUIRED)
#find_external_library(TIFF "" "libtiff.so" NOT_REQUIRED)
#find_external_library(OPENEXR "OpenEXR" "libIlmImf.so;libIlmThread.so" NOT_REQUIRED)
#find_external_library(Z "" "libz.so" NOT_REQUIRED)
#find_external_library(GDAL "" "libgdal.so" NOT_REQUIRED)
#find_external_library(PROJ4 "" "libproj.so" NOT_REQUIRED)


# Needed?
#find_package(OpenMP)
#if(OPENMP_FOUND)
#  message("Found OPENMP")
#  SET( CMAKE_C_FLAGS    "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}" )
#  SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
#endif()


set(OPENCV_LIB_NAMES libopencv_calib3d.so     libopencv_reg.so
                     libopencv_core.so        libopencv_shape.so
                     libopencv_features2d.so  libopencv_stitching.so
                     libopencv_flann.so       libopencv_superres.so
                     libopencv_hdf.so         libopencv_surface_matching.so
                     libopencv_highgui.so     libopencv_videoio.so
                     libopencv_imgcodecs.so   libopencv_video.so
                     libopencv_imgproc.so     libopencv_xfeatures2d.so
                     libopencv_ml.so          libopencv_ximgproc.so
                     libopencv_objdetect.so   libopencv_xobjdetect.so
                     libopencv_photo.so       libopencv_xphoto.so)
find_external_library(OPENCV "" "${OPENCV_LIB_NAMES}" NOT_REQUIRED)


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

# TODO: Remove these files!
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
set(ASP_CORE_LIB_DEPENDENCIES ${VISIONWORKBENCH_LIBRARIES})

# --- ASP_SPICEIO ------------------------------------------------------------
get_all_source_files( "SpiceIO"       ASP_SPICEIO_SRC_FILES)
get_all_source_files( "SpiceIO/tests" ASP_SPICEIO_TEST_FILES)
set(ASP_SPICEIO_LIB_DEPENDENCIES AspCore)

# --- ASP_ISISIO ------------------------------------------------------------
get_all_source_files( "IsisIO"       ASP_ISISIO_SRC_FILES)
get_all_source_files( "IsisIO/tests" ASP_ISISIO_TEST_FILES)
set(ASP_ISISIO_LIB_DEPENDENCIES AspCore)

# --- ASP_CAMERA ------------------------------------------------------------
get_all_source_files( "Core"       ASP_CAMERA_SRC_FILES)
get_all_source_files( "Core/tests" ASP_CAMERA_TEST_FILES)
set(ASP_CAMERA_LIB_DEPENDENCIES AspCore)

# --- ASP_SESSIONS ------------------------------------------------------------
## This code is more complicated and is specified in the lower level file
get_all_source_files( "Sessions"       ASP_MPI_SRC_FILES)
get_all_source_files( "Sessions/tests" ASP_MPI_TEST_FILES)
set(ASP_SESSIONS_LIB_DEPENDENCIES AspCore AspIsisIO AspCamera)

# --- ASP_GUI ------------------------------------------------------------
get_all_source_files( "GUI"       ASP_GUI_SRC_FILES)
get_all_source_files( "GUI/tests" ASP_GUI_TEST_FILES)
set(ASP_GUI_LIB_DEPENDENCIES AspCore)

#=========================================================================

# TODO: Don't just use this for all tools
set(FULL_LIBRARY_LIST
  ${LAPACK_LIBRARIES} 
  ${JPEG_LIBRARIES} 
  ${GDAL_LIBRARY}
  ${PROJ4_LIBRARY}
  ${VISIONWORKBENCH_LIBRARIES}
  ${ISIS_LIBRARIES}
    AspCore
    AspCamera
    AspIsisIO
    AspSpiceIO
    AspSessions
    AspGUI
	${Boost_LIBRARIES} 
)


#=========================================================================
# Add all of the library subdirectories

add_subdirectory(Core)
add_subdirectory(IsisIO)
add_subdirectory(Sessions)
add_subdirectory(SpiceIO)
add_subdirectory(Camera)
add_subdirectory(Sessions)
add_subdirectory(GUI)

# Add the non-library subdirectories

add_subdirectory(Python)
add_subdirectory(Tools)
add_subdirectory(WVCorrect)
add_subdirectory(Hidden)
add_subdirectory(IceBridge)











