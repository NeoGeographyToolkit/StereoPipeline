

#TODO: Remove this, the find functions should handle it!
set(BASESYSTEM_INSTALL_DIR "/home/smcmich1/repo/BaseSystem")

set(VISIONWORKBENCH_INSTALL_DIR "/home/smcmich1/repo/visionworkbench/build")


# Libraries use paths like "asp/blah/blah.h" so we just need to add this one include path
include_directories( ../ )


# Add the correct version of boost

set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "${BASESYSTEM_INSTALL_DIR}")
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "${BASESYSTEM_INSTALL_DIR}")

# TODO: More robust finder packages, for now just use the fixed paths into the BaseSystem directory

#TODO: Replace with FindVisionWorkbench module
# Set up VisionWorkbench
message("--------------------------------------------------------------")
message(" Vision Workbench installation directory: ${VISIONWORKBENCH_INSTALL_DIR}")
set(VISIONWORKBENCH_INCLUDE_DIR ${VISIONWORKBENCH_INSTALL_DIR}/include)
set(VISIONWORKBENCH_LIBRARY_DIR ${VISIONWORKBENCH_INSTALL_DIR}/lib)

file(GLOB TEMP "${VISIONWORKBENCH_INSTALL_DIR}/lib/*.so")
set(VISIONWORKBENCH_LIBRARIES)
foreach(f ${TEMP})
  get_filename_component(FILENAME ${f} NAME)
  set(VISIONWORKBENCH_LIBRARIES ${VISIONWORKBENCH_LIBRARIES} ${FILENAME})
endforeach(f)

include_directories(${VISIONWORKBENCH_INCLUDE_DIR})
link_directories(${VISIONWORKBENCH_LIBRARY_DIR})

message("--------------------------------------------------------------")
message("Searching for boost...")

set(BOOST_ROOT "${BASESYSTEM_INSTALL_DIR}/include/boost-1_54")
set(BOOST_LIBRARYDIR  "${BASESYSTEM_INSTALL_DIR}/lib")
set(BOOST_INCLUDEDIR "${BASESYSTEM_INSTALL_DIR}/include/boost-1_54/boost")
set(Boost_NO_SYSTEM_PATHS ON) #This does not seem to work
#set(Boost_DEBUG 1) # Uncomment for useful finding boost debug output
set(BOOST_MIN_VERSION "1.54.0")
set(REQUIRED_BOOST_LIBS program_options system filesystem regex date_time thread iostreams)

find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS ${REQUIRED_BOOST_LIBS})

if(Boost_FOUND)
    message("Successfully found Boost version ${Boost_VERSION}!")
    message("---Boost Include Dir = ${Boost_INCLUDE_DIR}")
    #set(Boost_LIBRARY_DIR ${BASESYSTEM_INSTALL_DIR}/lib) # Override this value
    message("---Boost Library Dir = ${Boost_LIBRARY_DIRS}")
    message("---Boost libraries = ${Boost_LIBRARIES}")
    include_directories(${Boost_INCLUDE_DIR})
    link_directories(${Boost_LIBRARY_DIR})
endif(Boost_FOUND)
message("--------------------------------------------------------------")

#message("--------------------------------------------------------------")
#message("Searching for Google Protocol Buffers...")

#set(BOOST_ROOT "${BASESYSTEM_INSTALL_DIR}/boost")
#set(BOOST_MIN_VERSION "1.54.0")
#FIND_PACKAGE(Protobuf ${BOOST_MIN_VERSION} REQUIRED)

#if(PROTOBUF_FOUND)
#    message("Successfully found Google Protocol Buffers")
#    message("---Protobuf library file = ${PROTOBUF_LIBRARY}")
#    message("---Protobuf Include Dir = ${PROTOBUF_INCLUDE_DIR}")
#    include_directories(${PROTOBUF_INCLUDE_DIRS})
#    target_link_libraries(foo ${PROTOBUF_LIBRARIES})
#endif()
#message("--------------------------------------------------------------")
set(PROTOBUF_FOUND)
set(PROTOBUF_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include   )
set(PROTOBUF_LIBRARY     ${BASESYSTEM_INSTALL_DIR}/lib/libprotobuf.so)
include_directories("${PROTOBUF_INCLUDE_DIR}")
#    target_link_libraries(foo ${Boost_LIBRARIES})
#TODO Set additional boost libraries

message("--------------------------------------------------------------")
message("Searching for QT...")
# TODO: Make sure this finds the correct install!
#set (QT_QMAKE_EXECUTABLE  ${BASESYSTEM_INSTALL_DIR}/bin/qmake)
#SET(CMAKE_PREFIX_PATH ${BASESYSTEM_INSTALL_DIR} ${CMAKE_PREFIX_PATH}) 
#set(REQUIRED_QT_LIBS QtCore QtXml QtGui ) #TODO: Figure this out
#find_package(Qt4 4.8.5 REQUIRED )# ${REQUIRED_QT_LIBS})

# Testing: Manual QT settings
set(QT_FOUND True)

set(QT_INCLUDE_DIRS ${BASESYSTEM_INSTALL_DIR}/include/Qt
				    ${BASESYSTEM_INSTALL_DIR}/include/Qt3Support
				    ${BASESYSTEM_INSTALL_DIR}/include/QtCore
				    ${BASESYSTEM_INSTALL_DIR}/include/QtDesigner
				    ${BASESYSTEM_INSTALL_DIR}/include/QtGui
				    ${BASESYSTEM_INSTALL_DIR}/include/QtHelp
				    ${BASESYSTEM_INSTALL_DIR}/include/QtMultimedia
				    ${BASESYSTEM_INSTALL_DIR}/include/QtNetwork
				    ${BASESYSTEM_INSTALL_DIR}/include/QtSql
				    ${BASESYSTEM_INSTALL_DIR}/include/QtSvg
				    ${BASESYSTEM_INSTALL_DIR}/include/QtTest
				    ${BASESYSTEM_INSTALL_DIR}/include/QtUiTools
				    ${BASESYSTEM_INSTALL_DIR}/include/QtXml
				    ${BASESYSTEM_INSTALL_DIR}/include/QtXmlPatterns)
set(QT_LIBRARY_DIR 	${BASESYSTEM_INSTALL_DIR}/lib)
set(QT_LIBRARIES 	libQt3Support.so
					libQtCLucene.so
					libQtCore.so
					libQtDesignerComponents.so
					libQtDesigner.so
					libQtGui.so
					libQtHelp.so
					libQtMultimedia.so
					libQtNetwork.so
					libQtSql.so
					libQtSvg.so
					libQtTest.so
					libQtXmlPatterns.so
					libQtXml.so )

if(QT_FOUND)
    message("QT include dirs  " ${QT_INCLUDE_DIRS})
    include_directories(${QT_INCLUDE_DIRS})
    link_directories(${QT_LIBRARY_DIR})
endif(QT_FOUND)
message("--------------------------------------------------------------")

# Include NAIF
set(NAIF_INLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include/naif)
include_directories(${NAIF_INLUDE_DIR})


# Look for more required packages
find_package(LAPACK REQUIRED)
set(HAVE_PKG_LAPACK LAPACK_FOUND)
if(LAPACK_FOUND)
  message("Found LAPACK")
endif()

#TODO: Find in correct place!
#find_package(JPEG REQUIRED)
set(JPEG_FOUND True)
set(JPEG_LIBRARIES   ${BASESYSTEM_INSTALL_DIR}/lib/libjpeg.so)
set(JPEG_LIBRARY     ${BASESYSTEM_INSTALL_DIR}/lib/)
set(JPEG_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include/)

set(HAVE_PKG_JPEG JPEG_FOUND)
if(JPEG_FOUND)
  message("Found JPEG")
endif()

find_package(PNG REQUIRED)
set(HAVE_PKG_PNG PNG_FOUND)
if(PNGFOUND)
  message("Found PNG")
endif()

find_package(TIFF REQUIRED)
set(HAVE_PKG_TIFF TIFF_FOUND)
if(TIFF_FOUND)
  message("Found TIFF")
endif()

find_package(GDAL REQUIRED)
set(HAVE_PKG_GDAL GDAL_FOUND)
if(GDAL_FOUND)
  message("Found GDAL at " ${GDAL_LIBRARY})
  include_directories(${GDAL_INCLUDE_DIR})
endif()

find_package(Protobuf REQUIRED)
set(HAVE_PKG_PROTOBUF PROTOBUF_FOUND)
if(PROTOBUF_FOUND)
  message("Found PROTOBUF")
endif()

find_package(PythonLibs REQUIRED)
set(HAVE_PKG_PYTHONLIBS PYTHONLIBS_FOUND)
if(PYTHONLIBS_FOUND)
  message("Found PYTHONLIBS")
endif()

#TODO: Use FindProj cmake file to do this!
set(HAVE_PKG_PROJ4 true)
set(PROJ4_LIBRARY     ${BASESYSTEM_INSTALL_DIR}/lib/libproj.so)
set(PROJ4_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include)


#TODO: Look for each of these installations!
set(HAVE_PKG_OPENEXR)
set(HAVE_PKG_HDF)
set(HAVE_PKG_CAIROMM)
set(HAVE_PKG_RABBITMQ_C)
set(HAVE_PKG_ZEROMQ)
set(HAVE_PKG_LIBKML)
set(HAVE_PKG_OPENCV)
set(HAVE_PKG_FLANN)





# Add miscellaneous include directories needed
set(OPEN_EXR_INCLUDE_DIR ${BASESYSTEM_INSTALL_DIR}/include/OpenEXR)
include_directories("${OPEN_EXR_INCLUDE_DIR}")



message("--------------------------------------------------------------")
message("Listing all include directories...")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
message("--------------------------------------------------------------")



#=========================================================================
# Define each of the libaries
#TODO: Handle .lo files!

# --- SP_CONTROLNETTK ------------------------------------------------------------
get_all_source_files( "ControlNetTK" SP_CONTROLNETTK_SRC_FILES)
get_all_source_files( "BundleAdjustment/tests" SP_CONTROLNETTK_TEST_FILES)
set(SP_CONTROLNETTK_LIB_DEPENDENCIES SpCore)


# --- SP_CORE ------------------------------------------------------------
get_all_source_files( "Core" SP_CORE_SRC_FILES)
get_all_source_files( "Core/tests" SP_CORE_TEST_FILES)
set(SP_CORE_LIB_DEPENDENCIES )

# --- SP_ISISIO ------------------------------------------------------------
get_all_source_files( "IsisIO" SP_ISISIO_SRC_FILES)
get_all_source_files( "IsisIO/tests" SP_ISISIO_TEST_FILES)
set(SP_ISISIO_LIB_DEPENDENCIES SpCore)

# --- SP_MPI ------------------------------------------------------------
get_all_source_files( "MPI" SP_MPI_SRC_FILES)
get_all_source_files( "MPI/tests" SP_MPI_TEST_FILES)
set(SP_MPI_LIB_DEPENDENCIES SpCore)

# --- SP_SPICEIO ------------------------------------------------------------
get_all_source_files( "SpiceIO" SP_SPICEIO_SRC_FILES)
get_all_source_files( "SpiceIO/tests" SP_SPICEIO_TEST_FILES)
set(SP_SPICEIO_LIB_DEPENDENCIES SpCore)



#=========================================================================
# Add all of the library subdirectories

add_subdirectory(ControlNetTK)
add_subdirectory(Core)
add_subdirectory(IsisIO)
#add_subdirectory(MPI) # Requires OpenMPI?
add_subdirectory(SpiceIO)
add_subdirectory(Tools)



#=========================================================================
# Define targets

# Targets are defined in /tools/CMakeLists.txt












