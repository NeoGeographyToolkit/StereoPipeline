dnl __BEGIN_LICENSE__
dnl  Copyright (c) 2009-2020, United States Government as represented by the
dnl  Administrator of the National Aeronautics and Space Administration. All
dnl  rights reserved.
dnl
dnl  The NGT platform is licensed under the Apache License, Version 2.0 (the
dnl  "License"); you may not use this file except in compliance with the
dnl  License. You may obtain a copy of the License at
dnl  http://www.apache.org/licenses/LICENSE-2.0
dnl
dnl  Unless required by applicable law or agreed to in writing, software
dnl  distributed under the License is distributed on an "AS IS" BASIS,
dnl  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
dnl  See the License for the specific language governing permissions and
dnl  limitations under the License.
dnl __END_LICENSE__


m4_pattern_forbid([^AX_])
m4_pattern_allow([^AX_CFLAGS$])
m4_pattern_allow([^AX_INCLUDE_DIR$])
m4_pattern_allow([^AX_LIBDIR$])
m4_pattern_allow([^AX_OTHER_LIBDIR$])
m4_pattern_allow([^PKG_[A-Z_]+_CPPFLAGS$])
m4_pattern_allow([^PKG_[A-Z_]+_LDFLAGS$])
m4_pattern_allow([^PKG_[A-Z_]+_LIBS$])
m4_pattern_allow([^PKG_PATHS$])
m4_pattern_allow([^PKG_PATHS_[A-Z_]+$])
m4_pattern_allow([^PKG_BOOST_INCDIR$])
m4_pattern_allow([^PKG_BOOST_LIBDIR$])
m4_pattern_allow([^PKG_VW_LIBS_TEMP$])
m4_pattern_allow([^PKG_ARBITRARY_QT_NEEDED_HEADERS$])

##################################################
# A few starting bits
##################################################

AC_PREREQ([2.59])
dnl TODO BEFORE RELEASE: update package version
AC_INIT([NASA Ames Stereo Pipeline], [2.7.0-alpha], [asp@nx.arc.nasa.gov], [StereoPipeline])
#AC_INIT([NASA Ames Stereo Pipeline], [2.6.2], [asp@nx.arc.nasa.gov], [StereoPipeline])
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE([nostdinc 1.8.5])
AX_CONFIG_HEADER_PREFIX([src/asp/asp_config.h], [ASP_])

dnl Fix for early autoconf that doesn't export these
AC_SUBST([abs_top_srcdir])
AC_SUBST([abs_top_builddir])

# This var isn't defined in autoconf < 2.60
if test -z "$docdir" ; then
  docdir='${datadir}/doc/${PACKAGE_TARNAME}'
  AC_SUBST(docdir)
fi

m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl TODO RIGHT BEFORE RELEASE: update libtool version
dnl if abi changes in any way:
dnl     current++
dnl     revision=0
dnl     if new abi is backward-compatible:
dnl         age += 1
dnl     else:
dnl         age = 0
dnl else:
dnl     revision++
dnl (see http://sources.redhat.com/autobook/autobook/autobook_91.html)
dnl
dnl LIBTOOL_VERSION=current:revision:age
LIBTOOL_VERSION=4:0:0
AC_SUBST(LIBTOOL_VERSION)

m4_divert_text([INIT_PREPARE], [dnl
# load the config.options
if test -f ./config.options; then
  AX_LOG([using config.options << EOF])
  AX_LOG([`cat ./config.options`])
  AX_LOG([EOF])
  . ./config.options
fi
])

##################################################
# Find the necessary programs
##################################################

# Make sure these are set so AC_PROG_CC doesn't initialize them with
# its defaults, since we take care of setting debug and optimization
# levels ourselves.
CFLAGS="$CFLAGS"
CXXFLAGS="$CXXFLAGS"

# Disable static libraries by default
AM_DISABLE_STATIC

# Find the tools
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AX_PROG_AR
AC_PROG_LIBTOOL
AC_PROG_MAKE_SET
AC_LANG(C++)
AX_COMMON_OPTIONS

# Does the compiler support noreturn?
AX_CHECK_FUNCTION_ATTRIBUTE([noreturn])
AX_CHECK_FUNCTION_ATTRIBUTE([deprecated])

# Looking for posix headers
AC_CHECK_HEADERS([unistd.h pwd.h])
AC_CHECK_FUNCS([getpwuid getpid])

##################################################
# Compilation options
##################################################

AX_ARG_ENABLE(strict-aliasing, no, [none],           [allow strict aliasing (dangerous)])
AX_ARG_ENABLE(qt-qmake,      yes, [none],            [Allow qmake detection of QT])


##################################################
# Handle options
##################################################



##################################################
# distribution options
##################################################




##################################################
# package checks
##################################################

AX_PKG_PTHREADS

AX_PKG(OPEN_MPI, [], [-D_REENTRANT  -Wl,-u,_munmap -Wl,-multiply_defined,suppress -lmpi_cxx -lmpi -lopen-rte -lopen-pal -lutil], [mpi.h])

AX_PKG_BOOST([BOOST_PROGRAM_OPTIONS BOOST_SYSTEM BOOST_FILESYSTEM BOOST_THREAD BOOST_GRAPH],
  BOOST_PROGRAM_OPTIONS,
    [AX_PKG_BOOST_LIB(PROGRAM_OPTIONS, [-lboost_program_options], [boost/program_options.hpp], [boost::program_options::options_description("Rawr");])],
  BOOST_SYSTEM,
    [AX_PKG_BOOST_LIB(SYSTEM,          [-lboost_system],          [boost/system/error_code.hpp], [boost::system::error_code();])],
  BOOST_FILESYSTEM,
    [AX_PKG_BOOST_LIB(FILESYSTEM,      [-lboost_filesystem],      [boost/filesystem/path.hpp], [boost::filesystem::path();], [$PKG_BOOST_SYSTEM_LIBS])],
  BOOST_REGEX,
    [AX_PKG_BOOST_LIB(REGEX,           [-lboost_regex],           [boost/regex.hpp], [boost::regex(".*");])],
  BOOST_DATETIME,
    [AX_PKG_BOOST_LIB(DATETIME,        [-lboost_date_time],       [boost/date_time/gregorian/gregorian_types.hpp], [], [$PKG_BOOST_SYSTEM_LIBS])],
  BOOST_THREAD,
    [AX_PKG_BOOST_LIB(THREAD,          [-lboost_thread],          [boost/thread/thread.hpp], [boost::mutex();], [$PKG_BOOST_DATETIME_LIBS], [boost/thread/condition.hpp])],
  BOOST_IOSTREAMS,
    [AX_PKG_BOOST_LIB(IOSTREAMS,       [-lboost_iostreams],       [boost/iostreams/device/mapped_file.hpp], [boost::iostreams::mapped_file_source();])],
  BOOST_GRAPH,
    [AX_PKG_BOOST_LIB(GRAPH,           [],                        [boost/graph/adjacency_list.hpp])],
  BOOST_MPI,
    [AX_PKG_BOOST_LIB(MPI,             [-lboost_mpi],             [boost/mpi.hpp], [], [$PKG_OPEN_MPI_LIBS])],
  BOOST_SERIALIZATION,
    [AX_PKG_BOOST_LIB(SERIALIZATION,   [-lboost_serialization],   [boost/serialization/string.hpp], [], [$PKG_OPEN_MPI_LIBS $PKG_BOOST_MPI_LIBS])]
)

if test x"$HAVE_PKG_BOOST" = "xyes"; then
  AX_PKG_BOOST_CHECK_VERSION([VW_])
  AC_MSG_NOTICE([Using Boost header version $BOOST_VERSION, library suffix $PKG_BOOST_SAME_SUFFIX])
fi

# This provides an easy way for users to override the threads options
# if they are replacing Boost threads with something else for their platform.
AX_PKG(THREADS, [BOOST_THREAD], [], [])

AX_PKG(M, [], [-lm], [math.h])

AX_PKG_ONE_OF(LAPACK,
  APPLE_LAPACK,
    [AX_PKG_APPLE(LAPACK, [], [-framework Accelerate])],
  INTEL_LAPACK,
    [case $host_cpu in
       ia64)   AX_PKG(INTEL_LAPACK, [PTHREADS M], [-lmkl_lapack -lmkl -lmkl_i2p -lguide], [mkl_lapack.h], [], [64],    [dgelss_]);;
       x86_64) AX_PKG(INTEL_LAPACK, [PTHREADS M], [-lmkl_lapack -lmkl -lmkl_def -lguide], [mkl_lapack.h], [], [em64t], [dgelss_]);;
       *)      AX_PKG(INTEL_LAPACK, [PTHREADS M], [-lmkl_lapack -lmkl -lguide],           [mkl_lapack.h], [], [32],    [dgelss_]);;
     esac],
  CLAPACK,
    [AX_PKG(CLAPACK, [], [-lclapack -lblas -lf2c],     [], [], [], [dgelss_])],
  SLAPACK,
    [AX_PKG(SLAPACK, [], [-llapack -lblas],            [], [], [], [dgelss_])],
  FLAPACK,
    [AX_PKG(FLAPACK, [], [-llapack -lblas -lgfortran], [], [], [], [dgelss_])],
  STANDALONE_LAPACK_BLAS,
    [AX_PKG(STANDALONE_BLAS, [], [-lblas], [])
     AX_PKG(STANDALONE_LAPACK, [STANDALONE_BLAS], [-llapack], [], [], [], [dgelss_])
     AX_GROUP_PKG(STANDALONE_LAPACK_AND_BLAS, [STANDALONE_LAPACK STANDALONE_BLAS])],
  STANDALONE_FLAPACK_FBLAS,
    [AX_PKG(STANDALONE_F2C, [], [-lf2c], [])
     AX_PKG(STANDALONE_FBLAS, [STANDALONE_F2C], [-lblas], [])
     AX_PKG(STANDALONE_FLAPACK, [STANDALONE_FBLAS STANDALONE_F2C], [-llapack], [], [], [], [dgelss_])
     AX_GROUP_PKG(STANDALONE_FLAPACK_FBLAS, [STANDALONE_FLAPACK STANDALONE_FBLAS STANDALONE_F2C])])

AX_PKG(PROTOBUF, [PTHREADS], [-lprotobuf], [google/protobuf/stubs/common.h])
#AC_PATH_TOOL( PROTOC, [protoc], [no], [$PKG_PATHS_PROTOBUF/bin$PATH_SEPARATOR$PATH])
#if test x"$PROTOC" = "xno" && test x"$HAVE_PKG_PROTOBUF" = "xyes"; then
#  AC_MSG_ERROR([cannot find protoc used by protobuf])
#fi
AX_PKG(FLANN, [], [-lflann_cpp], [flann/flann.hpp])
AX_PKG(Z, [], [-lz], [zlib.h])
AX_PKG(PNG, [Z], [-lpng], [png.h])
AX_PKG(JPEG, [], [-ljpeg], [stdio.h jpeglib.h]) # jpeglib.h requires stdio.h to have been included
AX_PKG(JP2, [JPEG], [-lopenjp2], [stdio.h openjpeg-2.0/openjpeg.h]) # same as above
AX_PKG(TIFF, [Z JP2], [-ltiff], [tiff.h])
AX_PKG(PROJ4, [M], [-lproj], [proj_api.h])
AX_PKG(GEOTIFF, [PROJ4 TIFF], [-lgeotiff], [geotiff.h])
AX_PKG(GDAL, [GEOTIFF PNG], [-lgdal], [gdal.h])
AX_PKG(ILMBASE, [M Z PTHREADS], [-lImath -lHalf -lIex -lIlmThread], [Iex.h], [OpenEXR])
AX_PKG(OPENEXR, [ILMBASE], [-lIlmImf], [ImfOutputFile.h], [OpenEXR])

AX_PKG(OPENCV,     [], [-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_videoio -lopencv_imgcodecs -lopencv_shape -lopencv_video -lopencv_superres -lopencv_objdetect -lopencv_stitching -lopencv_photo -lopencv_features2d -lopencv_xfeatures2d -lopencv_flann -lopencv_calib3d],  [ opencv2/core.hpp  opencv2/imgproc/imgproc.hpp opencv2/features2d.hpp opencv2/xfeatures2d.hpp opencv2/objdetect.hpp opencv2/calib3d.hpp])

AX_PKG(LASZIP,  [], [-llaszip], [laszip/laszip.hpp])
AX_PKG(LIBLAS,  [BOOST_THREAD BOOST BOOST_PROGRAM_OPTIONS LASZIP GDAL], [-llas], [liblas/liblas.hpp])
# Vision Workbench Core
AX_PKG(VW_CORE,  [BOOST_THREAD BOOST BOOST_PROGRAM_OPTIONS LAPACK], [-lvwCore],   [vw/Core/Log.h] )
AX_PKG(VW_MATH,  [VW_CORE],                   [-lvwMath],   [vw/Math/LinearAlgebra.h], [], [], [], [FLANN])
AX_PKG(VW_IMAGE, [VW_CORE],                   [-lvwImage],  [vw/Image/ImageView.h])
AX_PKG(VW_FILEIO,[VW_IMAGE Z OPENEXR GDAL BOOST_SYSTEM BOOST_FILESYSTEM],     [-lvwFileIO], [vw/FileIO/DiskImageResourceOpenEXR.h vw/FileIO/DiskImageResourceGDAL.h])
AX_PKG(VW_GEOMETRY, [VW_CORE], [-lvwGeometry], [vw/Geometry/Box.h])

AX_PKG(VW, [VW_FILEIO VW_IMAGE VW_MATH VW_CORE], [-lvwTools], [vw/tools/Common.h])

# Double Check again that VW_FILEIO really was built with the correct 3rdParty.
AX_EXTRACT_CPP_SYMBOL([VW_HAVE_PKG_OPENEXR], [#include <vw/config.h>],
  [VW_HAS_OPENEXR=$output], [VW_HAS_OPENEXR=0], ["$PKG_VW_CORE_CPPFLAGS"])
AX_EXTRACT_CPP_SYMBOL([VW_HAVE_PKG_GDAL], [#include <vw/config.h>],
  [VW_HAS_GDAL=$output], [VW_HAS_GDAL=0], ["$PKG_VW_CORE_CPPFLAGS"])
if test x"$HAVE_PKG_VW_CORE" = "xyes"; then
   if test x"$VW_HAS_OPENEXR" = "x0"; then
      AC_MSG_ERROR([VW doesn't have OpenEXR support! Rebuild VW with OpenEXR support.])
   fi
   if test x"$VW_HAS_GDAL" = "x0"; then
      AC_MSG_ERROR([VW doesn't have GDAL support! Rebuild VW with GDAL support.])
   fi
fi

# Vision Workbench Secondaries
AX_PKG(VW_CAMERA,         [VW PROJ4 BOOST_IOSTREAMS], [-lvwCamera],       [vw/Camera/CAHVModel.h])
AX_PKG(VW_STEREO,         [VW_CAMERA],          [-lvwStereo],       [vw/Stereo/CostFunctions.h])
AX_PKG(VW_CARTOGRAPHY,    [VW_CAMERA],          [-lvwCartography],  [vw/Cartography/GeoReference.h], [], [], [], [])
AX_PKG(VW_MOSAIC,         [VW_CARTOGRAPHY],     [-lvwMosaic],       [vw/Mosaic/ImageComposite.h])
AX_PKG(VW_INTEREST_POINT, [VW],                 [-lvwInterestPoint],[vw/InterestPoint/InterestData.h])
AX_PKG(VW_BUNDLEADJUSTMENT, [VW_INTEREST_POINT VW_CARTOGRAPHY VW_STEREO], [-lvwBundleAdjustment], [vw/BundleAdjustment/AdjustBase.h])

AM_CONDITIONAL(HAVE_PKG_VW_BUNDLEADJUSTMENT, [test "$HAVE_PKG_VW_BUNDLEADJUSTMENT" = "yes"])

AX_GROUP_PKG(VW_ALL,     [VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT])

AX_PKG_ONE_OF(GL,
  APPLE_GL,
    [AX_PKG_APPLE([GL], [], [-framework OpenGL -framework GLUT -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib])],
  LINUX_GL,
    [AX_PKG(X11, [], [-lXext -lX11 -lXmu -lXi -lXt], [])
     AX_PKG(LINUX_GL, [X11], [-lglut -lGLU -lGL], [GL/gl.h GL/glu.h GL/glut.h])])

AX_PKG(OPENSCENEGRAPH, [GL], [-losg -losgDB -losgSim -losgVolume -lOpenThreads -losgViewer -losgText -losgGA -losgFX -losgTerrain -losgUtil], [osg/ref_ptr])

AX_PKG_ONE_OF(SPICE,
    SPICE_SUBDIR, [AX_PKG(SPICE_SUBDIR, [], [-lcspice], [SpiceUsr.h SpiceZfc.h], [naif])],
    SPICE_TOP,    [AX_PKG(SPICE_TOP,    [], [-lcspice], [SpiceUsr.h SpiceZfc.h])])

if test "$HAVE_PKG_QT" = no; then
    ENABLE_QT_QMAKE=no
fi

if test "$ENABLE_QT_QMAKE" = no; then
  AX_GROUP_PKG([QT_QMAKE])
else
  # Qt with AutoTroll.
  AT_WITH_QT([+gui +sql +opengl +network], [], [], [HAVE_PKG_QT_QMAKE="no:missing QT"], [])

  if test x"$HAVE_PKG_QT_QMAKE" != x"no:missing QT"; then
      AT_REQUIRE_QT_VERSION([4.0], [HAVE_PKG_QT_QMAKE="no:wrong QT version"], [])
  fi

  # We seed QT_QMAKE's cppflags and libs with the ones qmake figured out
  PKG_QT_QMAKE_CPPFLAGS="$PKG_QT_QMAKE_CPPFLAGS $QT_CPPFLAGS"
  AX_PKG(QT_QMAKE, [], [$QT_LIBS], [QVector])
fi

AX_PKG_ONE_OF(QT,
    ARBITRARY_QT,
     [if test -z "$QT_ARBITRARY_MODULES" -a -z "$HAVE_PKG_ARBITRARY_MODULES"; then
        HAVE_PKG_ARBITRARY_QT="no:Set \$QT_ARBITRARY_MODULES"
      else
        for module in $QT_ARBITRARY_MODULES; do
          PKG_ARBITRARY_QT_NEEDED_HEADERS="$PKG_ARBITRARY_QT_NEEDED_HEADERS ${module}"
        done
        PKG_ARBITRARY_QT_CPPFLAGS="$PKG_ARBITRARY_QT_CPPFLAGS -DQT_SHARED"
        AX_PKG(ARBITRARY_QT, [], [], [$PKG_ARBITRARY_QT_NEEDED_HEADERS])
      fi],
    APPLE_QMAKE_QT, [AX_PKG_APPLE(QMAKE_QT, [GL QT_QMAKE], [])],
    LINUX_QMAKE_QT,
     [AX_PKG(QT_QMAKE_OPENGL,  [QT_QMAKE], [], [Qt5OpenGL])
      AX_PKG(QT_QMAKE_GUI,     [QT_QMAKE], [], [QLabel])
      AX_PKG(QT_QMAKE_SQL,     [QT_QMAKE], [], [Qt5Sql])
      AX_PKG(QT_QMAKE_NETWORK, [QT_QMAKE], [], [QtNetwork])
      AX_GROUP_PKG(LINUX_QMAKE_QT, [QT_QMAKE QT_QMAKE_GUI QT_QMAKE_OPENGL QT_QMAKE_SQL QT_QMAKE_NETWORK])],
    GUESS_QT,
     [PKG_QT_GUESS_BASE_CPPFLAGS="$PKG_QT_GUESS_BASE_CPPFLAGS -DQT_SHARED"
      AX_PKG(QT_GUESS_BASE,   [],                           [],           [QtCore/qstring.h])
      AX_PKG(QT_GUESS_CORE,   [QT_GUESS_BASE],              [-lQt5Core],   [Qt5Core],   [Qt5Core])
      AX_PKG(QT_GUESS_GUI,    [QT_GUESS_CORE],              [-lQt5Gui],    [Qt5Gui],    [Qt5Gui])
      AX_PKG(QT_GUESS_SQL,    [QT_GUESS_CORE],              [-lQt5Sql],    [Qt5Sql],    [Qt5Sql])
      AX_PKG(QT_GUESS_NETWORK,[QT_GUESS_CORE],              [-lQt5Network],[Qt5Network],[Qt5Network])
      AX_PKG(QT_GUESS_OPENGL, [QT_GUESS_CORE QT_GUESS_GUI], [-lQt5OpenGL], [Qt5OpenGL], [Qt5OpenGL])
      AX_GROUP_PKG(GUESS_QT,  [QT_GUESS_BASE QT_GUESS_CORE QT_GUESS_GUI QT_GUESS_SQL QT_GUESS_NETWORK QT_GUESS_OPENGL])])

# Create qt.conf so that we can run stereo_gui in dev mode
AX_PKG_SET_QT_PLUGINS_PATH([$BASE])

AX_PKG_ONE_OF(QWT,
    APPLE_QWT, [AX_PKG_APPLE(QWT, [QT], [-framework qwt])],
    PLAIN_QWT, [AX_PKG(PLAIN_QWT, [QT], [-lqwt], [qwt.h])],
    SUFFX_QWT, [AX_PKG(SUFFX_QWT, [QT], [-lqwt-qt4], [qwt.h])])

AX_PKG_ONE_OF(GSL,
    GSL_HASBLAS,  [AX_PKG(GSL_HASBLAS,  [LAPACK], [-lgsl])],
    GSL_NEEDBLAS, [AX_PKG(GSL_NEEDBLAS, [LAPACK], [-lgsl -lgslcblas])])

AX_PKG(CURL,    [], [-lcurl], [curl/curl.h])
AX_PKG(GEOID, [], [-legm2008], [])
AX_PKG(PCL,    [FLANN Z PNG BOOST BOOST_REGEX BOOST_THREAD BOOST_FILESYSTEM BOOST_SYSTEM BOOST_PROGRAM_OPTIONS BOOST_DATETIME BOOST_IOSTREAMS], 
                    [-lpcl_common -lpcl_io_ply -lpcl_keypoints -lpcl_recognition -lpcl_search -lpcl_surface -lpcl_features   -lpcl_io -lpcl_ml -lpcl_registration -lpcl_segmentation -lpcl_tracking -lpcl_filters -lpcl_kdtree  -lpcl_octree -lpcl_sample_consensus -lpcl_stereo], [pcl/pcl_base.h])
AX_PKG(GEOS,    [], [-lgeos -lgeos_c], [geos.h])
AX_PKG(XERCESC, [CURL], [-lxerces-c], [xercesc/dom/DOM.hpp xercesc/util/XMLString.hpp xercesc/parsers/XercesDOMParser.hpp])
AX_PKG(SUITESPARSE, [], [-lsuitesparseconfig], [SuiteSparse_config.h])
AX_PKG(AMD, [], [-lamd], [], [], [], [], [SUITESPARSE])
AX_PKG(COLAMD, [], [-lcolamd], [], [], [], [], [SUITESPARSE])
AX_PKG(CHOLMOD, [AMD COLAMD LAPACK], [-lcholmod])
AX_PKG(HDF5, [], [-lhdf5 -lhdf5_hl -lhdf5_cpp -lhdf5_hl_cpp])
AX_PKG(GLOG,    [], [-lglog], [glog/logging.h])
AX_PKG(CERES, [CHOLMOD GLOG], [-lceres], [ceres/ceres.h])
AX_PKG_ONE_OF(SUPERLU,
    SUPERLU_STATIC, [AX_PKG(SUPERLU_STATIC, [LAPACK], [-lyou_should_override_me])],
    SUPERLU_PLAIN,  [AX_PKG(SUPERLU_PLAIN,  [LAPACK], [-lsuperlu], [])],
    SUPERLU_VER_30, [AX_PKG(SUPERLU_VER_30, [LAPACK], [-lsuperlu_3.0])])

AX_PKG_ONE_OF(KAKADU,
    KAKADU_SPLIT,   [AX_PKG(KAKADU_SPLIT,   [], [-lkdu -lkdu_apps])],
    KAKADU_63_JOIN, [AX_PKG(KAKADU_63_JOIN, [], [-lkdu_a63R])])

if test x"$host_vendor" = "xapple"; then
    AX_PKG(ISIS3RDPARTY, [LAPACK GSL QWT GEOS SPICE XERCESC PROTOBUF CHOLMOD GEOTIFF TIFF HDF5],        [],[],[],[],[],[KAKADU])
    OPENMPFLAGS=""
    AC_DEFINE([OSX_BUILD], [1], [Building on OSX?])
else
    AX_PKG(ISIS3RDPARTY, [LAPACK GSL QWT GEOS SPICE XERCESC SUPERLU PROTOBUF CHOLMOD GEOTIFF TIFF HDF5 OPENCV PCL],[],[],[],[],[],[KAKADU])
    OPENMPFLAGS="-fopenmp"
    AC_DEFINE([OSX_BUILD], [0], [Building on OSX?])
fi
AC_SUBST(OPENMPFLAGS)

AX_PKG(ISIS, [ISIS3RDPARTY QT], [-lisis3], [SpecialPixel.h])
if test x"$HAVE_PKG_ISIS" = "xyes"; then
#  AX_PKG_ISIS_CHECK_VERSION()
  AC_DEFINE_UNQUOTED([ISIS_VERSION],
                     ["3.5.0"],
                     [The version of ISIS that this software was built against.])
fi

AX_PKG(EIGEN,  [BOOST_THREAD BOOST BOOST_PROGRAM_OPTIONS], [], [])
AX_PKG(LIBNABO, [], [-lnabo], [])
AX_PKG(LIBPOINTMATCHER, [LIBNABO EIGEN BOOST_THREAD BOOST_FILESYSTEM BOOST_SYSTEM BOOST_PROGRAM_OPTIONS BOOST_DATETIME], [-lpointmatcher], [])

# These are here (instead of inside the PKG macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(HAVE_PKG_SPICE,          [test "$HAVE_PKG_SPICE" = "yes"])
AM_CONDITIONAL(HAVE_PKG_ISIS,           [test "$HAVE_PKG_ISIS" = "yes"])
AM_CONDITIONAL(HAVE_PKG_OPENSCENEGRAPH, [test "$HAVE_PKG_OPENSCENEGRAPH" = "yes"])
AM_CONDITIONAL(HAVE_PKG_PROTOBUF,       [test "$HAVE_PKG_PROTOBUF" = "yes"])
AM_CONDITIONAL(HAVE_PKG_FLANN,          [test "$HAVE_PKG_FLANN" = "yes"])
AM_CONDITIONAL(HAVE_PKG_OPENCV,         [test "$HAVE_PKG_OPENCV"     = "yes"])

##################################################
# module definitions
##################################################

AX_GROUP_PKG(BOOST_MOST, [BOOST_DATETIME BOOST_IOSTREAMS BOOST])

# Module
# Usage: AX_MODULE(<name>, <directory>, <library>, <default>, <prerequisites>, <required dependencies>[, <optional dependencies>])
AX_MODULE(CORE,     [src/asp/Core],     [libaspCore.la],     yes, [], [VW_ALL LASZIP LIBLAS OPENCV], [LAPACK])
AX_MODULE(CAMERA,   [src/asp/Camera],   [libaspCamera.la],   yes, [CORE], [VW], [XERCESC])
AX_MODULE(SPICEIO,  [src/asp/SpiceIO],  [libaspSpiceIO.la],  yes, [CORE], [VW SPICE])
AX_MODULE(ISISIO,   [src/asp/IsisIO],   [libaspIsisIO.la],   yes, [CORE], [ISIS VW_CARTOGRAPHY])
AX_MODULE(SESSIONS, [src/asp/Sessions], [libaspSessions.la], yes, [CORE], [CAMERA ISISIO SPICEIO], [])
AX_MODULE(GUI,      [src/asp/GUI],      [libaspGUI.la],      yes, [CORE VW_GEOMETRY], [QT VW_BUNDLEADJUSTMENT], [])
AX_MODULE(PYTHON,   [src/asp/Python],   [],                  yes, [],     [],   [])

# Tools - The yes/no means to enable by default
AX_APP(PC_ALIGN,         [src/asp/Tools], yes, [CORE LIBPOINTMATCHER CERES])
AX_APP(BUNDLE_ADJUST,    [src/asp/Tools], yes, [CORE VW_BUNDLEADJUSTMENT SESSIONS CERES OPENCV])
AX_APP(SFS,              [src/asp/Tools], yes, [SESSIONS CERES])
AX_APP(DEM_MOSAIC,       [src/asp/Tools], yes, [CORE])
AX_APP(DEMPROFILE,       [src/asp/Tools], no,  [CORE])
AX_APP(DISPARITYDEBUG,   [src/asp/Tools], yes, [CORE])
AX_APP(GEODIFF,          [src/asp/Tools], yes, [CORE])
AX_APP(HSVMERGE,         [src/asp/Tools], yes, [CORE])
AX_APP(LRONACJITREG,     [src/asp/Tools], yes, [CORE])
AX_APP(MER2CAMERA,       [src/asp/Tools], yes, [CORE])
AX_APP(ORBITVIZ,         [src/asp/Tools], yes, [SESSIONS])
AX_APP(ORTHOPROJECT,     [src/asp/Tools], yes, [SESSIONS])
AX_APP(POINT2DEM,        [src/asp/Tools], yes, [CORE])
AX_APP(POINT2MESH,       [src/asp/Tools], yes, [OPENSCENEGRAPH CORE])
AX_APP(POINT2LAS,        [src/asp/Tools], yes, [CORE])
AX_APP(DEM_GEOID,        [src/asp/Tools], yes, [GEOID CORE])
AX_APP(MAPPROJECT,       [src/asp/Tools], yes, [SESSIONS])
AX_APP(RPC_GEN,          [src/asp/Tools], yes, [SESSIONS])
AX_APP(TIF_MOSAIC,       [src/asp/Tools], yes, [CORE])
AX_APP(STEREO,           [src/asp/Tools], yes, [SESSIONS])
AX_APP(STEREO_TRI,       [src/asp/Tools], yes, [SESSIONS VW_BUNDLEADJUSTMENT CERES])
AX_APP(STEREO_GUI,       [src/asp/Tools], yes, [VW_BUNDLEADJUSTMENT SESSIONS GUI])
AX_APP(WV_CORRECT,       [src/asp/Tools], yes, [SESSIONS])
AX_APP(IMAGE_CALC,       [src/asp/Tools], yes, [CORE])
AX_APP(PANSHARP,         [src/asp/Tools], yes, [CORE CAMERA])
AX_APP(PC_MERGE,         [src/asp/Tools], yes, [CORE])
AX_APP(DATUM_CONVERT,    [src/asp/Tools], yes, [CORE])
AX_APP(ASTER2ASP,        [src/asp/Tools], yes, [CORE CAMERA])
AX_APP(ADD_SPOT_RPC,     [src/asp/Tools], yes, [CORE CAMERA])
AX_APP(DISP_AVG,         [src/asp/WVCorrect], yes, [CORE])
AX_APP(ICEBRIDGE,        [src/asp/IceBridge], yes, [CORE])
AX_APP(ORTHO2PINHOLE,    [src/asp/IceBridge], yes, [SESSIONS])
AX_APP(CSV_FILTER,       [src/asp/Hidden], yes, [CORE])
AX_APP(S2P_PPRC,         [src/asp/Hidden], yes, [CORE])
AX_APP(S2P_FLTR,         [src/asp/Hidden], yes, [CORE])
AX_APP(CAM2RPC,          [src/asp/Tools], yes, [SESSIONS])
AX_APP(CAMERA_FOOTPRINT, [src/asp/Tools], yes, [SESSIONS])

# These are here (instead of inside the APP macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(MAKE_MODULE_CORE,     [test "$MAKE_MODULE_CORE"     = "yes"])
AM_CONDITIONAL(MAKE_MODULE_CAMERA,   [test "$MAKE_MODULE_CAMERA"   = "yes"])
AM_CONDITIONAL(MAKE_MODULE_ISISIO,   [test "$MAKE_MODULE_ISISIO"   = "yes"])
AM_CONDITIONAL(MAKE_MODULE_SPICEIO,  [test "$MAKE_MODULE_SPICEIO"  = "yes"])
AM_CONDITIONAL(MAKE_MODULE_SESSIONS, [test "$MAKE_MODULE_SESSIONS" = "yes"])
AM_CONDITIONAL(MAKE_MODULE_GUI,      [test "$MAKE_MODULE_GUI"      = "yes"])
AM_CONDITIONAL(MAKE_MODULE_PYTHON,   [test "$MAKE_MODULE_PYTHON"   = "yes"])

AM_CONDITIONAL(MAKE_APP_PC_ALIGN,      [test "$MAKE_APP_PC_ALIGN"      = "yes"])
AM_CONDITIONAL(MAKE_APP_BUNDLE_ADJUST, [test "$MAKE_APP_BUNDLE_ADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_SFS,           [test "$MAKE_APP_SFS"           = "yes"])
AM_CONDITIONAL(MAKE_APP_DEM_MOSAIC,    [test "$MAKE_APP_DEM_MOSAIC"    = "yes"])
AM_CONDITIONAL(MAKE_APP_DEMPROFILE,    [test "$MAKE_APP_DEMPROFILE"    = "yes"])
AM_CONDITIONAL(MAKE_APP_DISPARITYDEBUG, [test "$MAKE_APP_DISPARITYDEBUG" = "yes"])
AM_CONDITIONAL(MAKE_APP_GEODIFF,       [test "$MAKE_APP_GEODIFF"  = "yes"])
AM_CONDITIONAL(MAKE_APP_HSVMERGE,      [test "$MAKE_APP_HSVMERGE" = "yes"])
AM_CONDITIONAL(MAKE_APP_LRONACJITREG,  [test "$MAKE_APP_LRONACJITREG" == "yes"])
AM_CONDITIONAL(MAKE_APP_MER2CAMERA,    [test "$MAKE_APP_MER2CAMERA" = "yes"])
AM_CONDITIONAL(MAKE_APP_ORBITVIZ,      [test "$MAKE_APP_ORBITVIZ"   = "yes"])
AM_CONDITIONAL(MAKE_APP_ORTHOPROJECT,      [test "$MAKE_APP_ORTHOPROJECT" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2DEM,  [test "$MAKE_APP_POINT2DEM"  = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2MESH, [test "$MAKE_APP_POINT2MESH" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2LAS,  [test "$MAKE_APP_POINT2LAS"  = "yes"])
AM_CONDITIONAL(MAKE_APP_DEM_GEOID,  [test "$MAKE_APP_DEM_GEOID"  = "yes"])
AM_CONDITIONAL(MAKE_APP_MAPPROJECT, [test "$MAKE_APP_MAPPROJECT" = "yes"])
AM_CONDITIONAL(MAKE_APP_RPC_GEN,    [test "$MAKE_APP_RPC_GEN"    = "yes"])
AM_CONDITIONAL(MAKE_APP_TIF_MOSAIC, [test "$MAKE_APP_TIF_MOSAIC" = "yes"])
AM_CONDITIONAL(MAKE_APP_STEREO,     [test "$MAKE_APP_STEREO"     = "yes"])
AM_CONDITIONAL(MAKE_APP_STEREO_TRI, [test "$MAKE_APP_STEREO_TRI" = "yes"])
AM_CONDITIONAL(MAKE_APP_STEREO_GUI, [test "$MAKE_APP_STEREO_GUI" = "yes"])
AM_CONDITIONAL(MAKE_APP_WV_CORRECT, [test "$MAKE_APP_WV_CORRECT" = "yes"])
AM_CONDITIONAL(MAKE_APP_IMAGE_CALC, [test "$MAKE_APP_IMAGE_CALC" = "yes"])
AM_CONDITIONAL(MAKE_APP_PANSHARP,   [test "$MAKE_APP_PANSHARP"   = "yes"])
AM_CONDITIONAL(MAKE_APP_PC_MERGE,   [test "$MAKE_APP_PC_MERGE"   = "yes"])
AM_CONDITIONAL(MAKE_APP_DATUM_CONVERT, [test "$MAKE_APP_DATUM_CONVERT" = "yes"])
AM_CONDITIONAL(MAKE_APP_DISP_AVG,   [test "$MAKE_APP_DISP_AVG"   = "yes"])
AM_CONDITIONAL(MAKE_APP_ASTER2ASP,  [test "$MAKE_APP_ASTER2ASP"  = "yes"])
AM_CONDITIONAL(MAKE_APP_ICEBRIDGE,     [test "$MAKE_APP_ICEBRIDGE"  = "yes"])
AM_CONDITIONAL(MAKE_APP_ORTHO2PINHOLE, [test "$MAKE_APP_ORTHO2PINHOLE" = "yes"])
AM_CONDITIONAL(MAKE_APP_CSV_FILTER,  [test "$MAKE_APP_CSV_FILTER"   = "yes"])
AM_CONDITIONAL(MAKE_APP_CAM2RPC, [test "$MAKE_APP_CAM2RPC" = "yes"])
AM_CONDITIONAL(MAKE_APP_S2P_PPRC,  [test "$MAKE_APP_S2P_PPRC"   = "yes"])
AM_CONDITIONAL(MAKE_APP_S2P_FLTR,  [test "$MAKE_APP_S2P_FLTR"   = "yes"])
AM_CONDITIONAL(MAKE_APP_CAMERA_FOOTPRINT,   [test "$MAKE_APP_CAMERA_FOOTPRINT"   = "yes"])

##################################################
# final processing
##################################################

ASP_CPPFLAGS="-I\${top_srcdir}/src $OTHER_CPPFLAGS"
ASP_LDFLAGS="$OTHER_LDFLAGS"

# Do this last, to make sure it's not overridden
if test x"$ENABLE_STRICT_ALIASING" = x"no"; then
    CXXFLAGS="$CXXFLAGS -fno-strict-aliasing"
fi

AX_LOG([using CPPFLAGS=$CPPFLAGS])
AX_LOG([using ASP_CPPFLAGS=$ASP_CPPFLAGS])
AX_LOG([using CFLAGS=$CFLAGS])
AX_LOG([using CXXFLAGS=$CXXFLAGS])
AX_LOG([using LDFLAGS=$LDFLAGS])
AX_LOG([using ASP_LDFLAGS=$ASP_LDFLAGS])
AX_LOG([using ASP_ENABLE_EXCEPTIONS=$ENABLE_EXCEPTIONS])

AC_SUBST(ASP_CPPFLAGS)
AC_SUBST(ASP_LDFLAGS)


##################################################
# makefiles to process
##################################################

AC_CONFIG_FILES([                        \
  Makefile                               \
  data/Makefile                          \
  src/Makefile                           \
  src/asp/Core/Makefile                  \
  src/asp/Core/tests/Makefile            \
  src/asp/Camera/Makefile                \
  src/asp/Camera/tests/Makefile          \
  src/asp/IsisIO/Makefile                \
  src/asp/IsisIO/tests/Makefile          \
  src/asp/Makefile                       \
  src/asp/Sessions/Makefile              \
  src/asp/Sessions/tests/Makefile        \
  src/asp/SpiceIO/Makefile               \
  src/asp/GUI/Makefile                   \
  src/asp/Python/Makefile                \
  src/asp/Tools/Makefile                 \
  src/asp/WVCorrect/Makefile             \
  src/asp/IceBridge/Makefile             \
  src/asp/Hidden/Makefile
])

AC_OUTPUT
