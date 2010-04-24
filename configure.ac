dnl __BEGIN_LICENSE__
dnl Copyright (C) 2009-2010 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
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
AC_INIT([NASA Ames Stereo Pipeline], [1.0.1_pre], [asp@nx.arc.nasa.gov], [StereoPipeline])
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE([nostdinc 1.8.5])
AX_CONFIG_HEADER_PREFIX([src/asp/asp_config.h], [ASP_])

dnl Fix for early autoconf that doesn't export these
AC_SUBST([abs_top_srcdir])
AC_SUBST([abs_top_builddir])

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
    [AX_PKG_BOOST_LIB(DATETIME,        [-lboost_date_time],       [boost/date_time/gregorian/gregorian_types.hpp])],
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
    [AX_PKG_APPLE(LAPACK, [], [-framework vecLib])],
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

# Vision Workbench Core
AX_PKG(VW_CORE,  [BOOST_THREAD BOOST BOOST_PROGRAM_OPTIONS LAPACK], [-lvwCore],   [vw/Core/Log.h] )
AX_PKG(VW_MATH,  [VW_CORE],                   [-lvwMath],   [vw/Math/LinearAlgebra.h])
AX_PKG(VW_IMAGE, [VW_CORE],                   [-lvwImage],  [vw/Image/ImageView.h])
AX_PKG(VW_FILEIO,[VW_IMAGE BOOST_SYSTEM BOOST_FILESYSTEM],     [-lvwFileIO], [vw/FileIO/DiskImageResourceOpenEXR.h vw/FileIO/DiskImageResourceGDAL.h])
AX_GROUP_PKG(VW, [VW_FILEIO VW_IMAGE VW_MATH VW_CORE])

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
AX_PKG(VW_PLATE,          [VW_FILEIO],          [-lvwPlate],        [vw/Plate/PlateFile.h])
AX_PKG(VW_CAMERA,         [VW BOOST_IOSTREAMS], [-lvwCamera],       [vw/Camera/CAHVModel.h])
AX_PKG(VW_STEREO,         [VW_CAMERA],          [-lvwStereo],       [vw/Stereo/OptimizedCorrelator.h])
AX_PKG(VW_CARTOGRAPHY,    [VW_CAMERA],          [-lvwCartography],  [vw/Cartography/GeoReference.h])
AX_PKG(VW_INTEREST_POINT, [VW],                 [-lvwInterestPoint],[vw/InterestPoint/InterestData.h])
AX_PKG(VW_PHOTOMETRY,     [VW],                 [-lvwPhotometry])
AX_PKG(VW_BUNDLEADJUSTMENT, [VW_INTEREST_POINT VW_CARTOGRAPHY VW_STEREO], [-lvwBundleAdjustment], [vw/BundleAdjustment/AdjustBase.h])

AM_CONDITIONAL(HAVE_PKG_VW_BUNDLEADJUSTMENT, [test "$HAVE_PKG_VW_BUNDLEADJUSTMENT" = "yes"])

AX_PKG_ONE_OF(GL,
  APPLE_GL,
    [AX_PKG_APPLE([GL], [], [-framework OpenGL -framework GLUT -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib])],
  LINUX_GL,
    [AX_PKG(X11, [], [-lXext -lX11 -lXmu -lXi -lXt], [])
     AX_PKG(LINUX_GL, [X11], [-lglut -lGLU -lGL], [GL/gl.h GL/glu.h GL/glut.h])])

AX_PKG(OPENSCENEGRAPH, [GL], [-losg -losgDB -lOpenThreads -losgViewer -losgText -losgGA -losgFX -losgTerrain -losgUtil], [osg/ref_ptr])

AX_PKG(SPICE, [], [-lcspice], [SpiceUsr.h SpiceZfc.h])

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
          PKG_ARBITRARY_QT_NEEDED_HEADERS="$PKG_ARBITRARY_QT_NEEDED_HEADERS Qt${module}"
          PKG_ARBITRARY_QT_LIBS="-lQt${module} $PKG_ARBITRARY_QT_LIBS"
        done
      fi
      PKG_ARBITRARY_QT_CPPFLAGS="$PKG_ARBITRARY_QT_CPPFLAGS -DQT_SHARED"
      AX_PKG(ARBITRARY_QT, [], [-lQtCore], [$PKG_ARBITRARY_QT_NEEDED_HEADERS])],
    APPLE_QMAKE_QT, [AX_PKG_APPLE(QMAKE_QT, [GL QT_QMAKE], [])],
    LINUX_QMAKE_QT,
     [AX_PKG(QT_QMAKE_OPENGL,  [QT_QMAKE], [], [QtOpenGL])
      AX_PKG(QT_QMAKE_GUI,     [QT_QMAKE], [], [QLabel])
      AX_PKG(QT_QMAKE_SQL,     [QT_QMAKE], [], [QtSql])
      AX_PKG(QT_QMAKE_NETWORK, [QT_QMAKE], [], [QtNetwork])
      AX_GROUP_PKG(LINUX_QMAKE_QT, [QT_QMAKE QT_QMAKE_GUI QT_QMAKE_OPENGL QT_QMAKE_SQL QT_QMAKE_NETWORK])],
    GUESS_QT,
     [PKG_QT_GUESS_BASE_CPPFLAGS="$PKG_QT_GUESS_BASE_CPPFLAGS -DQT_SHARED"
      AX_PKG(QT_GUESS_BASE,   [],                           [],           [QtCore/qstring.h])
      AX_PKG(QT_GUESS_CORE,   [QT_GUESS_BASE],              [-lQtCore],   [QtCore],   [QtCore])
      AX_PKG(QT_GUESS_GUI,    [QT_GUESS_CORE],              [-lQtGui],    [QtGui],    [QtGui])
      AX_PKG(QT_GUESS_SQL,    [QT_GUESS_CORE],              [-lQtSql],    [QtSql],    [QtSql])
      AX_PKG(QT_GUESS_NETWORK,[QT_GUESS_CORE],              [-lQtNetwork],[QtNetwork],[QtNetwork])
      AX_PKG(QT_GUESS_OPENGL, [QT_GUESS_CORE QT_GUESS_GUI], [-lQtOpenGL], [QtOpenGL], [QtOpenGL])
      AX_GROUP_PKG(GUESS_QT,  [QT_GUESS_BASE QT_GUESS_CORE QT_GUESS_GUI QT_GUESS_SQL QT_GUESS_NETWORK QT_GUESS_OPENGL])])

AX_PKG_ONE_OF(QWT,
    APPLE_QWT, [AX_PKG_APPLE(QWT, [QT], [-lqwt.5])],
    PLAIN_QWT, [AX_PKG(PLAIN_QWT, [QT], [-lqwt])],
    SUFFX_QWT, [AX_PKG(SUFFX_QWT, [QT], [-lqwt-qt4])])

AX_PKG_ONE_OF(GSL,
    GSL_HASBLAS,  [AX_PKG(GSL_HASBLAS,  [LAPACK], [-lgsl])],
    GSL_NEEDBLAS, [AX_PKG(GSL_NEEDBLAS, [LAPACK], [-lgsl -lgslcblas])])

if test x"$host_vendor" = "xapple"; then
    AX_PKG(GEOS,    [],             [-lgeos-3.0.0], [])
    AX_PKG(SUPERLU, [],             [-lsuperlu_3.0], [])
    AX_PKG(XERCESC, [],             [-lxerces-c.28], [])
else
    AX_PKG(GEOS,    [],             [-lgeos])
    AX_PKG(SUPERLU, [LAPACK],       [-lsuperlu])
    AX_PKG(XERCESC, [],             [-lxerces-c])
fi

dnl We need to link ISIS3RDPARTY against LAPACK because, on some systems,
dnl the ISIS compilation does not resolve certain BLAS symbols that its
dnl dependent 3rd party libraries depend on (for whatever reason).
dnl Thus, we link against LAPACK here because it defines these missing
dnl BLAS symbols in the event that we need them.
AX_PKG(ISIS3RDPARTY, [SUPERLU LAPACK GSL QWT GEOS SPICE XERCESC], [])
AX_PKG(ISIS, [ISIS3RDPARTY QT], [-lisis3], [SpecialPixel.h])

AX_PKG(PROTOBUF, [PTHREADS], [-lprotobuf], [google/protobuf/stubs/common.h])

# These are here (instead of inside the PKG macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(HAVE_PKG_SPICE, [test "$HAVE_PKG_SPICE" = "yes"])
AM_CONDITIONAL(HAVE_PKG_ISIS, [test "$HAVE_PKG_ISIS" = "yes"])
AM_CONDITIONAL(HAVE_PKG_OPENSCENEGRAPH, [test "$HAVE_PKG_OPENSCENEGRAPH" = "yes"])

##################################################
# module definitions
##################################################

AX_GROUP_PKG(BOOST_MOST, [BOOST_DATETIME BOOST_IOSTREAMS BOOST])
AX_GROUP_PKG(VW_ALL,     [VW_CARTOGRAPHY VW_STEREO VW_CAMERA VW_INTEREST_POINT VW])

# Module
AX_MODULE(CORE,     [src/asp/Core],     [libaspCore.la],     yes, [], [VW_ALL BOOST_MOST], [LAPACK])
AX_MODULE(SPICEIO,  [src/asp/SpiceIO],  [libaspSpiceIO.la],  yes, [], [VW BOOST_MOST SPICE])
AX_MODULE(ISISIO,   [src/asp/IsisIO],   [libaspIsisIO.la],   yes, [], [ISIS VW_CARTOGRAPHY BOOST])
AX_MODULE(SESSIONS, [src/asp/Sessions], [libaspSessions.la], yes, [CORE], [], [ISISIO SPICEIO])
AX_MODULE(MPI,      [src/asp/MPI],      [],                  no,  [VW_BUNDLEADJUSTMENT ISISIO CORE], [BOOST_SERIALIZATION BOOST_MPI OPEN_MPI VW_ALL])

# Tools
AX_APP(STEREO,           [src/asp/Tools], yes, [SESSIONS])
AX_APP(ORTHOPROJECT,     [src/asp/Tools], yes, [SESSIONS])
AX_APP(BUNDLEADJUST,     [src/asp/Tools], no,  [VW_BUNDLEADJUSTMENT SESSIONS])
AX_APP(ORBITVIZ,         [src/asp/Tools], yes, [SESSIONS])
AX_APP(DISPARITYDEBUG,   [src/asp/Tools], yes, [VW_STEREO])
AX_APP(POINT2MESH,       [src/asp/Tools], yes, [OPENSCENEGRAPH VW])
AX_APP(POINT2DEM,        [src/asp/Tools], yes, [CORE])
AX_APP(RMAX2CAHVOR,      [src/asp/Tools], no,  [SESSIONS])
AX_APP(RMAXADJUST,       [src/asp/Tools], no,  [VW_BUNDLEADJUSTMENT SESSIONS])
AX_APP(BUNDLEVIS,        [src/asp/Tools], no,  [OPENSCENEGRAPH VW_BUNDLEADJUSTMENT])
AX_APP(ISISADJUST,       [src/asp/Tools], yes, [VW_BUNDLEADJUSTMENT ISISIO CORE])
AX_APP(RESULTS,          [src/asp/Tools], yes, [ISISIO])
AX_APP(RECONSTRUCT,      [src/asp/Tools], no,  [VW_PHOTOMETRY VW_CARTOGRAPHY VW])
AX_APP(ALIGNDEM,         [src/asp/Tools], yes, [VW_CARTOGRAPHY VW_INTEREST_POINT])
AX_APP(GEODIFF,          [src/asp/Tools], yes, [ISISIO VW BOOST])
AX_APP(DEMPROFILE,       [src/asp/Tools], yes, [VW_CARTOGRAPHY VW])
AX_APP(PLATEORTHOPROJECT,[src/asp/Tools], no,  [SESSIONS ISISIO VW_PLATE])
AX_APP(HSVMERGE,         [src/asp/Tools], yes, [VW_CARTOGRAPHY VW])

# Toolkits (like module, but doesn't build a library)
AX_MODULE(PHOTOMETRYTK, [src/asp/PhotometryTK], [libaspPhotometryTK.la], no,  [PROTOBUF VW_PLATE], [BOOST])

# These are here (instead of inside the APP macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(MAKE_MODULE_CORE, [test "$MAKE_MODULE_CORE" = "yes"])
AM_CONDITIONAL(MAKE_MODULE_ISISIO, [test "$MAKE_MODULE_ISISIO" = "yes"])
AM_CONDITIONAL(MAKE_MODULE_SPICEIO, [test "$MAKE_MODULE_SPICEIO" = "yes"])
AM_CONDITIONAL(MAKE_MODULE_SESSIONS, [test "$MAKE_MODULE_SESSIONS" = "yes"])
AM_CONDITIONAL(MAKE_MODULE_MPI, [test "$MAKE_MODULE_MPI" = "yes"])
AM_CONDITIONAL(MAKE_MODULE_PHOTOMETRYTK, [test "$MAKE_MODULE_PHOTOMETRYTK" = "yes"])

AM_CONDITIONAL(MAKE_APP_STEREO, [test "$MAKE_APP_STEREO" = "yes"])
AM_CONDITIONAL(MAKE_APP_STEREOGUI, [test "$MAKE_APP_STEREOGUI" = "yes"])
AM_CONDITIONAL(MAKE_APP_ORTHOPROJECT, [test "$MAKE_APP_ORTHOPROJECT" = "yes"])
AM_CONDITIONAL(MAKE_APP_BUNDLEADJUST, [test "$MAKE_APP_BUNDLEADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_ORBITVIZ, [test "$MAKE_APP_ORBITVIZ" = "yes"])
AM_CONDITIONAL(MAKE_APP_DISPARITYDEBUG, [test "$MAKE_APP_DISPARITYDEBUG" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2MESH, [test "$MAKE_APP_POINT2MESH" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2DEM, [test "$MAKE_APP_POINT2DEM" = "yes"])
AM_CONDITIONAL(MAKE_APP_RMAX2CAHVOR, [test "$MAKE_APP_RMAX2CAHVOR" = "yes"])
AM_CONDITIONAL(MAKE_APP_RMAXADJUST, [test "$MAKE_APP_RMAXADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_BUNDLEVIS, [test "$MAKE_APP_BUNDLEVIS" = "yes"])
AM_CONDITIONAL(MAKE_APP_ISISADJUST, [test "$MAKE_APP_ISISADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_RESULTS, [test "$MAKE_APP_RESULTS" = "yes"])
AM_CONDITIONAL(MAKE_APP_RECONSTRUCT, [test "$MAKE_APP_RECONSTRUCT" = "yes"])
AM_CONDITIONAL(MAKE_APP_ALIGNDEM, [test "$MAKE_APP_ALIGNDEM" = "yes"])
AM_CONDITIONAL(MAKE_APP_GEODIFF, [test "$MAKE_APP_GEODIFF" = "yes"])
AM_CONDITIONAL(MAKE_APP_DEMPROFILE, [test "$MAKE_APP_DEMPROFILE" = "yes"])
AM_CONDITIONAL(MAKE_APP_PLATEORTHOPROJECT, [test "$MAKE_APP_PLATEORTHOPROJECT" = "yes"])
AM_CONDITIONAL(MAKE_APP_HSVMERGE, [test "$MAKE_APP_HSVMERGE" = "yes"])

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

AC_CONFIG_FILES([                       \
  Makefile                              \
  src/Makefile                          \
  src/asp/Makefile                      \
  src/asp/Core/Makefile                 \
  src/asp/SpiceIO/Makefile              \
  src/asp/IsisIO/Makefile               \
  src/asp/IsisIO/tests/Makefile         \
  src/asp/Sessions/Makefile             \
  src/asp/Sessions/ISIS/Makefile        \
  src/asp/Sessions/Keypoint/Makefile    \
  src/asp/Sessions/Pinhole/Makefile     \
  src/asp/Sessions/RMAX/Makefile        \
  src/asp/MPI/Makefile                  \
  src/asp/Tools/Makefile                \
  src/asp/PhotometryTK/Makefile         \
  src/asp/PhotometryTK/tests/Makefile   \
])

AC_OUTPUT
