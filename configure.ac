dnl __BEGIN_LICENSE__
dnl Copyright (C) 2006, 2007 United States Government as represented by
dnl the Administrator of the National Aeronautics and Space Administration.
dnl All Rights Reserved.
dnl __END_LICENSE__

m4_pattern_forbid([^AX_])
m4_pattern_allow([AX_CFLAGS])
m4_pattern_allow([AX_INCLUDE_DIR])
m4_pattern_allow([AX_LIBDIR])
m4_pattern_allow([AX_OTHER_LIBDIR])

##################################################
# A few starting bits
##################################################

dnl TODO BEFORE RELEASE: update package version
AC_INIT([NASA Ames Stereo Pipeline], [2.1], [asp@nx.arc.nasa.gov], [StereoPipeline])
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE([nostdinc])
AX_CONFIG_HEADER_PREFIX([src/asp_config.h], [ASP_])

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
LIBTOOL_VERSION=1:0:0
AC_SUBST(LIBTOOL_VERSION)

# load the config.options
if test -f ./config.options; then
  AX_LOG([using config.options << EOF])
  AX_LOG([`cat ./config.options`])
  AX_LOG([EOF])
  source ./config.options
fi



##################################################
# Find the necessary programs
##################################################

# Make sure these are set so AC_PROG_CC doesn't initialize them with
# its defaults, since we take care of setting debug and optimization
# levels ourselves.
CFLAGS="$CFLAGS"
CXXFLAGS="$CXXFLAGS"

# Find the tools
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AX_PROG_AR
AC_PROG_LIBTOOL
AC_PROG_MAKE_SET
AC_LANG(C++)
AX_COMMON_OPTIONS




##################################################
# Compilation options
##################################################

AX_ARG_ENABLE(strict-aliasing, no, [none],           [allow strict aliasing (dangerous)])

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

AX_PKG_BOOST
if test x"$HAVE_PKG_BOOST" == "xyes"; then
  AX_PKG_BOOST_CHECK_VERSION([ASP_])
  AC_MSG_NOTICE([Stereo will be built using Boost version $BOOST_VERSION])

  AX_PKG_BOOST_LIB(PROGRAM_OPTIONS, [-lboost_program_options], [boost/program_options.hpp])
  AX_PKG_BOOST_LIB(SYSTEM,          [-lboost_system],          [boost/system/error_code.hpp])
  AX_PKG_BOOST_LIB(FILESYSTEM,      [-lboost_filesystem],      [boost/filesystem/path.hpp], [$PKG_BOOST_SYSTEM_LIBS])
  AX_PKG_BOOST_LIB(THREAD,          [-lboost_thread],          [boost/thread/thread.hpp])
  AX_PKG_BOOST_LIB(IOSTREAMS,       [-lboost_iostreams],       [boost/iostreams/stream_buffer.hpp])
fi

# This provides an easy way for users to override the threads options
# if they are replacing Boost threads with something else for their platform.
AX_PKG(THREADS, [BOOST_THREAD], [], [])

AX_PKG_ONE_OF(LAPACK,
  APPLE_LAPACK,
    [AX_PKG_APPLE(LAPACK, [], [-framework vecLib])],
  CLAPACK,
    [AX_PKG(CLAPACK, [], [-lclapack -lblas -lf2c], [])],
  SLAPACK,
    [AX_PKG(SLAPACK, [], [-llapack -lblas], [])],
  FLAPACK,
    [AX_PKG(FLAPACK, [], [-llapack -lblas -lgfortran], [])],
  STANDALONE_LAPACK_BLAS,
    [AX_PKG(STANDALONE_BLAS, [], [-lblas], [])
     AX_PKG(STANDALONE_LAPACK, [], [-llapack], [])
     AX_GROUP_PKG(STANDALONE_LAPACK_AND_BLAS, [STANDALONE_LAPACK STANDALONE_BLAS])],
  STANDALONE_FLAPACK_FBLAS,
    [AX_PKG(STANDALONE_F2C, [], [-lf2c], [])
     AX_PKG(STANDALONE_FBLAS, [STANDALONE_F2C], [-lblas], [])
     AX_PKG(STANDALONE_FLAPACK, [STANDALONE_F2C], [-llapack], [])
     AX_GROUP_PKG(STANDALONE_FLAPACK_FBLAS, [STANDALONE_FLAPACK STANDALONE_FBLAS STANDALONE_F2C])])

AX_PKG(VW,                [BOOST BOOST_THREAD BOOST_PROGRAM_OPTIONS],  [-lvw],            [vw/Image/ImageView.h vw/Math/LinearAlgebra.h])
AX_PKG(VW_CAMERA,         [VW BOOST_FILESYSTEM BOOST_IOSTREAMS], [-lvwCamera],      [vw/Camera/CAHVModel.h])
AX_PKG(VW_STEREO,         [VW VW_CAMERA],        [-lvwStereo],      [vw/Stereo/OptimizedCorrelator.h])
AX_PKG(VW_CARTOGRAPHY,    [VW VW_CAMERA],        [-lvwCartography], [vw/Cartography/GeoReference.h])
AX_PKG(VW_INTEREST_POINT, [VW],                  [-lvwInterestPoint])

AX_PKG(CUDA,              [],      [-lcuda -lcudart], [cuda.h])

AX_PKG_ONE_OF(GL,
  APPLE_GL,
    [AX_PKG_APPLE([GL], [], [-framework OpenGL -framework GLUT -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib])],
  LINUX_GL,
    [AX_PKG(X11, [], [-lXext -lX11 -lXmu -lXi -lXt], [])
     AX_PKG(LINUX_GL, [X11], [-lGL -lGLU -lglut], [GL/gl.h GL/glu.h GL/glut.h], [])])

AX_PKG(OPENSCENEGRAPH, [GL], [-losg -losgDB -lOpenThreads -losgViewer -losgText -losgGA -losgFX -losgTerrain -losgUtil], [osg/ref_ptr])

AX_PKG_ONE_OF(MBA,
    MBA11, [AX_PKG(MBA11, [BOOST], [-lMBA-1.1],         [MBA.h])],
    MBA10, [AX_PKG(MBA10, [BOOST], [-lMBA_1.0_gpl_opt], [MBA.h])])

AX_PKG(SPICE, [], [-lcspice], [SpiceUsr.h SpiceZfc.h])

# Qt with AutoTroll.
AT_WITH_QT([+gui +sql +opengl], [], [], [HAVE_PKG_QT_BASE="no:missing QT"], [])

if test x"$HAVE_PKG_QT_BASE" != x"no:missing QT"; then
    AT_REQUIRE_QT_VERSION([4.0], [HAVE_PKG_QT_BASE="no:wrong QT version"], [])
fi

# We seed QT_BASE's cppflags and libs with the ones qmake figured out
PKG_QT_BASE_CPPFLAGS="$PKG_QT_BASE_CPPFLAGS $QT_CPPFLAGS"
AX_PKG(QT_BASE, [], [$QT_LIBS], [QVector])

AX_PKG_ONE_OF(QT,
    APPLE_QT, [AX_PKG_APPLE(QT, [GL QT_BASE], [])],
    LINUX_QT,
     [AX_PKG(QT_OPENGL,  [QT_BASE], [], [QtOpenGL])
      AX_PKG(QT_GUI,     [QT_BASE], [], [QLabel])
      AX_PKG(QT_SQL,     [QT_BASE], [], [QtSql])
      AX_GROUP_PKG(LINUX_QT, [QT_BASE QT_GUI QT_OPENGL QT_SQL])])

AX_PKG_ONE_OF(QWT,
    APPLE_QWT, [AX_PKG_APPLE(QWT, [QT_BASE], [-lqwt.5])],
    PLAIN_QWT, [AX_PKG(PLAIN_QWT, [QT_BASE], [-lqwt])],
    SUFFX_QWT, [AX_PKG(SUFFX_QWT, [QT_BASE], [-lqwt-qt4])])

AX_PKG_ONE_OF(GSL,
    GSL_HASBLAS,  [AX_PKG(GSL_HASBLAS,  [LAPACK], [-lgsl])],
    GSL_ASNEEDED, [AX_PKG(GSL_ASNEEDED, [LAPACK], [-Wl,-no-as-needed,-lgsl,-lgslcblas,-as-needed])],
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
AX_PKG(ISIS, [QT ISIS3RDPARTY], [-lisis3], [SpecialPixel.h])

# These are here (instead of inside the PKG macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(HAVE_PKG_SPICE, [test "$HAVE_PKG_SPICE" = "yes"])
AM_CONDITIONAL(HAVE_PKG_ISIS, [test "$HAVE_PKG_ISIS" = "yes"])

##################################################
# module definitions
##################################################

AX_GROUP_PKG(BOOST_COMMON, [BOOST BOOST_PROGRAM_OPTIONS])
AX_GROUP_PKG(BOOST_ALL,    [BOOST_COMMON BOOST_FILESYSTEM])
AX_GROUP_PKG(VW_ALL,       [VW VW_CAMERA VW_CARTOGRAPHY VW_STEREO VW_INTEREST_POINT])

AX_APP(STEREO,         [src], yes, [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(STEREOGUI,      [src], no,  [BOOST_ALL VW_ALL LAPACK MBA QT],  [ISIS SPICE])
AX_APP(ORTHOPROJECT,   [src], yes, [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(BUNDLEADJUST,   [src], no,  [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(ORBITVIZ,       [src], no,  [BOOST_ALL VW_ALL LAPACK MBA],     [ISIS SPICE])
AX_APP(DISPARITYDEBUG, [src], yes, [BOOST_COMMON VW VW_STEREO])
AX_APP(POINT2MESH,     [src], yes, [BOOST_COMMON VW OPENSCENEGRAPH])
AX_APP(POINT2MESH2,    [src], yes, [BOOST_COMMON VW OPENSCENEGRAPH])
AX_APP(POINT2DEM,      [src], yes, [BOOST_COMMON VW VW_CARTOGRAPHY])
AX_APP(NURBS,          [src], no,  [BOOST_COMMON VW VW_STEREO MBA])
AX_APP(CTXIMAGE,       [src], no,  [BOOST_COMMON VW VW_CARTOGRAPHY])
AX_APP(RMAX2CAHVOR,    [src], no,  [BOOST_COMMON VW VW_CAMERA BOOST_FILESYSTEM])
AX_APP(RMAXADJUST,     [src], no,  [BOOST_ALL VW_ALL LAPACK MBA])
AX_APP(BUNDLEVIS,      [src], no,  [BOOST_COMMON OPENSCENEGRAPH VW VW_CAMERA])
AX_APP(ISISADJUST,     [src], yes, [BOOST_ALL VW_ALL LAPACK MBA ISIS SPICE])
AX_APP(RESULTS,        [src], yes, [BOOST_ALL VW_ALL ISIS])
AX_APP(CUDATEST,       [src], no,  [VW CUDA])
AX_APP(RECONSTRUCT,    [src], no,  [BOOST_ALL VW VW_CARTOGRAPHY])
AX_APP(BATEST,         [src], no,  [BOOST_ALL VW_ALL LAPACK MBA])
AX_APP(CNETEDITOR,     [src], no,  [BOOST_ALL VW VW_CAMERA])

# These are here (instead of inside the APP macro where they belong)
# for backwards compatability with older versions of automake.
AM_CONDITIONAL(MAKE_APP_STEREO, [test "$MAKE_APP_STEREO" = "yes"])
AM_CONDITIONAL(MAKE_APP_STEREOGUI, [test "$MAKE_APP_STEREOGUI" = "yes"])
AM_CONDITIONAL(MAKE_APP_ORTHOPROJECT, [test "$MAKE_APP_ORTHOPROJECT" = "yes"])
AM_CONDITIONAL(MAKE_APP_BUNDLEADJUST, [test "$MAKE_APP_BUNDLEADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_ORBITVIZ, [test "$MAKE_APP_ORBITVIZ" = "yes"])
AM_CONDITIONAL(MAKE_APP_DISPARITYDEBUG, [test "$MAKE_APP_DISPARITYDEBUG" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2MESH, [test "$MAKE_APP_POINT2MESH" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2MESH2, [test "$MAKE_APP_POINT2MESH2" = "yes"])
AM_CONDITIONAL(MAKE_APP_POINT2DEM, [test "$MAKE_APP_POINT2DEM" = "yes"])
AM_CONDITIONAL(MAKE_APP_NURBS, [test "$MAKE_APP_NURBS" = "yes"])
AM_CONDITIONAL(MAKE_APP_CTXIMAGE, [test "$MAKE_APP_CTXIMAGE" = "yes"])
AM_CONDITIONAL(MAKE_APP_RMAX2CAHVOR, [test "$MAKE_APP_RMAX2CAHVOR" = "yes"])
AM_CONDITIONAL(MAKE_APP_RMAXADJUST, [test "$MAKE_APP_RMAXADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_BUNDLEVIS, [test "$MAKE_APP_BUNDLEVIS" = "yes"])
AM_CONDITIONAL(MAKE_APP_ISISADJUST, [test "$MAKE_APP_ISISADJUST" = "yes"])
AM_CONDITIONAL(MAKE_APP_RESULTS, [test "$MAKE_APP_RESULTS" = "yes"])
AM_CONDITIONAL(MAKE_APP_CUDATEST, [test "$MAKE_APP_CUDATEST" = "yes"])
AM_CONDITIONAL(MAKE_APP_RECONSTRUCT, [test "$MAKE_APP_RECONSTRUCT" = "yes"])
AM_CONDITIONAL(MAKE_APP_BATEST, [test "$MAKE_APP_BATEST" = "yes"])
AM_CONDITIONAL(MAKE_APP_CNETEDITOR, [test "$MAKE_APP_CNETEDITOR" = "yes"])


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
  src/tests/Makefile                    \
])

AC_OUTPUT
