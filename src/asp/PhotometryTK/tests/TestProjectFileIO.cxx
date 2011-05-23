// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <asp/PhotometryTK/ProjectFileIO.h>
#include <list>
#include <boost/foreach.hpp>

using namespace vw;
using namespace asp;
using namespace asp::pho;

namespace fs = boost::filesystem;

TEST(ProjectFileIO, circle_test) {

  // Write test data
  ProjectMeta proj_st;
  proj_st.set_name("monkey");
  proj_st.set_reflectance( ProjectMeta::LUNARL_GASKALL );
  proj_st.set_num_cameras( 256 );
  proj_st.set_min_pixval( 1 );
  proj_st.set_max_pixval( 2 );

  std::list<CameraMeta> cam_st;
  for ( size_t i = 0; i < 256; i++ ) {
    cam_st.push_back( CameraMeta() );
    cam_st.back().set_name( "monkey" );
    cam_st.back().set_init_error( 0.0 );
    cam_st.back().set_last_error( 0.0 );
    cam_st.back().set_curr_error( 0.0 );
  }
  cam_st.front().set_name( "front" );
  cam_st.back().set_name( "back" );
  size_t i = 0;
  BOOST_FOREACH( CameraMeta& cam, cam_st ) {
    cam.set_exposure_t( i );
    i++;
  }
  write_pho_project( "test.ptk", proj_st,
                     cam_st.begin(), cam_st.end() );

  // Read test data
  {
    ProjectMeta proj_end;
    std::list<CameraMeta> cam_end;
    read_pho_project( "test.ptk", proj_end,
                      cam_end );

    EXPECT_EQ( proj_st.name(), proj_end.name() );
    EXPECT_EQ( proj_st.reflectance(),
               proj_end.reflectance() );
    EXPECT_EQ( proj_st.num_cameras(),
               proj_end.num_cameras() );
    EXPECT_EQ( proj_st.num_cameras(),
               cam_st.size() );
    EXPECT_EQ( proj_end.num_cameras(),
               cam_end.size() );
    EXPECT_EQ( cam_st.front().name(),
               cam_end.front().name() );
    EXPECT_EQ( cam_st.back().name(),
               cam_end.back().name() );
    size_t i = 0;
    BOOST_FOREACH( CameraMeta const& cam, cam_end ) {
      EXPECT_NEAR( double(i), cam.exposure_t(), 1e-6 );
      i++;
    }
    EXPECT_EQ( proj_st.min_pixval(),
	       proj_end.min_pixval() );
    EXPECT_EQ( proj_st.max_pixval(),
	       proj_end.max_pixval() );
  }

  fs::remove_all("test.ptk");
}
