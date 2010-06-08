// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <asp/PhotometryTK/ProjectFileIO.h>
#include <list>

using namespace vw;
using namespace asp;
using namespace asp::pho;

TEST(ProjectFileIO, circle_test) {

  // Write test data
  ProjectMeta proj_st;
  proj_st.set_name("monkey");
  proj_st.set_reflectance( ProjectMeta::LUNARL_GASKALL );
  proj_st.set_num_cameras( 2 );

  std::list<CameraMeta> cam_st;
  cam_st.push_back( CameraMeta() );
  cam_st.push_back( CameraMeta() );
  cam_st.front().set_name( "front" );
  cam_st.back().set_name( "back" );
  cam_st.front().set_exposure_t( 2 );
  cam_st.back().set_exposure_t( 4 );
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
  }
}
