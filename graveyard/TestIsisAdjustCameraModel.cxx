// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#include <test/Helpers.h>

#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/PolyEquation.h>
#include <asp/IsisIO/RPNEquation.h>

#include <boost/foreach.hpp>

using namespace vw;
using namespace vw::camera;
using namespace asp;

class IsisAdjustCameraTest : public ::testing::Test {
protected:
  IsisAdjustCameraTest() {}

  virtual void SetUp() {
    files.push_back("E1701676.reduce.cub");
    files.push_back("5165r.cub");
  }

  Vector2 generate_random( int const& xsize,
                           int const& ysize ) {
    Vector2 pixel;
    pixel[0] = rand() % ( 10 * xsize - 10 ) + 10;
    pixel[0] /= 10;
    pixel[1] = rand() % ( 10 * ysize - 10 ) + 10;
    pixel[1] /= 10;
    return pixel;
  }

  void create_pixels( IsisAdjustCameraModel const& cam ) {
    pixels.clear();
    points.clear();
    for ( unsigned i = 0; i < 100; i++ ) {
      pixels.push_back( generate_random(cam.samples(),cam.lines()) );
      points.push_back( cam.pixel_to_vector(pixels.back()) );
      points.back() *= 100000; // 100 km
      points.back() += cam.camera_center(pixels.back());
    }
  }

  // Feed the camera noise to make sure we are not using stored
  // values.
  void fuzz_camera( IsisAdjustCameraModel const& cam ) {
    Vector2 noise = generate_random( cam.samples(),
                                     cam.lines() );
    Vector3 temp = cam.pixel_to_vector( noise );
  }

  std::vector<Vector2> pixels;
  std::vector<Vector3> points;
  std::vector<std::string> files;
};

TEST_F(IsisAdjustCameraTest, NoFunctions) {

  if (!asp::isis::IsisEnv()) {
    vw_out() << "ISISROOT or ISISDATA was not set. ISIS unit tests won't be run."
             << std::endl;
    return;
  }

  BOOST_FOREACH( std::string const& cube, files ) {
    boost::shared_ptr<BaseEquation> blank( new PolyEquation(0) );
    IsisAdjustCameraModel cam( cube, blank, blank );
    IsisCameraModel noa_cam( cube );
    create_pixels( cam );
    fuzz_camera( cam );

    for ( unsigned i = 0; i < pixels.size(); i++ ) {
      // Test Position hasn't changed
      fuzz_camera( cam );
      EXPECT_VECTOR_NEAR( cam.camera_center( pixels[i] ),
                          noa_cam.camera_center( pixels[i] ),
                          0.001 );
      EXPECT_MATRIX_NEAR( cam.camera_pose( pixels[i] ).rotation_matrix(),
                          noa_cam.camera_pose( pixels[i] ).rotation_matrix(),
                          0.001 );

      EXPECT_TRUE( cam.ephemeris_time(pixels[i]) );
      EXPECT_TRUE( cam.sun_position(pixels[i]) != Vector3() );
      EXPECT_EQ( cam.ephemeris_time(pixels[i]),
                 noa_cam.ephemeris_time(pixels[i]));
      EXPECT_VECTOR_NEAR( cam.sun_position(pixels[i]),
                          noa_cam.sun_position(pixels[i]), 1e-8 );

      // Test Circle Projection
      Vector2 rpixel = cam.point_to_pixel( points[i] );
      EXPECT_VECTOR_NEAR( pixels[i], rpixel, 0.001 );
    }

    // check enforcement that pose returns the rotation from camera
    // frame to world frame.
    Vector2 center_pixel( cam.lines(), cam.samples() );
    center_pixel /= 2;
    Quat center_pose = cam.camera_pose(center_pixel);
    double angle_from_z =
      acos(dot_prod(Vector3(0,0,1),inverse(center_pose).rotate(cam.pixel_to_vector(center_pixel))));
    EXPECT_LT( angle_from_z, 0.5 );
    center_pose = noa_cam.camera_pose(center_pixel);
    angle_from_z =
      acos(dot_prod(Vector3(0,0,1),inverse(center_pose).rotate(noa_cam.pixel_to_vector(center_pixel))));
    EXPECT_LT( angle_from_z, 0.5 );
  }
}

TEST_F(IsisAdjustCameraTest, PolyFunctions) {
  if (!asp::isis::IsisEnv()) {
    vw_out() << "ISISROOT or ISISDATA was not set. ISIS unit tests won't be run."
             << std::endl;
    return;
  }

  BOOST_FOREACH( std::string const& cube, files ) {
    boost::shared_ptr<BaseEquation> position( new PolyEquation(1) );
    (*position)[0] = 1000;
    (*position)[1] = 10;
    (*position)[2] = 2000;
    (*position)[3] = -10;
    (*position)[4] = -11000;
    (*position)[5] = 5;
    boost::shared_ptr<BaseEquation> pose( new PolyEquation(0) );
    (*position)[0] = .07;
    (*position)[1] = -.1;
    (*position)[2] = .02;
    IsisAdjustCameraModel cam( cube, position, pose );
    create_pixels( cam );
    fuzz_camera( cam );

    for ( unsigned i = 0; i < pixels.size(); i++ ) {
      Vector2 rpixel = cam.point_to_pixel( points[i] );
      EXPECT_VECTOR_NEAR( pixels[i], rpixel, 0.001 );
      (*position)[2] += 1000;
      Vector3 rpoint = cam.pixel_to_vector( rpixel );
      fuzz_camera(cam);
      rpoint *= 100000;
      rpoint += cam.camera_center( rpixel );
      EXPECT_NEAR( rpoint[0] - points[i][0], 0, .001 );
      EXPECT_NEAR( rpoint[1] - points[i][1], 1000, .001 );
      EXPECT_NEAR( rpoint[2] - points[i][2], 0, .001 );
      (*position)[2] -= 1000;
    }
  }
}

TEST_F(IsisAdjustCameraTest, RPNFunctions) {
  if (!asp::isis::IsisEnv()) {
    vw_out() << "ISISROOT or ISISDATA was not set. ISIS unit tests won't be run."
             << std::endl;
    return;
  }
  BOOST_FOREACH( std::string const& cube, files ) {
    std::string xpos_eq = "t 2 * 100 / 99 +";
    std::string ypos_eq = "t .8 * 1000 -";
    std::string zpos_eq = "t .5 * 2000 +";
    std::string xang_eq = ".005";
    std::string yang_eq = "-.013 t *";
    std::string zang_eq = "0";
    boost::shared_ptr<BaseEquation> position( new RPNEquation( xpos_eq,
                                                               ypos_eq,
                                                               zpos_eq ) );
    boost::shared_ptr<BaseEquation> pose( new RPNEquation( xang_eq,
                                                           yang_eq,
                                                           zang_eq ) );
    IsisAdjustCameraModel cam( cube, position, pose );
    create_pixels( cam );
    fuzz_camera( cam );

    for ( unsigned i = 0; i < pixels.size(); i++ ) {
      Vector2 rpixel = cam.point_to_pixel( points[i] );
      EXPECT_VECTOR_NEAR( pixels[i], rpixel, 0.001 );
      (*position)[2] -= 500;
      Vector3 rpoint = cam.pixel_to_vector( rpixel );
      fuzz_camera( cam );
      rpoint *= 100000;
      rpoint += cam.camera_center( rpixel );
      EXPECT_NEAR( rpoint[0] - points[i][0], -500, .001 );
      EXPECT_NEAR( rpoint[1] - points[i][1], 0, .001 );
      EXPECT_NEAR( rpoint[2] - points[i][2], 0, .001 );
      (*position)[2] += 500;
    }
  }
}
