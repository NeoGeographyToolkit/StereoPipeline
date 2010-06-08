// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>

#include <asp/IsisIO/IsisAdjustCameraModel.h>

using namespace vw;
using namespace vw::camera;
using namespace asp;

Vector2 generate_random( int const& xsize,
                         int const& ysize ) {
  Vector2 pixel;
  pixel[0] = rand() % ( 10 * xsize - 10 ) + 10;
  pixel[0] /= 10;
  pixel[1] = rand() % ( 10 * ysize - 10 ) + 10;
  pixel[1] /= 10;
  return pixel;
}

TEST(IsisAdjustCameraModel, NoFunctions) {
  std::vector<std::string> files;
  files.push_back("E1701676.reduce.cub");
  files.push_back("5165r.cub");
  srand( time(NULL) );
  for ( uint j = 0; j < files.size(); j++ ) {
    boost::shared_ptr<BaseEquation> blank( new PolyEquation(0) );
    IsisAdjustCameraModel cam( files[j], blank, blank );

    std::cout << "File: " << files[j] << "\n";
    std::cout << "-------------------------------------\n";

    for ( uint i = 0; i < 100; i++ ) {
      Vector2 pixel = generate_random( cam.samples(),
                                       cam.lines() );
      Vector3 point = cam.pixel_to_vector( pixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise to make sure we are not using stored values
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.pixel_to_vector( noise );
      }
      point *= 100000; // 100 km below
      point += cam.camera_center( pixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise to make sure we are not using stored values
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.camera_center( noise );
      }

      Vector2 rpixel = cam.point_to_pixel( point );
      EXPECT_NEAR( pixel[0], rpixel[0], 0.001 );
      EXPECT_NEAR( pixel[1], rpixel[1], 0.001 );
    }
  }
}

TEST(IsisAdjustCameraModel, PolyFunctions) {
  std::vector<std::string> files;
  files.push_back("E1701676.reduce.cub");
  files.push_back("5165r.cub");
  srand( time(NULL) );
  for ( uint j = 0; j < files.size(); j++ ) {
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
    IsisAdjustCameraModel cam( files[j], position, pose );
    std::cout << "File: " << files[j] << "\n";
    std::cout << "---------------------------------------\n";

    for ( uint i = 0; i < 100; i++ ) {
      Vector2 pixel = generate_random( cam.samples(),
                                       cam.lines() );
      Vector3 point = cam.pixel_to_vector( pixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.pixel_to_vector( noise );
      }
      point *= 100000; // 100 km
      point += cam.camera_center( pixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.camera_center( noise );
      }
      Vector2 rpixel = cam.point_to_pixel( point );
      EXPECT_NEAR( pixel[0], rpixel[0], 0.001 );
      EXPECT_NEAR( pixel[1], rpixel[1], 0.001 );
      (*position)[2] += 1000;
      Vector3 rpoint = cam.pixel_to_vector( rpixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.pixel_to_vector( noise );
      }
      rpoint *= 100000; // 100 km
      rpoint += cam.camera_center( rpixel );
      EXPECT_NEAR( rpoint[0] - point[0], 0, .001 );
      EXPECT_NEAR( rpoint[1] - point[1], 1000, .001 );
      EXPECT_NEAR( rpoint[2] - point[2], 0, .001 );
    }
  }
}

TEST(IsisAdjustCameraModel, RPNFunctions) {
  std::vector<std::string> files;
  files.push_back("E1701676.reduce.cub");
  files.push_back("5165r.cub");
  srand( time(NULL) );
  for ( uint j = 0; j < files.size(); j++ ) {
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
    IsisAdjustCameraModel cam( files[j], position, pose );
    std::cout << "File: " << files[j] << "\n";
    std::cout << "---------------------------------------\n";
    std::cout << "Position ET offset: " << position->get_time_offset() << "\n";
    std::cout << "Pose ET offset: " << pose->get_time_offset() << "\n";

    for ( uint i = 0; i < 2; i++ ) {
      Vector2 pixel = generate_random( cam.samples(),
                                       cam.lines() );
      Vector3 point = cam.pixel_to_vector( pixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.pixel_to_vector( noise );
      }
      point *= 80000; // 80 km
      point += cam.camera_center( pixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.camera_center( noise );
      }
      Vector2 rpixel = cam.point_to_pixel( point );
      EXPECT_NEAR( pixel[0], rpixel[0], 0.001 );
      EXPECT_NEAR( pixel[1], rpixel[1], 0.001 );
      (*position)[2] -= 500;
      Vector3 rpoint = cam.pixel_to_vector( rpixel );
      for ( uint k = 0; k < 5; k++ ) {
        // Apply noise
        Vector2 noise = generate_random( cam.samples(),
                                         cam.lines() );
        Vector3 temp = cam.pixel_to_vector( noise );
      }
      rpoint *= 80000; // 80 km
      rpoint += cam.camera_center( rpixel );
      EXPECT_NEAR( rpoint[0] - point[0], -500, .001 );
      EXPECT_NEAR( rpoint[1] - point[1], 0, .001 );
      EXPECT_NEAR( rpoint[2] - point[2], 0, .001 );
    }
  }
}
