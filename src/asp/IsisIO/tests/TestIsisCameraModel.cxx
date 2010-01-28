// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <gtest/gtest.h>

#include <time.h>

#include <vw/Math/Vector.h>
#include <vw/Core/Debugging.h>
#include <asp/IsisIO/IsisCameraModel.h>

// Additional Headers required for ISIS
#include <Cube.h>
#include <Camera.h>
#include <CameraFocalPlaneMap.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>

using namespace vw;
using namespace vw::camera;

double DELTA = 1e-8;

TEST(IsisCameraModel, groundmap_chk) {
  // Run two methods ..
  // solve for vector out with and without the groundmap solution
  // prove that they get the same values

  std::vector<std::string> files;
  files.push_back("E1701676.crop.cub");
  files.push_back("5165r.cub");

  for ( uint j = 0; j < files.size(); j++ ) {
    Isis::Cube* cube_ptr = new Isis::Cube;
    cube_ptr->Open( files[j].c_str() );
    Isis::Camera* cam = cube_ptr->Camera();

    // Ripping out the parts of the Camera Model
    Isis::CameraDistortionMap* distortmap = cam->DistortionMap();
    Isis::CameraFocalPlaneMap* focalmap = cam->FocalPlaneMap();
    Isis::CameraDetectorMap* detectmap = cam->DetectorMap();

    // Building test set
    std::vector<Vector2> pixel_sets;
    srand( time(NULL) );
    for ( uint i = 0; i < 1000; i++ ) {
      Vector2 pixel;
      pixel[0] = rand() % ( 10 * cube_ptr->Samples() ) + 10;
      pixel[0] /= 10;
      pixel[1] = rand() % ( 10 * cube_ptr->Lines() ) + 10;
      pixel[1] /= 10;
      pixel_sets.push_back(pixel);
    }

    std::vector<Vector3> g_solution_sets, nog_solution_sets;

    Timer *t = new Timer("No GroundMap Solution");

    for ( uint i = 0; i < pixel_sets.size(); i++ ) {
      Vector2 pixel = pixel_sets[i];
      Vector3 nog_solution;

      // No Ground Map Solution
      detectmap->SetParent( pixel[0], pixel[1] );
      focalmap->SetDetector( detectmap->DetectorSample(),
                             detectmap->DetectorLine() );
      distortmap->SetFocalPlane( focalmap->FocalPlaneX(),
                                 focalmap->FocalPlaneY() );
      nog_solution[0] = distortmap->UndistortedFocalPlaneX();
      nog_solution[1] = distortmap->UndistortedFocalPlaneY();
      nog_solution[2] = distortmap->UndistortedFocalPlaneZ();
      nog_solution /= norm_2(nog_solution);
      std::vector<double> lookC(3); // Should make fancy func for std vec and vec
      lookC[0] = nog_solution[0];
      lookC[1] = nog_solution[1];
      lookC[2] = nog_solution[2];
      std::vector<double> lookJ = cam->InstrumentRotation()->J2000Vector(lookC);
      lookC = cam->BodyRotation()->ReferenceVector(lookJ);
      nog_solution[0] = lookC[0];
      nog_solution[1] = lookC[1];
      nog_solution[2] = lookC[2];

      nog_solution_sets.push_back(nog_solution);
    }

    delete(t);
    t = new Timer("Ground Map Solution:");

    for ( uint i = 0; i < pixel_sets.size(); i++ ) {

      Vector2 pixel = pixel_sets[i];
      Vector3 g_solution;

      // Ground Map Solution
      cam->SetImage(pixel[0],pixel[1]);
      double p[3];
      cam->InstrumentPosition(p);
      Vector3 instrument( p[0], p[1], p[2] );

      cam->Coordinate(p);
      Vector3 ground( p[0], p[1], p[2] );
      g_solution = normalize( ground - instrument );

      g_solution_sets.push_back(g_solution);
    }

    delete t;

    for ( uint i = 0; i < pixel_sets.size(); i++ ) {
      EXPECT_NEAR( nog_solution_sets[i][0], g_solution_sets[i][0], DELTA );
      EXPECT_NEAR( nog_solution_sets[i][1], g_solution_sets[i][1], DELTA );
      EXPECT_NEAR( nog_solution_sets[i][2], g_solution_sets[i][2], DELTA );
    }

    cube_ptr->Close();
  }
}

TEST(IsisCameraModel, camera_model) {
  // Circle Check
  std::vector<std::string> files;
  files.push_back("E1701676.crop.cub");
  files.push_back("5165r.cub");
  srand( time(NULL) );
  for ( uint j = 0; j < files.size(); j++ ) {

    IsisCameraModel cam(files[j]);

    for ( uint i = 0; i < 500; i++ ) {
      Vector2 pixel;
      pixel[0] = rand() % ( 10 * cam.getSamples() ) + 10;
      pixel[0] /= 10;
      pixel[1] = rand() % ( 10 * cam.getLines() ) + 10;
      pixel[1] /= 10;

      Vector3 point = cam.pixel_to_vector( pixel );
      point *= 200;
      point += cam.camera_center( pixel );

      Vector2 rpixel = cam.point_to_pixel( point );
      EXPECT_NEAR( pixel[0], rpixel[0], 0.001 );
      EXPECT_NEAR( pixel[1], rpixel[1], 0.001 );
    }

  }
}
