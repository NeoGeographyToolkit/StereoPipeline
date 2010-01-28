// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <gtest/gtest.h>

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

TEST(IsisCameraModel, linescan_groundmap_chk) {
  // Run two methods ..
  // solve for vector out with and without the groundmap solution
  // prove that they get the same values
  Isis::Cube* cube_ptr = new Isis::Cube;
  cube_ptr->Open("E1701676.crop.cub");
  Isis::Camera* cam = cube_ptr->Camera();

  // Ripping out the parts of the Camera Model
  Isis::CameraDistortionMap* distortmap = cam->DistortionMap();
  Isis::CameraFocalPlaneMap* focalmap = cam->FocalPlaneMap();
  Isis::CameraDetectorMap* detectmap = cam->DetectorMap();

  // Building test set
  std::vector<Vector2> pixel_sets;
  pixel_sets.push_back(Vector2(10,2000));
  pixel_sets.push_back(Vector2(50,10));
  pixel_sets.push_back(Vector2(100,900));
  pixel_sets.push_back(Vector2(300,717));
  pixel_sets.push_back(Vector2(500,42));
  pixel_sets.push_back(Vector2(225,991));
  pixel_sets.push_back(Vector2(10,100));
  pixel_sets.push_back(Vector2(900,10));
  pixel_sets.push_back(Vector2(199,880));
  pixel_sets.push_back(Vector2(360,217));
  pixel_sets.push_back(Vector2(800,1742));
  pixel_sets.push_back(Vector2(677,1691));

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
    EXPECT_NEAR( nog_solution_sets[i][0], g_solution_sets[i][0], 1e-4 );
    EXPECT_NEAR( nog_solution_sets[i][1], g_solution_sets[i][1], 1e-4 );
    EXPECT_NEAR( nog_solution_sets[i][2], g_solution_sets[i][2], 1e-4 );
  }

  cube_ptr->Close();
}

TEST(IsisCameraModel, frame_groundmap_chk) {
  // Run two methods ..
  // solve for vector out with and without the groundmap solution
  // prove that they get the same values

}
