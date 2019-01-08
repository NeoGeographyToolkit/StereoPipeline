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


#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography/Datum.h>
#include <asp/Camera/OpticalBarModel.h>

#include <boost/scoped_ptr.hpp>
#include <test/Helpers.h>



using namespace vw;
using namespace asp;
using namespace asp::camera;
using namespace vw::test;
using namespace vw::camera;


TEST(OpticalBarModel, CreateCamera) {

  vw::cartography::Datum d("WGS84");
  //Vector3 llh(-122.065046, 37.419733, 250000);
  Vector3 llh(0.0, 90.0, 250000);
  Vector3 angles(0,0,0); // Looking down at the north pole
  
  Vector3 gcc = d.geodetic_to_cartesian(llh);

  double   pixel_size         = 7*10e-6;
  Vector2i image_size(12000, 4000);
  Vector2  center_loc_pixels = Vector2(-4.3, -1.2) + image_size/2;
  double   focal_length       = 609.602/1000.0;
  double   scan_angle_radians = 70  * M_PI/180;
  double   scan_rate_radians  = 192 * M_PI/180;
  bool     scan_left_to_right = true;
  double   forward_tilt_radians = 0;
  Vector3  initial_position(gcc);
  Vector3  initial_orientation(angles);
  double   velocity = 7800;
  bool     use_motion_comp = true;

  OpticalBarModel* raw_ptr = new OpticalBarModel(image_size, center_loc_pixels, pixel_size,
                                                 focal_length, scan_angle_radians, scan_rate_radians, scan_left_to_right,
                                                 forward_tilt_radians,
                                                 initial_position, initial_orientation,
                                                 velocity, use_motion_comp);

  // Basic file I/O
  OpticalBarModel cpy;
  EXPECT_NO_THROW(raw_ptr->write("test.tsai"));
  EXPECT_NO_THROW(cpy.read ("test.tsai"));

  EXPECT_VECTOR_EQ(raw_ptr->get_image_size(), cpy.get_image_size());

  typedef boost::shared_ptr<vw::camera::CameraModel> CameraModelPtr;
  CameraModelPtr cam1;
  cam1 = CameraModelPtr(raw_ptr);

  ASSERT_TRUE( cam1.get() != 0 );


  // A more accurate test is just to project out and back into the same camera
  for ( size_t i = 0; i < 3000; i += 500 ) {
    for ( size_t j = 0; j < 2400; j += 500 ) {
      Vector2 pixel(i,j);
      Vector3 center = cam1->camera_center(pixel);
      EXPECT_VECTOR_NEAR( pixel,
                          cam1->point_to_pixel( center + 2e4*cam1->pixel_to_vector(pixel) ),
                          1e-1 /*pixels*/);
    }
  }

}

