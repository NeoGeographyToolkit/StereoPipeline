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
  /*
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
  */

  Vector3   gcc(-2470899.9105873918, 5542443.1851118281, 2502370.2339997198);
  Matrix3x3 rot_mat(-0.057502980758053046, -0.74594045857037883,  0.66352561327483939,
                    -0.16417103320903326,  -0.64851223071215924, -0.74328982131589649,
                     0.98475442576259453,  -0.15167306578477252, -0.085170429471915887);
  vw::Quat q(rot_mat);
  Vector3  initial_position(gcc);
  Vector3  initial_orientation = q.axis_angle();
  
  double   pixel_size =  0.000112;
  Vector2i image_size(3910, 2290);
  Vector2  center_loc_pixels = Vector2(1957.2358579744291, 1112.512227705716);
  double   focal_length       = 1.9600000381469727;
  double   scan_angle_radians = 1.2915399999999999;
  double   scan_rate_radians  = 3.0125446909089169;
  bool     scan_left_to_right = true;
  double   forward_tilt_radians = 0;
  double   velocity = 5253.0854975797192;
  bool     use_motion_comp = true;


  OpticalBarModel* raw_ptr = new OpticalBarModel(image_size, center_loc_pixels, pixel_size,
                                                 focal_length, scan_angle_radians, scan_rate_radians, scan_left_to_right,
                                                 forward_tilt_radians,
                                                 initial_position, initial_orientation,
                                                 velocity, use_motion_comp);

  //OpticalBarModel* raw_ptr = new OpticalBarModel("/home/smcmich1/data/KH7/left16.tsai");
  //OpticalBarModel* raw_ptr = new OpticalBarModel("/home/smcmich1/data/KH4B/bundle_v2_2/out-v2_opt_bar.tsai");
  //OpticalBarModel* raw_ptr = new OpticalBarModel("/home/smcmich1/data/KH4B/v2_test.tsai");


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
  for ( size_t i = 0; i < 3000; i += 200 ) {
    for ( size_t j = 0; j < 2400; j += 200 ) {
      Vector2 pixel(i,j);
      Vector3 center = cam1->camera_center(pixel);
      EXPECT_VECTOR_NEAR( pixel,
                          cam1->point_to_pixel( center + 2e4*cam1->pixel_to_vector(pixel) ),
                          1e-2 /*pixels*/);
    }
  }

  
  /*
  Vector3   gcc2(-2470746.042265798, 5537165.6573024355, 2515786.8430585163);
  Matrix3x3 rot_mat2(-0.029523776962128767, -0.72544994731979884,  0.68764141856609462,
                     -0.18849289287502574,  -0.67155812862389763, -0.71657526416613915,
                      0.98163067185652575,  -0.15077152852202924, -0.11691522679974731);
  vw::Quat q2(rot_mat);
  Vector3  initial_position2(gcc2);
  Vector3  initial_orientation2 = q2.axis_angle();
  
  double   pixel_size2 =  0.000112;
  Vector2i image_size2(3958, 2289);
  center_loc_pixels = Vector2(1981.2633058472611, 1112.0264145058445);
  focal_length       = 1.9600000381469727;
  scan_angle_radians = 1.2915399999999999;
  scan_rate_radians  = 3.0125446909089169;
  scan_left_to_right = true;
  forward_tilt_radians = 0;
  velocity = 5253.0854975797192;
  use_motion_comp = true;

  OpticalBarModel* raw_ptr2 = new OpticalBarModel(image_size2, center_loc_pixels, pixel_size2,
                                                 focal_length, scan_angle_radians, scan_rate_radians, scan_left_to_right,
                                                 forward_tilt_radians,
                                                 initial_position2, initial_orientation2,
                                                 velocity, use_motion_comp);
  
  //OpticalBarModel* raw_ptr2 = new OpticalBarModel("/home/smcmich1/data/KH7/right16.tsai");
  //OpticalBarModel* raw_ptr2 = new OpticalBarModel("/home/smcmich1/data/KH4B/bundle_v6_2/out-v6_opt_bar.tsai");
  OpticalBarModel* raw_ptr2 = new OpticalBarModel("/home/smcmich1/data/KH4B/v6_test.tsai");

  CameraModelPtr cam2;
  cam2 = CameraModelPtr(raw_ptr2);
  
  
  bool least_squares_refine = false;
  double angle_tol = 0.0;
  vw::stereo::StereoModel model(cam1.get(), cam2.get(), least_squares_refine, angle_tol);

  // KH4B matching pixels
  std::vector<Vector2> pixels1, pixels2;
  pixels1.push_back(Vector2(213.098403931, 52.2872962952)); // Top Left
  pixels2.push_back(Vector2(73.009803772, 20.6448688507));
  
  pixels1.push_back(Vector2(13478.8212891, 105.892700195)); // Top Right
  pixels2.push_back(Vector2(13581.4511719, 94.3644943237));
  
  pixels1.push_back(Vector2(292.00491333, 919.954101562)); // Bottom Left
  pixels2.push_back(Vector2(418.534362793, 881.189758301));

  pixels1.push_back(Vector2(13466.0078125, 931.140014648)); // Bottom Right
  pixels2.push_back(Vector2(13294.6064453, 910.764892578));
  
  //pixels1.push_back(Vector2());
  //pixels2.push_back(Vector2());
  
  */
  
  
  /* // KH7 matching pixels
  Vector2 pix1(851.479797363, 1053.04174805); // From bundle adjust
  Vector2 pix2(343.897613525, 1501.32043457);
  //// Very roughly, 113.382928,  22.939696
  
  //Vector2 pix1(643, 1342); // Manually selected IP on river feature.
  //Vector2 pix2(252, 1797);
  // Close to 113.413161, 22.920289
  
  //Vector2 pix1(3639.6,167.803); // From stereo
  //Vector2 pix2(2815.35,614.75);
  */
  /*
  double error = 0;
  for (size_t i=0; i<pixels1.size(); ++i) {
    
    Vector3 point = model(pixels1[i], pixels2[i], error);

    Vector3 gdc = d.cartesian_to_geodetic(point);
    
    std::cout << "Point = " << point << std::endl;
    std::cout << "GDC = " << gdc << std::endl;
    std::cout << "error = " << error << std::endl;
  }
  */
  //std::cout << "cam1 = " << *raw_ptr << std::endl;
  //std::cout << "cam2 = " << *raw_ptr2 << std::endl;
  
  //EXPECT_EQ(error, -999);
  
}

