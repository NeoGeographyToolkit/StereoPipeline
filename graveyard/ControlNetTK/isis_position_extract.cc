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


#include <vw/Core.h>
#include <vw/Math/Vector.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>

// This executable is not meant to be used by people directly. This
// is a helper utility for pairlist_degree.py

using namespace vw;

int main( int argc, char* argv[] ) {

  Vector2i image_size;
  boost::shared_ptr<camera::CameraModel> camera_model;
  if ( argc == 2 ) {
    // ISIS Camera
    std::string input_cube( argv[1] );
    camera_model = boost::shared_ptr<camera::CameraModel>(new camera::IsisCameraModel( input_cube ));
    image_size[0] = boost::dynamic_pointer_cast<camera::IsisCameraModel>(camera_model)->samples();
    image_size[1] = boost::dynamic_pointer_cast<camera::IsisCameraModel>(camera_model)->lines();
  } else if ( argc == 3 ) {
    // Adjusted ISIS Camera
    std::string input_cube( argv[1] );
    std::string adjust_file;
    if ( boost::ends_with(input_cube, "isis_adjust") ) {
      adjust_file = input_cube;
      input_cube = std::string( argv[2] );
    } else {
      adjust_file = std::string( argv[2] );
    }
    std::ifstream adjust( adjust_file.c_str() );
    boost::shared_ptr<asp::BaseEquation> positionF = asp::read_equation(adjust);
    boost::shared_ptr<asp::BaseEquation> poseF = asp::read_equation(adjust);
    adjust.close();

    camera_model = boost::shared_ptr<camera::CameraModel>(new camera::IsisAdjustCameraModel( input_cube, positionF, poseF ));
    image_size[0] = boost::dynamic_pointer_cast<camera::IsisAdjustCameraModel>(camera_model)->samples();
    image_size[1] = boost::dynamic_pointer_cast<camera::IsisAdjustCameraModel>(camera_model)->lines();
  } else
    return 1;

  // Figure out the average position of the camera by running 5 locations.
  // This is only important for linescan cameras.
  Vector3 camera_position;
  camera_position += camera_model->camera_center(Vector2())/5;
  camera_position += camera_model->camera_center(image_size-Vector2(1,1))/5;
  camera_position += camera_model->camera_center(Vector2(0,image_size[1]-1))/5;
  camera_position += camera_model->camera_center(Vector2(image_size[0]-1,0))/5;
  camera_position += camera_model->camera_center(image_size/2)/5;

  std::cout << std::setprecision(15) << camera_position[0] << " "
            << camera_position[1] << " " << camera_position[2] << "\n";
  return 0;
}
