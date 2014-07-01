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
#include <vw/Camera.h>

using namespace vw;

camera::CameraModel* load_camera( std::string file ) {
  if ( boost::ends_with(file, ".cahvore") ) {
    return new camera::CAHVOREModel( file );
  } else if ( boost::ends_with(file, ".cahvor") ||
              boost::ends_with(file, ".cmod") ) {
    return new camera::CAHVORModel( file );
  } else if ( boost::ends_with(file, ".cahv") ||
              boost::ends_with(file, ".pin") ) {
    return new camera::CAHVModel( file );
  } else if ( boost::ends_with(file, ".pinhole") ||
              boost::ends_with(file, ".tsai") ) {
    return new camera::PinholeModel( file );
  } else {
    vw_throw( ArgumentErr() << "Unknown camera \"" << file << "\".\n" );
  }
  // Should never be hit;
  return NULL;
}

int main( int argc, char* argv[] ) {

  Vector2i image_size;
  boost::shared_ptr<camera::CameraModel> camera_model;
  if ( argc == 2 ) {
    camera_model = boost::shared_ptr<camera::CameraModel>( load_camera( argv[1] ) );
  } else if ( argc == 3 ) {
    // hangle adjust camera
    if ( boost::ends_with( argv[1], ".adjust" ) ) {
      camera::AdjustedCameraModel* adjusted =
        new camera::AdjustedCameraModel( boost::shared_ptr<camera::CameraModel>(
                                                      load_camera( argv[2] )));
      adjusted->read( argv[1] );
      camera_model = boost::shared_ptr<camera::CameraModel>( adjusted );
    } else {
      camera::AdjustedCameraModel* adjusted =
        new camera::AdjustedCameraModel( boost::shared_ptr<camera::CameraModel>(
                                                      load_camera( argv[1] )));
      adjusted->read( argv[2] );
      camera_model = boost::shared_ptr<camera::CameraModel>( adjusted );
    }
  } else {
    return 1;
  }

  Vector3 camera_position =
    camera_model->camera_center(Vector2());
  std::cout << std::setprecision(15) << camera_position[0] << " "
            << camera_position[1] << " " << camera_position[2] << "\n";

  return 0;
}
