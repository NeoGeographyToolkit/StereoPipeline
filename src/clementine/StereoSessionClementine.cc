#include <boost/shared_ptr.hpp>

#include <vw/Camera/PinholeModel.h> 
#include <vw/Math/EulerAngles.h>
#include <vw/FileIO.h>

#include "clementine/StereoSessionClementine.h"

#include "stereo.h"
#include "file_lib.h"
#include "Spice.h"

#include <list>

using namespace std;
using namespace vw;
using namespace vw::camera;

void StereoSessionClementine::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                              boost::shared_ptr<camera::CameraModel> &cam2) {  
  PinholeModel* cam_ptr1 = new PinholeModel(m_left_camera_file);
  PinholeModel* cam_ptr2 = new PinholeModel(m_right_camera_file);
  cam1 = boost::shared_ptr<camera::CameraModel>(cam_ptr1);
  cam2 = boost::shared_ptr<camera::CameraModel>(cam_ptr2);
}


