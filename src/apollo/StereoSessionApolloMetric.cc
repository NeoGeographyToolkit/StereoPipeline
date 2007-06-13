#include <boost/shared_ptr.hpp>

#include <vw/Camera/PinholeModel.h>

#include "apollo/StereoSessionApolloMetric.h"

#include "stereo.h"
#include "file_lib.h"

using namespace vw;
using namespace vw::camera;

void StereoSessionApolloMetric::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                              boost::shared_ptr<camera::CameraModel> &cam2) {
  cam1 = boost::shared_ptr<camera::CameraModel>(new PinholeModel(m_left_camera_file));
  cam2 = boost::shared_ptr<camera::CameraModel>(new PinholeModel(m_right_camera_file));
}

void StereoSessionApolloMetric::pre_pointcloud_hook(std::string const& input_file, std::string & output_file) {
  
  boost::shared_ptr<camera::CameraModel> left_camera, right_camera;
  this->camera_models(left_camera, right_camera);
  write_orbital_reference_model(m_out_prefix + "-OrbitViz.vrml", *left_camera, *right_camera);
  StereoSessionKeypoint::pre_pointcloud_hook(input_file, output_file);

}


