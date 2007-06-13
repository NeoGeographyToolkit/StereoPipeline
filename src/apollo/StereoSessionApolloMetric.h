#ifndef __STEREO_SESSION_APOLLO_METRIC_H__
#define __STEREO_SESSION_APOLLO_METRIC_H__

#include "StereoSession.h"
#include "StereoSessionKeypoint.h"

class StereoSessionApolloMetric: public StereoSessionKeypoint {
  
public:

  virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                             boost::shared_ptr<vw::camera::CameraModel> &cam2);

  void pre_pointcloud_hook(std::string const& input_file, std::string & output_file);

  static StereoSession* construct() { return new StereoSessionApolloMetric; }
};

#endif
