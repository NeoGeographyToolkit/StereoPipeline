#ifndef __STEREO_SESSION_APOLLO_METRIC_H__
#define __STEREO_SESSION_APOLLO_METRIC_H__

#include "StereoSession.h"
#include "StereoSessionKeypoint.h"

class StereoSessionApolloMetric: public StereoSessionKeypoint {
  
public:
  
  virtual boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                                  std::string camera_file = "");

  void pre_pointcloud_hook(std::string const& input_file, std::string & output_file);

  static StereoSession* construct() { return new StereoSessionApolloMetric; }
};

#endif
