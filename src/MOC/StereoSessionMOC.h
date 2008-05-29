#ifndef __STEREO_SESSION_MOC_H__
#define __STEREO_SESSION_MOC_H__

#include "StereoSession.h"
#include "StereoSessionKeypoint.h"

class StereoSessionMOC: public StereoSessionKeypoint {
  
public:

  boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                          std::string camera_file);

  static StereoSession* construct() { return new StereoSessionMOC; }
};

#endif // __STEREO_SESSION_MOC_H__
