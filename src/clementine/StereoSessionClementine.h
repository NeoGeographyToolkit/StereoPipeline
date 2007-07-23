#ifndef __STEREO_SESSION_CLEMENTINE_H__
#define __STEREO_SESSION_CLEMENTINE_H__

#include "StereoSession.h"
#include "StereoSessionKeypoint.h"

class StereoSessionClementine: public StereoSessionKeypoint {
  
public:

  virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                             boost::shared_ptr<vw::camera::CameraModel> &cam2);

  static StereoSession* construct() { return new StereoSessionClementine; }
};

#endif
