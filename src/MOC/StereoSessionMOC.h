#ifndef __STEREO_SESSION_MOC_H__
#define __STEREO_SESSION_MOC_H__

#include "StereoSession.h"
#include "StereoSessionKeypoint.h"

class StereoSessionMOC: public StereoSessionKeypoint {
  
public:

  virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                             boost::shared_ptr<vw::camera::CameraModel> &cam2);

  static StereoSession* construct() { return new StereoSessionMOC; }
};

#endif // __STEREO_SESSION_MOC_H__
