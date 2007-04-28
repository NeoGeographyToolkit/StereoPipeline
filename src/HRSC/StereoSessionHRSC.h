#ifndef __STEREO_SESSION_HRSC_H__
#define __STEREO_SESSION_HRSC_H__

#include "StereoSession.h"
#include "StereoSessionKeypoint.h"

class StereoSessionHRSC: public StereoSessionKeypoint {
  
public:

  virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                             boost::shared_ptr<vw::camera::CameraModel> &cam2);

  static StereoSession* construct() { return new StereoSessionHRSC; }
};

#endif // __PINHOLE_STEREO_SESSION_H__
