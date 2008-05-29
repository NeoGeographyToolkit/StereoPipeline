#ifndef __STEREO_SESSION_HRSC_H__
#define __STEREO_SESSION_HRSC_H__

#include "StereoSession.h"
#include "StereoSessionKeypoint.h"

class StereoSessionHRSC: public StereoSessionKeypoint {
  
public:

  boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                          std::string camera_file);

  static StereoSession* construct() { return new StereoSessionHRSC; }
};

#endif // __PINHOLE_STEREO_SESSION_H__
