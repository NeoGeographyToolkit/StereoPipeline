#ifndef __RMAX_STEREO_SESSION_H__
#define __RMAX_STEREO_SESSION_H__

#include "StereoSessionPinhole.h"

class StereoSessionRmax: public StereoSession {
  
public:

  virtual ~StereoSessionRmax() {}

  // Correct lens distortion and epipolar-rectify the images
  virtual boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                                  std::string camera_file = "");

  static StereoSession* construct() { return new StereoSessionRmax; }
};

#endif // __RMAX_STEREO_SESSION_H__
