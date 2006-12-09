
#ifndef __STEREO_METEDATA_H__
#define __STEREO_METEDATA_H__

#include <vw/Camera/OrbitingPushbroomModel.h>

class StereoImageMetadata {
public:
  virtual ~StereoImageMetadata() {}
  virtual vw::camera::OrbitingPushbroomModel camera_model() {}
};

#endif // __STEREO_METEDATA_H__
