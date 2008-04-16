#ifndef __STEREO_SESSION_ISIS_H__
#define __STEREO_SESSION_ISIS_H__

#include "StereoSession.h"

class StereoSessionIsis: public StereoSession {
  
public:

  virtual void camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                             boost::shared_ptr<vw::camera::CameraModel> &cam2);

  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string & output_file1, std::string & output_file2);

  static StereoSession* construct() { return new StereoSessionIsis; }
};

#endif // __STEREO_SESSION_ISIS_H__
