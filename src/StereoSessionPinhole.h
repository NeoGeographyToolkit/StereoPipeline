#ifndef __PINHOLE_STEREO_SESSION_H__
#define __PINHOLE_STEREO_SESSION_H__

#include "StereoSession.h"

class StereoSessionPinhole: public StereoSession {
  
public:

  virtual ~StereoSessionPinhole() {}

  // Correct lens distortion and epipolar-rectify the images
  virtual boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                                  std::string camera_file = "");

  // Stage 1: Preprocessing
  //
  // Pre file is a pair of images.            ( ImageView<PixelT> )
  // Post file is a grayscale images.         ( ImageView<PixelGray<flaot> > )
  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string &output_file1, std::string &output_file2);

  static StereoSession* construct() { return new StereoSessionPinhole; }
};

#endif // __PINHOLE_STEREO_SESSION_H__
