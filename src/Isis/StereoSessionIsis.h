#ifndef __STEREO_SESSION_ISIS_H__
#define __STEREO_SESSION_ISIS_H__

#include "StereoSession.h"

class StereoSessionIsis: public StereoSession {
  
public:

  virtual ~StereoSessionIsis() {}

  virtual boost::shared_ptr<vw::camera::CameraModel> camera_model(std::string image_file, 
                                                                  std::string camera_file = "");

  // Stage 1: Preprocessing
  //
  // Pre file is a pair of images.            ( ImageView<PixelT> )
  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string & output_file1, std::string & output_file2);

  // Stage 4: Point cloud generation
  //
  // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
  virtual void pre_pointcloud_hook(std::string const& input_file, std::string & output_file);

  static StereoSession* construct() { return new StereoSessionIsis; }

private:
  vw::math::Matrix<double> determine_image_alignment(std::string const& input_file1, std::string const& input_file2, float lo, float hi);

};

#endif // __STEREO_SESSION_ISIS_H__
