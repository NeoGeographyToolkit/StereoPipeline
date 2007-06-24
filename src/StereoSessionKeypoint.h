#ifndef __STEREO_SESSION_KEYPOINT_H__
#define __STEREO_SESSION_KEYPOINT_H__

#include "StereoSession.h"

#include <vw/Image.h>
#include <vw/Math.h>

// This abstract class overrides the default pre-processing behavior
// by adding keypoint alignment of the images.
class StereoSessionKeypoint : public StereoSession {
  
public:

  StereoSessionKeypoint() { m_sub_sampling = 1; }
  StereoSessionKeypoint(unsigned int sub_sampling) { m_sub_sampling = sub_sampling; }

  virtual void initialize(struct DFT_F& stereo_defaults);

  // Stage 1: Preprocessing
  //
  // Pre file is a pair of images.            ( ImageView<PixelT> )
  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string & output_file1, std::string & output_file2);

  // Stage 4: Point cloud generation
  //
  // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
  virtual void pre_pointcloud_hook(std::string const& input_file, std::string & output_file);

  void set_sub_sampling(const unsigned int sub_sampling) { m_sub_sampling = sub_sampling; }
  void get_sub_sampling(const unsigned int sub_sampling) const { m_sub_sampling; }

protected:
  vw::math::Matrix<double> determine_image_alignment(std::string const& input_file1, std::string const& input_file2);
  // To speed up things one can optionally sub-sample the images
  unsigned int m_sub_sampling;
  virtual std::string create_subsampled_align_image(std::string const& image_file, std::string const& suffix);
  void scale_align_matrix(vw::math::Matrix<double> & align_matrix);
};


#endif // __STEREO_SESSION_KEYPOINT_H__
