// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionKeypoint.h
///

#ifndef __STEREO_SESSION_KEYPOINT_H__
#define __STEREO_SESSION_KEYPOINT_H__

#include <asp/Sessions/StereoSession.h>

#include <vw/Image.h>
#include <vw/Math.h>

// This abstract class overrides the default pre-processing behavior
// by adding keypoint alignment of the images.
class StereoSessionKeypoint : public StereoSession {

public:

  // Stage 1: Preprocessing
  //
  // Pre file is a pair of images.            ( ImageView<PixelT> )
  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string & output_file1, std::string & output_file2);

  // Stage 4: Point cloud generation
  //
  // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
  virtual void pre_pointcloud_hook(std::string const& input_file, std::string & output_file);

protected:

  // To speed up things one can optionally sub-sample the images
  virtual std::string create_subsampled_align_image(std::string const& image_file, std::string const& suffix);
  void scale_align_matrix(vw::math::Matrix<double> & align_matrix);
};


#endif // __STEREO_SESSION_KEYPOINT_H__
