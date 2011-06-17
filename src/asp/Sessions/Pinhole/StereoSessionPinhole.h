// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionPinhole.h
///

#ifndef __STEREO_SESSION_PINHOLE_H__
#define __STEREO_SESSION_PINHOLE_H__

#include <asp/Sessions/StereoSession.h>

namespace asp {

  class StereoSessionPinhole: public StereoSession {

  public:

    virtual ~StereoSessionPinhole() {}

    // Correct lens distortion and epipolar-rectify the images
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "");

    // Stage 1: Preprocessing
    //
    // Pre file is a pair of images.            ( ImageView<PixelT> )
    // Post file is a grayscale images.         ( ImageView<PixelGray<flaot> > )
    virtual void pre_preprocessing_hook(std::string const& input_file1,
                                        std::string const& input_file2,
                                        std::string &output_file1,
                                        std::string &output_file2);

    // Stage 4: Point cloud generation
    //
    // Pre file is a disparity map.  ( ImageView<PixelDisparity<float> > )
    // Post file is point image.     ( ImageView<Vector3> )
    virtual void pre_pointcloud_hook(std::string const& input_file,
                                     std::string & output_file);

    static StereoSession* construct() { return new StereoSessionPinhole; }
  };

}

#endif // __STEREO_SESSION_PINHOLE_H__
