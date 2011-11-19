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

  class StereoSessionPinhole : public StereoSession {
    // This should probably be factored into the greater StereoSession
    // as ISIS Session does something very similar to this.
    template <class ViewT>
    vw::Vector4f gather_stats( vw::ImageViewBase<ViewT> const& view_base,
                               std::string const& tag) {
      using namespace vw;
      vw_out(InfoMessage) << "\t--> Computing statistics for the "+tag+" image\n";
      ViewT image = view_base.impl();
      int stat_scale = int(ceil(sqrt(float(image.cols())*float(image.rows()) / 1000000)));

      ChannelAccumulator<vw::math::CDFAccumulator<float> > accumulator;
      for_each_pixel( subsample( edge_extend(image, ConstantEdgeExtension()),
                                 stat_scale ), accumulator );
      Vector4f result( accumulator.quantile(0),
                       accumulator.quantile(1),
                       accumulator.approximate_mean(),
                       accumulator.approximate_stddev() );
      vw_out(InfoMessage) << "\t  " << tag << ": [ lo: " << result[0] << " hi: " << result[1]
                          << " m: " << result[2] << " s: " << result[3] << " ]\n";
      return result;
    }

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
