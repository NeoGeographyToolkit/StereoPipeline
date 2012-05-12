// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file StereoSessionPinhole.h
///

#ifndef __STEREO_SESSION_PINHOLE_H__
#define __STEREO_SESSION_PINHOLE_H__

#include <asp/Sessions/StereoSession.h>

namespace asp {

  class StereoSessionPinhole : public StereoSession {

  protected:
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
    virtual vw::ImageViewRef<vw::PixelMask<vw::Vector2f> >
    pre_pointcloud_hook(std::string const& input_file);

    static StereoSession* construct() { return new StereoSessionPinhole; }
  };

}

#endif // __STEREO_SESSION_PINHOLE_H__
