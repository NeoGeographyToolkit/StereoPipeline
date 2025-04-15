// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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


/// \file ImageNormalization.h
///

#ifndef __IMAGE_NORMALIZATION_H__
#define __IMAGE_NORMALIZATION_H__

#include <vw/Image/Algorithms.h>

#include <boost/shared_ptr.hpp>

namespace vw {
  class DiskImageResource;
}

namespace asp {

typedef vw::Vector<vw::float32,6> Vector6f;

/// Returns the correct nodata value from the input images or the input options
void get_nodata_values(boost::shared_ptr<vw::DiskImageResource> left_rsrc,
                        boost::shared_ptr<vw::DiskImageResource> right_rsrc,
                        float & left_nodata_value,
                        float & right_nodata_value);

// Calculate the min and max for all images to normalize, while respecting
// given options.
void calcImageSeqMinMax(bool force_use_entire_range,
                        bool individually_normalize,
                        bool use_percentile_stretch,
                        bool do_not_exceed_min_max,
                        std::vector<vw::Vector6f> const& image_stats,
                        // Outputs
                        std::vector<double> & min_vals,
                        std::vector<double> & max_vals);

/// Normalize the intensity of two images based on input statistics
void normalize_images(bool force_use_entire_range,
                      bool individually_normalize,
                      bool use_percentile_stretch,
                      bool do_not_exceed_min_max,
                      vw::Vector6f const& left_stats,
                      vw::Vector6f const& right_stats,
                      vw::ImageViewRef<vw::PixelMask<float>> & left_img,
                      vw::ImageViewRef<vw::PixelMask<float>> & right_img);

} // end namespace asp

#endif // __IMAGE_NORMALIZATION_H__
