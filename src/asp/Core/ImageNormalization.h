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

  /// Normalize the intensity of two grayscale images based on input statistics
  template<class ImageT>
  void normalize_images(bool force_use_entire_range,
                        bool individually_normalize,
                        bool use_percentile_stretch,
                        bool do_not_exceed_min_max,
                        vw::Vector6f const& left_stats,
                        vw::Vector6f const& right_stats,
                        ImageT & left_img, ImageT & right_img){
    
    // These arguments must contain: (min, max, mean, std)
    VW_ASSERT(left_stats.size() == 6 && right_stats.size() == 6,
              vw::ArgumentErr() << "Expecting a vector of size 6 in normalize_images()\n");

    // If the input stats don't contain the stddev, must use the entire range version.
    // - This should only happen when normalizing ISIS images for ip_matching purposes.
    if ((left_stats[3] == 0) || (right_stats[3] == 0))
      force_use_entire_range = true;

    if (force_use_entire_range) { // Stretch between the min and max values
      if (individually_normalize) {
        vw::vw_out() << "\t--> Individually normalize images to their respective min max\n";
        left_img = normalize(left_img, left_stats [0], left_stats [1], 0.0, 1.0);
        right_img = normalize(right_img, right_stats[0], right_stats[1], 0.0, 1.0);
      } else { // Normalize using the same stats
        double low = std::min(left_stats[0], right_stats[0]);
        double hi  = std::max(left_stats[1], right_stats[1]);
        vw::vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";
        left_img = normalize(left_img, low, hi, 0.0, 1.0);
        right_img = normalize(right_img, low, hi, 0.0, 1.0);
      }
    } else { // Don't force the entire range
      double left_min, left_max, right_min, right_max;
      if (use_percentile_stretch) {
        // Percentile stretch
        left_min  = left_stats [4];
        left_max  = left_stats [5];
        right_min = right_stats[4];
        right_max = right_stats[5];
      } else {
        // Two standard deviation stretch
        left_min  = left_stats [2] - 2*left_stats [3];
        left_max  = left_stats [2] + 2*left_stats [3];
        right_min = right_stats[2] - 2*right_stats[3];
        right_max = right_stats[2] + 2*right_stats[3];
        
        if (do_not_exceed_min_max) {
          // This is important for ISIS which may have special pixels beyond the min and max
          left_min = std::max(left_min,   (double)left_stats[0]);
          left_max = std::min(left_max,   (double)left_stats[1]);
          right_min = std::max(right_min, (double)right_stats[0]);
          right_max = std::min(right_max, (double)right_stats[1]);
        }
      }

      // The images are normalized so most pixels fall into this range,
      // but the data is not clamped so some pixels can fall outside this range.
      if (individually_normalize > 0) {
        vw::vw_out() << "\t--> Individually normalize images\n";
        left_img = normalize(left_img, left_min,  left_max,  0.0, 1.0);
        right_img = normalize(right_img, right_min, right_max, 0.0, 1.0);
      } else { // Normalize using the same stats
        double low = std::min(left_min, right_min);
        double hi  = std::max(left_max, right_max);
        vw::vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";

        double std_ratio = std::min(left_stats[3], right_stats[3]) / 
                           std::max(left_stats[3], right_stats[3]);
        if (std_ratio < 0.2)
          vw::vw_out(vw::WarningMessage) 
            << "The standard deviations of the two images are very different. "
            << "Consider using the option --individually-normalize.\n";

        if (!do_not_exceed_min_max) {
          left_img = normalize(left_img, low, hi, 0.0, 1.0);
          right_img = normalize(right_img, low, hi, 0.0, 1.0);
        }else{
          left_img  = normalize(left_img,  std::max(low, left_min),  std::min(hi, left_max),
                                0.0, 1.0);
          right_img = normalize(right_img, std::max(low, right_min), std::min(hi, right_max),
                                0.0, 1.0);
        }
      }
    }
  
    return;
  }
  
}

#endif // __IMAGE_NORMALIZATION_H__
