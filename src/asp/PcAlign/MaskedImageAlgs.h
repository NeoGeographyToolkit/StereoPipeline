// __BEGIN_LICENSE__
//  Copyright (c) 2006-2025, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

/// \file MaskedImageAlgs.h
///
/// Algorithms that operate on masked float images. All computations are with
/// double precision, for better accuracy.
///
#ifndef __ASP_PC_ALIGN_MASKEDIMAGEALGS_H__
#define __ASP_PC_ALIGN_MASKEDIMAGEALGS_H__

#include <vw/Image/ImageView.h>

namespace vw {
  
// Find the valid pixels. Use long long, to avoid integer overflow.
long long validCount(vw::ImageView<vw::PixelMask<float>> const& img);

// Compute the median of the valid pixels
double maskedMedian(vw::ImageView<vw::PixelMask<float>> const& img);
  
// Comput normalized median absolute deviation
double normalizedMad(vw::ImageView<vw::PixelMask<float>> const& img, 
                     double median);
  
// Find the mean of valid pixels
double maskedMean(vw::ImageView<vw::PixelMask<float>> const& img);
  
// Find the std dev of valid pixels
double maskedStdDev(vw::ImageView<vw::PixelMask<float>> const& img, double mean);
  
// Filter outside this range
void rangeFilter(vw::ImageView<vw::PixelMask<float>> & diff, 
                 double min_val, double max_val);
  
// Invalidate pixels in first image that are invalid in second image
void intersectValid(vw::ImageView<vw::PixelMask<float>> & img1, 
                    vw::ImageView<vw::PixelMask<float>> const& img2);
  
// Filter by normalized median absolute deviation with given factor
void madFilter(vw::ImageView<vw::PixelMask<float>> & diff, double outlierFactor);
  
// Group y-values into bins based on their corresponding x-values and computes
// a statistic (like mean, sum, etc.) for each bin. This reimplements
// scipy.stats.binned_statistic. Do not return bin number, as we don't need it.
// Also some stats that are not needed were not implemented.
void binnedStatistics(vw::ImageView<vw::PixelMask<float>> const& x, 
                      vw::ImageView<vw::PixelMask<float>> const& y,
                      std::string stat, int nbins, 
                      vw::Vector2 const& bin_range,
                      // Outputs
                      std::vector<double> & bin_stat,
                      std::vector<double> & bin_edges,
                      std::vector<double> & bin_centers);
  
} // end namespace vw

#endif // __ASP_PC_ALIGN_MASKEDIMAGEALGS_H__

