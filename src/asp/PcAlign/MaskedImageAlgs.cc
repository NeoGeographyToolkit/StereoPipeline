// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file MaskedImageAlgs.cc
///
/// Algorithms that operate on masked float images. All computations are with
/// double precision, for better accuracy.
///
#include <asp/PcAlign/MaskedImageAlgs.h>
#include <vw/Image/PixelMask.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Statistics.h>
#include <vw/Core/Exception.h>

#include <omp.h>

namespace vw {

// Find the valid pixels. Use long long, to avoid integer overflow.
long long validCount(vw::ImageView<vw::PixelMask<float>> const& img) {
  
  long long count = 0;
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row)))
        count++;
    }
  }
  
  return count;
}

// Compute the median of the valid pixels
double maskedMedian(vw::ImageView<vw::PixelMask<float>> const& img) {

  // Allocate enough space first  
  std::vector<float> vals(img.cols()*img.rows());
  vals.clear();
  
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row)))
        vals.push_back(img(col, row).child());
    }
  }
  
  if (vals.empty())
    vw::vw_throw(vw::ArgumentErr() << "No valid pixels found in median calculation.\n");
  
  return vw::math::destructive_median(vals);
}

// Comput normalized median absolute deviation
double normalizedMad(vw::ImageView<vw::PixelMask<float>> const& img, 
                     double median) {
  
  // Compute the median absolute deviation
  std::vector<float> vals(img.cols()*img.rows());
  vals.clear();
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row)))
        vals.push_back(std::abs(img(col, row).child() - median));
    }
  }
  
  if (vals.empty())
    vw::vw_throw(vw::ArgumentErr() << "No valid pixels found in MAD calculation.\n");
  
  double mad = vw::math::destructive_median(vals);

  // The normalization factor is to make this equivalent to the standard deviation  
  return  1.4826 * mad;
}

// Find the mean of valid pixels
double maskedMean(vw::ImageView<vw::PixelMask<float>> const& img) {
  
  double sum = 0.0;
  long long count = 0;
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row))) {
        sum += img(col, row).child();
        count++;
      }
    }
  }
  
  if (count == 0)
    return 0.0;
  
  return sum / count;
}

// Find the std dev of valid pixels
double maskedStdDev(vw::ImageView<vw::PixelMask<float>> const& img, double mean) {
  
  double sum = 0.0;
  long long count = 0;
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (is_valid(img(col, row))) {
        sum += (img(col, row).child() - mean) * (img(col, row).child() - mean);
        count++;
      }
    }
  }
  
  if (count == 0)
    return 0.0;
  
  return sqrt(sum / count);
}
  
// Filter outside this range
void rangeFilter(vw::ImageView<vw::PixelMask<float>> & diff, 
                 double min_val, double max_val) {

  #pragma omp parallel for
  for (int col = 0; col < diff.cols(); col++) {
    for (int row = 0; row < diff.rows(); row++) {
      if (is_valid(diff(col, row)) && 
          (diff(col, row).child() < min_val || diff(col, row).child() > max_val))
        diff(col, row).invalidate();
    }
  }
}

// Invalidate pixels in first image that are invalid in second image
void intersectValid(vw::ImageView<vw::PixelMask<float>> & img1, 
                    vw::ImageView<vw::PixelMask<float>> const& img2) {
  
  #pragma omp parallel for
  for (int col = 0; col < img1.cols(); col++) {
    for (int row = 0; row < img1.rows(); row++) {
      if (!is_valid(img2(col, row)))
        img1(col, row).invalidate();
    }
  }
}

// Filter by normalized median absolute deviation with given factor
void madFilter(vw::ImageView<vw::PixelMask<float>> & diff, double outlierFactor) {
  
  double median = maskedMedian(diff);
  double mad = normalizedMad(diff, median);
  double min_val = median - outlierFactor * mad;
  double max_val = median + outlierFactor * mad;
  rangeFilter(diff, min_val, max_val);
}

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
                      std::vector<double> & bin_centers) {

  // x and y must have the same size
  if (x.cols() != y.cols() || x.rows() != y.rows())
    vw::vw_throw(vw::ArgumentErr() 
                 << "The x and y images must have the same size.\n");
  
  double bin_width = (bin_range[1] - bin_range[0]) / nbins;
  
  // Resize output vectors
  bin_stat.resize(nbins);
  bin_edges.resize(nbins + 1);
  bin_centers.resize(nbins);

  for (int i = 0; i <= nbins; i++)
    bin_edges[i] = bin_range[0] + i * bin_width;

  // To get the bin center shift right by half a bin width. Skip the last edge.
  // There are as many bin centers as there are bins
  for (int i = 0; i < nbins; i++)
    bin_centers[i] = bin_edges[i] + bin_width / 2.0;

  // Accumulate the values in each bin. It is enough to keep this as float.
  std::vector<std::vector<float>> bin_values(nbins);
  for (int col = 0; col < x.cols(); col++) {
    for (int row = 0; row < x.rows(); row++) {
      
      if (!is_valid(x(col, row)) || !is_valid(y(col, row)))
        continue; // Skip invalid pixels
      
      // Skip values outside range
      if (x(col, row).child() < bin_range[0] || x(col, row).child() > bin_range[1])
        continue;
      
      // Bin index  
      int bin_index = (int)std::floor((x(col, row).child() - bin_range[0]) / bin_width);
      if (bin_index >= nbins) // This can happen due to numerical errors
        bin_index = nbins - 1;
      if (bin_index < 0)
        bin_index = 0;
        
      bin_values[bin_index].push_back(y(col, row).child());
    }
  }
  
  // Compute the statistic for each bin
  #pragma omp parallel for
  for (int i = 0; i < nbins; i++) {
    
    if (bin_values[i].empty()) {
      bin_stat[i] = 0;
      continue;
    }
    
    if (stat == "mean")
      bin_stat[i] = vw::math::mean(bin_values[i]);
     else if (stat == "count")
       bin_stat[i] = bin_values[i].size();
     else if (stat == "median")
       bin_stat[i] = vw::math::destructive_median(bin_values[i]);
    else
      vw::vw_throw(vw::ArgumentErr() << "Invalid statistic: " << stat << ".\n");
   }
}

} // end namespace vw
