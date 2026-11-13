// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file BundleAdjustMatchFiles.h
///
/// Match-file processing for bundle_adjust: convergence angles, mapprojected
/// match offsets, and clean match file creation.

#ifndef __BUNDLE_ADJUST_MATCH_FILES_H__
#define __BUNDLE_ADJUST_MATCH_FILES_H__

#include <asp/Camera/BundleAdjustParamMap.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/Bathymetry.h>

#include <vw/Cartography/Datum.h>
#include <vw/FileIO/DiskImageView.h>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace vw {
    class InterestPoint;
}

namespace asp {

// A structure to hold percentiles of given sorted values. This sorts the inputs.
// The input can be float or double. We will keep the result as double.
struct MatchPairStats {
  int left_cam_index, right_cam_index, num_vals;
  double val25, val50, val75, val85, val95;
  MatchPairStats(): left_cam_index(0), right_cam_index(0), num_vals(0), val25(0), val50(0),
                    val75(0), val85(0), val95(0) {}
  template<class T>
  void populate(int left_index, int right_index, std::vector<T> & vals) {
    std::sort(vals.begin(), vals.end());
    left_cam_index  = left_index;
    right_cam_index = right_index;
    num_vals = vals.size();
    if (num_vals > 0) {
      val25 = vals[0.25*num_vals];
      val50 = vals[0.50*num_vals];
      val75 = vals[0.75*num_vals];
      val85 = vals[0.85*num_vals];
      val95 = vals[0.95*num_vals];
    }
  }
};

struct HorizVertErrorStats {
  int left_cam_index, right_cam_index;
  float horiz_error_median, vert_error_median;
  float horiz_error_mean, vert_error_mean;
  float horiz_error_stddev, vert_error_stddev;
  int num_errors;
  HorizVertErrorStats(): left_cam_index(0), right_cam_index(0),
                         horiz_error_median(0), vert_error_median(0),
                         horiz_error_mean(0), vert_error_mean(0),
                         horiz_error_stddev(0), vert_error_stddev(0),
                         num_errors(0) {}
};

// Mapproject interest points onto a DEM and find the norm of their
// disagreement in meters. It is assumed that dem_georef
// was created by bilinear interpolation. The cameras must be with
// the latest adjustments applied to them.
void calcPairMapprojOffsets(int left_cam_index, int right_cam_index,
                           std::vector<vw::CamPtr>            const& optimized_cams,
                           std::vector<vw::InterestPoint> const& left_ip,
                           std::vector<vw::InterestPoint> const& right_ip,
                           vw::GeoReference      const& dem_georef,
                           vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                           // Will append below
                           std::vector<vw::Vector<float, 4>>       & mapprojPoints,
                           std::vector<float>                      & mapprojOffsets);

// Calculate convergence angles. Remove the outliers flagged earlier,
// if remove_outliers is true. Compute offsets of mapprojected matches,
// if a DEM is given. These are done together as they rely on
// reloading interest point matches, which is expensive so the matches
// are used for both operations.

void matchFilesProcessing(vw::ControlNetwork       const& cnet,
                          asp::BaBaseOptions           const& opt,
                          std::vector<vw::CamPtr>      const& optimized_cams,
                          bool                                remove_outliers,
                          std::set<int>                const& outliers,
                          std::set<std::pair<int, int>> const& outlier_obs,
                          std::string                  const& mapproj_dem,
                          bool                                propagate_errors,
                          vw::Vector<double>           const& horizontal_stddev_vec,
                          bool                                save_clean_matches,
                          std::map<std::pair<int, int>, std::string> const& match_files,
                          bool                                matches_as_txt);

} // end namespace asp

#endif // __BUNDLE_ADJUST_MATCH_FILES_H__
