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

/// \file BundleAdjustUtils.h
///

#ifndef __BUNDLE_ADJUST_UTILS_H__
#define __BUNDLE_ADJUST_UTILS_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/BBox.h>
#include <vw/Camera/CameraModel.h>
#include <vw/BundleAdjustment/CameraRelation.h>

#include <string>
#include <vector>
#include <set>

#include <boost/smart_ptr/shared_ptr.hpp>

// Forward declarations
namespace vw {

  namespace cartography {
    class GeoReference;
  }

  namespace geometry {
    class dPoly;
  }
  
  template<typename PixelT>
  class ImageViewRef;

  template<typename PixelT>
  class PixelMask;
}

namespace asp {
  
const int NUM_XYZ_PARAMS  = 3;
const int NUM_QUAT_PARAMS = 4;
const int PIXEL_SIZE      = 2;

typedef vw::ba::CameraRelationNetwork<vw::ba::JFeature> CRN;
  
// Compute the camera footprint polygon shape and its bounding box. Used a
// cached result if available. Cache the current result if computed.
void camera_footprint(std::string const& dem_file,
                      std::string const& image_file,
                      vw::CamPtr  const& camera_model,
                      std::string const& out_prefix,
                      // Outputs
                      vw::geometry::dPoly & footprint,
                      vw::BBox2 & footprint_bbox);

// Determine which camera images overlap by finding the lon-lat
// bounding boxes of their footprints given the specified DEM, expand
// them by a given percentage, and see if those intersect. A higher
// percentage should be used when there is more uncertainty in input
// camera poses. Specify as: 'dem.tif 15'.
void buildOverlapList(std::string const& out_prefix,
                      std::string const& dem_file,
                      double pct_for_overlap,
                      int overlap_limit,
                      bool match_first_to_last,
                      std::vector<std::string> const& image_files,
                      std::vector<vw::CamPtr> const& camera_models,
                      std::set<std::pair<std::string, std::string>> & overlap_list);

/// Ensure that no images, camera files, or adjustment names are duplicate.
/// That will cause the output files to overwrite each other!
void check_for_duplicates(std::vector<std::string> const& image_files,
                          std::vector<std::string> const& camera_files,
                          std::string const& out_prefix);

// Shoot rays from all matching interest points. Intersect those with a DEM. Find
// their average. Project it vertically onto the DEM. Invalid or uncomputable
// xyz are set to the zero vector.
void update_tri_pts_from_dem(vw::ba::ControlNetwork const& cnet,
                             std::set<int> const& outliers,
                             std::vector<vw::CamPtr> const& camera_models,
                             vw::cartography::GeoReference const& dem_georef,
                             vw::ImageViewRef<vw::PixelMask<double>> const& masked_dem,
                             // Output
                             std::vector<vw::Vector3> & dem_xyz_vec);

// Flag outliers by reprojection error with input cameras. This assumes that
// the input cameras are pretty accurate.
void flag_initial_outliers(vw::ba::ControlNetwork const& cnet,
                            asp::CRN const& crn,
                            std::vector<vw::CamPtr> const& camera_models,
                            double max_init_reproj_error,
                            // Output
                            std::set<int> & outliers);

// Manufacture a CSM state file from an adjust file
std::string csmStateFile(std::string const& adjustFile);

// Manufacture an RPC state file from an adjust file
std::string rpcAdjustedFile(std::string const& adjustFile);

// Put the triangulated points in a vector. Update the cnet from the DEM,
// if we have one.
void formTriVec(std::vector<vw::Vector3> const& dem_xyz_vec,
                bool have_dem,
                // Outputs
                vw::ba::ControlNetwork & cnet,
                std::vector<double>    & orig_tri_points_vec,
                std::vector<double>    & tri_points_vec);

// Average all y pixel residuals per row then fill in from neighbors 
void residualsPerRow(vw::ba::ControlNetwork const& cnet,
                     asp::CRN const& crn,
                     std::set<int> const& outliers,
                     std::vector<std::string> const& image_files,
                     std::vector<vw::CamPtr> const& camera_models,
                     // Output
                     std::vector<std::vector<double>> & residuals);


// A type alias for clarity, representing the mapping from an image pair to its sigma.
using MatchPairSigmaMap = std::map<std::pair<std::string, std::string>, double>;

// Read sigmas for some pairs of input images. The file is in format: 
// image1 image2 sigma, with space as separator. Any order of image1 and image2
// is supported. 
typedef std::map<std::pair<int, int>, double> MatchSigmasMap;
void readMatchPairSigmas(std::string const& sigmaFilename,
                         std::vector<std::string> const& imageFiles,
                         MatchSigmasMap & matchSigmas);

} // end namespace asp

#endif // __BUNDLE_ADJUST_UTILS_H__
