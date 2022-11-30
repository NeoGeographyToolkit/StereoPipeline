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

#include <string>
#include <vector>
#include <set>

#include <boost/smart_ptr/shared_ptr.hpp>

// Forward declarations
namespace vw {

  namespace camera {
    class CameraModel;
  }
  
  namespace ba {
    class ControlNetwork;
    class JFeature;

    template<typename T>
    class CameraRelationNetwork;
  }
  
  namespace cartography {
    class GeoReference;
  }

  template<typename PixelT>
  class ImageViewRef;

  template<typename PixelT>
  class PixelMask;
}

namespace asp{
  /// Read both kinds of adjustments
  void read_adjustments(std::string const& filename,
                        bool        & piecewise_adjustments,
                        vw::Vector2 & adjustment_bounds,
                        std::vector<vw::Vector3> & position_correction,
                        std::vector<vw::Quat>    & pose_correction,
			vw::Vector2 & pixel_offset, double & scale,
			std::string & session);

  /// Write piecewise adjustments
  void write_adjustments(std::string const & filename,
                         vw::Vector2 const & adjustment_bounds,
                         std::vector<vw::Vector3> const& position_correction,
                         std::vector<vw::Quat>    const& pose_correction,
                         std::string              const& session);

  /// Write global adjustments
  // TODO(oalexan1): This should be unified with analogous logic in VW
  void write_adjustments(std::string const& filename,
                         vw::Vector3 const& position_correction,
                         vw::Quat    const& pose_correction);

  ///
  void compute_stereo_residuals(std::vector<boost::shared_ptr<vw::camera::CameraModel>>
                                const& camera_models,
                                vw::ba::ControlNetwork const& cnet);

  // Compute a camera footprint's bounding box. Used a cached result if available.
  // Cache the current result if computed.
  vw::BBox2 camera_bbox_with_cache(std::string const& dem_file,
                                   std::string const& image_file,
                                   boost::shared_ptr<vw::camera::CameraModel> const&
                                   camera_model,
                                   std::string const& out_prefix);
  
  // Determine which camera images overlap by finding the lon-lat
  // bounding boxes of their footprints given the specified DEM, expand
  // them by a given percentage, and see if those intersect. A higher
  // percentage should be used when there is more uncertainty in input
  // camera poses. Specify as: 'dem.tif 15'.
  void build_overlap_list_based_on_dem
  /*        */ (std::string const& out_prefix,
                std::string const& dem_file,
                double pct_for_overlap,
                std::vector<std::string> const& image_files,
                std::vector<boost::shared_ptr<vw::camera::CameraModel>> const& camera_models,
                std::set<std::pair<std::string, std::string>> & overlap_list);

  /// Create the adjusted camera file name from the original camera filename,
  /// unless it is empty, and then use the image file name.
  /// - Convert dir1/image1.cub to out-prefix-image1.adjust
  std::string bundle_adjust_file_name(std::string const& prefix, std::string const& input_img,
                                      std::string const& input_cam);
  
  /// Ensure that no images, camera files, or adjustment names are duplicate.
  /// That will cause the output files to overwrite each other!
  void check_for_duplicates(std::vector<std::string> const& image_files,
                            std::vector<std::string> const& camera_files,
                            std::string const& out_prefix);

  // Make a list of all of the image pairs to find matches for
  void determine_image_pairs(// Inputs
                             int overlap_limit,
                             bool match_first_to_last,
                             std::vector<std::string> const& image_files,
                             // if having optional preexisting camera positions
                             bool got_est_cam_positions,
                             // Optional filter distance, set to -1 if not used
                             double position_filter_dist,
                             // Estimated camera positions, set to empty if missing
                             std::vector<vw::Vector3> const& estimated_camera_gcc,
                             // Optional preexisting list
                             bool have_overlap_list,
                             std::set<std::pair<std::string, std::string>> const& overlap_list,
                             // Output
                             std::vector<std::pair<int,int>> & all_pairs);

  /// Load a DEM from disk to use for interpolation.
  void create_interp_dem(std::string const& dem_file,
                         vw::cartography::GeoReference & dem_georef,
                         vw::ImageViewRef<vw::PixelMask<double>> & interp_dem);

  // Given an xyz point in ECEF coordinates, update its height above datum
  // by interpolating into a DEM. The user must check the return status.
  bool update_point_height_from_dem(vw::cartography::GeoReference const& dem_georef,
                                    vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                                    // Output
                                    vw::Vector3 & xyz);
  
  /// Try to update the elevation of a GCC coordinate from a DEM.
  /// - Returns false if the point falls outside the DEM or in a hole.
  bool update_point_height_from_dem(double* point, vw::cartography::GeoReference const& dem_georef,
                                    vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem);
  
  // Given a set of xyz points and a DEM to interpolate into,
  // create a vector of xyz points which are updated to be at the height given
  // by the DEM. This assumes that interp_dem is created outside of this
  // function with bilinear interpolation, via asp::create_interp_dem().
  // Invalid or uncomputable xyz are set to the zero vector.
  void update_point_height_from_dem(vw::ba::ControlNetwork const& cnet,
                                    std::set<int> const& outliers,
                                    vw::cartography::GeoReference const& dem_georef,
                                    vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                                    // Output
                                    std::vector<vw::Vector3> & dem_xyz_vec);
  
  // Shoot rays from all matching interest point. Intersect those with a DEM.
  // Find their average.
  void calc_avg_intersection_with_dem(vw::ba::ControlNetwork const& cnet,
                                      vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
                                      std::set<int> const& outliers,
                                      std::vector<boost::shared_ptr<vw::camera::CameraModel>> const&
                                      camera_models,
                                      vw::cartography::GeoReference const& dem_georef,
                                      vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                                      // Output
                                      std::vector<vw::Vector3> & dem_xyz_vec);
  

  // Flag outliers by reprojection error with input cameras. This assumes that
  // the input cameras are pretty accurate.
  void flag_initial_outliers(vw::ba::ControlNetwork const& cnet,
                             vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
                             std::vector<boost::shared_ptr<vw::camera::CameraModel>>
                             const& camera_models,
                             double max_init_reproj_error,
                             // Output
                             std::set<int> & outliers);
  
  // Manufacture a CSM state file from an adjust file
  std::string csmStateFile(std::string const& adjustFile);
}

#endif // __BUNDLE_ADJUST_UTILS_H__
