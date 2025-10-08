// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

// \file BundleAdjustOptions.cc

// Options for bundle adjustment.
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/ImageNormalization.h>

#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Geometry/dPoly.h>
#include <boost/filesystem.hpp>
#include <string>

namespace fs = boost::filesystem;

namespace asp {

void BaOptions::copy_to_asp_settings() const {
  asp::stereo_settings().ip_detect_method        = ip_detect_method;
  asp::stereo_settings().epipolar_threshold      = epipolar_threshold;
  asp::stereo_settings().ip_inlier_factor        = ip_inlier_factor;
  asp::stereo_settings().ip_uniqueness_thresh    = ip_uniqueness_thresh;
  asp::stereo_settings().num_scales              = num_scales;
  asp::stereo_settings().nodata_value            = nodata_value;
  asp::stereo_settings().aster_use_csm           = aster_use_csm;
  asp::stereo_settings().ip_per_tile             = ip_per_tile;
  asp::stereo_settings().ip_per_image            = ip_per_image;
  asp::stereo_settings().matches_per_tile        = matches_per_tile;
  asp::stereo_settings().matches_per_tile_params = matches_per_tile_params;
  asp::stereo_settings().no_datum                = no_datum;
  asp::stereo_settings().use_least_squares       = false; // never true with ba

  // Note that by default rough homography and tri filtering are disabled
  // as input cameras may be too inaccurate for that.
  asp::stereo_settings().skip_rough_homography      = !enable_rough_homography;
  asp::stereo_settings().disable_tri_filtering      = !enable_tri_filtering;

  // Do not pass this as it will results in filtering by elevation and lonlat
  // with unoptimized cameras. We will do that filtering with optimized
  // cameras later.
  //asp::stereo_settings().elevation_limit            = elevation_limit;
  //asp::stereo_settings().lon_lat_limit              = lon_lat_limit;

  asp::stereo_settings().individually_normalize     = individually_normalize;
  asp::stereo_settings().force_reuse_match_files    = force_reuse_match_files;
  asp::stereo_settings().min_triangulation_angle    = min_triangulation_angle;
  asp::stereo_settings().ip_triangulation_max_error = ip_triangulation_max_error;
  asp::stereo_settings().ip_num_ransac_iterations   = ip_num_ransac_iterations;
  asp::stereo_settings().ip_edge_buffer_percent     = ip_edge_buffer_percent;
  asp::stereo_settings().ip_debug_images            = ip_debug_images;
  asp::stereo_settings().ip_normalize_tiles         = ip_normalize_tiles;
  asp::stereo_settings().flann_method               = flann_method;
  asp::stereo_settings().propagate_errors           = propagate_errors;
  asp::stereo_settings().ip_nodata_radius           = ip_nodata_radius;
  
  // The setting below is not used, but populate it for completeness
  asp::stereo_settings().horizontal_stddev          = vw::Vector2(horizontal_stddev,
                                                                  horizontal_stddev);
  asp::stereo_settings().validate();
}

// Read the mapprojected data, if having the option --mapprojected-data.
void setupMapprojectedData(asp::BaOptions & opt,
                           bool need_no_matches,
                           std::vector<std::string> & map_files,
                           std::string & mapproj_dem) {

  // Clear the outputs
  map_files.clear();
  mapproj_dem.clear();
  
  if (!need_no_matches) {
    if (!opt.mapprojected_data_list.empty()) {
      asp::read_list(opt.mapprojected_data_list, map_files);
      opt.mapprojected_data = "non-empty"; // put a token value, to make it non-empty
    } else if (opt.mapprojected_data != "") {
      std::istringstream is(opt.mapprojected_data);
      std::string file;
      while (is >> file)
        map_files.push_back(file);
    }
    
    if (!opt.mapprojected_data.empty()) {
      // Read the mapproj DEM from the first image geoheader or as the last element
      // in the list.
      if (opt.image_files.size() + 1 != map_files.size() &&
          opt.image_files.size()     != map_files.size()) 
        vw::vw_throw(vw::ArgumentErr() << "Error: Expecting as many mapprojected images as "
          << "cameras and, potentially, a DEM at the end of the mapprojected list.\n");

      if (opt.image_files.size() == map_files.size()) {
        // Read the DEM from first image header  
        std::string adj_key, img_file_key, cam_type_key, cam_file_key, dem_file_key;
        std::string adj_prefix_raw, image_file_raw, cam_type, cam_file_raw, dem_file;
        asp::read_mapproj_header(map_files.at(0),
                                  adj_key, img_file_key, cam_type_key, 
                                  cam_file_key, dem_file_key,
                                  adj_prefix_raw, image_file_raw, cam_type,
                                  cam_file_raw, 
                                  mapproj_dem); // read here
      } else if (opt.image_files.size() + 1 == map_files.size()) {
        // Pull out the dem from the list
        mapproj_dem = map_files.back();
        map_files.erase(map_files.end() - 1);
      }
    }
    
    // The mapproj DEM must be non-empty
    if (!map_files.empty() && mapproj_dem.empty())
      vw::vw_throw(vw::ArgumentErr()
                << "Interest point matching with mapprojected images requires a DEM. "
                << "None found either on the command line or in a geoheader of any "
                << "mapprojected image.\n");
  }
  
  if (!map_files.empty()) {
    if (!opt.input_prefix.empty() || !opt.initial_transform_file.empty() ||
        need_no_matches)
      vw::vw_throw(vw::ArgumentErr()
                << "Cannot use mapprojected data with initial adjustments, "
                << "an initial transform, or ISIS cnet or NVM input.\n");
  }
}

/// Looks in the input camera position file to generate a GCC position for
/// each input camera.
/// - If no match is found, the coordinate is (0,0,0)
int loadEstimCameraPositions(asp::BaOptions &opt,
                             std::vector<vw::Vector3> & estimated_camera_gcc) {
  estimated_camera_gcc.clear();
  if (opt.camera_position_file == "")
    return 0;

  // Read the input csv file
  asp::CsvConv conv;
  conv.parse_csv_format(opt.csv_format_str, opt.csv_srs);
  std::list<asp::CsvConv::CsvRecord> pos_records;
  typedef std::list<asp::CsvConv::CsvRecord>::const_iterator RecordIter;
  conv.read_csv_file(opt.camera_position_file, pos_records);

  // Set up a GeoReference object using the datum
  vw::cartography::GeoReference geo;
  geo.set_datum(opt.datum); // We checked for a datum earlier
  conv.parse_georef(geo);

  // For each input camera, find the matching position in the record list
  const int num_cameras = opt.image_files.size();
  estimated_camera_gcc.resize(num_cameras);

  const RecordIter no_match = pos_records.end();
  int num_matches_found = 0;
  for (int i=0; i<num_cameras; i++) {

    // Search for this image file in the records
    std::string file_name = opt.image_files[i];
    RecordIter iter;
    for (iter=pos_records.begin(); iter!=pos_records.end(); iter++) {
      // Match if the string in the file is contained in the input image string.
      // - May need to play around with this in the future!
      std::string field = iter->file;
      if (file_name.find(field) != std::string::npos) {
        estimated_camera_gcc[i] = conv.csv_to_cartesian(*iter, geo);
        break; // Match found, stop the iterator here.
      }
    }
    if (iter == no_match) {
      vw::vw_out(vw::WarningMessage) << "Camera file " << file_name
         << " not found in camera position file.\n";
      estimated_camera_gcc[i] = vw::Vector3(0,0,0);
    } else {
      num_matches_found++;
    }
  } // End loop to find position record for each camera

  return num_matches_found;
}

// Compute statistics for the designated range of images (or mapprojected
// images), and perhaps the footprints. Or, compute the ip per image (before
// matching). These distinct operations use much shared logic, so are put in the
// same function. In parallel_bundle_adjust this function is called separately
// for different ranges.
void computeStatsOrIp(asp::BaOptions const& opt, 
                      std::vector<std::string> const& files_for_stats,
                      std::string const& dem_file_for_overlap, 
                      std::string const& normalization_bounds_file, 
                      bool calcIp) {

  using namespace vw;
  
  int num_images = files_for_stats.size();
  if (num_images != opt.image_files.size())
    vw_throw(ArgumentErr() << "Book-keeping error in number of images.\n");
  
  std::map<std::string, vw::Vector2> bounds_map;
  if (calcIp)
    asp::readNormalizationBounds(normalization_bounds_file, files_for_stats, bounds_map);

  // Assign the images which this job should compute statistics for.
  std::vector<size_t> image_stats_indices;
  for (size_t i = opt.job_id; i < num_images; i += opt.num_parallel_jobs)
    image_stats_indices.push_back(i);

  for (size_t i = 0; i < image_stats_indices.size(); i++) {

    size_t index = image_stats_indices[i];

    // The stats need to be computed for the mapprojected image, if provided
    std::string image_path = files_for_stats[index];

    // Call a bunch of stuff to get the nodata value
    boost::shared_ptr<DiskImageResource> rsrc(vw::DiskImageResourcePtr(image_path));
    float nodata = -std::numeric_limits<float>::max();
    float dummy_nodata = nodata;
    asp::get_nodata_values(rsrc, rsrc, nodata, dummy_nodata);

    // Set up the image view. If the user provided a custom no-data value,
    // values no more than that are masked.
    float user_nodata = asp::stereo_settings().nodata_value;
    DiskImageView<float> image_view(rsrc);
    ImageViewRef<PixelMask<float>> masked_image;
    if (!std::isnan(user_nodata))
      masked_image = create_mask_less_or_equal(image_view, user_nodata);
    else
      masked_image = create_mask(image_view, nodata);

    // Use caching function call to compute the image statistics.
    if (!calcIp)
      asp::gather_stats(masked_image, image_path, opt.out_prefix, image_path,
                        asp::stereo_settings().force_reuse_match_files);

    // Compute and cache the camera footprint polygon and its bbox
    if (opt.auto_overlap_params != "" && !calcIp) {
      vw::geometry::dPoly footprint;
      vw::BBox2 footprint_bbox;
      asp::camera_footprint(dem_file_for_overlap,
                            opt.image_files[index], // use the original image
                            opt.camera_models[index],
                            opt.out_prefix,
                            footprint, footprint_bbox); // outputs
    }
    
    if (calcIp) {
      // This closely resembles the logic in normalize_images() and
      // ip_matching(), but done per image.
      if (asp::openCvDetectMethod()) {
        vw::Vector2 bounds = bounds_map[image_path];
        masked_image = normalize(masked_image, bounds[0], bounds[1], 0.0, 1.0);
      }
      
      // Detect ip and write them to file. Skip if cached ip newer than image exist.
      vw::ip::InterestPointList ip;
      std::string vwip_file = vw::ip::ip_filename(opt.out_prefix, image_path);
      bool use_cached_ip = false;
      if (fs::exists(vwip_file) && first_is_newer(vwip_file, image_path))
        use_cached_ip = true;
      asp::detect_ip(ip, 
                     apply_mask(masked_image, nodata),
                     asp::stereo_settings().ip_per_tile,
                     vwip_file, nodata, use_cached_ip);
      
    }  
  }
  
  return;
} // end function computeStatsOrIp

} // end namespace asp
