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

/// \file BundleAdjustUtils.cc
///

#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/ImageUtils.h>

#include <vw/Core/Log.h>
#include <vw/Camera/CameraModel.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Image/Interpolation.h>
#include <vw/Cartography/shapeFile.h>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ba;

namespace fs = boost::filesystem;

namespace asp {

// See the .h file for documentation
void camera_footprint(std::string const& dem_file,
                      std::string const& image_file,
                      vw::CamPtr  const& camera_model,
                      std::string const& out_prefix,
                      // Outputs
                      vw::geometry::dPoly & footprint,
                      vw::BBox2 & footprint_bbox) {
  
  vw_out() << "Computing ground footprint bounding box of: " + image_file << std::endl;

  // Initialize outputs
  footprint = vw::geometry::dPoly();
  footprint_bbox = vw::BBox2();

  std::string stem = fs::path(image_file).stem().string(); 
  std::string box_path = out_prefix + '-' + stem + "-bbox.txt";
  if (fs::exists(box_path)) {
    double min_x, min_y, max_x, max_y;
    std::ifstream ifs(box_path);
    if (ifs >> min_x >> min_y >> max_x >> max_y) {
      footprint_bbox.min() = vw::Vector2(min_x, min_y);
      footprint_bbox.max() = vw::Vector2(max_x, max_y);
      vw_out() << "Read cached ground footprint bbox from: " << box_path << ":\n" 
               << footprint_bbox << "\n";
      return;
    }
  }

  // Read the DEM and supporting data
  vw::cartography::GeoReference dem_georef;
  DiskImageView<float> dem_disk_image(dem_file);
  ImageViewRef<PixelMask<float>> dem;
  boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResourcePtr(dem_file));
  if (dem_rsrc->has_nodata_read())
    dem = create_mask(dem_disk_image, dem_rsrc->nodata_read());
  else
    dem = pixel_cast<PixelMask<float>>(dem_disk_image); // all pixels are valid
  
  bool has_georef = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!has_georef)
    vw_throw( ArgumentErr() << "There is no georeference information in: "
              << dem_file << ".\n" );

  try {
    DiskImageView<float> img(image_file);
    bool quick = false;     // Do a thorough job
    float mean_gsd = 0.0;
    int num_samples = 100; // should be enough
    std::vector<Vector3> coords;
    footprint_bbox 
      = vw::cartography::camera_bbox(dem, dem_georef, dem_georef,
                                     camera_model, img.cols(), img.rows(),
                                     mean_gsd, quick, &coords, num_samples);
   
   // Convert the coordinates from ECEF to points in the DEM projection 
   for (size_t i = 0;  i < coords.size(); i++) {
     coords[i] = dem_georef.datum().cartesian_to_geodetic(coords[i]);
     coords[i] = dem_georef.geodetic_to_point(coords[i]);
   }

   // Compute the convex hull of the projected coordinates, for easier
   // intersection operations later
   vw::geometry::convexHull(coords, footprint);
   
  } catch (std::exception const& e) {
    vw_throw( ArgumentErr() << e.what() << "\n"
              << "Failed to compute the footprint of camera image: " << image_file
              << " onto DEM: " << dem_file << ".\n");
  }

  vw_out() << "Writing: " << box_path << "\n";
  std::ofstream ofs(box_path.c_str());
  ofs.precision(17);
  ofs << footprint_bbox.min().x() << " " <<  footprint_bbox.min().y() << " "
      << footprint_bbox.max().x() << " " <<  footprint_bbox.max().y() << "\n";
  ofs.close();
}

// See the .h file for the documentation.
void buildOverlapList(std::string const& out_prefix, 
                      std::string const& dem_file, 
                      double pct_for_overlap,
                      int overlap_limit,
                      bool match_first_to_last,
                      std::vector<std::string> const& image_files,
                      std::vector<vw::CamPtr>  const& camera_models,
                      std::set<std::pair<std::string, std::string>> &
                      overlap_list) {

  // Wipe the output
  overlap_list.clear();
  
  // Sanity check
  if (image_files.size() != camera_models.size())
    vw_throw( ArgumentErr() << "Expecting as many images as cameras.\n");

  int num_images = image_files.size();
  std::vector<vw::BBox2> boxes(num_images);
  std::vector<vw::geometry::dPoly> footprints(num_images);
  for (int it = 0; it < num_images; it++) {
    // By this stage the camera bboxes are already computed and cached,
    // they just need to be loaded.
    asp::camera_footprint(dem_file, image_files[it], camera_models[it], out_prefix, 
                          footprints[it], boxes[it]);

    // Expand the box by the given factor
    expand_box_by_pct(boxes[it], pct_for_overlap);
  }

  // See which boxes overlap. While this is an O(N^2) computation, likely N is
  // at most a thousand or two, which should be manageable. A 2D tree of box
  // corners could be used, and two boxes would then overlap if corners from one
  // box are contained in a second box. That would be a O(N * log(N)) lookup. If
  // match_first_to_last is true, need to look past num_images, so wrap around
  // and look at earlier images.
  // TODO(oalexan1): Use a tree structure.
  for (int it1 = 0; it1 < num_images; it1++) {
    int num_added = 0;
    
    int end2 = num_images;
    if (match_first_to_last)
      end2 = num_images + overlap_limit;
      
    for (int it2 = it1 + 1; it2 < end2; it2++) {
      
      // Wrap around if needed
      int local_it2 = it2 % num_images;
      
      BBox2 box = boxes[it1]; // deep copy
      box.crop(boxes[local_it2]);
      if (!box.empty() && num_added < overlap_limit) {
        auto pair1 = std::make_pair(image_files[it1], image_files[local_it2]);
        auto pair2 = std::make_pair(image_files[local_it2], image_files[it1]);
        
        if (overlap_list.find(pair1) != overlap_list.end() ||
            overlap_list.find(pair2) != overlap_list.end())
          continue; // already added
        
        // Can't have matches to itself
        if (it1 == local_it2) 
          continue;
          
        // Add the pair with first index less than the second
        std::pair<std::string, std::string> pair;
        if (it1 < local_it2)
          pair = std::make_pair(image_files[it1], image_files[local_it2]);
        else
          pair = std::make_pair(image_files[local_it2], image_files[it1]);  
        overlap_list.insert(pair);
        
        num_added++;
      }
      
      // Stop when added enough
      if (num_added >= overlap_limit) 
        break;
    }
  }

  return;
}

/// Ensure that the basename (without extension) of all images, camera files, or
/// adjustment names are different. Later these will be used for match files,
/// and we want match files corresponding to different images to be different.
void check_for_duplicates(std::vector<std::string> const& image_files,
                               std::vector<std::string> const& camera_files,
                               std::string const& out_prefix) {

  if (image_files.size() != camera_files.size())
    vw_throw(vw::ArgumentErr() << "Expecting as many images as cameras.\n");
  
  std::set<std::string> img_set, cam_set, adj_set;
  for (size_t i = 0; i < camera_files.size(); i++) {
    std::string img = vw::ip::strip_path("", image_files[i]);
    std::string cam = vw::ip::strip_path("", camera_files[i]);
    std::string ba_name = asp::bundle_adjust_file_name(out_prefix, 
                                                       image_files[i], camera_files[i]);
    std::string adj_base = vw::ip::strip_path(out_prefix, ba_name);

    if (img_set.find(img) != img_set.end()) 
      vw_throw(vw::ArgumentErr() << "Found duplicate image: " << img << "\n");
    
    if (cam != "" && cam_set.find(cam) != cam_set.end()) 
      vw_throw(vw::ArgumentErr() << "Found duplicate camera: " << cam << "\n");

    if (adj_set.find(adj_base) != adj_set.end()) 
      vw_throw(vw::ArgumentErr() << "Found duplicate adjustment name: " << adj_base << "\n");

    img_set.insert(img);
    if (cam != "") 
      cam_set.insert(cam); // camera file can be empty
    adj_set.insert(adj_base);
    
  }
}

// Given an xyz point in ECEF coordinates, update its height above datum by
// interpolating into a DEM. The input must already be prepared for
// interpolation. The user must check the return status.
bool update_point_height_from_dem(vw::cartography::GeoReference const& dem_georef,
                                  vw::ImageViewRef<PixelMask<double>> const& interp_dem,
                                  // Output
                                  vw::Vector3 & xyz) {

  // Points at planet center are outliers
  if (xyz == Vector3(0, 0, 0))
    return false;
  
  Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);
  Vector2 ll  = subvector(llh, 0, 2);
  Vector2 pix = dem_georef.lonlat_to_pixel(ll);
  if (!interp_dem.pixel_in_bounds(pix)) {
    xyz = Vector3(0, 0, 0);
    return false;
  }

  PixelMask<double> height = interp_dem(pix[0], pix[1]);
  if (!is_valid(height)) {
    xyz = Vector3(0, 0, 0);
    return false;
  }
  
  llh[2] = height.child();

  // NaN check
  if (llh[2] != llh[2])  {
    xyz = Vector3(0, 0, 0);
    return false;
  }
  
  // Overwrite the input
  xyz = dem_georef.datum().geodetic_to_cartesian(llh);

  return true;
}

// Shoot rays from all matching interest points. Intersect those with a DEM. Find
// their average. Project it vertically onto the DEM. Invalid or uncomputable
// xyz are set to the zero vector.
// TODO(oalexan1): This code can be slow, but using multiple threads makes it
// even slower, likely because of having to share the masked_dem image. To speed
// it up one could break the loop over features into several parts. Each would
// load and have its own masked_dem image. Even then there may be some global
// cache for all images, which would slow things down.
// There is a function with the same name and logic for rig_calibrator.
void updateTriPtsFromDem(vw::ba::ControlNetwork const& cnet,
                         std::set<int> const& outliers,
                         std::vector<vw::CamPtr> const& camera_models,
                         vw::cartography::GeoReference const& dem_georef,
                         vw::ImageViewRef<vw::PixelMask<double>> const& masked_dem,
                         // Output
                         std::vector<vw::Vector3> & dem_xyz_vec) {

  // Put this note as this part can take a long time
  vw::vw_out() << "Updating triangulated points with DEM height.\n";
  
  int num_tri_points = cnet.size();
  dem_xyz_vec.resize(num_tri_points, vw::Vector3(0, 0, 0));

  // Project vertically onto the DEM. This needs interpolation into the DEM
  vw::PixelMask<double> invalid_val;
  vw::ImageViewRef<vw::PixelMask<double>> interp_dem
   = vw::interpolate(masked_dem, vw::BilinearInterpolation(), 
                     vw::ValueEdgeExtension<vw::PixelMask<float>>(invalid_val));

  // Prepare for measuring progress and elapsed time
  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / std::max(1, num_tri_points);
  tpc.report_progress(0);
  vw::Stopwatch sw;
  sw.start();

  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    
    tpc.report_incremental_progress(inc_amount);

    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
      continue; // GCP do not get modified
    
    if (outliers.find(ipt) != outliers.end())
      continue; // Skip outliers
    
    // The initial triangulated point
    Vector3 xyz_guess = cnet[ipt].position();

    // Points at planet center are outliers. This check is likely redundant,
    // but good to have.
    if (xyz_guess == Vector3(0, 0, 0))
      continue;

    Vector3 accumulated_xyz(0,0,0);
    int num_intersections = 0;
    
    for (size_t m = 0; m < cnet[ipt].size(); m++) {

      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = cnet[ipt][m].position();
        
      // Intersect ray with DEM
      bool treat_nodata_as_zero = false;
      bool has_intersection = false;
      double height_error_tol = 0.001; // 1 mm should be enough
      double max_abs_tol      = 1e-14; // abs cost fun change b/w iterations
      double max_rel_tol      = 1e-14;
      int num_max_iter        = 25;   // Using many iterations can be very slow
      int icam = cnet[ipt][m].image_id();
      Vector3 dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
        (camera_models[icam]->camera_center(observation),
         camera_models[icam]->pixel_to_vector(observation),
         vw::pixel_cast<vw::PixelMask<float>>(masked_dem), 
         dem_georef, treat_nodata_as_zero, has_intersection,
         height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);

      if (!has_intersection) 
        continue;

      accumulated_xyz += dem_xyz;
      num_intersections++;
    }

    // Average the successful intersections.
    if (num_intersections > 0) 
      dem_xyz_vec[ipt] = accumulated_xyz / double(num_intersections);
    else
      dem_xyz_vec[ipt] = Vector3();

    if (dem_xyz_vec[ipt] == Vector3())
      continue; // Skip invalid points
      
    Vector3 observation = dem_xyz_vec[ipt];
    if (update_point_height_from_dem(dem_georef, interp_dem, observation))
      dem_xyz_vec[ipt] = observation;
    
  } // end iterating over points
  
  tpc.report_finished();

  sw.stop();
  vw::vw_out() << "Elapsed time in updating triangulated points from DEM: "
               << sw.elapsed_seconds() << " seconds.\n";

  return;
}

// Flag outliers by reprojection error with input cameras. This assumes that
// the input cameras are pretty accurate.
void flag_initial_outliers(vw::ba::ControlNetwork const& cnet,
                           asp::CRN const& crn,
                           std::vector<vw::CamPtr> const& camera_models,
                           double max_init_reproj_error,
                           // Output
                           std::set<int> & outliers) {
  // Wipe the output
  outliers.clear();

  int num_cameras = camera_models.size();
  int num_tri_points = cnet.size();

  for (int icam = 0; icam < (int)crn.size(); icam++) {

    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {
      
      // The index of the triangulated point
      int ipt = (**fiter).m_point_id;
      
      VW_ASSERT(icam < num_cameras, ArgumentErr() 
                  << "Out of bounds in the number of cameras.");
      VW_ASSERT(ipt < num_tri_points, ArgumentErr() 
                  << "Out of bounds in the number of points.");

      if (outliers.find(ipt) != outliers.end()) {
        // Is an outlier
        continue;
      }
      
      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;

      Vector3 const& tri_point = cnet[ipt].position(); // alias
      
      // Flag outliers produced when building or triangulating the control network
      if (cnet[ipt].ignore()) {
        outliers.insert(ipt);
        continue;
      }

      if (tri_point == Vector3(0, 0, 0)) {
        // Points at planet center are outliers
        outliers.insert(ipt);
        continue;
      }
      
      vw::Vector2 pix;
      try {
        pix = camera_models[icam]->point_to_pixel(tri_point);
        bool is_good = (norm_2(pix - observation) <= max_init_reproj_error);
      
        if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
          is_good = true; // GCP are not filtered with max_init_reproj_error

        if (!is_good) { // this checks for NaN too
          outliers.insert(ipt);
          continue;
        }
      } catch (...) {
        outliers.insert(ipt);
        continue;
      }
    }
    
  } // end iterating over cameras

  return;
}

// Manufacture a CSM state file from an adjust file
std::string csmStateFile(std::string const& adjustFile) {

  std::string csmFile = adjustFile;
  
  // If the suffix we want to add is already present, remove it first
  std::string suff = ".adjusted_state";
  auto it = csmFile.find(suff);
  if (it != std::string::npos)
    csmFile.replace(it, suff.size(), "");
  
  csmFile = boost::filesystem::path(csmFile).replace_extension(suff + ".json").string();

  return csmFile;
}

// Manufacture an RPC state file from an adjust file. Use the .xml extension
// for the output file.
std::string rpcAdjustedFile(std::string const& adjustFile) {

  std::string rpcFile = adjustFile;
  
  // If the suffix we want to add is already present, remove it first
  std::string suff = ".adjusted_rpc";
  auto it = rpcFile.find(suff);
  if (it != std::string::npos)
    rpcFile.replace(it, suff.size(), "");
  
  rpcFile = boost::filesystem::path(rpcFile).replace_extension(suff + ".xml").string();
  return rpcFile;
}

// Put the triangulated points in a vector. Update the cnet from the DEM,
// if we have one.
void formTriVec(std::vector<vw::Vector3> const& dem_xyz_vec,
                bool have_dem,
                // Outputs
                vw::ba::ControlNetwork & cnet,
                std::vector<double>    & orig_tri_points_vec,
                std::vector<double>    & tri_points_vec) {

  int num_tri_points = cnet.size();
  if (num_tri_points == 0)
    vw::vw_throw(ArgumentErr() << "No triangulated ground points were found.\n"); 

  orig_tri_points_vec.resize(num_tri_points*NUM_XYZ_PARAMS, 0.0);
  tri_points_vec.resize(num_tri_points*NUM_XYZ_PARAMS, 0.0);

  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    // We overwrite the triangulated point when we have an input DEM.
    // It is instructive to examine the pointmap residual file to see
    // what effect that has on residuals.  This point will likely try
    // to move back somewhat to its triangulated position during
    // optimization, depending on the strength of the weight which
    // tries to keep it back in place.
    Vector3 tri_point = cnet[ipt].position();
    
    // The original triangulated point, before the override or optimization
    for (int q = 0; q < NUM_XYZ_PARAMS; q++)
      orig_tri_points_vec[ipt*NUM_XYZ_PARAMS + q] = tri_point[q];
    
    bool is_gcp = (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint);

    if (have_dem && dem_xyz_vec.at(ipt) != Vector3(0, 0, 0) && !is_gcp) {
      tri_point = dem_xyz_vec.at(ipt);

      // Update in the cnet too
      cnet[ipt].set_position(Vector3(tri_point[0], tri_point[1], tri_point[2]));
      
      // Ensure we can track it later
      cnet[ipt].set_type(vw::ba::ControlPoint::PointFromDem); 
    }
    
    for (int q = 0; q < NUM_XYZ_PARAMS; q++)
      tri_points_vec[ipt*NUM_XYZ_PARAMS + q] = tri_point[q];
  }
  return;
}

// A function to do a moving average. The input vector can have nan where there 
// are no values. Have an option to to do this average only if needed to fill in.
// TODO(oalexan1): If the logic in residualsPerRow() and supporting functionality
// here is not useful, it may need to be wiped.
void movingAverage(std::vector<double> & vec, int window_size, bool fill_only,
                   bool & changed) {
  
  changed = false;
  int n = vec.size();
  
  // Window must be odd and positive
  if (window_size <= 0 || window_size % 2 == 0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "Expecting a positive odd number for the moving average window size.\n");

  int half_len = window_size / 2;
  
  // Make a copy of the input
  std::vector<double> vec_copy = vec; // deep copy
  
  double nan = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < n; i++) {
    
    if (fill_only && !std::isnan(vec[i]))
      continue; // no need to change this value
      
    changed = true;
      
    // Iterate over the window
    double sum = 0.0, count = 0.0;
    for (int win = -half_len; win <= half_len; win++) {
      int ind = i + win;
      if (ind < 0 || ind >= n)
        continue; // out of bounds
      if (std::isnan(vec_copy[ind]))
        continue; // skip nan
      sum += vec_copy[ind];
      count++;
    }
    if (count > 0)
      vec[i] = sum / count;
    else
      vec[i] = nan;
  }
  
  return;
}

// A function to strip all leading nan from vector. Do it in place.
void stripLeadingNan(std::vector<double> & vec) {
  
  int n = vec.size();
  int first_good = 0;
  for (int i = 0; i < n; i++) {
    if (!std::isnan(vec[i])) {
      first_good = i;
      break;
    }
  }
  
  if (first_good > 0) {
    // Shift the vector
    for (int i = first_good; i < n; i++)
      vec[i - first_good] = vec[i];
    // Resize
    vec.resize(n - first_good);
  }
  
  return;
}

// Strip trailing nan from vector. Do it in place.
void stripTrailingNan(std::vector<double> & vec) {
  
  int n = vec.size();
  int last_good = n - 1;
  for (int i = n - 1; i >= 0; i--) {
    if (!std::isnan(vec[i])) {
      last_good = i;
      break;
    }
  }
  
  if (last_good < n - 1) {
    // Resize
    vec.resize(last_good + 1);
  }
  
  return;
}

// Average all y pixel residuals per row then fill in from neighbors. This is
// useful for producing a jitter residual per image row, from which one may try
// to study its power spectrum and dominant frequencies.
void residualsPerRow(vw::ba::ControlNetwork const& cnet,
                     asp::CRN const& crn,
                     std::set<int> const& outliers,
                     std::vector<std::string> const& image_files,
                     std::vector<vw::CamPtr> const& camera_models,
                     // Output
                     std::vector<std::vector<double>> & residuals) {

  int numImages = image_files.size();

  // Sanity check
  if ((int)camera_models.size() != numImages || (int)crn.size() != numImages)
    vw_throw(ArgumentErr() 
            << "Number of imgages, of cameras, and control network sizes do not match.\n");

  // Wipe the output
  residuals.clear();
  residuals.resize(numImages);
    
  for (int icam = 0; icam < (int)crn.size(); icam++) {
    
    vw::Vector2 dims = vw::file_image_size(image_files[icam]);
    int numLines = dims[1];
    residuals[icam].resize(numLines, 0.0);
    
    std::vector<double> count(numLines, 0.0);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    for (auto const& feature_ptr: crn[icam]) {
        
      // The index of the 3D point
      int ipt = feature_ptr->m_point_id;

      if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
        continue; // Skip GCP
      
      if (outliers.find(ipt) != outliers.end())
        continue; // Skip outliers
        
      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = feature_ptr->m_location;
        
      // Ideally this point projects back to the pixel observation, so use the
      // triangulated position as initial guess.
      Vector3 xyz = cnet[ipt].position();

      if (xyz == Vector3(0, 0, 0))
        continue; // Skip outliers

      // Project into the camera
      vw::Vector2 pix = camera_models[icam]->point_to_pixel(xyz);
      
      // Image row
      int row = round(observation[1]);
      if (row < 0 || row > numLines - 1)
        continue; // out of bounds
      
      // Accumulate the residual
      residuals[icam][row] += (pix[1] - observation[1]);
      count[row] += 1.0;
    } // end loop over features
    
    // Average the residuals. Put naN where there is no data
    for (int row = 0; row < numLines; row++) {
      if (count[row] > 0)
        residuals[icam][row] /= count[row];
      else
        residuals[icam][row] = nan;
    }
    
    // Do a moving average with a length of 11
    bool fill_only = false;
    bool changed = false;
    int window_size = 11;
    movingAverage(residuals[icam], window_size, fill_only, changed);
    
    // Strip leading and trailing nan. There can be plenty because of
    // lack of features there.
    stripLeadingNan(residuals[icam]);
    stripTrailingNan(residuals[icam]);
    
    // Now continue doing this only to fill in missing values
    fill_only = true;
    int attempts = 0;
    while (changed) {
      movingAverage(residuals[icam], window_size, fill_only, changed);
      attempts++;
      // Throw an error after 10 attempts
      if (attempts > 10) {
        vw::vw_out() << "No luck after attempts: " << icam << " " << attempts << "\n";
        break;
      }
    }
    
  } // end loop over cameras

  return;     
} // end function residualsPerRow

// Read sigmas for some pairs of input images. The file is in format: 
// image1 image2 sigma, with space as separator. Any order of image1 and image2
// is supported. 
void readMatchPairSigmas(std::string const& sigmaFilename,
                         std::vector<std::string> const& imageFiles,
                         MatchSigmasMap & matchSigmas) {

  // Wipe the output
  matchSigmas.clear();

  // Form image file to index map
  std::map<std::string, int> img2index;
  for (size_t i = 0; i < imageFiles.size(); i++)
    img2index[imageFiles[i]] = i;
  
  // Open the file for reading    
  std::ifstream infile(sigmaFilename.c_str());
  if (!infile.is_open())
    vw::vw_throw(vw::ArgumentErr() << "Could not open file: " << sigmaFilename << "\n");
  
  std::string line;
  int line_num = 0;
  while (std::getline(infile, line)) {
      line_num++;

      // Skip lines that are empty or contain only whitespace
      if (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos)
        continue;

      // Try to parse the three elements: image, image, double
      std::istringstream iss(line);
      std::string img1, img2;
      double sigma;
      if (!(iss >> img1 >> img2 >> sigma))
          vw::vw_throw(vw::ArgumentErr() << "Could not parse line " << line_num
                        << " in file: " << sigmaFilename << "\n");
          
      // Check that both images are known
      auto it1 = img2index.find(img1);
      if (it1 == img2index.end())
        vw::vw_throw(vw::ArgumentErr() << "Unknown image: " << img1 
                      << " in line " << line_num << " of file: " << sigmaFilename << "\n");
      int ind1 = it1->second;
      auto it2 = img2index.find(img2);
      if (it2 == img2index.end())
        vw::vw_throw(vw::ArgumentErr() << "Unknown image: " << img2 
                      << " in line " << line_num << " of file: " << sigmaFilename << "\n");
      int ind2 = it2->second;
      
      // Indices must not be same
      if (ind1 == ind2)
        vw::vw_throw(vw::ArgumentErr() << "Same image listed twice in line " 
                      << line_num << " of file: " << sigmaFilename << "\n");
        
      // Form the key as pair. Allow also other order of images
      auto key1 = std::make_pair(ind1, ind2);
      auto key2 = std::make_pair(ind2, ind1);
      
      // These must not exist
      if (matchSigmas.find(key1) != matchSigmas.end() ||
          matchSigmas.find(key2) != matchSigmas.end())
        vw::vw_throw(vw::ArgumentErr() << "Duplicate image pair in line " 
                      << line_num << " of file: " << sigmaFilename << "\n");
        
      matchSigmas[key1] = sigma;
      matchSigmas[key2] = sigma;
      
      std::cout << "--added ind1, ind2, sigma: " << ind1 << " " << ind2 
                << " " << sigma << "\n";
  }
  
  return;
}

} // end namespace asp
