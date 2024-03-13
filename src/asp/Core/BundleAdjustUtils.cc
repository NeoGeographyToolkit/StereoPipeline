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
#include <vw/InterestPoint/Matcher.h>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ba;

namespace fs = boost::filesystem;

namespace asp {

// Read adjustments
// TODO(oalexan1): Integrate with the VW function for that
void read_adjustments(std::string const& filename,
                           vw::Vector3      & position_correction,
                           Quat             & pose_correction,
                           Vector2          & pixel_offset,
                           double           & scale) {

  // Initialize the outputs
  pixel_offset = Vector2();
  scale = 1.0;
  
  Vector3 pos;
  Vector4 q_buf;
  std::ifstream istr(filename.c_str());

  // Read the adjustments
  if (!(istr >> pos[0] >> pos[1] >> pos[2] 
             >> q_buf[0] >> q_buf[1] >> q_buf[2] >> q_buf[3])) 
    vw::vw_throw(vw::ArgumentErr() << "Failed to read adjustments from: " 
                 << filename << ".\n");

  // The adjustments may have an offset and a scale
  double a, b, c;
  if (istr >> a >> b >> c){
    pixel_offset = Vector2(a, b);
    scale = c;
  }
  
  position_correction = pos;
  pose_correction = Quat(q_buf);
}

void write_adjustments(std::string const& filename,
                       Vector3 const& position_correction,
                       Quat const& pose_correction) {
  std::ofstream ostr(filename.c_str());
  ostr.precision(17);
  ostr << position_correction[0] << " " << position_correction[1] << " "
       << position_correction[2] << "\n";
  ostr << pose_correction.w() << " " << pose_correction.x() << " "
       << pose_correction.y() << " " << pose_correction.z() << " " << "\n";
  ostr.close();
}

void compute_stereo_residuals(std::vector<vw::CamPtr> const& camera_models,
                                   ControlNetwork const& cnet) {

  // Compute pre-adjustment residuals and convert to bundles
  int n = 0;
  double error_sum = 0;
  double min_error = ScalarTypeLimits<double>::highest();
  double max_error = ScalarTypeLimits<double>::lowest();
  for (size_t i = 0; i < cnet.size(); ++i) {
    for (size_t j = 0; j+1 < cnet[i].size(); ++j) {
      ++n;
      size_t cam1 = cnet[i][j].image_id();
      size_t cam2 = cnet[i][j+1].image_id();
      Vector2 pix1 = cnet[i][j].position();
      Vector2 pix2 = cnet[i][j+1].position();

      StereoModel sm(camera_models[cam1].get(), camera_models[cam2].get());
      double error;
      sm(pix1,pix2,error);
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
  }
  vw_out() << "\nStereo intersection residuals -- min: " << min_error
           << "  max: " << max_error << "  average: " << (error_sum/n) << "\n";
}

// See the .h file for documentation
vw::BBox2 camera_bbox_with_cache(std::string const& dem_file,
                                 std::string const& image_file,
                                 vw::CamPtr  const& camera_model,
                                 std::string const& out_prefix) {
  
  namespace fs = boost::filesystem;

  vw_out() << "Computing ground footprint bounding box of: " + image_file << std::endl;

  vw::BBox2 box;
  
  std::string box_path = out_prefix + '-' + fs::path(image_file).stem().string() + "-bbox.txt";
  if (fs::exists(box_path)) {
    double min_x, min_y, max_x, max_y;
    std::ifstream ifs(box_path);
    if (ifs >> min_x >> min_y >> max_x >> max_y) {
      box.min() = vw::Vector2(min_x, min_y);
      box.max() = vw::Vector2(max_x, max_y);
      vw_out() << "Read cached ground footprint bbox from: " << box_path << ":\n" << box << "\n";
      return box;
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
    float auto_res = -1.0;  // Will be updated
    bool quick = false;     // Do a thorough job
    box = vw::cartography::camera_bbox(dem, dem_georef, dem_georef,
                                       camera_model, img.cols(), img.rows(),
                                       auto_res, quick);
  } catch (std::exception const& e) {
    vw_throw( ArgumentErr() << e.what() << "\n"
              << "Failed to compute the footprint of camera image: " << image_file
              << " onto DEM: " << dem_file << ".\n");
  }

  vw_out() << "Writing: " << box_path << "\n";
  std::ofstream ofs(box_path.c_str());
  ofs.precision(17);
  ofs << box.min().x() << " " <<  box.min().y() << " "
      << box.max().x() << " " <<  box.max().y() << "\n";
  ofs.close();
  
  return box;
}

// Expand a box by a given percentage (typically pct is between 0 and 100)
void expand_box_by_pct(vw::BBox2 & box, double pct) {
  
  // Check the pct is non-negative
  if (pct < 0.0) 
    vw_throw(ArgumentErr() << "Invalid percentage when expanding a box: " 
              << pct << ".\n");
    
  double factor = pct / 100.0;
  double half_extra_x = 0.5 * box.width()  * factor;
  double half_extra_y = 0.5 * box.height() * factor;
  box.min() -= Vector2(half_extra_x, half_extra_y);
  box.max() += Vector2(half_extra_x, half_extra_y);
}

// See the .h file for the documentation.
void build_overlap_list_based_on_dem(std::string const& out_prefix, 
                                     std::string const& dem_file, 
                                     double pct_for_overlap,
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
  for (int it = 0; it < num_images; it++) {
    // By this stage the camera bboxes are already computed and cached,
    // they just need to be loaded.
    boxes[it] = asp::camera_bbox_with_cache(dem_file, image_files[it], camera_models[it],  
                                            out_prefix);

    // Expand the box by the given factor
    expand_box_by_pct(boxes[it], pct_for_overlap);
  }

  // See which boxes overlap. While this is an O(N^2) computation,
  // likely N is at most a thousand or two, which should be
  // manageable. A 2D tree of box corners could be used, and two boxes
  // would then overlap if corners from one box are contained in a
  // second box. That would be a O(N * log(N)) lookup.
  // TODO(oalexan1): Use a tree.
  for (int it1 = 0; it1 < num_images; it1++) {
    for (int it2 = it1 + 1; it2 < num_images; it2++) {
      BBox2 box = boxes[it1]; // deep copy
      box.crop(boxes[it2]);
      if (!box.empty())
        overlap_list.insert(std::make_pair(image_files[it1], image_files[it2]));
    }
  }

  return;
}

// Convert dir1/image1.cub or dir1/image1.xml to out-prefix-image1.adjust
std::string bundle_adjust_file_name(std::string const& prefix,
                                         std::string const& input_img,
                                         std::string const& input_cam){

  // Create the adjusted camera file name from the original camera filename,
  // unless it is empty, and then use the image file name.
  std::string file = input_cam;
  if (file == "")
    file = input_img;

  return prefix + "-" + fs::path(file).stem().string() + ".adjust";
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
    std::string ba_name = asp::bundle_adjust_file_name(out_prefix, img, cam);
    std::string adj = vw::ip::strip_path(out_prefix, ba_name);

    if (img_set.find(img) != img_set.end()) 
      vw_throw(vw::ArgumentErr() << "Found duplicate image: " << img << "\n");
    
    if (cam != "" && cam_set.find(cam) != cam_set.end()) 
      vw_throw(vw::ArgumentErr() << "Found duplicate camera: " << cam << "\n");

    if (adj_set.find(adj) != adj_set.end()) 
      vw_throw(vw::ArgumentErr() << "Found duplicate adjustment name: " << adj << "\n");

    img_set.insert(img);
    if (cam != "") cam_set.insert(cam);
    adj_set.insert(adj);
    
  }
}

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
                                std::set<std::pair<std::string, std::string>> const&
                                overlap_list,
                                // Output
                                std::vector<std::pair<int,int>> & all_pairs) {

  // Wipe the output
  all_pairs.clear();

  // Need this to avoid repetitions
  std::set<std::pair<int, int>> local_set;
  
  int num_images = image_files.size();
  for (int i0 = 0; i0 < num_images; i0++){

    for (int j0 = i0 + 1; j0 <= i0 + overlap_limit; j0++){

      // Make copies of i and j which we can modify
      int i = i0, j = j0;

      if (j >= num_images) {
        
        if (!match_first_to_last)
          continue; // out of bounds

        j = j % num_images; // wrap around

        if (i == j) 
          continue; // can't have matches to itself

        if (i > j) 
          std::swap(i, j);
      }
      
      // Apply the overlap list if manually specified. Otherwise every
      // image pair i, j as above will be matched.
      if (have_overlap_list) {
        auto pair1 = std::make_pair(image_files[i], image_files[j]);
        auto pair2 = std::make_pair(image_files[j], image_files[i]);
        if (overlap_list.find(pair1) == overlap_list.end() &&
            overlap_list.find(pair2) == overlap_list.end())
          continue;
      }

      // If this option is set, don't try to match cameras that are too far apart.
      if (got_est_cam_positions && (position_filter_dist > 0)) {
        Vector3 this_pos  = estimated_camera_gcc[i];
        Vector3 other_pos = estimated_camera_gcc[j];
        if ((this_pos  != Vector3(0,0,0)) && // If both positions are known
            (other_pos != Vector3(0,0,0)) && // and they are too far apart
            (norm_2(this_pos - other_pos) > position_filter_dist)) {
          vw_out() << "Skipping position: " << this_pos << " and "
                   << other_pos << " with distance " << norm_2(this_pos - other_pos)
                   << std::endl;
          continue; // Skip this image pair
        }
      }

      local_set.insert(std::make_pair(i,j));
    }
  }

  // The pairs without repetition
  for (auto it = local_set.begin(); it != local_set.end(); it++)
    all_pairs.push_back(*it);
}

// Given an xyz point in ECEF coordinates, update its height above datum
// by interpolating into a DEM. The user must check the return status.
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

// Shoot rays from all matching interest point. Intersect those with a DEM. Find
// their average. Project it vertically onto the DEM. Invalid or uncomputable
// xyz are set to the zero vector.
void update_point_from_dem(vw::ba::ControlNetwork const& cnet,
                           asp::CRNJ const& crn,
                           std::set<int> const& outliers,
                           std::vector<vw::CamPtr> const& camera_models,
                           vw::cartography::GeoReference const& dem_georef,
                           vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                           // Output
                           std::vector<vw::Vector3> & dem_xyz_vec) {

  int num_tri_points = cnet.size();

  dem_xyz_vec = std::vector<vw::Vector3>(num_tri_points, vw::Vector3(0, 0, 0));
  std::vector<int> dem_xyz_count(num_tri_points, 0);
  
  for (int icam = 0; icam < (int)crn.size(); icam++) {
    
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {
        
      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
        continue; // GCP do not get modified
      
      if (outliers.find(ipt) != outliers.end())
        continue; // Skip outliers
        
      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;
        
      // Ideally this point projects back to the pixel observation, so use the
      // triangulated position as initial guess.
      Vector3 xyz_guess = cnet[ipt].position();

      // Points at planet center are outliers. This check is likely redundant,
      // but good to have.
      if (xyz_guess == Vector3(0, 0, 0))
        continue;

      bool treat_nodata_as_zero = false;
      bool has_intersection = false;
      double height_error_tol = 0.001; // 1 mm should be enough
      double max_abs_tol      = 1e-14; // abs cost fun change b/w iterations
      double max_rel_tol      = 1e-14;
      int num_max_iter        = 25;   // Using many iterations can be very slow

      Vector3 dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
        (camera_models[icam]->camera_center(observation),
         camera_models[icam]->pixel_to_vector(observation),
         interp_dem, dem_georef, treat_nodata_as_zero, has_intersection,
         height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);

      if (!has_intersection) 
        continue;

      dem_xyz_vec[ipt] += dem_xyz;
      dem_xyz_count[ipt]++;
    }
  }

  // Average the successful intersections
  for (size_t xyz_it = 0; xyz_it < dem_xyz_vec.size(); xyz_it++) {
    if (dem_xyz_count[xyz_it] > 0) 
      dem_xyz_vec[xyz_it] = dem_xyz_vec[xyz_it] / double(dem_xyz_count[xyz_it]);
    else
      dem_xyz_vec[xyz_it] = Vector3();
  }

  // Project vertically onto the DEM
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
      continue; // GCP keep their own thing
    
    if (outliers.find(ipt) != outliers.end())
      continue; // Skip outliers
    
    if (dem_xyz_vec[ipt] == Vector3())
      continue; // Skip invalid points
      
    Vector3 observation = dem_xyz_vec[ipt];
    if (update_point_height_from_dem(dem_georef, interp_dem,  
                                     observation)) {
      dem_xyz_vec[ipt] = observation;
    }
    
  }

  return;
}

// Flag outliers by reprojection error with input cameras. This assumes that
// the input cameras are pretty accurate.
void flag_initial_outliers(vw::ba::ControlNetwork const& cnet,
                           asp::CRNJ const& crn,
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
      
      VW_ASSERT(icam < num_cameras, ArgumentErr() << "Out of bounds in the number of cameras.");
      VW_ASSERT(ipt < num_tri_points, ArgumentErr() << "Out of bounds in the number of points.");

      if (outliers.find(ipt) != outliers.end()) {
        // Is an outlier
        continue;
      }
      
      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;

      Vector3 const& tri_point = cnet[ipt].position(); // alias

      if (tri_point == Vector3(0, 0, 0)) {
        // Points at planet center are outliers
        outliers.insert(ipt);
        continue;
      }
      
      vw::Vector2 pix;
      try {
        pix = camera_models[icam]->point_to_pixel(tri_point);
        bool is_good = (norm_2(pix - observation) <= max_init_reproj_error);
        if (!is_good) { // this checks for NaN too
          outliers.insert(ipt);
          continue;
        }
      } catch(...) {
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

} // end namespace asp