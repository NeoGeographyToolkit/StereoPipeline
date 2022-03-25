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

#include <vw/Core/Log.h>
#include <vw/Camera/CameraModel.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/CameraBBox.h>

#include <asp/Core/BundleAdjustUtils.h>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ba;

std::string g_piecewise_adj_str = "PIECEWISE_ADJUSTMENTS";
std::string g_session_str = "SESSION";

void asp::read_adjustments(std::string const& filename,
                           bool & piecewise_adjustments,
                           vw::Vector2 & adjustment_bounds,
                           std::vector<Vector3> & position_correction,
                           std::vector<Quat> & pose_correction,
			   Vector2 & pixel_offset,
			   double & scale,
                           std::string & session) {

  // Initialize the outputs
  piecewise_adjustments = false;
  adjustment_bounds = Vector2();
  position_correction.clear();
  pose_correction.clear();
  pixel_offset = Vector2();
  scale = 1.0;
  session = "dg"; // default session, for historical reasons
  
  Vector3 pos;
  Vector4 q_buf;
  std::ifstream istr(filename.c_str());

  // Peek to see if the file contains piecewise adjustments
  std::string line;
  if (!std::getline(istr, line))
    vw_throw( ArgumentErr() << "Could not read adjustment file: " << filename << "\n" );
  if (line == g_piecewise_adj_str) {
    piecewise_adjustments = true;

    // Read the session
    std::string a, b;
    if (istr >> a >> b) {
      if (a == g_session_str) {
        session = b;
      }
    }
    
    if (! (istr >> adjustment_bounds[0] >> adjustment_bounds[1]))
      vw_throw( ArgumentErr() << "Could not read adjustment bounds from: " << filename << "\n");
  }else{
    // No piecewise adjustments. Rewind to beginning.
    piecewise_adjustments = false;
    istr.clear();
    istr.seekg(0, std::ios::beg);
  }

  // Read the actual adjustments
  while (1){
    if (! (istr >> pos[0] >> pos[1] >> pos[2]) ) break;
    if (! (istr >> q_buf[0] >> q_buf[1] >> q_buf[2] >> q_buf[3]) ) break;
    
    // The adjustments that are not piecewise may have an offset and a scale
    if (!piecewise_adjustments) {
      double a, b, c;
      if (istr >> a >> b >> c){
	pixel_offset = Vector2(a, b);
	scale = c;
      }
    }
    
    position_correction.push_back(pos);
    pose_correction.push_back(Quat(q_buf));
  }
}

// Write piecewise adjustments
void asp::write_adjustments(std::string const& filename,
                            vw::Vector2 const& adjustment_bounds,
                            std::vector<vw::Vector3> const& position_correction,
                            std::vector<vw::Quat> const& pose_correction,
                            std::string const& session) {

  std::ofstream ostr(filename.c_str());
  ostr.precision(18);

  ostr << g_piecewise_adj_str << std::endl;
  ostr << g_session_str << " " << boost::to_lower_copy(session) << std::endl;
  ostr << adjustment_bounds[0] << ' ' << adjustment_bounds[1] << std::endl;

  for (size_t adj = 0; adj < position_correction.size(); adj++) {
    ostr << position_correction[adj][0] << " "
         << position_correction[adj][1] << " "
         << position_correction[adj][2] << "\n";
    ostr << pose_correction[adj].w() << " "
         << pose_correction[adj].x() << " "
         << pose_correction[adj].y() << " "
         << pose_correction[adj].z() << " " << "\n";
  }
  ostr.close();
}

void asp::write_adjustments(std::string const& filename,
                       Vector3 const& position_correction,
                       Quat const& pose_correction) {
  std::ofstream ostr(filename.c_str());
  ostr.precision(18);
  ostr << position_correction[0] << " " << position_correction[1] << " "
       << position_correction[2] << "\n";
  ostr << pose_correction.w() << " " << pose_correction.x() << " "
       << pose_correction.y() << " " << pose_correction.z() << " " << "\n";
  ostr.close();
}

void asp::compute_stereo_residuals(std::vector<boost::shared_ptr<CameraModel>> const& camera_models,
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

      StereoModel sm( camera_models[cam1].get(),
                      camera_models[cam2].get() );
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
vw::BBox2 asp::camera_bbox_with_cache(std::string const& dem_file,
                                      std::string const& image_file,
                                      boost::shared_ptr<vw::camera::CameraModel> const&
                                      camera_model,
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
      vw_out() << "Read cached footprint bbox from: " << box_path << ":\n" << box << "\n";
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

// See the .h file for the documentation.
void asp::build_overlap_list_based_on_dem
/*        */ (std::string const& out_prefix, std::string const& dem_file, double pct_for_overlap,
              std::vector<std::string> const& image_files,
              std::vector<boost::shared_ptr<vw::camera::CameraModel>> const& camera_models,
              std::set<std::pair<std::string, std::string>> & overlap_list) {

  // Wipe the output
  overlap_list.clear();
  
  // Sanity check
  if (image_files.size() != camera_models.size())
    vw_throw( ArgumentErr() << "Expecting as many images as cameras.\n");
  
  int num_images = image_files.size();
  std::vector<vw::BBox2> boxes(num_images);
  for (int it = 0; it < num_images; it++) {
    boxes[it] = asp::camera_bbox_with_cache(dem_file, image_files[it], camera_models[it],  
                                            out_prefix);

    // Expand the box by the given factor
    double factor = pct_for_overlap / 100.0;

    // The expansion factor can be negative, but not if it results in an empty box
    if (factor <= -1.0) 
      vw_throw(ArgumentErr() << "Invalid percentage when computing the footprint of camera image: "
               << pct_for_overlap  << ".\n");
      
    double half_extra_x = 0.5 * boxes[it].width()  * factor;
    double half_extra_y = 0.5 * boxes[it].height() * factor;
    boxes[it].min() -= Vector2(half_extra_x, half_extra_y);
    boxes[it].max() += Vector2(half_extra_x, half_extra_y);
  }

  // See which boxes overlap. While this is an O(N^2) computation,
  // likely N is at most a thousand or two, which should be
  // manageable. A 2D tree of box corners could be used, and two boxes
  // would then overlap if corners from one box are contained in a
  // second box. That would be a O(N * log(N)) lookup.
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
