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

#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/ImageUtils.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>

#include <boost/filesystem.hpp>
#include <string>

namespace fs = boost::filesystem;

namespace asp {

using namespace vw;

// Convert dir1/image1.cub or dir1/image1.xml to out-prefix-image1.adjust
std::string bundle_adjust_file_name(std::string const& prefix,
                                    std::string const& input_img,
                                    std::string const& input_cam) {

  // Create the adjusted camera file name from the original camera filename,
  // unless it is empty, and then use the image file name.
  std::string file = input_cam;
  if (file == "")
    file = input_img;

  // Find the basename using boost
  file = fs::path(file).filename().string();

  // Find the last dot in the file name. If not found, set it to the length of
  // the string.
  size_t dot = file.rfind(".");
  if (dot == std::string::npos)
    dot = file.size();
  // Find the substring until the dot
  file = file.substr(0, dot);
  
  return prefix + "-" + file + ".adjust";
}

// If we have adjusted camera models, load them. The adjustment may be in the
// rotation matrix, camera center, or pixel offset. Otherwise return unadjusted
// cameras.
vw::CamPtr load_adjusted_model(vw::CamPtr cam,
                               std::string const& image_file,
                               std::string const& camera_file,
                               std::string const& ba_prefix,
                               vw::Vector2 const& pixel_offset) {

  // Any tool using adjusted camera models must pre-populate the
  // prefix at which to find them.
  if (ba_prefix == "" && pixel_offset == vw::Vector2())
    return cam; // Return the unadjusted cameras if there is no adjustment

  Vector3 position_correction;
  Quat pose_correction;

  // These must start initialized. Note that we may have a pixel
  // offset passed in from outside, or a pixel offset and scale
  // that we read from an adjust file. We will throw an error
  // below if both scenarios happen.
  Vector2 local_pixel_offset = pixel_offset;
  double local_scale = 1.0;

  // Ensure these vectors are populated even when there are no corrections to read,
  // as we may still have pixel offset.
  position_correction = Vector3();
  pose_correction = Quat(math::identity_matrix<3>());

  if (ba_prefix != "") { // If a bundle adjustment file was specified

    // Get full BA file path
    std::string adjust_file 
      = asp::bundle_adjust_file_name(ba_prefix, image_file, camera_file);

    // This is a fix for when what is passed in is a mapprojected image. Need to
    // peek and find the raw image and camera file names in the provided image
    // geoheader.
    if (!boost::filesystem::exists(adjust_file)) {
      std::string adj_key, img_file_key, cam_type_key, cam_file_key, dem_file_key;
      std::string adj_prefix_raw, image_file_raw, cam_type, cam_file_raw, dem_file;
      asp::read_mapproj_header(image_file, adj_key, img_file_key, cam_type_key, 
                               cam_file_key, dem_file_key,
                               adj_prefix_raw, image_file_raw, cam_type,
                               cam_file_raw, dem_file);
      if (image_file_raw != "")
        adjust_file = asp::bundle_adjust_file_name(ba_prefix, image_file_raw, 
                                                    cam_file_raw);
    }

    if (!boost::filesystem::exists(adjust_file))
      vw_throw(InputErr() << "Missing adjusted camera model: " << adjust_file << ".\n");

    vw_out() << "Using adjusted camera model: " << adjust_file << std::endl;
    asp::read_adjustments(adjust_file, position_correction, pose_correction,
			  local_pixel_offset, local_scale); // these will change

    if (local_pixel_offset != Vector2() || local_scale != 1.0) {
      // We read a custom scale and pixel offset passed by the user. But then
      // the pixel offset passed in by the caller is not valid. Instead of
      // sorting things out simply refuse to allow this scenario.
      if (pixel_offset != Vector2()) {
        vw_throw(InputErr() << "Cannot use crop win functionality with custom "
                 << "scale and pixel offset in .adjust files.\n");
      }
    }else{
      // In this case we have local_pixel_offset == (0, 0) local_scale == 1.0.
      // So use the pixel_offset passed in by the caller. Scale will stay at 1.0.
      local_pixel_offset = pixel_offset;
    }

  } // End case for parsing bundle adjustment file

  // Create the adjusted camera model object with the info we loaded
  return vw::CamPtr
             (new vw::camera::AdjustedCameraModel(cam, position_correction,
                                                 pose_correction, local_pixel_offset,
                                                 local_scale));
}

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

// Return the camera pixel offset, if having --left-image-crop-win and same for right.
vw::Vector2 camera_pixel_offset(bool isMapProjected,
                                std::string const& left_image_file,
                                std::string const& right_image_file,
                                std::string const& curr_image_file) {
  // For map-projected images we don't apply a pixel offset.
  // When we need to do stereo on cropped images, we just
  // crop the images together with their georeferences.
  if (isMapProjected)
    return Vector2(0, 0);

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  if (!crop_left && !crop_right)
    return Vector2(0, 0); // No offset needed
    
  vw::Vector2 left_pixel_offset(0, 0), right_pixel_offset(0, 0);
  if (crop_left) left_pixel_offset  = stereo_settings().left_image_crop_win.min();
  if (crop_right) right_pixel_offset = stereo_settings().right_image_crop_win.min();
  
  if (curr_image_file == left_image_file)
    return left_pixel_offset;
  else if (curr_image_file == right_image_file)
    return right_pixel_offset;
  else
    // If the image files were not specified, no offset and no error.
    if ((left_image_file != "") || (right_image_file != ""))
      vw_throw(ArgumentErr() 
               << "Supplied image file does not match left or right image file.");

  return Vector2(0, 0);
}

} //end namespace asp
