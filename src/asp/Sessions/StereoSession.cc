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


/// \file StereoSession.cc
///
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Geometry.h>
#include <vw/Image/PixelMask.h>
#include <vw/Image/PixelTypeInfo.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/Map2CamTrans.h>

#include <asp/Sessions/StereoSession.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Camera/AdjustedLinescanDGModel.h>

#include <boost/filesystem/operations.hpp>

#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>

using namespace vw;


namespace asp {


  // Pass over all the string variables we use
  void StereoSession::initialize( vw::cartography::GdalWriteOptions const& options,
                                  std::string const& left_image_file,
                                  std::string const& right_image_file,
                                  std::string const& left_camera_file,
                                  std::string const& right_camera_file,
                                  std::string const& out_prefix,
                                  std::string const& input_dem) {
    m_options           = options;
    m_left_image_file   = left_image_file;
    m_right_image_file  = right_image_file;
    m_left_camera_file  = left_camera_file;
    m_right_camera_file = right_camera_file;
    m_out_prefix        = out_prefix;
    m_input_dem         = input_dem;
    
    // Do any other initalization steps needed
    init_disk_transform();
  }


  void StereoSession::init_disk_transform() {

    if (!isMapProjected()) // Nothing to do for non map-projected types.
      return;

    // Back up the bundle-adjust prefix that should be used only with the
    // original camera model, not with the model used in mapprojection
    // (e.g., the original camera model could have been DG, but in
    // map-projection we could have used RPC).
    std::string ba_pref_bk = stereo_settings().bundle_adjust_prefix;

    // Load the models used in map-projection. If map-projection was
    // done with adjusted models, apply those adjustments.

    std::string l_adj_prefix, r_adj_prefix;
    {
      std::string adj_key = "BUNDLE_ADJUST_PREFIX";
      // We only read our own map projections which are written in GDAL format
      boost::shared_ptr<vw::DiskImageResource> l_rsrc(new vw::DiskImageResourceGDAL(m_left_image_file ));
      boost::shared_ptr<vw::DiskImageResource> r_rsrc(new vw::DiskImageResourceGDAL(m_right_image_file));
      vw::cartography::read_header_string(*l_rsrc.get(), adj_key, l_adj_prefix);
      vw::cartography::read_header_string(*r_rsrc.get(), adj_key, r_adj_prefix);
    }

    // TODO: Verify that the DEM encoded in the map-projected image above
    // is the same as m_input_dem used below.

    // When loading camera models from the image files, we either use the sensor model for
    // the current session type or else the RPC model which is often used as an approximation.

    const Vector2 zero_pixel_offset(0,0);
    stereo_settings().bundle_adjust_prefix = "";
    if ( (l_adj_prefix != "" && l_adj_prefix != "NONE") )
      stereo_settings().bundle_adjust_prefix = l_adj_prefix;

    if (uses_rpc_map_projection())
      m_left_map_proj_model = load_rpc_camera_model(m_left_image_file,  m_left_camera_file, zero_pixel_offset);
    else // Use the native model
      m_left_map_proj_model = load_camera_model(m_left_image_file,  m_left_camera_file, zero_pixel_offset);

    stereo_settings().bundle_adjust_prefix = "";
    if (r_adj_prefix != "" && r_adj_prefix != "NONE")
      stereo_settings().bundle_adjust_prefix = r_adj_prefix;
    if (uses_rpc_map_projection())
      m_right_map_proj_model = load_rpc_camera_model(m_right_image_file, m_right_camera_file, zero_pixel_offset);
    else // Use the native model
      m_right_map_proj_model = load_camera_model(m_right_image_file, m_right_camera_file, zero_pixel_offset);

    // Go back to the original bundle-adjust prefix now that we have
    // loaded the models used in map-projection.
    stereo_settings().bundle_adjust_prefix = ba_pref_bk;

    VW_ASSERT( m_left_map_proj_model.get() && m_right_map_proj_model.get(),
              ArgumentErr() << "StereoSession: Unable to locate map "
              << "projection camera model inside input files!" );

    // Double check that we can read the DEM and that it has cartographic information.
    VW_ASSERT(!m_input_dem.empty(), InputErr() << "StereoSession: Require input DEM." );
    if (!boost::filesystem::exists(m_input_dem))
      vw_throw( ArgumentErr() << "StereoSession: DEM \"" << m_input_dem << "\" does not exist." );
  }
  

  // A default IP matching implementation that derived classes can use
  bool StereoSession::ip_matching(std::string const& input_file1,
                                  std::string const& input_file2,
                                  vw::Vector2 const& uncropped_image_size,
                                  Vector6f    const& stats1,
                                  Vector6f    const& stats2,
                                  int ip_per_tile,
                                  float nodata1, float nodata2,
                                  vw::camera::CameraModel* cam1,
                                  vw::camera::CameraModel* cam2,
                                  std::string const& match_filename,
                                  std::string const  left_ip_file,
                                  std::string const  right_ip_file){

    vw_out() << "\t--> Matching interest points in StereoSession.\n";

    if (uses_map_projected_inputs()) {
      vw_throw( vw::ArgumentErr() << "StereoSession: IP matching is not implemented for map-projected images since we do not align them!");
      return false;
    }

    bool crop_left  = ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    bool crop_right = ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

    bool rebuild = (!is_latest_timestamp(match_filename, input_file1, input_file2,
                                         m_left_camera_file, m_right_camera_file));

    // We make an exception and not rebuild if explicitly asked
    if (stereo_settings().force_reuse_match_files && boost::filesystem::exists(match_filename))
      rebuild = false;
      
    // If we crop the images we must always create new matching files
    if (!crop_left && !crop_right && !rebuild) {
      vw_out() << "\t--> Using cached match file: " << match_filename << "\n";
      return true;
    }
    
    // Create DiskImageResource objects
    // - A little messy to make sure it works with SPOT5 which will not work without the camera file
    //   but the camera file does not match if the image is cropped.
    // - Ideally there would be a function to make this cleaner.
    boost::shared_ptr<DiskImageResource> rsrc1, rsrc2;
    if (input_file1 == m_left_image_file)
      rsrc1 = vw::DiskImageResourcePtr(m_left_image_file);
    else // Tiff input
      rsrc1 = vw::DiskImageResourcePtr(input_file1);
    if (input_file2 == m_right_image_file)
      rsrc2 = vw::DiskImageResourcePtr(m_right_image_file);
    else // Tiff input
      rsrc2 = vw::DiskImageResourcePtr(input_file2);

    DiskImageView<float> image1(rsrc1), image2(rsrc2);
    ImageViewRef<float> image1_norm=image1, image2_norm=image2;
    // Get normalized versions of the images for OpenCV based methods
    if ( (stereo_settings().ip_matching_method != DETECT_IP_METHOD_INTEGRAL) &&
       (stats1[0] != stats1[1]) ) { // Don't normalize if no stats were provided!
      vw_out() << "\t--> Normalizing images for IP detection using stats " << stats1 << "\n";
      normalize_images(stereo_settings().force_use_entire_range,
                       stereo_settings().individually_normalize,
                       true, // Use percentile based stretch for ip matching
                       stats1,      stats2,
                       image1_norm, image2_norm);
    }

    bool nadir_facing = this->is_nadir_facing();

    // If cameras are null then we cannot use them
    if (cam1 == NULL || cam2 == NULL) {
      nadir_facing = false;
    }
    
    bool inlier = false;
    if (nadir_facing) {
      // Run an IP matching function that takes the camera and datum info into account

      bool use_sphere_for_datum = false; // Assume Mars is not a sphere
      cartography::Datum datum = this->get_datum(cam1, use_sphere_for_datum);

      // This is a bugfix. For RPC models, we must never intersect with
      // a datum whose height is outside of the domain of applicability
      // of the RPC model, as that can lead to very incorrect results.  
      const asp::RPCModel *rpc_cam
          = dynamic_cast<const asp::RPCModel*>(vw::camera::unadjusted_model(cam1));
      if (rpc_cam != NULL) {
        Vector3 lonlatheight_offset = rpc_cam->lonlatheight_offset();
        Vector3 lonlatheight_scale  = rpc_cam->lonlatheight_scale();
        double mid_ht = lonlatheight_offset[2];
        double min_ht = mid_ht - lonlatheight_scale[2];
        double max_ht = mid_ht + lonlatheight_scale[2];
        if (max_ht < 0) 
          vw_out() << "Warning: The RPC model maximum height is below the zero datum.\n";

        if (min_ht > 0) 
          vw_out() << "Warning: The RPC model minimum height is above the zero datum.\n";

        if (max_ht < 0 || min_ht > 0) {
          vw_out() << "RPC model min and max heights above datum: "
             << min_ht << ' ' << max_ht << ".\n";
          vw_out() << "Adjusting the datum to compensate, for the purpose of alignment.\n";
          vw_out() << "The new datum height will be at " << mid_ht
             << " relative to the previous one.\n";
          //vw_out() << "Old datum: " << datum << std::endl;
          datum.set_semi_major_axis(datum.semi_major_axis() + mid_ht);
          datum.set_semi_minor_axis(datum.semi_minor_axis() + mid_ht);
          //vw_out() << "New datum: " << datum << std::endl;
        }
      } // End RPC case

      // A smaller value here makes IP more unique, but also fewer. 
      double ip_uniqueness_thresh = stereo_settings().ip_uniqueness_thresh;

      // TODO: Improve calculation of epipolar parameter!
      // This computes a distance used for throwing out interest points.
      // - It has to be computed using the entire (not cropped) image size!
      // A larger value will keep more (but of lower quality) points.     
      double epipolar_threshold = norm_2(uncropped_image_size)/15;
      if (stereo_settings().epipolar_threshold > 0)
        epipolar_threshold = stereo_settings().epipolar_threshold;
      vw_out() << "\t    Using epipolar threshold = " << epipolar_threshold << std::endl;
      vw_out() << "\t    IP uniqueness threshold  = " << ip_uniqueness_thresh  << std::endl;
      vw_out() << "\t    Datum:                     " << datum << std::endl;
      if (stereo_settings().skip_rough_homography) {
	vw_out() << "\t    Skipping rough homography.\n";
        inlier = ip_matching_no_align(!supports_multi_threading(), cam1, cam2,
                                      image1_norm, image2_norm,
                                      ip_per_tile, datum,
                                      epipolar_threshold, ip_uniqueness_thresh,
                                      match_filename,
                                      left_ip_file, right_ip_file,
                                      nodata1, nodata2);
      } else {
	vw_out() << "\t    Using rough homography.\n";
        inlier = ip_matching_w_alignment(!supports_multi_threading(), cam1, cam2,
                                         image1_norm, image2_norm,
                                         ip_per_tile,
                                         datum, match_filename,
                                         epipolar_threshold, ip_uniqueness_thresh,
                                         left_ip_file, nodata1, nodata2);
      }
    } else { // Not nadir facing
      // Run a simpler purely image-based matching function
      double ip_inlier_factor = stereo_settings().ip_inlier_factor;
      int    inlier_threshold = round(ip_inlier_factor*150.0); // by default this is 10.

      // HACK: If the otherwise unused epipolar threshold is set, use it as
      //       the inlier threshold.
      if (stereo_settings().epipolar_threshold > 0)
        inlier_threshold = stereo_settings().epipolar_threshold;

      vw_out() << "\t    Not using a datum in interest point matching.\n";
      inlier = homography_ip_matching(image1_norm, image2_norm,
				      ip_per_tile,
				      inlier_threshold,
				      match_filename,
				      left_ip_file, right_ip_file,
				      nodata1, nodata2);
    }
    if (!inlier) {
      boost::filesystem::remove(match_filename);
      vw_throw(IOErr() << "Unable to match left and right images.");
    }
    return inlier;
  } // End function ip_matching()

  // Peek inside the images and camera models and return the datum and projection,
  // or at least the datum, packaged in a georef.
  vw::cartography::GeoReference StereoSession::get_georef() {

    vw::cartography::GeoReference georef;

    // First try to see if the image is map-projected.
    bool has_georef = read_georeference(georef, m_left_image_file);

    if (!has_georef) {
      // The best we can do is to get the datum, even non-projected
      // images have that. Create however a fake valid georeference to
      // go with this datum, otherwise we can't read the datum when we
      // needed it later.

      georef = vw::cartography::GeoReference();
      Matrix3x3 transform = georef.transform();

      // assume these are degrees, does not mater much, but it needs be small enough
      double small = 1e-8;
      transform(0,0) = small;
      transform(1,1) = small;
      transform(0,2) = small;
      transform(1,2) = small;
      georef.set_transform(transform);

      georef.set_geographic();

      boost::shared_ptr<vw::camera::CameraModel> cam = this->camera_model(m_left_image_file,
                                                                          m_left_camera_file);
      bool use_sphere_for_datum = true;       // Spherical datum for non-Earth, as done usually
      georef.set_datum(this->get_datum(cam.get(), use_sphere_for_datum));
    }

    return georef;
  }

  // Default implementation of this function.  Derived classes will probably override this.
  void StereoSession::camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                    boost::shared_ptr<vw::camera::CameraModel> &cam2) const{
    cam1 = camera_model(m_left_image_file,  m_left_camera_file);
    cam2 = camera_model(m_right_image_file, m_right_camera_file);
  }

boost::shared_ptr<vw::camera::CameraModel>
StereoSession::camera_model(std::string const& image_file, std::string const& camera_file) const{
  
  vw_out() << "Loading camera model: " << image_file << ' ' << camera_file << "\n";

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 pixel_offset = camera_pixel_offset(m_input_dem,
                                                 m_left_image_file,
                                                 m_right_image_file,
                                                 image_file);
  
  if (camera_file == "") // No camera file provided, use the image file.
    return load_camera_model(image_file, image_file, pixel_offset);
  else // Camera file provided
    return load_camera_model(image_file, camera_file, pixel_offset);
}


  // Processing Hooks. The default is to do nothing.
  void StereoSession::pre_preprocessing_hook(bool adjust_left_image_size,
                                             std::string const& input_file1,
                                             std::string const& input_file2,
                                             std::string      & output_file1,
                                             std::string      & output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  void StereoSession::post_preprocessing_hook(std::string const& input_file1,
                                              std::string const& input_file2,
                                              std::string &output_file1,
                                              std::string &output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  void StereoSession::pre_correlation_hook(std::string const& input_file1,
                                           std::string const& input_file2,
                                           std::string      & output_file1,
                                           std::string      & output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  void StereoSession::post_correlation_hook(std::string const& input_file,
                                            std::string      & output_file) {
    output_file = input_file;
  }

  void StereoSession::pre_filtering_hook(std::string const& input_file,
                                         std::string      & output_file) {
    output_file = input_file;
  }

  void StereoSession::post_filtering_hook(std::string const& input_file,
                                          std::string      & output_file) {
    output_file = input_file;
  }

  ImageViewRef<PixelMask<Vector2f> >
  StereoSession::pre_pointcloud_hook(std::string const& input_file) {
    return DiskImageView<PixelMask<Vector2f> >( input_file );
  }

  void StereoSession::post_pointcloud_hook(std::string const& input_file,
                                           std::string      & output_file) {
    output_file = input_file;
  }



  void StereoSession::get_nodata_values(boost::shared_ptr<vw::DiskImageResource> left_rsrc,
                                        boost::shared_ptr<vw::DiskImageResource> right_rsrc,
                                        float & left_nodata_value,
                                        float & right_nodata_value){

    // The no-data value read from options overrides the value present in the image files.
    left_nodata_value  = std::numeric_limits<float>::quiet_NaN();
    right_nodata_value = std::numeric_limits<float>::quiet_NaN();
    if (left_rsrc->has_nodata_read ()) left_nodata_value  = left_rsrc->nodata_read();
    if (right_rsrc->has_nodata_read()) right_nodata_value = right_rsrc->nodata_read();

    float opt_nodata = stereo_settings().nodata_value;
    if (!std::isnan(opt_nodata)){

      if ( opt_nodata < left_nodata_value )
        vw_out(WarningMessage) << "It appears that the user-supplied no-data value is less than"
                               << " the no-data value of left image. This may not be what was intended.\n";
      if ( opt_nodata < right_nodata_value )
        vw_out(WarningMessage) << "It appears that the user-supplied no-data value is less than"
                               << " the no-data value of right image. This may not be what was intended.\n";

      left_nodata_value  = opt_nodata;
      right_nodata_value = opt_nodata;
    }

    return;
  }

bool StereoSession::
shared_preprocessing_hook(vw::cartography::GdalWriteOptions & options,
                          std::string const                 & left_input_file,
                          std::string const                 & right_input_file,
                          std::string                       & left_output_file,
                          std::string                       & right_output_file,
                          std::string                       & left_cropped_file,
                          std::string                       & right_cropped_file,
                          float                             & left_nodata_value,
                          float                             & right_nodata_value,
                          bool                              & has_left_georef,
                          bool                              & has_right_georef,
                          vw::cartography::GeoReference     & left_georef,
                          vw::cartography::GeoReference     & right_georef){

  {
    // Retrieve nodata values and let the handles go out of scope right away.

    // For this to work the ISIS type must be registered with the
    // DiskImageResource class.  - This happens in "stereo.cc", so
    // these calls will create DiskImageResourceIsis objects.
    boost::shared_ptr<DiskImageResource>
      left_rsrc (DiskImageResourcePtr(left_input_file )),
      right_rsrc(DiskImageResourcePtr(right_input_file));
    this->get_nodata_values(left_rsrc,         right_rsrc,
                            left_nodata_value, right_nodata_value);
  }

  // Set output file paths
  left_output_file  = this->m_out_prefix + "-L.tif";
  right_output_file = this->m_out_prefix + "-R.tif";

  left_cropped_file  = left_input_file;
  right_cropped_file = right_input_file;

  // Enforce no predictor in compression, it works badly with L.tif and R.tif.
  options = this->m_options;
  options.gdal_options["PREDICTOR"] = "1";

  // Read the georef if available
  has_left_georef  = read_georeference(left_georef,  left_cropped_file);
  has_right_georef = read_georeference(right_georef, right_cropped_file);
  if ( stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef = false;
    has_right_georef = false;
  }

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  // If the output files already exist and are newer than the input files,
  // and we don't crop both left and right images, then there is nothing to do here.
  // Note: Must make sure all outputs are initialized before we
  // get to this part where we exit early.
  
  std::vector<std::string> check_files;
  check_files.push_back(left_input_file    );
  check_files.push_back(right_input_file   );
  check_files.push_back(m_left_camera_file );
  check_files.push_back(m_right_camera_file);
  bool rebuild = (!is_latest_timestamp(left_output_file, check_files ) ||
                  !is_latest_timestamp(right_output_file, check_files)  );

  if ( !rebuild && !crop_left && !crop_right) {
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelGray<float32> > out_left (left_output_file );
      DiskImageView<PixelGray<float32> > out_right(right_output_file);
      vw_out(InfoMessage) << "\t--> Using cached normalized input images.\n";
      vw_settings().reload_config();
      return true; // Return true if we exist early since the images exist
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  // See if to crop the images
  if (crop_left) {
    // Crop the image, will use them from now on. Crop the georef as well, if available.
    left_cropped_file = this->m_out_prefix + "-L-cropped.tif";

    has_left_georef = read_georeference(left_georef, left_input_file);
    bool has_nodata = true;

    DiskImageView<float> left_orig_image(left_input_file);
    BBox2i left_win = stereo_settings().left_image_crop_win;
    left_win.crop (bounding_box(left_orig_image));

    vw_out() << "\t--> Writing cropped image: " << left_cropped_file << "\n";
    block_write_gdal_image(left_cropped_file,
                           crop(left_orig_image, left_win),
                           has_left_georef, crop(left_georef, left_win),
                           has_nodata, left_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  }
  if (crop_right) {
    // Crop the image, will use them from now on. Crop the georef as well, if available.
    right_cropped_file = this->m_out_prefix + "-R-cropped.tif";

    has_right_georef = read_georeference(right_georef, right_input_file);
    bool has_nodata = true;

    DiskImageView<float> right_orig_image(right_input_file);
    BBox2i right_win = stereo_settings().right_image_crop_win;
    right_win.crop(bounding_box(right_orig_image));

    vw_out() << "\t--> Writing cropped image: " << right_cropped_file << "\n";
    block_write_gdal_image(right_cropped_file,
                           crop(right_orig_image, right_win),
                           has_right_georef,
                           crop(right_georef, right_win),
                           has_nodata, right_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  }


  // Re-read the georef, since it changed above.
  has_left_georef  = read_georeference(left_georef,  left_cropped_file);
  has_right_georef = read_georeference(right_georef, right_cropped_file);
  if ( stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef  = false;
    has_right_georef = false;
  }

  return false; // don't exit early
}

void StereoSession::get_input_image_crops(vw::BBox2i &left_image_crop, vw::BBox2i &right_image_crop) const {

  // Set the ROIs to the entire image if the input crop windows are not set.
  Vector2i left_size  = file_image_size(m_left_image_file );
  Vector2i right_size = file_image_size(m_right_image_file);

  if (stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0))
    left_image_crop  = stereo_settings().left_image_crop_win;
  else
    left_image_crop = BBox2i(0, 0, left_size [0], left_size [1]);

  if (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0))
    right_image_crop = stereo_settings().right_image_crop_win;
  else
    right_image_crop = BBox2i(0, 0, right_size[0], right_size[1]);
}


//------------------------------------------------------------------------------
// Code for handling disk-to-sensor transform


// TODO: Move this function somewhere else!
/// Computes a Map2CamTrans given a DEM, image, and a sensor model.
inline StereoSession::tx_type
getTransformFromMapProject(const std::string &input_dem_path,
                           const std::string &img_file_path,
                           boost::shared_ptr<vw::camera::CameraModel> map_proj_model_ptr) {

  // Read in data necessary for the Map2CamTrans object
  cartography::GeoReference dem_georef, image_georef;
  if (!read_georeference(dem_georef, input_dem_path))
    vw_throw( ArgumentErr() << "The DEM \"" << input_dem_path << "\" lacks georeferencing information.");
  if (!read_georeference(image_georef, img_file_path))
    vw_throw( ArgumentErr() << "The image \"" << img_file_path << "\" lacks georeferencing information.");

  bool call_from_mapproject = false;
  DiskImageView<float> img(img_file_path);
  return StereoSession::tx_type(new cartography::Map2CamTrans(map_proj_model_ptr.get(),
                                   image_georef, dem_georef, input_dem_path,
                                   Vector2(img.cols(), img.rows()),
                                   call_from_mapproject));
}


// Return the left and right map-projected images. These are the same
// as the input images unless it is desired to use cropped images.
inline std::string left_mapproj(std::string const& left_image,
                                std::string const& out_prefix){
  if ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)){
    return out_prefix + "-L-cropped.tif";
  }
  return left_image;
}
inline std::string right_mapproj(std::string const& right_image,
                                      std::string const& out_prefix){
  if ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) ){
    return out_prefix + "-R-cropped.tif";
  }
  return right_image;
}


typename StereoSession::tx_type
StereoSession::tx_left_homography() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-L.exr" );
  }
  return tx_type( new vw::HomographyTransform(tx) );
}

typename StereoSession::tx_type
StereoSession::tx_right_homography() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-R.exr" );
  }
  return tx_type( new vw::HomographyTransform(tx) );
}

typename StereoSession::tx_type
StereoSession::tx_identity() const {
  Matrix<double> tx = math::identity_matrix<3>();
  return tx_type( new vw::HomographyTransform(tx) );
}


typename StereoSession::tx_type
StereoSession::tx_left_map_trans() const {
  std::string left_map_proj_image = left_mapproj(m_left_image_file, m_out_prefix);
  if (!m_left_map_proj_model)
    vw_throw( ArgumentErr() << "Map projection model not loaded for image " << left_map_proj_image);
  return getTransformFromMapProject(m_input_dem, left_map_proj_image, m_left_map_proj_model);
}
typename StereoSession::tx_type
StereoSession::tx_right_map_trans() const {
  std::string right_map_proj_image = right_mapproj(m_right_image_file, m_out_prefix);
  if (!m_right_map_proj_model)
    vw_throw( ArgumentErr() << "Map projection model not loaded for image " << right_map_proj_image);
  return getTransformFromMapProject(m_input_dem, right_map_proj_image, m_right_map_proj_model);
}


boost::shared_ptr<vw::camera::CameraModel> 
StereoSession::load_rpc_camera_model(std::string const& image_file, 
                                     std::string const& camera_file,
                                     Vector2 pixel_offset) const{
  std::string err1, err2;
  try {
    if (camera_file != ""){
      return load_adjusted_model(m_camera_loader.load_rpc_camera_model(camera_file),
                                image_file, camera_file, pixel_offset);
    }
  }
  catch(std::exception const& e1) {
    err1 = e1.what();
  }
  try {
    return load_adjusted_model(m_camera_loader.load_rpc_camera_model(image_file),
                              image_file, camera_file, pixel_offset);
  }
  catch(std::exception const& e2) {
    err2 = e2.what();
  }

  // For Cartosat, GDAL chokes. The user must move {image}_RPC_ORG.TXT
  // to {image}_RPC.TXT
  std::string truncated =  boost::filesystem::path(image_file).replace_extension("").string();
  std::string org_file = truncated + "_RPC_ORG.TXT";
  std::string rpc_file = truncated + "_RPC.TXT";
  std::string msg = "";
  if (boost::filesystem::exists(org_file))
    msg = "Detected file: " + org_file + ". If this is Cartosat data, any such files "
      "must be moved to names like " + rpc_file
      + " by overwriting those files if necessary.\n";
    
  // Raise a custom exception if both failed
  vw_throw(ArgumentErr() << "Unable to load RPC model from either:\n" << image_file
          << "\nor:\n" << camera_file << "\n"
	   << err1 << "\n" << err2 << "\n" << msg);
} // End function load_rpc_camera_model


vw::Vector2 StereoSession::camera_pixel_offset(std::string const& input_dem,
                                               std::string const& left_image_file,
                                               std::string const& right_image_file,
                                               std::string const& curr_image_file){
  // For map-projected images we don't apply a pixel offset.
  // When we need to do stereo on cropped images, we just
  // crop the images together with their georeferences.
  if (input_dem != "")
    return Vector2();

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  vw::Vector2 left_pixel_offset, right_pixel_offset;
  if (crop_left ) left_pixel_offset  = stereo_settings().left_image_crop_win.min();
  if (crop_right) right_pixel_offset = stereo_settings().right_image_crop_win.min();
  
  if (curr_image_file == left_image_file)
    return left_pixel_offset;
  else if (curr_image_file == right_image_file)
    return right_pixel_offset;
  else
    // If the image files were not specified, no offset and no error.
    if ((left_image_file != "") || (right_image_file != ""))
      vw_throw(ArgumentErr() << "Supplied image file does not match left or right image file.");

  return Vector2();
}


boost::shared_ptr<vw::camera::CameraModel>
StereoSession::load_adjusted_model(boost::shared_ptr<vw::camera::CameraModel> cam,
                                   std::string const& image_file,
                                   std::string const& camera_file,
                                   vw::Vector2 const& pixel_offset){

  // Any tool using adjusted camera models must pre-populate the
  // prefix at which to find them.
  std::string ba_pref = stereo_settings().bundle_adjust_prefix;
  if (ba_pref == "" && pixel_offset == vw::Vector2())
    return cam; // Just return if nothing is adjusting the camera

  std::vector<Vector3> position_correction;
  std::vector<Quat   > pose_correction;

  // These must start initialized. Note that we may have a pixel
  // offset passed in from outside, or a pixel offset and scale
  // that we read from an adjust file. We will throw an error
  // below if both scenarios happen.
  Vector2 local_pixel_offset = pixel_offset;
  double local_scale = 1.0;

  // Ensure these vectors are populated even when there are no corrections to read,
  // as we may still have pixel offset.
  position_correction.push_back(Vector3());
  pose_correction.push_back(Quat(math::identity_matrix<3>()));

  if (ba_pref != "") { // If a bundle adjustment file was specified

    // Get full BA file path
    std::string adjust_file = asp::bundle_adjust_file_name(ba_pref, image_file, camera_file);

    if (!boost::filesystem::exists(adjust_file))
      vw_throw(InputErr() << "Missing adjusted camera model: " << adjust_file << ".\n");

    vw_out() << "Using adjusted camera model: " << adjust_file << std::endl;
    bool piecewise_adjustments;
    Vector2 adjustment_bounds;
    std::string session;
    asp::read_adjustments(adjust_file, piecewise_adjustments,
                          adjustment_bounds, position_correction, pose_correction,
			  local_pixel_offset, local_scale, // these will change
			  session);

    if (local_pixel_offset != Vector2() || local_scale != 1.0) {
      // We read a custom scale and pixel offset passed by the user. But then
      // the pixel offset passed in by the caller is not valid. Instead of
      // sorting things out simply refuse to allow this scenario.
      if (pixel_offset != Vector2()) {
	      vw_throw(InputErr() << "Cannot use crop-win functionality with custom scale and pixel offset in .adjust files.\n");
      }
    }else{
      // In this case we have local_pixel_offset == (0, 0) local_scale == 1.0.
      // So use the pixel_offset passed in by the caller. Scale will stay at 1.0.
      local_pixel_offset = pixel_offset;
    }
    
    if (position_correction.empty() || pose_correction.empty())
      vw_throw(InputErr() << "Unable to read corrections.\n");

    // Handle the case of piecewise adjustments for DG and other cameras
    if (piecewise_adjustments) {

      DiskImageView<float> img(image_file);
      Vector2i image_size(img.cols(), img.rows());

      if ( session == "dg" || session == "dgmaprpc") {

        // Create the adjusted DG model
        boost::shared_ptr<camera::CameraModel> adj_dg_cam
          (new AdjustedLinescanDGModel(cam,
                                       stereo_settings().piecewise_adjustment_interp_type,
                                       adjustment_bounds, position_correction,
                                       pose_correction, image_size));

        // Apply the pixel offset and pose corrections. So this a second adjustment
        // on top of the first.
        boost::shared_ptr<camera::CameraModel> adj_dg_cam2
          (new vw::camera::AdjustedCameraModel(adj_dg_cam, Vector3(),
                                               Quat(math::identity_matrix<3>()), pixel_offset));

        return adj_dg_cam2;
      }else{
         // Create the generic adjusted model
         boost::shared_ptr<camera::CameraModel> adj_generic_cam
           (new PiecewiseAdjustedLinescanModel(cam,
                                               stereo_settings().piecewise_adjustment_interp_type,
                                               adjustment_bounds, position_correction,
                                               pose_correction, image_size));

         // Apply the pixel offset and pose corrections. So this a second adjustment
         // on top of the first.
         boost::shared_ptr<camera::CameraModel> adj_generic_cam2
           (new vw::camera::AdjustedCameraModel(adj_generic_cam, Vector3(),
                                                Quat(math::identity_matrix<3>()), pixel_offset));

         return adj_generic_cam2;
      }

    } // End case for piecewise DG adjustment

  } // End case for parsing bundle adjustment file

  // Create VW adjusted camera model object with the info we loaded
  return boost::shared_ptr<camera::CameraModel>(new vw::camera::AdjustedCameraModel
                                                (cam, position_correction[0],
                                                 pose_correction[0], local_pixel_offset,
						 local_scale));
}

} // End namespace asp
