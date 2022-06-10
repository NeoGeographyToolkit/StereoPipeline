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
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Camera/AdjustedLinescanDGModel.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Sessions/StereoSessionASTER.h>

#include <boost/filesystem/operations.hpp>

#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>

using namespace vw;

namespace asp {

  // Pass over all the string variables we use
  void StereoSession::initialize(vw::cartography::GdalWriteOptions const& options,
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
    
    // Do any other initialization steps needed
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
      boost::shared_ptr<vw::DiskImageResource>
        l_rsrc(new vw::DiskImageResourceGDAL(m_left_image_file ));
      boost::shared_ptr<vw::DiskImageResource>
        r_rsrc(new vw::DiskImageResourceGDAL(m_right_image_file));
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
      m_left_map_proj_model = load_rpc_camera_model(m_left_image_file,  m_left_camera_file,
                                                    zero_pixel_offset);
    else // Use the native model
      m_left_map_proj_model = load_camera_model(m_left_image_file,  m_left_camera_file,
                                                zero_pixel_offset);

    stereo_settings().bundle_adjust_prefix = "";
    if (r_adj_prefix != "" && r_adj_prefix != "NONE")
      stereo_settings().bundle_adjust_prefix = r_adj_prefix;
    if (uses_rpc_map_projection())
      m_right_map_proj_model = load_rpc_camera_model(m_right_image_file, m_right_camera_file,
                                                     zero_pixel_offset);
    else // Use the native model
      m_right_map_proj_model = load_camera_model(m_right_image_file, m_right_camera_file,
                                                 zero_pixel_offset);

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
                                  std::string const  right_ip_file) {

    vw_out() << "\t--> Matching interest points in StereoSession.\n";

    if (uses_map_projected_inputs()) {
      vw_throw( vw::ArgumentErr() << "StereoSession: IP matching is not implemented for "
                << "map-projected images since we do not align them!");
      return false;
    }

    bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

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

    // If having to rebuild then wipe the old data
    if (boost::filesystem::exists(left_ip_file)) 
      boost::filesystem::remove(left_ip_file);
    if (boost::filesystem::exists(right_ip_file)) 
      boost::filesystem::remove(right_ip_file);
    if (boost::filesystem::exists(match_filename)) 
      boost::filesystem::remove(match_filename);
    
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
    ImageViewRef<float> image1_norm = image1, image2_norm = image2;
    // Get normalized versions of the images for OpenCV based methods
    if ( (stereo_settings().ip_matching_method != DETECT_IP_METHOD_INTEGRAL) &&
       (stats1[0] != stats1[1]) ) { // Don't normalize if no stats were provided!
      vw_out() << "\t--> Normalizing images for IP detection using stats " << stats1 << "\n";
      bool do_not_exceed_min_max = false;
      asp::normalize_images(stereo_settings().force_use_entire_range,
                            stereo_settings().individually_normalize,
                            true, // Use percentile based stretch for ip matching
                            do_not_exceed_min_max,
                            stats1,      stats2,
                            image1_norm, image2_norm);
    }
    
    bool have_datum = this->have_datum();

    // If cameras are null then we cannot use them
    if (cam1 == NULL || cam2 == NULL) {
      have_datum = false;
    }
    
    bool inlier = false;
    if (have_datum) {
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
             << min_ht << ' ' << max_ht << " meters.\n";
          vw_out() << "Adjusting the datum to compensate, for the purpose of alignment.\n";
          vw_out() << "The new datum height will be at " << mid_ht
             << " meters relative to the previous one.\n";
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

      if (!stereo_settings().correlator_mode) {
        boost::shared_ptr<vw::camera::CameraModel> cam = this->camera_model(m_left_image_file,
                                                                            m_left_camera_file);
        bool use_sphere_for_datum = true;       // Spherical datum for non-Earth, as done usually
        georef.set_datum(this->get_datum(cam.get(), use_sphere_for_datum));
      }
    }

    return georef;
  }

  // Default implementation of this function.  Derived classes will probably override this.
  void StereoSession::camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                    boost::shared_ptr<vw::camera::CameraModel> &cam2) {
    cam1 = camera_model(m_left_image_file,  m_left_camera_file);
    cam2 = camera_model(m_right_image_file, m_right_camera_file);
  }

boost::shared_ptr<vw::camera::CameraModel>
StereoSession::camera_model(std::string const& image_file, std::string const& camera_file,
                            bool quiet) {

  if (stereo_settings().correlator_mode) {
    // No cameras exist, so make some dummy cameras. Recall that we
    // set the session to rpc in this mode so that it is assumed that
    // the cameras may hiding in the images rather than kept
    // separately. In this mode the cameras should not actually get
    // used.
    vw::Vector<double, 20> v; v[0] = 1.0; // to a void a zero denominator
    vw::Vector2 v2(1.0, 1.0); // to avoid division by 0
    vw::Vector3 v3(1.0, 1.0, 1.0);
    boost::shared_ptr<vw::camera::CameraModel>
      cam(new RPCModel(vw::cartography::Datum("WGS84"), v, v, v, v, v2, v2, v3, v3));
    return cam;
  }
  
  // If the desired camera is already loaded, do not load it again.
  std::pair<std::string, std::string> image_cam_pair = std::make_pair(image_file, camera_file);
  
  auto map_it = m_camera_model.find(image_cam_pair);
  if (map_it != m_camera_model.end()) 
    return map_it->second;

  // Sometime when we do many attempts at loading cameras we don't want to print
  // this message. 
  if (!quiet) 
    vw_out() << "Loading camera model: " << image_file << ' ' << camera_file << "\n";

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 pixel_offset = camera_pixel_offset(m_input_dem,
                                                 m_left_image_file,
                                                 m_right_image_file,
                                                 image_file);
  
  if (camera_file == "") // No camera file provided, use the image file.
    m_camera_model[image_cam_pair] = load_camera_model(image_file, image_file, pixel_offset);
  else // Camera file provided
    m_camera_model[image_cam_pair] = load_camera_model(image_file, camera_file, pixel_offset);

  return m_camera_model[image_cam_pair];
}

// Find ip matches and determine the alignment matrices
void StereoSession::determine_image_alignment(// Inputs
                                              std::string  const& out_prefix,
                                              std::string  const& left_cropped_file,
                                              std::string  const& right_cropped_file,
                                              std::string  const& left_uncropped_file,
                                              vw::Vector6f const& left_stats,
                                              vw::Vector6f const& right_stats,
                                              float left_nodata_value,
                                              float right_nodata_value,
                                              boost::shared_ptr<vw::camera::CameraModel> left_cam, 
                                              boost::shared_ptr<vw::camera::CameraModel> right_cam,
                                              bool adjust_left_image_size,
                                              // In-out
                                              vw::Matrix<double> & align_left_matrix,
                                              vw::Matrix<double> & align_right_matrix,
                                              vw::Vector2i & left_size,
                                              vw::Vector2i & right_size) {
  
  // Define the file name containing IP match information.
  std::string match_filename    = ip::match_filename(out_prefix,
                                                     left_cropped_file, right_cropped_file);
  std::string left_ip_filename  = ip::ip_filename(out_prefix, left_cropped_file);
  std::string right_ip_filename = ip::ip_filename(out_prefix, right_cropped_file);
  
  // Detect matching interest points between the left and right input images.
  // The output is written directly to a file.
  DiskImageView<float> left_orig_image(left_uncropped_file);
  vw::Vector2 uncropped_left_image_size = bounding_box(left_orig_image).size();
  this->ip_matching(left_cropped_file, right_cropped_file,
                    uncropped_left_image_size,
                    left_stats, right_stats,
                    stereo_settings().ip_per_tile,
                    left_nodata_value, right_nodata_value,
                    left_cam.get(), right_cam.get(),
                    match_filename, left_ip_filename, right_ip_filename);
  
  // Load the interest points results from the file we just wrote
  std::vector<ip::InterestPoint> left_ip, right_ip;
  ip::read_binary_match_file(match_filename, left_ip, right_ip);
  
  // Compute the appropriate alignment matrix based on the input points
  if (stereo_settings().alignment_method == "homography") {
    left_size = homography_rectification(adjust_left_image_size,
                                         left_size, right_size, left_ip, right_ip,
                                         align_left_matrix, align_right_matrix);
    vw_out() << "\t--> Aligning right image to left using matrices:\n"
               << "\t      " << align_left_matrix  << "\n"
             << "\t      " << align_right_matrix << "\n";
  } else {
    // affineepipolar and local_epipolar
    bool crop_to_shared_area = true;
    left_size
      = affine_epipolar_rectification(left_size,         right_size,
                                      stereo_settings().global_alignment_threshold,
                                      stereo_settings().alignment_num_ransac_iterations,
                                      left_ip,           right_ip,
                                      crop_to_shared_area,
                                      align_left_matrix, align_right_matrix);
    vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
             << "\t      " << submatrix(align_left_matrix, 0,0,2,3) << "\n"
             << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
  }
  // Write out both computed matrices to disk
  write_matrix(out_prefix + "-align-L.exr", align_left_matrix);
  write_matrix(out_prefix + "-align-R.exr", align_right_matrix);
  
  // Because the images are now aligned they are the same size
  right_size = left_size;
}
  
// Default preprocessing hook. Some sessions may override it.
void StereoSession::preprocessing_hook(bool adjust_left_image_size,
                                       std::string const& left_input_file,
                                       std::string const& right_input_file,
                                       std::string      & left_output_file,
                                       std::string      & right_output_file) {

  std::string left_cropped_file, right_cropped_file;
  vw::cartography::GdalWriteOptions options;
  float left_nodata_value, right_nodata_value;
  bool has_left_georef, has_right_georef;
  vw::cartography::GeoReference left_georef, right_georef;
  bool exit_early =
    StereoSession::shared_preprocessing_hook(options,
                                             left_input_file,   right_input_file,
                                             left_output_file,  right_output_file,
                                             left_cropped_file, right_cropped_file,
                                             left_nodata_value, right_nodata_value,
                                             has_left_georef,   has_right_georef,
                                             left_georef,       right_georef);

  if (exit_early)
    return;
  
  // Load the cropped images
  DiskImageView<float> left_disk_image(left_cropped_file),
    right_disk_image(right_cropped_file);
  
  // Get the image sizes. Later alignment options can choose to
  // change this parameters (such as affine epipolar alignment).
  Vector2i left_size  = file_image_size(left_cropped_file);
  Vector2i right_size = file_image_size(right_cropped_file);
  
  // Set up image masks
  ImageViewRef<PixelMask<float>> left_masked_image
    = create_mask_less_or_equal(left_disk_image,  left_nodata_value);
  ImageViewRef<PixelMask<float>> right_masked_image
    = create_mask_less_or_equal(right_disk_image, right_nodata_value);
  
  // Compute input image statistics
  Vector6f left_stats  = gather_stats(left_masked_image,  "left",
                                      this->m_out_prefix, left_cropped_file);
  Vector6f right_stats = gather_stats(right_masked_image, "right",
                                      this->m_out_prefix, right_cropped_file);
  
  ImageViewRef<PixelMask<float>> Limg, Rimg;
  
  // Use no-data in interpolation and edge extension
  PixelMask<float>nodata_pix(0); nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> ext_nodata(nodata_pix); 
  
  // Initialize alignment matrices and get the input image sizes.
  Matrix<double> align_left_matrix  = math::identity_matrix<3>(),
    align_right_matrix = math::identity_matrix<3>();
  
  // Generate aligned versions of the input images according to the
  // options.
  vw_out() << "\t--> Applying alignment method: " << stereo_settings().alignment_method << "\n";
  if (stereo_settings().alignment_method == "epipolar") {
    
    epipolar_alignment(left_masked_image, right_masked_image, ext_nodata,  
                       // Outputs
                       Limg, Rimg);
    
  } else if (stereo_settings().alignment_method == "homography"     ||
             stereo_settings().alignment_method == "affineepipolar" ||
             stereo_settings().alignment_method == "local_epipolar") {
    
    // Load the cameras
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    if (this->name() != "aster") {
      this->camera_models(left_cam, right_cam);
    }else{
      vw_out() << "Using the RPC model instead of the exact ASTER model for interest point "
               << "matching, for speed. This does not affect the final DEM accuracy.\n";
      StereoSessionASTER * aster_session = dynamic_cast<StereoSessionASTER*>(this);
      if (aster_session == NULL) 
        vw_throw( ArgumentErr() << "ASTER session is expected." );
      aster_session->rpc_camera_models(left_cam, right_cam);
    }
    
    determine_image_alignment(// Inputs
                              m_out_prefix, left_cropped_file, right_cropped_file,  
                              left_input_file,
                              left_stats, right_stats, left_nodata_value, right_nodata_value,  
                              left_cam, right_cam,
                              adjust_left_image_size,  
                              // In-out
                              align_left_matrix, align_right_matrix, left_size, right_size);
    
    // Apply the alignment transform to both input images
    Limg = transform(left_masked_image,
                     HomographyTransform(align_left_matrix),
                     left_size.x(), left_size.y());
    Rimg = transform(right_masked_image,
                     HomographyTransform(align_right_matrix),
                     right_size.x(), right_size.y());
      
  } else {
    // No alignment, just provide the original files.
    Limg = left_masked_image;
    Rimg = right_masked_image;
  } // End of image alignment block
  
  // Apply our normalization options.
  bool use_percentile_stretch = false;
  bool do_not_exceed_min_max = (this->name() == "isis" ||
                                this->name() == "isismapisis");
  // TODO(oalexan1): Should one add above "csm" and "csmmapcsm"?
  asp::normalize_images(stereo_settings().force_use_entire_range,
                        stereo_settings().individually_normalize,
                        use_percentile_stretch, 
                        do_not_exceed_min_max,
                        left_stats, right_stats, Limg, Rimg);

  if (stereo_settings().alignment_method == "local_epipolar") {
    // Save these stats for local epipolar alignment, as they will be used
    // later in each tile.
    std::string left_stats_file  = this->m_out_prefix + "-lStats.tif";
    std::string right_stats_file = this->m_out_prefix + "-rStats.tif";
    vw_out() << "Writing: " << left_stats_file << ' ' << right_stats_file << std::endl;
    vw::Vector<float32> left_stats2  = left_stats;  // cast
    vw::Vector<float32> right_stats2 = right_stats; // cast
    write_vector(left_stats_file,  left_stats2 );
    write_vector(right_stats_file, right_stats2);
  }
  
  // The output no-data value must be < 0 as we scale the images to [0, 1].
  bool has_nodata = true;
  float output_nodata = -32768.0;
  vw_out() << "\t--> Writing pre-aligned images.\n";
  vw_out() << "\t--> Writing: " << left_output_file << ".\n";
  block_write_gdal_image(left_output_file, apply_mask(Limg, output_nodata),
                         has_left_georef, left_georef,
                         has_nodata, output_nodata, options,
                         TerminalProgressCallback("asp","\t  L:  "));
    
  vw_out() << "\t--> Writing: " << right_output_file << ".\n";
  if (stereo_settings().alignment_method == "none") {
    // Do not crop the right image to have the same dimensions as the
    // left image. Since there is no alignment, and images may not be
    // georeferenced, we do not know what portion of the right image
    // corresponds best to the left image, so cropping may throw away
    // an area where the left and right images overlap.
    block_write_gdal_image(right_output_file, apply_mask(Rimg, output_nodata),
                           has_right_georef, right_georef,
                           has_nodata, output_nodata, options,
                           TerminalProgressCallback("asp","\t  R:  "));
  } else {
    // Crop the right aligned image consistently with how the alignment
    // transform expects it. The resulting R.tif will have the same
    // size as L.tif. Extra pixels will be filled with nodata.
    block_write_gdal_image(right_output_file,
                           apply_mask(crop(edge_extend(Rimg, ext_nodata), 
                                           bounding_box(Limg)), output_nodata),
                           has_right_georef, right_georef,
                           has_nodata, output_nodata, options,
                           TerminalProgressCallback("asp","\t  R:  ") );
  }

  if (this->do_bathymetry())
    this->align_bathy_masks(options);
    
} // End function preprocessing_hook
  
void StereoSession::pre_filtering_hook(std::string const& input_file,
                                       std::string      & output_file) {
  output_file = input_file;
}

ImageViewRef<PixelMask<Vector2f> >
StereoSession::pre_pointcloud_hook(std::string const& input_file) {
  return DiskImageView<PixelMask<Vector2f> >( input_file );
}

// A little function whose goal is to avoid repeating same logic in a handful of places
void crop_bathy_mask(vw::cartography::GdalWriteOptions const& options,
                     std::string const& input_mask_file, std::string const& input_image_file,
                     BBox2i const& crop_win, std::string const& cropped_mask_file) {
  
  if (input_mask_file == "") 
    vw_throw( ArgumentErr() << "Required bathy mask file was not specified.");

  // Sanity check, input image and mask must have same size
  DiskImageView<float> input_image(input_image_file);
  DiskImageView<float> input_bathy_mask(input_mask_file);
  if (input_bathy_mask.cols() != input_image.cols() ||
      input_bathy_mask.rows() != input_image.rows()) 
    vw_throw(ArgumentErr() << "Input image and input bathy mask don't have the same dimensions.");
  
  float mask_nodata_value = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(input_mask_file, mask_nodata_value))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from " << input_mask_file);

  vw::cartography::GeoReference georef;
  bool has_georef = read_georeference(georef, input_image_file);

  bool has_mask_nodata = true;
  vw_out() << "\t--> Writing cropped mask: " << cropped_mask_file << "\n";
  block_write_gdal_image(cropped_mask_file,
                         crop(input_bathy_mask, crop_win),
                         has_georef, crop(georef, crop_win),
                         has_mask_nodata, mask_nodata_value,
                         options,
                         TerminalProgressCallback("asp", "\t:  "));
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

  // Retrieve nodata values and let the handles go out of scope right away.
  // For this to work the ISIS type must be registered with the
  // DiskImageResource class. This happens in "stereo.cc", so
  // these calls will create DiskImageResourceIsis objects.
  {
    boost::shared_ptr<DiskImageResource>
      left_rsrc (DiskImageResourcePtr(left_input_file )),
      right_rsrc(DiskImageResourcePtr(right_input_file));
    asp::get_nodata_values(left_rsrc,         right_rsrc,
                           left_nodata_value, right_nodata_value);
  }

  // Set output file paths
  left_output_file  = this->m_out_prefix + "-L.tif";
  right_output_file = this->m_out_prefix + "-R.tif";

  // Enforce no predictor in compression, it works badly with L.tif and R.tif.
  options = this->m_options;
  options.gdal_options["PREDICTOR"] = "1";

  // Read the georef if available in the input images
  has_left_georef  = read_georeference(left_georef,  left_input_file);
  has_right_georef = read_georeference(right_georef, right_input_file);
  if ( stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef = false;
    has_right_georef = false;
  }

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  // Here either the input image or the cropped images will be returned,
  // depending on whether the crop actually happens
  left_cropped_file = this->left_cropped_image();
  right_cropped_file = this->right_cropped_image();
  
  // If the output files already exist and are newer than the input files,
  // and we don't crop both left and right images, then there is nothing to do here.
  // Note: Must make sure all outputs are initialized before we
  // get to this part where we exit early.
  
  bool do_bathy = StereoSession::do_bathymetry();

  std::vector<std::string> check_files;
  check_files.push_back(left_input_file);
  check_files.push_back(right_input_file);
  check_files.push_back(m_left_camera_file);
  check_files.push_back(m_right_camera_file);
  bool rebuild = (!is_latest_timestamp(left_output_file, check_files) ||
                  !is_latest_timestamp(right_output_file, check_files));

  if (do_bathy) {
    rebuild = (rebuild ||
               (!is_latest_timestamp(left_aligned_bathy_mask(), check_files) ||
                !is_latest_timestamp(right_aligned_bathy_mask(), check_files)));
  }
  
  if (!rebuild && !crop_left && !crop_right) {
    try {
      vw_log().console_log().rule_set().add_rule(-1, "fileio");
      DiskImageView<PixelGray<float32> > out_left (left_output_file );
      DiskImageView<PixelGray<float32> > out_right(right_output_file);

      if (do_bathy) {
        DiskImageView<float> left_bathy_mask (left_aligned_bathy_mask());
        DiskImageView<float> right_bathy_mask(right_aligned_bathy_mask());
      }
      
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
    // Crop and save the left image to left_cropped_file
    has_left_georef = read_georeference(left_georef, left_input_file);
    bool has_nodata = true;

    DiskImageView<float> left_orig_image(left_input_file);
    BBox2i left_win = stereo_settings().left_image_crop_win;
    left_win.crop(bounding_box(left_orig_image));

    ImageViewRef<float> left_cropped_image = crop(left_orig_image, left_win);

    if (stereo_settings().left_image_clip != "") {
      // Replace the crop with a given clip. This is a very rarely used option.
      // It can be handy when investigating CCD artifacts correction.
      left_cropped_image = DiskImageView<float>(stereo_settings().left_image_clip);
      if (left_cropped_image.cols() != left_win.width() ||
          left_cropped_image.rows() != left_win.height() ) {
        vw_throw( ArgumentErr() << "The image specified via --left-image-clip has different "
                  << "dimensions than set via --left-image-crop-win.");
      }
    }
    
    vw_out() << "\t--> Writing cropped image: " << left_cropped_file << "\n";
    block_write_gdal_image(left_cropped_file,
                           left_cropped_image,
                           has_left_georef, crop(left_georef, left_win),
                           has_nodata, left_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  }
  
  if (crop_right) {
    // Crop the right image and write to right_cropped_file
    has_right_georef = read_georeference(right_georef, right_input_file);
    bool has_nodata = true;

    DiskImageView<float> right_orig_image(right_input_file);
    BBox2i right_win = stereo_settings().right_image_crop_win;
    right_win.crop(bounding_box(right_orig_image));

    ImageViewRef<float> right_cropped_image = crop(right_orig_image, right_win);

    if (stereo_settings().right_image_clip != "") {
      // Replace the crop with a given clip. This is a very rarely used option.
      // It can be handy when investigating CCD artifacts correction.
      right_cropped_image = DiskImageView<float>(stereo_settings().right_image_clip);
      if (right_cropped_image.cols() != right_win.width() ||
          right_cropped_image.rows() != right_win.height() ) {
        vw_throw( ArgumentErr() << "The image specified via --right-image-clip has different "
                  << "dimensions than set via --right-image-crop-win.");
      }
    }
    
    vw_out() << "\t--> Writing cropped image: " << right_cropped_file << "\n";
    block_write_gdal_image(right_cropped_file,
                           right_cropped_image,
                           has_right_georef,
                           crop(right_georef, right_win),
                           has_nodata, right_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  }
  
  // Re-read the georef, since it may have changed above.
  has_left_georef  = read_georeference(left_georef,  left_cropped_file);
  has_right_georef = read_georeference(right_georef, right_cropped_file);
  if ( stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef  = false;
    has_right_georef = false;
  }
  
  return false; // don't exit early
}

void StereoSession::read_bathy_masks(float & left_bathy_nodata,  float & right_bathy_nodata, 
                                     vw::ImageViewRef<vw::PixelMask<float>> & left_bathy_mask,
                                     vw::ImageViewRef<vw::PixelMask<float>> & right_bathy_mask) {
  
  std::string left_cropped_mask_file = left_cropped_bathy_mask();
  left_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(left_cropped_mask_file, left_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << left_cropped_mask_file);
  left_bathy_mask = create_mask(DiskImageView<float>(left_cropped_mask_file),
                                left_bathy_nodata);

  std::string right_cropped_mask_file = right_cropped_bathy_mask();
  right_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(right_cropped_mask_file, right_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << right_cropped_mask_file);
  right_bathy_mask = create_mask(DiskImageView<float>(right_cropped_mask_file),
                                right_bathy_nodata);
  
  // The left image (after crop) better needs to have the same dims
  // as the left mask after crop, and same for the right
  DiskImageView<float> left_image(this->left_cropped_image());
  DiskImageView<float> right_image(this->right_cropped_image());
  if (left_bathy_mask.cols() != left_image.cols()   || 
      left_bathy_mask.rows() != left_image.rows()   || 
      right_bathy_mask.cols() != right_image.cols() || 
      right_bathy_mask.rows() != right_image.rows() ) {
    vw_throw( ArgumentErr() << "The dimensions of bathymetry masks don't agree "
              << "with the image sizes (after crop win, if applicable)." );
  }
  
}

void StereoSession::read_aligned_bathy_masks
(vw::ImageViewRef<vw::PixelMask<float>> & left_aligned_bathy_mask_image,
 vw::ImageViewRef<vw::PixelMask<float>> & right_aligned_bathy_mask_image) {

  std::string left_aligned_mask_file = left_aligned_bathy_mask();
  float left_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(left_aligned_mask_file, left_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << left_aligned_mask_file);
  left_aligned_bathy_mask_image = create_mask(DiskImageView<float>(left_aligned_mask_file),
                                              left_bathy_nodata);
  
  std::string right_aligned_mask_file = right_aligned_bathy_mask();
  float right_bathy_nodata = -std::numeric_limits<float>::max();
  if (!vw::read_nodata_val(right_aligned_mask_file, right_bathy_nodata))
    vw_throw(ArgumentErr() << "Unable to read the nodata value from "
             << right_aligned_mask_file);
  right_aligned_bathy_mask_image = create_mask(DiskImageView<float>(right_aligned_mask_file),
                                right_bathy_nodata);
}

bool StereoSession::do_bathymetry() const {
  return (stereo_settings().left_bathy_mask != "" || 
          stereo_settings().right_bathy_mask != "");
}

// Align the bathy masks. This will be called in stereo_pprc and, if
// needed, in stereo_tri. Skip this if the masks already exit and are
// not older than the images. This code mirrors very closely the logic
// for how the images are aligned.
void StereoSession::align_bathy_masks(vw::cartography::GdalWriteOptions const& options) {

  bool do_bathy = StereoSession::do_bathymetry();
  
  if (!do_bathy)
    return;

  // Check the timestamp of aligned masks
  std::vector<std::string> check_files;
  check_files.push_back(m_left_image_file);
  check_files.push_back(m_right_image_file);
  check_files.push_back(m_left_camera_file);
  check_files.push_back(m_right_camera_file);
  check_files.push_back(stereo_settings().left_bathy_mask);
  check_files.push_back(stereo_settings().right_bathy_mask);
  
  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  
  bool rebuild = (!is_latest_timestamp(left_aligned_bathy_mask(), check_files) ||
                  !is_latest_timestamp(right_aligned_bathy_mask(), check_files));
  
  if (!rebuild && !crop_left && !crop_right) {
    try {
      vw_log().console_log().rule_set().add_rule(-1, "fileio");
      DiskImageView<float> left_bathy_mask (left_aligned_bathy_mask());
      DiskImageView<float> right_bathy_mask(right_aligned_bathy_mask());
      
      vw_out(InfoMessage) << "\t--> Using cached aligned bathy masks.\n";
      vw_settings().reload_config();
      return; // no need to rebuild, the results exit and are good
      
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  // See if to crop the masks
  if (crop_left) {
    DiskImageView<float> left_orig_image(m_left_image_file);
    BBox2i left_win = stereo_settings().left_image_crop_win;
    left_win.crop(bounding_box(left_orig_image));
    crop_bathy_mask(options, stereo_settings().left_bathy_mask,  
                    m_left_image_file, left_win, left_cropped_bathy_mask());
  }
  if (crop_right) {
    DiskImageView<float> right_orig_image(m_right_image_file);
    BBox2i right_win = stereo_settings().right_image_crop_win;
    right_win.crop(bounding_box(right_orig_image));
    crop_bathy_mask(options, stereo_settings().right_bathy_mask,  
                    m_right_image_file, right_win, right_cropped_bathy_mask());
  }
  
  // Read the unaligned cropped masks
  ImageViewRef<PixelMask<float>> left_bathy_mask, right_bathy_mask;
  float left_bathy_nodata = -std::numeric_limits<float>::max();
  float right_bathy_nodata = -std::numeric_limits<float>::max();
  StereoSession::read_bathy_masks(left_bathy_nodata, right_bathy_nodata,
                                  left_bathy_mask, right_bathy_mask);

  // Use no-data in interpolation and edge extension.
  PixelMask<float>bathy_nodata_pix(0); bathy_nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> bathy_ext_nodata(bathy_nodata_pix); 

  // Get the aligned size from the images already aligned
  Vector2i left_size = file_image_size(this->m_out_prefix + "-L.tif");

  // Read alignment matrices
  Matrix<double> align_left_matrix = math::identity_matrix<3>();
  std::string left_matrix_file = this->m_out_prefix + "-align-L.exr";
  if (stereo_settings().alignment_method == "affineepipolar" ||
      stereo_settings().alignment_method == "local_epipolar") {
    if (boost::filesystem::exists(left_matrix_file))
      read_matrix(align_left_matrix, left_matrix_file);
    else
      vw_throw(NoImplErr() << "Could not read: " << left_matrix_file);
  }

  Matrix<double> align_right_matrix = math::identity_matrix<3>();
  std::string right_matrix_file = this->m_out_prefix + "-align-R.exr";
  if (stereo_settings().alignment_method == "homography"     ||
      stereo_settings().alignment_method == "affineepipolar" ||
      stereo_settings().alignment_method == "local_epipolar") {
    if (boost::filesystem::exists(right_matrix_file))
      read_matrix(align_right_matrix, right_matrix_file);
    else
      vw_throw(NoImplErr() << "Could not read " << right_matrix_file);
  }

  // Generate aligned versions of the masks according to the options.
  ImageViewRef<PixelMask<float>> left_aligned_bathy_mask, right_aligned_bathy_mask;
  if (stereo_settings().alignment_method == "homography"     ||
      stereo_settings().alignment_method == "affineepipolar" ||
      stereo_settings().alignment_method == "local_epipolar") {
    
    left_aligned_bathy_mask = transform(left_bathy_mask,
                                        HomographyTransform(align_left_matrix),
                                        left_size.x(), left_size.y());

    // Note how we use left_size and not right_size
    right_aligned_bathy_mask = transform(right_bathy_mask,
                                         HomographyTransform(align_right_matrix),
                                         left_size.x(), left_size.y());
    
  } else if (stereo_settings().alignment_method == "none") {
    // No alignment
    left_aligned_bathy_mask  = left_bathy_mask;
    right_aligned_bathy_mask = right_bathy_mask;
  } // End of image alignment block

  bool has_bathy_nodata = true;
  float output_nodata = -32768.0;
  
  // Read the georef of the cropped left and right images saved before the masks
  vw::cartography::GeoReference left_georef, right_georef;
  bool has_left_georef  = read_georeference(left_georef,  this->left_cropped_image());
  bool has_right_georef = read_georeference(right_georef, this->right_cropped_image());
  
  std::string left_aligned_bathy_mask_file = StereoSession::left_aligned_bathy_mask();
  vw_out() << "\t--> Writing: " << left_aligned_bathy_mask_file << ".\n";
  block_write_gdal_image(left_aligned_bathy_mask_file,
                         apply_mask(left_aligned_bathy_mask, left_bathy_nodata),
                         has_left_georef, left_georef,
                         has_bathy_nodata, left_bathy_nodata, options,
                         TerminalProgressCallback("asp","\t  L bathy mask:  "));

  // Use same logic as when the right aligned image is written
  if (stereo_settings().alignment_method == "none") {
    std::string right_aligned_bathy_mask_file = StereoSession::right_aligned_bathy_mask();
    vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << ".\n";
    block_write_gdal_image(right_aligned_bathy_mask_file,
                           apply_mask(right_aligned_bathy_mask, right_bathy_nodata),
                           has_right_georef, right_georef,
                           has_bathy_nodata, right_bathy_nodata, options,
                           TerminalProgressCallback("asp","\t  R bathy mask:  "));
  } else {
    std::string right_aligned_bathy_mask_file = StereoSession::right_aligned_bathy_mask();
    vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << ".\n";
    block_write_gdal_image(right_aligned_bathy_mask_file,
                           apply_mask(crop(edge_extend(right_aligned_bathy_mask,
                                                       bathy_ext_nodata),
                                           // Note how we use the left aligned mask bbox
                                           bounding_box(left_aligned_bathy_mask)),
                                      right_bathy_nodata),
                           has_right_georef, right_georef,
                           has_bathy_nodata, right_bathy_nodata,
                           options,
                           TerminalProgressCallback("asp","\t  R bathy mask:  ") );
  }
}
  
// Return the left and right cropped images. These are the same
// as the input images unless the cropping is on.
std::string StereoSession::left_cropped_image() const{
  std::string cropped_image = m_left_image_file;
  if (stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0))
    cropped_image = m_out_prefix + "-L-cropped.tif";
  return cropped_image;
}
  
std::string StereoSession::right_cropped_image() const{
  std::string cropped_image = m_right_image_file;
  if (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0))
    cropped_image = m_out_prefix + "-R-cropped.tif";
  return cropped_image;
}
  
// Apply epipolar alignment to images, if the camera models are pinhole. This will
// be reimplemented in StereoSessionPinhole.
void StereoSession::epipolar_alignment(vw::ImageViewRef<vw::PixelMask<float>> left_masked_image,
                                       vw::ImageViewRef<vw::PixelMask<float>> right_masked_image,
                                       vw::ValueEdgeExtension<vw::PixelMask<float>> ext_nodata,
                                       // Outputs
                                       vw::ImageViewRef<vw::PixelMask<float>> & Limg, 
                                       vw::ImageViewRef<vw::PixelMask<float>> & Rimg) {
  vw_throw(ArgumentErr() << "Epipolar alignment is only implemented for pinhole cameras.");
}
  
std::string StereoSession::left_cropped_bathy_mask() const {
  if (!do_bathymetry()) 
    vw_throw( ArgumentErr() << "The left cropped bathy mask is requested when "
              << "bathymetry mode is not on." );

  bool crop_left = (stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0));
  if (!crop_left) 
    return stereo_settings().left_bathy_mask;

  return this->m_out_prefix + "-L_cropped_bathy_mask.tif";
}
  
std::string StereoSession::right_cropped_bathy_mask() const {
  if (!do_bathymetry()) 
    vw_throw( ArgumentErr() << "The right cropped bathy mask is requested when "
              << "bathymetry mode is not on." );

  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  if (!crop_right) 
    return stereo_settings().right_bathy_mask;

  return this->m_out_prefix + "-R_cropped_bathy_mask.tif";
}

std::string StereoSession::left_aligned_bathy_mask() const {
  return m_out_prefix + "-L_aligned_bathy_mask.tif";
}
  
std::string StereoSession::right_aligned_bathy_mask() const {
  return m_out_prefix + "-R_aligned_bathy_mask.tif";
}

void StereoSession::get_input_image_crops(vw::BBox2i &left_image_crop,
                                          vw::BBox2i &right_image_crop) const {

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
    vw_throw( ArgumentErr() << "The DEM \"" << input_dem_path
              << "\" lacks georeferencing information.");
  if (!read_georeference(image_georef, img_file_path))
    vw_throw( ArgumentErr() << "The image \"" << img_file_path
              << "\" lacks georeferencing information.");

  bool call_from_mapproject = false;
  DiskImageView<float> img(img_file_path);
  return StereoSession::tx_type(new cartography::Map2CamTrans(map_proj_model_ptr.get(),
                                   image_georef, dem_georef, input_dem_path,
                                   Vector2(img.cols(), img.rows()),
                                   call_from_mapproject));
}

typename StereoSession::tx_type
StereoSession::tx_left_homography() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ||
       stereo_settings().alignment_method == "local_epipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-L.exr" );
  }
  return tx_type( new vw::HomographyTransform(tx) );
}

typename StereoSession::tx_type
StereoSession::tx_right_homography() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ||
       stereo_settings().alignment_method == "local_epipolar" ) {
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
  std::string left_map_proj_image = this->left_cropped_image();
  if (!m_left_map_proj_model)
    vw_throw( ArgumentErr() << "Map projection model not loaded for image "
              << left_map_proj_image);
  return getTransformFromMapProject(m_input_dem, left_map_proj_image, m_left_map_proj_model);
}
typename StereoSession::tx_type
StereoSession::tx_right_map_trans() const {
  std::string right_map_proj_image = this->right_cropped_image();
  if (!m_right_map_proj_model)
    vw_throw( ArgumentErr() << "Map projection model not loaded for image "
              << right_map_proj_image);
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
  vw_throw(ArgumentErr() << "Unable to load RPC model from either " << image_file
          << " or " << camera_file << ".\n"
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
        vw_throw(InputErr() << "Cannot use crop win functionality with custom "
                 << "scale and pixel offset in .adjust files.\n");
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
