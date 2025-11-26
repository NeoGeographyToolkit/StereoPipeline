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

// Keep here interest-point matching logic for StereoSession, as this logic
// can be slow to compile.

/// \file StereoSessionIp.cc
///
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/FileUtils.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Sessions/StereoSessionASTER.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Core/ImageUtils.h>

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
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Cartography/GeoTransform.h>

#include <boost/filesystem/operations.hpp>

#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>

using namespace vw;
using namespace vw::cartography;

namespace fs = boost::filesystem;

namespace asp {

// A default IP matching implementation that derived classes can use
bool StereoSession::ip_matching(std::string const& input_file1,
                                std::string const& input_file2,
                                vw::Vector2 const& uncropped_image_size,
                                Vector6f    const& stats1,
                                Vector6f    const& stats2,
                                float nodata1, float nodata2,
                                vw::camera::CameraModel* cam1,
                                vw::camera::CameraModel* cam2,
                                std::string const& match_filename,
                                std::string const  left_ip_file,
                                std::string const  right_ip_file,
                                vw::BBox2i const& bbox1,
                                vw::BBox2i const& bbox2) {

  vw_out() << "\t--> Matching interest points in StereoSession.\n";

  // Fix for ASTER. This will modify the pointers to the cameras for this function,
  // but not the cameras in the caller of this function.
  boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
  if (this->name() == "aster") {
    vw_out() << "Using the RPC model instead of the exact ASTER model for interest point "
              << "matching, for speed. This does not affect the accuracy of final results.\n";
    StereoSessionASTER * aster_session = dynamic_cast<StereoSessionASTER*>(this);
    if (aster_session == NULL)
      vw_throw(ArgumentErr() << "ASTER session is expected.");
    aster_session->rpc_camera_models(left_cam, right_cam);
    cam1 = left_cam.get();
    cam2 = right_cam.get();
  }

  // Sanity checks. Must be here since we will use this code in stereo and bundle_adjust.
  if (asp::stereo_settings().matches_per_tile > 0) {

   if (asp::stereo_settings().ip_per_tile < asp::stereo_settings().matches_per_tile ||
      asp::stereo_settings().ip_per_image > 0)
        vw::vw_throw(vw::ArgumentErr()
          << "When setting --matches-per-tile, must set --ip-per-tile to at least "
          << "a factor of that, and do not set --ip-per-image.\n");

    Vector2i params = asp::stereo_settings().matches_per_tile_params;
    if (params[0] <= 0 || params[1] < params[0] || params[1] > 2 * params[0])
      vw::vw_throw(vw::ArgumentErr()
          << "First value in --matches-per-tile-params must be positive, and second one must be no less than first one but no more than twice the first one.\n");
  }

  bool inlier = false;

  // Try to give a useful error message if things fail
  std::string err = "";

  try {

  // If we crop the images we must always create new matching files.
  // Otherwise, do not rebuild with externally provided match files,
  // or if a match file newer than the image and cameras is found in
  // the output directory.
  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  bool rebuild = (!asp::first_is_newer(match_filename, input_file1, input_file2,
                                  m_left_camera_file, m_right_camera_file));
  if (!crop_left && !crop_right &&
      (stereo_settings().force_reuse_match_files ||
       stereo_settings().clean_match_files_prefix != "" ||
       stereo_settings().match_files_prefix != ""))
    rebuild = false;
  if (crop_left || crop_right)
    rebuild = true;

  if (boost::filesystem::exists(match_filename) && !rebuild) {
    vw_out() << "\t--> Using cached match file: " << match_filename << "\n";
    return true;
  }
  
  // If any cropping is done, or if the vwip file is old, or of using rough
  // homography, then remove the old vwip files.
  if (boost::filesystem::exists(left_ip_file) && 
      (crop_left || crop_right || !asp::first_is_newer(left_ip_file, input_file1) ||
       !asp::stereo_settings().skip_rough_homography))
    boost::filesystem::remove(left_ip_file);
  if (boost::filesystem::exists(right_ip_file) && 
      (crop_left || crop_right || !asp::first_is_newer(right_ip_file, input_file2) ||
       !asp::stereo_settings().skip_rough_homography))
    boost::filesystem::remove(right_ip_file);
  if (boost::filesystem::exists(match_filename)) {
    vw_out() << "Removing old match file: " << match_filename << "\n";
    // It is hoped the logic before here was such that we will not
    // wipe external match files given by --match-files-prefix or
    // --clean-match-files-prefix.
    boost::filesystem::remove(match_filename);
  }

  // Create DiskImageResource objects. It is a little messy to make sure
  // it works with SPOT5 which will not work without the camera file
  // but the camera file does not match if the image is cropped.
  // Ideally there would be a function to make this cleaner.
  boost::shared_ptr<DiskImageResource> rsrc1, rsrc2;
  if (input_file1 == m_left_image_file)
    rsrc1 = vw::DiskImageResourcePtr(m_left_image_file);
  else // Tiff input
    rsrc1 = vw::DiskImageResourcePtr(input_file1);
  if (input_file2 == m_right_image_file)
    rsrc2 = vw::DiskImageResourcePtr(m_right_image_file);
  else // Tiff input
    rsrc2 = vw::DiskImageResourcePtr(input_file2);

  // Open the images
  ImageViewRef<float> image1 = DiskImageView<float>(rsrc1);
  ImageViewRef<float> image2 = DiskImageView<float>(rsrc2);
  
  // If the user provided a custom no-data value, values no more than that are
  // masked. Otherwise, only values equal to no-data are masked.
  ImageViewRef<PixelMask<float>> masked_image1, masked_image2; 
  float user_nodata = stereo_settings().nodata_value;
    if (!std::isnan(user_nodata)) {
    masked_image1 = create_mask_less_or_equal(image1, user_nodata);
    masked_image2 = create_mask_less_or_equal(image2, user_nodata);
  } else {
    // Mask the no-data values in the images
    masked_image1 = create_mask(image1, nodata1);
    masked_image2 = create_mask(image2, nodata2);
  }

  // Get normalized versions of the images for OpenCV based methods.
  // Similar logic is used in bundle_adjust per image.
  if (asp::openCvDetectMethod()) {
    vw_out() << "\t--> Normalizing images for IP detection using stats " << stats1 << "\n";
    asp::normalize_images(stereo_settings().force_use_entire_range,
                          stereo_settings().individually_normalize,
                          asp::openCvDetectMethod(),
                          asp::doNotExceedMinMax(),
                          stats1, stats2,
                          masked_image1, masked_image2);
  }

  bool have_datum = this->have_datum();

  // If cameras are null then we cannot use them
  if (cam1 == NULL || cam2 == NULL)
    have_datum = false;

  // Jobs set to 2x the number of cores. This is just in case all jobs are not equal.
  // The total number of interest points will be divided up among the jobs.
  size_t number_of_jobs = vw_settings().default_num_threads() * 2;
  if (vw_settings().default_num_threads() == 1) // the user wants one thread
    number_of_jobs = 1;

#if __APPLE__
  // Fix due to OpenBLAS crashing and/or giving different results
  // each time.
  // TODO(oalexan1): Revisit this.
  number_of_jobs = std::min(int(vw_settings().default_num_threads()), 1);
  vw_out() << "\t    Using " << number_of_jobs << " thread(s) for matching.\n";
#endif

  if (have_datum) {
    // Run an IP matching function that takes the camera and datum info into account

    bool use_sphere_for_non_earth = true; // Assume Mars is a sphere
    cartography::Datum datum = this->get_datum(cam1, use_sphere_for_non_earth);

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
        vw_out() << "Old datum: " << datum << std::endl;
        datum = vw::cartography::Datum(datum.name(),
                                       datum.spheroid_name(),
                                       datum.meridian_name(),
                                       datum.semi_major_axis() + mid_ht,
                                       datum.semi_minor_axis() + mid_ht,
                                       datum.meridian_offset());
        vw_out() << "New datum: " << datum << std::endl;
      }
    } // End RPC case

    // A smaller value here makes IP more unique, but also fewer
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
    inlier = match_ip_with_datum(!supports_multi_threading(),
                                 !stereo_settings().skip_rough_homography,
                                 cam1, cam2,
                                 apply_mask(masked_image1, nodata1),
                                 apply_mask(masked_image2, nodata2),
                                 asp::stereo_settings().ip_per_tile,
                                 datum, match_filename, number_of_jobs,
                                 epipolar_threshold, ip_uniqueness_thresh,
                                 left_ip_file, right_ip_file,
                                 nodata1, nodata2);
  } else { // No datum
    // Run a simpler purely image-based matching function
    double ip_inlier_factor = stereo_settings().ip_inlier_factor;
    // --ip-inlier-factor is 0.2 in bundle_adjust and stereo. 
    // The default value below should be 200. Lower values were observed
    // to filter too many inliers.
    int inlier_threshold = round(1000.0 * ip_inlier_factor);

    // HACK: If the otherwise unused epipolar threshold is set, use it as
    // the inlier threshold.
    // TODO(oalexan1): This may need to be removed.
    if (stereo_settings().epipolar_threshold > 0)
      inlier_threshold = stereo_settings().epipolar_threshold;

    vw_out() << "\t    Not using a datum in interest point matching.\n";
    bool use_cached_ip = true; // When ip should not be cached they were already wiped
    inlier = homography_ip_matching(apply_mask(masked_image1, nodata1),
                                    apply_mask(masked_image2, nodata2),
                                    asp::stereo_settings().ip_per_tile,
                                    inlier_threshold,
                                    match_filename, number_of_jobs,
                                    left_ip_file, right_ip_file,
                                    use_cached_ip,
                                    nodata1, nodata2, bbox1, bbox2);
  }

  } catch (std::exception const& e) {
    err = e.what();
  }

  if (!inlier || err != "") {
    boost::filesystem::remove(match_filename);

    std::string msg = "Unable to find enough interest point matches in the images. Check if the images are similar enough in illumination and if they have enough overlap.\n";
    if (err != "")
      msg += "A more technical error message is as follows.\n" + err;

    vw_throw(IOErr() << msg);
  }

  return inlier;
} // End function ip_matching()

// Find ip matches and determine the alignment matrices
void StereoSession::imageAlignment(// Inputs
                                   std::string  const& out_prefix,
                                   std::string  const& left_cropped_file,
                                   std::string  const& right_cropped_file,
                                   std::string  const& left_uncropped_file,
                                   vw::Vector6f const& left_stats,
                                   vw::Vector6f const& right_stats,
                                   float left_nodata_value,
                                   float right_nodata_value,
                                   vw::CamPtr left_cam,
                                   vw::CamPtr right_cam,
                                   bool adjust_left_image_size,
                                   // In-out
                                   vw::Matrix<double> & align_left_matrix,
                                   vw::Matrix<double> & align_right_matrix,
                                   vw::Vector2i & left_size,
                                   vw::Vector2i & right_size) {

  // Define the file name containing IP match information.
  std::string match_filename
    = asp::stereo_match_filename(left_cropped_file, right_cropped_file, out_prefix);

  std::string left_ip_filename  = ip::ip_filename(out_prefix, left_cropped_file);
  std::string right_ip_filename = ip::ip_filename(out_prefix, right_cropped_file);

  // Detect matching interest points between the left and right input images.
  // The output is written directly to a file.
  DiskImageView<float> left_orig_image(left_uncropped_file);
  vw::Vector2 uncropped_left_image_size = bounding_box(left_orig_image).size();
  this->ip_matching(left_cropped_file, right_cropped_file,
                    uncropped_left_image_size,
                    left_stats, right_stats,
                    left_nodata_value, right_nodata_value,
                    left_cam.get(), right_cam.get(),
                    match_filename, left_ip_filename, right_ip_filename);

  // Load the interest points results from the file we just wrote
  std::vector<ip::InterestPoint> left_ip, right_ip;
  ip::read_binary_match_file(match_filename, left_ip, right_ip);

  // Compute the appropriate alignment matrix based on the input points
  if (stereo_settings().alignment_method == "homography") {
    bool tight_inlier_threshold = false;
    left_size = homography_rectification(adjust_left_image_size, tight_inlier_threshold,
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
  write_matrix_as_txt(out_prefix + "-align-L.txt", align_left_matrix);
  write_matrix_as_txt(out_prefix + "-align-R.txt", align_right_matrix);

  // Because the images are now aligned they are the same size
  right_size = left_size;
}

// A wrapper around ip matching. Can also work with NULL cameras. Various settings
// are passed in via asp::stereo_settings().
void matchIp(std::string const& out_prefix,
             bool enable_rough_homography,
             double pct_for_overlap,
             asp::SessionPtr session,
             std::string const& image1_path,  std::string const& image2_path,
             vw::camera::CameraModel* cam1,   vw::camera::CameraModel* cam2,
             std::string const& match_filename) {

  boost::shared_ptr<DiskImageResource>
    rsrc1(vw::DiskImageResourcePtr(image1_path)),
    rsrc2(vw::DiskImageResourcePtr(image2_path));
  if ((rsrc1->channels() > 1) || (rsrc2->channels() > 1))
    vw_throw(ArgumentErr()
             << "Error: Input images can only have a single channel!\n\n");
  float nodata1, nodata2;
  asp::get_nodata_values(rsrc1, rsrc2, asp::stereo_settings().nodata_value, nodata1, nodata2);

  // IP matching may not succeed for all pairs

  // Get masked views of the images to get statistics from. If the user provided
  // a custom no-data value, values no more than that are masked. Otherwise only 
  // the exact no-data values are masked.
  ImageViewRef<float> image1_view = DiskImageView<float>(rsrc1);
  ImageViewRef<float> image2_view = DiskImageView<float>(rsrc2);
  ImageViewRef<PixelMask<float>> masked_image1, masked_image2;
  float user_nodata = asp::stereo_settings().nodata_value;
  if (!std::isnan(user_nodata)) {
    masked_image1 = create_mask_less_or_equal(image1_view, user_nodata);
    masked_image2 = create_mask_less_or_equal(image2_view, user_nodata);
  } else {
    masked_image1 = create_mask(image1_view, nodata1);
    masked_image2 = create_mask(image2_view, nodata2);
  }

  // Since we computed statistics earlier, this will just be loading files.
  vw::Vector<vw::float32,6> image1_stats, image2_stats;
  image1_stats = asp::gather_stats(masked_image1, image1_path,
                                   out_prefix, image1_path,
                                   asp::stereo_settings().force_reuse_match_files);
  image2_stats = asp::gather_stats(masked_image2, image2_path,
                                   out_prefix, image2_path,
                                   asp::stereo_settings().force_reuse_match_files);

  // Rough homography cannot use / cache vwip
  std::string vwip_file1, vwip_file2; 
  if (enable_rough_homography) {
    vw::vw_out(vw::WarningMessage)
      << "Option --enable-rough-homography is set. Will not save interest "
      << "point matches per image before matching (vwip), as these depend "
      << "on the pair of images used.\n";
  } else {
    vwip_file1 = ip::ip_filename(out_prefix, image1_path);
    vwip_file2 = ip::ip_filename(out_prefix, image2_path);
  }
  
  // For mapprojected images and given the overlap params,
  // can restrict the matching to a smaller region.
  vw::BBox2 bbox1, bbox2;
  if (pct_for_overlap >= 0 && cam1 == NULL && cam2 == NULL) {
    vw::cartography::GeoReference georef1, georef2;
    bool has_georef1 = vw::cartography::read_georeference(georef1, image1_path);
    bool has_georef2 = vw::cartography::read_georeference(georef2, image2_path);
    if (has_georef1 && has_georef2) {
      bbox1 = vw::bounding_box(masked_image1);
      bbox2 = vw::bounding_box(masked_image2);
      // Expand the boxes by pct
      expand_box_by_pct(bbox1, pct_for_overlap);
      expand_box_by_pct(bbox2, pct_for_overlap);
      // Transform each box to the other image's pixel coordinates
      vw::cartography::GeoTransform trans(georef1, georef2);
      vw::BBox2 trans_bbox1 = trans.forward_bbox(bbox1);
      vw::BBox2 trans_bbox2 = trans.reverse_bbox(bbox2);
      // The first box will be the 2nd transformed box, then cropped
      // to bounding box of first image
      bbox1 = trans_bbox2;
      bbox1.crop(bounding_box(masked_image1));
      // Same logic for the second box.
      bbox2 = trans_bbox1;
      bbox2.crop(bounding_box(masked_image2));
    }
  }

  // The match files (.match) are cached unless the images or camera
  // are newer than them.
  session->ip_matching(image1_path, image2_path,
                       Vector2(masked_image1.cols(), masked_image1.rows()),
                       image1_stats, image2_stats,
                       nodata1, nodata2, cam1, cam2, match_filename,
                       vwip_file1, vwip_file2, bbox1, bbox2);
}

} // End namespace asp
