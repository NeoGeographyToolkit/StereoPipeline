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

/// \file StereoSessionIsis.cc
///


// Vision Workbench
#include <vw/Core/Settings.h>
#include <vw/Core/Log.h>
#include <vw/Math/Functors.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/EdgeExtension.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/ImageMath.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/Transform.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/FileIO/DiskImageResourceOpenEXR.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/InterestPoint/Descriptor.h>
#include <vw/InterestPoint/Detector.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/MatrixIO.h>
#include <vw/Cartography/Datum.h>

// Stereo Pipeline
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/PhotometricOutlier.h>
#include <asp/Camera/CsmModel.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/Equation.h>
#include <asp/Sessions/StereoSessionIsis.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/math/special_functions/next.hpp> // boost::float_next
#include <boost/shared_ptr.hpp>

#include <algorithm>

using namespace vw;
using namespace vw::camera;

//#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1


// #include <asp/IsisIO/IsisCameraModel.h>
// #include <asp/IsisIO/DiskImageResourceIsis.h>
// #include <asp/IsisIO/Equation.h>

// // Boost
// #include <boost/filesystem/operations.hpp>
// #include <boost/math/special_functions/next.hpp> // boost::float_next
// #include <boost/shared_ptr.hpp>
// namespace fs = boost::filesystem;

// #include <algorithm>

// using namespace vw;
// using namespace vw::camera;
// using namespace asp;


// // Allows FileIO to correctly read/write these pixel types
// namespace vw {
//   template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };

namespace asp {

void StereoSessionIsis::pre_preprocessing_hook(bool adjust_left_image_size,
                       std::string const& left_input_file,
                       std::string const& right_input_file,
                       std::string      & left_output_file,
                       std::string      & right_output_file) {

  std::string left_cropped_file, right_cropped_file;
  vw::cartography::GdalWriteOptions options;
  float left_nodata_value, right_nodata_value;
  bool  has_left_georef,   has_right_georef;
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
  DiskImageView<float> left_disk_image (left_cropped_file),
                       right_disk_image(right_cropped_file);

  // Get the image sizes. Later alignment options can choose to change
  // this parameters, such as affine epipolar.
  Vector2i left_size  = file_image_size(left_cropped_file),
           right_size = file_image_size(right_cropped_file);

  // These variables will be true if we reduce the valid range for ISIS images
  // using the nodata value provided by the user.
  bool will_apply_user_nodata_left  = false,
       will_apply_user_nodata_right = false;

  // TODO: A lot of this normalization code should be shared with the base class!
  // Mask the pixels outside of the isis range and <= nodata.
  boost::shared_ptr<DiskImageResourceIsis>
    left_isis_rsrc (new DiskImageResourceIsis(left_input_file)),
    right_isis_rsrc(new DiskImageResourceIsis(right_input_file));
  float left_lo, left_hi, left_mean, left_std;
  float right_lo, right_hi, right_mean, right_std;
  ImageViewRef< PixelMask <float> > left_masked_image
    = find_ideal_isis_range(left_disk_image, left_isis_rsrc, left_nodata_value,
                            "left", will_apply_user_nodata_left,
                            left_lo, left_hi, left_mean, left_std);
  ImageViewRef< PixelMask <float> > right_masked_image
    = find_ideal_isis_range(right_disk_image, right_isis_rsrc, right_nodata_value,
                            "right", will_apply_user_nodata_right,
                            right_lo, right_hi, right_mean, right_std);

  // Handle mutual normalization if requested
  float left_lo_out  = left_lo,  left_hi_out  = left_hi,
	right_lo_out = right_lo, right_hi_out = right_hi;
  if (stereo_settings().individually_normalize == 0) {
    // Find the outer range of both files
    float mutual_lo = std::min(left_lo, right_lo);
    float mutual_hi = std::max(left_hi, right_hi);
    vw_out() << "\t--> Normalizing globally to: ["<<mutual_lo<<" "<<mutual_hi<<"]\n";
    // Set the individual hi/lo values to the mutual values
    left_lo_out  = mutual_lo;
    left_hi_out  = mutual_hi;
    right_lo_out = mutual_lo;
    right_hi_out = mutual_hi;
  } else{
    vw_out() << "\t--> Individually normalizing.\n";
  }

  // Fill in the stats blocks. No percentile stretch is available, so
  // just use the min and max for positions 4 and 5.
  Vector6f left_stats;
  left_stats [0] = left_lo;
  left_stats [1] = left_hi;
  left_stats [2] = left_mean;
  left_stats [3] = left_std;
  left_stats [4] = left_lo;
  left_stats [5] = left_hi;
  
  Vector6f right_stats;
  right_stats[0] = right_lo;
  right_stats[1] = right_hi;
  right_stats[2] = right_mean;
  right_stats[3] = right_std;
  right_stats[4] = right_lo;
  right_stats[5] = right_hi;

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
  
  // Image alignment block. Generate aligned versions of the input
  // images according to the options.
  Matrix<double> align_left_matrix  = math::identity_matrix<3>();
  Matrix<double> align_right_matrix = math::identity_matrix<3>();
  if (stereo_settings().alignment_method == "homography"     ||
      stereo_settings().alignment_method == "affineepipolar" ||
      stereo_settings().alignment_method == "local_epipolar") {
    
    // Define the file name containing IP match information.
    std::string match_filename    = ip::match_filename(this->m_out_prefix,
                                                       left_cropped_file, right_cropped_file);
    std::string left_ip_filename  = ip::ip_filename(this->m_out_prefix, left_cropped_file);
    std::string right_ip_filename = ip::ip_filename(this->m_out_prefix, right_cropped_file);
    
    // Find matching interest points between the two input images
    DiskImageView<float> left_orig_image(left_input_file);
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    this->camera_models(left_cam, right_cam);
    this->ip_matching(left_cropped_file,   right_cropped_file,
                      bounding_box(left_orig_image).size(),
                      left_stats, right_stats,
                      stereo_settings().ip_per_tile,
                      left_nodata_value, right_nodata_value,
                      left_cam.get(),    right_cam.get(),
                      match_filename, left_ip_filename, right_ip_filename
                    );
    // Read in the interest point data we just wrote to disk
    std::vector<ip::InterestPoint> left_ip, right_ip;
    ip::read_binary_match_file(match_filename, left_ip, right_ip);

    // Compute the appropriate transform matrix between the two input images.
    if (stereo_settings().alignment_method == "homography") {
      left_size = homography_rectification(adjust_left_image_size,
                                           left_size, right_size, left_ip, right_ip,
                                           align_left_matrix, align_right_matrix);
      vw_out() << "\t--> Aligning left and right images using matrices:\n"
               << "\t      " << align_left_matrix  << "\n"
               << "\t      " << align_right_matrix << "\n";
    } else {
      // affineepipolar and local_epipolar
      bool crop_to_shared_area = true;
      left_size
        = affine_epipolar_rectification(left_size, right_size,
                                        stereo_settings().global_alignment_threshold,
                                        stereo_settings().alignment_num_ransac_iterations,
                                        left_ip,   right_ip,
                                        crop_to_shared_area,
                                        align_left_matrix,
                                        align_right_matrix);
      vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
               << "\t      " << submatrix(align_left_matrix ,0,0,2,3) << "\n"
               << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
    }
    // Write the computed transform matrices to disk
    write_matrix(this->m_out_prefix + "-align-L.exr", align_left_matrix);
    write_matrix(this->m_out_prefix + "-align-R.exr", align_right_matrix);
    right_size = left_size; // Because the images are now aligned
                            // .. they are the same size.
  } else if (stereo_settings().alignment_method == "epipolar") {
    vw_throw(NoImplErr() << "StereoSessionISIS does not support epipolar rectification");
  } // End alignment block


  // Apply alignment and normalization
  bool will_apply_user_nodata = (will_apply_user_nodata_left || will_apply_user_nodata_right);

  // Write output images
  write_preprocessed_isis_image(options, will_apply_user_nodata,
                                 left_masked_image, left_output_file, "left",
                                 left_lo, left_hi, left_lo_out, left_hi_out,
                                 align_left_matrix, left_size,
                                 has_left_georef, left_georef);
  write_preprocessed_isis_image(options, will_apply_user_nodata,
                                 right_masked_image, right_output_file, "right",
                                 right_lo, right_hi, right_lo_out, right_hi_out,
                                 align_right_matrix, right_size,
                                 has_right_georef, right_georef);
}

// Only used with mask_flatfield option?
std::string write_shadow_mask(vw::cartography::GdalWriteOptions const& opt,
                              std::string const& output_prefix,
                              std::string const& input_image,
                              std::string const& mask_postfix) {
  // This thresholds at -25000 as the input sub4s for Apollo that I've
  // processed have a range somewhere between -32000 and +32000. -ZMM
  DiskImageView<PixelGray<float> > disk_image(input_image);
  DiskImageView<uint8> disk_mask(output_prefix + mask_postfix);
  ImageViewRef<uint8> mask = apply_mask(intersect_mask(create_mask(disk_mask),
                                        create_mask(threshold(disk_image,-25000,0,1.0))));
  std::string output_mask = output_prefix+mask_postfix.substr(0,mask_postfix.size()-4)+"Debug.tif";

  block_write_gdal_image(output_mask, mask, opt, TerminalProgressCallback("asp","\t  Shadow:"));
  return output_mask;
}

// Pre file is a pair of grayscale images.  (ImageView<PixelGray<float> >)
// Post file is a disparity map.            (ImageView<PixelMask<Vector2f> >)
void StereoSessionIsis::pre_filtering_hook(std::string const& input_file,
                     std::string      & output_file) {
  output_file = input_file;

  if (stereo_settings().mask_flatfield) {
    // ****************************************************
    // The following code is for Apollo Metric Camera ONLY!
    // (use at your own risk)
    // ****************************************************
    vw_out() << "\t--> Masking pixels that are less than 0.0. "
             << "(NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    output_file = this->m_out_prefix + "-R-masked.exr";

    std::string shadowLmask_name = write_shadow_mask(this->m_options,
                                                     this->m_out_prefix,
                                                     this->m_left_image_file,  "-lMask.tif");
    std::string shadowRmask_name = write_shadow_mask(this->m_options,
                                                     this->m_out_prefix,
                                                     this->m_right_image_file, "-rMask.tif");

    DiskImageView<uint8> shadowLmask(shadowLmask_name);
    DiskImageView<uint8> shadowRmask(shadowRmask_name);

    DiskImageView<PixelMask<Vector2f> > disparity_disk_image(input_file);
    ImageViewRef <PixelMask<Vector2f> > disparity_map
      = stereo::disparity_mask(disparity_disk_image, shadowLmask, shadowRmask);

    DiskImageResourceOpenEXR disparity_map_rsrc(output_file, disparity_map.format());
    Vector2i block_size(std::min<size_t>(vw_settings().default_tile_size(), disparity_map.cols()),
                        std::min<size_t>(vw_settings().default_tile_size(),
                                         disparity_map.rows()));
    disparity_map_rsrc.set_block_write_size(block_size);
    block_write_image(disparity_map_rsrc, disparity_map,
                      TerminalProgressCallback("asp", "\t--> Saving Mask :"));
  }
} // End function pre_filtering_hook()

/// Returns the target datum to use for a given camera model.
/// Note the parameter use_sphere_for_datum.
/// During alignment, we'd like to use the most accurate
/// non-spherical datum, hence radii[2]. However, for the purpose
/// of creating a DEM on non-Earth planets people usually just use
/// a spherrical datum, which we'll do as well.  Maybe at some
/// point this needs to change.
vw::cartography::Datum StereoSessionIsis::get_datum(const vw::camera::CameraModel* cam,
                                                    bool use_sphere_for_datum) const {
  const IsisCameraModel * isis_cam
    = dynamic_cast<const IsisCameraModel*>(vw::camera::unadjusted_model(cam));
  VW_ASSERT(isis_cam != NULL, ArgumentErr() << "StereoSessionISIS: Invalid camera.\n");

  return isis_cam->get_datum(use_sphere_for_datum);
}

// TODO(oalexan1):  Can we share more code with the DG implementation?

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionIsis::load_camera_model
      (std::string const& image_file, std::string const& camera_file, Vector2 pixel_offset) const{

  return load_adjusted_model(m_camera_loader.load_isis_camera_model(camera_file),
                            image_file, camera_file, pixel_offset);
}

// Reverse any pre-alignment that was done to the disparity.
ImageViewRef<PixelMask<Vector2f> >
StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file) {

  std::string dust_result = input_file;
  if (stereo_settings().mask_flatfield) {
    // ****************************************************
    // The following code is for Apollo Metric Camera ONLY!
    // (use at your own risk)
    // ****************************************************
    vw_out() << "\t--> Masking pixels that appear to be dust. "
             << "(NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    photometric_outlier_rejection(this->m_options, this->m_out_prefix, input_file,
                                  dust_result, stereo_settings().corr_kernel[0]);
  }
  return DiskImageView<PixelMask<Vector2f>>(dust_result);
} // End function pre_pointcloud_hook()
  
}

//#endif  // ASP_HAVE_PKG_ISISIO
