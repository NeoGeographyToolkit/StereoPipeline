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

/// \file StereoSessionGdal.cc
///
#include <vw/Image/ImageMath.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Transform.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Math/Matrix.h>
#include <vw/Cartography/Datum.h>

#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Sessions/StereoSessionGdal.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPC_XML.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace vw;
using namespace asp;

namespace asp {

  void StereoSessionGdal::pre_preprocessing_hook(bool adjust_left_image_size,
                                                 std::string const& left_input_file,
                                                 std::string const& right_input_file,
                                                 std::string      & left_output_file,
                                                 std::string      & right_output_file) {
    
    using namespace vw;
    
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

    if (exit_early) return;

    // Load the cropped images
    DiskImageView<float> left_disk_image (left_cropped_file),
      right_disk_image(right_cropped_file);

    // Set up image masks
    ImageViewRef< PixelMask<float> > left_masked_image
      = create_mask_less_or_equal(left_disk_image,  left_nodata_value);
    ImageViewRef< PixelMask<float> > right_masked_image
      = create_mask_less_or_equal(right_disk_image, right_nodata_value);

    ImageViewRef< PixelMask<float> > left_bathy_mask, right_bathy_mask;
    bool do_bathy = StereoSession::do_bathymetry();
    float left_bathy_nodata = -std::numeric_limits<float>::max();
    float right_bathy_nodata = -std::numeric_limits<float>::max();
    if (do_bathy) 
      StereoSession::read_bathy_masks(left_masked_image, right_masked_image,
                                      left_bathy_nodata, right_bathy_nodata,
                                      left_bathy_mask, right_bathy_mask);

    // Compute input image statistics
    Vector6f left_stats  = gather_stats(left_masked_image,  "left",
                                        this->m_out_prefix, left_cropped_file);
    Vector6f right_stats = gather_stats(right_masked_image, "right",
                                        this->m_out_prefix, right_cropped_file);

    ImageViewRef< PixelMask<float> > Limg, Rimg;
    ImageViewRef< PixelMask<float> > left_aligned_bathy_mask, right_aligned_bathy_mask;

    // Image alignment block - Generate aligned versions of the input
    // images according to the options.
  if (stereo_settings().alignment_method == "homography"     ||
      stereo_settings().alignment_method == "affineepipolar" ||
      stereo_settings().alignment_method == "local_epipolar") {
      // Define the file name containing IP match information.
      std::string match_filename    = ip::match_filename(this->m_out_prefix,
                                                         left_cropped_file, right_cropped_file);
      std::string left_ip_filename  = ip::ip_filename(this->m_out_prefix, left_cropped_file);
      std::string right_ip_filename = ip::ip_filename(this->m_out_prefix, right_cropped_file);

      // Detect matching interest points between the left and right input images.
      // - The output is written directly to file!
      DiskImageView<float> left_orig_image(left_input_file);
      boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
      this->camera_models(left_cam, right_cam);
      this->ip_matching(left_cropped_file, right_cropped_file,
                        bounding_box(left_orig_image).size(),
                        left_stats, right_stats,
                        stereo_settings().ip_per_tile,
                        left_nodata_value, right_nodata_value,
                        left_cam.get(), right_cam.get(),
                        match_filename, left_ip_filename, right_ip_filename);

      // Load the interest points results from the file we just wrote.
      std::vector<ip::InterestPoint> left_ip, right_ip;
      ip::read_binary_match_file(match_filename, left_ip, right_ip);

      // Initialize alignment matrices and get the input image sizes.
      Matrix<double> align_left_matrix  = math::identity_matrix<3>(),
        align_right_matrix = math::identity_matrix<3>();
      Vector2i left_size  = file_image_size(left_cropped_file),
        right_size = file_image_size(right_cropped_file);

      // Compute the appropriate alignment matrix based on the input points
      if (stereo_settings().alignment_method == "homography") {
        left_size = homography_rectification(adjust_left_image_size,
                                             left_size,         right_size,
                                             left_ip,           right_ip,
                                             align_left_matrix, align_right_matrix);
        vw_out() << "\t--> Aligning right image to left using matrices:\n"
                 << "\t      " << align_left_matrix  << "\n"
                 << "\t      " << align_right_matrix << "\n";
      } else {
        // affineepipolar and local_epipolar
        left_size = affine_epipolar_rectification(left_size,         right_size,
                                                  stereo_settings().global_alignment_threshold,
                                                  stereo_settings().alignment_num_ransac_iterations,
                                                  left_ip,           right_ip,
                                                  align_left_matrix, align_right_matrix);
        vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
                 << "\t      " << submatrix(align_left_matrix, 0,0,2,3) << "\n"
                 << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
      }
      // Write out both computed matrices to disk
      write_matrix(this->m_out_prefix + "-align-L.exr", align_left_matrix);
      write_matrix(this->m_out_prefix + "-align-R.exr", align_right_matrix);

      // Apply the alignment transform to both input images
      Limg = transform(left_masked_image,
		       HomographyTransform(align_left_matrix),
		       left_size.x(), left_size.y());
      if (do_bathy) 
        left_aligned_bathy_mask = transform(left_bathy_mask,
                                            HomographyTransform(align_left_matrix),
                                            left_size.x(), left_size.y());
      
      Rimg = transform(right_masked_image,
		       HomographyTransform(align_right_matrix),
		       left_size.x(), left_size.y());
      if (do_bathy) 
        right_aligned_bathy_mask = transform(right_bathy_mask,
                                             HomographyTransform(align_right_matrix),
                                             left_size.x(), left_size.y());
      
      
    } else if (stereo_settings().alignment_method == "epipolar") {
      vw_throw(NoImplErr() << "StereoSessionGdal does not support epipolar rectification");
    } else {
      // No alignment, just provide the original files.
      Limg = left_masked_image;
      Rimg = right_masked_image;
      if (do_bathy) {
        left_aligned_bathy_mask  = left_bathy_mask;
        right_aligned_bathy_mask = right_bathy_mask;
      }
    } // End of image alignment block

    // Apply our normalization options.
    bool use_percentile_stretch = false;
    bool do_not_exceed_min_max = (this->name() == "isis" ||
                                  this->name() == "isismapisis");
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
    bool has_bathy_nodata = true;
    float output_nodata = -32768.0;

    std::string left_aligned_bathy_mask_file = StereoSession::left_aligned_bathy_mask();
    std::string right_aligned_bathy_mask_file = StereoSession::right_aligned_bathy_mask();
    
    // The left image is written out with no alignment warping.
    vw_out() << "\t--> Writing pre-aligned images.\n";
    vw_out() << "\t--> Writing: " << left_output_file << ".\n";
    block_write_gdal_image(left_output_file, apply_mask(Limg, output_nodata),
                           has_left_georef, left_georef,
                           has_nodata, output_nodata, options,
                           TerminalProgressCallback("asp","\t  L:  "));
    if (do_bathy) {
      vw_out() << "\t--> Writing: " << left_aligned_bathy_mask_file << ".\n";
      block_write_gdal_image(left_aligned_bathy_mask_file,
                             apply_mask(left_aligned_bathy_mask, left_bathy_nodata),
                             has_left_georef, left_georef,
                             has_bathy_nodata, left_bathy_nodata, options,
                             TerminalProgressCallback("asp","\t  L mask:  "));
    }
    
    if (stereo_settings().alignment_method == "none") {
      vw_out() << "\t--> Writing: " << right_output_file << ".\n";
      block_write_gdal_image(right_output_file, apply_mask(Rimg, output_nodata),
                             has_right_georef, right_georef,
                             has_nodata, output_nodata, options,
                             TerminalProgressCallback("asp","\t  R:  "));
      if (do_bathy) {
        vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << ".\n";
        block_write_gdal_image(right_aligned_bathy_mask_file,
                               apply_mask(right_aligned_bathy_mask, right_bathy_nodata),
                               has_right_georef, right_georef,
                               has_bathy_nodata, right_bathy_nodata, options,
                               TerminalProgressCallback("asp","\t  R mask:  "));
      }
      
    } else {
      // Write out the right image cropped to align with the left image.
      vw_out() << "\t--> Writing: " << right_output_file << ".\n";
      block_write_gdal_image(right_output_file,
                             apply_mask(crop(edge_extend(Rimg, ConstantEdgeExtension()),
                                             bounding_box(Limg)), output_nodata),
                             has_right_georef, right_georef,
                             has_nodata, output_nodata, options,
                             TerminalProgressCallback("asp","\t  R:  "));

      if (do_bathy) {
        vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << ".\n";
        block_write_gdal_image(right_aligned_bathy_mask_file,
                               apply_mask(crop(edge_extend(right_aligned_bathy_mask,
                                                           ConstantEdgeExtension()),
                                               bounding_box(Limg)), right_bathy_nodata),
                               has_right_georef, right_georef,
                               has_bathy_nodata, right_bathy_nodata, options,
                               TerminalProgressCallback("asp","\t  R mask:  "));
      }
    }
  } // End function pre_preprocessing_hook

} // End namespace asp
