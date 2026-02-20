// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file StereoSessionIo.cc
///

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/IsisIO/IsisSpecialPixels.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Bathymetry.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/AlignmentUtils.h>

#include <vw/Core/Log.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Image/PixelMask.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Math/Geometry.h>

using namespace vw;
using namespace vw::cartography;

namespace asp {

// Find the masked images and stats. Handles ISIS special pixels if needed.
void StereoSession::
calcStatsMaskedImages(// Inputs
                      vw::ImageViewRef<float> const& left_cropped_image,
                      vw::ImageViewRef<float> const& right_cropped_image,
                      float left_nodata_value, float right_nodata_value,
                      std::string const& left_input_file,
                      std::string const& right_input_file,
                      std::string const& left_cropped_file,
                      std::string const& right_cropped_file,
                      // Outputs
                      vw::ImageViewRef<vw::PixelMask<float>> & left_masked_image,
                      vw::ImageViewRef<vw::PixelMask<float>> & right_masked_image,
                      vw::Vector6f & left_stats,
                      vw::Vector6f & right_stats) const {

  // Form the masked imagess
  left_masked_image = create_mask(left_cropped_image, left_nodata_value);
  right_masked_image = create_mask(right_cropped_image, right_nodata_value);

  // Handle ISIS special pixels, for the isis session only. May need to use for
  // csm and isis mapprojected images.
  bool isIsis = (this->name() == "isis");
  if (isIsis) {
    adjustIsisImage(left_input_file, left_nodata_value, left_masked_image);
    adjustIsisImage(right_input_file, right_nodata_value, right_masked_image);
  }

  // In --stereo-dist-mode, want the stats from the original images which should already exist
  std::string left_file, right_file;
  if (!asp::stereo_settings().stereo_dist_mode) {
    left_file = left_cropped_file;
    right_file = right_cropped_file;
  } else {
    left_file = left_input_file;
    right_file = right_input_file;
  }

  // For ISIS, do not exceed min and max as there could be special pixels
  bool adjust_min_max_with_std = isIsis && !stereo_settings().force_use_entire_range;

  // Compute input image statistics
  vw::Stopwatch sw1;
  sw1.start();
  left_stats = gather_stats(left_masked_image,
                            this->m_out_prefix, left_file,
                            asp::stereo_settings().force_reuse_match_files,
                            adjust_min_max_with_std);
  sw1.stop();
  vw_out() << "Left image stats time: " << sw1.elapsed_seconds() << "\n";
  vw::Stopwatch sw2;
  sw2.start();
  right_stats = gather_stats(right_masked_image,
                             this->m_out_prefix, right_file,
                             asp::stereo_settings().force_reuse_match_files,
                             adjust_min_max_with_std);
  sw2.stop();
  vw_out() << "Right image stats time: " << sw2.elapsed_seconds() << "\n";
}

// Default preprocessing hook. Some sessions may override it.
void StereoSession::preprocessing_hook(bool adjust_left_image_size,
                                       std::string const& left_input_file,
                                       std::string const& right_input_file,
                                       std::string      & left_output_file,
                                       std::string      & right_output_file) {

  // Handle skip_image_normalization mode. Create symlinks instead of normalized
  // images, but still compute stats further down (needed for subpixel modes 2
  // and 3). The validation for this mode was done earlier in stereo_pprc.cc.
  if (stereo_settings().skip_image_normalization) {
    vw_out(WarningMessage) << "Skipping image normalization.\n";
    createSymLinks(left_input_file, right_input_file, m_out_prefix,
                   left_output_file, right_output_file);
  }

  std::string left_cropped_file, right_cropped_file;
  ImageViewRef<float> left_cropped_image, right_cropped_image;
  vw::GdalWriteOptions options;
  float left_nodata_value, right_nodata_value;
  bool has_left_georef, has_right_georef;
  vw::cartography::GeoReference left_georef, right_georef;
  bool exit_early =
    StereoSession::prepareInputImages(options,
                                      left_input_file,    right_input_file,
                                      left_output_file,   right_output_file,
                                      left_cropped_file,  right_cropped_file,
                                      left_cropped_image, right_cropped_image,
                                      left_nodata_value,  right_nodata_value,
                                      has_left_georef,    has_right_georef,
                                      left_georef,        right_georef);

  if (exit_early)
    return;

  // For skip_image_normalization, use L.tif/R.tif paths for stats file naming
  // so that later stages (stereo_rfne) can find them.
  if (stereo_settings().skip_image_normalization) {
    left_cropped_file  = left_output_file;
    right_cropped_file = right_output_file;
  }

  ImageViewRef<PixelMask<float>> left_masked_image, right_masked_image;
  Vector6f left_stats, right_stats;

  // Set up the image masks and compute the stats. If the user provided a custom
  // no-data value, values no more than that have been masked by now in
  // prepareInputImages.
  this->calcStatsMaskedImages(// Inputs
                              left_cropped_image, right_cropped_image,
                              left_nodata_value, right_nodata_value,
                              left_input_file, right_input_file,
                              left_cropped_file, right_cropped_file,
                              // Outputs
                              left_masked_image, right_masked_image,
                              left_stats, right_stats);

  // If the user only wanted stats, stop here. Doing exit(0) seems to be simpler
  // than tracing back a flag. This is needed for stereo_dist.
  if (stereo_settings().stop_after_stats) {
    vw_out() << "Stopping after computing image statistics.\n";
    exit(0);
  }

  // For skip_image_normalization, we created symlinks above and computed stats.
  // No alignment or image writing is needed.
  if (stereo_settings().skip_image_normalization)
    return;

  // Initialize the alignment matrices
  Matrix<double> align_left_matrix  = math::identity_matrix<3>();
  Matrix<double> align_right_matrix = math::identity_matrix<3>();

  ImageViewRef<PixelMask<float>> Limg, Rimg;
  bool isis_session = (this->name() == "isis" || this->name() == "isismapisis");

  // Use no-data in interpolation and edge extension
  // TODO(oalexan1): Maybe using 0 for nodata_pix is not good. May need to use
  // -32768.0.
  PixelMask<float>nodata_pix(0); nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> ext_nodata(nodata_pix);

  // Generate aligned versions of the input images according to the options
  vw_out() << "\t--> Applying alignment method: "
           << stereo_settings().alignment_method << "\n";
  if (stereo_settings().alignment_method == "epipolar") {
    if (isis_session)
      vw_throw(NoImplErr() << "StereoSessionISIS does not support epipolar rectification");
    epipolar_alignment(left_masked_image, right_masked_image, ext_nodata,
                       // Outputs
                       Limg, Rimg);

  } else if (stereo_settings().alignment_method == "homography"     ||
             stereo_settings().alignment_method == "affineepipolar" ||
             stereo_settings().alignment_method == "local_epipolar") {

    // Load the cameras
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    this->camera_models(left_cam, right_cam);

    // Get the image sizes. Later alignment options can choose to
    // change this parameters (such as affine epipolar alignment).
    Vector2i left_size(left_cropped_image.cols(), left_cropped_image.rows());
    Vector2i right_size(right_cropped_image.cols(), right_cropped_image.rows());

    imageAlignment(// Inputs
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
    if (!asp::stereo_settings().stereo_dist_mode) {
      Limg = left_masked_image;
      Rimg = right_masked_image;
    } else {
      // Do the delayed cropping. This allows writing the cropped images
      // only after normalization avoiding an unnecessary write.
      Limg = crop(left_masked_image, asp::stereo_settings().left_image_crop_win);
      Rimg = crop(right_masked_image, asp::stereo_settings().right_image_crop_win);
      // Also crop the georefs
      left_georef = crop(left_georef, asp::stereo_settings().left_image_crop_win);
      right_georef = crop(right_georef, asp::stereo_settings().right_image_crop_win);
    }
  } // End of image alignment block

  // Apply our normalization options.
  bool use_percentile_stretch = false;
  bool do_not_exceed_min_max = isis_session; // for isis, do not exceed min/max
  // TODO(oalexan1): Should one add above "csm" and "csmmapcsm" / "csmmaprpc"?
  asp::normalize_images(stereo_settings().force_use_entire_range,
                        stereo_settings().individually_normalize,
                        use_percentile_stretch,
                        do_not_exceed_min_max,
                        left_stats, right_stats, Limg, Rimg);

  // The output no-data value must be < 0 as we scale the images to [0, 1].
  bool has_nodata = true;
  float output_nodata = -32768.0;
  vw_out() << "\t--> Writing pre-aligned images.\n";
  vw_out() << "\t--> Writing: " << left_output_file << "\n";
  vw::Stopwatch sw3;
  sw3.start();
  block_write_gdal_image(left_output_file, apply_mask(Limg, output_nodata),
                         has_left_georef, left_georef,
                         has_nodata, output_nodata, options,
                         TerminalProgressCallback("asp","\t  L:  "));
  sw3.stop();
  vw_out() << "Writing left image elapsed time: " << sw3.elapsed_seconds() << " s\n";

  vw_out() << "\t--> Writing: " << right_output_file << "\n";
  vw::Stopwatch sw4;
  sw4.start();

  // With no alignment, do not crop the right image to have the same dimensions
  // as the left image. Since there is no alignment, and images may not be
  // georeferenced, we do not know what portion of the right image corresponds
  // best to the left image, so cropping may throw away an area where the left
  // and right images overlap.
  if (stereo_settings().alignment_method != "none")
    Rimg = crop(edge_extend(Rimg, ext_nodata), bounding_box(Limg));

  block_write_gdal_image(right_output_file, apply_mask(Rimg, output_nodata),
                         has_right_georef, right_georef,
                         has_nodata, output_nodata, options,
                         TerminalProgressCallback("asp","\t  R:  "));
  sw4.stop();
  vw_out() << "Writing right image elapsed time: " << sw4.elapsed_seconds() << " s\n";

  // For bathy runs only
  if (asp::doBathy(asp::stereo_settings()))
    this->align_bathy_masks(options);

} // End function preprocessing_hook


bool StereoSession::prepareInputImages(vw::GdalWriteOptions          & options,
                                       std::string const             & left_input_file,
                                       std::string const             & right_input_file,
                                       std::string                   & left_output_file,
                                       std::string                   & right_output_file,
                                       std::string                   & left_cropped_file,
                                       std::string                   & right_cropped_file,
                                       vw::ImageViewRef<float>       & left_cropped_image,
                                       vw::ImageViewRef<float>       & right_cropped_image,
                                       float                         & left_nodata_value,
                                       float                         & right_nodata_value,
                                       bool                          & has_left_georef,
                                       bool                          & has_right_georef,
                                       vw::cartography::GeoReference & left_georef,
                                       vw::cartography::GeoReference & right_georef) {

  // Retrieve nodata values and let the handles go out of scope right away.
  // For this to work the ISIS type must be registered with the
  // DiskImageResource class. This happens in "stereo.cc", so
  // these calls will create DiskImageResourceIsis objects.
  {
    boost::shared_ptr<DiskImageResource>
      left_rsrc (DiskImageResourcePtr(left_input_file)),
      right_rsrc(DiskImageResourcePtr(right_input_file));
    asp::get_nodata_values(left_rsrc, right_rsrc,
                           stereo_settings().nodata_value,
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
  if (stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef = false;
    has_right_georef = false;
  }

  bool crop_left  = do_crop_left();
  bool crop_right = do_crop_right();

  // For --stereo-dist-mode will do the cropping when we normalize to avoid
  // an extra write.
  if (asp::stereo_settings().stereo_dist_mode) {
    crop_left = false;
    crop_right = false;
  }

  // Here either the input image or the cropped images will be returned,
  // depending on whether the crop actually happens
  left_cropped_file = this->left_cropped_image(crop_left);
  right_cropped_file = this->right_cropped_image(crop_right);

  // If the output files already exist and are newer than the input files, and
  // we don't crop both left and right images, then there is nothing to do here.
  // Note: Must make sure all outputs are initialized before we get to this part
  // where we exit early. For stereo dist mode the L.tif and R.tif do not exist
  // yet as those would be per-tile.
  std::vector<std::string> check_files;
  check_files.push_back(left_input_file);
  check_files.push_back(right_input_file);
  check_files.push_back(m_left_camera_file);
  check_files.push_back(m_right_camera_file);
  bool rebuild = (!asp::stereo_settings().stereo_dist_mode)  &&
                 (!first_is_newer(left_output_file, check_files) ||
                  !first_is_newer(right_output_file, check_files));
  bool do_bathy = asp::doBathy(asp::stereo_settings());
  if (do_bathy) {
    rebuild = (rebuild ||
               (!first_is_newer(left_aligned_bathy_mask(), check_files) ||
                !first_is_newer(right_aligned_bathy_mask(), check_files)));
  }

  // Consider the case of multi-band images
  if (!rebuild) {
    int lc = vw::get_num_channels(left_input_file);
    int rc = vw::get_num_channels(right_input_file);
    if (lc > 1 || rc > 1) {
      vw_out(vw::WarningMessage)
        << "Always recomputing the inputs for multi-band images as the "
        << "provided band may change.\n";
      rebuild = true;
    }
  }

  // These will throw if the files do not exist
  if (!rebuild && !crop_left && !crop_right) {
    try {
      vw_log().console_log().rule_set().add_rule(-1, "fileio");
      if (!asp::stereo_settings().stereo_dist_mode) {
        DiskImageView<PixelGray<float32>> out_left (left_output_file);
        DiskImageView<PixelGray<float32>> out_right(right_output_file);
      }

      if (do_bathy) {
        DiskImageView<float> left_bathy_mask (left_aligned_bathy_mask());
        DiskImageView<float> right_bathy_mask(right_aligned_bathy_mask());
      }

      if (!asp::stereo_settings().stereo_dist_mode)
        vw_out(InfoMessage) << "\t--> Using cached normalized input images.\n";
      vw_settings().reload_config();
      if (!asp::stereo_settings().stereo_dist_mode)
        return true; // Return true if we exist early since the images exist
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  // Load the desired band. Subtract 1 to make it start from 0.
  int ch = asp::stereo_settings().band - 1;
  ImageViewRef<float> left_orig_image = vw::read_channel<float>(left_input_file, ch);
  ImageViewRef<float> right_orig_image = vw::read_channel<float>(right_input_file, ch);

  // If the user provided a custom no-data value, values no more than that are
  // masked.
  float user_nodata = stereo_settings().nodata_value;
  if (!std::isnan(user_nodata)) {
    left_orig_image = apply_mask(create_mask_less_or_equal(left_orig_image, user_nodata),
                                  user_nodata);
    right_orig_image = apply_mask(create_mask_less_or_equal(right_orig_image, user_nodata),
                                   user_nodata);
  }

  // See if to crop the images
  if (crop_left) {
    // Crop and save the left image to left_cropped_file
    has_left_georef = read_georeference(left_georef, left_input_file);
    bool has_nodata = true;
    BBox2i left_win = stereo_settings().left_image_crop_win;
    left_win.crop(bounding_box(left_orig_image));
    // Return a handle to the cropped image
    left_cropped_image = crop(left_orig_image, left_win);

    if (stereo_settings().left_image_clip != "") {
      // Replace the crop with a given clip. This is a very rarely used option.
      // It can be handy when investigating CCD artifacts correction.
      left_cropped_image = DiskImageView<float>(stereo_settings().left_image_clip);
      if (left_cropped_image.cols() != left_win.width() ||
          left_cropped_image.rows() != left_win.height()) {
        vw_throw(ArgumentErr() << "The image specified via --left-image-clip has different "
                  << "dimensions than set via --left-image-crop-win.");
      }
    }

    // Crop the georef as well
    vw_out() << "\t--> Writing cropped image: " << left_cropped_file << "\n";
    block_write_gdal_image(left_cropped_file,
                           left_cropped_image,
                           has_left_georef, crop(left_georef, left_win),
                           has_nodata, left_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  } else {
    // Return a handle to the desired channel of the input image
    left_cropped_image = left_orig_image;
  }

  if (crop_right) {
    // Crop the right image and write to right_cropped_file
    has_right_georef = read_georeference(right_georef, right_input_file);
    bool has_nodata = true;
    BBox2i right_win = stereo_settings().right_image_crop_win;
    right_win.crop(bounding_box(right_orig_image));
    // Return a handle to the cropped image
    right_cropped_image = crop(right_orig_image, right_win);

    if (stereo_settings().right_image_clip != "") {
      // Replace the crop with a given clip. This is a very rarely used option.
      // It can be handy when investigating CCD artifacts correction.
      right_cropped_image = DiskImageView<float>(stereo_settings().right_image_clip);
      if (right_cropped_image.cols() != right_win.width() ||
          right_cropped_image.rows() != right_win.height()) {
        vw_throw(ArgumentErr() << "The image specified via --right-image-clip has different "
                  << "dimensions than set via --right-image-crop-win.");
      }
    }

    // Crop the georef as well
    vw_out() << "\t--> Writing cropped image: " << right_cropped_file << "\n";
    block_write_gdal_image(right_cropped_file,
                           right_cropped_image,
                           has_right_georef,
                           crop(right_georef, right_win),
                           has_nodata, right_nodata_value,
                           options,
                           TerminalProgressCallback("asp", "\t:  "));
  } else {
    // Return a handle to the desired channel of the input image
    right_cropped_image = right_orig_image;
  }

  // Re-read the georef, since it may have changed above.
  has_left_georef  = read_georeference(left_georef,  left_cropped_file);
  has_right_georef = read_georeference(right_georef, right_cropped_file);
  if (stereo_settings().alignment_method != "none") {
    // If any alignment at all happens, the georef will be messed up.
    has_left_georef  = false;
    has_right_georef = false;
  }

  return false; // don't exit early
}

} // End namespace asp
