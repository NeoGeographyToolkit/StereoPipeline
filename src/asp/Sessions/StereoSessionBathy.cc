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

/// \file StereoSessionBathy.cc
///

#include <asp/Sessions/StereoSession.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Bathymetry.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/AlignmentUtils.h>

#include <vw/Core/Log.h>
#include <vw/Image/PixelMask.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Math/Geometry.h>

#include <boost/filesystem/operations.hpp>

using namespace vw;
using namespace vw::cartography;

namespace asp {

// A little function whose goal is to avoid repeating same logic in a handful of places
void crop_bathy_mask(vw::GdalWriteOptions const& options,
                     std::string const& input_mask_file, std::string const& input_image_file,
                     BBox2i const& crop_win, std::string const& cropped_mask_file) {

  if (input_mask_file == "")
    vw_throw(ArgumentErr() << "Required bathy mask file was not specified.");

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

void StereoSession::read_aligned_bathy_masks
(vw::ImageViewRef<vw::PixelMask<float>> & left_aligned_bathy_mask_image,
 vw::ImageViewRef<vw::PixelMask<float>> & right_aligned_bathy_mask_image) {

  float left_nodata = -std::numeric_limits<float>::max();
  float right_nodata = -std::numeric_limits<float>::max();
  left_aligned_bathy_mask_image = vw::read_bathy_mask(left_aligned_bathy_mask(), left_nodata);
  right_aligned_bathy_mask_image = vw::read_bathy_mask(right_aligned_bathy_mask(), right_nodata);
}

std::string StereoSession::left_cropped_bathy_mask() const {
  if (!asp::doBathy(asp::stereo_settings()))
    vw_throw(ArgumentErr() << "The left cropped bathy mask is requested when "
              << "bathymetry mode is not on.");

  bool crop_left = do_crop_left();
  if (!crop_left)
    return stereo_settings().left_bathy_mask;

  return this->m_out_prefix + "-L_cropped_bathy_mask.tif";
}

std::string StereoSession::right_cropped_bathy_mask() const {
  if (!asp::doBathy(asp::stereo_settings()))
    vw_throw(ArgumentErr() << "The right cropped bathy mask is requested when "
              << "bathymetry mode is not on.");

  bool crop_right = do_crop_right();
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

// Align the bathy masks. This will be called in stereo_pprc and, if
// needed, in stereo_tri. Skip this if the masks already exit and are
// not older than the images. This code mirrors very closely the logic
// for how the images are aligned.
// TODO(oalexan1): Such duplication of logic is not good.
void StereoSession::align_bathy_masks(vw::GdalWriteOptions const& options) {

  bool do_bathy = asp::doBathy(asp::stereo_settings());

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

  bool crop_left  = do_crop_left();
  bool crop_right = do_crop_right();

  bool rebuild = (!first_is_newer(left_aligned_bathy_mask(), check_files) ||
                  !first_is_newer(right_aligned_bathy_mask(), check_files));

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
  Vector2i left_size;
  std::string left_aligned_file = this->m_out_prefix + "-L.tif";
  if (boost::filesystem::exists(left_aligned_file))
    left_size = file_image_size(left_aligned_file);
  else
    vw_throw(NoImplErr() << "Could not read: " << left_aligned_file);

  // Read alignment matrices
  vw::Matrix<double> align_left_matrix
   = asp::alignmentMatrix(this->m_out_prefix, asp::stereo_settings().alignment_method,
                              "left");
  vw::Matrix<double> align_right_matrix
   = asp::alignmentMatrix(this->m_out_prefix, asp::stereo_settings().alignment_method,
                              "right");

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
  bool has_left_georef  = read_georeference(left_georef,
                                            this->left_cropped_image(crop_left));
  bool has_right_georef = read_georeference(right_georef,
                                            this->right_cropped_image(crop_right));

  std::string left_aligned_bathy_mask_file = StereoSession::left_aligned_bathy_mask();
  vw_out() << "\t--> Writing: " << left_aligned_bathy_mask_file << "\n";
  block_write_gdal_image(left_aligned_bathy_mask_file,
                         apply_mask(left_aligned_bathy_mask, left_bathy_nodata),
                         has_left_georef, left_georef,
                         has_bathy_nodata, left_bathy_nodata, options,
                         TerminalProgressCallback("asp","\t  L bathy mask:  "));

  // Use same logic as when the right aligned image is written
  if (stereo_settings().alignment_method == "none") {
    std::string right_aligned_bathy_mask_file = StereoSession::right_aligned_bathy_mask();
    vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << "\n";
    block_write_gdal_image(right_aligned_bathy_mask_file,
                           apply_mask(right_aligned_bathy_mask, right_bathy_nodata),
                           has_right_georef, right_georef,
                           has_bathy_nodata, right_bathy_nodata, options,
                           TerminalProgressCallback("asp","\t  R bathy mask:  "));
  } else {
    std::string right_aligned_bathy_mask_file = StereoSession::right_aligned_bathy_mask();
    vw_out() << "\t--> Writing: " << right_aligned_bathy_mask_file << "\n";
    block_write_gdal_image(right_aligned_bathy_mask_file,
                           apply_mask(crop(edge_extend(right_aligned_bathy_mask,
                                                       bathy_ext_nodata),
                                           // Note how we use the left aligned mask bbox
                                           bounding_box(left_aligned_bathy_mask)),
                                      right_bathy_nodata),
                           has_right_georef, right_georef,
                           has_bathy_nodata, right_bathy_nodata,
                           options,
                           TerminalProgressCallback("asp","\t  R bathy mask:  "));
  }
}

void StereoSession::read_bathy_masks(float & left_bathy_nodata,
                                     float & right_bathy_nodata,
                                     vw::ImageViewRef<vw::PixelMask<float>> & left_bathy_mask,
                                     vw::ImageViewRef<vw::PixelMask<float>> & right_bathy_mask) {

  left_bathy_mask = vw::read_bathy_mask(left_cropped_bathy_mask(), left_bathy_nodata);
  right_bathy_mask = vw::read_bathy_mask(right_cropped_bathy_mask(), right_bathy_nodata);

  // The left image (after crop) better needs to have the same dims
  // as the left mask after crop, and same for the right
  bool crop_left  = do_crop_left();
  bool crop_right = do_crop_right();
  DiskImageView<float> left_image(this->left_cropped_image(crop_left));
  DiskImageView<float> right_image(this->right_cropped_image(crop_right));
  if (left_bathy_mask.cols() != left_image.cols()   ||
      left_bathy_mask.rows() != left_image.rows()   ||
      right_bathy_mask.cols() != right_image.cols() ||
      right_bathy_mask.rows() != right_image.rows()) {
    vw_throw(ArgumentErr() << "The dimensions of bathymetry masks don't agree "
              << "with the image sizes (after crop win, if applicable).");
  }
}

} // End namespace asp
