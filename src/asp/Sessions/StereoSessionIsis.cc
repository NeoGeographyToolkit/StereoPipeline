// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

// Stereo Pipeline
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/StereoSettings.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/IsisSpecialPixels.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/ImageNormalization.h>

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
#include <vw/FileIO/MatrixIO.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Cartography/Datum.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Filter.h>
#include <vw/Core/Stopwatch.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/math/special_functions/next.hpp> // boost::float_next
#include <boost/shared_ptr.hpp>

#include <algorithm>

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
namespace fs = boost::filesystem;

namespace asp {

// Process a single ISIS image to find an ideal min max. The reason we
// need to do this, is for ASP to get image intensity values in
// the range of 0-1. To some extent we are compressing the dynamic
// range, but we try to minimize that.
// TODO(oalexan1): This function must be merged gather_stats(). That will
// alow merging of ISIS and non-ISIS preprocessing_hook() functions.
ImageViewRef<PixelMask<float>>
find_ideal_isis_range(ImageViewRef<float> const& image,
                      boost::shared_ptr<DiskImageResourceIsis> isis_rsrc,
                      float nodata_value,
                      std::string const& tag,
                      Vector6f & stats) {

  float isis_lo = isis_rsrc->valid_minimum();
  float isis_hi = isis_rsrc->valid_maximum();

  // Force the low value to be greater than the nodata value
  if (!boost::math::isnan(nodata_value) && nodata_value >= isis_lo) {
    // The new lower bound is the next floating point number > nodata_value.
    isis_lo = boost::math::float_next(nodata_value);
    if (isis_hi < isis_lo)
      isis_hi = isis_lo;
  }

  ImageViewRef<PixelMask<float>> masked_image = create_mask(image, isis_lo, isis_hi);

  // TODO(oalexan1): Merge with gather_stats().
  // Calculating statistics. We subsample the images so statistics
  // only does about a million samples.
  float isis_mean, isis_std;
  {
    vw_out(InfoMessage) << "\t--> Computing statistics for " + tag + "\n";
    int stat_scale = int(ceil(sqrt(float(image.cols())*float(image.rows()) / 1000000)));
    ChannelAccumulator<math::CDFAccumulator<float>> accumulator;
    for_each_pixel(subsample(edge_extend(masked_image, ConstantEdgeExtension()),
                             stat_scale), accumulator);
    isis_lo   = accumulator.quantile(0);
    isis_hi   = accumulator.quantile(1);
    isis_mean = accumulator.approximate_mean();
    isis_std  = accumulator.approximate_stddev();
    stats[4]  = accumulator.quantile(0.02);
    stats[5]  = accumulator.quantile(0.98);

    vw_out(InfoMessage) << "\t  "+tag+": [ lo:" << isis_lo << " hi:" << isis_hi
                        << " m: " << isis_mean << " s: " << isis_std <<  "]\n";
  }

  // Normalizing to -+2 sigmas around mean
  if (stereo_settings().force_use_entire_range == 0) {
    vw_out(InfoMessage) << "\t--> Adjusting hi and lo to -+2 sigmas around mean.\n";

    // Do not exceed isis_lo and isis_hi as there we may have special pixels
    if (isis_lo < isis_mean - 2*isis_std)
      isis_lo = isis_mean - 2*isis_std;
    if (isis_hi > isis_mean + 2*isis_std)
      isis_hi = isis_mean + 2*isis_std;

    vw_out(InfoMessage) << "\t    "+tag+" changed: [ lo:"
                        << isis_lo << " hi:" << isis_hi << "]\n";
  }

  stats[0] = isis_lo;
  stats[1] = isis_hi;
  stats[2] = isis_mean;
  stats[3] = isis_std;

  // Remove any special pixels
  // TODO(oalexan1): This should move higher up, before statistics calculation. 
  ImageViewRef<float> processed_image = vw::apply_mask(masked_image, isis_lo);
  processed_image = remove_isis_special_pixels(processed_image, isis_lo, isis_hi, isis_lo);
  masked_image = vw::create_mask(processed_image, isis_lo);
  
  return masked_image;
}

// This actually modifies and writes the pre-processed image.
void write_preprocessed_isis_image(vw::GdalWriteOptions const& opt,
                                   bool apply_user_nodata,
                                   ImageViewRef<PixelMask <float>> masked_image,
                                   std::string const& out_file,
                                   std::string const& tag,
                                   float isis_lo, float isis_hi,
                                   float out_lo,  float out_hi,
                                   Matrix<double> const& matrix,
                                   Vector2i const& crop_size,
                                   bool has_georef,
                                   vw::cartography::GeoReference const& georef) {

  // The output no-data value must be < 0 as we scale the images to [0, 1].
  bool has_nodata = true;
  float output_nodata = -32768.0;
  ImageViewRef<float> image_sans_mask = apply_mask(masked_image, isis_lo);
  ImageViewRef<float> processed_image
    = remove_isis_special_pixels(image_sans_mask, isis_lo, isis_hi, out_lo);

  ImageViewRef<PixelMask<float>> local_masked;
  ImageViewRef<uint8> mask;
  if (apply_user_nodata) {
    // Implement --no-data-value
    mask = channel_cast_rescale<uint8>(select_channel(masked_image, 1));
    local_masked = copy_mask(processed_image, create_mask(mask));
  } else {
    local_masked = vw::pixel_cast<vw::PixelMask<float>>(processed_image);
  }

  // Use no-data in interpolation and edge extension
  PixelMask<float> nodata_pix(output_nodata);
  nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> ext(nodata_pix);
  
  // Apply the alignment transform if any
  ImageViewRef<PixelMask<float>> trans_image;
  if (matrix == math::identity_matrix<3>())
    trans_image = crop(edge_extend(local_masked, ext),
                          0, 0, crop_size[0], crop_size[1]);
  else
    trans_image = transform(local_masked, HomographyTransform(matrix),
                              crop_size[0], crop_size[1]);

  // Normalize
  ImageViewRef<PixelMask<float>> normalized_image =
    normalize(trans_image, out_lo, out_hi, 0.0, 1.0);

  vw_out() << "\t--> Writing normalized image: " << out_file << "\n";
  block_write_gdal_image(out_file, apply_mask(normalized_image, output_nodata),
                         has_georef, georef,
                         has_nodata, output_nodata, opt,
                         TerminalProgressCallback("asp", "\t  "+tag+":  "));
}

StereoSessionIsis::StereoSessionIsis() {
  char * isis_ptr = getenv("ISISDATA");
  if (isis_ptr == NULL || std::string(isis_ptr) == "") {
    vw_throw(ArgumentErr() << "The environmental variable ISISDATA must be "
             << "set to point to the location of your supporting ISIS data. "
             << "See the documentation for more information.");
  }

  std::string base_dir = std::string(isis_ptr) + "/base";
  if (!fs::exists(base_dir) || !fs::is_directory(base_dir)) {
    vw_throw(ArgumentErr() << "Missing ISIS data base directory: " <<
             base_dir << "\n");
  }
}

// TODO(oalexan1): See about fully integrating this with StereoSession::preprocessing_hook()
void StereoSessionIsis::preprocessing_hook(bool adjust_left_image_size,
                       std::string const& left_input_file,
                       std::string const& right_input_file,
                       std::string      & left_output_file,
                       std::string      & right_output_file) {

  std::string left_cropped_file, right_cropped_file;
  ImageViewRef<float> left_cropped_image, right_cropped_image;
  vw::GdalWriteOptions options;
  float left_nodata_value, right_nodata_value;
  bool has_left_georef, has_right_georef;
  vw::cartography::GeoReference left_georef, right_georef;
  bool exit_early =
    StereoSession::shared_preprocessing_hook(options,
                                             left_input_file,    right_input_file,
                                             left_output_file,   right_output_file,
                                             left_cropped_file,  right_cropped_file,
                                             left_cropped_image, right_cropped_image,
                                             left_nodata_value,  right_nodata_value,
                                             has_left_georef,    has_right_georef,
                                             left_georef,        right_georef);

  if (exit_early)
    return;

  // Get the image sizes. Later alignment options can choose to change
  // this parameters, such as affine epipolar.
  Vector2i left_size(left_cropped_image.cols(), left_cropped_image.rows());
  Vector2i right_size(right_cropped_image.cols(), right_cropped_image.rows());

  // TODO: A lot of this normalization code should be shared with the base class!
  // Mask the pixels outside of the isis range and <= nodata.
  boost::shared_ptr<DiskImageResourceIsis>
    left_isis_rsrc (new DiskImageResourceIsis(left_input_file)),
    right_isis_rsrc(new DiskImageResourceIsis(right_input_file));
  Vector6f left_stats, right_stats;
  ImageViewRef<PixelMask<float>> left_masked_image
    = find_ideal_isis_range(left_cropped_image, left_isis_rsrc, left_nodata_value,
                            "left", left_stats);
  ImageViewRef<PixelMask<float>> right_masked_image
    = find_ideal_isis_range(right_cropped_image, right_isis_rsrc, right_nodata_value,
                            "right", right_stats);

  // These stats will be needed later on
  if (stereo_settings().alignment_method == "local_epipolar")
    asp::saveStats(this->m_out_prefix, left_stats, right_stats);
  
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

  // Generate aligned versions of the input images according to the
  // options.
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
    Limg = left_masked_image;
    Rimg = right_masked_image;
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
  if (this->do_bathymetry())
    this->align_bathy_masks(options);

} // End function preprocessing_hook

bool StereoSessionIsis::supports_multi_threading () const {
  return false;
}

// Pre file is a pair of grayscale images.  (ImageView<PixelGray<float>>)
// Post file is a disparity map.            (ImageView<PixelMask<Vector2f>>)
void StereoSessionIsis::pre_filtering_hook(std::string const& input_file,
                                           std::string      & output_file) {
  output_file = input_file;
} // End function pre_filtering_hook()

/// Returns the target datum to use for a given camera model. Note the parameter
/// use_sphere_for_non_earth. During alignment, we'd like to use the most
/// accurate non-spherical datum, hence radii[2]. However, for the purpose of
/// creating a DEM on non-Earth planets people usually just use a spherical
/// datum, which we'll do as well.  Maybe at some point this needs to change.
vw::cartography::Datum StereoSessionIsis::get_datum(const vw::camera::CameraModel* cam,
                                                    bool use_sphere_for_non_earth) const {
  const IsisCameraModel * isis_cam
    = dynamic_cast<const IsisCameraModel*>(vw::camera::unadjusted_model(cam));
  VW_ASSERT(isis_cam != NULL, ArgumentErr() << "StereoSessionISIS: Invalid camera.\n");

  return isis_cam->get_datum_isis(use_sphere_for_non_earth);
}

// TODO(oalexan1):  Can we share more code with the DG implementation?

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionIsis::load_camera_model(std::string const& image_file,
                                     std::string const& camera_file,
                                     std::string const& ba_prefix,
                                     Vector2 pixel_offset) const {

  // If the camera file is empty, then we assume the image file has the camera.
  std::string l_cam = camera_file;
  if (l_cam.empty())
    l_cam = image_file;

  return load_adjusted_model(m_camera_loader.load_isis_camera_model(l_cam),
                            image_file, camera_file, ba_prefix, pixel_offset);
}

// Reverse any pre-alignment that was done to the disparity.
ImageViewRef<PixelMask<Vector2f>>
StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file) {
  return DiskImageView<PixelMask<Vector2f>>(input_file);
} // End function pre_pointcloud_hook()

}

#endif  // ASP_HAVE_PKG_ISIS
