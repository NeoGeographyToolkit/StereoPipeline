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
                      std::string const& image_path,
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
    vw_out(InfoMessage) << "Computing statistics for " + image_path << "\n";
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

  }

  // Normalizing to -+2 sigmas around mean
  if (stereo_settings().force_use_entire_range == 0) {
    vw_out(InfoMessage) << "\t--> Adjusting hi and lo to -+2 sigmas around mean.\n";

    // Do not exceed isis_lo and isis_hi as there we may have special pixels
    if (isis_lo < isis_mean - 2*isis_std)
      isis_lo = isis_mean - 2*isis_std;
    if (isis_hi > isis_mean + 2*isis_std)
      isis_hi = isis_mean + 2*isis_std;
  }

  vw_out(InfoMessage) << "\t: [ lo:" << isis_lo << " hi:" << isis_hi
                      << " m: " << isis_mean << " s: " << isis_std <<  "]\n";

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

// Find the masked images and stats. This is reimplemented for ISIS to take 
// into account special pixels.
void StereoSessionIsis::
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

  // TODO: A lot of this normalization code should be shared with the base class!
  // Mask the pixels outside of the isis range and <= nodata.
  boost::shared_ptr<DiskImageResourceIsis>
    left_isis_rsrc (new DiskImageResourceIsis(left_input_file)),
    right_isis_rsrc(new DiskImageResourceIsis(right_input_file));
  left_masked_image
    = find_ideal_isis_range(left_cropped_image, left_isis_rsrc, left_nodata_value,
                            left_cropped_file, left_stats);
  right_masked_image
    = find_ideal_isis_range(right_cropped_image, right_isis_rsrc, right_nodata_value,
                            right_cropped_file, right_stats);
}

bool StereoSessionIsis::supports_multi_threading () const {
  return false;
}

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

}

#endif  // ASP_HAVE_PKG_ISIS
