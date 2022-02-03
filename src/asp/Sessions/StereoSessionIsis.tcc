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


/// \file StereoSessionIsis.tcc
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


// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/math/special_functions/next.hpp> // boost::float_next
#include <boost/shared_ptr.hpp>

#include <algorithm>

using namespace vw;
using namespace vw::camera;

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1

namespace asp{

// Process a single ISIS image to find an ideal min max. The reason we
// need to do this, is for ASP to get image intensity values in
// the range of 0-1. To some extent we are compressing the dynamic
// range, but we try to minimize that.
inline ImageViewRef< PixelMask<float> >
find_ideal_isis_range(DiskImageView<float> const& image,
                      boost::shared_ptr<DiskImageResourceIsis> isis_rsrc,
                      float nodata_value,
                      std::string const& tag,
                      bool & will_apply_user_nodata,
                      float & isis_lo, float & isis_hi,
                      float & isis_mean, float& isis_std) {

  will_apply_user_nodata = false;
  isis_lo = isis_rsrc->valid_minimum();
  isis_hi = isis_rsrc->valid_maximum();

  // Force the low value to be greater than the nodata value
  if (!boost::math::isnan(nodata_value) && nodata_value >= isis_lo){
    // The new lower bound is the next floating point number > nodata_value.
    will_apply_user_nodata = true;
    isis_lo = boost::math::float_next(nodata_value);
    if (isis_hi < isis_lo)
      isis_hi = isis_lo;
  }

  ImageViewRef< PixelMask<float> > masked_image = create_mask(image, isis_lo, isis_hi);

  // TODO: Is this same process a function in DG?
  // Calculating statistics. We subsample the images so statistics
  // only does about a million samples.
  {
    vw_out(InfoMessage) << "\t--> Computing statistics for " + tag + "\n";
    int stat_scale = int(ceil(sqrt(float(image.cols())*float(image.rows()) / 1000000)));
    ChannelAccumulator<math::CDFAccumulator<float> > accumulator;
    for_each_pixel(subsample(edge_extend(masked_image, ConstantEdgeExtension()),
                             stat_scale),
                   accumulator);
    isis_lo   = accumulator.quantile(0);
    isis_hi   = accumulator.quantile(1);
    isis_mean = accumulator.approximate_mean();
    isis_std  = accumulator.approximate_stddev();

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

  return masked_image;
}

// This actually modifies and writes the pre-processed image.
inline
void write_preprocessed_isis_image(vw::cartography::GdalWriteOptions const& opt,
                                    bool will_apply_user_nodata,
                                    ImageViewRef< PixelMask <float> > masked_image,
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

  // Use no-data in interpolation and edge extension.
  PixelMask<float> nodata_pix(0);
  nodata_pix.invalidate();
  ValueEdgeExtension< PixelMask<float> > ext(nodata_pix); 

  if (will_apply_user_nodata){

    // If the user specifies a no-data value, mask all pixels <= that
    // value. Note: this causes non-trivial erosion at image
    // boundaries where invalid pixels show up if homography is used.

    ImageViewRef<uint8> mask = channel_cast_rescale<uint8>(select_channel(masked_image, 1));

    ImageViewRef< PixelMask<float> > normalized_image =
      normalize(copy_mask(processed_image, create_mask(mask)), out_lo, out_hi, 0.0, 1.0);

    ImageViewRef< PixelMask<float> > applied_image;
    if (matrix == math::identity_matrix<3>()) {
      applied_image = crop(edge_extend(normalized_image, ext),
                           0, 0, crop_size[0], crop_size[1]);
    } else {
      applied_image = transform(normalized_image, HomographyTransform(matrix),
                                crop_size[0], crop_size[1]);
    }

    vw_out() << "\t--> Writing normalized image: " << out_file << "\n";
    block_write_gdal_image(out_file, apply_mask(applied_image, output_nodata),
                            has_georef, georef,
                            has_nodata, output_nodata, opt,
                            TerminalProgressCallback("asp", "\t  "+tag+":  "));
  }else{

    // Set invalid pixels to the minimum pixel value. Causes less
    // erosion and the results are good.

    ImageViewRef<float> normalized_image =
      normalize(processed_image, out_lo, out_hi, 0.0, 1.0);

    ImageViewRef<float> applied_image;
    if (matrix == math::identity_matrix<3>()) {
      applied_image = crop(edge_extend(normalized_image, ext),
                           0, 0, crop_size[0], crop_size[1]);
    } else {
      applied_image = transform(normalized_image, HomographyTransform(matrix),
                                crop_size[0], crop_size[1]);
    }

    vw_out() << "\t--> Writing normalized image: " << out_file << "\n";
    block_write_gdal_image(out_file, applied_image,
                            has_georef, georef,
                            has_nodata, output_nodata, opt,
                            TerminalProgressCallback("asp", "\t  "+tag+":  "));
  }

}


inline bool StereoSessionIsis::supports_multi_threading () const {
  return false;
}
  
} // end namespace asp


#endif  // ASP_HAVE_PKG_ISISIO
