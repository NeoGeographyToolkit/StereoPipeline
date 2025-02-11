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

// Stereo Pipeline
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/PhotometricOutlier.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/Equation.h>
#include <asp/Sessions/StereoSessionIsis.h>

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
#include <vw/FileIO/MatrixIO.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Cartography/Datum.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Filter.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/math/special_functions/next.hpp> // boost::float_next
#include <boost/shared_ptr.hpp>

// Isis Headers
#include <SpecialPixel.h>

#include <algorithm>

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
namespace fs = boost::filesystem;

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1

// // Allows FileIO to correctly read/write these pixel types
// namespace vw {
//   template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };

namespace asp {

//  IsisSpecialPixelFunc
//
/// Replace ISIS missing data values with a pixel value of your choice.
template <class PixelT>
class IsisSpecialPixelFunc: public vw::UnaryReturnSameType {
  PixelT m_replacement_low;
  PixelT m_replacement_high;
  PixelT m_replacement_null;
  
  // Private
  IsisSpecialPixelFunc() : m_replacement_low(0), m_replacement_high(0), m_replacement_null(0) {}
  
public:
  IsisSpecialPixelFunc(PixelT const& pix_l, PixelT const& pix_h, PixelT const& pix_n):
    m_replacement_low(pix_l), m_replacement_high(pix_h), m_replacement_null(pix_n) {}
  
  // Helper to determine special across different channel types
  template <typename ChannelT, typename T = void>
  struct Helper {
    static inline bool IsSpecial( ChannelT const& arg ) { return false; }
    static inline bool IsHighPixel( ChannelT const& arg ) { return false; }
    static inline bool IsNull( ChannelT const& arg ) { return false; }
  };
  template<typename T> struct Helper<double, T> {
    static inline bool IsSpecial( double const& arg ) {
      return arg < Isis::VALID_MIN8;
    }
    static inline bool IsHighPixel( double const& arg ) {
      return arg == Isis::HIGH_INSTR_SAT8 || arg == Isis::HIGH_REPR_SAT8;
    }
    static inline bool IsNull( double const& arg ) {
      return arg == Isis::NULL8;
    }
  };
  template<typename T> struct Helper<float, T> {
    static inline bool IsSpecial( float const& arg ) {
      return arg < Isis::VALID_MIN4;
    }
    static inline bool IsHighPixel( float const& arg ) {
      return arg == Isis::HIGH_INSTR_SAT4 || arg == Isis::HIGH_REPR_SAT4;
    }
    static inline bool IsNull( float const& arg ) {
      return arg == Isis::NULL4;
    }
  };
  template<typename T> struct Helper<short, T>{
    static inline bool IsSpecial( short const& arg ) {
      return arg < Isis::VALID_MIN2;
    }
    static inline bool IsHighPixel( short const& arg ) {
      return arg == Isis::HIGH_INSTR_SAT2 || arg == Isis::HIGH_REPR_SAT2;
    }
    static inline bool IsNull( short const& arg ) {
      return arg == Isis::NULL2;
    }
  };
  template<typename T> struct Helper<unsigned short, T> {
    static inline bool IsSpecial( unsigned short const& arg ) {
      return arg < Isis::VALID_MINU2 || arg > Isis::VALID_MAXU2;
    }
    static inline bool IsHighPixel( unsigned short const& arg ) {
      return arg == Isis::HIGH_INSTR_SATU2 || arg == Isis::HIGH_REPR_SATU2;
    }
    static inline bool IsNull( unsigned short const& arg ) {
      return arg == Isis::NULLU2;
    }
  };
  template<typename T> struct Helper<unsigned char, T> {
    static inline bool IsSpecial( unsigned char const& arg ) {
      return arg < Isis::VALID_MIN1 || arg > Isis::VALID_MAX1;
    }
    static inline bool IsHighPixel( unsigned char const& arg ) {
      return arg == Isis::HIGH_INSTR_SAT1 || arg == Isis::HIGH_REPR_SAT1;
    }
    static inline bool IsNull( unsigned char const& arg ) {
      return arg == Isis::NULL1;
    }
  };

  PixelT operator() (PixelT const& pix) const {
    using namespace vw;
    typedef typename CompoundChannelType<PixelT>::type channel_type;
    typedef Helper<channel_type, void> help;
    for (size_t n = 0; n < CompoundNumChannels<PixelT>::value; ++n) {
      if (help::IsSpecial(compound_select_channel<const channel_type&>(pix,n))) {
        if (help::IsHighPixel(compound_select_channel<const channel_type&>(pix,n)))
          return m_replacement_high;
        else if ( help::IsNull(compound_select_channel<const channel_type&>(pix,n)))
          return m_replacement_null;
        else
          return m_replacement_low;
      }
    }
    return pix;
  }
};

template <class ViewT>
vw::UnaryPerPixelView<ViewT, IsisSpecialPixelFunc<typename ViewT::pixel_type> >
remove_isis_special_pixels(vw::ImageViewBase<ViewT> &image,
                           typename ViewT::pixel_type r_low  = typename ViewT::pixel_type(),
                           typename ViewT::pixel_type r_high = typename ViewT::pixel_type(),
                           typename ViewT::pixel_type r_null = typename ViewT::pixel_type()) {
  return vw::per_pixel_filter(image.impl(),
                              IsisSpecialPixelFunc<typename ViewT::pixel_type>
                              (r_low,r_high,r_null));
}
  
// This actually modifies and writes the pre-processed image.
void write_preprocessed_isis_image(vw::GdalWriteOptions const& opt,
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
  
// Process a single ISIS image to find an ideal min max. The reason we
// need to do this, is for ASP to get image intensity values in
// the range of 0-1. To some extent we are compressing the dynamic
// range, but we try to minimize that.
ImageViewRef<PixelMask<float>>
find_ideal_isis_range(ImageViewRef<float> const& image,
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

  ImageViewRef<PixelMask<float>> masked_image = create_mask(image, isis_lo, isis_hi);

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
  ImageViewRef<PixelMask<float>> left_masked_image
    = find_ideal_isis_range(left_cropped_image, left_isis_rsrc, left_nodata_value,
                            "left", will_apply_user_nodata_left,
                            left_lo, left_hi, left_mean, left_std);
  ImageViewRef< PixelMask <float> > right_masked_image
    = find_ideal_isis_range(right_cropped_image, right_isis_rsrc, right_nodata_value,
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
  
  Matrix<double> align_left_matrix  = math::identity_matrix<3>();
  Matrix<double> align_right_matrix = math::identity_matrix<3>();
    
  // Image alignment block. Generate aligned versions of the input
  // images according to the options.
  if (stereo_settings().alignment_method == "epipolar") {
    
    vw_throw(NoImplErr() << "StereoSessionISIS does not support epipolar rectification");
    
  } else if (stereo_settings().alignment_method == "homography"     ||
             stereo_settings().alignment_method == "affineepipolar" ||
             stereo_settings().alignment_method == "local_epipolar") {

    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    this->camera_models(left_cam, right_cam);
    determine_image_alignment(// Inputs
                              m_out_prefix, left_cropped_file, right_cropped_file,  
                              left_input_file,
                              left_stats, right_stats, left_nodata_value, right_nodata_value,  
                              left_cam, right_cam,
                              adjust_left_image_size,  
                              // In-out
                              align_left_matrix, align_right_matrix, left_size, right_size);
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

bool StereoSessionIsis::supports_multi_threading () const {
  return false;
}
  
// Pre file is a pair of grayscale images.  (ImageView<PixelGray<float> >)
// Post file is a disparity map.            (ImageView<PixelMask<Vector2f> >)
void StereoSessionIsis::pre_filtering_hook(std::string const& input_file,
                                           std::string      & output_file) {
  output_file = input_file;
} // End function pre_filtering_hook()

/// Returns the target datum to use for a given camera model.
/// Note the parameter use_sphere_for_non_earth.
/// During alignment, we'd like to use the most accurate
/// non-spherical datum, hence radii[2]. However, for the purpose
/// of creating a DEM on non-Earth planets people usually just use
/// a spherical datum, which we'll do as well.  Maybe at some
/// point this needs to change.
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

#endif  // ASP_HAVE_PKG_ISISIO
