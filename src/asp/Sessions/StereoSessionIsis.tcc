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
                      float & isis_lo, float & isis_hi ) {

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
  // Calculating statistics. We subsample the images so statistics only does about a million samples.
  float isis_mean, isis_std;
  {
    vw_out(InfoMessage) << "\t--> Computing statistics for the "+tag+" image\n";
    int stat_scale = int(ceil(sqrt(float(image.cols())*float(image.rows()) / 1000000)));
    ChannelAccumulator<math::CDFAccumulator<float> > accumulator;
    for_each_pixel(subsample(edge_extend(masked_image, ConstantEdgeExtension()),
                             stat_scale ),
                   accumulator );
    isis_lo   = accumulator.quantile(0);
    isis_hi   = accumulator.quantile(1);
    isis_mean = accumulator.approximate_mean();
    isis_std  = accumulator.approximate_stddev();

    vw_out(InfoMessage) << "\t  "+tag+": [ lo:" << isis_lo << " hi:" << isis_hi
                        << " m: " << isis_mean << " s: " << isis_std <<  "]\n";
  }

  // Normalizing to -+2 sigmas around mean
  if ( stereo_settings().force_use_entire_range == 0 ) {
    vw_out(InfoMessage) << "\t--> Adjusting hi and lo to -+2 sigmas around mean.\n";

    if ( isis_lo < isis_mean - 2*isis_std )
      isis_lo = isis_mean - 2*isis_std;
    if ( isis_hi > isis_mean + 2*isis_std )
      isis_hi = isis_mean + 2*isis_std;

    vw_out(InfoMessage) << "\t    "+tag+" changed: [ lo:"
                        << isis_lo << " hi:" << isis_hi << "]\n";
  }

  return masked_image;
}

// This actually modifies and writes the pre-processed image.
inline
void write_preprocessed_isis_image( BaseOptions const& opt,
                                    bool will_apply_user_nodata,
                                    ImageViewRef< PixelMask <float> > masked_image,
                                    std::string const& out_file,
                                    std::string const& tag,
                                    float isis_lo, float isis_hi,
                                    float out_lo,  float out_hi,
                                    Matrix<double> const& matrix,
                                    Vector2i const& crop_size ) {

  // The output no-data value must be < 0 as we scale the images to [0, 1].
  float output_nodata = -32768.0;

  ImageViewRef<float> image_sans_mask = apply_mask(masked_image, isis_lo);

  ImageViewRef<float> processed_image
    = remove_isis_special_pixels(image_sans_mask, isis_lo, isis_hi, out_lo);

  if (will_apply_user_nodata){

    // If the user specifies a no-data value, mask all pixels <= that
    // value. Note: this causes non-trivial erosion at image
    // boundaries where invalid pixels show up if homography is used.

    ImageViewRef<uint8> mask = channel_cast_rescale<uint8>(select_channel(masked_image, 1));

    ImageViewRef< PixelMask<float> > normalized_image =
      normalize(copy_mask(processed_image, create_mask(mask)), out_lo, out_hi, 0.0, 1.0);

    ImageViewRef< PixelMask<float> > applied_image;
    if ( matrix == math::identity_matrix<3>() ) {
      applied_image = crop(edge_extend(normalized_image, ZeroEdgeExtension()),
                           0, 0, crop_size[0], crop_size[1]);
    } else {
      applied_image = transform(normalized_image, HomographyTransform(matrix),
                                crop_size[0], crop_size[1]);
    }

    vw_out() << "\t--> Writing normalized image: " << out_file << "\n";
    block_write_gdal_image( out_file, apply_mask(applied_image, output_nodata), output_nodata, opt,
                            TerminalProgressCallback("asp", "\t  "+tag+":  "));

  }else{

    // Set invalid pixels to the minimum pixel value. Causes less
    // erosion and the results are good.

    ImageViewRef<float> normalized_image =
      normalize(processed_image, out_lo, out_hi, 0.0, 1.0);

    ImageViewRef<float> applied_image;
    if ( matrix == math::identity_matrix<3>() ) {
      applied_image = crop(edge_extend(normalized_image, ZeroEdgeExtension()),
                           0, 0, crop_size[0], crop_size[1]);
    } else {
      applied_image = transform(normalized_image, HomographyTransform(matrix),
                                crop_size[0], crop_size[1]);
    }

    vw_out() << "\t--> Writing normalized image: " << out_file << "\n";
    block_write_gdal_image( out_file, applied_image, output_nodata, opt,
                            TerminalProgressCallback("asp", "\t  "+tag+":  "));
  }

}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE>
vw::cartography::Datum StereoSessionIsisBase<DISKTRANSFORM_TYPE>::get_datum(const vw::camera::CameraModel* cam) const
{
  const IsisCameraModel * isis_cam
    = dynamic_cast<const IsisCameraModel*>(vw::camera::unadjusted_model(cam));
  VW_ASSERT(isis_cam != NULL, ArgumentErr() << "StereoSessionISIS: Invalid left camera.\n");
  Vector3 radii = isis_cam->target_radii();
  cartography::Datum datum("","","", (radii[0] + radii[1]) / 2, radii[2], 0);
  return datum;
}

// TODO: Can we share more code with the DG implementation?

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE>
void StereoSessionIsisBase<DISKTRANSFORM_TYPE>::
pre_preprocessing_hook(bool adjust_left_image_size,
                       std::string const& left_input_file,
                       std::string const& right_input_file,
                       std::string      & left_output_file,
                       std::string      & right_output_file) {

  // Set output file paths
  left_output_file  = this->m_out_prefix + "-L.tif";
  right_output_file = this->m_out_prefix + "-R.tif";

  bool crop_left_and_right =
    ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)) &&
    ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) );

  // If the output files already exist, and we don't crop both left
  // and right images, then there is nothing to do here.
  if ( boost::filesystem::exists(left_output_file)  &&
       boost::filesystem::exists(right_output_file) &&
       (!crop_left_and_right)) {
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelGray<float32> > out_left (left_output_file );
      DiskImageView<PixelGray<float32> > out_right(right_output_file);
      vw_out(InfoMessage) << "\t--> Using cached normalized input images.\n";
      vw_settings().reload_config();
      return;
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  } // End check for existing output files

  // Get handles to the input images which are explicitly ISIS typed
  boost::shared_ptr<DiskImageResourceIsis>
    left_isis_rsrc (new DiskImageResourceIsis(left_input_file )),
    right_isis_rsrc(new DiskImageResourceIsis(right_input_file));

  // Retrieve nodata values
  float left_nodata_value, right_nodata_value;
  {
    // For this to work the ISIS type must be registered with the
    // DiskImageResource class.  - This happens in "stereo.cc", so
    // these calls will create DiskImageResourceIsis objects.
    boost::shared_ptr<DiskImageResource>
      left_rsrc (DiskImageResource::open(left_input_file )),
      right_rsrc(DiskImageResource::open(right_input_file));
    this->get_nodata_values(left_rsrc, right_rsrc,
                            left_nodata_value, right_nodata_value);
  }

  // Enforce no predictor in compression, it works badly with L.tif and R.tif.
  asp::BaseOptions options = this->m_options;
  options.gdal_options["PREDICTOR"] = "1";

  std::string left_cropped_file = left_input_file,
    right_cropped_file = right_input_file;

  // See if to crop the images
  if (crop_left_and_right) {
    // Crop the images, will use them from now on
    left_cropped_file  = this->m_out_prefix + "-L-cropped.tif";
    right_cropped_file = this->m_out_prefix + "-R-cropped.tif";

    DiskImageView<float> left_orig_image(left_input_file);
    stereo_settings().left_image_crop_win.crop(bounding_box(left_orig_image));
    vw_out() << "\t--> Writing cropped image: " << left_cropped_file << "\n";
    block_write_gdal_image(left_cropped_file,
                           crop(left_orig_image,
                                stereo_settings().left_image_crop_win),
                           left_nodata_value, options,
                           TerminalProgressCallback("asp", "\t:  "));

    DiskImageView<float> right_orig_image(right_input_file);
    stereo_settings().right_image_crop_win.crop(bounding_box(right_orig_image));
    vw_out() << "\t--> Writing cropped image: " << right_cropped_file << "\n";
    block_write_gdal_image(right_cropped_file,
                           crop(right_orig_image,
                                stereo_settings().right_image_crop_win),
                           right_nodata_value, options,
                           TerminalProgressCallback("asp", "\t:  "));
  }

  // Load the cropped images
  DiskImageView<float> left_disk_image(left_cropped_file),
    right_disk_image(right_cropped_file);

  // Getting image sizes. Later alignment options can choose to change this parameters. (Affine Epipolar).
  Vector2i left_size  = file_image_size(left_cropped_file ),
           right_size = file_image_size(right_cropped_file);

  // These variables will be true if we reduce the valid range for ISIS images
  // using the nodata value provided by the user.
  bool will_apply_user_nodata_left  = false,
       will_apply_user_nodata_right = false;

  // TODO: A lot of this normalization code should be shared with the base class!
  // Mask the pixels outside of the isis range and <= nodata.
  float left_lo, left_hi, right_lo, right_hi;
  ImageViewRef< PixelMask <float> > left_masked_image
    = find_ideal_isis_range(left_disk_image, left_isis_rsrc, left_nodata_value,
                            "left", will_apply_user_nodata_left, left_lo, left_hi);
  ImageViewRef< PixelMask <float> > right_masked_image
    = find_ideal_isis_range(right_disk_image, right_isis_rsrc, right_nodata_value,
                            "right", will_apply_user_nodata_right, right_lo,
                            right_hi);

  // Handle mutual normalization if requested
  float left_lo_out = left_lo,  left_hi_out = left_hi,
        right_lo_out = right_lo, right_hi_out = right_hi;
  if (stereo_settings().individually_normalize == 0 ) {
    // Find the outer range of both files
    float mutual_lo = std::min(left_lo, right_lo);
    float mutual_hi = std::max(left_hi, right_hi);
    vw_out() << "\t--> Normalizing globally to: ["
             <<mutual_lo<<" "<<mutual_hi<<"]\n";
    // Set the individual hi/lo values to the mutual values
    left_lo_out  = mutual_lo;
    left_hi_out  = mutual_hi;
    right_lo_out = mutual_lo;
    right_hi_out = mutual_hi;
  } else{
    vw_out() << "\t--> Individually normalizing.\n";
  }

  // Image alignment block - Generate aligned versions of the input
  // images according to the options.
  Matrix<double> align_left_matrix  = math::identity_matrix<3>(),
                 align_right_matrix = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {

    // Define the file name containing IP match information.
    std::string match_filename = ip::match_filename(this->m_out_prefix,
                                                    left_cropped_file,
                                                    right_cropped_file);

    // Find matching interest points between the two input images
    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    this->camera_models(left_cam, right_cam);
    this->ip_matching(left_cropped_file,   right_cropped_file,
                      stereo_settings().ip_per_tile,
                      left_nodata_value, right_nodata_value, match_filename,
                      left_cam.get(),    right_cam.get());
    // Read in the interest point data we just wrote to disk
    std::vector<ip::InterestPoint> left_ip, right_ip;
    ip::read_binary_match_file(match_filename, left_ip, right_ip);

    // Compute the appropriate transform matrix between the two input images.
    if ( stereo_settings().alignment_method == "homography" ) {
      left_size = homography_rectification(adjust_left_image_size,
                                           left_size, right_size, left_ip, right_ip,
                                           align_left_matrix, align_right_matrix );
      vw_out() << "\t--> Aligning right image to left using matrices:\n"
               << "\t      " << align_left_matrix  << "\n"
               << "\t      " << align_right_matrix << "\n";
    } else {
      left_size = affine_epipolar_rectification(left_size, right_size,
                                                left_ip,   right_ip,
                                                align_left_matrix,
                                                align_right_matrix);
      vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
               << "\t      " << submatrix(align_left_matrix ,0,0,2,3) << "\n"
               << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
    }
    // Write the computed transform matrices to disk
    write_matrix( this->m_out_prefix + "-align-L.exr", align_left_matrix );
    write_matrix( this->m_out_prefix + "-align-R.exr", align_right_matrix);
    right_size = left_size; // Because the images are now aligned
                            // .. they are the same size.
  } else if ( stereo_settings().alignment_method == "epipolar" ) {
    vw_throw( NoImplErr() << "StereoSessionISIS does not support epipolar rectification" );
  } // End alignment block


  // Apply alignment and normalization
  bool will_apply_user_nodata = ( will_apply_user_nodata_left || will_apply_user_nodata_right);

  // Write output images
  write_preprocessed_isis_image( options, will_apply_user_nodata,
                                 left_masked_image, left_output_file, "left",
                                 left_lo, left_hi, left_lo_out, left_hi_out,
                                 align_left_matrix, left_size );
  write_preprocessed_isis_image( options, will_apply_user_nodata,
                                 right_masked_image, right_output_file, "right",
                                 right_lo, right_hi, right_lo_out, right_hi_out,
                                 align_right_matrix, right_size );
}

// Only used with mask_flatfield option?
inline std::string write_shadow_mask( BaseOptions const& opt,
                                      std::string const& output_prefix,
                                      std::string const& input_image,
                                      std::string const& mask_postfix ) {
  // This thresholds at -25000 as the input sub4s for Apollo that I've
  // processed have a range somewhere between -32000 and +32000. -ZMM
  DiskImageView<PixelGray<float> > disk_image( input_image );
  DiskImageView<uint8> disk_mask( output_prefix + mask_postfix );
  ImageViewRef<uint8> mask = apply_mask(intersect_mask(create_mask(disk_mask),
                                        create_mask(threshold(disk_image,-25000,0,1.0))));
  std::string output_mask = output_prefix+mask_postfix.substr(0,mask_postfix.size()-4)+"Debug.tif";

  block_write_gdal_image(output_mask, mask, opt, TerminalProgressCallback("asp","\t  Shadow:"));
  return output_mask;
}

// Stage 2: Correlation
//
// Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
// Post file is a disparity map.            ( ImageView<PixelMask<Vector2f> > )
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE>
void StereoSessionIsisBase<DISKTRANSFORM_TYPE>
::pre_filtering_hook(std::string const& input_file,
                     std::string      & output_file) {
  output_file = input_file;

  if (stereo_settings().mask_flatfield) {
    // ****************************************************
    // The following code is for Apollo Metric Camera ONLY!
    // (use at your own risk)
    // ****************************************************
    vw_out() << "\t--> Masking pixels that are less than 0.0.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    output_file = this->m_out_prefix + "-R-masked.exr";

    std::string shadowLmask_name = write_shadow_mask(this->m_options, this->m_out_prefix, this->m_left_image_file,  "-lMask.tif");
    std::string shadowRmask_name = write_shadow_mask(this->m_options, this->m_out_prefix, this->m_right_image_file, "-rMask.tif");

    DiskImageView<uint8> shadowLmask(shadowLmask_name);
    DiskImageView<uint8> shadowRmask(shadowRmask_name);

    DiskImageView<PixelMask<Vector2f> > disparity_disk_image(input_file);
    ImageViewRef <PixelMask<Vector2f> > disparity_map = stereo::disparity_mask(disparity_disk_image, shadowLmask, shadowRmask);

    DiskImageResourceOpenEXR disparity_map_rsrc(output_file, disparity_map.format() );
    Vector2i block_size(std::min<size_t>(vw_settings().default_tile_size(), disparity_map.cols()),
                        std::min<size_t>(vw_settings().default_tile_size(), disparity_map.rows()));
    disparity_map_rsrc.set_block_write_size(block_size);
    block_write_image(disparity_map_rsrc, disparity_map, TerminalProgressCallback( "asp", "\t--> Saving Mask :"));
  }
} // End function pre_filtering_hook()

// Reverse any pre-alignment that was done to the disparity.
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE>
ImageViewRef<PixelMask<Vector2f> > StereoSessionIsisBase<DISKTRANSFORM_TYPE>
::pre_pointcloud_hook(std::string const& input_file) {

  std::string dust_result = input_file;
  if ( stereo_settings().mask_flatfield ) {
    // ****************************************************
    // The following code is for Apollo Metric Camera ONLY!
    // (use at your own risk)
    // ****************************************************
    vw_out() << "\t--> Masking pixels that appear to be dust.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    photometric_outlier_rejection(this->m_options, this->m_out_prefix, input_file,
                                  dust_result, stereo_settings().corr_kernel[0]);
  }
  return DiskImageView<PixelMask<Vector2f> >(dust_result);
} // End function pre_pointcloud_hook(





} // end namespace asp



#endif  // ASP_HAVE_PKG_ISISIO
