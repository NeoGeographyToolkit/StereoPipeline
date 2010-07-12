// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSessionIsis.cc
///

// Vision Workbench
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/RANSAC.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Cartography.h>
// Stereo Pipeline
#include <asp/Sessions/ISIS/StereoSessionIsis.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Core/StereoSettings.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/Sessions/ISIS/PhotometricOutlier.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/shared_ptr.hpp>
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace asp;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

// Do not attempt interest point alignment; assume ISIS images are
// already map projected.  Simply remove the special pixels and
// normalize between 0 and 1 (so that the image masks are found
// properly)
void
StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1,
                                          std::string const& input_file2,
                                          std::string & output_file1,
                                          std::string & output_file2) {
  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  try {
    DiskImageView<PixelGray<float32> > out1(output_file1);
    DiskImageView<PixelGray<float32> > out2(output_file2);
    vw_out(InfoMessage) << "Skipping normalization step, using cached images: " <<
      output_file1 << " and " << output_file2 << "\n";
    return;
  } catch (vw::Exception const& e) {}

  DiskImageView<PixelGray<float> > left_disk_image(input_file1);
  DiskImageView<PixelGray<float> > right_disk_image(input_file2);
  DiskImageResourceIsis left_rsrc(input_file1);
  DiskImageResourceIsis right_rsrc(input_file2);

  float left_lo, left_hi, right_lo, right_hi;
  float left_mean, left_std, right_mean, right_std;

  // Calculating statistics. We subsample the images so statistics
  // only does about a million samples.
  {
    vw_out(InfoMessage) << "\t--> Computing statistics for the left image\n";
    int left_stat_scale = int(ceil(sqrt(float(left_disk_image.cols())*float(left_disk_image.rows()) / 1000000)));
    ImageViewRef<PixelMask<PixelGray<float> > > left_valid =
      subsample(create_mask( left_disk_image, left_rsrc.valid_minimum(), left_rsrc.valid_maximum() ),
                left_stat_scale );
    ChannelAccumulator<math::CDFAccumulator<float> > accumulator;
    for_each_pixel( left_valid, accumulator );
    left_lo = accumulator.quantile(0);
    left_hi = accumulator.quantile(1);
    left_mean = accumulator.approximate_mean();
    left_std  = accumulator.approximate_stddev();

    vw_out(InfoMessage) << "\t    Left: [ lo:" << left_lo << " hi:" << left_hi
                        << " m: " << left_mean << " s: " << left_std <<  "]\n";
  }
  {
    vw_out(InfoMessage) << "\t--> Computing statistics values for the right image\n";
    int right_stat_scale = int(ceil(sqrt(float(right_disk_image.cols())*float(right_disk_image.rows()) / 1000000)));
    ImageViewRef<PixelMask<PixelGray<float> > > right_valid =
      subsample(create_mask( right_disk_image, right_rsrc.valid_minimum(),
                             right_rsrc.valid_maximum() ),
                right_stat_scale );
    ChannelAccumulator<math::CDFAccumulator<float> > accumulator;
    for_each_pixel( right_valid, accumulator );
    right_lo = accumulator.quantile(0);
    right_hi = accumulator.quantile(1);
    right_mean = accumulator.approximate_mean();
    right_std  = accumulator.approximate_stddev();

    vw_out(InfoMessage) << "\t    Right: [ lo:" << right_lo << " hi:"
                        << right_hi << " m: " << right_mean << " s: "
                        << right_std << "]\n";
  }

  // Normalizing to -+2 sigmas around mean
  if ( stereo_settings().force_max_min == 0 ) {
    vw_out(InfoMessage) << "\t--> Adjusting hi and lo to -+2 sigmas around mean.\n";

    if ( left_lo < left_mean - 2*left_std )
      left_lo = left_mean - 2*left_std;
    if ( right_lo < right_mean - 2*right_std )
      right_lo = right_mean - 2*right_std;
    if ( left_hi > left_mean + 2*left_std )
      left_hi = left_mean + 2*left_std;
    if ( right_hi > right_mean + 2*right_std )
      right_hi = right_mean + 2*right_std;

    vw_out(InfoMessage) << "\t    Left changed: [ lo:"
                        << left_lo << " hi:" << left_hi << "]\n";
    vw_out(InfoMessage) << "\t    Right changed: [ lo:"
                        << right_lo << " hi:" << right_hi << "]\n";
  }

  // Working out alignment
  float lo = std::min (left_lo, right_lo);  // Finding global
  float hi = std::max (left_hi, right_hi);
  Matrix<double> align_matrix(3,3);
  align_matrix.set_identity();
  if ( stereo_settings().keypoint_alignment) {
    if ( left_rsrc.is_map_projected() ||
         right_rsrc.is_map_projected() ) {
      vw_out(WarningMessage,"console") << "One or more of the input files is Map Projected.\n"
                                       << "\tInterest Point alignment is not recommend in this case.\n";
    }
    DiskImageView<PixelGray<float> > left_disk_image(input_file1);
    DiskImageView<PixelGray<float> > right_disk_image(input_file2);
    ImageViewRef<PixelGray<float> > left_view =
      normalize(remove_isis_special_pixels(left_disk_image, lo),
                lo, hi, 0, 1.0);
    ImageViewRef<PixelGray<float> > right_view =
      normalize(remove_isis_special_pixels(right_disk_image, lo),
                lo, hi, 0, 1.0);
    align_matrix = determine_image_align(input_file1, input_file2,
                                         left_view, right_view );
  }
  write_matrix( m_out_prefix + "-align.exr", align_matrix );

  // Apply alignment and normalization
  ImageViewRef<PixelGray<float> > Limg;
  ImageViewRef<PixelGray<float> > Rimg;
  if (stereo_settings().individually_normalize == 0 ) {
    vw_out() << "\t--> Normalizing globally to: ["<<lo<<" "<<hi<<"]\n";
    Limg = clamp(normalize(remove_isis_special_pixels(left_disk_image,
                                                      left_lo, left_hi, lo),
                           lo, hi, 0.0, 1.0));
    Rimg = clamp(transform(normalize(remove_isis_special_pixels(right_disk_image, right_lo, right_hi, lo),
                                     lo, hi, 0.0, 1.0),
                           HomographyTransform(align_matrix),
                           left_disk_image.cols(), left_disk_image.rows()));
  } else {
    vw_out() << "\t--> Individually normalizing.\n";
    Limg = clamp(normalize(remove_isis_special_pixels(left_disk_image, left_lo,
                                                      left_hi, left_lo),
                           left_lo, left_hi, 0.0, 1.0));
    Rimg = clamp(transform(normalize(remove_isis_special_pixels(right_disk_image, right_lo,
                                                                right_hi, right_lo),
                                     right_lo, right_hi, 0.0, 1.0),
                           HomographyTransform(align_matrix),
                           left_disk_image.cols(), left_disk_image.rows()));
  }
  // Write the results to disk.
  vw_out() << "\t--> Writing normalized images.\n";
  DiskImageResourceGDAL left_out_rsrc( output_file1, Limg.format(),
                                       Vector2i(vw_settings().default_tile_size(),
                                                vw_settings().default_tile_size()) );
  block_write_image( left_out_rsrc, Limg,
                     TerminalProgressCallback("asp", "\t    Left:  "));

  DiskImageResourceGDAL right_out_rsrc( output_file2, Rimg.format(),
                                        Vector2i(vw_settings().default_tile_size(),
                                                 vw_settings().default_tile_size()) );
  block_write_image( right_out_rsrc, Rimg,
                     TerminalProgressCallback("asp", "\t    Right: "));
}

inline std::string write_shadow_mask( std::string const& output_prefix,
                                      std::string const& input_image,
                                      std::string const& mask_postfix ) {
  // This thresholds at -25000 as the input sub4s for Apollo that I've
  // processed have a range somewhere between -32000 and +32000. -ZMM
  DiskImageView<PixelGray<float> > disk_image( input_image );
  DiskImageView<uint8> disk_mask( output_prefix + mask_postfix );
  ImageViewRef<uint8> mask =
    apply_mask(intersect_mask(create_mask(disk_mask),
                              create_mask(threshold(disk_image,-25000,0,1.0))));
  std::string output_mask =
    output_prefix+mask_postfix.substr(0,mask_postfix.size()-4)+"Debug.tif";

  DiskImageResourceGDAL out_mask_rsrc( output_mask, mask.format(),
                                       Vector2i(vw_settings().default_tile_size(),
                                                vw_settings().default_tile_size()) );
  block_write_image( out_mask_rsrc, mask );
  return output_mask;
}

// Stage 2: Correlation
//
// Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
// Post file is a disparity map.            ( ImageView<PixelMask<Vector2f> > )
void
StereoSessionIsis::pre_filtering_hook(std::string const& input_file,
                                      std::string & output_file) {
  output_file = input_file;

  // ****************************************************
  // The following code is for Apollo Metric Camera ONLY!
  // (use at your own risk)
  // ****************************************************
  if (stereo_settings().mask_flatfield) {
    vw_out() << "\t--> Masking pixels that are less than 0.0.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    output_file = m_out_prefix + "-R-masked.exr";

    std::string shadowLmask_name =
      write_shadow_mask( m_out_prefix, m_left_image_file,
                         "-lMask.tif" );
    std::string shadowRmask_name =
      write_shadow_mask( m_out_prefix, m_right_image_file,
                         "-rMask.tif" );

    DiskImageView<uint8> shadowLmask( shadowLmask_name );
    DiskImageView<uint8> shadowRmask( shadowRmask_name );

    DiskImageView<PixelMask<Vector2f> > disparity_disk_image(input_file);
    ImageViewRef<PixelMask<Vector2f> > disparity_map =
      stereo::disparity_mask(disparity_disk_image,
                             shadowLmask, shadowRmask );

    DiskImageResourceOpenEXR disparity_map_rsrc(output_file, disparity_map.format() );
    disparity_map_rsrc.set_tiled_write(std::min(vw_settings().default_tile_size(),disparity_map.cols()),
                                       std::min(vw_settings().default_tile_size(),disparity_map.rows()));
    block_write_image( disparity_map_rsrc, disparity_map,
                       TerminalProgressCallback( "asp", "\t--> Saving Mask :") );
  }
}

// Reverse any pre-alignment that was done to the images.
void
StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file,
                                       std::string & output_file) {

  // ****************************************************
  // The following code is for Apollo Metric Camera ONLY!
  // (use at your own risk)
  // ****************************************************
  std::string dust_result = input_file;
  if ( stereo_settings().mask_flatfield ) {
    vw_out() << "\t--> Masking pixels that appear to be dust.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    photometric_outlier_rejection( m_out_prefix, input_file,
                                   dust_result, stereo_settings().h_kern );
  }

  DiskImageView<PixelMask<Vector2f> > disparity_map(dust_result);
  output_file = m_out_prefix + "-F-corrected.exr";

  // We used a homography to line up the images, we may want
  // to generate pre-alignment disparities before passing this information
  // onto the camera model in the next stage of the stereo pipeline.
  Matrix<double> align_matrix;
  try {
    read_matrix(align_matrix, m_out_prefix + "-align.exr");
    vw_out(DebugMessage) << "Alignment Matrix: " << align_matrix << "\n";
  } catch (vw::IOErr &e) {
    vw_out() << "\nCould not read in aligment matrix: " << m_out_prefix
             << "-align.exr.  Exiting. \n\n";
    exit(1);
  }

  // Remove pixels that are outside the bounds of the secondary image.
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
  ImageViewRef<PixelMask<Vector2f> > result =
    stereo::disparity_range_mask(stereo::transform_disparities(disparity_map,
                                          HomographyTransform(align_matrix)),
                                 Vector2f(0,0),
                                 Vector2f( right_disk_image.cols(),
                                           right_disk_image.rows() ) );

  DiskImageResourceOpenEXR disparity_corrected_rsrc(output_file, result.format() );
  disparity_corrected_rsrc.set_tiled_write(std::min(vw_settings().default_tile_size(),disparity_map.cols()),
                                           std::min(vw_settings().default_tile_size(),disparity_map.rows()));
  block_write_image( disparity_corrected_rsrc, result,
                     TerminalProgressCallback("asp", "\t    Processing:"));
}

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionIsis::camera_model(std::string image_file,
                                std::string camera_file) {

  if (boost::ends_with(boost::to_lower_copy(camera_file), ".isis_adjust")){
    vw_out() << "\t--> Using adjusted Isis Camera Model: " << camera_file << "\n";

    // Creating Equations for the files
    std::ifstream input( camera_file.c_str() );
    boost::shared_ptr<asp::BaseEquation> posF = read_equation( input );
    boost::shared_ptr<asp::BaseEquation> poseF = read_equation( input );
    input.close();

    // Finally creating camera model
    return boost::shared_ptr<camera::CameraModel>(new IsisAdjustCameraModel( image_file, posF, poseF ));

  } else {
    vw_out() << "\t--> Using standard Isis camera model: " << image_file << "\n";
    return boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(image_file));
  }

}


