// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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
#include <vw/FileIO.h>
#include <vw/Math/Functors.h>
#include <vw/Math/Geometry.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Cartography.h>

// Stereo Pipeline
#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Sessions/ISIS/StereoSessionIsis.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/Sessions/ISIS/PhotometricOutlier.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/shared_ptr.hpp>
namespace fs = boost::filesystem;

#include <algorithm>

using namespace vw;
using namespace vw::camera;
using namespace asp;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

// Process a single ISIS image to find an ideal min max. The reason we
// need to do this, is that ASP is to get image intensity values in
// the range of 0-1. To some extent we are compressing the dynamic
// range, but we try to minimize that.
void find_ideal_isis_range( std::string const& in_file,
                            std::string const& tag,
                            float & isis_lo, float & isis_hi ) {

  boost::shared_ptr<DiskImageResourceIsis> isis_rsrc( new DiskImageResourceIsis(in_file) );
  DiskImageView<PixelGray<float> > disk_image(isis_rsrc);

  float isis_mean, isis_std;

  // Calculating statistics. We subsample the images so statistics
  // only does about a million samples.
  {
    vw_out(InfoMessage) << "\t--> Computing statistics for the "+tag+" image\n";
    int left_stat_scale = int(ceil(sqrt(float(disk_image.cols())*float(disk_image.rows()) / 1000000)));
    ChannelAccumulator<math::CDFAccumulator<float> > accumulator;
    for_each_pixel(
      subsample(create_mask( edge_extend(disk_image, ConstantEdgeExtension()),
                             isis_rsrc->valid_minimum(),
                             isis_rsrc->valid_maximum() ),
                left_stat_scale ),
      accumulator );
    isis_lo = accumulator.quantile(0);
    isis_hi = accumulator.quantile(1);
    isis_mean = accumulator.approximate_mean();
    isis_std  = accumulator.approximate_stddev();

    vw_out(InfoMessage) << "\t  "+tag+": [ lo:" << isis_lo << " hi:" << isis_hi
                        << " m: " << isis_mean << " s: " << isis_std <<  "]\n";
  }

  // Normalizing to -+2 sigmas around mean
  if ( stereo_settings().force_max_min == 0 ) {
    vw_out(InfoMessage) << "\t--> Adjusting hi and lo to -+2 sigmas around mean.\n";

    if ( isis_lo < isis_mean - 2*isis_std )
      isis_lo = isis_mean - 2*isis_std;
    if ( isis_hi > isis_mean + 2*isis_std )
      isis_hi = isis_mean + 2*isis_std;

    vw_out(InfoMessage) << "\t    "+tag+" changed: [ lo:"
                        << isis_lo << " hi:" << isis_hi << "]\n";
  }
}

// This actually modifies and writes the pre-processed image.
void write_preprocessed_isis_image( BaseOptions const& opt,
                                    std::string const& in_file,
                                    std::string const& out_file,
                                    std::string const& tag,
                                    float isis_lo, float isis_hi,
                                    float out_lo, float out_hi,
                                    Matrix<double> const& matrix,
                                    Vector2i const& crop_size ) {
  DiskImageView<PixelGray<float> > disk_image(in_file);
  ImageViewRef<PixelGray<float> > applied_image;
  if ( matrix == math::identity_matrix<3>() ) {
    applied_image =
      crop(edge_extend(clamp(normalize(remove_isis_special_pixels(disk_image,
                                                                  isis_lo, isis_hi, out_lo),
                           out_lo, out_hi, 0.0, 1.0)),
                       ZeroEdgeExtension()),
           0, 0, crop_size[0], crop_size[1]);
  } else {
    applied_image =
      transform(clamp(normalize(remove_isis_special_pixels(disk_image,
                                                           isis_lo, isis_hi,
                                                           out_lo),
                                out_lo, out_hi, 0.0, 1.0)),
                HomographyTransform(matrix),
                crop_size[0], crop_size[1]);
  }

  // Write the results to disk.
  vw_out() << "\t--> Writing normalized images.\n";
  block_write_gdal_image( out_file, applied_image, opt,
                          TerminalProgressCallback("asp", "\t  "+tag+":  ") );
}

void
asp::StereoSessionIsis::pre_preprocessing_hook(std::string const& input_file1,
                                               std::string const& input_file2,
                                               std::string & output_file1,
                                               std::string & output_file2) {
  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";

  if ( fs::exists(output_file1) && fs::exists(output_file2) ) {
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelGray<float32> > out1(output_file1);
      DiskImageView<PixelGray<float32> > out2(output_file2);
      vw_out(InfoMessage) << "\t--> Using cached normalized input images.\n";
      vw_settings().reload_config();
      return;
    } catch (vw::ArgumentErr const& e) {
      // This throws on a corrupted file.
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
    }
  }

  float left_lo, left_hi, right_lo, right_hi;
  find_ideal_isis_range( input_file1, "left", left_lo, left_hi );
  find_ideal_isis_range( input_file2, "right", right_lo, right_hi );

  // Working out alignment
  float lo = std::min(left_lo, right_lo);  // Finding global
  float hi = std::max(left_hi, right_hi);
  Matrix<double> align_matrix(3,3);
  align_matrix.set_identity();
  if ( stereo_settings().alignment_method == "homography" ) {
    std::string match_filename
      = ip::match_filename(m_out_prefix, input_file1, input_file2);

    if (!fs::exists(match_filename)) {
      boost::shared_ptr<camera::CameraModel> cam1, cam2;
      camera_models( cam1, cam2 );

      boost::shared_ptr<IsisCameraModel> isis_cam =
        boost::dynamic_pointer_cast<IsisCameraModel>(cam1);

      Vector3 radii = isis_cam->target_radii();
      cartography::Datum datum("","","", (radii[0] + radii[1]) / 2, radii[2], 0);

      boost::shared_ptr<DiskImageResource> rsrc1( DiskImageResource::open(input_file1) ),
        rsrc2( DiskImageResource::open(input_file2) );

      bool inlier =
        ip_matching_w_alignment( cam1.get(), cam2.get(),
                                 DiskImageView<PixelGray<float> >(rsrc1),
                                 DiskImageView<PixelGray<float> >(rsrc2),
                                 datum, match_filename,
                                 rsrc1->has_nodata_read() ? rsrc1->nodata_read() :
                                 std::numeric_limits<double>::quiet_NaN(),
                                 rsrc2->has_nodata_read() ? rsrc2->nodata_read() :
                                 std::numeric_limits<double>::quiet_NaN() );
      if ( !inlier ) {
        fs::remove( match_filename );
        vw_throw( IOErr() << "Unable to match left and right images." );
      }
    }

    std::vector<ip::InterestPoint> ip1, ip2;
    ip::read_binary_match_file( match_filename, ip1, ip2  );
    align_matrix = homography_fit(ip2, ip1, bounding_box(DiskImageView<PixelGray<float> >(input_file1)) );

    write_matrix( m_out_prefix + "-align.exr", align_matrix );
    vw_out() << "\t--> Aligning right image to left using homography:\n"
             << "\t      " << align_matrix << "\n";
  } else if ( stereo_settings().alignment_method == "epipolar" ) {
    vw_throw( NoImplErr() << "StereoSessionISIS doesn't support epipolar rectification" );
  }

  // Getting left image size
  Vector2i left_size = file_image_size( input_file1 ),
    right_size = file_image_size( input_file2 );

  // Apply alignment and normalization
  if (stereo_settings().individually_normalize == 0 ) {
    vw_out() << "\t--> Normalizing globally to: ["<<lo<<" "<<hi<<"]\n";
    write_preprocessed_isis_image( m_options, input_file1, output_file1, "left",
                                   left_lo, left_hi, lo, hi,
                                   math::identity_matrix<3>(), left_size );
    if ( stereo_settings().alignment_method == "none" )
      write_preprocessed_isis_image( m_options, input_file2, output_file2, "right",
                                     right_lo, right_hi, lo, hi,
                                     math::identity_matrix<3>(), right_size );
    else
      write_preprocessed_isis_image( m_options, input_file2, output_file2, "right",
                                     right_lo, right_hi, lo, hi,
                                     align_matrix, left_size );
  } else {
    vw_out() << "\t--> Individually normalizing.\n";
    write_preprocessed_isis_image( m_options, input_file1, output_file1, "left",
                                   left_lo, left_hi, left_lo, left_hi,
                                   math::identity_matrix<3>(), left_size );
    if ( stereo_settings().alignment_method == "none" )
      write_preprocessed_isis_image( m_options, input_file2, output_file2, "right",
                                     right_lo, right_hi, right_lo, right_hi,
                                     math::identity_matrix<3>(), right_size );
    else
      write_preprocessed_isis_image( m_options, input_file2, output_file2, "right",
                                     right_lo, right_hi, right_lo, right_hi,
                                     align_matrix, left_size );
  }
}

inline std::string write_shadow_mask( BaseOptions const& opt,
                                      std::string const& output_prefix,
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

  block_write_gdal_image( output_mask, mask, opt,
                          TerminalProgressCallback("asp","\t  Shadow:") );
  return output_mask;
}

// Stage 2: Correlation
//
// Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
// Post file is a disparity map.            ( ImageView<PixelMask<Vector2f> > )
void
asp::StereoSessionIsis::pre_filtering_hook(std::string const& input_file,
                                           std::string & output_file) {
  output_file = input_file;

  if (stereo_settings().mask_flatfield) {
    // ****************************************************
    // The following code is for Apollo Metric Camera ONLY!
    // (use at your own risk)
    // ****************************************************
    vw_out() << "\t--> Masking pixels that are less than 0.0.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    output_file = m_out_prefix + "-R-masked.exr";

    std::string shadowLmask_name =
      write_shadow_mask( m_options, m_out_prefix, m_left_image_file,
                         "-lMask.tif" );
    std::string shadowRmask_name =
      write_shadow_mask( m_options, m_out_prefix, m_right_image_file,
                         "-rMask.tif" );

    DiskImageView<uint8> shadowLmask( shadowLmask_name );
    DiskImageView<uint8> shadowRmask( shadowRmask_name );

    DiskImageView<PixelMask<Vector2f> > disparity_disk_image(input_file);
    ImageViewRef<PixelMask<Vector2f> > disparity_map =
      stereo::disparity_mask(disparity_disk_image,
                             shadowLmask, shadowRmask );

    DiskImageResourceOpenEXR disparity_map_rsrc(output_file, disparity_map.format() );
    Vector2i block_size(std::min<size_t>(vw_settings().default_tile_size(),
                                         disparity_map.cols()),
                        std::min<size_t>(vw_settings().default_tile_size(),
                                         disparity_map.rows()));
    disparity_map_rsrc.set_block_write_size(block_size);
    block_write_image( disparity_map_rsrc, disparity_map,
                       TerminalProgressCallback( "asp", "\t--> Saving Mask :") );
  }
}

// Reverse any pre-alignment that was done to the disparity.
ImageViewRef<PixelMask<Vector2f> >
asp::StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file) {

  std::string dust_result = input_file;
  if ( stereo_settings().mask_flatfield ) {
    // ****************************************************
    // The following code is for Apollo Metric Camera ONLY!
    // (use at your own risk)
    // ****************************************************
    vw_out() << "\t--> Masking pixels that appear to be dust.  (NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    photometric_outlier_rejection( m_options, m_out_prefix, input_file,
                                   dust_result, stereo_settings().corr_kernel[0] );
  }

  DiskImageView<PixelMask<Vector2f> > disparity_map(dust_result);

  ImageViewRef<PixelMask<Vector2f> > result;
  if ( stereo_settings().alignment_method == "homography" ) {
    // We used a homography to line up the images, we may want
    // to generate pre-alignment disparities before passing this information
    // onto the camera model in the next stage of the stereo pipeline.
    Matrix<double> align_matrix;
    read_matrix(align_matrix, m_out_prefix + "-align.exr");
    vw_out(DebugMessage,"asp") << "Alignment Matrix: " << align_matrix << "\n";

    // Remove pixels that are outside the bounds of the secondary image.
    DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);
    result =
      stereo::disparity_range_mask(stereo::transform_disparities(disparity_map,
                                                                 HomographyTransform(align_matrix)),
                                   Vector2f(0,0),
                                   Vector2f( right_disk_image.cols(),
                                             right_disk_image.rows() ) );
  } else {
    result = disparity_map;
  }

  return result;
}

boost::shared_ptr<vw::camera::CameraModel>
asp::StereoSessionIsis::camera_model(std::string const& image_file,
                                     std::string const& camera_file) {

  if (boost::ends_with(boost::to_lower_copy(camera_file), ".isis_adjust")){
    // Creating Equations for the files
    std::ifstream input( camera_file.c_str() );
    boost::shared_ptr<asp::BaseEquation> posF = read_equation( input );
    boost::shared_ptr<asp::BaseEquation> poseF = read_equation( input );
    input.close();

    // Finally creating camera model
    return boost::shared_ptr<camera::CameraModel>(new IsisAdjustCameraModel( image_file, posF, poseF ));

  } else {
    return boost::shared_ptr<camera::CameraModel>(new IsisCameraModel(image_file));
  }

}
