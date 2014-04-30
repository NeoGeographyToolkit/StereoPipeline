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


/// \file StereoSessionDG.cc
///
#include <vw/Image/ImageMath.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Transform.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Math/Matrix.h>
#include <vw/Cartography/Datum.h>

#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Sessions/DG/LinescanDGModel.h>
#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/RPC/RPCModel.h>
#include <asp/Sessions/DG/XML.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <xercesc/util/PlatformUtils.hpp>

using namespace vw;
using namespace asp;
using namespace xercesc;

namespace pt = boost::posix_time;
namespace fs = boost::filesystem;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector2f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
}

// Helper class for converting to floating point seconds based on a
// given reference.
class SecondsFrom {
  pt::ptime m_reference;
public:
  SecondsFrom( pt::ptime const& time ) : m_reference(time) {}

  double operator()( pt::ptime const& time ) const {
    return double( (time - m_reference).total_microseconds() ) / 1e6;
  }
};


namespace asp {

  pt::ptime parse_time(std::string str){
    try{
      return pt::time_from_string(str);
    }catch(...){
      vw_throw(ArgumentErr() << "Failed to parse time from string: "
               << str << "\n");
    }
    return pt::time_from_string(str); // never reached
  }


  // These are initializers and closers for Xercesc since we use it to
  // read our RPC models.
  StereoSessionDG::StereoSessionDG() {
    XMLPlatformUtils::Initialize();
  }

  StereoSessionDG::~StereoSessionDG() {
    XMLPlatformUtils::Terminate();
  }

  // Provide our camera model
  boost::shared_ptr<camera::CameraModel>
  StereoSessionDG::camera_model( std::string const& /*image_file*/,
                                 std::string const& camera_file ) {

    GeometricXML geo;
    AttitudeXML att;
    EphemerisXML eph;
    ImageXML img;
    RPCXML rpc;
    read_xml( camera_file, geo, att, eph, img, rpc );

    // Convert measurements in millimeters to pixels.
    geo.principal_distance /= geo.detector_pixel_pitch;
    geo.detector_origin /= geo.detector_pixel_pitch;

    bool correct_velocity_aberration = !stereo_settings().disable_correct_velocity_aberration;

    // Convert all time measurements to something that boost::date_time can read.
    boost::replace_all( eph.start_time, "T", " " );
    boost::replace_all( img.tlc_start_time, "T", " " );
    boost::replace_all( img.first_line_start_time, "T", " " );
    boost::replace_all( att.start_time, "T", " " );

    // Convert UTC time measurements to line measurements. Ephemeris
    // start time will be our reference frame to calculate seconds
    // against.
    SecondsFrom convert( parse_time( eph.start_time ) );

    // I'm going make the assumption that EPH and ATT are sampled at the
    // same rate and time.
    VW_ASSERT( eph.position_vec.size() == att.quat_vec.size(),
               MathErr() << "Ephemeris and Attitude don't have the same number of samples." );
    VW_ASSERT( eph.start_time == att.start_time && eph.time_interval == att.time_interval,
               MathErr() << "Ephemeris and Attitude don't seem to sample with the same t0 or dt." );

    // Convert ephemeris to be position of camera. Change attitude to be
    // be the rotation from camera frame to world frame. We also add an
    // additional rotation to the camera frame so X is the horizontal
    // direction to the picture and +Y points down the image (in the
    // direction of flight).
    Quat sensor_coordinate = math::euler_xyz_to_quaternion(Vector3(0,0,geo.detector_rotation * M_PI/180.0 - M_PI/2));
    for ( size_t i = 0; i < eph.position_vec.size(); i++ ) {
      eph.position_vec[i] += att.quat_vec[i].rotate( geo.perspective_center );
      att.quat_vec[i] = att.quat_vec[i] * geo.camera_attitude * sensor_coordinate;
    }

    // Load up the time interpolation class. If the TLCList only has
    // one entry ... then we have to manually drop in the slope and
    // offset.
    if ( img.tlc_vec.size() == 1 ) {
      double direction = 1;
      if ( boost::to_lower_copy( img.scan_direction ) !=
           "forward" ) {
        direction = -1;
      }
      img.tlc_vec.push_back( std::make_pair(img.tlc_vec.front().first +
                                            img.avg_line_rate, direction) );
    }

    // Build the TLCTimeInterpolation object and do a quick sanity check.
    camera::TLCTimeInterpolation tlc_time_interpolation( img.tlc_vec,
                                                         convert( parse_time( img.tlc_start_time ) ) );
    VW_ASSERT( fabs( convert( parse_time( img.first_line_start_time ) ) -
                     tlc_time_interpolation( 0 ) ) < fabs( 1.0 / (10.0 * img.avg_line_rate ) ),
               MathErr() << "First Line Time and output from TLC lookup table do not agree of the ephemeris time for the first line of the image." );

    typedef LinescanDGModel<camera::PiecewiseAPositionInterpolation, camera::LinearPiecewisePositionInterpolation, camera::SLERPPoseInterpolation, camera::TLCTimeInterpolation> camera_type;
    typedef boost::shared_ptr<camera::CameraModel> result_type;
    double et0 = convert( parse_time( eph.start_time ) );
    double at0 = convert( parse_time( att.start_time ) );
    double edt = eph.time_interval;
    double adt = att.time_interval;
    return result_type(new camera_type(camera::PiecewiseAPositionInterpolation(eph.position_vec, eph.velocity_vec,
                                                                               et0, edt ),
                                       camera::LinearPiecewisePositionInterpolation(eph.velocity_vec, et0, edt),
                                       camera::SLERPPoseInterpolation(att.quat_vec, at0, adt),
                                       tlc_time_interpolation, img.image_size,
                                       subvector(inverse(sensor_coordinate).rotate(Vector3(geo.detector_origin[0],
                                                                                           geo.detector_origin[1],
                                                                                           0)), 0, 2),
                                       geo.principal_distance, correct_velocity_aberration)
                      );
  }

  bool StereoSessionDG::ip_matching( std::string const& match_filename,
                                     double left_nodata_value,
                                     double right_nodata_value ) {
    // Load the unmodified images
    DiskImageView<float> left_disk_image( m_left_image_file ),
      right_disk_image( m_right_image_file );

    boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
    camera_models( left_cam, right_cam );

    return
      ip_matching_w_alignment( left_cam.get(), right_cam.get(),
                               left_disk_image, right_disk_image,
                               cartography::Datum("WGS84"), match_filename,
                               left_nodata_value,
                               right_nodata_value);
  }

  StereoSessionDG::left_tx_type
  StereoSessionDG::tx_left() const {
    Matrix<double> tx = math::identity_matrix<3>();
    if ( stereo_settings().alignment_method == "homography" ||
         stereo_settings().alignment_method == "affineepipolar" ) {
      read_matrix( tx, m_out_prefix + "-align-L.exr" );
    }
    return left_tx_type( tx );
  }

  StereoSessionDG::right_tx_type
  StereoSessionDG::tx_right() const {
    Matrix<double> tx = math::identity_matrix<3>();
    if ( stereo_settings().alignment_method == "homography" ||
         stereo_settings().alignment_method == "affineepipolar" ) {
      read_matrix( tx, m_out_prefix + "-align-R.exr" );
    }
    return right_tx_type( tx );
  }

  void StereoSessionDG::pre_preprocessing_hook(std::string const& left_input_file,
                                               std::string const& right_input_file,
                                               std::string &left_output_file,
                                               std::string &right_output_file) {

    boost::shared_ptr<DiskImageResource>
      left_rsrc( DiskImageResource::open(m_left_image_file) ),
      right_rsrc( DiskImageResource::open(m_right_image_file) );

    float left_nodata_value, right_nodata_value;
    get_nodata_values(left_rsrc, right_rsrc, left_nodata_value, right_nodata_value);

    // Load the unmodified images
    DiskImageView<float> left_disk_image( left_rsrc ), right_disk_image( right_rsrc );

    // Filenames of normalized images
    left_output_file = m_out_prefix + "-L.tif";
    right_output_file = m_out_prefix + "-R.tif";

    // If these files already exist, don't bother writting them again.
    bool rebuild = false;
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<float> test_left(left_output_file);
      DiskImageView<float> test_right(right_output_file);
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
      rebuild = true;
    } catch (vw::ArgumentErr const& e ) {
      // Throws on a corrupted file.
      vw_settings().reload_config();
      rebuild = true;
    }

    if (!rebuild) {
      vw_out() << "\t--> Using cached L and R files.\n";
      return;
    }

    ImageViewRef< PixelMask<float> > left_masked_image
      = create_mask_less_or_equal(left_disk_image, left_nodata_value);
    ImageViewRef< PixelMask<float> > right_masked_image
      = create_mask_less_or_equal(right_disk_image, right_nodata_value);

    Vector4f left_stats  = gather_stats( left_masked_image,  "left" );
    Vector4f right_stats = gather_stats( right_masked_image, "right" );

    ImageViewRef< PixelMask<float> > Limg, Rimg;
    std::string lcase_file = boost::to_lower_copy(m_left_camera_file);

    if ( stereo_settings().alignment_method == "homography" ||
         stereo_settings().alignment_method == "affineepipolar" ) {
      std::string match_filename
        = ip::match_filename(m_out_prefix, left_input_file, right_input_file);

      if (!fs::exists(match_filename)) {
        // This is calling an internal virtualized method.
        bool inlier =
          ip_matching( match_filename,
                       left_nodata_value,
                       right_nodata_value );
        if ( !inlier ) {
          fs::remove( match_filename );
          vw_throw( IOErr() << "Unable to match left and right images." );
        }
      } else {
        vw_out() << "\t--> Using cached match file: " << match_filename << "\n";
      }

      std::vector<ip::InterestPoint> left_ip, right_ip;
      ip::read_binary_match_file( match_filename, left_ip, right_ip  );
      Matrix<double> align_left_matrix = math::identity_matrix<3>(),
        align_right_matrix = math::identity_matrix<3>();

      Vector2i left_size = file_image_size( left_input_file ),
        right_size = file_image_size( right_input_file );

      if ( stereo_settings().alignment_method == "homography" ) {
        left_size =
          homography_rectification( left_size, right_size, left_ip, right_ip,
                                    align_left_matrix, align_right_matrix );
        vw_out() << "\t--> Aligning right image to left using matrices:\n"
                 << "\t      " << align_left_matrix << "\n"
                 << "\t      " << align_right_matrix << "\n";
      } else {
        left_size =
          affine_epipolar_rectification( left_size, right_size,
                                         left_ip, right_ip,
                                         align_left_matrix,
                                         align_right_matrix );
        vw_out() << "\t--> Aligning left and right images using affine matrices:\n"
                 << "\t      " << submatrix(align_left_matrix,0,0,2,3) << "\n"
                 << "\t      " << submatrix(align_right_matrix,0,0,2,3) << "\n";
      }
      write_matrix( m_out_prefix + "-align-L.exr", align_left_matrix );
      write_matrix( m_out_prefix + "-align-R.exr", align_right_matrix );

      // Applying alignment transform
      Limg = transform(left_masked_image,
                       HomographyTransform(align_left_matrix),
                       left_size.x(), left_size.y() );
      Rimg = transform(right_masked_image,
                       HomographyTransform(align_right_matrix),
                       left_size.x(), left_size.y() );
    } else if ( stereo_settings().alignment_method == "epipolar" ) {
      vw_throw( NoImplErr() << "StereoSessionDG does not support epipolar rectification" );
    } else {
      // Do nothing just provide the original files.
      Limg = left_masked_image;
      Rimg = right_masked_image;
    }

    // Apply our normalization options.
    normalize_images(stereo_settings().force_use_entire_range,
                     stereo_settings().individually_normalize,
                     left_stats, right_stats, Limg, Rimg);

    // The output no-data value must be < 0 as we scale the images to [0, 1].
    float output_nodata = -32768.0;

    // Enforce no predictor in compression, it works badly with L.tif and R.tif.
    asp::BaseOptions options = m_options;
    options.gdal_options["PREDICTOR"] = "1";

    vw_out() << "\t--> Writing pre-aligned images.\n";
    block_write_gdal_image( left_output_file, apply_mask(Limg, output_nodata),
                            output_nodata, options,
                            TerminalProgressCallback("asp","\t  L:  ") );

    if ( stereo_settings().alignment_method == "none" )
      block_write_gdal_image( right_output_file, apply_mask(Rimg, output_nodata),
                              output_nodata, options,
                              TerminalProgressCallback("asp","\t  R:  ") );
    else
      block_write_gdal_image( right_output_file,
                              apply_mask(crop(edge_extend(Rimg, ConstantEdgeExtension()), bounding_box(Limg)), output_nodata),
                              output_nodata, options,
                              TerminalProgressCallback("asp","\t  R:  ") );
  }

}
