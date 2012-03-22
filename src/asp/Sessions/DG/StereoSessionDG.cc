// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file StereoSessionDG.cc
///

// Ames Stereo Pipeline
#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Sessions/DG/LinescanDGModel.h>
#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/DG/XML.h>

// Vision Workbench
#include <vw/Camera/Extrinsics.h>
#include <vw/Math/EulerAngles.h>

// Std
#include <iostream>
#include <string>

// Other
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>

// Boost
#include <boost/date_time/posix_time/posix_time.hpp>


using namespace vw;
using namespace asp;
namespace pt = boost::posix_time;
namespace fs = boost::filesystem;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
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

// Xerces-C initialize
asp::StereoSessionDG::StereoSessionDG() {
  xercesc::XMLPlatformUtils::Initialize();
}

// Provide our camera model
boost::shared_ptr<camera::CameraModel>
asp::StereoSessionDG::camera_model( std::string const& /*image_file*/,
                                    std::string const& camera_file ) {
  GeometricXML geo;
  AttitudeXML att;
  EphemerisXML eph;
  ImageXML img;
  read_xml( camera_file, geo, att, eph, img );

  // Convert measurements in millimeters to pixels.
  geo.principal_distance /= geo.detector_pixel_pitch;
  geo.detector_origin /= geo.detector_pixel_pitch;

  // Convert all time measurements to something that boost::date_time can read.
  boost::replace_all( eph.start_time, "T", " " );
  boost::replace_all( img.tlc_start_time, "T", " " );
  boost::replace_all( img.first_line_start_time, "T", " " );
  boost::replace_all( att.start_time, "T", " " );

  // Convert UTC time measurements to line measurements. Ephemeris
  // start time will be our reference frame to calculate seconds
  // against.
  SecondsFrom convert( pt::time_from_string( eph.start_time ) );

  // I'm going make the assumption that EPH and ATT are sampled at the
  // same rate and time.
  VW_ASSERT( eph.position_vec.size() == att.quat_vec.size(),
             MathErr() << "Ephemeris and Attitude don't have the same number of samples." );
  VW_ASSERT( eph.start_time == att.start_time && eph.time_interval == att.time_interval,
             MathErr() << "Ephemeris and Attitude don't seem to sample with the same t0 or dt." );

  // I also don't support optical distortion yet.
  VW_ASSERT( geo.optical_polyorder <= 0,
             NoImplErr() << "Cameras with optical distortion are not supported currently." );

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

  typedef LinescanDGModel<camera::PiecewiseAPositionInterpolation, camera::SLERPPoseInterpolation, camera::TLCTimeInterpolation> camera_type;
  typedef boost::shared_ptr<camera::CameraModel> result_type;

  return result_type( new camera_type( camera::PiecewiseAPositionInterpolation( eph.position_vec, eph.velocity_vec,
                                                                                convert( pt::time_from_string( eph.start_time ) ),
                                                                                eph.time_interval ),
                                       camera::SLERPPoseInterpolation( att.quat_vec,
                                                                       convert( pt::time_from_string( att.start_time ) ),
                                                                       att.time_interval ),
                                       camera::TLCTimeInterpolation( img.tlc_vec,
                                                                     convert( pt::time_from_string( img.tlc_start_time ) ) ),
                                       img.image_size, subvector(inverse(sensor_coordinate).rotate(Vector3(geo.detector_origin[0],
                                                                                                  geo.detector_origin[1], 0 ) ), 0, 2 ),
                                       geo.principal_distance ) );
}

void asp::StereoSessionDG::pre_preprocessing_hook(std::string const& input_file1,
                                                  std::string const& input_file2,
                                                  std::string &output_file1,
                                                  std::string &output_file2) {

  // Load the images
  DiskImageView<PixelGray<float> > left_disk_image(m_left_image_file);
  DiskImageView<PixelGray<float> > right_disk_image(m_right_image_file);

  Vector4f left_stats = gather_stats( left_disk_image, "left" ),
    right_stats = gather_stats( right_disk_image, "right" );

  ImageViewRef<PixelGray<float> > Limg, Rimg;
  std::string lcase_file = boost::to_lower_copy(m_left_camera_file);

  if ( stereo_settings().alignment_method == "homography" ) {
    std::string match_filename =
      fs::basename(input_file1) + "__" +
      fs::basename(input_file2) + ".match";

    if (!fs::exists(match_filename)) {
      boost::shared_ptr<camera::CameraModel> cam1, cam2;
      camera_models( cam1, cam2 );

      bool inlier =
        ip_matching_w_alignment( cam1.get(), cam2.get(),
                                 left_disk_image, right_disk_image,
                                 cartography::Datum("WGS84"), match_filename );
      if ( !inlier ) {
        fs::remove( match_filename );
        vw_throw( IOErr() << "Unable to match left and right images." );
      }
    }

    std::vector<ip::InterestPoint> ip1, ip2;
    ip::read_binary_match_file( match_filename, ip1, ip2  );
    Matrix<double> align_matrix =
      homography_fit(ip2, ip1, bounding_box(left_disk_image) );
    write_matrix( m_out_prefix + "-align.exr", align_matrix );

    vw_out() << "\t--> Aligning right image to left using homography:\n"
             << "\t      " << align_matrix << "\n";

    // Applying alignment transform
    Limg = left_disk_image;
    Rimg = transform(right_disk_image,
                     HomographyTransform(align_matrix),
                     left_disk_image.cols(), left_disk_image.rows());
  } else if ( stereo_settings().alignment_method == "epipolar" ) {
    vw_throw( NoImplErr() << "StereoSessionDG doesn't support epipolar rectification" );
  } else {
    // Do nothing just provide the original files.
    Limg = left_disk_image;
    Rimg = right_disk_image;
  }

  // Apply our normalization options
  if ( stereo_settings().force_max_min > 0 ) {
    if ( stereo_settings().individually_normalize > 0 ) {
      vw_out() << "\t--> Individually normalize images to their respective Min Max\n";
      Limg = normalize( Limg, left_stats[0], left_stats[1], 0, 1.0 );
      Rimg = normalize( Rimg, right_stats[0], right_stats[1], 0, 1.0 );
    } else {
      float low = std::min(left_stats[0], right_stats[0]);
      float hi  = std::max(left_stats[1], right_stats[1]);
      vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";
      Limg = normalize( Limg, low, hi, 0, 1.0 );
      Rimg = normalize( Rimg, low, hi, 0, 1.0 );
    }
  } else {
    if ( stereo_settings().individually_normalize > 0 ) {
      vw_out() << "\t--> Individually normalize images to their respective 4 std dev window\n";
      Limg = normalize( Limg, left_stats[2] - 2*left_stats[3],
                        left_stats[2] + 2*left_stats[3], 0, 1.0 );
      Rimg = normalize( Rimg, right_stats[2] - 2*right_stats[3],
                        right_stats[2] + 2*right_stats[3], 0, 1.0 );
    } else {
      float low = std::min(left_stats[2] - 2*left_stats[3],
                           right_stats[2] - 2*right_stats[3]);
      float hi  = std::max(left_stats[2] + 2*left_stats[3],
                           right_stats[2] + 2*right_stats[3]);
      vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";
      Limg = normalize( Limg, low, hi, 0, 1.0 );
      Rimg = normalize( Rimg, low, hi, 0, 1.0 );
    }
  }

  output_file1 = m_out_prefix + "-L.tif";
  output_file2 = m_out_prefix + "-R.tif";
  vw_out() << "\t--> Writing pre-aligned images.\n";
  block_write_gdal_image( output_file1, Limg, m_options,
                          TerminalProgressCallback("asp","\t  L:  ") );
  block_write_gdal_image( output_file2, crop(edge_extend(Rimg,ConstantEdgeExtension()),bounding_box(Limg)), m_options,
                          TerminalProgressCallback("asp","\t  R:  ") );
}

// Xerces-C terminate
asp::StereoSessionDG::~StereoSessionDG() {
  xercesc::XMLPlatformUtils::Terminate();
}
