// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file StereoSessionDG.cc
///

// Ames Stereo Pipeline
#include <asp/Core/StereoSettings.h>
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

// Xerces-C terminate
asp::StereoSessionDG::~StereoSessionDG() {
  xercesc::XMLPlatformUtils::Terminate();
}
