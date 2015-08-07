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


/// \file CameraModelLoader.h
///

#ifndef __STEREO_SESSION_CAMERAMODELLOADER_H__
#define __STEREO_SESSION_CAMERAMODELLOADER_H__


#include <vw/Camera.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Math/Matrix.h>
#include <xercesc/util/PlatformUtils.hpp>

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/IsisIO/Equation.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Camera/DG_XML.h>
#include <asp/Camera/RPCModel.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>

// TODO: Break this up. Each of these functions must go back to their
// individual Session directories rather than being collected here.

namespace asp {

  // !!! This class is not meant to be invoked directly !!!
  // Use instead the interface load_camera_model in StereoSessionConcrete.tcc.

  class CameraModelLoader {
  public:

    typedef boost::shared_ptr<vw::camera::CameraModel> CameraModelPtr;

    // Setup/teardown code is handled here
    CameraModelLoader();
    ~CameraModelLoader();

    // Camera model loading functions
    // All private functions!
    boost::shared_ptr<asp::RPCModel> load_rpc_camera_model(std::string const& path) const;
    CameraModelPtr load_dg_camera_model     (std::string const& path) const;
    CameraModelPtr load_pinhole_camera_model(std::string const& path) const;
    CameraModelPtr load_isis_camera_model   (std::string const& path) const;
    
    boost::shared_ptr<vw::camera::CAHVModel> load_cahv_pinhole_camera_model(std::string const& path) const;

  }; // End class StereoSessionFactory

// Function definitions

inline CameraModelLoader::CameraModelLoader()
{
  xercesc::XMLPlatformUtils::Initialize();
}

inline CameraModelLoader::~CameraModelLoader()
{
  xercesc::XMLPlatformUtils::Terminate();
}


/// Load an RPC camera file
inline boost::shared_ptr<asp::RPCModel> CameraModelLoader::load_rpc_camera_model(std::string const& path) const
{
  // Try the default loading method
  RPCModel* rpc_model = NULL;
  try {
    RPCXML rpc_xml; // This is for reading XML files
    rpc_xml.read_from_file(path);
    rpc_model = new RPCModel(*rpc_xml.rpc_ptr()); // Copy the value
  } catch (...) {}
  if (!rpc_model) // The default loading method failed, try the backup method.
  {
    rpc_model = new RPCModel(path); // This is for reading .tif files
  }

  // We don't catch an error here because the user will need to
  // know of a failure at this point.
  return boost::shared_ptr<asp::RPCModel>(rpc_model);
}



// TODO: Move this!
/// Helper class for converting to floating point seconds based on a given reference.
class SecondsFrom
{
  boost::posix_time::ptime m_reference;
public:
  inline SecondsFrom( boost::posix_time::ptime const& time ) : m_reference(time) {}

  inline double operator()( boost::posix_time::ptime const& time ) const {
    return double( (time - m_reference).total_microseconds() ) / 1e6;
  }
};

/// Wrapper around boost::posix_time::time_from_string()
inline boost::posix_time::ptime parse_time(std::string str)
{
  try{
    return boost::posix_time::time_from_string(str);
  }catch(...){
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse time from string: " << str << "\n");
  }
  return boost::posix_time::time_from_string(str); // Never reached!
}

/// Load a DG camera file
inline boost::shared_ptr<vw::camera::CameraModel> CameraModelLoader::load_dg_camera_model(std::string const& path) const
{
  //vw_out() << "DEBUG - Loading DG camera file: " << camera_file << std::endl;

  // Parse the Digital Globe XML file
  GeometricXML geo;
  AttitudeXML  att;
  EphemerisXML eph;
  ImageXML     img;
  RPCXML       rpc;
  read_xml( path, geo, att, eph, img, rpc );

  // Convert measurements in millimeters to pixels.
  geo.principal_distance /= geo.detector_pixel_pitch;
  geo.detector_origin    /= geo.detector_pixel_pitch;

  bool correct_velocity_aberration = !stereo_settings().disable_correct_velocity_aberration;

  // Convert all time measurements to something that boost::date_time can read.
  boost::replace_all( eph.start_time,            "T", " " );
  boost::replace_all( img.tlc_start_time,        "T", " " );
  boost::replace_all( img.first_line_start_time, "T", " " );
  boost::replace_all( att.start_time,            "T", " " );

  // Convert UTC time measurements to line measurements. Ephemeris
  // start time will be our reference frame to calculate seconds against.
  SecondsFrom convert( parse_time( eph.start_time ) );

  // I'm going make the assumption that EPH and ATT are sampled at the same rate and time.
  VW_ASSERT( eph.position_vec.size() == att.quat_vec.size(),
             vw::MathErr() << "Ephemeris and Attitude don't have the same number of samples." );
  VW_ASSERT( eph.start_time == att.start_time && eph.time_interval == att.time_interval,
             vw::MathErr() << "Ephemeris and Attitude don't seem to sample with the same t0 or dt." );

  // Convert ephemeris to be position of camera. Change attitude to
  // be the rotation from camera frame to world frame. We also add an
  // additional rotation to the camera frame so X is the horizontal
  // direction to the picture and +Y points down the image (in the direction of flight).
  vw::Quat sensor_coordinate = vw::math::euler_xyz_to_quaternion(vw::Vector3(0,0,geo.detector_rotation - M_PI/2));
  for ( size_t i = 0; i < eph.position_vec.size(); i++ ) {
    eph.position_vec[i] += att.quat_vec[i].rotate( geo.perspective_center );
    att.quat_vec[i] = att.quat_vec[i] * geo.camera_attitude * sensor_coordinate;
  }

  // Load up the time interpolation class. If the TLCList only has
  // one entry ... then we have to manually drop in the slope and offset.
  if ( img.tlc_vec.size() == 1 ) {
    double direction = 1;
    if ( boost::to_lower_copy( img.scan_direction ) != "forward" ) {
      direction = -1;
    }
    img.tlc_vec.push_back( std::make_pair(img.tlc_vec.front().first +
                                          img.avg_line_rate, direction) );
  }

  // Build the TLCTimeInterpolation object and do a quick sanity check.
  vw::camera::TLCTimeInterpolation tlc_time_interpolation( img.tlc_vec,
                                                               convert( parse_time( img.tlc_start_time ) ) );
  VW_ASSERT( fabs( convert( parse_time( img.first_line_start_time ) ) -
                   tlc_time_interpolation( 0 ) ) < fabs( 1.0 / (10.0 * img.avg_line_rate ) ),
             vw::MathErr() << "First Line Time and output from TLC lookup table do not agree of the ephemeris time for the first line of the image." );


  double et0 = convert( parse_time( eph.start_time ) );
  double at0 = convert( parse_time( att.start_time ) );
  double edt = eph.time_interval;
  double adt = att.time_interval;
  return CameraModelPtr(new DGCameraModel(vw::camera::PiecewiseAPositionInterpolation(eph.position_vec, eph.velocity_vec, et0, edt ),
                                          vw::camera::LinearPiecewisePositionInterpolation(eph.velocity_vec, et0, edt),
                                          vw::camera::SLERPPoseInterpolation(att.quat_vec, at0, adt),
                                          tlc_time_interpolation, img.image_size,
                                          subvector(inverse(sensor_coordinate).rotate(vw::Vector3(geo.detector_origin[0],
                                                                                                  geo.detector_origin[1],
                                                                                                  0)
                                                                                     ),
                                                    0, 2),
                                          geo.principal_distance, correct_velocity_aberration)
                    );
} // End function load_dg_camera_model()




/// Load a pinhole camera model
inline boost::shared_ptr<vw::camera::CameraModel> CameraModelLoader::load_pinhole_camera_model(std::string const& path) const
{
  // Keypoint alignment and everything else just gets camera models
  std::string lcase_file = boost::to_lower_copy(path);
  if (boost::ends_with(lcase_file,".cahvore") ) {
    return CameraModelPtr( new vw::camera::CAHVOREModel(path) );
  } else if (boost::ends_with(lcase_file,".cahvor") ||
             boost::ends_with(lcase_file,".cmod"  )   ) {
    return CameraModelPtr( new vw::camera::CAHVORModel(path) );
  } else if ( boost::ends_with(lcase_file,".cahv") ||
              boost::ends_with(lcase_file,".pin" )   ) {
    return CameraModelPtr( new vw::camera::CAHVModel(path) );
  } else if ( boost::ends_with(lcase_file,".pinhole") ||
              boost::ends_with(lcase_file,".tsai"   )   ) {
    return CameraModelPtr( new vw::camera::PinholeModel(path) );
  } else {
    vw::vw_throw(vw::ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
  }
}

inline boost::shared_ptr<vw::camera::CAHVModel> CameraModelLoader::load_cahv_pinhole_camera_model(std::string const& path) const
{
  // Get the image size
  vw::DiskImageView<float> disk_image(path);
  vw::Vector2i image_size(disk_image.cols(), disk_image.rows());

  // Load the appropriate camera model object and if necessary 
  // convert it to the CAHVModel type.
  std::string lcase_file = boost::to_lower_copy(path);
  boost::shared_ptr<vw::camera::CAHVModel> cahv(new vw::camera::CAHVModel);
  if (boost::ends_with(lcase_file, ".cahvore") ) {
    vw::camera::CAHVOREModel cahvore(path);
    *(cahv.get()) = vw::camera::linearize_camera(cahvore, image_size, image_size);
  } else if (boost::ends_with(lcase_file, ".cahvor")  ||
             boost::ends_with(lcase_file, ".cmod"  )   ) {
    vw::camera::CAHVORModel cahvor(path);
    *(cahv.get()) = vw::camera::linearize_camera(cahvor, image_size, image_size);

  } else if ( boost::ends_with(lcase_file, ".cahv") ||
              boost::ends_with(lcase_file, ".pin" )) {
    *(cahv.get()) = vw::camera::CAHVModel(path);
    
  } else if ( boost::ends_with(lcase_file, ".pinhole") ||
              boost::ends_with(lcase_file, ".tsai"   )   ) {
    vw::camera::PinholeModel left_pin(path);
    *(cahv.get()) = vw::camera::linearize_camera(left_pin);
    
  } else {
    vw_throw(vw::ArgumentErr() << "CameraModelLoader::load_cahv_pinhole_camera_model - unsupported camera file type.\n");
  }

  return cahv;
}



/// Load an ISIS camera model
inline boost::shared_ptr<vw::camera::CameraModel> CameraModelLoader::load_isis_camera_model(std::string const& path) const
{
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  return CameraModelPtr(new vw::camera::IsisCameraModel(path));
#endif
  // If ISIS was not enabled in the build, just throw an exception.
  vw::vw_throw( vw::NoImplErr() << "\nCannot load ISIS files because ISIS was not enabled in the build!.\n");

} // End function load_isis_camera_model()








} // end namespace asp

#endif // __STEREO_SESSION_CAMERAMODELLOADER_H__
