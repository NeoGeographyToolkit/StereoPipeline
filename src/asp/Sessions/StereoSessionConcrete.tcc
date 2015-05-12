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


/// \file StereoSessionConcrete.tcc
///
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math.h>
#include <vw/Camera.h>


#include <vw/Image/ImageViewRef.h>
#include <vw/Image/MaskViews.h>
#include <vw/Stereo/DisparityMap.h>

#include <asp/asp_config.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Common.h>



#include <asp/Sessions/RPC/RPCModel.h>

#include <xercesc/util/PlatformUtils.hpp>



#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/IsisIO/Equation.h>


#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>


// For the DG loader
#include <vw/Math/EulerAngles.h>
#include <vw/Math/Matrix.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Camera/Extrinsics.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <asp/Sessions/DG/LinescanDGModel.h>
#include <asp/Sessions/DG/XML.h>

using namespace vw;

using namespace vw::camera; // For the Pinhole load function

namespace asp {



// These are initializers and closers for Xercesc since we use it to read our (XML) RPC models.
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::StereoSessionConcrete() {
  xercesc::XMLPlatformUtils::Initialize();
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::~StereoSessionConcrete() {
  xercesc::XMLPlatformUtils::Terminate();
}



// Overwrite this function so we can insert additional init code
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
void StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
  initialize(BaseOptions const& options,
             std::string const& left_image_file,
             std::string const& right_image_file,
             std::string const& left_camera_file,
             std::string const& right_camera_file,
             std::string const& out_prefix,
             std::string const& input_dem) {

  // Initialize the base class
  StereoSession::initialize(options, 
                            left_image_file,  right_image_file, 
                            left_camera_file, right_camera_file, 
                            out_prefix, input_dem);

  // Do any other initalization steps needed
  init_sensor_model  (Int2Type<STEREOMODEL_TYPE  >());
  init_disk_transform(Int2Type<DISKTRANSFORM_TYPE>());

}

// For non map projected inputs, keep the same default function as the base class.  Otherwise throw!
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
inline bool StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
ip_matching(std::string const& input_file1,
            std::string const& input_file2,
            float nodata1, float nodata2,
            std::string const& match_filename,
            vw::camera::CameraModel* cam1,
            vw::camera::CameraModel* cam2)
{
  if (IsTypeMapProjected<DISKTRANSFORM_TYPE>::value) {
    vw_throw( vw::ArgumentErr() << "StereoSessionConcrete: IP matching is not implemented as no alignment is applied to map-projected images.");
    return false;
  }
  else // Inputs are not map projected
    return StereoSession::ip_matching(input_file1, input_file2, nodata1, nodata2, match_filename, cam1, cam2);
}

//---------------------------------------------------------------------------

// Helper function to read RPC models. TODO MOVE THIS!!!!!!!!!!!!!!!!!!
inline RPCModel* read_rpc_model(std::string const& image_file,
                       std::string const& camera_file) {
  RPCModel* rpc_model = NULL;
  try {
    rpc_model = new RPCModel(image_file);
  } catch (NotFoundErr const& err) {}

  if (!rpc_model) {

    if (camera_file == ""){
      vw_throw( NotFoundErr()
                << "RPCModel: Could not find the RPC model in " << image_file
                << ", and no XML camera file was provided.\n" );
    }

    RPCXML rpc_xml;
    rpc_xml.read_from_file(camera_file);
    rpc_model = new RPCModel(*rpc_xml.rpc_ptr()); // Copy the value

    // We don't catch an error here because the user will need to
    // know of a failure at this point. We previously opened the
    // xml file safely before.
  }
  return rpc_model;
}



// TODO: Move this!
/// Helper class for converting to floating point seconds based on a given reference.
class SecondsFrom {
  boost::posix_time::ptime m_reference;
public:
  inline SecondsFrom( boost::posix_time::ptime const& time ) : m_reference(time) {}

  inline double operator()( boost::posix_time::ptime const& time ) const {
    return double( (time - m_reference).total_microseconds() ) / 1e6;
  }
};

/// Wrapper around boost::posix_time::time_from_string()
inline boost::posix_time::ptime parse_time(std::string str){
  try{
    return boost::posix_time::time_from_string(str);
  }catch(...){
    vw_throw(ArgumentErr() << "Failed to parse time from string: " << str << "\n");
  }
  return boost::posix_time::time_from_string(str); // Never reached!
}


inline boost::shared_ptr<vw::camera::CameraModel> load_dg_camera_model(std::string const& camera_file)
{
  // Parse the Digital Globe XML file
  GeometricXML geo;
  AttitudeXML  att;
  EphemerisXML eph;
  ImageXML     img;
  RPCXML       rpc;
  read_xml( camera_file, geo, att, eph, img, rpc );

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
             MathErr() << "Ephemeris and Attitude don't have the same number of samples." );
  VW_ASSERT( eph.start_time == att.start_time && eph.time_interval == att.time_interval,
             MathErr() << "Ephemeris and Attitude don't seem to sample with the same t0 or dt." );

  // Convert ephemeris to be position of camera. Change attitude to
  // be the rotation from camera frame to world frame. We also add an
  // additional rotation to the camera frame so X is the horizontal
  // direction to the picture and +Y points down the image (in the direction of flight).
  Quat sensor_coordinate = math::euler_xyz_to_quaternion(Vector3(0,0,geo.detector_rotation - M_PI/2));
  for ( size_t i = 0; i < eph.position_vec.size(); i++ ) {
    eph.position_vec[i] += att.quat_vec[i].rotate( geo.perspective_center );
    att.quat_vec[i] = att.quat_vec[i] * geo.camera_attitude * sensor_coordinate;
  }

  // Load up the time interpolation class. If the TLCList only has
  // one entry ... then we have to manually drop in the slope and offset.
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

  typedef LinescanDGModel<camera::PiecewiseAPositionInterpolation, 
                          camera::LinearPiecewisePositionInterpolation, 
                          camera::SLERPPoseInterpolation, 
                          camera::TLCTimeInterpolation> camera_type;
  typedef boost::shared_ptr<camera::CameraModel> result_type;
  double et0 = convert( parse_time( eph.start_time ) );
  double at0 = convert( parse_time( att.start_time ) );
  double edt = eph.time_interval;
  double adt = att.time_interval;
  return result_type(new camera_type(camera::PiecewiseAPositionInterpolation(eph.position_vec, eph.velocity_vec, et0, edt ),
                                     camera::LinearPiecewisePositionInterpolation(eph.velocity_vec, et0, edt),
                                     camera::SLERPPoseInterpolation(att.quat_vec, at0, adt),
                                     tlc_time_interpolation, img.image_size,
                                     subvector(inverse(sensor_coordinate).rotate(Vector3(geo.detector_origin[0],
                                                                                         geo.detector_origin[1], 
                                                                                         0)
                                                                                ), 
                                               0, 2),
                                     geo.principal_distance, correct_velocity_aberration)
                    );
} // End function load_dg_camera_model()








//==========================================================================




/// Checks the DEM and loads the RPC camera models
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
void StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
       init_disk_transform(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT>) {
  
  // Verify that we can read the camera models
  // - In the future we can support other types of models here.
  m_left_map_proj_model  = boost::shared_ptr<asp::RPCModel>(read_rpc_model(m_left_image_file,  m_left_camera_file ));
  m_right_map_proj_model = boost::shared_ptr<asp::RPCModel>(read_rpc_model(m_right_image_file, m_right_camera_file));

  VW_ASSERT( m_left_map_proj_model.get() && m_right_map_proj_model.get(),
             ArgumentErr() << "StereoSessionConcrete: Unable to locate RPC inside input files." );
  
  // Double check that we can read the DEM and that it has cartographic information.
  VW_ASSERT(!m_input_dem.empty(), InputErr() << "StereoSessionDGMapRPC : Require input DEM" );
  if (!boost::filesystem::exists(m_input_dem))
    vw_throw( ArgumentErr() << "StereoSessionConcrete: DEM \"" << m_input_dem << "\" does not exist." );

}


// TODO: Do we need those weird vw enum things?


//----------------------------------------------------------------------------------------------
// Code for reading different camera models

// ---> Move these helper functions!

//TODO: Move this!
// General function for loading an ISIS camera model
inline boost::shared_ptr<vw::camera::CameraModel>
load_isis_camera_model(std::string const& image_file,
                       std::string const& camera_file) {

  if (boost::ends_with(boost::to_lower_copy(camera_file), ".isis_adjust")){
    // Creating Equations for the files
    std::ifstream input( camera_file.c_str() );
    boost::shared_ptr<BaseEquation> posF  = read_equation(input);
    boost::shared_ptr<BaseEquation> poseF = read_equation(input);
    input.close();

    // Finally creating camera model
    return boost::shared_ptr<vw::camera::CameraModel>(new camera::IsisAdjustCameraModel(image_file, posF, poseF));
  } else {
    return boost::shared_ptr<vw::camera::CameraModel>(new camera::IsisCameraModel(image_file));
  }
} // End function load_isis_camera_model()


// TODO: Move this function somewhere else!
/// Computes a Map2CamTrans given a DEM, image, and a sensor model.
inline cartography::Map2CamTrans
getTransformFromMapProject(const std::string &input_dem_path,
                           const std::string &img_file_path,
                           boost::shared_ptr<vw::camera::CameraModel> map_proj_model_ptr) {

  // Read in data necessary for the Map2CamTrans object
  cartography::GeoReference dem_georef, image_georef;
  if (!read_georeference(dem_georef, input_dem_path))
    vw_throw( ArgumentErr() << "The DEM \"" << input_dem_path << "\" lacks georeferencing information.");
  if (!read_georeference(image_georef, img_file_path))
    vw_throw( ArgumentErr() << "The image \"" << img_file_path << "\" lacks georeferencing information.");

  bool call_from_mapproject = false;
  DiskImageView<float> img(img_file_path);
  return cartography::Map2CamTrans(map_proj_model_ptr.get(),
                                   image_georef, dem_georef, input_dem_path,
                                   Vector2(img.cols(), img.rows()), 
                                   call_from_mapproject);
}





// Redirect to the correct function depending on the template parameters
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel>
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::camera_model(std::string const& image_file,
                                                                         std::string const& camera_file) {
  return load_camera_model(Int2Type<STEREOMODEL_TYPE>(), image_file, camera_file);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel> 
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model(Int2Type<STEREOMODEL_TYPE_ISIS>, 
                                                                              std::string const& image_file, 
                                                                              std::string const& camera_file) {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  return load_isis_camera_model(image_file, camera_file); // Just call the standalone function
#endif
  return boost::shared_ptr<vw::camera::CameraModel>(); // If no ISIS
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel> 
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model(Int2Type<STEREOMODEL_TYPE_DG>, 
                                                                              std::string const& image_file, 
                                                                              std::string const& camera_file) {
  return load_dg_camera_model(camera_file); // Call the standalone function
  //return boost::shared_ptr<vw::camera::CameraModel>(); //DEBUG
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel> 
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model(Int2Type<STEREOMODEL_TYPE_RPC>, 
                                                                              std::string const& image_file, 
                                                                              std::string const& camera_file) {
  return boost::shared_ptr<camera::CameraModel>(read_rpc_model(image_file, camera_file)); // Just call the standalone function
}
// The PINHOLE function unfortunately needs to be written out here.
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel> 
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model(Int2Type<STEREOMODEL_TYPE_PINHOLE>, 
                                                                              std::string const& image_file, 
                                                                              std::string const& camera_file) {
  if ( stereo_settings().alignment_method == "epipolar" ) {
    // Load the image
    DiskImageView<float> left_image (m_left_image_file );
    DiskImageView<float> right_image(m_right_image_file);

    Vector2i left_image_size (left_image.cols(),  left_image.rows() ),
             right_image_size(right_image.cols(), right_image.rows());

    bool is_left_camera = true;
    if (camera_file == m_left_camera_file)
      is_left_camera = true;
    else if (camera_file == m_right_camera_file)
      is_left_camera = false;
    else
      (ArgumentErr() << "StereoSessionPinhole: supplied camera model filename does not match the name supplied in the constructor.");

    // Return the appropriate camera model object
    std::string lcase_file = boost::to_lower_copy(m_left_camera_file);
    CAHVModel left_cahv, right_cahv;
    if (boost::ends_with(lcase_file, ".cahvore") ) {
      CAHVOREModel left_cahvore (m_left_camera_file );
      CAHVOREModel right_cahvore(m_right_camera_file);
      left_cahv  = linearize_camera(left_cahvore,  left_image_size,  left_image_size);
      right_cahv = linearize_camera(right_cahvore, right_image_size, right_image_size);
    } else if (boost::ends_with(lcase_file, ".cahvor")  ||
               boost::ends_with(lcase_file, ".cmod"  )   ) {
      CAHVORModel left_cahvor (m_left_camera_file );
      CAHVORModel right_cahvor(m_right_camera_file);
      left_cahv  = linearize_camera(left_cahvor,  left_image_size,  left_image_size);
      right_cahv = linearize_camera(right_cahvor, right_image_size, right_image_size);
    } else if ( boost::ends_with(lcase_file, ".cahv") ||
                boost::ends_with(lcase_file, ".pin" )) {
      left_cahv  = CAHVModel(m_left_camera_file );
      right_cahv = CAHVModel(m_right_camera_file);

    } else if ( boost::ends_with(lcase_file, ".pinhole") ||
                boost::ends_with(lcase_file, ".tsai"   )   ) {
      PinholeModel left_pin (m_left_camera_file );
      PinholeModel right_pin(m_right_camera_file);
      left_cahv  = linearize_camera(left_pin );
      right_cahv = linearize_camera(right_pin);
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }

    // Create epipolar recitified camera views
    boost::shared_ptr<CAHVModel> epipolar_left_cahv (new CAHVModel);
    boost::shared_ptr<CAHVModel> epipolar_right_cahv(new CAHVModel);
    epipolar(left_cahv, right_cahv, *epipolar_left_cahv, *epipolar_right_cahv);

    if (is_left_camera)
      return epipolar_left_cahv;
    else
      return epipolar_right_cahv;
  } else { // Not epipolar
    // Keypoint alignment and everything else just gets camera models
    std::string lcase_file = boost::to_lower_copy(camera_file);
    if (boost::ends_with(lcase_file,".cahvore") ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVOREModel(camera_file) );
    } else if (boost::ends_with(lcase_file,".cahvor") ||
               boost::ends_with(lcase_file,".cmod"  )   ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVORModel(camera_file) );
    } else if ( boost::ends_with(lcase_file,".cahv") ||
                boost::ends_with(lcase_file,".pin" )   ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new CAHVModel(camera_file) );
    } else if ( boost::ends_with(lcase_file,".pinhole") ||
                boost::ends_with(lcase_file,".tsai"   )   ) {
      return boost::shared_ptr<vw::camera::CameraModel>( new PinholeModel(camera_file) );
    } else {
      vw_throw(ArgumentErr() << "PinholeStereoSession: unsupported camera file type.\n");
    }
  } // End not epipolar case
  return boost::shared_ptr<vw::camera::CameraModel>(); // Never reached

}




//----------------------------------------------------------------------------------------------
// Code for handling disk-to-sensor transform


// Redirect to the appropriate function
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left() const {
  return tx_left(Int2Type<DISKTRANSFORM_TYPE>());
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right() const {
  return tx_right(Int2Type<DISKTRANSFORM_TYPE>());
}
// All the specific transform functions are after here

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MATRIX>) const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-L.exr" );
  }
  return tx_type( tx );
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MATRIX>) const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-R.exr" );
  }
  return tx_type( tx );
}


template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MATRIX_RIGHT>) const {
  Matrix<double> tx = math::identity_matrix<3>();
  return tx_type( tx );
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MATRIX_RIGHT>) const {
  if ( stereo_settings().alignment_method == "homography" ) {
    Matrix<double> align_matrix;
    read_matrix( align_matrix, m_out_prefix + "-align-R.exr" );
    return tx_type( align_matrix );
  }
  return tx_type( math::identity_matrix<3>() );
}



template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT>) const {
  return getTransformFromMapProject(m_input_dem, m_left_image_file, m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT>) const {
  return getTransformFromMapProject(m_input_dem, m_right_image_file, m_right_map_proj_model);
}





  
} // End namespace asp

