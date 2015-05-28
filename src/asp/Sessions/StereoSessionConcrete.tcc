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
#include <asp/Core/Common.h>

//#include <asp/Sessions/RPC/RPCModel.h>

//#include <xercesc/util/PlatformUtils.hpp>

#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/IsisIO/Equation.h>


#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>




using namespace vw;

using namespace vw::camera; // For the Pinhole load function

namespace asp {


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



//==========================================================================


/// Checks the DEM and loads the RPC camera models
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
void StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
       init_disk_transform(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC>) {
  
  // Verify that we can read the camera models
  m_left_map_proj_model  = load_camera_model(Int2Type<STEREOMODEL_TYPE_RPC>(), m_left_image_file,  m_left_camera_file );
  m_right_map_proj_model = load_camera_model(Int2Type<STEREOMODEL_TYPE_RPC>(), m_right_image_file, m_right_camera_file);

  VW_ASSERT( m_left_map_proj_model.get() && m_right_map_proj_model.get(),
             ArgumentErr() << "StereoSessionConcrete: Unable to locate RPC inside input files." );
  
  // Double check that we can read the DEM and that it has cartographic information.
  VW_ASSERT(!m_input_dem.empty(), InputErr() << "StereoSessionConcrete : Require input DEM" );
  if (!boost::filesystem::exists(m_input_dem))
    vw_throw( ArgumentErr() << "StereoSessionConcrete: DEM \"" << m_input_dem << "\" does not exist." );

}

/// Checks the DEM and loads the ISIS camera models
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
void StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
       init_disk_transform(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS>) {

  // Verify that we can read the camera models
  m_left_map_proj_model  = load_camera_model(Int2Type<STEREOMODEL_TYPE_ISIS>(), m_left_image_file,  m_left_camera_file );
  m_right_map_proj_model = load_camera_model(Int2Type<STEREOMODEL_TYPE_ISIS>(), m_right_image_file, m_right_camera_file);

  VW_ASSERT( m_left_map_proj_model.get() && m_right_map_proj_model.get(),
             ArgumentErr() << "StereoSessionConcrete: Unable to locate ISIS model inside input files." );
  
  // Double check that we can read the DEM and that it has cartographic information.
  VW_ASSERT(!m_input_dem.empty(), InputErr() << "StereoSessionConcrete : Require input DEM" );
  if (!boost::filesystem::exists(m_input_dem))
    vw_throw( ArgumentErr() << "StereoSessionConcrete: DEM \"" << m_input_dem << "\" does not exist." );

}


/// Checks the DEM and loads the pinhole camera models
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
void StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
       init_disk_transform(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE>) {

  // Verify that we can read the camera models
  m_left_map_proj_model  = m_camera_loader.load_pinhole_camera_model(m_left_camera_file );
  m_right_map_proj_model = m_camera_loader.load_pinhole_camera_model(m_right_camera_file);

  VW_ASSERT( m_left_map_proj_model.get() && m_right_map_proj_model.get(),
             ArgumentErr() << "StereoSessionConcrete: Unable to locate Pinhole model inside input files." );
  
  // Double check that we can read the DEM and that it has cartographic information.
  VW_ASSERT(!m_input_dem.empty(), InputErr() << "StereoSessionConcrete : Require input DEM" );
  if (!boost::filesystem::exists(m_input_dem))
    vw_throw( ArgumentErr() << "StereoSessionConcrete: DEM \"" << m_input_dem << "\" does not exist." );

}


// TODO: Do we need those weird vw enum things?


//----------------------------------------------------------------------------------------------
// Code for reading different camera models


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
  if (camera_file == "") // No camera file provided, use the image file.
    return load_camera_model(Int2Type<STEREOMODEL_TYPE>(), image_file, image_file);
  else // Camera file provided
    return load_camera_model(Int2Type<STEREOMODEL_TYPE>(), image_file, camera_file);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel> 
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model(Int2Type<STEREOMODEL_TYPE_ISIS>, 
                                                                              std::string const& image_file, 
                                                                              std::string const& camera_file) {
  // We have to do a check here to make sure we pass the files in the correct order
  if (boost::ends_with(boost::to_lower_copy(camera_file), ".isis_adjust")) {
    return m_camera_loader.load_isis_camera_model(image_file, camera_file); 
  }
  else {
    return m_camera_loader.load_isis_camera_model(camera_file);
  }
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel> 
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model(Int2Type<STEREOMODEL_TYPE_DG>, 
                                                                              std::string const& image_file, 
                                                                              std::string const& camera_file) {
  return m_camera_loader.load_dg_camera_model(camera_file);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel> 
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model(Int2Type<STEREOMODEL_TYPE_RPC>, 
                                                                              std::string const& image_file, 
                                                                              std::string const& camera_file) {
  // We don't know where the camera model is so try both files
  try {
    return m_camera_loader.load_rpc_camera_model(image_file);
  }
  catch(...) {}
  try {
    return m_camera_loader.load_rpc_camera_model(camera_file);
  }
  catch(...) {}
  // Raise a custom exception if both failed
  vw_throw(ArgumentErr() << "Unable to load RPC model from either:.\n" << image_file
                         << " or:\n" << camera_file);
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

    // TODO: Why is this needed?  We want to make this a static function!
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
  } else { // Not epipolar, just load the camera model.
    return m_camera_loader.load_pinhole_camera_model(camera_file);
  } // End not epipolar case

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


//TODO: Consolidate all these map projected functions which are identical
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC>) const {
  return getTransformFromMapProject(m_input_dem, m_left_image_file, m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC>) const {
  return getTransformFromMapProject(m_input_dem, m_right_image_file, m_right_map_proj_model);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS>) const {
  return getTransformFromMapProject(m_input_dem, m_left_image_file, m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS>) const {
  return getTransformFromMapProject(m_input_dem, m_right_image_file, m_right_map_proj_model);
}


template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE>) const {
  return getTransformFromMapProject(m_input_dem, m_left_image_file, m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE>) const {
  return getTransformFromMapProject(m_input_dem, m_right_image_file, m_right_map_proj_model);
}  


} // End namespace asp

