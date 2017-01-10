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

// TODO: Serious refactoring needed. Most of this code must go to
// a file named MapProj/StereoSessionMapProj.h. Code specific to
// pinhole models must go to that session.

/// \file StereoSessionConcrete.tcc
///
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math.h>
#include <vw/Camera.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/MaskViews.h>
#include <vw/Stereo/DisparityMap.h>

#include <asp/Core/Common.h>
#include <asp/Core/InterestPointMatching.h>

#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Sessions/StereoSessionPinhole.h> // TODO: temporary

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
  initialize(vw::cartography::GdalWriteOptions const& options,
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
  init_disk_transform(DISKTRANSFORM_TYPE);

}

// For non map projected inputs, keep the same default function as the base class.  Otherwise throw!
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
inline bool StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
ip_matching(std::string  const& input_file1,
            std::string  const& input_file2,
            vw::Vector2  const& uncropped_image_size,
            Vector6f const& stats1,
            Vector6f const& stats2,
            int ip_per_tile,
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
    return StereoSession::ip_matching(input_file1, input_file2,
                                      uncropped_image_size,
                                      stats1,      stats2,
                                      ip_per_tile,
                                      nodata1, nodata2,
                                      match_filename, cam1, cam2);
}



//==========================================================================


/// Checks the DEM and loads the RPC camera models
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
void StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::
       init_disk_transform(STEREOSESSION_DISKTRANSFORM_TYPE disk_transform_type) {

  // Verify that we can read the camera models
  STEREOSESSION_STEREOMODEL_TYPE model_type_to_load;
  switch(disk_transform_type) {
    // These three types need to load a pair of camera models
    case DISKTRANSFORM_TYPE_MAP_PROJECT_RPC:      model_type_to_load = STEREOMODEL_TYPE_RPC;      break;
    case DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS:     model_type_to_load = STEREOMODEL_TYPE_ISIS;     break;
    case DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE:  model_type_to_load = STEREOMODEL_TYPE_PINHOLE;  break;

    default: // The other types don't need to do anything in this function!
      return;
  };

  vw_out() << "Loading the camera models that were used in map-projection\n";

  // Back up the bundle-adjust prefix that should be used only with the
  // original camera model, not with the model used in mapprojection
  // (e.g., the original camera model could have been DG, but in
  // map-projection we could have used RPC).
  std::string ba_pref_bk = stereo_settings().bundle_adjust_prefix;

  // Load the models used in map-projection. If map-projection was
  // done with adjusted models, apply those adjustments.
  std::string l_adj_prefix, r_adj_prefix;
  {
    std::string adj_key = "BUNDLE_ADJUST_PREFIX";
    boost::shared_ptr<vw::DiskImageResource> l_rsrc
      (new vw::DiskImageResourceGDAL(m_left_image_file));
    vw::cartography::read_header_string(*l_rsrc.get(), adj_key, l_adj_prefix);
    boost::shared_ptr<vw::DiskImageResource> r_rsrc
      (new vw::DiskImageResourceGDAL(m_right_image_file));
    vw::cartography::read_header_string(*r_rsrc.get(), adj_key, r_adj_prefix);
  }

  // TODO: Verify that the DEM encoded in the map-projected image above
  // is the same as m_input_dem used below.

  stereo_settings().bundle_adjust_prefix = "";
  if ( (l_adj_prefix != "" && l_adj_prefix != "NONE") )
    stereo_settings().bundle_adjust_prefix = l_adj_prefix;
  m_left_map_proj_model
    = load_camera_model(model_type_to_load,
                        m_left_image_file,  m_left_camera_file );

  stereo_settings().bundle_adjust_prefix = "";
  if (r_adj_prefix != "" && r_adj_prefix != "NONE")
    stereo_settings().bundle_adjust_prefix = r_adj_prefix;
  m_right_map_proj_model
    = load_camera_model(model_type_to_load,
                        m_right_image_file, m_right_camera_file);

  // Go back to the original bundle-adjust prefix now that we have
  // loaded the models used in map-projection.
  stereo_settings().bundle_adjust_prefix = ba_pref_bk;

  VW_ASSERT( m_left_map_proj_model.get() && m_right_map_proj_model.get(),
             ArgumentErr() << "StereoSessionConcrete: Unable to locate map "
             << "projection camera model inside input files!" );

  // Double check that we can read the DEM and that it has cartographic information.
  VW_ASSERT(!m_input_dem.empty(), InputErr() << "StereoSessionConcrete : Require input DEM." );
  if (!boost::filesystem::exists(m_input_dem))
    vw_throw( ArgumentErr() << "StereoSessionConcrete: DEM \"" << m_input_dem
              << "\" does not exist." );

  vw_out() << "Done loading the camera models used in map-projection\n";

}

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
  vw_out() << "Loading camera model: " << image_file << ' ' << camera_file << "\n";

  if (camera_file == "") // No camera file provided, use the image file.
    return load_camera_model(STEREOMODEL_TYPE, image_file, image_file);
  else // Camera file provided
    return load_camera_model(STEREOMODEL_TYPE, image_file, camera_file);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
boost::shared_ptr<vw::camera::CameraModel>
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::load_camera_model
(STEREOSESSION_STEREOMODEL_TYPE model_type, std::string const& image_file, std::string const& camera_file){

  // Retrieve the pixel offset (if any) to cropped images
  vw::Vector2 pixel_offset = asp::camera_pixel_offset(m_input_dem,
                                                 m_left_image_file,
                                                 m_right_image_file,
                                                 image_file);

  //std::cout << "For camera files: " << image_file << "\n" << camera_file << "\n";
  //std::cout << "Loaded offset: " << pixel_offset << "\n";

  std::string err1, err2;
  switch(model_type){
    case STEREOMODEL_TYPE_ISIS:
        return load_adjusted_model(m_camera_loader.load_isis_camera_model(camera_file),
                                   image_file, camera_file, pixel_offset);
  case STEREOMODEL_TYPE_DG:
    return load_adjusted_model(m_camera_loader.load_dg_camera_model(camera_file),
                               image_file, camera_file, pixel_offset);
  case STEREOMODEL_TYPE_SPOT5:
    return load_adjusted_model(m_camera_loader.load_spot5_camera_model(camera_file),
                               image_file, camera_file, pixel_offset);
  case STEREOMODEL_TYPE_ASTER:
    return load_adjusted_model(m_camera_loader.load_ASTER_camera_model(camera_file),
                               image_file, camera_file, pixel_offset);
  case STEREOMODEL_TYPE_RPC:
    try {
      if (camera_file != ""){
        return load_adjusted_model(m_camera_loader.load_rpc_camera_model(camera_file),
                                   image_file, camera_file, pixel_offset);
      }
    }
    catch(std::exception const& e1) {
      err1 = e1.what();
    }
    try {
      return load_adjusted_model(m_camera_loader.load_rpc_camera_model(image_file),
                                 image_file, camera_file, pixel_offset);
    }
    catch(std::exception const& e2) {
      err2 = e2.what();
    }
    // Raise a custom exception if both failed
    vw_throw(ArgumentErr() << "Unable to load RPC model from either:\n" << image_file
             << "\nor:\n" << camera_file << "\n"
             << err1 << "\n" << err2 << "\n");

  default: break; // This must be the pinhole case
  };

  return asp::load_adj_pinhole_model(image_file, camera_file,
                                     m_left_image_file, m_right_image_file,
                                     m_left_camera_file, m_right_camera_file,
                                     m_input_dem);

}

//------------------------------------------------------------------------------
// Code for handling disk-to-sensor transform


// Return the left and right map-projected images. These are the same
// as the input images unless it is desired to use cropped images.
inline std::string left_mapproj(std::string const& left_image,
                                std::string const& out_prefix){
  if ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)){
    return out_prefix + "-L-cropped.tif";
  }
  return left_image;
}
inline std::string right_mapproj(std::string const& right_image,
                                      std::string const& out_prefix){
  if ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) ){
    return out_prefix + "-R-cropped.tif";
  }
  return right_image;
}


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
  return getTransformFromMapProject(m_input_dem,
                                    left_mapproj(m_left_image_file, m_out_prefix),
                                    m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC>) const {
  return getTransformFromMapProject(m_input_dem,
                                    right_mapproj(m_right_image_file, m_out_prefix),
                                    m_right_map_proj_model);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS>) const {
  return getTransformFromMapProject(m_input_dem,
                                    left_mapproj(m_left_image_file, m_out_prefix),
                                    m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS>) const {
  return getTransformFromMapProject(m_input_dem,
                                    right_mapproj(m_right_image_file, m_out_prefix),
                                    m_right_map_proj_model);
}


template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE>) const {
  return getTransformFromMapProject(m_input_dem,
                                    left_mapproj(m_left_image_file, m_out_prefix),
                                    m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE>) const {
  return getTransformFromMapProject(m_input_dem,
                                    right_mapproj(m_right_image_file, m_out_prefix),
                                    m_right_map_proj_model);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_SPOT5>) const {
  return getTransformFromMapProject(m_input_dem,
                                    left_mapproj(m_left_image_file, m_out_prefix),
                                    m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_SPOT5>) const {
  return getTransformFromMapProject(m_input_dem,
                                    right_mapproj(m_right_image_file, m_out_prefix),
                                    m_right_map_proj_model);
}

template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_left(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ASTER>) const {
  return getTransformFromMapProject(m_input_dem,
                                    left_mapproj(m_left_image_file, m_out_prefix),
                                    m_left_map_proj_model);
}
template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE,
          STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>
typename StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_type
StereoSessionConcrete<DISKTRANSFORM_TYPE,STEREOMODEL_TYPE>::tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ASTER>) const {
  return getTransformFromMapProject(m_input_dem,
                                    right_mapproj(m_right_image_file, m_out_prefix),
                                    m_right_map_proj_model);
}

} // End namespace asp
