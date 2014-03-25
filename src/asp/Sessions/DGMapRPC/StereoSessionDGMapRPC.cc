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


/// \file StereoSessionDGMapRPC.cc
///
#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Sessions/RPC/RPCModel.h>
#include <asp/Sessions/RPC/StereoSessionRPC.h>
#include <asp/Sessions/DGMapRPC/StereoSessionDGMapRPC.h>
#include <asp/Sessions/DG/XML.h>

using namespace vw;
using namespace asp;
namespace fs = boost::filesystem;

// Initializer to determine what kind of input we have.
void StereoSessionDGMapRPC::initialize(BaseOptions const& options,
                                       std::string const& left_image_file,
                                       std::string const& right_image_file,
                                       std::string const& left_camera_file,
                                       std::string const& right_camera_file,
                                       std::string const& out_prefix,
                                       std::string const& input_dem,
                                       std::string const& extra_argument1,
                                       std::string const& extra_argument2,
                                       std::string const& extra_argument3 ) {
  StereoSessionDG::initialize( options, left_image_file,
                               right_image_file, left_camera_file,
                               right_camera_file, out_prefix,
                               input_dem, extra_argument1,
                               extra_argument2, extra_argument3 );

  // Verify that we can read the camera models
  m_left_model = boost::shared_ptr<RPCModel>(StereoSessionRPC::read_rpc_model(left_image_file, left_camera_file));
  m_right_model = boost::shared_ptr<RPCModel> (StereoSessionRPC::read_rpc_model(right_image_file, right_camera_file));
  VW_ASSERT( m_left_model.get() && m_right_model.get(),
             ArgumentErr() << "StereoSessionDGMapRPC: Unable to locate RPC inside input files." );
  
  // Double check that we can read the DEM and that it has
  // cartographic information.
  VW_ASSERT( !input_dem.empty(),
             InputErr() << "StereoSessionDGMapRPC : Require input DEM" );
  if ( !fs::exists( input_dem ) )
    vw_throw( ArgumentErr() << "StereoSessionDGMapRPC: DEM \"" << input_dem
              << "\" does not exist." );

  // Verify that center of our lonlat boundaries from the RPC models
  // actually projects into the DEM. (?)
}

bool StereoSessionDGMapRPC::ip_matching( std::string const& match_filename,
                                         double left_nodata_value,
                                         double right_nodata_value ) {

  // This code will never be reached, since for map-projected images we never
  // perform homography or affineepipolar alignment.

  // In fact, it is very hard to write this function properly, since
  // the Map2CamTrans class which needs to be used here is not thread
  // safe and cannot operate on randomly accessed pixels, it must be
  // invoked only wholesale on tiles, with each tile getting a copy of
  // that transform.
  vw_throw( ArgumentErr() << "StereoSessionDGMapRPC: IP matching is not implemented as no alignment is applied to map-projected images.");
}

StereoSessionDGMapRPC::left_tx_type
StereoSessionDGMapRPC::tx_left() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-L.exr" );
  }

  cartography::GeoReference dem_georef, image_georef;
  if (!read_georeference( dem_georef, m_input_dem ) )
    vw_throw( ArgumentErr() << "The DEM \"" << m_input_dem
              << "\" lacks georeferencing information.");
  if (!read_georeference( image_georef, m_left_image_file ) )
    vw_throw( ArgumentErr() << "The image \"" << m_left_image_file
              << "\" lacks georeferencing information.");

  bool call_from_mapproject = false;
  DiskImageView<float> img(m_left_image_file);
  
  // Load DEM rsrc. Map2CamTrans is casting to float internally
  // no matter what the original type of the DEM file was.
  boost::shared_ptr<DiskImageResource>
    dem_rsrc( DiskImageResource::open( m_input_dem ) );

  // This composes the two transforms as it is possible to do
  // homography and affineepipolar alignment options with map
  // projected imagery.
  return left_tx_type( cartography::Map2CamTrans
                       (m_left_model.get(),
                        image_georef, dem_georef, dem_rsrc,
                        Vector2(img.cols(), img.rows()), call_from_mapproject),
                       HomographyTransform(tx) );
}

StereoSessionDGMapRPC::right_tx_type
StereoSessionDGMapRPC::tx_right() const {
  Matrix<double> tx = math::identity_matrix<3>();
  if ( stereo_settings().alignment_method == "homography" ||
       stereo_settings().alignment_method == "affineepipolar" ) {
    read_matrix( tx, m_out_prefix + "-align-R.exr" );
  }

  cartography::GeoReference dem_georef, image_georef;
  if (!read_georeference( dem_georef, m_input_dem ) )
    vw_throw( ArgumentErr() << "The DEM \"" << m_input_dem
              << "\" lacks georeferencing information.");
  if (!read_georeference( image_georef, m_right_image_file ) )
    vw_throw( ArgumentErr() << "The image \"" << m_right_image_file
              << "\" lacks georeferencing information.");

  bool call_from_mapproject = false;
  DiskImageView<float> img(m_right_image_file);
  
  // Load DEM rsrc. Map2CamTrans is casting to float internally
  // no matter what the original type of the DEM file was.
  boost::shared_ptr<DiskImageResource>
    dem_rsrc( DiskImageResource::open( m_input_dem ) );

  // This composes the two transforms as it is possible to do
  // homography and affineepipolar alignment options with map
  // projected imagery.
  return right_tx_type( cartography::Map2CamTrans
                        (m_right_model.get(),
                         image_georef, dem_georef, dem_rsrc,
                         Vector2(img.cols(), img.rows()), call_from_mapproject),
                        HomographyTransform(tx) );
}
