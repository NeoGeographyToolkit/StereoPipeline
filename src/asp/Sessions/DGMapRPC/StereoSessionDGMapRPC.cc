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

  VW_ASSERT( !input_dem.empty(),
             InputErr() << "StereoSessionDGMapRPC : Require input DEM" );

  boost::scoped_ptr<RPCModel> model1, model2;
  // Try and pull RPC Camera Models from left and right images.
  try {
    model1.reset( new RPCModel(left_image_file) );
    model2.reset( new RPCModel(right_image_file) );
  } catch ( NotFoundErr const& err ) {}

  // If the above failed to load the RPC Model, try from the XML.
  if ( !model1.get() || !model2.get() ) {
    // Do not catch as these are required.
    RPCXML rpc_xml;
    rpc_xml.read_from_file( left_camera_file );
    model1.reset( new RPCModel( *rpc_xml.rpc_ptr() ) ); // Copy the ptr
    rpc_xml.read_from_file( right_camera_file );
    model2.reset( new RPCModel( *rpc_xml.rpc_ptr() ) );
  }
  VW_ASSERT( model1.get() && model2.get(),
             ArgumentErr() << "StereoSessionDGMapRPC: Unable to locate RPC inside input files." );

  // Double check that we can read the DEM and that it has
  // cartographic information.
  if ( !fs::exists( input_dem ) )
    vw_throw( ArgumentErr() << "StereoSessionDGMapRPC: DEM \"" << input_dem
              << "\" does not exist." );

  // Verify that center of our lonlat boundaries from the RPC models
  // actually projects into the DEM. (?)
}

bool StereoSessionDGMapRPC::ip_matching( std::string const& match_filename,
                                         double left_nodata_value,
                                         double right_nodata_value ) {
  // Load the unmodified images
  DiskImageView<float> left_disk_image( m_left_image_file ),
    right_disk_image( m_right_image_file );

  boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
  camera_models( left_cam, right_cam );

  boost::scoped_ptr<RPCModel>
    left_rpc( StereoSessionRPC::read_rpc_model( m_left_image_file, m_left_camera_file ) ),
    right_rpc( StereoSessionRPC::read_rpc_model( m_right_image_file, m_right_camera_file ) );
  cartography::GeoReference left_georef, right_georef, dem_georef;
  read_georeference( left_georef, m_left_image_file );
  read_georeference( right_georef, m_right_image_file );
  read_georeference( dem_georef, m_input_dem );
  boost::shared_ptr<DiskImageResource>
    dem_rsrc( DiskImageResource::open( m_input_dem ) );
  TransformRef
    left_tx( cartography::MapTransform( left_rpc.get(), left_georef,
                                        dem_georef, dem_rsrc ) ),
    right_tx( cartography::MapTransform( right_rpc.get(), right_georef,
                                         dem_georef, dem_rsrc ) );
  return
    asp::ip_matching( left_cam.get(), right_cam.get(),
                      left_disk_image, right_disk_image,
                      cartography::Datum("WGS84"), match_filename,
                      left_nodata_value,
                      right_nodata_value,
                      left_tx, right_tx, false );
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
    vw_throw( ArgumentErr() << "The DEM \"" << m_input_dem << "\" lacks georeferencing information.");
  if (!read_georeference( image_georef, m_left_image_file ) )
    vw_throw( ArgumentErr() << "The image \"" << m_left_image_file << "\" lacks georeferencing information.");

  // Load DEM rsrc. MapTransform is casting to float internally
  // no matter what the original type of the DEM file was.
  boost::shared_ptr<DiskImageResource>
    dem_rsrc( DiskImageResource::open( m_input_dem ) );

  // This composes the two transforms as it is possible to do
  // homography and affineepipolar alignment options with map
  // projected imagery.
  return left_tx_type( cartography::MapTransform
                       (StereoSessionRPC::read_rpc_model(m_left_image_file, m_left_camera_file),
                        image_georef, dem_georef, dem_rsrc),
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
    vw_throw( ArgumentErr() << "The DEM \"" << m_input_dem << "\" lacks georeferencing information.");
  if (!read_georeference( image_georef, m_right_image_file ) )
    vw_throw( ArgumentErr() << "The image \"" << m_right_image_file << "\" lacks georeferencing information.");

  // Load DEM rsrc. MapTransform is casting to float internally
  // no matter what the original type of the DEM file was.
  boost::shared_ptr<DiskImageResource>
    dem_rsrc( DiskImageResource::open( m_input_dem ) );

  // This composes the two transforms as it is possible to do
  // homography and affineepipolar alignment options with map
  // projected imagery.
  return right_tx_type( cartography::MapTransform
                        (StereoSessionRPC::read_rpc_model(m_right_image_file, m_right_camera_file),
                         image_georef, dem_georef, dem_rsrc),
                        HomographyTransform(tx) );
}
