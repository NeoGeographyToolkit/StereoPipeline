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

#include <vw/Core.h>
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
using namespace vw;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/DG/XML.h>
#include <asp/Sessions/RPC/RPCMapTransform.h>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include "ogr_spatialref.h"

struct Options : asp::BaseOptions {
  // Input
  std::string dem_file, image_file, camera_model_file, output_file;

  // Settings
  std::string target_srs_string;
  float target_resolution;
  BBox2 target_projwin;
  double nodata_value;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("nodata-value", po::value(&opt.nodata_value), "Nodata value to use on output.")
    ("t_srs", po::value(&opt.target_srs_string), "Target spatial reference set. This mimicks the  gdal option.")
    ("tr", po::value(&opt.target_resolution)->default_value(0), "Set output file resolution (in target georeferenced units per pixel)")
    ("t_projwin", po::value(&opt.target_projwin),
     "Selects a subwindow from the source image for copying but with the corners given in georeferenced coordinates (xmin ymin xmax ymax). Max is exclusive.");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("dem", po::value(&opt.dem_file))
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_model_file))
    ("output-file", po::value(&opt.output_file));

  po::positional_options_description positional_desc;
  positional_desc.add("dem", 1);
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-file",1);

  std::string usage("[options] <dem> <camera-image> <camera-model> <output>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage );

  if ( !vm.count("dem") || !vm.count("camera-image") ||
       !vm.count("camera-model") || !vm.count("t_srs") )
    vw_throw( ArgumentErr() << "Requires <dem> <camera-image> <camera-model> and <t_srs> input in order to proceed.\n\n"
              << usage << general_options );

  if ( opt.output_file.empty() )
    opt.output_file = fs::basename( opt.image_file ) + "_rpcmapped.tif";
}

int main( int argc, char* argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Load DEM
    boost::shared_ptr<DiskImageResource>
      dem_rsrc( DiskImageResource::open( opt.dem_file ) );
    cartography::GeoReference dem_georef;
    read_georeference( dem_georef, opt.dem_file );

    // Read projection. Work out output bounding box in points using
    // original camera model.
    cartography::GeoReference target_georef;
    boost::replace_first(opt.target_srs_string,
                         "IAU2000:","DICT:IAU2000.wkt,");
    VW_OUT(DebugMessage,"asp") << "Asking GDAL to decipher: \""
                               << opt.target_srs_string << "\"\n";
    OGRSpatialReference gdal_spatial_ref;
    if (gdal_spatial_ref.SetFromUserInput( opt.target_srs_string.c_str() ))
      vw_throw( ArgumentErr() << "Failed to parse: \"" << opt.target_srs_string << "\"." );
    char *wkt = NULL;
    gdal_spatial_ref.exportToWkt( &wkt );
    std::string wkt_string(wkt);
    delete[] wkt;

    target_georef.set_wkt( wkt_string );

    // Work out target resolution and image size
    asp::StereoSessionDG session;
    boost::shared_ptr<camera::CameraModel>
      camera_model( session.camera_model( opt.image_file, opt.camera_model_file ) );
    Vector2i image_size = asp::file_image_size( opt.image_file );
    BBox2 point_bounds =
      opt.target_resolution <= 0 ?
      camera_bbox( target_georef, camera_model,
                   image_size.x(), image_size.y(),
                   opt.target_resolution ) :
      camera_bbox( target_georef, camera_model,
                   image_size.x(), image_size.y() );
    if ( opt.target_projwin != BBox2() ) {
      point_bounds = opt.target_projwin;
      if ( point_bounds.min().y() > point_bounds.max().y() )
        std::swap( point_bounds.min().y(),
                   point_bounds.max().y() );
      vw_out() << "Cropping to " << point_bounds << " pt. " << std::endl;
      point_bounds.max().x() -= opt.target_resolution;
      point_bounds.min().y() += opt.target_resolution;
    }

    {
      Matrix3x3 transform = math::identity_matrix<3>();
      transform(0,0) = opt.target_resolution;
      transform(1,1) = -opt.target_resolution;
      transform(0,2) = point_bounds.min().x();
      transform(1,2) = point_bounds.max().y();
      if ( target_georef.pixel_interpretation() ==
           cartography::GeoReference::PixelAsArea ) {
        transform(0,2) -= 0.5 * opt.target_resolution;
        transform(1,2) += 0.5 * opt.target_resolution;
      }
      target_georef.set_transform( transform );
    }
    vw_out() << "Output Georeference:\n" << target_georef << std::endl;

    BBox2i target_image_size =
      target_georef.point_to_pixel_bbox( point_bounds );
    vw_out() << "Creating output file that is " << target_image_size.size() << " px.\n";

    // Load RPC
    asp::RPCXML xml;
    xml.read_from_file( opt.camera_model_file );

    boost::shared_ptr<DiskImageResource>
      src_rsrc( DiskImageResource::open( opt.image_file ) );

    // Raster output image
    if ( src_rsrc->has_nodata_read() ) {
      asp::block_write_gdal_image( opt.output_file,
                                   apply_mask(transform( create_mask( DiskImageView<float>( src_rsrc), src_rsrc->nodata_read() ),
                                                         asp::RPCMapTransform( *xml.rpc_ptr(),
                                                                               target_georef,
                                                                               dem_georef,
                                                                               dem_rsrc ),
                                                         target_image_size.width(),
                                                         target_image_size.height(),
                                                         ValueEdgeExtension<PixelMask<float> >( PixelMask<float>() ),
                                                         BicubicInterpolation() ),
                                              src_rsrc->nodata_read() ),
                                   target_georef, opt,
                                   TerminalProgressCallback("","") );
    } else {
      asp::block_write_gdal_image(opt.output_file,
                                  transform( DiskImageView<float>( src_rsrc ),
                                             asp::RPCMapTransform( *xml.rpc_ptr(),
                                                                   target_georef,
                                                                   dem_georef,
                                                                   dem_rsrc ),
                                             target_image_size.width(),
                                             target_image_size.height(),
                                             ZeroEdgeExtension(),
                                             BicubicInterpolation() ),
                                  target_georef, opt,
                                  TerminalProgressCallback("","") );
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
