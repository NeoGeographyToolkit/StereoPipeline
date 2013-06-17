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

/// \file rpc_mapproject.cc
///
/// This program will project a camera image onto a DEM using the RPC
/// camera model.

#include <vw/Core.h>
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
using namespace vw;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/DG/XML.h>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include "ogr_spatialref.h"

struct Options : asp::BaseOptions {
  // Input
  std::string dem_file, image_file, camera_model_file, output_file, stereo_session;

  // Settings
  std::string target_srs_string;
  float target_resolution;
  BBox2 target_projwin;
  double nodata_value;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  // To do: Use mpp and tr for backward comp. Put this in doc.
  // to do: Tell folks about mandatory -t flag.
  // To do: Tell that t_srs is now optional.
  // To do: Tell about the output nodata value.
  // To do: Test the new rpc by doing stereo with old and new rpc_mapproject.
  // To do: Tell about the fix with small DEM.
  general_options.add_options()
    // To do: The nodata-value is not respected.
    // To do: use nan below instead of 0?
    ("nodata-value", po::value(&opt.nodata_value)->default_value(0),
     "Nodata value to use on output.")
    ("t_srs", po::value(&opt.target_srs_string)->default_value(""),
     "Target spatial reference set. This mimicks the gdal option. If not provided use the one from the DEM.") // To do: Put note on the doc that this is now optional.
    ("tr", po::value(&opt.target_resolution)->default_value(0),
     "Set output file resolution (in target georeferenced units per pixel)")
    ("session-type,t", po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. [options: pinhole isis dg rpc]")
    ("t_projwin", po::value(&opt.target_projwin),
     "Selects a subwindow from the source image for copying, with the corners given in georeferenced coordinates (xmin ymin xmax ymax). Max is exclusive.");

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
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( !vm.count("dem") || !vm.count("camera-image") ||
       !vm.count("camera-model") )
    vw_throw( ArgumentErr() << "Requires <dem> <camera-image> and <camera-model> "
              << "input in order to proceed.\n\n"
              << usage << general_options );

  if ( opt.output_file.empty() )
    vw_throw( ArgumentErr() << "Missing output file.\n\n"
              << usage << general_options );

  // When doing stereo, we usually guess the session type to be dg if
  // the camera model is an xml file, yet this tool will most likely
  // be used with rpc sessions, hence the user must be explicit about
  // which session is desired.
  if ( boost::iends_with(boost::to_lower_copy(opt.camera_model_file), ".xml") &&
       opt.stereo_session == "" ){
    vw_throw( ArgumentErr() << "Unable to guess session type. Please specify "
              << "whether it is dg or rpc via the -t option.\n\n"
              << usage << general_options );
  }

}

template <class ImageT>
void write_parallel_cond( std::string const& filename,
                          ImageViewBase<ImageT> const& image,
                          cartography::GeoReference const& georef,
                          bool use_nodata, double nodata_val,
                          Options const& opt,
                          TerminalProgressCallback const& tpc ) {
  // ISIS is not thread safe so we must switch out base on what the
  // session is.
  vw_out() << "Writing: " << filename << "\n";
  if (use_nodata){
    if ( opt.stereo_session == "isis" ) {
      asp::write_gdal_georeferenced_image(filename, image.impl(), georef, nodata_val, opt, tpc);
    } else {
      asp::block_write_gdal_image(filename, image.impl(), georef, nodata_val, opt, tpc);
    }
  }else{
    if ( opt.stereo_session == "isis" ) {
      asp::write_gdal_georeferenced_image(filename, image.impl(), georef, opt, tpc);
    } else {
      asp::block_write_gdal_image(filename, image.impl(), georef, opt, tpc);
    }
  }

}

int main( int argc, char* argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // We create a stereo session where both of the cameras and images
    // are the same, because we want to take advantage of the stereo
    // pipeline's ability to generate camera models for various
    // missions.  Hence, we create two identical camera models, but
    // only one is used.
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSession::create(opt.stereo_session, // in-out
                                                   opt,
                                                   opt.image_file, opt.image_file,
                                                   opt.camera_model_file,
                                                   opt.camera_model_file,
                                                   opt.output_file) );
    boost::shared_ptr<camera::CameraModel> camera_model =
      session->camera_model(opt.image_file, opt.camera_model_file);

    // Safety check that the users are not trying to map project map
    // projected images.
    {
      cartography::GeoReference dummy_georef;
      VW_ASSERT( !read_georeference( dummy_georef, opt.image_file ),
                 ArgumentErr() << "Your input camera image is already map "
                 << "projected. The expected input is required to be "
                 << "unprojected or raw camera imagery." );
    }

    // Load DEM
    boost::shared_ptr<DiskImageResource>
      dem_rsrc( DiskImageResource::open( opt.dem_file ) );
    cartography::GeoReference dem_georef;
    read_georeference( dem_georef, opt.dem_file );

    // Read projection. Work out output bounding box in points using
    // original camera model.
    cartography::GeoReference target_georef = dem_georef;
    if (opt.target_srs_string != ""){
      boost::replace_first(opt.target_srs_string,
                           "IAU2000:","DICT:IAU2000.wkt,");
      VW_OUT(DebugMessage,"asp") << "Asking GDAL to decipher: \""
                                 << opt.target_srs_string << "\"\n";
      OGRSpatialReference gdal_spatial_ref;
      if (gdal_spatial_ref.SetFromUserInput( opt.target_srs_string.c_str() ))
        vw_throw( ArgumentErr() << "Failed to parse: \"" << opt.target_srs_string
                  << "\"." );
      char *wkt = NULL;
      gdal_spatial_ref.exportToWkt( &wkt );
      std::string wkt_string(wkt);
      delete[] wkt;
      target_georef.set_wkt( wkt_string );
    }

    // Find the camera bbox target resolution unless user-supplied.
    float auto_res;
    Vector2i image_size = asp::file_image_size( opt.image_file );
    BBox2 point_bounds =
      camera_bbox( target_georef, camera_model,
                   image_size.x(), image_size.y(), auto_res);
    if (opt.target_resolution <= 0) opt.target_resolution = auto_res;

    if ( opt.target_projwin != BBox2() ) {
      point_bounds = opt.target_projwin;
      if ( point_bounds.min().y() > point_bounds.max().y() )
        std::swap( point_bounds.min().y(),
                   point_bounds.max().y() );
      point_bounds.max().x() -= opt.target_resolution;
      point_bounds.min().y() += opt.target_resolution;
    }

    // In principle the corners of the projection box can be
    // arbitrary.  However, we will force them to be at integer
    // multiples of pixel dimensions. This is needed if we want to do
    // tiling, that is break the DEM into tiles, project on individual
    // tiles, and then combine the tiles nicely without seams into a
    // single projected image. The tiling solution provides a nice
    // speedup when dealing with ISIS images, when projection runs
    // only with one thread.
    double s = opt.target_resolution;
    int min_x         = (int)round(point_bounds.min().x() / s);
    int min_y         = (int)round(point_bounds.min().y() / s);
    int output_width  = (int)round(point_bounds.width()   / s);
    int output_height = (int)round(point_bounds.height()  / s);
    point_bounds = s * BBox2(min_x, min_y, output_width, output_height);

    vw_out() << "Cropping to " << point_bounds << " pt. " << std::endl;

    Matrix3x3 T = target_georef.transform();
    // This polarity checking is to make sure the output has been
    // transposed after going through reprojection. Normally this is
    // the case. Yet with grid data from GMT, it is not.
    if ( T(0,0) < 0 )
      T(0,2) = point_bounds.max().x();
    else
      T(0,2) = point_bounds.min().x();
    T(0,0) = opt.target_resolution;
    T(1,1) = -opt.target_resolution;
    T(1,2) = point_bounds.max().y();
    if ( target_georef.pixel_interpretation() ==
         cartography::GeoReference::PixelAsArea ) {
      T(0,2) -= 0.5 * opt.target_resolution;
      T(1,2) += 0.5 * opt.target_resolution;
    }
    target_georef.set_transform( T );
    vw_out() << "Output georeference:\n" << target_georef << std::endl;

    BBox2i target_image_size =
      target_georef.point_to_pixel_bbox( point_bounds );
    vw_out() << "Creating output file that is " << target_image_size.size()
             << " px.\n";

    // Check if the four corners of the projected image are at valid
    // DEM points.
    double nodata = std::numeric_limits<double>::quiet_NaN();
    if (dem_rsrc->has_nodata_read()) nodata = dem_rsrc->nodata_read();
    DiskImageView<float> dem(dem_rsrc);
    int x[] = {0, target_image_size.width()-1, target_image_size.width()-1, 0};
    int y[] = {0, 0, target_image_size.height()-1, target_image_size.height()-1};
    for (int i = 0; i < 4; i++){
      Vector2i img_pix(x[i], y[i]);
      Vector2 lon_lat = target_georef.pixel_to_lonlat(img_pix);
      Vector2i dem_pix = round(dem_georef.lonlat_to_pixel(lon_lat));
      int c = dem_pix[0], r = dem_pix[1];
      bool isBad = (c < 0 || c >= dem.cols() || r < 0 || r >= dem.rows() ||
                    dem(c, r) == nodata);
      if (isBad){
        vw_out(WarningMessage)
          << "It appears that the map-projected image is not contained "
          << "entirely within the DEM. This may result in large memory usage."
          << std::endl;
        break;
      }
    }

    boost::shared_ptr<DiskImageResource>
      src_rsrc( DiskImageResource::open( opt.image_file ) );

    // Write output image
    asp::create_out_dir(opt.output_file);
    bool use_nodata = src_rsrc->has_nodata_read();
    double nodata_val = std::numeric_limits<double>::quiet_NaN();
    if ( use_nodata) {
      nodata_val = src_rsrc->nodata_read();
      write_parallel_cond
        (opt.output_file,
         apply_mask
         (transform
          (create_mask(DiskImageView<float>( src_rsrc), nodata_val),
           cartography::MapTransform( camera_model.get(), target_georef,
                                      dem_georef, dem_rsrc ),
           target_image_size.width(), target_image_size.height(),
           ValueEdgeExtension<PixelMask<float> >( PixelMask<float>() ),
           BicubicInterpolation() ), nodata_val ),
         target_georef, use_nodata, nodata_val, opt, TerminalProgressCallback("","") );
    } else {
      write_parallel_cond
        (opt.output_file,
         transform(DiskImageView<float>( src_rsrc ),
                   cartography::MapTransform( camera_model.get(), target_georef,
                                              dem_georef, dem_rsrc ),
                   target_image_size.width(), target_image_size.height(),
                   ZeroEdgeExtension(), BicubicInterpolation() ),
         target_georef, use_nodata, nodata_val, opt, TerminalProgressCallback("","") );
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
