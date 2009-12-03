// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file othoproject.cc
///
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>
#include <vw/Cartography/OrthoImageView.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

#include <asp/Sessions.h>

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

template <class ViewT, class DemViewT>
GeoReference compute_geotransform_from_camera(ImageViewBase<ViewT> const& view,
                                              boost::shared_ptr<CameraModel> camera,
                                              ImageViewBase<DemViewT> const& dem,
                                              GeoReference const& dem_georef,
                                              double scale,
                                              double &output_width, double& output_height) {

  // Adopt the datum & projection of the parent DEM.
  GeoReference geo = dem_georef;

  // Compute the longitude/latitude bounding box and the lonlat pixel scale
  float lonlat_scale;
  BBox2 image_bbox = camera_bbox(dem_georef, camera, view.impl().cols(), view.impl().rows(), lonlat_scale);
  BBox2 dem_bbox = dem_georef.lonlat_bounding_box(dem);

  // Use a bbox where both the image and the DEM have valid data.
  // Throw an error if there turns out to be no overlap.
  BBox2 bbox = image_bbox;
  bbox.crop(dem_bbox);

  if (bbox.width() == 0 || bbox.height() == 0) {
    vw_out(0) << "Image bounding box (" << image_bbox << ") and DEM bounding box (" << dem_bbox << ") have no overlap.  Are you sure that your input files overlap?\n";
    return 0;
  }

  float diff1 = bbox.max().x()-bbox.min().x();

  // Convert the lon/lat bounding box into the map projection.
  bbox.min() = geo.lonlat_to_point(bbox.min());
  bbox.max() = geo.lonlat_to_point(bbox.max());
  float diff2 = bbox.max().x()-bbox.min().x();

  // Convert the scale into the projected space
  double projected_space_scale = lonlat_scale * diff2 / diff1;

  // If the user has supplied a scale, we override our computed value
  // here.
  if (scale != 0) {
    projected_space_scale = scale;
  }

  // Compute the affine transform for this image.
  Matrix3x3 trans = geo.transform();

  trans(0,0) = projected_space_scale;
  trans(1,1) = -projected_space_scale;
  trans(0,2) = bbox.min().x();
  trans(1,2) = bbox.max().y();
  geo.set_transform(trans);

  // Compute the width and height of the output image so that it
  // contains the entirety of the computed bounding box.

  //***** FIXME: This seems backwards to me, which probably means that
  //***** lonlat_bounding_box() is backwards, but I need to get this
  //***** to work right now so I'll investigate later.
  output_height = (bbox.max().x() - bbox.min().x()) / projected_space_scale;
  output_width = (bbox.max().y() - bbox.min().y()) / projected_space_scale;

  return geo;
}


//***********************************************************************
// MAIN
//***********************************************************************

int main(int argc, char* argv[]) {

  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  std::string dem_file, image_file, camera_model_file, output_file;
  std::string stereo_session_string;
  double mpp;
  double ppd;
  double nodata_value;
  float lo = 0, hi = 0;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("mpp", po::value<double>(&mpp), "Specify the output resolution of the orthoimage in meters per pixel.")
    ("ppd", po::value<double>(&ppd), "Specify the output resolution of the orthoimage in pixels per degree.")
    ("nodata-value", po::value<double>(&nodata_value), "Specify the pixel used in this DEM to denote missing data.")
    ("match-dem,m", "Match the georeferencing parameters and dimensions of the input DEM.")
    ("min", po::value<float>(&lo), "Explicitly specify the range of the normalization (for ISIS images only).")
    ("max", po::value<float>(&hi), "Explicitly specify the range of the normalization (for ISIS images only).")
    ("session-type,t", po::value<std::string>(&stereo_session_string), "Select the stereo session type to use for processing. [default: pinhole]")
    ("draw-debug,d", "Draw the normalized DEM that ortho is rasterizing on")
    ("double-raster-size", "Double render size to get around artifact that we don't auto crop with terrain in mind");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("dem", po::value<std::string>(&dem_file), "DEM Input File")
    ("camera-image", po::value<std::string>(&image_file), "Camera Input file")
    ("camera-model", po::value<std::string>(&camera_model_file), "Camera Model File")
    ("output-file", po::value<std::string>(&output_file), "Output filename");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("dem", 1);
  positional_options_desc.add("camera-image", 1);
  positional_options_desc.add("camera-model", 1);
  positional_options_desc.add("output-file", 1);

  po::options_description all_options;
  all_options.add(visible_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify( vm );

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  if( vm.count("help") ||
      !vm.count("dem") ||
      !vm.count("camera-image") || !vm.count("camera-model")) {
    vw_out(0) << "\nUsage: orthoproject [options] <dem filename> <camera image> <camera model> <output filename>\n";
    vw_out(0) << visible_options << std::endl;
    return 1;
  }

  // Create a fresh stereo session and query it for the camera models.
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  // If the user hasn't specified a stereo session type, we take a
  // guess here based on the file suffixes.
  if (stereo_session_string.size() == 0) {
    if ( boost::iends_with(camera_model_file, ".cahvor") ||
         boost::iends_with(camera_model_file, ".cahv") ||
         boost::iends_with(camera_model_file, ".pin") ||
         boost::iends_with(camera_model_file, ".tsai") ) {
      vw_out(0) << "\t--> Detected pinhole camera files.  Executing pinhole stereo pipeline.\n";
      stereo_session_string = "pinhole";
    }

    else if (boost::iends_with(image_file, ".cub") ) {
      vw_out(0) << "\t--> Detected ISIS cube files.  Executing ISIS stereo pipeline.\n";
      stereo_session_string = "isis";
    }

    else {
      vw_out(0) << "\n\n******************************************************************\n";
      vw_out(0) << "Could not determine stereo session type.   Please set it explicitly\n";
      vw_out(0) << "using the -t switch.  Options include: [pinhole isis].\n";
      vw_out(0) << "******************************************************************\n\n";
      exit(0);
    }
  }

  // Isis Session command handling
  if (stereo_session_string == "isis") {
    if (!vm.count("output-file")) {
      // User didn't provide an output file, meaning camera_model was not an isis_adjust file.
      output_file = camera_model_file;
    }
  } else {
    if (!vm.count("output-file")) {
      vw_out(0) << "\nUsage: orthoproject [options] <dem filename> <camera image> <camera model> <output filename>\n";
      vw_out(0) << visible_options << std::endl;
      return 1;
    }
  }


  // Okay, here's a total hack.  We create a stereo session where both
  // of the imagers and images are the same, because we want to take
  // advantage of the stereo pipeline's ability to generate camera
  // models for various missions.  Hence, we create two identical
  // camera models, but only one is used.  The last four empty strings
  // are dummy arguments.
  StereoSession* session = StereoSession::create(stereo_session_string);
  session->initialize(image_file, image_file, camera_model_file, camera_model_file,
                      output_file, "", "","","" );
  boost::shared_ptr<camera::CameraModel> camera_model;
  camera_model = session->camera_model(image_file, camera_model_file);

  // Open the DEM
  GeoReference dem_georef;
  cartography::read_georeference(dem_georef, dem_file);

  DiskImageView<PixelGray<float> > dem_disk_image(dem_file);
  ImageViewRef<PixelMask<PixelGray<float> > > dem = pixel_cast<PixelMask<PixelGray<float> > >(dem_disk_image);

  if (vm.count("nodata-value")) {
    vw_out(0) << "\t--> Using " << nodata_value << " as the missing data value for the DEM.\n";
    dem = create_mask(dem_disk_image, nodata_value);
  }

  DiskImageView<PixelGrayA<uint8> > texture_disk_image(image_file);
  ImageViewRef<PixelGrayA<uint8> > texture_image = texture_disk_image;
  DiskImageView<PixelGrayA<float> > float_texture_disk_image(image_file);

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  // ISIS cubes need to be normalized because their pixels are often
  // photometrically calibrated.
  if (stereo_session_string == "isis") {
    if (lo == 0 && hi == 0) {
      min_max_channel_values(float_texture_disk_image, lo, hi);
      vw_out(0) << "\t--> Normalizing ISIS pixel range range: [" << lo << "  " << hi << "]\n";
    } else {
      vw_out(0) << "\t--> Using user-specified normalization range: [" << lo << "  " << hi << "]\n";
    }
    texture_image = channel_cast_rescale<uint8>(normalize_retain_alpha(remove_isis_special_pixels(float_texture_disk_image, PixelGrayA<float>(lo)), lo, hi, 0, 1.0));
  }
#endif

  // Parse the scale of the output image from the command line
  // arguments.  This value can be set either in meters per pixel or
  // pixels per degree.  At the moment, we haven't done the math to
  // convert ppd to meters in a projected coordinate system and mpp to
  // degrees in an unprojected coordinate system.  We throw an error
  // in these cases for now.
  double scale = 0;
  if (vm.count("ppd")) {
    if (dem_georef.is_projected()) {
      vw_out(0) << "Input DEM is in a map projection.  Cannot specify resolution in pixels per degree.  Use meters per pixel (-mpp) instead.";
      return 1;
    } else {
      scale = 1/ppd;
    }
  } else if (vm.count("mpp")) {
    if (dem_georef.is_projected()) {
      scale = mpp;
    } else {
      vw_out(0) << "Input DEM is in a simple cylindrical map projection (Plate Carree).  Cannot specify resolution in meters per pixel.  Use pixels per degree (-ppd) instead.";
      return 1;
    }
  }

  vw_out(0) << "\nOrthoprojecting:\n";

  GeoReference drg_georef = dem_georef;

  // If the user has supplied a scale, we use it.  Otherwise, we
  // compute the optimal scale of image based on the camera model.
  BBox2 projection_bbox, dem_bbox;
  {
    float mpp_auto_scale;
    BBox2 image_bbox = camera_bbox( dem, dem_georef, camera_model,
                                    texture_image.cols(), texture_image.rows(), mpp_auto_scale);

    if ( vm.count("double-raster-size") ) {
      double width = image_bbox.width();
      double height = image_bbox.height();
      image_bbox.min() -= Vector2(width,height)/2;
      image_bbox.max() += Vector2(width,height)/2;
    }
    dem_bbox = dem_georef.bounding_box(dem);

    // Use a bbox where both the image and the DEM have valid data.
    // Throw an error if there turns out to be no overlap.
    projection_bbox = image_bbox;
    projection_bbox.crop(dem_bbox);

    if ( projection_bbox.width() == 0 ||
         projection_bbox.height() == 0) {
      vw_out(0) << "Image bounding box (" << image_bbox << ") and DEM bounding box (" << dem_bbox << ") have no overlap.  Are you sure that your input files overlap?\n";
      return 1;
    }

    if ( scale == 0 ) {
      scale = mpp_auto_scale;
    }

  }

  double output_width = projection_bbox.width() / scale;
  double output_height = projection_bbox.height() / scale;

  Matrix3x3 drg_trans = drg_georef.transform();
  // Fixing for translation
  drg_trans(0,2) = projection_bbox.min().x();
  drg_trans(1,2) = projection_bbox.min().y();
  // This weird polarity checking is to make sure the output has been
  // transposed after going through reprojection. Normally this is the
  // case. Yet with grid data from GMT, it is not.     -ZMM
  if ( drg_trans(0,0) < 0 )
    drg_trans(0,2) = projection_bbox.max().x();
  drg_trans(0,0) = scale;
  if ( drg_trans(1,1) > 0 )
    drg_trans(1,2) = projection_bbox.max().y();
  drg_trans(1,1) = -scale;
  drg_georef.set_transform(drg_trans);

  GeoTransform trans(dem_georef, drg_georef);
  ImageViewRef<PixelMask<PixelGray<float> > > output_dem = crop(transform(dem, trans,
                                                                          ZeroEdgeExtension(),
                                                                          BicubicInterpolation()),
                                                                BBox2i(0,0,output_width,output_height));
  if ( vm.count("draw-debug") ) {
    write_image( camera_model_file+"_debug.tif", normalize(output_dem) );
  }


  ImageViewRef<PixelGrayA<uint8> > final_result = orthoproject(output_dem, drg_georef,
                                                               texture_image, camera_model,
                                                               BicubicInterpolation(), ZeroEdgeExtension());

  DiskImageResourceGDAL rsrc(output_file, final_result.format(), Vector2i(vw_settings().default_tile_size(),vw_settings().default_tile_size()) );
  write_georeference(rsrc, drg_georef);
  write_image(rsrc, final_result, TerminalProgressCallback());


//   // If the user has specified that we match the georeferencing
//   // parameters of the DEM, we use the DEM's georef as the output
//   // georef.  Otherwise we compute one of our own that contains the
//   // entirety of the input image and matches the pixel scale of the
//   // input image as closely as possible.
//   if (vm.count("match-dem")) {
//     write_georeferenced_image(output_file,
//                               orthoproject(dem, dem_georef,
//                                            texture_image, camera_model,
//                                            BilinearInterpolation(), ZeroEdgeExtension()),
//                               dem_georef,
//                               TerminalProgressCallback() );
//   }  else {
//     double output_width, output_height;
//     GeoReference output_georef = compute_geotransform_from_camera(texture_image, camera_model,
//                                                                   dem, dem_georef, scale,
//                                                                   output_width, output_height);
//     GeoTransform trans(dem_georef, output_georef);
//     ImageViewRef<PixelMask<PixelGray<float> > > output_dem = crop(transform(dem, trans,
//                                                                             ZeroEdgeExtension(),
//                                                                             BilinearInterpolation()),
//                                                                   BBox2i(0,0,output_width,output_height));

//     ImageViewRef<PixelGrayA<uint8> > final_result = orthoproject(output_dem, output_georef,
//                                                                  texture_image, camera_model,
//                                                                  BilinearInterpolation(), ZeroEdgeExtension());

//     DiskImageResourceGDAL rsrc(output_file, final_result.format());
//     rsrc.set_native_block_size(Vector2i(1024,1024));
//     write_georeference(rsrc, output_georef);
//     write_image(rsrc, final_result, TerminalProgressCallback());
//   }
}
