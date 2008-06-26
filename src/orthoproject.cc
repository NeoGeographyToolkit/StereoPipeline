/************************************************************************
 *     File: stereo.cc
 *     Date: April 2005
 *       By: Michael Broxton and Larry Edwards
 *      For: NASA Ames Research Center, Intelligent Mechanisms Group 
 * Function: Main program for the stereo pipeline 
 ************************************************************************/
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

#include "asp_config.h"
#include "StereoSession.h"
using namespace std;

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif

#include "HRSC/StereoSessionHRSC.h"
#include "MOC/StereoSessionMOC.h"
#include "apollo/StereoSessionApolloMetric.h"
#include "MRO/StereoSessionCTX.h"
#include "RMAX/StereoSessionRmax.h"

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
    std::cout << "Image bounding box (" << image_bbox << ") and DEM bounding box (" << dem_bbox << ") have no overlap.  Are you sure that your input files overlap?\n";
    exit(0);
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
  output_width = (bbox.max().x() - bbox.min().x()) / projected_space_scale;
  output_height = (bbox.max().y() - bbox.min().y()) / projected_space_scale;

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
  int debug_level;
  unsigned cache_size;
  std::string dem_file, image_file, camera_model_file, output_file;
  std::string stereo_session_string;
  double mpp;
  double ppd;
  double missing_pixel_value;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("mpp", po::value<double>(&mpp), "Specify the output resolution of the orthoimage in meters per pixel.")
    ("ppd", po::value<double>(&ppd), "Specify the output resolution of the orthoimage in pixels per degree.")
    ("missing-pixel", po::value<double>(&missing_pixel_value)->default_value(-6000), "Specify the pixel used in this DEM to denote missing data.")
    ("match-dem,m", "Match the georeferencing parameters and dimensions of the input DEM.")
    ("cache", po::value<unsigned>(&cache_size)->default_value(2048), "Cache size, in megabytes")
    ("session-type,t", po::value<std::string>(&stereo_session_string)->default_value("pinhole"), "Select the stereo session type to use for processing. [default: pinhole]")
    ("debug-level,d", po::value<int>(&debug_level)->default_value(vw::DebugMessage-1), "Set the debugging output level. (0-50+)");

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
      !vm.count("camera-image") || !vm.count("camera-model") || 
      !vm.count("output-file")) {
    std::cout << "\nUsage: orthoproject [options] <dem filename> <camera image> <camera model> <output filename>\n";
    std::cout << visible_options << std::endl;
    return 1;
  }

  // Set the Vision Workbench debug level
  set_debug_level(debug_level);
  Cache::system_cache().resize( cache_size*1024*1024 ); // Set cache to 1Gb

  // Create a fresh stereo session and query it for the camera models.
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "metric", &StereoSessionApolloMetric::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

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

  if (vm.count("missing-pixel")) {
    std::cout << "\t--> Using " << missing_pixel_value << " as the missing data value for the DEM.\n";
    dem = create_mask(dem_disk_image, missing_pixel_value);
  }

  DiskImageView<PixelGrayA<uint8> > texture_disk_image(image_file);
  ImageViewRef<PixelGrayA<uint8> > texture_image = texture_disk_image;
  DiskImageView<PixelGrayA<float> > float_texture_disk_image(image_file);

  // ISIS cubes need to be normalized because their pixels are often
  // photometrically calibrated.
  if (stereo_session_string == "isis") {
    float lo, hi;
    isis_min_max_channel_values(float_texture_disk_image, lo, hi);
    vw_out(InfoMessage) << "Normalizing ISIS Pixel Range: [" << lo << " " << hi << "]    " << std::flush;
    texture_image = channel_cast_rescale<uint8>(normalize_retain_alpha(remove_isis_special_pixels(float_texture_disk_image, PixelGrayA<float>(lo)), lo, hi, 0, 1.0));
  }

  // Parse the scale of the output image from the command line
  // arguments.  This value can be set either in meters per pixel or
  // pixels per degree.  At the moment, we haven't done the math to
  // convert ppd to meters in a projected coordinate system and mpp to
  // degrees in an unprojected coordinate system.  We throw an error
  // in these cases for now.
  double scale = 0;
  if (vm.count("ppd")) {
    if (dem_georef.is_projected()) {
      std::cout << "Input DEM is in a map projection.  Cannot specify resolution in pixels per degree.  Use meters per pixel (-mpp) instead.";
      exit(0);
    } else {
      scale = 1/ppd;
    }
  } else if (vm.count("mpp")) {
    if (dem_georef.is_projected()) {
      scale = mpp;
    } else {
      std::cout << "Input DEM is in a simple cylindrical map projection (Plate Carree).  Cannot specify resolution in meters per pixel.  Use pixels per degree (-ppd) instead.";
      exit(0);      
    }
  }

  vw_out(0) << "\nOrthoprojecting:\n";

  // If the user has specified that we match the georeferencing
  // parameters of the DEM, we use the DEM's georef as the output
  // georef.  Otherwise we compute one of our own that contains the
  // entirety of the input image and matches the pixel scale of the
  // input image as closely as possible.
  if (vm.count("match-dem")) {
    write_georeferenced_image(output_file, 
                              orthoproject(dem, dem_georef, 
                                           texture_image, camera_model, 
                                           BilinearInterpolation(), ZeroEdgeExtension()), 
                              dem_georef,
                              TerminalProgressCallback() );
  }  else {
    double output_width, output_height;
    GeoReference output_georef = compute_geotransform_from_camera(texture_image, camera_model, 
                                                                  dem, dem_georef, scale,
                                                                  output_width, output_height);
    GeoTransform trans(dem_georef, output_georef);
    ImageViewRef<PixelMask<PixelGray<float> > > output_dem = crop(transform(dem, trans, 
                                                                ZeroEdgeExtension(), 
                                                                BilinearInterpolation()),
                                                      BBox2i(0,0,output_width,output_height));
    write_georeferenced_image(output_file, 
                              orthoproject(output_dem, output_georef, 
                                           texture_image, camera_model, 
                                           BilinearInterpolation(), ZeroEdgeExtension()),
                              output_georef,
                              TerminalProgressCallback() );
  }

}
