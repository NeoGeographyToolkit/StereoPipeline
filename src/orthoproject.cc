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
using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;


#include "StereoSession.h"
using namespace std;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
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

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1024), "Cache size, in megabytes")
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

  // Okay, here's a total hack.  We create a stereo session where both
  // of the imagers and images are the same, because we want to take
  // advantage of the stereo pipeline's ability to generate camera
  // models for various missions.  Hence, we create two identical
  // camera models, but only one is used.  The last four empty strings
  // are dummy arguments.
  StereoSession* session = StereoSession::create(stereo_session_string);
  session->initialize(image_file, image_file, camera_model_file, camera_model_file, 
                      output_file, "", "","","" );
  boost::shared_ptr<camera::CameraModel> camera_model, unused_camera_model;
  session->camera_models(camera_model, unused_camera_model);
  
  // Open the DEM
  GeoReference georef;
  cartography::read_georeference(georef, dem_file);
  DiskImageView<double> dem(dem_file);

  // Compute the bounding box that encompasses all of the
  // available points.
  BBox3 dem_bbox;
  vw_out(InfoMessage) << "Computing DEM bounding box...\n";
  TerminalProgressCallback progress_callback;
  progress_callback.report_progress(0);

  for (unsigned int j = 0; j < dem.rows(); j++) {
    progress_callback.report_progress(float(j)/dem.rows());
    for (unsigned int i= 0; i < dem.cols(); i++) {
      Vector3 lon_lat = georef.transform() * Vector3(i,j,1);
      Vector3 lon_lat_alt = lon_lat / lon_lat(2);
      lon_lat_alt(2) = dem(i,j);
      dem_bbox.grow(lon_lat_alt);
    }
  }
  progress_callback.report_finished();

  std::cout << "DEM BBox: " << dem_bbox << "\n";

  std::cout << "Converting back to cartesian coordinates.\n";
  //  ImageViewRef<Vector3> point_image = dtm_to_point_image(dem, georef);
  ImageViewRef<Vector3> point_image = ImageView<Vector3>(10,10); // placeholder
  point_image = add_datum(point_image, georef.datum());
  point_image = lon_lat_radius_to_xyz(point_image);

  // Save out the results to a temporary file on disk.
  DiskCacheImageView<Vector3> cached_point_image(point_image, "exr");
  
  DiskImageView<PixelGray<uint8> > texture_disk_image(image_file);
  EdgeExtensionView<DiskImageView<PixelGray<uint8> >, ZeroEdgeExtension> texture_image(texture_disk_image, ZeroEdgeExtension());
  
  vw_out(InfoMessage) << "Orthoprojecting...\n";
  TerminalProgressCallback progress_callback2;
  progress_callback2.report_progress(0);

  ImageView<PixelGray<uint8> > result(cached_point_image.cols(), 
                                      cached_point_image.rows());
  for (unsigned int j = 0; j < cached_point_image.rows(); ++j) {
    for (unsigned int i = 0; i < cached_point_image.cols(); ++i) {
      Vector2 imaged_pix = camera_model->point_to_pixel(cached_point_image(i,j));
      result(i,j) = texture_image(imaged_pix(0),imaged_pix(1));
    }
  }
  
  // Write out the results
  write_georeferenced_image(output_file, result, georef);

  return(0);
}
