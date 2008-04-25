/************************************************************************
 *     File: orbitviz.cc
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
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::cartography;

#include "stereo.h"
#include "file_lib.h"
#include "StereoSession.h"
#include "SurfaceNURBS.h"
#include "MRO/DiskImageResourceDDD.h"	   // support for Malin DDD image files
#include "KML.h"

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif

#include "HRSC/StereoSessionHRSC.h"
#include "MOC/StereoSessionMOC.h"
#include "apollo/StereoSessionApolloMetric.h"
#include "clementine/StereoSessionClementine.h"
#include "MRO/StereoSessionCTX.h"

using namespace std;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

//***********************************************************************
// MAIN
//***********************************************************************

int main(int argc, char* argv[]) {

  // Register the DDD file handler with the Vision Workbench
  // DiskImageResource system.  DDD is the proprietary format used by
  // Malin Space Science Systems.
  DiskImageResource::register_file_type(".ddd",
                                        DiskImageResourceDDD::type_static(),
                                        &DiskImageResourceDDD::construct_open,
                                        &DiskImageResourceDDD::construct_create);
  
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif 

  // Register all stereo session types
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "metric", &StereoSessionApolloMetric::construct);
  StereoSession::register_session_type( "clementine", &StereoSessionClementine::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  std::string stereo_session_string;
  std::string in_file1, in_file2, cam_file1, cam_file2, out_file;
  double scale;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("session-type,t", po::value<std::string>(&stereo_session_string)->default_value("pinhole"), "Select the stereo session type to use for processing. [default: pinhole]")
    ("scale", po::value<double>(&scale)->default_value(1.0), "Scale the size of the coordinate axes by this amount");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(&in_file2), "Right Input Image")
    ("left-camera-model", po::value<std::string>(&cam_file1), "Left Camera Model File")
    ("right-camera-model", po::value<std::string>(&cam_file2), "Right Camera Model File")
    ("output-file", po::value<std::string>(&out_file)->default_value("orbit.kml"), "Output filename");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("left-camera-model", 1);
  positional_options_desc.add("right-camera-model", 1);
  positional_options_desc.add("output-file", 1);

  po::options_description all_options;
  all_options.add(visible_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify( vm );

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  if( vm.count("help") ||
      !vm.count("left-input-image") || !vm.count("right-input-image") || 
      !vm.count("left-camera-model") || !vm.count("right-camera-model") || 
      !vm.count("output-file")) {
    std::cout << "\nUsage: " << argv[0] << " [options] <Left_input_image> <Right_input_image> <Left_camera_file> <Right_camera_file> <output_file_prefix>\n"
              << "  the extensions are automaticaly added to the output files\n"
              << "  the parameters should be in stereo.default\n\n";
    std::cout << visible_options << std::endl;
    return 1;
  }


  StereoSession* session = StereoSession::create(stereo_session_string);
  session->initialize(in_file1, in_file2, cam_file1, cam_file2, 
                      out_file, "", "", "", "");

  // Generate some camera models
  boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
  session->camera_models(camera_model1, camera_model2);


  // Create the KML file.
  KMLStateVectorViz kml(out_file, "test", scale);
  kml.append_body_state("Camera 1", camera_model1->camera_center(Vector2()), camera_model1->camera_pose(Vector2()));
  kml.append_body_state("Camera 2", camera_model2->camera_center(Vector2()), camera_model2->camera_pose(Vector2()));
  kml.close();
  exit(0);
}
