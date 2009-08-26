#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/random.hpp>

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjustmentSparse.h>
#include <vw/Camera/BundleAdjustReport.h>
#include <vw/Math.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Core/Log.h>
#include <vw/Core/ProgressCallback.h>

using namespace vw;
using namespace vw::camera;

#include "StereoSession.h"
#include "BundleAdjustUtils.h"
#include "ControlNetworkLoader.h"

#define CONFIG_FILE "ba_test.cfg"

/*
 * Typedefs and constants
 */

const double Pi = 3.1415926535897932;

typedef boost::minstd_rand base_rng_type;

// Camera types
typedef Vector<Vector3, 2>        CameraParams;
typedef std::vector<CameraParams> CameraParamVector;
typedef std::vector<PinholeModel> CameraVector;

// Camera constants
const double CameraElevation     = 100000.0; // meters
const double FoVOverlapPct       = 70.0;
//const double CameraFocalLength   = 0.610;    // Apollo Panoramic Camera (meters)
const double CameraFocalLength   = 0.0762;   // Apollo Metric Camera (meters)
const double PixelScale          = 10.0;     // meters per pixel
const int    ImgXPx              = 4000;     // x-dimension of synthetic image (pixels)
const int    ImgYPx              = 4000;     // y-dimension of synthetic image (pixels)
const double SurfaceHeightMargin = 3000.0;   // meters

/*
 * Program options
 */
/* {{{ ProgramOptions */
struct ProgramOptions {
  std::vector<std::string> image_files;
  std::vector<std::string> gcp_files;
  std::string              cnet_file;
  std::string              stereosession_type;
  double                   epsilon;
  double                   sigma_1;
  double                   sigma_2;
  double                   lambda;
  double                   robust_outlier_threshold;
  int                      min_tiepoints;
  int                      min_matches;
  int                      num_cameras;
  friend std::ostream& operator<<(std::ostream& ostr, ProgramOptions o);
};


std::ostream& operator<<(std::ostream& ostr, ProgramOptions o) {
  ostr << std::endl << "Configured Options" << std::endl;
  ostr << "----------------------------------------------------" << std::endl;
  ostr << "Stereo Session Type:             " << o.stereosession_type << std::endl;
  ostr << "Epsilon:                         " << o.epsilon << std::endl;
  ostr << "Sigma 1:                         " << o.sigma_1 << std::endl;
  ostr << "Sigma 2:                         " << o.sigma_2 << std::endl;
  ostr << "Lambda:                          " << o.lambda  << std::endl;
  ostr << "Robust Outlier Threshold:        " << o.robust_outlier_threshold << std::endl;
  ostr << "# of Cameras:                    " << o.num_cameras << std::endl;
  ostr << "Min # of Tiepoints per camera:   " << o.min_tiepoints << std::endl;
  ostr << "Minimum # of Matches:            " << o.min_matches << std::endl;
  return ostr;
}
/* }}} ProgramOptions */

/*
 * Function definitions
 */

/* {{{ initialize_rng */
base_rng_type initialize_rng() {
  base_rng_type rng(42u);
  rng.seed(static_cast<unsigned int>(std::time(0)));
  return rng;
}
/* }}} */

/* {{{ generate_camera_params */
// Generate synthetic camera parameters
CameraParamVector generate_camera_params(int num_cameras) {
  CameraParamVector cam_params;

  vw::vw_out(vw::InfoMessage) << "Generating camera parameters" << std::endl;

  // Cameras are created by incrementing camera_position in the x-direction
  // by a specified percentage of the calculated field of view on the surface
  double cameraFoV_X = static_cast<double>(ImgXPx * PixelScale);
  double x_increment = FoVOverlapPct * cameraFoV_X;
  double x = 0.0;
  double y = 0.0;
  double z = CameraElevation;

  for (int i = 0; i < num_cameras; i++) {
    Vector3 camera_position(x,y,z);
    // TODO: 1) what is the initial camera pose? Assuming we need a 90 deg
    // rotation to point camera directly down
    //       2) is this necessary? could we just assume camera is not rotated?
    Vector3 camera_pose(Pi/2, 0.0, 0.0);
    CameraParams param_vector(camera_position, camera_pose);
    cam_params.push_back(param_vector);
    x += x_increment;
  }
  return cam_params;
}
/* }}} generate_camera_params */

/* {{{ print_camera_params */
void print_camera_params(CameraParamVector cp) {
  CameraParamVector::iterator iter;
  int num_cameras = cp.size();

  for (int i = 0; i < num_cameras; i++) {
    Vector3 position = cp[i][0];
    Vector3 pose = cp[i][1];
    std::cout << "[" <<  i << "]\t";
    std::cout << position[0] << "\t" 
              << position[1] << "\t" 
              << position[2] << "\t";
    std::cout << pose[0] << "\t" 
              << pose[1] << "\t" 
              << pose[2] << std::endl; 
  }
}
/* }}} print_camera_params */

/* {{{ add_noise_to_cameras */
CameraParamVector 
add_noise_to_cameras(CameraParamVector camera_params) {
  return camera_params;
}
/* }}} add_noise_to_cameras */

/* {{{ add_noise_to_pixel */

Vector2 add_noise_to_pixel(Vector2 const pixel)
/*  base_rng_type & generator, 
		char *inlierType, 
    double const & inlierSd, 
    int const & inlierDf, 
    char *outlierType, 
    double const & outlierSd, 
		int const & outlierDf, 
    double const & outlierFrequency) */
{
  return pixel;
}
/* }}} add_noise_to_pixel */

/* {{{ generate_camera_models */
CameraVector generate_camera_models( CameraParamVector camera_params) {
  CameraVector camera_vec;
  double f_u, f_v, c_u, c_v;
  Matrix<double, 3, 3> rotation;
  Vector3 camera_center, camera_pose;

  vw::vw_out(vw::InfoMessage) << "Generating camera models" << std::endl;

  int num_cameras = camera_params.size();
  f_u = f_v = CameraFocalLength * PixelScale; // focal length shd be in meters
  c_u = c_v = 0.0;

  for (int i = 0; i < num_cameras; i++) {
    camera_center = camera_params[i][0];
    camera_pose   = camera_params[i][1];
    rotation = vw::math::euler_to_rotation_matrix(camera_pose[0],camera_pose[1],camera_pose[2],"xyz");
    camera_vec.push_back(PinholeModel(camera_center, rotation, f_u, f_v, c_u, c_v));
  }
  return camera_vec;
}
/* }}} genrate_camera_models */

/* {{{ point_in_image */
bool point_in_image( PinholeModel &c, const Vector2 &p) {
  double x = c.camera_center()[0];
  double x_min = x - ImgXPx / 2 * PixelScale;
  double x_max = x + ImgXPx / 2 * PixelScale;
  double y_max = ImgYPx / 2 * PixelScale;
  double y_min = -1 * y_max;

  if (p[0] <= x_max && p[0] >= x_min && p[1] <= y_max && p[1] >= y_min)
    return true;
  else
    return false;
}
/* }}} point_in_image */

/* {{{ generate_control_network */
boost::shared_ptr<ControlNetwork> 
generate_control_network(base_rng_type rng, CameraVector cameras, int min_tiepoints)
{
  boost::shared_ptr<ControlNetwork> control_network(new ControlNetwork("Synthetic Control Network"));
  int num_cameras = cameras.size();
  std::vector<int> point_counts (num_cameras, 0);

  vw::vw_out(vw::InfoMessage) << "Generating control network" << std::endl;;

  // Create random number generators for each dimension

  // In camera reference frame -- calculate the beginning and end of the
  // region covered by at least two cameras
  double x_min = cameras[1].camera_center()[0] - ImgXPx / 2 * PixelScale;
  double x_max = cameras[num_cameras-2].camera_center()[0] + ImgXPx / 2 * PixelScale;
  boost::uniform_real<> x_range(x_min, x_max);
  boost::variate_generator<base_rng_type, boost::uniform_real<> > x_rng(rng, x_range);
  vw::vw_out(vw::DebugMessage) << "\tx_min=" << x_min << "\tx_max=" << x_max << std::endl;

  // In camera reference frame -- y coordinates just extend to the top and
  // bottom of the imaged region
  double y_max = ImgYPx / 2 * PixelScale;
  double y_min = -1 * y_max;
  boost::uniform_real<> y_range(y_min, y_max);
  boost::variate_generator<base_rng_type, boost::uniform_real<> > y_rng(rng, y_range);
  vw::vw_out(vw::DebugMessage) << "\ty_min=" << y_min << "\ty_max=" << y_max << std::endl;
   
  // In camera reference frame -- camera elevation is 100km, so the
  // height of the surface is 0, and z value is uniformly distributed
  // around 0
  double z_min = 0.0 - SurfaceHeightMargin;
  double z_max = 0.0 + SurfaceHeightMargin;
  boost::uniform_real<> z_range(z_min, z_max);
  boost::variate_generator<base_rng_type, boost::uniform_real<> > z_rng(rng, z_range);
  vw::vw_out(vw::DebugMessage) << "\tz_min=" << z_min << "\tz_max=" << z_max << std::endl;

  // Generate random 3D points and reproject them into cameras until we
  // have enough points visible in each camera
  do {
    // generate random point
    Vector3 world_point(x_rng(), y_rng(), z_rng());
    //std::cout << world_point << std::endl;

    // create control point object
    ControlPoint cp;
    cp.set_position(world_point);

    // check each camera to see whether this point is in the image
    std::vector<ControlMeasure> measures;
    for (int i = 0; i < num_cameras; i++) {
      Vector2 img_point, noisy_img_point;
      try {
        // TODO: is this in the right coordinates?
        img_point = cameras[i].point_to_pixel(world_point); 
        // TODO: check this -- don't think it's working
        if (point_in_image(cameras[i], img_point)) {
          ControlMeasure cm;
          cm.set_image_id(i);
          // TODO: is this in the right coordinates?
          noisy_img_point = add_noise_to_pixel(img_point);
          cm.set_position(noisy_img_point);
          measures.push_back(cm);
          //std::cout << i << " " << world_point << std::endl;
        }
      }
      catch (vw::camera::PointToPixelErr()) {
        // ignore; just go back and check the next camera
      }
    }

    // Only keep this control point if it is visible in at least two
    // cameras
    if (measures.size() > 1) {
      // Increment the point counts for the appropriate cameras
      for (int i = 0; i < measures.size(); i++) {
        point_counts[measures[i].image_id()]++;
      }

      cp.add_measures(measures);
      control_network->add_control_point(cp);
    }
  }
  // keep going until each camera has at least min_tiepoints visible
  // points
  while (*min_element(point_counts.begin(), point_counts.end()) > min_tiepoints);

  return control_network;
}
/* }}} generate_control_network */

/* {{{ parse_options */
ProgramOptions parse_options(int argc, char* argv[]) {
  std::ifstream config_file (CONFIG_FILE, std::ifstream::in);
  ProgramOptions opts;

  // Generic Options
  po::options_description generic_opts("Options");
  generic_opts.add_options()
    ("help,?", "Display this help message")
    ("verbose,v", "Verbose output")
    ("print-config","Print configuration options and exit");

  // Test Configuration Options
  po::options_description test_opts("Test Configuration");
  test_opts.add_options()
    ("epsilon", po::value<double>(&opts.epsilon), "epsilon value")
    ("sigma-1", po::value<double>(&opts.sigma_1), "sigma_1 value")
    ("sigma-2", po::value<double>(&opts.sigma_2), "sigma_2 value")
    ("min-tiepoints-per-image", po::value<int>(&opts.min_tiepoints),"") // is the the same as min-matches?
    ("number-of-cameras", po::value<int>(&opts.num_cameras)->default_value(10),"");

  // Bundle Adjustment options
  po::options_description ba_opts("Bundle Adjustment Configuration");
  ba_opts.add_options()
    ("session-type,t", po::value<std::string>(&opts.stereosession_type)->default_value("isis"), 
        "Select the stereo session type to use for processing.")
    ("cnet,c", po::value<std::string>(&opts.cnet_file), 
        "Load a control network from a file")
    ("lambda,l", po::value<double>(&opts.lambda), 
        "Set the initial value of the LM parameter lambda")
    ("robust-threshold", po::value<double>(&opts.robust_outlier_threshold)->default_value(10.0), 
        "Set the threshold for robust cost functions")
    ("save-iteration-data,s", 
        "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt.")
    ("min-matches", po::value<int>(&opts.min_matches)->default_value(30), 
        "Set the minimum  number of matches between images that will be considered.");

  // Hidden Options (command line arguments)
  po::options_description hidden_opts("");
  //hidden_opts.add_options()
  //  ("input-files", po::value<std::vector<std::string> >(&image_files));

  // positional options spec
  po::positional_options_description p;
  //p.add("input-files", -1); // copied from BA -- will change
  
  // Allowed options includes generic and test config options
  po::options_description allowed_opts("Allowed Options");
  allowed_opts.add(generic_opts).add(test_opts).add(ba_opts);

  // All options included in command line options group
  po::options_description cmdline_opts;
  cmdline_opts.add(generic_opts).add(test_opts).add(ba_opts).add(hidden_opts);

  // test, bundle adjustment, and hidden options can be passed via config file
  po::options_description config_file_opts;
  config_file_opts.add(test_opts).add(ba_opts).add(hidden_opts);

  // Parse options on command line and config file
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(cmdline_opts).allow_unregistered().positional(p).run(), vm );
  po::store(po::parse_config_file(config_file, config_file_opts, true), vm);
  po::notify(vm);

  // Print usage message if requested
  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] [bundle adjustment options] " << std::endl
    << std::endl << allowed_opts << std::endl;
  if ( vm.count("help") ) {
    std::cout << usage.str() << std::endl;
    exit(1);
  }

  // Print config options if requested
  if (vm.count("print-config")) {
    std::cout << opts << std::endl;
    exit(0);
  } 

  vw::vw_log().console_log().rule_set().clear();
  vw::vw_log().console_log().rule_set().add_rule(vw::WarningMessage, "console");
  if ( vm.count("verbose") ) {
    vw::vw_log().console_log().rule_set().add_rule(vw::DebugMessage, "console");
  }

  return opts;
}
/* }}} parse_options */

int main(int argc, char* argv[]) {

  ProgramOptions config = parse_options(argc, argv);
  
  base_rng_type rng = initialize_rng();

  CameraParamVector camera_params = generate_camera_params(config.num_cameras);
  print_camera_params(camera_params);

  CameraVector cameras = generate_camera_models(camera_params);

  boost::shared_ptr<ControlNetwork> cnet = generate_control_network(rng, cameras, config.min_tiepoints);

  CameraParamVector noisy_camera_params = add_noise_to_cameras(camera_params);
  //print_camera_params(noisy_camera_params);
  
  CameraVector noisy_cameras = generate_camera_models(noisy_camera_params);

  //write_camera_models();

  //cnet->write_binary_control_network("control");

  return 0;
}



