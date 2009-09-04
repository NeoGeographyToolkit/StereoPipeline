#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;
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

using std::cout;
using std::endl;
using std::ios;
using std::setiosflags;
using std::setw;
using std::setprecision;

/*
 * Typedefs and constants
 */

const double Pi = 3.1415926535897932;

typedef boost::minstd_rand base_rng_type;
typedef std::vector<boost::variate_generator<base_rng_type, boost::uniform_real<> > > RNGVector;

// Camera types
typedef Vector<Vector3, 2>        CameraParams;
typedef std::vector<CameraParams> CameraParamVector;
typedef std::vector<PinholeModel> CameraVector;

// Camera constants
const double CameraElevation     = 100000.0; // meters
const double FoVOverlapPct       = 0.70;
//const double CameraFocalLength   = 0.610;    // Apollo Panoramic Camera (meters)
//const double CameraFocalLength   = 0.0762;   // Apollo Metric Camera (meters)
//const double PixelScale          = 10.0;     // meters per pixel
const double CameraFoV_X         = 40000;    // meters
const double CameraFoV_Y         = 40000;    // meters
const int    ImgXPx              = 4000;     // x-dimension of synthetic image (pixels)
const int    ImgYPx              = 4000;     // y-dimension of synthetic image (pixels)
const double SurfaceHeightMargin = 3000.0;   // meters

const std::string ConfigFileDefault = "ba_test.cfg";
const std::string CnetFile          = "control.txt";
const std::string NoisyCnetFile     = "noisy_control.txt";
const std::string CnetPrefix        = "control";
const std::string NoisyCnetPrefix   = "noisy_control";
const std::string CameraPrefix      = "camera";
const std::string NoisyCameraPrefix = "noisy_camera";
const std::string TrueWPFile        = "wp_ground_truth.txt";
const std::string TrueCamFile       = "cam_ground_truth.txt";

/*
 * Noise Parameters
 */
/* {{{ NoiseParams */
enum NoiseT { NONE, NORMAL, LAPLACE, STUDENT }; 

struct NoiseParams{
  NoiseT  inlierType;
  double  inlierSd;
  int     inlierDf;
  NoiseT  outlierType;
  double  outlierSd; 
  int     outlierDf;
  double  outlierFreq;
  friend std::ostream& operator<<(std::ostream& ostr, NoiseParams p);
};

std::ostream& operator<<(std::ostream& ostr, NoiseParams p) {
  ostr << "Inlier Noise Type: ";
  switch (p.inlierType) {
    case NONE:
      ostr << "None"; break;
    case NORMAL:
      ostr << "Normal"; break;
    case LAPLACE:
      ostr << "LaPlace"; break;
    case STUDENT:
      ostr << "Student's T"; break;
    default:
      ostr << "unrecognized type";
  }
  ostr << endl;
  ostr << "Inlier Sigma: " << p.inlierSd << endl;
  ostr << "Inlier DF: " << p.inlierDf << endl;
  ostr << "Outlier Frequency: " << p.outlierFreq << endl;
  ostr << "Outlier Noise Type: ";
  switch (p.outlierType) {
    case NONE:
      ostr << "None"; break;
    case NORMAL:
      ostr << "Normal"; break;
    case LAPLACE:
      ostr << "LaPlace"; break;
    case STUDENT:
      ostr << "Student's T"; break;
    default:
      ostr << "unrecognized type";
  }
  ostr << endl;
  ostr << "Outlier Sigma: " << p.outlierSd << endl;
  ostr << "Outlier DF: " << p.outlierDf << endl;
  return ostr;
}

NoiseT string_to_noise_type(std::string &s) {
  NoiseT t = NONE;
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  if (s == "normal") t=NORMAL;
  else if (s == "laplace") t=LAPLACE;
  else if (s == "student") t=STUDENT;
  return t;
}

/* }}} NoiseParams */

/*
 * Program options
 */
/* {{{ ProgramOptions */
struct ProgramOptions {
  NoiseParams pixel_params;
  NoiseParams camera_xyz_params;
  NoiseParams camera_euler_params;
  fs::path    data_dir;
  fs::path    config_file;
  int         min_tiepoints;
  int         num_cameras;
  friend std::ostream& operator<<(std::ostream& ostr, ProgramOptions o);
};


std::ostream& operator<<(std::ostream& ostr, ProgramOptions o) {
  ostr << endl << "Configured Options (read from " << o.config_file << ")" << endl;
  ostr << "----------------------------------------------------" << endl;
  ostr << "Pixel noise parameters:" << endl;
  ostr << o.pixel_params << endl;
  ostr << "Camera XYZ noise parameters:" << endl;
  ostr << o.camera_xyz_params << endl;
  ostr << "Camera Euler angle noise parameters:" << endl;
  ostr << o.camera_euler_params << endl;
  ostr << "# of Cameras: " << o.num_cameras << endl;
  ostr << "Min # of Tiepoints per camera: " << o.min_tiepoints << endl;
  ostr << "Data output directory: " << o.data_dir << endl;
  return ostr;
}
/* }}} ProgramOptions */

/*
 * Function definitions
 */

/* {{{ parse_options */
ProgramOptions parse_options(int argc, char* argv[]) {
  ProgramOptions opts;

  // Generic Options
  po::options_description generic_opts("Options");
  generic_opts.add_options()
    ("help,?", "Display this help message")
    ("verbose,v", "Verbose output")
    ("debug,d", "Debugging output")
    ("config-file,f", po::value<fs::path>(&opts.config_file)->default_value(ConfigFileDefault), 
        "File containing configuration options (if not given, defaults to reading ba_test.cfg in the current directory")
    ("print-config","Print configuration options and exit");

  // Test Configuration Options
  std::string pixelInlierType;
  std::string pixelOutlierType;
  std::string xyzInlierType;
  std::string xyzOutlierType;
  std::string eulerInlierType;
  std::string eulerOutlierType;

  po::options_description test_opts("Test Data Configuration");
  test_opts.add_options()
    ("pixel-inlier-noise-type", po::value<std::string>(&pixelInlierType), 
        "Type of noise to add to inlier pixel coordinates [None, Normal, Laplace, Student]")
    ("pixel-inlier-df", po::value<int>(&opts.pixel_params.inlierDf), 
        "Degrees of freedom for inlier pixel noise")
    ("pixel-inlier-sigma", po::value<double>(&opts.pixel_params.inlierSd),
        "sigma for inlier pixel noise")
    ("pixel-outlier-noise-type", po::value<std::string>(&pixelOutlierType),
        "Type of noise to add to outlier pixel coordinates [None, Normal, Laplace, Student]")
    ("pixel-outlier-df", po::value<int>(&opts.pixel_params.outlierDf), 
        "Degrees of freedom for outlier pixel noise")
    ("pixel-outlier-sigma", po::value<double>(&opts.pixel_params.outlierSd),
        "sigma for outlier pixel noise")
    ("pixel-outlier-freq", po::value<double>(&opts.pixel_params.outlierFreq),
        "outlier frequency for pixel noise")
    ("xyz-inlier-noise-type", po::value<std::string>(&xyzInlierType), 
        "Type of noise to add to inlier camera xyz coordinates [None, Normal, Laplace, Student]")
    ("xyz-inlier-df", po::value<int>(&opts.camera_xyz_params.inlierDf), 
        "Degrees of freedom for inlier camera xyz noise")
    ("xyz-inlier-sigma", po::value<double>(&opts.camera_xyz_params.inlierSd),
        "sigma for inlier xyz noise")
    ("xyz-outlier-noise-type", po::value<std::string>(&xyzOutlierType),
        "Type of noise to add to outlier camera xyz coordinates [None, Normal, Laplace, Student]")
    ("xyz-outlier-df", po::value<int>(&opts.camera_xyz_params.outlierDf), 
        "Degrees of freedom for outlier camera xyz noise")
    ("xyz-outlier-sigma", po::value<double>(&opts.camera_xyz_params.outlierSd),
        "sigma for outlier camera xyz noise")
    ("xyz-outlier-freq", po::value<double>(&opts.camera_xyz_params.outlierFreq),
        "outlier frequency for camera xyz noise")
    ("euler-inlier-noise-type", po::value<std::string>(&eulerInlierType), 
        "Type of noise to add to inlier camera euler coordinates [None, Normal, Laplace, Student]")
    ("euler-inlier-df", po::value<int>(&opts.camera_euler_params.inlierDf), 
        "Degrees of freedom for inlier camera euler noise")
    ("euler-inlier-sigma", po::value<double>(&opts.camera_euler_params.inlierSd),
        "sigma for inlier euler noise")
    ("euler-outlier-noise-type", po::value<std::string>(&eulerOutlierType),
        "Type of noise to add to outlier camera euler coordinates [None, Normal, Laplace, Student]")
    ("euler-outlier-df", po::value<int>(&opts.camera_euler_params.outlierDf), 
        "Degrees of freedom for outlier camera euler noise")
    ("euler-outlier-sigma", po::value<double>(&opts.camera_euler_params.outlierSd),
        "sigma for outlier camera euler noise")
    ("euler-outlier-freq", po::value<double>(&opts.camera_euler_params.outlierFreq),
        "outlier frequency for camera euler noise")
    ("min-tiepoints-per-image", po::value<int>(&opts.min_tiepoints),"") // is the the same as min-matches?
    ("number-of-cameras", po::value<int>(&opts.num_cameras)->default_value(10),"")
    ("data-dir", po::value<fs::path>(&opts.data_dir)->default_value("."),
        "Directory to write generated data files into");

  // Allowed options includes generic and test config options
  po::options_description allowed_opts("Allowed Options");
  allowed_opts.add(generic_opts).add(test_opts);

  // All options included in command line options group
  po::options_description cmdline_opts;
  cmdline_opts.add(generic_opts).add(test_opts);

  // test options can be passed via config file
  po::options_description config_file_opts;
  config_file_opts.add(test_opts);

  // Parse options on command line and config file
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(cmdline_opts).allow_unregistered().run(), vm );
  std::ifstream config_file_istr(
      boost::any_cast<fs::path>(vm["config-file"].value()).string().c_str(), 
      std::ifstream::in);
  po::store(po::parse_config_file(config_file_istr, config_file_opts, true), vm);
  po::notify(vm);

  // Print usage message if requested
  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] " << endl
    << endl << allowed_opts << endl;
  if ( vm.count("help") ) {
    cout << usage.str() << endl;
    exit(1);
  }

  opts.pixel_params.inlierType  = string_to_noise_type(pixelInlierType);
  opts.pixel_params.outlierType = string_to_noise_type(pixelOutlierType);
  opts.camera_xyz_params.inlierType    = string_to_noise_type(xyzInlierType);
  opts.camera_xyz_params.outlierType   = string_to_noise_type(xyzOutlierType);
  opts.camera_euler_params.inlierType  = string_to_noise_type(eulerInlierType);
  opts.camera_euler_params.outlierType = string_to_noise_type(eulerOutlierType);

  // Print config options if requested
  if (vm.count("print-config")) {
    cout << opts << endl;
    exit(0);
  } 

  // If data directory does not exist, create it
  if (fs::exists(opts.data_dir) && !fs::is_directory(opts.data_dir)) {
    std::cerr << "Error: " << opts.data_dir << " is not a directory." << endl;
    exit(1);
  } 
  else
    fs::create_directory(opts.data_dir);

  vw::vw_log().console_log().rule_set().clear();
  vw::vw_log().console_log().rule_set().add_rule(vw::WarningMessage, "console");
  if (vm.count("verbose"))
    vw::vw_log().console_log().rule_set().add_rule(DebugMessage, "console");
  if (vm.count("debug"))
    vw::vw_log().console_log().rule_set().add_rule(VerboseDebugMessage, "console");

  return opts;
}
/* }}} parse_options */

/* {{{ Noise Generators */

/* {{{ genUniform */
double genUniform(double const & a, double const & b, base_rng_type & rng){
  // a must be less than b
  assert(a < b);

  //  base_rng_type rng = rng;   
  boost::uniform_real<> uni_dist(a,b);
  boost::variate_generator<base_rng_type&, boost::uniform_real<> > uni(rng, uni_dist); 
  
  //rng.seed(static_cast<unsigned int>(std::time(0)));

  double val; 
  val = uni();

  return val;
}
/* }}} genUniform */

/* {{{ genBernoulli */
double genBernoulli(double const & p, base_rng_type & rng){

  //  base_rng_type rng = rng;   
  
  boost::bernoulli_distribution<> bern_dist(p);
  boost::variate_generator<base_rng_type&, boost::bernoulli_distribution<> > bern(rng, bern_dist);
 
  //rng.seed(static_cast<unsigned int>(std::time(0)));

  double val; 
  val = bern();

  return val;
}
/* }}} genBernoulli */

/* {{{ genExponential */
double genExponential(double const & lambda, base_rng_type & rng){

  //  base_rng_type rng = rng;   
  
  boost::exponential_distribution<> exp_dist(lambda);
  boost::variate_generator<base_rng_type&, boost::exponential_distribution<> > exp(rng, exp_dist);
  //rng.seed(static_cast<unsigned int>(std::time(0)));

  double val; 
  val = exp();

  return val;
}
/* }}} genExponential */

/* {{{ genLaplace */
double genLaplace(double const & sigma, base_rng_type & rng){

  //  base_rng_type rng = rng;   
  
  double p = 0.5;

  double bern = genBernoulli(p, rng);
  double exp = genExponential(1/sigma, rng);

  double val; 
  val = (bern - 0.5)*sigma*exp/(sqrt(2));

  return val;
}
/* }}} genLaplace */

/* {{{ genNormal */
double genNormal(double const & mu, double const & sigma, base_rng_type & rng){

  //  base_rng_type rng = rng;   
  
  boost::normal_distribution<> norm_dist(mu, sigma);
  boost::variate_generator<base_rng_type&, boost::normal_distribution<> > norm(rng, norm_dist);
 
  double val; 
  val = norm();

  return val;
}
/* }}} genNormal */

/* {{{ genChisquare */
double genChisquare(int const & df, base_rng_type & rng){

  //  base_rng_type rng = rng;   
  
  // Set up parameters for normal
  double mu = 0.0;
  double sigma = 1.0;

  double val = 0.0;
  
  for (int j = 0; j < df; j++){
    double temp = genNormal(mu, sigma, rng);
    val += temp*temp;
  }

 
  return val;
}
/* }}} genChisquare */

/* {{{ genStudent */
double genStudent(int const & df, base_rng_type & rng){

  //  base_rng_type rng = rng;   

  // Set up parameters for normal: should be 0,1
  double mu = 0.0;
  double sigma = 1.0;

  // From the Student T characterization, Z/sqrt(V/nu) where V is a Chi^2
  double norm = genNormal(mu, sigma, rng);
  double chi = genChisquare(df, rng);
  double val  = norm/(sqrt(chi/df));
  
  return val;
}
/* }}} genStudent */

/* }}} Noise Generators */

/* {{{ add_noise_to_vector */
template <typename T>
T add_noise_to_vector(T const &vec, base_rng_type &rng, NoiseParams const params) 
{
  NoiseT noiseType;
  double sigma;
  int df;

  // initialize return value
  T ret = vec;

  int size = vec.size();

  // Generate a uniform random variable to decide whether to create an outlier
  if (genUniform(0, 1, rng) < params.outlierFreq){
    noiseType = params.outlierType;
    sigma     = params.outlierSd;
    df        = params.outlierDf;
  }
  else {
    noiseType = params.inlierType;
    sigma     = params.inlierSd;
    df        = params.inlierDf;
  }

  switch ( noiseType ) {
     
    case NORMAL :
      for (int i = 0; i < size; i++)
        ret[i] += genNormal(0, sigma, rng);
      break;

    case LAPLACE : 
      for (int i = 0; i < size; i++)
        ret[i] += genLaplace(sigma, rng);
      break;
     
    case STUDENT :      
      for (int i = 0; i < size; i++)
        ret[i] += genLaplace(df, rng);
      break;   

  } 

  return ret;
}
/* }}} add_noise_to_vector */

/* {{{ add_noise_to_camera */
CameraParams
add_noise_to_camera(CameraParams cp, base_rng_type &rng, 
    NoiseParams xyz_params, NoiseParams euler_params) 
{

  Vector3 xyz = cp[0];
  Vector3 euler = cp[1];
  Vector3 noisy_xyz   = add_noise_to_vector(xyz, rng, xyz_params);    
  Vector3 noisy_euler = add_noise_to_vector(euler, rng, euler_params);
  CameraParams ret(noisy_xyz, noisy_euler);
  return ret;
}
/* }}} add_noise_to_cameras */

/* {{{ add_noise_to_camera_vector */
CameraParamVector add_noise_to_camera_vector(CameraParamVector const &cpv,
    base_rng_type &rng, NoiseParams xyz_params, NoiseParams euler_params)
{
  CameraParamVector cpv_ret;
  int num_cameras = cpv.size();

  for (int i = 0; i < num_cameras; i++) {
    cpv_ret.push_back(add_noise_to_camera(cpv[i], rng, xyz_params, euler_params));
  }

  return cpv_ret;
}
/* }}} add_noise_to_camera_vector */

/* {{{ add_noise_to_pixel */

Vector2 add_noise_to_pixel(Vector2 const pixel, base_rng_type &rng, NoiseParams params) {
  return add_noise_to_vector(pixel, rng, params);
}

//* }}} add_noise_to_pixel */

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

  vw::vw_out(vw::InfoMessage) << "Generating camera parameters" << endl;

  // Cameras are created by incrementing camera_position in the x-direction
  // by a specified percentage of the calculated field of view on the surface
  //double cameraFoV_X = static_cast<double>(ImgXPx * PixelScale);
  double x_increment = (1 - FoVOverlapPct) * CameraFoV_X;
  double x = 0.0;
  double y = 0.0;
  double z = CameraElevation;
  vw_out(DebugMessage) << "\tx_increment="
      << setiosflags(ios::fixed) << setprecision(0) << x_increment << endl;

  for (int i = 0; i < num_cameras; i++) {
    Vector3 camera_position(x,y,z);
    Vector3 camera_pose(0.0,0.0,0.0);
    CameraParams param_vector(camera_position, camera_pose);
    cam_params.push_back(param_vector);
    x += x_increment;
  }
  return cam_params;
}
/* }}} generate_camera_params */

/* {{{ print_camera_params */
void print_camera_params(CameraParamVector cp) {
  int num_cameras = cp.size();

  vw_out(DebugMessage) << "Camera Parameters:" << endl;
  for (int i = 0; i < num_cameras; i++) {
    Vector3 position = cp[i][0];
    Vector3 pose = cp[i][1];
    vw_out(DebugMessage) 
         << "[" <<  i << "]\t"
         << setprecision(0)
         << setiosflags(ios::fixed)
         << setiosflags(ios::right)
         << setw(10) << position[0]
         << setw(10) << position[1]
         << setw(10) << position[2]
         << setprecision(6)
         << setw(10) << pose[0]
         << setw(10) << pose[1]
         << setw(10) << pose[2] << endl; 
  }
}
/* }}} print_camera_params */

/* {{{ generate_camera_models */
CameraVector generate_camera_models( CameraParamVector camera_params) {
  CameraVector camera_vec;
  double f_u, f_v, c_u, c_v;
  Matrix<double, 3, 3> rotation;
  Vector3 camera_center, camera_pose;

  vw::vw_out(vw::InfoMessage) << "Generating camera models" << endl;

  int num_cameras = camera_params.size();
  // focal length shd be in pixels
  f_u = ImgXPx / CameraFoV_X * CameraElevation; 
  f_v = ImgYPx / CameraFoV_Y * CameraElevation; 
  //f_u = f_y = CameraFocalLength / PixelScale; // m / (m / p) = p
  c_u = c_v = 0.0;

  for (int i = 0; i < num_cameras; i++) {
    camera_center = camera_params[i][0];
    camera_pose   = camera_params[i][1];
    rotation = vw::math::euler_to_rotation_matrix(camera_pose[0],camera_pose[1],camera_pose[2],"xyz");
    camera_vec.push_back(PinholeModel(camera_center, rotation, f_u, f_v, c_u, c_v));
  }
  return camera_vec;
}
/* }}} generate_camera_models */

/* {{{ point_in_image */
bool point_in_image(const Vector2 &p) {
  double x_max = ImgXPx / 2;
  double x_min = -1 * x_max;
  double y_max = ImgYPx / 2;
  double y_min = -1 * y_max;

  if (p[0] <= x_max && p[0] >= x_min && p[1] <= y_max && p[1] >= y_min)
    return true;
  else
    return false;
}
/* }}} point_in_image */

/* {{{ generate_xyz_rngs */
RNGVector generate_xyz_rngs(base_rng_type &rng, CameraVector &cameras) {
  RNGVector rng_vec;
  int num_cameras = cameras.size();

  // In camera reference frame -- calculate the beginning and end of the
  // region covered by at least two cameras
  double x_min = cameras[1].camera_center()[0] - CameraFoV_X / 2;
  double x_max = cameras[num_cameras-2].camera_center()[0] + CameraFoV_X / 2;
  boost::uniform_real<> x_range(x_min, x_max);
  boost::variate_generator<base_rng_type, boost::uniform_real<> > x_rng(rng, x_range);
  rng_vec.push_back(x_rng);
  vw::vw_out(vw::DebugMessage) 
    << setiosflags(ios::fixed) << setprecision(0) 
    << "\tx_min=" << x_min << "\tx_max=" << x_max << endl;

  // In camera reference frame -- y coordinates just extend to the top and
  // bottom of the imaged region
  double y_max = CameraFoV_Y / 2;
  double y_min = -1 * y_max;
  boost::uniform_real<> y_range(y_min, y_max);
  boost::variate_generator<base_rng_type, boost::uniform_real<> > y_rng(rng, y_range);
  rng_vec.push_back(y_rng);
  vw::vw_out(vw::DebugMessage) 
    << setiosflags(ios::fixed) << setprecision(0) 
    << "\ty_min=" << y_min << "\ty_max=" << y_max << endl;
   
  // In camera reference frame -- camera elevation is 100km, so the
  // height of the surface is 0, and z value is uniformly distributed
  // around 0
  double z_min = 0.0 - SurfaceHeightMargin;
  double z_max = 0.0 + SurfaceHeightMargin;
  boost::uniform_real<> z_range(z_min, z_max);
  boost::variate_generator<base_rng_type, boost::uniform_real<> > z_rng(rng, z_range);
  rng_vec.push_back(z_rng);
  vw::vw_out(vw::DebugMessage) 
    << setiosflags(ios::fixed) << setprecision(0) 
    << "\tz_min=" << z_min << "\tz_max=" << z_max << endl;

  return rng_vec;
}
/* }}} generate_xyz_rngs */

/* {{{ generate_control_network */
boost::shared_ptr<ControlNetwork> 
generate_control_network(base_rng_type &rng, CameraVector &cameras, int &min_tiepoints)
{
  boost::shared_ptr<ControlNetwork> control_network(new ControlNetwork("Synthetic Control Network"));
  int num_cameras = cameras.size();
  std::vector<int> point_counts (num_cameras, 0);

  vw_out(InfoMessage) << "Generating control network" << endl;;

  // Create random number generators for each dimension
  RNGVector rng_vec = generate_xyz_rngs(rng, cameras);

  // Generate random 3D points and reproject them into cameras until we
  // have enough points visible in each camera
  do {
    // generate random point
    Vector3 world_point(rng_vec[0](), rng_vec[1](), rng_vec[2]());

    // create control point object
    ControlPoint cp;
    cp.set_position(world_point);

    // check each camera to see whether this point is in the image
    std::vector<ControlMeasure> measures;
    for (int i = 0; i < num_cameras; i++) {
      Vector2 img_point, noisy_img_point;
      try {
        img_point = cameras[i].point_to_pixel(world_point); 
      }
      catch (vw::camera::PointToPixelErr()) {
        // ignore; just go back and check the next camera
        continue;
      }
      vw_out(VerboseDebugMessage) 
        << "IP (" << img_point[0] << "," << img_point[1] << ")" << endl;

      if (point_in_image(img_point)) {
        ControlMeasure cm;
        cm.set_sigma(1,1);
        cm.set_image_id(i);
        cm.set_position(img_point);
        measures.push_back(cm);
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
      vw_out(VerboseDebugMessage) << "CP " 
        << "(" << world_point[0] << "," << world_point[1] << "," << world_point[2] << ")" 
        << " visible in " << measures.size() << " cameras" << endl;
    }

    int min_point_count = *min_element(point_counts.begin(), point_counts.end());
    vw_out(VerboseDebugMessage) << "Min point count: " << min_point_count << endl;
  }
  // keep going until each camera has at least min_tiepoints visible
  // points
  while (*min_element(point_counts.begin(), point_counts.end()) < min_tiepoints);

  return control_network;
}
/* }}} generate_control_network */

/* {{{ add_noise_to_control_network */
boost::shared_ptr<ControlNetwork>
add_noise_to_control_network(boost::shared_ptr<ControlNetwork> cnet, base_rng_type &rng, NoiseParams np)
{
  boost::shared_ptr<ControlNetwork> noisy_cnet(new ControlNetwork("Noisy Control Network"));

  int cnet_size = cnet->size();
  for (int i = 0; i < cnet_size; i++) {
    ControlPoint cp;
    cp.set_position((*cnet)[i].position());

    int num_measures = (*cnet)[i].size();
    for (int j = 0; j < num_measures; j++) {
      ControlMeasure cm;
      cm.set_sigma((*cnet)[i][j].sigma());
      cm.set_image_id((*cnet)[i][j].image_id());
      cm.set_position(add_noise_to_pixel((*cnet)[i][j].position(), rng, np));
      cp.add_measure(cm);
    }
    noisy_cnet->add_control_point(cp);
  }

  return noisy_cnet;
}

/* }}} add_noise_to_control_network */

/* {{{ write_control_network */
/* 
 * Writes out the control network to a tap-separated file for debugging
 */
void write_control_network(boost::shared_ptr<ControlNetwork> cnet, int num_cameras,
    std::string cnet_file, fs::path dir) 
{
  fs::ofstream cnetos (dir / cnet_file);

  // file header
  cnetos << "WP\t\t\t";
  for (int i = 0; i < num_cameras; i++) {
    cnetos << i << "\t\t";
  }
  cnetos << endl;

  ControlNetwork::iterator cnet_iter;
  for (cnet_iter = cnet->begin(); cnet_iter != cnet->end(); cnet_iter++) {
    ControlPoint cp = *cnet_iter;
    Vector3 pos = cp.position();
    cnetos << pos[0] << "\t" << pos[1] << "\t" << pos[2] << "\t";
    for (int i = 0; i < num_cameras; i++) {
      bool match = false;
      for (int j = 0; j < cp.size(); j++) {
        ControlMeasure cm = cp[j];
        if (cm.image_id() == i) {
          Vector2 cmpos = cm.position();
          cnetos << cmpos[0] << "\t" << cmpos[1];
          match = true;
          break;
        }
      }
      cnetos << (match ? "\t" : "\t\t");
    }
    cnetos << endl;
  }

  cnetos.close();
}
/* }}} write_control_network */

/* {{{ write_world_points */
void write_world_points(boost::shared_ptr<ControlNetwork> cnet,
        std::string file, fs::path dir)
{
  fs::ofstream os(dir / file.c_str());
  ControlNetwork::iterator cnet_iter;
  for (cnet_iter = cnet->begin(); cnet_iter != cnet->end(); cnet_iter++) {
    ControlPoint cp = *cnet_iter;
    Vector3 pos = cp.position();
    os << pos[0] << "\t" << pos[1] << "\t" << pos[2] << endl;
  }
  os.close();
}
/* }}} */

/* {{{ write_camera_params */
void write_camera_params(CameraParamVector params, std::string file, fs::path dir) {
  fs::ofstream os(dir / file);
  CameraParamVector::iterator iter;
  Vector3 c, p;

  for (iter = params.begin(); iter != params.end(); ++iter) {
    c = (*iter)[0];
    p = (*iter)[1];
    os << c[0] << "\t" << c[1] << "\t" << c[2] << "\t";
    os << p[0] << "\t" << p[1] << "\t" << p[2] << endl;
  }
  os.close();
}
/* }}} */

/* {{{ write_camera_models */
void write_camera_models(CameraVector cameras, std::string fname, fs::path dir) 
{
  int num_cameras = cameras.size();
  for (int i = 0; i < num_cameras; i++) {
    std::stringstream out;
    out << dir / fname << i << ".tsai";
    cameras[i].write_file(out.str());
  }
}
/* }}} */

int main(int argc, char* argv[]) {
  ProgramOptions config = parse_options(argc, argv);

  base_rng_type rng = initialize_rng();

  // Generate ground truth camera parameters
  CameraParamVector camera_params = generate_camera_params(config.num_cameras);
  print_camera_params(camera_params);

  // Add configured noise to camera parameters
  CameraParamVector noisy_camera_params = add_noise_to_camera_vector(
      camera_params, rng, config.camera_xyz_params, config.camera_euler_params);
  print_camera_params(noisy_camera_params);

  // Generate camera models from GT and noisy camera parameters
  CameraVector cameras = generate_camera_models(camera_params);
  CameraVector noisy_cameras = generate_camera_models(noisy_camera_params);

  // Generate random control network
  boost::shared_ptr<ControlNetwork> 
      cnet = generate_control_network(rng, cameras, config.min_tiepoints);
  // Add configured noise to control network
  boost::shared_ptr<ControlNetwork> 
      noisy_cnet = add_noise_to_control_network(cnet, rng, config.pixel_params);

  // Write camera model files
  write_camera_models(cameras, CameraPrefix, config.data_dir);
  write_camera_models(noisy_cameras, NoisyCameraPrefix, config.data_dir);

  // Write binary control network files
  std::string cnet_file = fs::path(config.data_dir / CnetPrefix).string();
  std::string noisy_cnet_file = fs::path(config.data_dir / NoisyCnetPrefix).string();
  cnet->write_binary_control_network(cnet_file);
  noisy_cnet->write_binary_control_network(noisy_cnet_file);

  // Write ground truth files for world points and camera parameters
  write_world_points(cnet, TrueWPFile, config.data_dir);
  write_camera_params(camera_params, TrueCamFile, config.data_dir);

  // Write control network text files
  write_control_network(cnet, config.num_cameras, CnetFile, config.data_dir);
  write_control_network(noisy_cnet, config.num_cameras, NoisyCnetFile, config.data_dir);

  return 0;
}



