// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file bundle_adjust.cc
///

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include "boost/filesystem/operations.hpp" 
#include "boost/filesystem/fstream.hpp"    
namespace po = boost::program_options;
namespace fs = boost::filesystem;                   

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjustmentSparse.h>
#include <vw/Camera/BundleAdjustReport.h>
#include <vw/Math.h>

using namespace vw;
using namespace vw::camera;

#include <stdlib.h>
#include <iostream>

#include "asp_config.h"
#include "StereoSession.h"
#include "BundleAdjustUtils.h"
#include "ControlNetworkLoader.h"

#include "RMAX/StereoSessionRmax.h"
#include "StereoSessionPinhole.h"
#include "StereoSettings.h"

#define CONFIG_FILE "ba_test.cfg"

const std::string CameraParamsReportFile = "iterCameraParam.txt";
const std::string PointsReportFile = "iterPointsParam.txt";
const int MaxIterations = 20;

using std::cout;
using std::endl;
using std::ios;
using std::setiosflags;
using std::setw;
using std::setprecision;

/*
 * Program Options
 */
/* {{{ ProgramOptions */
enum BundleAdjustmentT { REF, SPARSE, ROBUST_REF, ROBUST_SPARSE };

struct ProgramOptions {
  std::string stereosession_type;
  std::string cnet_file;
  BundleAdjustmentT bundle_adjustment_type;
  double lambda;
  double camera_position_sigma; // constraint on adjustment to camera position
  double camera_pose_sigma;     // constraint on adjustment to camera pose
  double gcp_sigma;           // constraint on adjustment to GCP position
  bool use_user_lambda;
  bool save_iteration_data;
  int min_matches;
  int report_level;
  std::vector<std::string> camera_files;
  friend std::ostream& operator<<(std::ostream& ostr, ProgramOptions o);
};

std::ostream& operator<<(std::ostream& ostr, ProgramOptions o) {
  ostr << endl << "Configured Options" << endl;
  ostr << "----------------------------------------------------" << endl;
  ostr << "Stereo session type: " << o.stereosession_type << endl;
  ostr << "Control network file: " << o.cnet_file << endl;
  ostr << "Bundle adjustment type: ";
  switch (o.bundle_adjustment_type) {
    case REF:
      ostr << "Reference"; break;
    case SPARSE:
      ostr << "Sparse"; break;
    case ROBUST_REF:
      ostr << "Robust Reference"; break;
    case ROBUST_SPARSE:
      ostr << "Robust Sparse"; break;
    default:
      ostr << "unrecognized type";
  }
  ostr << endl;
  if (o.use_user_lambda == true)
    ostr << "Lambda: " << o.lambda << endl;
  ostr << "Camera position sigma: " << o.camera_position_sigma << endl;
  ostr << "Camera pose sigma: " << o.camera_pose_sigma << endl;
  ostr << "Ground control point sigma: " << o.gcp_sigma << endl;
  ostr << "Minimum matches: " << o.min_matches << endl;
  ostr << "Save iteration data? " << o.save_iteration_data << endl;
  ostr << "Report level: " << o.report_level << endl;
  return ostr;
}

BundleAdjustmentT string_to_ba_type(std::string &s) {
  BundleAdjustmentT t = REF;
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  if (s == "sparse") t = SPARSE;
  else if (s == "robust_ref") t = ROBUST_REF;
  else if (s == "robust_sparse") t = ROBUST_SPARSE;
  return t;
}
/* }}} ProgramOptions */

/*
 * Function Definitions
 */
/* {{{ parse_options */
ProgramOptions parse_options(int argc, char* argv[]) {
  std::ifstream config_file (CONFIG_FILE, std::ifstream::in);
  ProgramOptions opts;
  
  // Generic Options (generic_options)
  po::options_description generic_options("Options");
  generic_options.add_options()
    ("help,?", "Display this help message")
    ("verbose,v", "Verbose output")
    ("debug,d", "Debugging output")
    ("print-config","Print configuration options and exit")
    ("report-level,r",po::value<int>(&opts.report_level)->default_value(10),"Changes the detail of the Bundle Adjustment Report");

  std::string ba_type;
  // Bundle adjustment options (ba_options) 
  po::options_description ba_options("Bundle Adjustment Configuration");
  ba_options.add_options()
    ("bundle-adjustment-type,b",
        po::value<std::string>(&ba_type)->default_value("ref"),
        "Select bundle adjustment type (options are: \"ref\", \"sparse\", \"robust_ref\", \"robust_sparse\")")
    ("session-type,t", 
        po::value<std::string>(&opts.stereosession_type)->default_value("pinhole"), 
        "Select the stereo session type to use for processing.")
    ("cnet,c", 
        po::value<std::string>(&opts.cnet_file), 
        "Load a control network from a file")
    ("lambda,l", 
        po::value<double>(&opts.lambda), 
        "Set the initial value of the LM parameter lambda")
    ("camera-position-sigma",
        po::value<double>(&opts.camera_position_sigma)->default_value(1.0),
        "Covariance constraint on camera position")
    ("camera-pose-sigma",
        po::value<double>(&opts.camera_pose_sigma)->default_value(1e-16),
        "Covariance constraint on camera pose")
    ("gcp-sigma",
        po::value<double>(&opts.gcp_sigma)->default_value(1e-16),
        "Covariance constraint on ground control points")
    ("save-iteration-data,s", 
        po::bool_switch(&opts.save_iteration_data), 
        "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt.")
    ("min-matches", 
        po::value<int>(&opts.min_matches)->default_value(30), 
        "Set the minimum  number of matches between images that will be considered.");

  // Hidden options, aka command line arguments (hidden_options)
  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&opts.camera_files));

  // Allowed options (includes generic and ba))
  po::options_description allowed_options("Allowed Options");
  allowed_options.add(generic_options).add(ba_options);

  // Commmand line options
  po::options_description cmdline_options;
  cmdline_options.add(generic_options).add(ba_options).add(hidden_options);

  // Config file options
  po::options_description config_file_options;
  config_file_options.add(ba_options).add(hidden_options);

  // Positional setup for hidden options
  po::positional_options_description p;
  p.add("input-files", -1);
 
  // Parse options on command line and config file 
  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(cmdline_options).positional(p).allow_unregistered().run(), vm );
  po::store(po::parse_config_file(config_file, config_file_options, true), vm);
  po::notify( vm );
  
  if (vm.count("lambda") > 0)
    opts.use_user_lambda = true;
  else
    opts.use_user_lambda = false;

  opts.bundle_adjustment_type = string_to_ba_type(ba_type);

  // Print usage message if requested
  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] " << endl
    << endl << allowed_options << endl;
  if ( vm.count("help") ) {
    cout << usage.str() << endl;
    exit(1);
  }

  // Print config options if requested
  if (vm.count("print-config")) {
    cout << opts << endl;
    exit(0);
  } 

  if ( vm.count("cnet") < 1) {
    cout << "Error: Must specify at least one input file!" << endl << endl;
    cout << usage.str() << endl;
    exit(1);
  }

 vw::vw_log().console_log().rule_set().clear();
  vw::vw_log().console_log().rule_set().add_rule(vw::WarningMessage, "console");
  if (vm.count("verbose"))
    vw::vw_log().console_log().rule_set().add_rule(DebugMessage, "console");
  if (vm.count("debug"))
    vw::vw_log().console_log().rule_set().add_rule(VerboseDebugMessage, "console");

  return opts;
}
/* }}} parse_options */

/* {{{ load_control_network */
boost::shared_ptr<ControlNetwork> load_control_network(std::string const &file) {
  boost::shared_ptr<ControlNetwork> cnet( new ControlNetwork("Control network"));

  vw_out(InfoMessage) << "Loading control network from file: " << file << endl;

  // Deciding which Control Network we have
  std::vector<std::string> tokens;
  boost::split( tokens, file, boost::is_any_of(".") );
  if ( tokens[tokens.size()-1] == "net" ) {
    // An ISIS style control network
    vw_out(DebugMessage) << "\tReading ISIS control network file" << endl;
    cnet->read_isis_pvl_control_network( file );
  } else if ( tokens[tokens.size()-1] == "cnet" ) {
    // A VW binary style
    vw_out(DebugMessage) << "\tReading VisionWorkbench binary control network file" 
      << endl;
    cnet->read_binary_control_network( file );
  } else {
    vw_throw( IOErr() << "Unknown control network file extension, \""
      << tokens[tokens.size()-1] << "\"." );
  }
  return cnet;
}
/* }}} load_control_network */

/* {{{ load_camera_models */
std::vector<boost::shared_ptr<CameraModel> > 
  load_camera_models(std::vector<std::string> const &camera_files, 
  StereoSession *session,
  std::string session_type) 
{
  vw_out(InfoMessage) << "Loading camera models" << endl;
  // TODO: this can be done without the stereo session; just
  // use StereoSessionPinhole::read_file()

  std::vector<boost::shared_ptr<CameraModel> > camera_models;
  std::vector<std::string>::const_iterator iter;

  for (iter = camera_files.begin(); iter != camera_files.end(); ++iter) {
    vw_out(DebugMessage) << "\t" << *iter << endl;

    if (session_type == "pinhole") {
      camera_models.push_back(session->camera_model(*iter, *iter));
    } else {
      camera_models.push_back(session->camera_model(*iter));
    }
  }

  return camera_models;
}
/* }}} load_camera_models */

/* {{{ prefix_from_filename */
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}
/* }}} */

/* {{{ clear_report_files */
void clear_report_files(std::string cam_file, std::string point_file) {
    std::ofstream ostr(cam_file.c_str(),std::ios::out);
    ostr << "";
    ostr.open(point_file.c_str(),std::ios::out);
    ostr << "";
    ostr.close();
}
/* }}} */

/* {{{ BundleAdjustmentModel */
// Bundle adjustment functor
class BundleAdjustmentModel : public camera::BundleAdjustmentModelBase<BundleAdjustmentModel, 6, 3> {

/* {{{ private members */
  typedef Vector<double,6> camera_vector_t;
  typedef Vector<double,3> point_vector_t;

  std::vector<boost::shared_ptr<CameraModel> > m_cameras;
  boost::shared_ptr<ControlNetwork> m_network; 

  std::vector<camera_vector_t> a; // camera parameter adjustments
  std::vector<point_vector_t>  b; // point coordinates
  std::vector<camera_vector_t> a_initial; 
  std::vector<point_vector_t>  b_initial;
  int m_num_pixel_observations;

  double m_camera_position_sigma;
  double m_camera_pose_sigma;
  double m_gcp_sigma;

/* }}} */

public:

/* {{{ constructor */
  BundleAdjustmentModel(std::vector<boost::shared_ptr<CameraModel> > const& cameras,
                        boost::shared_ptr<ControlNetwork> network,
                        double camera_position_sigma, double camera_pose_sigma,
                        double gcp_sigma) : 
    m_cameras(cameras), 
    m_network(network), 
    a(cameras.size()), 
    b(network->size()), 
    a_initial(cameras.size()), 
    b_initial(network->size()),
    m_camera_position_sigma(camera_position_sigma), 
    m_camera_pose_sigma(camera_pose_sigma),
    m_gcp_sigma(gcp_sigma) 
  {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();
    
    // a and a_initial start off with every element all zeros.
    for (unsigned j = 0; j < m_cameras.size(); ++j)
      a_initial[j] = a[j] = camera_vector_t();

    // b and b_initial start off with the initial positions of the 3d points
    for (unsigned i = 0; i < network->size(); ++i)
      b_initial[i] = b[i] = point_vector_t((*m_network)[i].position());
  }
/* }}} */

/* {{{ camera, point and pixel accessors */
  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters(int j) const { return a[j]; }
  camera_vector_t A_initial(int j)    const { return a_initial[j]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) { a[j] = a_j; }

  point_vector_t B_parameters(int i) const { return b[i]; }
  point_vector_t B_initial(int i)    const { return b_initial[i]; }
  void set_B_parameters(int i, point_vector_t const& b_i) { b[i] = b_i; }

  unsigned num_cameras() const { return a.size(); }
  unsigned num_points()  const { return b.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }
/* }}} */

/* {{{ control network accessor */
  // Give access to the control network
  boost::shared_ptr<ControlNetwork> control_network(void) {
    return m_network;
  }
/* }}} */

/* {{{ A and B inverse covariance */
  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double,camera_params_n,camera_params_n> 
  A_inverse_covariance ( unsigned j ) const 
  {
    Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/m_camera_position_sigma; 
    result(1,1) = 1/m_camera_position_sigma;
    result(2,2) = 1/m_camera_position_sigma;
    result(3,3) = 1/m_camera_pose_sigma;
    result(4,4) = 1/m_camera_pose_sigma;
    result(5,5) = 1/m_camera_pose_sigma;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  // NB: only applied to Ground Control Points
  inline Matrix<double,point_params_n,point_params_n> 
  B_inverse_covariance ( unsigned i ) const 
  {
    Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/m_gcp_sigma;
    result(1,1) = 1/m_gcp_sigma; 
    result(2,2) = 1/m_gcp_sigma; 
    return result;
  }
/* }}} */

/* {{{ operator() overload */

  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  Vector2 operator() ( unsigned i, unsigned j, camera_vector_t const& a_j, point_vector_t const& b_i ) const {
    Vector3 position_correction = subvector(a_j,0,3);
    Vector3 p = subvector(a_j,3,3);
    Quaternion<double> pose_correction = vw::math::euler_to_quaternion(p[0], p[1], p[2], "xyz");

    boost::shared_ptr<CameraModel> cam(
        new AdjustedCameraModel(m_cameras[j], position_correction, pose_correction));
    return cam->point_to_pixel(b_i);
  } 
/* }}} */

/* {{{ write_adjustment */
  void write_adjustment(int j, std::string const& filename) const {
    Vector3 position_correction = subvector(a[j],0,3);
    Vector3 p = subvector(a[j],3,3);
    Quaternion<double> pose_correction = vw::math::euler_to_quaternion(p[0], p[1], p[2], "xyz");
    write_adjustments(filename, position_correction, pose_correction);
  }
/* }}} */

/* {{{ adjusted_cameras */
  std::vector<boost::shared_ptr<camera::CameraModel> > adjusted_cameras() const {
    std::vector<boost::shared_ptr<camera::CameraModel> > result(m_cameras.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      Vector3 position_correction = subvector(a[j],0,3);
      Vector3 p = subvector(a[j],3,3);
      Quaternion<double> pose_correction = vw::math::euler_to_quaternion(p[0], p[1], p[2], "xyz");
      result[j] = boost::shared_ptr<camera::CameraModel>( 
          new AdjustedCameraModel( m_cameras[j], position_correction, pose_correction ) );
      // should this be in absolute coordinates?
    }
    return result;
  }
/* }}} */

/* {{{ error calculations */
  // Errors on the image plane
  void image_errors( std::vector<double>& pix_errors ) {
    pix_errors.clear();
    for (unsigned i = 0; i < m_network->size(); ++i)
      for(unsigned m = 0; m < (*m_network)[i].size(); ++m) {
        int camera_idx = (*m_network)[i][m].image_id();
        Vector2 pixel_error = (*m_network)[i][m].position() - (*this)(i, camera_idx, a[camera_idx],b[i]);
        pix_errors.push_back(norm_2(pixel_error));
      }
  }
  
  // Errors for camera position
  void camera_position_errors( std::vector<double>& camera_position_errors ) {
    camera_position_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      Vector3 position_initial = subvector(a_initial[j],0,3);
      Vector3 position_now = subvector(a[j],0,3);
      camera_position_errors.push_back(norm_2(position_initial-position_now));
    }
  }

  // Errors for camera pose
  std::string camera_pose_units(){ return "degrees"; } 
  void camera_pose_errors( std::vector<double>& camera_pose_errors ) {
    camera_pose_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      Vector3 pi = subvector(a_initial[j],3,3);
      Vector3 pn = subvector(a[j],3,3);
      Quaternion<double> pose_initial = vw::math::euler_to_quaternion(pi[0],pi[1],pi[2],"xyz");
      Quaternion<double> pose_now = vw::math::euler_to_quaternion(pn[0],pn[1],pn[2],"xyz");

      Vector3 axis_initial, axis_now;
      double angle_initial, angle_now;
      pose_initial.axis_angle(axis_initial, angle_initial);
      pose_now.axis_angle(axis_now, angle_now);

      camera_pose_errors.push_back(fabs(angle_initial-angle_now) * 180.0/M_PI);
    }
  }

  // Errors for ground control points
  void gcp_errors( std::vector<double>& gcp_errors ) {
    gcp_errors.clear();
    for (unsigned i=0; i < this->num_points(); ++i) {
      if ((*m_network)[i].type() == ControlPoint::GroundControlPoint)
        gcp_errors.push_back(norm_2(b_initial[i] - b[i]));
    }
  }
/* }}} */

/* {{{ write_adjusted_cameras_append */
  void write_adjusted_cameras_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);

    for (unsigned j=0; j < a.size();++j) {
      Vector3 position_correction = subvector(a[j],0,3);
      Vector3 p = subvector(a[j],3,3);
      Quaternion<double> pose_correction = vw::math::euler_to_quaternion(p[0], p[1], p[2],"xyz");

      camera::CAHVORModel cam;
      cam.C = position_correction;
      cam.A = Vector3(1,0,0);
      cam.H = Vector3(0,1,0);
      cam.V = Vector3(0,0,1);
      // = rmax_image_camera_model(m_image_infos[j],position_correction,pose_correction);
      ostr << j << "\t" << cam.C(0) << "\t" << cam.C(1) << "\t" << cam.C(2) << "\n";
      ostr << j << "\t" << cam.A(0) << "\t" << cam.A(1) << "\t" << cam.A(2) << "\n";
      ostr << j << "\t" << cam.H(0) << "\t" << cam.H(1) << "\t" << cam.H(2) << "\n";
      ostr << j << "\t" << cam.V(0) << "\t" << cam.V(1) << "\t" << cam.V(2) << "\n";
      ostr << j << "\t" << cam.O(0) << "\t" << cam.O(1) << "\t" << cam.O(2) << "\n";
      ostr << j << "\t" << cam.R(0) << "\t" << cam.R(1) << "\t" << cam.R(2) << "\n";
    }
  }
};
/* }}} write_adjusted_cameras_append */

/* }}} BundleAdjustmentModel */

int main(int argc, char* argv[]) {
  ProgramOptions config = parse_options(argc, argv);

  boost::shared_ptr<ControlNetwork> cnet = load_control_network(config.cnet_file);
 
  // Read in the camera model and image info for the input images.
  StereoSession* session = StereoSession::create(config.stereosession_type);
  if (config.stereosession_type == "pinhole") {
    stereo_settings().keypoint_alignment = true;
    stereo_settings().epipolar_alignment = false;
  }

  std::vector<boost::shared_ptr<CameraModel> > 
    camera_models = load_camera_models(
        config.camera_files, session, config.stereosession_type);

  BundleAdjustmentModel ba_model(camera_models, cnet, 
      config.camera_position_sigma, config.camera_pose_sigma, config.gcp_sigma);
  BundleAdjustmentSparse<BundleAdjustmentModel, L2Error> bundle_adjuster(ba_model, L2Error());
  //bundle_adjuster = create_bundle_adjuster<BundleAdjustmentSparse<BundleAdjustmentModel, L2Error> >(ba_model);

  // Set lambda if user has requested it
  if (config.use_user_lambda)
    bundle_adjuster.set_lambda(config.lambda);

  //Clearing the monitoring text files to be used for saving camera params
  if (config.save_iteration_data)
    clear_report_files(CameraParamsReportFile, PointsReportFile);

  BundleAdjustReport<BundleAdjustmentSparse<BundleAdjustmentModel, L2Error> > 
      reporter( "Bundle Adjust", ba_model, bundle_adjuster, config.report_level );

  double abs_tol=1e10, rel_tol=1e10;
  while (bundle_adjuster.update(abs_tol, rel_tol)) {
    reporter.loop_tie_in();

    // TODO: Could compute residuals here and then remove outliers and do one
    // more (special) iteration (in robust case) or refit entirely (in
    // non-robust case)
    
    // Writing Current Camera Parameters to file for later reading in MATLAB
    if (config.save_iteration_data) {
      
      //Writing this iterations camera data
      ba_model.write_adjusted_cameras_append(CameraParamsReportFile);
      
      //Writing this iterations point data
      std::ofstream ostr_points(PointsReportFile.c_str(),std::ios::app);

      for (unsigned i = 0; i < ba_model.num_points(); ++i){
        Vector<double, 3> current_point = ba_model.B_parameters(i);
        current_point /= current_point(3);
        ostr_points << i << "\t" << current_point(0) 
                         << "\t" << current_point(1) 
                         << "\t" << current_point(2) << "\n";
      }
    }
    
    if (bundle_adjuster.iterations() > MaxIterations 
        || abs_tol < 0.01 
        || rel_tol < 1e-16)
      break;
  }

  reporter.end_tie_in();

  for (unsigned int i=0; i < ba_model.num_cameras(); ++i)
    ba_model.write_adjustment(i, prefix_from_filename(config.camera_files[i])+".adjust");
  
  // Compute the post-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
  
  return 0;
}
