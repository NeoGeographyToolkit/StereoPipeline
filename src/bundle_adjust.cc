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
#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera/BundleAdjustReport.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Math/LevenbergMarquardt.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::ip;
using namespace vw::stereo;

#include <stdlib.h>
#include <iostream>

#include "asp_config.h"
#include "StereoSession.h"
#include "BundleAdjustUtils.h"
#include "ControlNetworkLoader.h"

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif
#if defined(ASP_HAVE_PKG_SPICE) && ASP_HAVE_PKG_SPICE == 1
#include "HRSC/StereoSessionHRSC.h"
#include "MOC/StereoSessionMOC.h"
#include "MRO/StereoSessionCTX.h"
#endif
#include "RMAX/StereoSessionRmax.h"

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}


// Bundle adjustment functor
class BundleAdjustmentModel : public camera::BundleAdjustmentModelBase<BundleAdjustmentModel, 7, 4> {

  typedef Vector<double,7> camera_vector_t;
  typedef Vector<double,4> point_vector_t;

  std::vector<boost::shared_ptr<CameraModel> > m_cameras;
  boost::shared_ptr<ControlNetwork> m_network; 

  std::vector<camera_vector_t> a;
  std::vector<point_vector_t> b;
  std::vector<camera_vector_t> a_initial;
  std::vector<point_vector_t> b_initial;
  int m_num_pixel_observations;

  double m_huber_pixel_threshold;

public:
  BundleAdjustmentModel(std::vector<boost::shared_ptr<CameraModel> > const& cameras,
                        boost::shared_ptr<ControlNetwork> network) : 
    m_cameras(cameras), m_network(network), a(cameras.size()), 
    b(network->size()), a_initial(cameras.size()), b_initial(network->size()) {

    m_huber_pixel_threshold = 100; // outlier rejection threshold (pixels)

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();
    
    // Set up the a and b vectors, storing the initial values.
    for (unsigned j = 0; j < m_cameras.size(); ++j) {
      a[j] = camera_vector_t();
      subvector(a[j],3,4) = Vector4(1,0,0,0);
      a_initial[j] = a[j];
    }

    for (unsigned i = 0; i < network->size(); ++i) {
      // We track 3d points as homogeneous vectors
      point_vector_t p;
      p(3) = 1.0;
      subvector(p,0,3) = (*m_network)[i].position();
      b[i] = normalize(p);
      b_initial[i] = b[i];
    }
  }

  double pixel_outlier_threshold() const { return m_huber_pixel_threshold; }

  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters(int j) const { return a[j]; }
  point_vector_t B_parameters(int i) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) { 
    subvector(a[j],0,3) = subvector(a_j,0,3);
    subvector(a[j],3,4) = normalize(subvector(a_j,3,4));
  }
  void set_B_parameters(int i, point_vector_t const& b_i) { 
    b[i] = normalize(b_i); 
  }

  // Return the initial parameters
  camera_vector_t A_initial(int j) const { return a_initial[j]; }
  point_vector_t B_initial(int i) const { return b_initial[i]; }

  unsigned num_cameras() const { return a.size(); }
  unsigned num_points() const { return b.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }
  
  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double,camera_params_n,camera_params_n> A_inverse_covariance ( unsigned j ) const {
    Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/400.0;
    result(1,1) = 1/400.0;
    result(2,2) = 1/400.0;
    result(3,3) = 1/1e-16;
    result(3,3) = 1/1e-16;
    result(3,3) = 1/1e-16;
    result(3,3) = 1/1e-16;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double,point_params_n,point_params_n> B_inverse_covariance ( unsigned i ) const {
    Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/1e-16; 
    result(1,1) = 1/1e-16; 
    result(2,2) = 1/1e-16; 
    result(3,3) = 1/1e-16; 
    return result;
  }

  void parse_camera_parameters(camera_vector_t a_j, 
                               Vector3 &position_correction,
                               Quaternion<double> &pose_correction) const {
    position_correction = subvector(a_j, 0, 3);
    Vector4 q = normalize(subvector(a_j, 3, 4));
    pose_correction = Quaternion<double>(q[0], q[1], q[2], q[3]);
  }

  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  Vector2 operator() ( unsigned i, unsigned j, camera_vector_t const& a_j, point_vector_t const& b_i ) const {
    Vector3 position_correction;
    Quaternion<double> pose_correction;
    parse_camera_parameters(a_j, position_correction, pose_correction);
    point_vector_t p = b_i/b_i(3); // Renormalize
    boost::shared_ptr<CameraModel> cam(new AdjustedCameraModel(m_cameras[j], position_correction, pose_correction));
    return cam->point_to_pixel(subvector(p,0,3));
  }    

  void write_adjustment(int j, std::string const& filename) const {
    Vector3 position_correction;
    Quaternion<double> pose_correction;
    parse_camera_parameters(a[j], position_correction, pose_correction);
    write_adjustments(filename, position_correction, pose_correction);
  }

  std::vector<boost::shared_ptr<camera::CameraModel> > adjusted_cameras() const {
    std::vector<boost::shared_ptr<camera::CameraModel> > result(m_cameras.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      Vector3 position_correction;
      Quaternion<double> pose_correction;
      parse_camera_parameters(a[j], position_correction, pose_correction);
      result[j] = boost::shared_ptr<camera::CameraModel>( new AdjustedCameraModel( m_cameras[j], position_correction, pose_correction ) );
    }
    return result;
  }

  // Errors on the image plane
  void image_errors( std::vector<double>& pix_errors ) {
    pix_errors.clear();
    for (unsigned i = 0; i < m_network->size(); ++i)
      for(unsigned m = 0; m < (*m_network)[i].size(); ++m) {
	int camera_idx = (*m_network)[i][m].image_id();
	Vector2 pixel_error = (*m_network)[i][m].position() - (*this)(i, camera_idx,
								   a[camera_idx],b[i]);
	pix_errors.push_back(norm_2(pixel_error));
      }
  }
  
  // Errors for camera position
  void camera_position_errors( std::vector<double>& camera_position_errors ) {
    camera_position_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      Vector3 position_initial, position_now;
      Quaternion<double> pose_initial, pose_now;

      parse_camera_parameters(a_initial[j], position_initial, pose_initial);
      parse_camera_parameters(a[j], position_now, pose_now);

      camera_position_errors.push_back(norm_2(position_initial-position_now));
    }
  }

  // Errors for camera pose
  std::string camera_pose_units(){ return "degrees"; } 
  void camera_pose_errors( std::vector<double>& camera_pose_errors ) {
    camera_pose_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      Vector3 position_initial, position_now;
      Quaternion<double> pose_initial, pose_now;

      parse_camera_parameters(a_initial[j], position_initial, pose_initial);
      parse_camera_parameters(a[j], position_now, pose_now);

      Vector3 axis_initial, axis_now;
      double angle_initial, angle_now;
      pose_initial.axis_angle(axis_initial, angle_initial);
      pose_now.axis_angle(axis_now, angle_now);

      camera_pose_errors.push_back(fabs(angle_initial-angle_now) * 180.0/M_PI);
    }
  }

  // Errors for gcp errors
  void gcp_errors( std::vector<double>& gcp_errors ) {
    gcp_errors.clear();
    for (unsigned i=0; i < this->num_points(); ++i)
      if ((*m_network)[i].type() == ControlPoint::GroundControlPoint) {
	point_vector_t p1 = b_initial[i]/b_initial[i](3);
	point_vector_t p2 = b[i]/b[i](3);
	gcp_errors.push_back(norm_2(subvector(p1,0,3) - subvector(p2,0,3)));
      }
  }

  // Give access to the control network
  boost::shared_ptr<ControlNetwork> control_network(void) {
    return m_network;
  }

  void write_adjusted_cameras_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);

    for (unsigned j=0; j < a.size();++j){
      Vector3 position_correction;
      Quaternion<double> pose_correction;
      parse_camera_parameters(a[j], position_correction, pose_correction);

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

int main(int argc, char* argv[]) {

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif 

  // Register all stereo session types
#if defined(ASP_HAVE_PKG_SPICE) && ASP_HAVE_PKG_SPICE == 1
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
#endif
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  std::vector<std::string> image_files;
  std::string cnet_file, stereosession_type;
  boost::shared_ptr<ControlNetwork> cnet( new ControlNetwork("My first control network"));
  double lambda;
  double robust_outlier_threshold;
  int report_level;
  int min_matches;

  po::options_description general_options("Options");
  general_options.add_options()
    ("session-type,t", po::value<std::string>(&stereosession_type)->default_value("isis"), "Select the stereo session type to use for processing.")
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0), "Set the threshold for robust cost functions.")
    ("nonsparse,n", "Run the non-sparse reference implentation of LM Bundle Adjustment.")
    ("save-iteration-data,s", "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt.")
    ("min-matches", po::value<int>(&min_matches)->default_value(30), "Set the minimum  number of matches between images that will be considered.")
    ("report-level,r",po::value<int>(&report_level)->default_value(10),"Changes the detail of the Bundle Adjustment Report")
    ("help", "Display this help message")
    ("verbose", "Verbose output");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&image_files));
  
  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);
  
  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <image filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    std::cout << usage << std::endl;
    return 1;
  }
  
  if( vm.count("input-files") < 1) {
    if ( vm.count("cnet") ) {
      std::cout << "Loading control network from file: " << cnet_file << "\n";

      // Deciding which Control Network we have
      std::vector<std::string> tokens;
      boost::split( tokens, cnet_file, boost::is_any_of(".") );
      if ( tokens[tokens.size()-1] == "net" ) {
	// An ISIS style control network
	cnet->read_isis_pvl_control_network( cnet_file );
      } else if ( tokens[tokens.size()-1] == "cnet" ) {
	// A VW binary style
	cnet->read_binary_control_network( cnet_file );
      } else {
	vw_throw( IOErr() << "Unknown Control Network file extension, \""
		  << tokens[tokens.size()-1] << "\"." );
      }
    } else {
      std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
      std::cout << usage.str();
      return 1;
    }
  }  

  // Read in the camera model and image info for the input images.
  StereoSession* session = StereoSession::create(stereosession_type);
  std::vector<boost::shared_ptr<CameraModel> > camera_models(image_files.size());
  std::cout << "Loading Camera Models:\n";
  for (unsigned i = 0; i < image_files.size(); ++i) {
    std::cout << "\t" << image_files[i] << "\n";
    camera_models[i] = session->camera_model(image_files[i]);
  }

  if (!vm.count("cnet") ) {
    build_control_network( cnet, camera_models,
			   image_files, 
			   min_matches );
    add_ground_control_points( cnet,
			       image_files );

    cnet->write_binary_control_network("control");
  }

  BundleAdjustmentModel ba_model(camera_models, cnet);
  //  BundleAdjustment<BundleAdjustmentModel, L1Error> bundle_adjuster(ba_model, cnet, L1Error());
  BundleAdjustment<BundleAdjustmentModel, CauchyError> bundle_adjuster(ba_model, CauchyError(robust_outlier_threshold));
  //  BundleAdjustment<BundleAdjustmentModel, HuberError> bundle_adjuster(ba_model, cnet, HuberError(robust_outlier_threshold));
  //  BundleAdjustment<BundleAdjustmentModel, PseudoHuberError> bundle_adjuster(ba_model, cnet, PseudoHuberError(robust_outlier_threshold));
  if (vm.count("lambda")) {
    bundle_adjuster.set_lambda(lambda);
  }

  //Clearing the monitoring text files to be used for saving camera params
  if (vm.count("save-iteration-data")){
    std::ofstream ostr("iterCameraParam.txt",std::ios::out);
    ostr << "";
    ostr.open("iterPointsParam.txt",std::ios::out);
    ostr << "";
    ostr.close();
  }

  // Reporter
  BundleAdjustReport<BundleAdjustmentModel, BundleAdjustment<BundleAdjustmentModel, CauchyError> > reporter( "Bundle Adjust", ba_model, bundle_adjuster, report_level );
  double abs_tol = 1e10, rel_tol=1e10;
  if (vm.count("nonsparse")) {
    while(bundle_adjuster.update_reference_impl(abs_tol, rel_tol)) {
      reporter.loop_tie_in();

      //Opening the monitoring files, to append this iteration
      std::ofstream ostr_camera("iterCameraParam.txt",std::ios::app);
      std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
      
      //Storing points
      for (unsigned i = 0; i < ba_model.num_points(); ++i){
	Vector<double,4> current_point = ba_model.B_parameters(i);
	current_point /= current_point(3);
	ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
      }
      
      //Storing camera
      for (unsigned j = 0; j < ba_model.num_cameras(); ++j){
	Vector<double,7> current_camera = ba_model.A_parameters(j);
	ostr_camera << j << "\t" << current_camera(0) << "\t" << current_camera(1) << "\t" << current_camera(2) << "\t" << current_camera(3) << "\t" << current_camera(4) << "\t" << current_camera(5) << "\n";
      }
      
      if (bundle_adjuster.iterations() > 20 || abs_tol < 0.01 || rel_tol < 1e-10)
	break;
    }
  } else {
    while(bundle_adjuster.update(abs_tol, rel_tol)) {
      reporter.loop_tie_in();

      // Writing Current Camera Parameters to file for later reading in MATLAB
      if (vm.count("save-iteration-data")) {
        
        //Writing this iterations camera data
        ba_model.write_adjusted_cameras_append("iterCameraParam.txt");
        
        //Writing this iterations point data
        std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
        for (unsigned i = 0; i < ba_model.num_points(); ++i){
          Vector<double,4> current_point = ba_model.B_parameters(i);
          current_point /= current_point(3);
          ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
        }
      }

      if (bundle_adjuster.iterations() > 20 || abs_tol < 0.01 || rel_tol < 1e-16)
        break;
    }
  }
  reporter.end_tie_in();

  for (unsigned int i=0; i < ba_model.num_cameras(); ++i)
    ba_model.write_adjustment(i, prefix_from_filename(image_files[i])+".adjust");

  // Compute the post-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
}
