// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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
#include <vw/BundleAdjustment.h>
#include <vw/Math.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

#include <stdlib.h>
#include <iostream>

#include <asp/Sessions.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/StereoSettings.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

// Bundle adjustment functor
class BundleAdjustmentModel : public ba::ModelBase<BundleAdjustmentModel, 6, 3> {

  typedef Vector<double,6> camera_vector_t;
  typedef Vector<double,3> point_vector_t;

  std::vector<boost::shared_ptr<CameraModel> > m_cameras;
  boost::shared_ptr<ControlNetwork> m_network;

  std::vector<camera_vector_t> a;
  std::vector<point_vector_t> b;
  std::vector<camera_vector_t> a_target;
  std::vector<point_vector_t> b_target;
  int m_num_pixel_observations;

public:
  BundleAdjustmentModel(std::vector<boost::shared_ptr<CameraModel> > const& cameras,
                        boost::shared_ptr<ControlNetwork> network) :
    m_cameras(cameras), m_network(network), a(cameras.size()),
    b(network->size()), a_target(cameras.size()), b_target(network->size()) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();

    // Set up the a and b vectors, storing the initial values.
    /*
    for (unsigned j = 0; j < m_cameras.size(); ++j) {
      a[j] = camera_vector_t();
      a_target[j] = a[j];
    }
    */

    for (unsigned i = 0; i < network->size(); ++i) {
      b[i] = (*m_network)[i].position();
      b_target[i] = b[i];
    }
  }

  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters(int j) const { return a[j]; }
  point_vector_t B_parameters(int i) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) {
    a[j] = a_j;
  }
  void set_B_parameters(int i, point_vector_t const& b_i) {
    b[i] = b_i;
  }

  // Return the initial parameters
  camera_vector_t A_target(int j) const { return a_target[j]; }
  point_vector_t B_target(int i) const { return b_target[i]; }

  unsigned num_cameras() const { return a.size(); }
  unsigned num_points() const { return b.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double,camera_params_n,camera_params_n>
  A_inverse_covariance ( unsigned /*j*/ ) const {
    Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/100.0;
    result(1,1) = 1/100.0;
    result(2,2) = 1/100.0;
    result(3,3) = 1/1e-1;
    result(4,4) = 1/1e-1;
    result(5,5) = 1/1e-1;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double,point_params_n,point_params_n>
  B_inverse_covariance ( unsigned /*i*/ ) const {
    Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/20;
    result(1,1) = 1/20;
    result(2,2) = 1/20;
    return result;
  }

  void parse_camera_parameters(camera_vector_t a_j,
                               Vector3 &position_correction,
                               Quat &pose_correction) const {
    position_correction = subvector(a_j, 0, 3);
    pose_correction = axis_angle_to_quaternion( subvector(a_j,3,3) );
  }

  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  Vector2 operator() ( unsigned /*i*/, unsigned j,
                       camera_vector_t const& a_j,
                       point_vector_t const& b_i ) const {
    Vector3 position_correction;
    Quat pose_correction;
    parse_camera_parameters(a_j, position_correction, pose_correction);
    AdjustedCameraModel cam(m_cameras[j], position_correction, pose_correction);
    return cam.point_to_pixel(b_i);
  }

  void write_adjustment(int j, std::string const& filename) const {
    Vector3 position_correction;
    Quat pose_correction;
    parse_camera_parameters(a[j], position_correction, pose_correction);
    write_adjustments(filename, position_correction, pose_correction);
  }

  std::vector<boost::shared_ptr<camera::CameraModel> >
  adjusted_cameras() const {
    std::vector<boost::shared_ptr<camera::CameraModel> > result(m_cameras.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      Vector3 position_correction;
      Quat pose_correction;
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
        Vector2 pixel_error;
        try {
          pixel_error = (*m_network)[i][m].position() -
            (*this)(i, camera_idx,a[camera_idx],b[i]);
        } catch ( camera::PixelToRayErr &e ) {}
        pix_errors.push_back(norm_2(pixel_error));
      }
  }

  // Errors for camera position
  void camera_position_errors( std::vector<double>& camera_position_errors ) {
    camera_position_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      Vector3 position_initial, position_now;
      Quat pose_initial, pose_now;

      parse_camera_parameters(a_target[j], position_initial, pose_initial);
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
      Quat pose_initial, pose_now;

      parse_camera_parameters(a_target[j], position_initial, pose_initial);
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
        point_vector_t p1 = b_target[i]/b_target[i](3);
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
      Quat pose_correction;
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

  void bundlevis_cameras_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    for ( unsigned j = 0; j < a.size(); j++ ) {
      Vector3 position_correction;
      Quat pose_correction;
      parse_camera_parameters(a[j], position_correction, pose_correction);
      AdjustedCameraModel cam(m_cameras[j],
                              position_correction, pose_correction);
      Vector3 position = cam.camera_center( Vector2() );
      Quat pose = cam.camera_pose( Vector2() );
      ostr << std::setprecision(18) << j << "\t" << position[0] << "\t"
           << position[1] << "\t" << position[2] << "\t";
      ostr << pose[0] << "\t" << pose[1] << "\t"
           << pose[2] << "\t" << pose[3] << "\n";
    }
  }

  void bundlevis_points_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    unsigned i = 0;
    BOOST_FOREACH( point_vector_t const& p, b ) {
      ostr << i++ << std::setprecision(18) << "\t" << p[0] << "\t"
           << p[1] << "\t" << p[2] << "\n";
    }
  }
};


// This sifts out from a vector of strings, a listing of GCPs.  This
// should be useful for those programs who accept their data in a mass
// input vector.
std::vector<std::string>
sort_out_gcps( std::vector<std::string>& image_files ) {
  std::vector<std::string> gcp_files;
  std::vector<std::string>::iterator it = image_files.begin();
  while ( it != image_files.end() ) {
    if ( boost::iends_with(*it, ".gcp") ){
      gcp_files.push_back( *it );
      it = image_files.erase( it );
    } else
      it++;
  }

  return gcp_files;
}

int main(int argc, char* argv[]) {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif

  // Register all stereo session types
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  std::vector<std::string> image_files, gcp_files;
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

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <image filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch ( po::error &e ) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    std::cout << usage.str() << std::endl;
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
        cnet->read_isis( cnet_file );
      } else if ( tokens[tokens.size()-1] == "cnet" ) {
        // A VW binary style
        cnet->read_binary( cnet_file );
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
  gcp_files = sort_out_gcps( image_files );

  // Read in the camera model and image info for the input images.
  StereoSession* session = StereoSession::create(stereosession_type);

  if (stereosession_type == "pinhole")
    stereo_settings().keypoint_alignment = true;

  std::vector<boost::shared_ptr<CameraModel> > camera_models;
  {
    TerminalProgressCallback progress("asp","Camera Models:");
    progress.report_progress(0);
    double tpc_inc = 1/double(image_files.size());
    BOOST_FOREACH( std::string const& input, image_files ) {
      progress.report_incremental_progress(tpc_inc);
      vw_out(DebugMessage,"asp") << "Loading: " << input << "\n";
      if (stereosession_type == "pinhole")
        camera_models.push_back(session->camera_model(input,input));
      else
        camera_models.push_back(session->camera_model(input));
    }
    progress.report_finished();
  }

  if (!vm.count("cnet") ) {
    build_control_network( (*cnet), camera_models,
                           image_files,
                           min_matches );
    add_ground_control_points( (*cnet),
                               image_files,
                               gcp_files );

    cnet->write_binary("control");
  }

  BundleAdjustmentModel ba_model(camera_models, cnet);
  AdjustSparse<BundleAdjustmentModel, L2Error> bundle_adjuster(ba_model, L2Error(), false, false);

  if (vm.count("lambda"))
    bundle_adjuster.set_lambda(lambda);

  //Clearing the monitoring text files to be used for saving camera params
  if (vm.count("save-iteration-data")){
    fs::remove("iterCameraParam.txt");
    fs::remove("iterPointsParam.txt");

    // Write the starting locations
    ba_model.bundlevis_cameras_append("iterCameraParam.txt");
    ba_model.bundlevis_points_append("iterPointsParam.txt");
  }

  BundleAdjustReport<AdjustSparse<BundleAdjustmentModel, L2Error> >
    reporter( "Bundle Adjust", ba_model, bundle_adjuster, report_level );

  double abs_tol = 1e10, rel_tol=1e10;
  double overall_delta = 2;
  int no_improvement_count = 0;
  while ( true ) {
    // Determine if it is time to quit
    if ( bundle_adjuster.iterations() >= 20 ) {
      reporter() << "Triggered 'Max Iterations'\n";
      break;
    } else if ( abs_tol < 0.01 ) {
      reporter() << "Triggered 'Abs Tol " << abs_tol << " < 0.01'\n";
      break;
    } else if ( rel_tol < 1e-6 ) {
      reporter() << "Triggered 'Rel Tol " << rel_tol << " < 1e-10'\n";
      break;
    } else if ( no_improvement_count > 4 ) {
      reporter() << "Triggered break, unable to improve after "
                 << no_improvement_count << " iterations\n";
      break;
    }

    overall_delta = bundle_adjuster.update( abs_tol, rel_tol );
    reporter.loop_tie_in();

    // Writing Current Camera Parameters to file for later reading
    if (vm.count("save-iteration-data")) {
      ba_model.bundlevis_cameras_append("iterCameraParam.txt");
      ba_model.bundlevis_points_append("iterPointsParam.txt");
    }
    if ( overall_delta == 0 )
      no_improvement_count++;
    else
      no_improvement_count = 0;
  }
  reporter.end_tie_in();

  for (unsigned i=0; i < ba_model.num_cameras(); ++i)
    ba_model.write_adjustment(i, fs::path(image_files[i]).replace_extension("adjust").string() );

  // Compute the post-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
}
