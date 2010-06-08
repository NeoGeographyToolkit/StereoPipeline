// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file rmax_adjust.cc
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
#include <vw/Math/LevenbergMarquardt.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

#include <stdlib.h>
#include <iostream>

#include <asp/Sessions/RMAX/RMAX.h>

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

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

class HelicopterBundleAdjustmentModel : public ba::ModelBase<HelicopterBundleAdjustmentModel, 6, 3> {

  typedef Vector<double,6> camera_vector_t;
  typedef Vector<double,3> point_vector_t;

  std::vector<ImageInfo> m_image_infos;

  boost::shared_ptr<ControlNetwork> m_network;

  std::vector<camera_vector_t> a;
  std::vector<point_vector_t> b;
  std::vector<camera_vector_t> a_target;
  std::vector<point_vector_t> b_target;
  int m_num_pixel_observations;

public:

  HelicopterBundleAdjustmentModel(std::vector<ImageInfo> const& image_infos,
                                  boost::shared_ptr<ControlNetwork> network) :
    m_image_infos(image_infos), m_network(network),
    a(image_infos.size()), b(network->size()),
    a_target(image_infos.size()), b_target(network->size()) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*m_network)[i].size();

    // Set up the a and b vectors, storing the initial values.
    for (unsigned j = 0; j < m_image_infos.size(); ++j) {
      a[j] = camera_vector_t();
      a_target[j] = a[j];
    }

    for (unsigned i = 0; i < m_network->size(); ++i) {
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

  // Approximate the jacobian for small variations in the a_j
  // parameters (camera parameters).
  virtual Matrix<double, 2, 6> A_jacobian ( unsigned i, unsigned j,
                                            Vector<double, 6> const& a_j,
                                            Vector<double, 3> const& b_i ) {
    Matrix<double> partial_derivatives = ba::ModelBase<HelicopterBundleAdjustmentModel, 6, 3>::A_jacobian(i,j,a_j,b_i);

    return partial_derivatives;
  }

  // Analytically computed jacobian for variations in the b_i
  // parameters (3d point locations).
  virtual Matrix<double, 2, 3> B_jacobian ( unsigned i, unsigned j,
                                            Vector<double, 6> const& a_j,
                                            Vector<double, 3> const& b_i ) {
    Matrix<double> partial_derivatives = ba::ModelBase<HelicopterBundleAdjustmentModel, 6, 3>::B_jacobian(i,j,a_j,b_i);

    return partial_derivatives;
  }

  // Return the initial parameters
  camera_vector_t A_target(int j) const { return a_target[j]; }
  point_vector_t B_target(int i) const { return b_target[i]; }

  // Return general sizes
  unsigned num_cameras() const { return a.size(); }
  unsigned num_points() const { return b.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  // Return pixel observations
  unsigned num_observations_of_point(const int& i) const { return (*m_network)[i].size(); }
  unsigned corresponding_camera_for_measure(const int& i, const int& m){
    return (*m_network)[i][m].image_id();
  }
  Vector2 pixel_observation_of_point(const int& i, const int& m) const {
    //Finding out which camera accounts for the m_th observations of point i
    unsigned int camera_id = (*m_network)[i][m].image_id();

    Vector<double,6> a_j = a[camera_id];
    Vector3 position_correction = subvector(a_j,0,3);
    Vector3 pose_correction = subvector(a_j,3,3);
    camera::CAHVORModel cam = rmax_image_camera_model(m_image_infos[camera_id],position_correction,pose_correction);

    Vector4 point_estimation = B_parameters(i);
    point_estimation = point_estimation/point_estimation(3);
    return cam.point_to_pixel(subvector(point_estimation,0,3));
  }

  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double,camera_params_n,camera_params_n> A_inverse_covariance ( unsigned /*j*/ ) {
    Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/1;  // Position sigma = 1 meter
    result(1,1) = 1/1;
    result(2,2) = 1/1;
    result(3,3) = 1;  // Pose sigma = 1 degrees
    result(4,4) = 1;
    result(5,5) = 1;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double,point_params_n,point_params_n> B_inverse_covariance ( unsigned /*i*/ ) {
    Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/1000.0;  // Point sigma = 1000 meters ( we set this to be
    result(1,1) = 1/1000.0;  // so large that it essentially removes point position
    result(2,2) = 1/1000.0;  // constraints from the bundle adjustment entirely. )
    return result;
  }

  void write_adjustment(int j, std::string const& filename) {
    std::ofstream ostr(filename.c_str());
    ostr << a[j][0] << " " << a[j][1] << " " << a[j][2] << "\n";
    ostr << a[j][3] << " " << a[j][4] << " " << a[j][5] << "\n";
  }

  void write_adjusted_camera(int j, std::string const& filename) {
    Vector<double,6> a_j = a[j];
    Vector3 position_correction = subvector(a_j, 0, 3);
    Vector3 pose_correction = subvector(a_j, 3,3);
    camera::CAHVORModel cam = rmax_image_camera_model(m_image_infos[j], position_correction, pose_correction);
    cam.write(filename);
  }

  void write_adjusted_cameras_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    for (unsigned j=0; j < a.size();++j){
      Vector<double,6> a_j = a[j];
      Vector3 position_correction = subvector(a_j,0,3);
      Vector3 pose_correction = subvector(a_j,3,3);
      camera::CAHVORModel cam = rmax_image_camera_model(m_image_infos[j],position_correction,pose_correction);
      ostr << j << "\t" << cam.C(0) << "\t" << cam.C(1) << "\t" << cam.C(2) << "\n";
      ostr << j << "\t" << cam.A(0) << "\t" << cam.A(1) << "\t" << cam.A(2) << "\n";
      ostr << j << "\t" << cam.H(0) << "\t" << cam.H(1) << "\t" << cam.H(2) << "\n";
      ostr << j << "\t" << cam.V(0) << "\t" << cam.V(1) << "\t" << cam.V(2) << "\n";
      ostr << j << "\t" << cam.O(0) << "\t" << cam.O(1) << "\t" << cam.O(2) << "\n";
      ostr << j << "\t" << cam.R(0) << "\t" << cam.R(1) << "\t" << cam.R(2) << "\n";
    }
  }

  std::vector<boost::shared_ptr<camera::CameraModel> > adjusted_cameras() {
    std::vector<boost::shared_ptr<camera::CameraModel> > result(a.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      Vector3 position_correction = subvector(a[j], 0, 3);
      Vector3 pose_correction = subvector(a[j], 3,3);

      camera::CAHVORModel *cahvor = new camera::CAHVORModel;
      *cahvor = rmax_image_camera_model(m_image_infos[j], position_correction, pose_correction);

      result[j] = boost::shared_ptr<camera::CameraModel>( cahvor );
    }
    return result;
  }

  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  Vector2 operator() ( unsigned /*i*/, unsigned j, Vector<double,6> const& a_j, Vector<double,3> const& b_i ) const {
    Vector3 position_correction = subvector(a_j, 0, 3);
    Vector3 pose_correction = subvector(a_j, 3,3);
    camera::CAHVORModel cam = rmax_image_camera_model(m_image_infos[j], position_correction, pose_correction);
    return cam.point_to_pixel(b_i);
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
      position_initial = subvector(a_target[j],0,3);
      position_now = subvector(a[j],0,3);

      camera_position_errors.push_back(norm_2(position_initial-position_now));
    }
  }

  // Errors for camera pose
  void camera_pose_errors( std::vector<double>& camera_pose_errors ) {
    camera_pose_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      Vector3 pose_initial, pose_now;
      pose_initial = subvector(a_target[j],3,3);
      pose_now = subvector(a[j],3,3);

      camera_pose_errors.push_back(norm_2(pose_initial-pose_now));
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
};

int main(int argc, char* argv[]) {

  std::vector<std::string> image_files;
  std::vector<std::string> gcp_files;
  std::string cnet_file;
  boost::shared_ptr<ControlNetwork> cnet( new ControlNetwork("My first control network"));
  double lambda;
  int min_matches;
  double robust_outlier_threshold;
  int max_iterations;
  int report_level;

  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
    ("min-matches", po::value<int>(&min_matches)->default_value(5), "Set the mininmum number of matches between images that will be considered.")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0), "Set the threshold for robust cost functions.")
    ("max-iterations", po::value<int>(&max_iterations)->default_value(25), "Set the maximum number of iterations.")
    ("save-iteration-data,s", "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt. Warning: This is slow as pixel observations need to be calculated on each step.")
    ("report-level,r",po::value<int>(&report_level)->default_value(10),"Changes the detail of the Bundle Adjustment Report")
    ("run-match,m", "Run ipmatch to create .match files from overlapping images.")
    ("match-debug-images,d", "Create debug images when you run ipmatch.")
    ("help,h", "Display this help message");

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
  usage << "Usage: " << argv[0] << " [options] <rmax image filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    std::cout << usage << std::endl;
    return 1;
  }

  if( vm.count("input-files") < 1 || image_files.size() < 2) {
    std::cout << "Error: Must specify at least two input files!" << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }
  gcp_files = sort_out_gcps( image_files );

  // Read in the camera model and RMAX image info for the input
  // images.
  std::vector<std::string> camera_files(image_files.size());
  std::vector<ImageInfo> image_infos(image_files.size());
  std::vector<boost::shared_ptr<CameraModel> > camera_models(image_files.size());
  for (unsigned i = 0; i < image_files.size(); ++i) {
    read_image_info( image_files[i], image_infos[i] );
    CAHVORModel *cahvor = new CAHVORModel;
    *cahvor = rmax_image_camera_model(image_infos[i]);
    camera_models[i] = boost::shared_ptr<CameraModel>(cahvor);
  }

  if ( vm.count("cnet") ) {
    // Loading a Control Network

    vw_out() << "Loading control network from file: " << cnet_file << "\n";
    cnet->read_binary(cnet_file);

  } else {
    // Building a Control Network
    build_control_network( *cnet, camera_models,
                           image_files, min_matches );
    add_ground_control_points( *cnet,
                               image_files,
                               gcp_files );

    cnet->write_binary("rmax_adjust");
  }

  HelicopterBundleAdjustmentModel ba_model(image_infos, cnet);
  AdjustSparse<HelicopterBundleAdjustmentModel, L2Error> bundle_adjuster(ba_model, L2Error());

  if (vm.count("lambda"))
    bundle_adjuster.set_lambda(lambda);

  //Clearing the monitoring text files to be used for saving camera params
  if (vm.count("save-iteration-data")){
    std::ofstream ostr("iterCameraParam.txt",std::ios::out);
    ostr << "";
    ostr.close();
    ostr.open("iterPointsParam.txt",std::ios::out);
    ostr << "";
    ostr.close();

    //Now I'm going to save the initial starting position of the cameras
    ba_model.write_adjusted_cameras_append("iterCameraParam.txt");
    std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
    for (unsigned i = 0; i < ba_model.num_points(); ++i){
      Vector<double,3> current_point = ba_model.B_parameters(i);
      ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
    }
  }

  // Reporter
  BundleAdjustReport<AdjustSparse<HelicopterBundleAdjustmentModel, L2Error> >
    reporter( "RMAX Adjust", ba_model, bundle_adjuster, report_level );

  // Performing Bundle Adjustment
  double abs_tol = 1e10, rel_tol=1e10;
  while(bundle_adjuster.update(abs_tol, rel_tol)) {
    reporter.loop_tie_in();

    // Writing Current Camera Parameters to file for later reading in MATLAB
    if (vm.count("save-iteration-data")) {

      //Writing this iterations camera data
      ba_model.write_adjusted_cameras_append("iterCameraParam.txt");

      //Writing this iterations point data, also saving the pixel param data
      std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
      for (unsigned i = 0; i < ba_model.num_points(); ++i){

        Vector<double,3> current_point = ba_model.B_parameters(i);
        ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
      }
    }

    if (bundle_adjuster.iterations() > max_iterations || abs_tol < 0.01 || rel_tol < 1e-10)
      break;
  }
  reporter.end_tie_in();

  for (unsigned int i=0; i < ba_model.num_cameras(); ++i)
    ba_model.write_adjustment(i, prefix_from_filename(image_files[i])+".rmax_adjust");

  // Compute the post-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
}
