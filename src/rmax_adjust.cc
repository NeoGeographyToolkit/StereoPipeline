#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include "boost/filesystem/operations.hpp" 
#include "boost/filesystem/fstream.hpp"    
namespace po = boost::program_options;
namespace fs = boost::filesystem;                   

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera/ControlNetwork.h>
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

#include "RMAX/RMAX.h"
#include "BundleAdjustUtils.h"

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

class HelicopterBundleAdjustmentModel : public camera::BundleAdjustmentModelBase<HelicopterBundleAdjustmentModel, 6, 4> {

  typedef Vector<double,6> camera_vector_t;
  typedef Vector<double,4> point_vector_t;

  std::vector<ImageInfo> m_image_infos;

  ControlNetwork m_network; 

  std::vector<camera_vector_t> a;
  std::vector<point_vector_t> b;
  std::vector<camera_vector_t> a_initial;
  std::vector<point_vector_t> b_initial;
  int m_num_pixel_observations;

public:

  HelicopterBundleAdjustmentModel(std::vector<ImageInfo> const& image_infos,
                                  ControlNetwork const& network) : 
    m_image_infos(image_infos), m_network(network), 
    a(image_infos.size()), b(network.size()),
    a_initial(image_infos.size()), b_initial(network.size()) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network.size(); ++i)
      m_num_pixel_observations += network[i].size();
    
    // Set up the a and b vectors, storing the initial values.
    for (unsigned j = 0; j < m_image_infos.size(); ++j) {
      a[j] = camera_vector_t();
      a_initial[j] = a[j];
    }

    for (unsigned i = 0; i < network.size(); ++i) {
      // We track 3d points as homogeneous vectors
      point_vector_t p;
      p(3) = 1.0;
      subvector(p,0,3) = m_network[i].position();
      b[i] = normalize(p);
      b_initial[i] = b[i];
    }
  }

  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters(int j) const { return a[j]; }
  point_vector_t B_parameters(int i) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) { 
    a[j] = a_j;
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
  inline Matrix<double,camera_params_n,camera_params_n> A_inverse_covariance ( unsigned j ) {
    Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/4e-1;  // Position sigma = 0.4 meters
    result(1,1) = 1/4e-1;
    result(2,2) = 1/4e-1;
    result(3,3) = 1/1;  // Pose sigma = 1.0 degrees
    result(4,4) = 1/1;
    result(5,5) = 1/1;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double,point_params_n,point_params_n> B_inverse_covariance ( unsigned i ) {
    Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/1000.0;  // Point sigma = 1000 meters ( we set this to be 
    result(1,1) = 1/1000.0;  // so large that it essentially removes point position
    result(2,2) = 1/1000.0;  // constraints from the bundle adjustment entirely. )
    result(3,3) = 1;  // constraints from the bundle adjustment entirely. )
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
      
      camera::CAHVORModel* cahvor = new camera::CAHVORModel;
      *cahvor = rmax_image_camera_model(m_image_infos[j], position_correction, pose_correction);

      result[j] = boost::shared_ptr<camera::CameraModel>( cahvor );
    }
    return result;
  }
  
  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  Vector2 operator() ( unsigned i, unsigned j, Vector<double,6> const& a_j, Vector<double,4> const& b_i ) const {
    Vector3 position_correction = subvector(a_j, 0, 3);
    Vector3 pose_correction = subvector(a_j, 3,3);
    camera::CAHVORModel cam = rmax_image_camera_model(m_image_infos[j], position_correction, pose_correction);
    point_vector_t p = b_i/b_i(3); // Renormalize
    return cam.point_to_pixel(subvector(p,0,3));
  }    

  void report_error() const {

    double pix_error_total = 0;
    double camera_position_error_total = 0;
    double camera_pose_error_total = 0;
    double gcp_error_total = 0;

    int idx = 0;
    for (unsigned i = 0; i < m_network.size(); ++i) {       // Iterate over control points
      for (unsigned m = 0; m < m_network[i].size(); ++m) {  // Iterate over control measures
        int camera_idx = m_network[i][m].image_id();
        Vector2 pixel_error = m_network[i][m].position() - (*this)(i, camera_idx, 
                                                                   a[camera_idx],b[i]); 
        pix_error_total += norm_2(pixel_error);
        ++idx;
      }
    }

    for (unsigned j=0; j < this->num_cameras(); ++j) {
      Vector3 position_initial, position_now;
      Vector3 pose_initial, pose_now;

      position_initial = subvector(a_initial[j],0,3);
      pose_initial = subvector(a_initial[j],3,3);
      position_now = subvector(a[j],0,3);
      pose_now = subvector(a[j],3,3);

      camera_position_error_total += norm_2(position_initial-position_now);
      camera_pose_error_total += norm_2(pose_initial-pose_now);
    }
    
    idx = 0;
    for (unsigned i=0; i < this->num_points(); ++i) {
      if (m_network[i].type() == ControlPoint::GroundControlPoint) {
        point_vector_t p1 = b_initial[i]/b_initial[i](3);
        point_vector_t p2 = b[i]/b[i](3);
        gcp_error_total += norm_2(subvector(p1,0,3) - subvector(p2,0,3));
        ++idx;
      }
    }
    
    std::cout << "   Pixel: " << pix_error_total/m_num_pixel_observations << "  "
              << "   Cam Position: " << camera_position_error_total/a.size() << "  "
              << "   Cam Pose: " << camera_pose_error_total/a.size() << "  ";
      if (m_network.num_ground_control_points() == 0) 
        std::cout << "  GCP: n/a\n";
      else 
        std::cout << "  GCP: " << gcp_error_total/m_network.num_ground_control_points() << "\n";

  }

};

int main(int argc, char* argv[]) {

  std::vector<std::string> image_files;
  std::string cnet_file;
  ControlNetwork cnet("My first control network");
  double lambda;
  int min_matches;
  double robust_outlier_threshold;
  int max_iterations;

  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
    ("min-matches", po::value<int>(&min_matches)->default_value(5), "Set the mininmum number of matches between images that will be considered.")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0), "Set the threshold for robust cost functions.")
    ("max-iterations", po::value<int>(&max_iterations)->default_value(25.0), "Set the maximum number of iterations.")
    ("nonsparse,n", "Run the non-sparse reference implentation of LM Bundle Adjustment.")
    ("save-iteration-data,s", "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt.")
    ("run-match,m", "Run ipmatch to create .match files from overlapping images.")
    ("match-debug-images,d", "Create debug images when you run ipmatch.")
    ("help", "Display this help message")
    ("verbose,v", "Verbose output");

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
  
  if( vm.count("input-files") < 1) {
    if ( vm.count("cnet") ) {
      std::cout << "Loading control network from file: " << cnet_file << "\n";
      cnet.read_control_network(cnet_file);
    } else {
      std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
      std::cout << usage.str();
      return 1;
    }
  }  
  
  // Read in the camera model and RMAX image info for the input
  // images.
  std::vector<std::string> camera_files(image_files.size());
  std::vector<ImageInfo> image_infos(image_files.size());
  std::vector<boost::shared_ptr<CameraModel> > camera_models(image_files.size());
  for (unsigned i = 0; i < image_files.size(); ++i) {
    read_image_info( image_files[i], image_infos[i] );
    CAHVORModel* cahvor = new CAHVORModel;
    *cahvor = rmax_image_camera_model(image_infos[i]);
    camera_models[i] = boost::shared_ptr<CameraModel>(cahvor);
  }
  
  if (!vm.count("cnet") ) {
    
    std::cout << "\nLoading Matches:\n";
    for (unsigned i = 0; i < image_files.size(); ++i) {
      for (unsigned j = i+1; j < image_files.size(); ++j) {
        std::string match_filename = 
          prefix_from_filename(image_files[i]) + "__" +
          prefix_from_filename(image_files[j]) + ".match";

        if ( vm.count("run-match")) {
          if (may_overlap(image_files[i], image_files[j])) {
            std::ostringstream cmd;
            cmd << "ipmatch " 
                << image_files[i] << " " 
                << image_files[j] << " ";
            if (vm.count("match-debug-images"))
              cmd << "-d";
            system(cmd.str().c_str());
          }
        }

        if ( fs::exists(match_filename) ) {
          // Locate all of the interest points between images that may
          // overlap based on a rough approximation of their bounding box.
          std::vector<InterestPoint> ip1, ip2;
          read_binary_match_file(match_filename, ip1, ip2);
          if (int(ip1.size()) < min_matches) {
            std::cout << "\t" << match_filename << "     " << i << " <-> " << j << " : " << ip1.size() << " matches.  [rejected]\n";
          } else {
            std::cout << "\t" << match_filename << "     " << i << " <-> " << j << " : " << ip1.size() << " matches.\n";
            add_matched_points(cnet,ip1,ip2,i,j,camera_models);
          }
        }
      }
    }    

    std::cout << "\nLoading Ground Control Points:\n";
    for (unsigned i = 0; i < image_files.size(); ++i) {
      std::string gcp_filename = prefix_from_filename(image_files[i]) + ".gcp";
      if ( fs::exists(gcp_filename) ) {
        int numpoints = add_ground_control_points(cnet, gcp_filename, i); 
        std::cout << "\t" << gcp_filename << "     " << " : " << numpoints << " GCPs.\n";
      }
    }

    cnet.write_control_network("control.cnet");
  }

  // Print pre-alignment residuals
  compute_stereo_residuals(camera_models, cnet);

  HelicopterBundleAdjustmentModel ba_model(image_infos, cnet);
  std::cout << "\nPerforming LM Bundle Adjustment.  Starting error:\n";
  ba_model.report_error();
  std::cout << "\n";

  //  BundleAdjustment<HelicopterBundleAdjustmentModel, L1Error> bundle_adjuster(ba_model, cnet, L1Error());
  BundleAdjustment<HelicopterBundleAdjustmentModel, CauchyError> bundle_adjuster(ba_model, cnet, CauchyError(robust_outlier_threshold));
  //  BundleAdjustment<HelicopterBundleAdjustmentModel, HuberError> bundle_adjuster(ba_model, cnet, HuberError(robust_outlier_threshold));
  //  BundleAdjustment<HelicopterBundleAdjustmentModel, PseudoHuberError> bundle_adjuster(ba_model, cnet, PseudoHuberError(robust_outlier_threshold));
  if (vm.count("lambda")) {
    std::cout << "Setting initial value of lambda to " << lambda << "\n";
    bundle_adjuster.set_lambda(lambda);
  }

  //Clearing the monitoring text files to be used for saving camera params
  if (vm.count("save-iteration-data")){
    std::ofstream ostr("iterCameraParam.txt",std::ios::out);
    ostr << "";
    ostr.close();
    ostr.open("iterPointsParam.txt",std::ios::out);
    ostr << "";
    ostr.close();
  }

  double abs_tol = 1e10, rel_tol=1e10;
  
  if (vm.count("nonsparse")) {
    while(bundle_adjuster.update_reference_impl(abs_tol, rel_tol)) {

      // Writing Current Camera Parameters to file for later reading in MATLAB
      if (vm.count("save-iteration-data")){

        //Writing this iterations camera data
        ba_model.write_adjusted_cameras_append("iterCameraParam.txt");
        
        //Writing this iterations point data
        std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
        for (unsigned i = 0; i < ba_model.num_points(); ++i){
          Vector<double,4> current_point = ba_model.B_parameters(i);
          current_point /= current_point(3);
          ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
        }
	
// 	//Opening the monitoring files, to append this iteration
// 	std::ofstream ostr_camera("iterCameraParam.txt",std::ios::app);
// 	std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);

// 	//Storing points
// 	for (unsigned i = 0; i < ba_model.num_points(); ++i){
// 	  Vector<double,3> current_point = ba_model.B_parameters(i);
// 	  ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
// 	}

// 	//Storing camera
// 	for (unsigned j = 0; j < ba_model.num_cameras(); ++j){
// 	  Vector<double,6> current_camera = ba_model.A_parameters(j);
// 	  ostr_camera << j << "\t" << current_camera(0) << "\t" << current_camera(1) << "\t" << current_camera(2) << "\t" << current_camera(3) << "\t" << current_camera(4) << "\t" << current_camera(5) << "\n";
// 	}
      }
      if (bundle_adjuster.iterations() > max_iterations || abs_tol < 0.01 || rel_tol < 1e-10)
        break;
    }
  } else {
    while(bundle_adjuster.update(abs_tol, rel_tol)) {
      
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
      
      if (bundle_adjuster.iterations() > max_iterations || abs_tol < 0.01 || rel_tol < 1e-10)
        break;
    }
  }
  std::cout << "\nFinished.  Iterations: "<< bundle_adjuster.iterations() << "\n";
  
  for (unsigned int i=0; i < ba_model.num_cameras(); ++i)
    ba_model.write_adjustment(i, prefix_from_filename(image_files[i])+".rmax_adjust");
  
  // Compute the post-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
  compute_stereo_residuals(adjusted_cameras, cnet);
}
