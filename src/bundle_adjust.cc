#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include "boost/filesystem/operations.hpp" 
#include "boost/filesystem/fstream.hpp"    
namespace po = boost::program_options;
namespace fs = boost::filesystem;                   

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjust.h>
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

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif
#include "HRSC/StereoSessionHRSC.h"
#include "MOC/StereoSessionMOC.h"
#include "apollo/StereoSessionApolloMetric.h"
#include "MRO/StereoSessionCTX.h"
#include "RMAX/StereoSessionRmax.h"

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}


// Bundle adjustment functor
class BundleAdjustmentModel : public camera::BundleAdjustmentModelBase<BundleAdjustmentModel, 6, 3> {
  std::vector<boost::shared_ptr<CameraModel> > m_cameras;
  std::vector<Vector<double,6> > m_adjustments;
  
public:
  BundleAdjustmentModel(std::vector<boost::shared_ptr<CameraModel> > const& cameras) : 
    m_cameras(cameras), m_adjustments(cameras.size()) {}

  unsigned size() const { return m_cameras.size(); }

  void update(std::vector<Vector<double, camera_params_n> > a) {
    m_adjustments = a;
  }
   
  void write_adjustment(int j, std::string const& filename) {
    Vector3 position_correction = subvector(m_adjustments[j], 0, 3);
    Vector3 pose_correction = subvector(m_adjustments[j], 3,3);
    write_adjustments(filename, position_correction, pose_correction);
  }

  std::vector<boost::shared_ptr<camera::CameraModel> > adjusted_cameras() {
    std::vector<boost::shared_ptr<camera::CameraModel> > result(m_cameras.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      Vector3 position_correction = subvector(m_adjustments[j], 0, 3);
      Vector3 pose_euler_angles = subvector(m_adjustments[j], 3,3);
      Quaternion<double> pose_correction = math::euler_to_quaternion(pose_euler_angles[0], pose_euler_angles[1], pose_euler_angles[2], "xyz");
      result[j] = boost::shared_ptr<camera::CameraModel>( new AdjustedCameraModel( m_cameras[j], position_correction, pose_correction ) );
    }
    return result;
  }
  
  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  Vector2 operator() ( unsigned i, unsigned j, Vector<double,6> const& a_j, Vector<double,3> const& b_i ) {
    Vector3 position_correction = subvector(a_j, 0, 3);
    Vector3 pose_euler_angles = subvector(a_j, 3,3);
    Quaternion<double> pose_correction = math::euler_to_quaternion(pose_euler_angles[0], pose_euler_angles[1], pose_euler_angles[2], "xyz");
    boost::shared_ptr<CameraModel> cam(new AdjustedCameraModel(m_cameras[j], position_correction, pose_correction));
    return cam->point_to_pixel(b_i);
  }    

  // Return the covariance of the camera parameters for camera j.
  inline Matrix<double,camera_params_n,camera_params_n> A_inverse_covariance ( unsigned j ) {
    Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/400;  // Position sigma = 400 meters
    result(1,1) = 1/400;
    result(2,2) = 1/400;
    result(3,3) = 1/1.0;  // Pose sigma = 1.0 degrees
    result(4,4) = 1/1.0;
    result(5,5) = 1/1.0;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline Matrix<double,point_params_n,point_params_n> B_inverse_covariance ( unsigned i ) {
    Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/1000.0;  // Point sigma = 1000 meters ( we set this to be 
    result(1,1) = 1/1000.0;  // so large that it essentially removes point position
    result(2,2) = 1/1000.0;  // constraints from the bundle adjustment entirely. )
    return result;
  }
  
  Vector<double,6> initial_parameters(unsigned j) const { 
//     struct timeval t;
//     assert(0 == gettimeofday(&t, NULL)); 

//     srandom(t.tv_usec);
//     Vector<double,6> params;
//     double norm1 = 10.0/((pow(2,31))-1);
//     double norm2 = 0.1/((pow(2,31))-1);
//     subvector(params,0,3) = Vector3(random()*norm1,random()*norm1,random()*norm1);
//     subvector(params,3,3) = Vector3(random()*norm2,random()*norm2,random()*norm2);
//     std::cout << "Seeding parameter vector: " << params << "\n";
//     return params;
    return Vector<double,6>(); 
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
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "metric", &StereoSessionApolloMetric::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  std::vector<std::string> image_files;
  std::string cnet_file, stereosession_type;
  ControlNetwork cnet("My first control network");
  double lambda;

  po::options_description general_options("Options");
  general_options.add_options()
    ("session-type,t", po::value<std::string>(&stereosession_type)->default_value("isis"), "Select the stereo session type to use for processing.")
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
    ("nonsparse,n", "Run the non-sparse reference implentation of LM Bundle Adjustment.")
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
      cnet.read_control_network(cnet_file);
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
    std::cout << "\nLoading Image Tie Points:\n";
    for (unsigned i = 0; i < image_files.size(); ++i) {
      for (unsigned j = i; j < image_files.size(); ++j) {
        std::string match_filename = 
        prefix_from_filename(image_files[i]) + "__" +
        prefix_from_filename(image_files[j]) + ".match";

        if ( fs::exists(match_filename) ) {
          // Locate all of the interest points between images that may
          // overlap based on a rough approximation of their bounding box.
          std::vector<InterestPoint> ip1, ip2;
          read_binary_match_file(match_filename, ip1, ip2);
          std::cout << "\t" << match_filename << "     " << i << " <-> " << j << " : " << ip1.size() << " matches.\n";
          add_matched_points(cnet,ip1,ip2,i,j,camera_models);
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

  BundleAdjustmentModel ba_model(camera_models);
  BundleAdjustment<BundleAdjustmentModel> bundle_adjuster(ba_model, cnet);
  if (vm.count("lambda")) {
    std::cout << "Setting initial value of lambda to " << lambda << "\n";
    bundle_adjuster.set_lambda(lambda);
  }

  std::cout << "\nPerforming Sparse LM Bundle Adjustment\n\n";
  double abs_tol = 1e10, rel_tol=1e10;
  if (vm.count("nonsparse")) {
    while(bundle_adjuster.update_reference_impl(abs_tol, rel_tol)) {
      if (bundle_adjuster.iterations() > 50 || abs_tol < 0.01 || rel_tol < 1e-10)
        break;
    }
  } else {
    while(bundle_adjuster.update(abs_tol, rel_tol)) {
      if (bundle_adjuster.iterations() > 50 || abs_tol < 0.01 || rel_tol < 1e-16)
        break;
    }
  }
  std::cout << "\nFinished.  Iterations: "<< bundle_adjuster.iterations() << "\n";

  for (unsigned int i=0; i < ba_model.size(); ++i)
    ba_model.write_adjustment(i, prefix_from_filename(image_files[i])+".adjust");

  // Compute the post-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
  compute_stereo_residuals(adjusted_cameras, cnet);
}
