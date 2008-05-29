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
  
  Vector<double,6> initial_parameters(unsigned j) const { return Vector<double,6>(); }
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
  std::string bundles_file, stereosession_type;
  std::vector<Bundle> bundles;
  double lambda;

  po::options_description general_options("Options");
  general_options.add_options()
    ("session-type,t", po::value<std::string>(&stereosession_type)->default_value("isis"), "Select the stereo session type to use for processing.")
    ("bundles,b", po::value<std::string>(&bundles_file), "Load bundles from file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
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
    if ( vm.count("bundles") ) {
      std::cout << "Loading bundles from file: " << bundles_file << "\n";
      read_bundles_file(bundles_file, bundles, image_files);
    } else {
      std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
      std::cout << usage.str();
      return 1;
    }
  }  

  // Read in the camera model and image info for the input images.
  StereoSession* session = StereoSession::create(stereosession_type);
  std::vector<boost::shared_ptr<CameraModel> > camera_models(image_files.size());
  for (unsigned i = 0; i < image_files.size(); ++i)
    camera_models[i] = session->camera_model(image_files[i]);

  if (!vm.count("bundles") ) {
    std::vector<MatchedPoints> matched_points_list;

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
          std::cout << "Loading " << match_filename << " (" << ip1.size() << " matches) \n";

          // Convert to the MatchedPoints data structure.  (A
          // kludge... should fix this....)
          MatchedPoints mp;  
          mp.id1 = i;  
          mp.id2 = j;
          mp.ip1.resize(ip1.size());
          mp.ip2.resize(ip2.size());
          for (unsigned p = 0; p < ip1.size(); ++p) {
            mp.ip1[p][0] = ip1[p].x;
            mp.ip1[p][1] = ip1[p].y;
            mp.ip2[p][0] = ip2[p].x;
            mp.ip2[p][1] = ip2[p].y;
          }
          matched_points_list.push_back(mp);

        }
      }
    }    

    // Create the bundles
    create_bundles(matched_points_list, camera_models, bundles);
    write_bundles_file(bundles, image_files);
  }

  // Print pre-alignment residuals
  compute_stereo_residuals(camera_models, bundles);

  BundleAdjustmentModel ba_model(camera_models);
  BundleAdjustment<BundleAdjustmentModel> bundle_adjuster(ba_model, bundles);
  if (vm.count("lambda")) {
    std::cout << "Setting initial value of lambda to " << lambda << "\n";
    bundle_adjuster.set_lambda(lambda);
  }

  std::cout << "Performing Sparse LM Bundle Adjustment\n";
  double abs_tol = 1e10, rel_tol=1e10;
  bundle_adjuster.update(abs_tol,rel_tol);
  //bundle_adjuster.update_reference_impl2(abs_tol,rel_tol);
  std::cout << "\n";
  int iterations = 0;
  while(bundle_adjuster.update(abs_tol, rel_tol)) {
    //while(bundle_adjuster.update_reference_impl2(abs_tol, rel_tol)) {
    iterations++;
    if (iterations > 30 || abs_tol < 0.001 || rel_tol < 1e-10)
      break;
  }
  std::cout << "\nFinished.  Iterations: "<< iterations << "\n";

  for (unsigned int i=0; i < ba_model.size(); ++i)
    ba_model.write_adjustment(i, prefix_from_filename(image_files[i])+".adjust");

  // Compute the pre-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
  compute_stereo_residuals(adjusted_cameras, bundles);
}
