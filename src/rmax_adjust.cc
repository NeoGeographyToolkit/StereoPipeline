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
#include "RMAX/RmaxInterestPoint.h"
//#include "RMAX/RmaxBundles.h"
#include "BundleAdjustUtils.h"

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

int main(int argc, char* argv[]) {

  std::vector<std::string> image_files;
  std::string cnet_file;
  ControlNetwork cnet("My first control network");
  double lambda;

  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
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

    if (!check_for_ipfiles(image_files))
      exit(0);

    std::cout << "\nLoading Matches:\n";
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
          std::cout << "\t " << i << " <-> " << j << " : " << ip1.size() << " matches.\n";
          add_matched_points(cnet,ip1,ip2,i,j,camera_models);
        }
      }
    }    
    cnet.write_control_network("control.cnet");
  }

  // Print pre-alignment residuals
  //  compute_stereo_residuals(camera_models, bundles);
  compute_stereo_residuals(camera_models, cnet);

  HelicopterBundleAdjustmentModel ba_model(image_infos);
  //  BundleAdjustment<HelicopterBundleAdjustmentModel> bundle_adjuster(ba_model, bundles);
  BundleAdjustment<HelicopterBundleAdjustmentModel> bundle_adjuster(ba_model, cnet);
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

  for (unsigned int i=0; i < ba_model.size(); ++i) {
    std::string base, extension;
    split_filename(image_files[i], base, extension);

    ba_model.write_adjustment(i, base+".rmax_adjust");
    ba_model.write_adjusted_camera(i, base+".rmax_adjust.cahvor");
  }    

  // Write out Adjust CAHVOR models.
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();

  // Compute the pre-adjustment residuals
  //  compute_stereo_residuals(adjusted_cameras, bundles);
  compute_stereo_residuals(adjusted_cameras, cnet);
}
