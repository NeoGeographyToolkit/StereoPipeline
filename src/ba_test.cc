#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

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

#include "StereoSession.h"
#include "BundleAdjustUtils.h"
#include "ControlNetworkLoader.h"


#define CONFIG_FILE "ba_test.cfg"

int main(int argc, char* argv[]) {
  std::vector<std::string> image_files, gcp_files;
  std::string cnet_file, stereosession_type;
  double epsilon, sigma_1, sigma_2, lambda, robust_outlier_threshold;
  int min_tiepoints, min_matches, num_cameras;
  std::ifstream config_file (CONFIG_FILE, std::ifstream::in);
  boost::shared_ptr<ControlNetwork> cnet( new ControlNetwork("My first control network"));

  // Options groups

  // Generic Options
  po::options_description generic_opts("Options");
  generic_opts.add_options()
    ("help,?", "Display this help message")
    ("verbose,v", "Verbose output");

  // Test Configuration Options
  po::options_description test_opts("Test Configuration");
  test_opts.add_options()
    ("epsilon", po::value<double>(&epsilon), "epsilon value")
    ("sigma-1", po::value<double>(&sigma_1), "sigma_1 value")
    ("sigma-2", po::value<double>(&sigma_2), "sigma_2 value")
    ("min-tiepoints-per-image", po::value<int>(&min_tiepoints),"") // is the the same as min-matches?
    ("number-of-cameras", po::value<int>(&num_cameras),"");

  // Bundle Adjustment options
  po::options_description ba_opts("Bundle Adjustment Configuration");
  ba_opts.add_options()
    ("session-type,t", po::value<std::string>(&stereosession_type)->default_value("isis"), 
        "Select the stereo session type to use for processing.")
    ("cnet,c", po::value<std::string>(&cnet_file), 
        "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), 
        "Set the initial value of the LM parameter lambda")
    ("robust-threshold", po::value<double>(&robust_outlier_threshold)->default_value(10.0), 
        "Set the threshold for robust cost functions")
    ("nonsparse,n", "Run the non-sparse reference implentation of LM Bundle Adjustment.")
    ("save-iteration-data,s", 
        "Saves all camera information between iterations to iterCameraParam.txt, it also saves point locations for all iterations in iterPointsParam.txt.")
    ("min-matches", po::value<int>(&min_matches)->default_value(30), 
        "Set the minimum  number of matches between images that will be considered.");

  // Hidden Options (command line arguments)
  po::options_description hidden_opts("");
  hidden_opts.add_options()
    ("input-files", po::value<std::vector<std::string> >(&image_files));

  // positional options spec
  po::positional_options_description p;
  p.add("input-files", -1); // copied from BA -- will change
  
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
  usage << "Usage: " << argv[0] << " [options] [bundle adjustment options] " 
      << "<image filenames>..." << std::endl << std::endl;
  usage << allowed_opts << std::endl;
  if ( vm.count("help") ) {
    std::cout << usage.str() << std::endl;
    return 1;
  }

  if( vm.count("input-files") < 1) {
    if ( vm.count("cnet") ) {
    } else {
      std::cout << "Error: Must specify at least one input file!" 
          << std::endl << std::endl;
      std::cout << usage.str();
      return 1;
    }
  }


  return 0;
}



