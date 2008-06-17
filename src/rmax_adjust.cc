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

int main(int argc, char* argv[]) {

  std::vector<std::string> image_files;
  std::string cnet_file;
  ControlNetwork cnet("My first control network");
  double lambda;
  int min_matches;

  po::options_description general_options("Options");
  general_options.add_options()
    ("cnet,c", po::value<std::string>(&cnet_file), "Load a control network from a file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
    ("min-matches", po::value<int>(&min_matches)->default_value(5), "Set the mininmum number of matches between images that will be considered.")
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

  HelicopterBundleAdjustmentModel ba_model(image_infos);
  BundleAdjustment<HelicopterBundleAdjustmentModel> bundle_adjuster(ba_model, cnet);
  if (vm.count("lambda")) {
    std::cout << "Setting initial value of lambda to " << lambda << "\n";
    bundle_adjuster.set_lambda(lambda);
  }

  //Something to remind the user in case they forgot what they were doing
  if (vm.count("nonsparse")){
    std::cout << "\nPerforming Non-Sparse LM Bundle Adjustment\n\n";
  }else{
    std::cout << "\nPerforming Sparse LM Bundle Adjustment\n\n";
  }
  
  //Clearing the monitoring text files to be used for saving camera params
  if (vm.count("saveiter")){
    std::ofstream ostr("iterCameraParam.txt",std::ios::out);
    ostr << "";
    ostr.open("iterPointsParam.txt",std::ios::out);
    ostr << "";
    ostr.close();
    delete ostr;
  }

  double abs_tol = 1e10, rel_tol=1e10;
  
  if (vm.count("nonsparse")) {
    while(bundle_adjuster.update_reference_impl(abs_tol, rel_tol)) {

      // Writing Current Camera Parameters to file for later reading in MATLAB
      if (vm.count("save-iteration-data")){
	
	//Opening the monitoring files, to append this iteration
	std::ofstream ostr_camera("iterCameraParam.txt",std::ios::app);
	std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);

	//Storing points
	for (unsigned i = 0; i < bundle_adjuster.num_points(); ++i){
	  Vector<double,3> current_point = bundle_adjuster.get_point(i);
	  ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
	}

	//Storing camera
	for (unsigned j = 0; j < bundle_adjuster.num_cameras(); ++j){
	  Vector<double,6> current_camera = bundle_adjuster.get_camera(j);
	  ostr_camera << j << "\t" << current_camera(0) << "\t" << current_camera(1) << "\t" << current_camera(2) << "\t" << current_camera(3) << "\t" << current_camera(4) << "\t" << current_camera(5) << "\n";
	}
      }

      if (bundle_adjuster.iterations() > 20 || abs_tol < 0.01 || rel_tol < 1e-10)
        break;
    }
  } else {
    while(bundle_adjuster.update(abs_tol, rel_tol)) {

      // Writing Current Camera Parameters to file for later reading in MATLAB
      if (vm.count("save-iteration-data")){
	
        //Writing this iterations camera data
	ba_model.write_adjusted_cameras_append("iterCameraParam.txt");
	
	//Writing this iterations point data
	std::ofstream ostr_points("iterPointsParam.txt",std::ios::app);
	for (unsigned i = 0; i < bundle_adjuster.num_points(); ++i){
	  Vector<double,3> current_point = bundle_adjuster.get_point(i);
	  ostr_points << i << "\t" << current_point(0) << "\t" << current_point(1) << "\t" << current_point(2) << "\n";
	}

      }

      if (bundle_adjuster.iterations() > 20 || abs_tol < 0.01 || rel_tol < 1e-10)
        break;
    }
  }
  std::cout << "\nFinished.  Iterations: "<< bundle_adjuster.iterations() << "\n";

  for (unsigned int i=0; i < ba_model.size(); ++i)
    ba_model.write_adjustment(i, prefix_from_filename(image_files[i])+".rmax_adjust");

  // Compute the post-adjustment residuals
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();
  compute_stereo_residuals(adjusted_cameras, cnet);
}
