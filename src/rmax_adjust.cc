/************************************************************************
 *     File: rmaxcahvor.cc
 *     Date: May 2007
 *       By: Todd Templeton
 *      For: NASA Ames Research Center, Intelligent Mechanisms Group 
 * Function: Extract CAHVOR camera model from RMAX images	
 *          				
 ************************************************************************/

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjust.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::ip;
using namespace vw::stereo;

#include <stdlib.h>
#include <iostream>

#include "RMAX/RMAX.h"

// Split filename into base and extension.
int split_filename(const std::string& filename, std::string& base, std::string& extension) {
  std::string::size_type dot = filename.find_last_of('.');
  if(dot == std::string::npos || dot <= 0)
    return -1;
  extension = filename.substr(dot);
  boost::to_lower(extension);
  base = filename.substr(0, dot);
  return 0;
}

void detect_interest_points(std::string filename1, std::string filename2, 
                            std::vector<Vector2> &final_ip1, std::vector<Vector2> &final_ip2) {

  DiskImageView<PixelGray<uint8> > left_disk_image (filename1);
  DiskImageView<PixelGray<uint8> > right_disk_image (filename2);

  vw_out(InfoMessage) << "\nInterest Point Detection:\n";

  // Interest Point module detector code.
  ScaledInterestPointDetector<LogInterestOperator> detector;
  InterestPointList ip1 = detect_interest_points(channels_to_planes(left_disk_image), detector);
  InterestPointList ip2 = detect_interest_points(channels_to_planes(right_disk_image), detector);
  vw_out(InfoMessage) << "Left image: " << ip1.size() << " points.  Right image: " << ip2.size() << "\n"; 

  // Generate descriptors for interest points.
  // TODO: Switch to a more sophisticated descriptor
  vw_out(InfoMessage) << "\tGenerating descriptors... " << std::flush;
  PatchDescriptorGenerator descriptor;
  descriptor(left_disk_image, ip1);
  descriptor(right_disk_image, ip2);
  vw_out(InfoMessage) << "done.\n";
 
  // The basic interest point matcher does not impose any
  // constraints on the matched interest points.
  vw_out(InfoMessage) << "\nInterest Point Matching:\n";
  double matcher_threshold = 0.8;

  // RANSAC needs the matches as a vector, and so does the matcher.
  // this is messy, but for now we simply make a copy.
  std::vector<InterestPoint> ip1_copy(ip1.size()), ip2_copy(ip2.size());
  std::copy(ip1.begin(), ip1.end(), ip1_copy.begin());
  std::copy(ip2.begin(), ip2.end(), ip2_copy.begin());

  InterestPointMatcher<L2NormMetric,NullConstraint> matcher(matcher_threshold);
  std::vector<InterestPoint> matched_ip1, matched_ip2;
  matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2, false, TerminalProgressCallback());
  vw_out(InfoMessage) << "Found " << matched_ip1.size() << " putative matches.\n";
  
  std::vector<Vector2> ransac_ip1(matched_ip1.size());
  std::vector<Vector2> ransac_ip2(matched_ip2.size());
  for (unsigned i = 0; i < matched_ip1.size();++i ) {
    ransac_ip1[i] = Vector2(matched_ip1[i].x, matched_ip1[i].y);
    ransac_ip2[i] = Vector2(matched_ip2[i].x, matched_ip2[i].y);
  }

  // RANSAC is used to fit a similarity transform between the
  // matched sets of points  
  vw_out(InfoMessage) << "\nRunning RANSAC..." << std::flush;
  RandomSampleConsensus<math::SimilarityFittingFunctor, InterestPointErrorMetric> ransac( vw::math::SimilarityFittingFunctor(),
                                                                                          InterestPointErrorMetric(), 
                                                                                          50 ); // inlier_threshold
  Matrix<double> H = ransac(ransac_ip1,ransac_ip2);

  ransac.inliers(H,ransac_ip1,ransac_ip2,final_ip1, final_ip2);
  vw_out(InfoMessage) << " done.\n";
}

int main(int argc, char* argv[]) {

  std::vector<std::string> image_files;

  po::options_description general_options("Options");
  general_options.add_options()
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
  
  if( vm.count("input-files") < 1 ) {
    std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }
    
  if( vm.count("verbose") ) {
    vw::set_debug_level(vw::VerboseDebugMessage);
  }

  // Read in the camera models for the selected images.
  std::vector<boost::shared_ptr<CameraModel> > camera_models(image_files.size());
  for (unsigned i = 0; i < image_files.size(); ++i) {
    CAHVORModel* cahvor = new CAHVORModel;
    *cahvor = rmax_image_camera_model(image_files[i]);
    
    // Write CAHVOR camera model to file.
    std::string base, extension;
    split_filename(image_files[i], base, extension);
    std::string camfile = base + ".cahvor";
    cahvor->write(camfile);

    camera_models[i] = boost::shared_ptr<CameraModel>(cahvor);
  }

  // Locate matching interest points
  std::vector<std::pair<std::vector<Vector2>, std::vector<Vector2> > > matched_points_list(image_files.size()-1);
  for (unsigned i = 0; i < matched_points_list.size(); ++i) {
    detect_interest_points(image_files[i], image_files[i+1], matched_points_list[i].first, matched_points_list[i].second);
  }

  std::cout << "\nMatching Results:\n";
  for (unsigned i = 0; i < matched_points_list.size(); ++i) {
    std::cout << "\tImage Pair " << i << " -- " << matched_points_list[i].first.size() << " matches. (" << matched_points_list[i].second.size()<< ")\n";
  }

  // Compute pre-adjustment residuals and convert to bundles
  typedef BundleAdjustment<CameraBundleAdjustmentModel> bundle_adjust_t;
  std::cout << "\nPre-adjustment residuals:\n";
  std::vector<bundle_adjust_t::Bundle> bundles;
  for (unsigned i = 0; i < matched_points_list.size(); ++i) {
    StereoModel sm(*(camera_models[i]), *(camera_models[i+1]));
    double error_sum = 0; 
    double min_error = ScalarTypeLimits<double>::highest();
    double max_error = ScalarTypeLimits<double>::lowest();
    for (unsigned j = 0; j < matched_points_list[i].first.size(); ++j) {
      bundle_adjust_t::Bundle b;
      double error;
      b.position = sm(matched_points_list[i].first[j],matched_points_list[i].second[j],error);
      b.imaged_pixel_locations.push_back(std::pair<uint32, Vector2>(i,matched_points_list[i].first[j]));
      b.imaged_pixel_locations.push_back(std::pair<uint32, Vector2>(i+1,matched_points_list[i].second[j]));
      bundles.push_back(b);
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
    std::cout << "\tImage Pair " << i << " -- Min: " << min_error << "  Max: " << max_error << "  Average: " << (error_sum/matched_points_list[i].first.size()) << "\n";
  }  

  //   std::cout << "Bundles: \n";
  //   for (unsigned i = 0; i < bundles.size(); ++i) {
  //     std::cout << "\tPosition: " << bundles[i].position << "  ";
  //     for (unsigned j = 0; j < bundles[i].imaged_pixel_locations.size(); ++j) {
  //       std::cout << "[ " << bundles[i].imaged_pixel_locations[j].first << " " << bundles[i].imaged_pixel_locations[j].second << " ]  ";
  //     }    
  //     std::cout << "\n";
  //   }

  bundle_adjust_t bundle_adjuster(camera_models, bundles);
  bundle_adjuster.update();

}
