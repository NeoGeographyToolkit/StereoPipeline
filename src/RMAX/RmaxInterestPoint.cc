#include <RMAX/RMAX.h>
#include <RMAX/RmaxInterestPoint.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>

#include "boost/filesystem.hpp"   
#include <iostream>               
using namespace boost::filesystem; 
using namespace vw;
using namespace vw::ip;

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

/// Erases a file suffix if one exists and returns the base string
static std::string suffix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(0, index);
  return result;
}

// Returns the name of the binary file containing the extracted
// interest points.
bool check_for_ipfiles(std::vector<std::string> > files) {

  for (unsigned i = 0; i < files.size; ++i) {
    
    keyfile = prefix_from_filename(files[i]) + ".vwip";
    if ( !exists( keyfile ) ) {
      vw_out(ErrorMessage) << "Missing interest point file: " << keyfile << ".  Regenerate it using the ipfind utility.\n";
      return false;
    }
  }
  return true;
}


// Duplicate matches for any given interest point probably indicate a
// poor match, so we cull those out here.
void remove_duplicates(std::vector<Vector3> &ip1, std::vector<Vector3> &ip2) {
  std::vector<Vector3> new_ip1, new_ip2;
  
  for (unsigned i = 0; i < ip1.size(); ++i) {
    bool bad_entry = false;
    for (unsigned j = 0; j < ip1.size(); ++j) {
      if (i != j && 
          (ip1[i] == ip1[j] || ip2[i] == ip2[j])) {
        bad_entry = true;
      }
    }
    if (!bad_entry) {
      new_ip1.push_back(ip1[i]);
      new_ip2.push_back(ip2[i]);
    }
  }
  
  ip1 = new_ip1;
  ip2 = new_ip2;
}


void match_interest_points(std::string image_filename1, std::string image_filename2, 
                           std::vector<Vector2> &final_ip1, std::vector<Vector2> &final_ip2) {
 
  std::string ip_filename1 = prefix_from_filename(image_filename1) + ".vwip";
  std::string ip_filename2 = prefix_from_filename(image_filename2) + ".vwip";

  // The basic interest point matcher does not impose any
  // constraints on the matched interest points.
  double matcher_threshold = 0.8;

  // RANSAC needs the matches as a vector, and so does the matcher.
  // this is messy, but for now we simply make a copy.
  std::vector<InterestPoint> ip1, ip2;
  ip1 = read_key_file(ip_filename1);
  ip2 = read_key_file(ip_filename2);

  std::cout << "Matching " << ip_filename1 << " and " << ip_filename2 << "... " << std::flush;
  InterestPointMatcher<L2NormMetric,NullConstraint> matcher(matcher_threshold);
  std::vector<InterestPoint> matched_ip1, matched_ip2;
  matcher(ip1, ip2, matched_ip1, matched_ip2, false);
  vw_out(InfoMessage) << "found " << matched_ip1.size() << " putative matches.... " << std::flush;
  
  std::vector<Vector3> ransac_ip1(matched_ip1.size());
  std::vector<Vector3> ransac_ip2(matched_ip2.size());
  for (unsigned i = 0; i < matched_ip1.size();++i ) {
    ransac_ip1[i] = Vector3(matched_ip1[i].x, matched_ip1[i].y,1);
    ransac_ip2[i] = Vector3(matched_ip2[i].x, matched_ip2[i].y,1);
  }

  remove_duplicates(ransac_ip1, ransac_ip2);
  
  // RANSAC is used to fit a similarity transform between the
  // matched sets of points  
  RandomSampleConsensus<math::SimilarityFittingFunctor, InterestPointErrorMetric> ransac( vw::math::SimilarityFittingFunctor(),
                                                                                          InterestPointErrorMetric(), 
                                                                                          10 ); // inlier_threshold
  std::vector<Vector3> result_ip1;
  std::vector<Vector3> result_ip2;
  try {
    Matrix<double> H = ransac(ransac_ip1,ransac_ip2);
    ransac.inliers(H,ransac_ip1,ransac_ip2,result_ip1, result_ip2);
  } catch (vw::ip::RANSACErr &e) {
    // Do nothing...
  }

  final_ip1.clear();
  final_ip2.clear();
  final_ip1.resize(result_ip1.size());
  final_ip2.resize(result_ip2.size());
  for (unsigned i = 0; i < result_ip1.size();++i ) {
    final_ip1[i](0) = result_ip1[i](0);
    final_ip1[i](1) = result_ip1[i](1);
    final_ip2[i](0) = result_ip2[i](0);
    final_ip2[i](1) = result_ip2[i](1);
  }
  
  vw_out(InfoMessage) << final_ip1.size() << " final matches.\n";
}
