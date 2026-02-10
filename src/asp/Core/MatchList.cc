// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#include <string>
#include <vector>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <asp/Core/MatchList.h>

using namespace vw;

namespace asp {

void MatchList::throwIfNoPoint(size_t image, size_t point) const {
  if ((image >= m_matches.size()) || (point >= m_matches[image].size()))
    vw_throw(ArgumentErr() << "IP " << image << ", " << point << " does not exist!\n");
}

void MatchList::resize(size_t num_images) {
  m_matches.resize(num_images);
  m_valid_matches.resize(num_images);
}

bool MatchList::addPoint(size_t image, vw::ip::InterestPoint const &pt, bool valid) {

  if (image >= m_matches.size())
    return false;
  
  // We will start with an interest point in the left-most image,
  // and add matches to it in the other images.
  // At any time, an image to the left must have no fewer ip than
  // images on the right. Upon saving, all images must
  // have the same number of interest points.
  size_t curr_pts = m_matches[image].size(); // # Pts from current image
  bool is_good = true;
  for (size_t i = 0; i < image; i++) { // Look through lower-id images
    if (m_matches[i].size() < curr_pts+1) {
      is_good = false;
    }
  }
  // Check all higher-id images, they should have the same # Pts as this one.
  for (size_t i = image+1; i < m_matches.size(); i++) {
    if (m_matches[i].size() > curr_pts) {
      is_good = false;
    }
  }

  if (!is_good)
    return false;

  m_matches[image].push_back(pt);
  m_valid_matches[image].push_back(true);
  return true;
}

size_t MatchList::getNumImages() const {
  return m_matches.size();
}

size_t MatchList::getNumPoints(size_t image) const {
  if (m_matches.empty())
    return 0;
  return m_matches[image].size();
}

vw::ip::InterestPoint const& MatchList::getPoint(size_t image, size_t point) const {
  throwIfNoPoint(image, point);
  return m_matches[image][point];
}

vw::Vector2 MatchList::getPointCoord(size_t image, size_t point) const {
  throwIfNoPoint(image, point);
  return vw::Vector2(m_matches[image][point].x, m_matches[image][point].y);
}

bool MatchList::pointExists(size_t image, size_t point) const {
  return ((image < m_matches.size()) && (point < m_matches[image].size()));
}

bool MatchList::isPointValid(size_t image, size_t point) const {
  throwIfNoPoint(image, point);
  return m_valid_matches[image][point];
}

void MatchList::setPointValid(size_t image, size_t point, bool newValue) {
  throwIfNoPoint(image, point);
  m_valid_matches[image][point] = newValue;
}

void MatchList::setPointPosition(size_t image, size_t point, float x, float y) {
  throwIfNoPoint(image, point);
  m_matches[image][point].x = x;
  m_matches[image][point].y = y;
}

int MatchList::findNearestMatchPoint(size_t image, vw::Vector2 P, double distLimit) const {
  if (image >= m_matches.size())
    return -1;

  double min_dist  = std::numeric_limits<double>::max();
  if (distLimit > 0)
    min_dist = distLimit;
  int    min_index = -1;
  std::vector<vw::ip::InterestPoint> const& ip = m_matches[image]; // alias
  for (size_t ip_iter = 0; ip_iter < ip.size(); ip_iter++) {
    Vector2 Q(ip[ip_iter].x, ip[ip_iter].y);
    double curr_dist = norm_2(Q-P);
    if (curr_dist < min_dist) {
      min_dist  = curr_dist;
      min_index = ip_iter;
    }
  }
  return min_index;
}

void MatchList::deletePointsForImage(size_t image) {
  if (image >= m_matches.size())
    vw_throw(ArgumentErr() << "Image " << image << " does not exist!\n");

  m_matches.erase      (m_matches.begin()       + image);
  m_valid_matches.erase(m_valid_matches.begin() + image);
}

bool MatchList::deletePointAcrossImages(size_t point) {

  // Sanity checks
  if (point >= getNumPoints())
    vw::vw_throw(ArgumentErr() << "Requested point for deletion does not exist!");
  
  for (size_t i = 0; i < m_matches.size(); i++) {
    if (m_matches[0].size() != m_matches[i].size()) {
      vw_throw(vw::ArgumentErr()
               << "Cannot delete matches. Must have the same number of "
               << "matches in each image.\n");
    }
  }

  for (size_t vec_iter = 0; vec_iter < m_matches.size(); vec_iter++) {
    m_matches[vec_iter].erase(m_matches[vec_iter].begin() + point);
    m_valid_matches[vec_iter].erase(m_valid_matches[vec_iter].begin() + point);
  }
  return true;
}

bool MatchList::allPointsValid() const {
  if (m_valid_matches.size() != m_matches.size())
    vw_throw(LogicErr() << "Valid matches out of sync with matches!\n");
  
  for (size_t i = 0; i < m_matches.size(); i++) {
    if (m_matches[0].size() != m_matches[i].size())
      return false;
    for (size_t j = 0; j < m_valid_matches[i].size(); j++) {
      if (!m_valid_matches[i][j])
        return false;
    }
  }
  
  return true;
}

bool MatchList::loadPointsFromGCPs(std::string const gcpPath,
                                   std::vector<std::string> const& imageNames) {
  using namespace vw::ba;
  if (getNumPoints() > 0) // Can't double-load points!
    return false;

  // Reset and resize
  *this = MatchList();
  int num_images = imageNames.size();
  this->resize(num_images);

  ControlNetwork cnet("gcp");
  cnet.get_image_list() = imageNames;
  std::vector<std::string> gcp_files;
  gcp_files.push_back(gcpPath);
  // The actual datum does not matter here as the GCP will not be edited or processed
  bool skip_datum_check = true;
  vw::cartography::Datum datum; 
  try {
    add_ground_control_points(cnet, gcp_files, datum, skip_datum_check);
  } catch (...) {
    // Do not complain if the GCP file does not exist. Maybe we want to create it.
    return true;
  }
  
  CameraRelationNetwork<JFeature> crn;
  crn.from_cnet(cnet);

  typedef CameraNode<JFeature>::iterator crn_iter;
  if (crn.size() != num_images && crn.size() != 0)
    vw_throw(ArgumentErr()
             << "The number of images in the control network does not "
             << "agree with the number of images to view.\n");

  // Load in all of the points
  for (size_t icam = 0; icam < crn.size(); icam++) {
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {
      Vector2 observation = (**fiter).m_location;
      vw::ip::InterestPoint ip(observation.x(), observation.y());
      m_matches[icam].push_back(ip);
      m_valid_matches[icam].push_back(true);
    }
  }

  // If any of the sizes do not match, reset everything!
  for (size_t icam = 0; icam < crn.size(); icam++) {
    if (m_matches[0].size() != m_matches[icam].size()) {
      *this = MatchList(); // reset
      resize(num_images); // use the correct size
      vw::vw_throw(ArgumentErr()
               << "Each GCP must be represented as a pixel in each image.\n");
    }
  }

  return true;
}

bool MatchList::loadPointsFromVwip(std::vector<std::string> const& vwipFiles,
                                   std::vector<std::string> const& imageNames) {

  using namespace vw::ba;

  if (getNumPoints() > 0) // Can't double-load points!
    return false;

  // Reset and resize
  *this = MatchList(); 
  size_t num_images = imageNames.size();
  this->resize(num_images);

  // Load in all of the points
  for (size_t i = 0; i < num_images; ++i) {
    //std::vector<InterestPoint> ip;
    m_matches[i] = vw::ip::read_binary_ip_file(vwipFiles[i]);
    // Keep the valid matches synced up
    size_t num_pts = m_matches[i].size();
    m_valid_matches[i].resize(num_pts);
    for (size_t j=0; j<num_pts; ++j)
       m_valid_matches[i][j] = true;
  }
  
  return true;
}

void MatchList::setIpValid(size_t image) {
  if (image >= getNumImages())
    return;
  const size_t num_ip = m_matches[image].size();
  
  m_valid_matches[image].resize(num_ip);
  for (size_t i=0; i<num_ip; ++i)
    m_valid_matches[image][i] = true;
}

// Populate the match files and leftIndices vectors
void populateMatchFiles(std::vector<std::string> const& image_files,
                        std::string const& output_prefix,
                        std::string const& first_match_file,
                        bool matches_as_txt,
                        // Outputs
                        std::vector<std::string> & matchFiles,
                        std::vector<size_t> & leftIndices,
                        bool & matchfiles_found) {

  int num_images = image_files.size();

  // Populate the outputs
  matchFiles.resize(num_images-1);
  leftIndices.resize(num_images-1);
  matchfiles_found = true; // if we fail, will be set to false
  
  std::string trial_match = "";
  int leftIndex = 0;
  std::vector<vw::ip::InterestPoint> left, right; // local variables
  for (size_t i = 1; i < image_files.size(); i++) {

    // Handle user-provided match file for two images
    if ((first_match_file != "") && (image_files.size() == 2)) {
      matchFiles [0] = first_match_file;
      leftIndices[0] = 0;
      break;
    }

    // Look for the match file in the default location, and if it
    // does not appear prompt the user or a path.

    // Look in default location 1, match from previous file to this file.
    try {
      trial_match = vw::ip::match_filename(output_prefix, image_files[i-1],
                                           image_files[i], matches_as_txt);
      leftIndex = i - 1;
      vw::ip::read_match_file(trial_match, left, right, matches_as_txt);
      vw::vw_out() << "Read " << left.size() << " matches from " << trial_match << "\n";

    } catch(...) {
      // Look in default location 2, match from first file to this file.
      try {
        trial_match = vw::ip::match_filename(output_prefix, image_files[0],
                                             image_files[i], matches_as_txt);
        leftIndex = 0;
        vw::ip::read_match_file(trial_match, left, right, matches_as_txt);
        vw::vw_out() << "Read " << left.size() << " matches from " << trial_match << "\n";

      } catch(...) {
        // Default locations failed, Start with a blank match file.
        trial_match = vw::ip::match_filename(output_prefix, image_files[i-1],
                                             image_files[i], matches_as_txt);
        matchfiles_found = false;
        leftIndex = i-1;
      }
    }
    
    matchFiles [i-1] = trial_match;
    leftIndices[i-1] = leftIndex;
  } // End loop looking for match files
} 

// Populate from two vectors of interest point matches
void MatchList::populateFromIpPair(std::vector<vw::ip::InterestPoint> const& ip1,
                                   std::vector<vw::ip::InterestPoint> const& ip2) {
  if (ip1.size() != ip2.size())
    vw_throw(ArgumentErr() << "The two interest point vectors must have the same size.\n");

  // Reset and resize
  *this = MatchList();
  resize(2);

  m_matches[0] = ip1;
  m_matches[1] = ip2;
  setIpValid(0);
  setIpValid(1);
}

bool MatchList::loadPointsFromMatchFiles(std::vector<std::string> const& matchFiles,
                                         std::vector<size_t> const& leftIndices,
                                         bool matches_as_txt) {

  // Count IP as in the same location if x and y are at least this close.
  const float ALLOWED_POS_DIFF = 0.5;
  
  // Can't double-load points!
  if ((getNumPoints() > 0) || (matchFiles.empty()))
    return false;

  const size_t num_images = matchFiles.size() + 1;
  // Make sure we have the right number of match files
  if ((matchFiles.size() != leftIndices.size()))
    return false;

  resize(0); // wipe first
  resize(num_images);

  // Loop through all of the matches
  size_t num_ip = 0;
  for (size_t i = 1; i < num_images; i++) {
    std::string match_file = matchFiles [i-1];
    size_t      j         = leftIndices[i-1];
    
    // Initialize all matches as invalid
    m_matches      [i].resize(num_ip);
    m_valid_matches[i].resize(num_ip);
    for (size_t v = 0; v < num_ip; v++) {
      m_matches      [i][v].x = v*10;  // TODO: Better way to spread these IP?
      m_matches      [i][v].y = v*10;
      m_valid_matches[i][v] = false;
    }

    std::vector<vw::ip::InterestPoint> left, right;
    try {
      vw_out() << "Reading match file: " << match_file << "\n";
      ip::read_match_file(match_file, left, right, matches_as_txt);
      vw::vw_out() << "Read " << left.size() << " matches.\n";
    } catch(...) {
      vw_out() << "IP load failed, leaving default invalid IP\n";
      continue;
    }
    if (i == 1) { // The first case is easy
      m_matches[0] = left;
      m_matches[1] = right;
      setIpValid(0);
      setIpValid(1);
      num_ip = left.size(); // The first image sets the number of IP
      continue;
    }

    // For other cases, we need to isolate the same IP in the left image.
    // Loop through the ip in the "left" image
    size_t count = 0;
    for (size_t pnew=0; pnew<left.size(); ++pnew) {

      // Look through the ip we already have for that image
      //  and see if any of them are at the same location
      for (size_t pold=0; pold<num_ip; ++pold) {

        float dx = fabs(left[pnew].x - m_matches[j][pold].x);
        float dy = fabs(left[pnew].y - m_matches[j][pold].y);
        if ((dx < ALLOWED_POS_DIFF) && (dy < ALLOWED_POS_DIFF))
        {
          // If we found a match, record it and move on to the next point.
          // - Note that we match left[] but we record right[]
          m_matches      [i][pold] = right[pnew];
          m_valid_matches[i][pold] = true;
          ++count;
          break;
        }
      } // End loop through m_matches[j]
      
      if (count == num_ip)
        break; // This means we matched all of the IP in the existing image!
      
    }  // End loop through left
    // Any points that did not match are left with their original value.
  }
  return true;
}

bool MatchList::savePointsToDisk(std::string const& prefix,
                                 std::vector<std::string> const& imageNames,
                                 std::string const& match_file,
                                 bool matches_as_txt) const {
  if (!allPointsValid() || (imageNames.size() != m_matches.size()))
    vw::vw_throw(vw::ArgumentErr()
                 << "Cannot write match files, not all points are valid.\n");

  const size_t num_image_files = imageNames.size();

  bool success = true;
  for (size_t i = 0; i < num_image_files; i++) {

    // Save both i to j matches and j to i matches if there are more than two images.
    // This is useful for SfS, though it is a bit of a hack.
    size_t beg = i + 1;
    if (num_image_files > 2) 
      beg = 0;

    for (size_t j = beg; j < num_image_files; j++) {

      if (i == j)
        continue; // don't save i <-> i matches

      std::string output_path = vw::ip::match_filename(prefix,
                                                       imageNames[i], imageNames[j],
                                                       matches_as_txt);
      if ((num_image_files == 2) && (match_file != ""))
        output_path = match_file;
      try {
        vw_out() << "Writing: " << output_path << "\n";
        ip::write_match_file(output_path, m_matches[i], m_matches[j], matches_as_txt);
      } catch(...) {
        vw::vw_throw(vw::ArgumentErr() << "Failed to save match file: "
                     << output_path << ".\n");
      }
    }
  }
  return success;
}

} // namespace asp
