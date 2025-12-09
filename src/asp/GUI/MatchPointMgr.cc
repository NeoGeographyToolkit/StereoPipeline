// __BEGIN_LICENSE__
//  Copyright (c) 2006-2024, United States Government as represented by the
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

#include <asp/GUI/MatchPointMgr.h>
#include <asp/GUI/MainWidget.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/IpMatchingAlgs.h>

#include <vw/Math/Vector.h>
#include <vw/InterestPoint/MatcherIO.h>

#include <QtGui>
#include <QPainter>

using namespace vw;

namespace asp {

void MatchPointMgr::drawInterestPoints(QPainter* paint,
                                       MainWidget* main_widget,
                                       int beg_image_id,
                                       int base_image_id,
                                       int window_width,
                                       int window_height) {

  // Highlight colors for various actions
  QColor ipColor              = QColor(255,  0,  0); // Red
  QColor ipInvalidColor       = QColor(255,163, 26); // Orange
  QColor ipAddHighlightColor  = QColor(64,255,  0); // Green
  QColor ipMoveHighlightColor = QColor(255,  0,255); // Magenta

  paint->setBrush(Qt::NoBrush);

  // If this point is currently being edited by the user, highlight it.
  // - Here we check to see if it has not been placed in all images yet.
  bool highlight_last = false;
  if (asp::stereo_settings().view_matches) {
    int lastImage = int(m_matchlist.getNumImages()) - 1;
    highlight_last
      = (m_matchlist.getNumPoints(beg_image_id) > m_matchlist.getNumPoints(lastImage));
  }

  std::vector<Vector2> ip_vec;
  if (asp::stereo_settings().view_matches) {
    for (size_t ip_iter = 0; ip_iter < m_matchlist.getNumPoints(beg_image_id); ip_iter++) {
      // Generate the pixel coord of the point
      Vector2 pt = m_matchlist.getPointCoord(beg_image_id, ip_iter);
      ip_vec.push_back(pt);
    }
  } else if (asp::stereo_settings().pairwise_matches &&
              beg_image_id < m_pairwiseMatches.ip_to_show.size()) {
    // Had to check if ip_to_show was initialized by now
    auto & ip_in_vec = m_pairwiseMatches.ip_to_show[beg_image_id]; // alias
    for (size_t ip_iter = 0; ip_iter < ip_in_vec.size(); ip_iter++) {
      ip_vec.push_back(Vector2(ip_in_vec[ip_iter].x, ip_in_vec[ip_iter].y));
    }
  }
  else if (asp::stereo_settings().pairwise_clean_matches &&
              beg_image_id < m_pairwiseCleanMatches.ip_to_show.size()) {
    // Had to check if ip_to_show was initialized by now
    auto & ip_in_vec = m_pairwiseCleanMatches.ip_to_show[beg_image_id]; // alias
    for (size_t ip_iter = 0; ip_iter < ip_in_vec.size(); ip_iter++) {
      ip_vec.push_back(Vector2(ip_in_vec[ip_iter].x, ip_in_vec[ip_iter].y));
    }
  }

  // Iterate over interest points
  for (size_t ip_iter = 0; ip_iter < ip_vec.size(); ip_iter++) {
    // Generate the pixel coord of the point
    Vector2 pt    = ip_vec[ip_iter];
    Vector2 world = m_app_data.image2world_trans(pt, base_image_id); // Use m_app_data
    Vector2 P     = main_widget->world2screen(world); // Use main_widget->world2screen

    // Do not draw points that are outside the viewing area
    if (P.x() < 0 || P.x() > window_width ||
        P.y() < 0 || P.y() > window_height) {
      continue;
    }

    paint->setPen(ipColor); // The default IP color
    paint->setBrush(ipColor); // make the point filled

    if (asp::stereo_settings().view_matches) {
      // Some special handling for when we add matches
      if (!m_matchlist.isPointValid(beg_image_id, ip_iter)) {
        paint->setPen(ipInvalidColor);
        paint->setBrush(ipInvalidColor);
      }

      // Highlighting the last point
      if (highlight_last && (ip_iter == m_matchlist.getNumPoints(beg_image_id)-1)) {
        paint->setPen(ipAddHighlightColor);
        paint->setBrush(ipAddHighlightColor);
      }

      if (static_cast<int>(ip_iter) == m_editMatchPointVecIndex) {
        paint->setPen(ipMoveHighlightColor);
        paint->setBrush(ipMoveHighlightColor);
      }
    }

    QPoint Q(P.x(), P.y());
    paint->drawEllipse(Q, 2, 2); // Draw the point

  } // End loop through points
} // End function drawInterestPoints

void MatchPointMgr::loadPairwiseMatches(int left_index, int right_index,
                                        std::string const& output_prefix) {

  pairwiseMatchList * pairwiseMatches = NULL;
  std::string match_file;
  auto index_pair = std::make_pair(left_index, right_index);
  
  if (asp::stereo_settings().pairwise_matches) {
    
    pairwiseMatches = &m_pairwiseMatches;
    if (!asp::stereo_settings().nvm.empty() ||
        !asp::stereo_settings().isis_cnet.empty()) {
      // Load from nvm or isis cnet
      match_file = "placeholder.match";
      pairwiseMatches->match_files[index_pair] = match_file; // flag it as loaded

      // Where we want these loaded 
      auto & left_ip = pairwiseMatches->matches[index_pair].first;   // alias
      auto & right_ip = pairwiseMatches->matches[index_pair].second; // alias

      if (left_ip.empty() && right_ip.empty())
        asp::matchesForPair(m_cnet, left_index, right_index, left_ip, right_ip);
      
    } else if (pairwiseMatches->match_files.find(index_pair) ==
               pairwiseMatches->match_files.end()) {
      // Load pairwise matches
      match_file = vw::ip::match_filename(output_prefix,
                                          m_app_data.images[left_index].name,
                                          m_app_data.images[right_index].name);
    }
  } else {
    // Load pairwise clean matches
    pairwiseMatches = &m_pairwiseCleanMatches;
    if (pairwiseMatches->match_files.find(index_pair) == 
        pairwiseMatches->match_files.end()) {
      match_file = vw::ip::clean_match_filename(output_prefix,
                                                m_app_data.images[left_index].name,
                                                m_app_data.images[right_index].name);
    }
  }
  
  // Ensure the ip per image are always empty but initialized. This will ensure that
  // later in MainWidget::viewMatches() we plot the intended matches.
  if (pairwiseMatches) {
    pairwiseMatches->ip_to_show.clear();
    pairwiseMatches->ip_to_show.resize(m_app_data.images.size());
  }
  
  // Where we want these loaded 
  auto & left_ip = pairwiseMatches->matches[index_pair].first; // alias
  auto & right_ip = pairwiseMatches->matches[index_pair].second; // alias
  
  // If the file was not loaded before, load it. Note that matches from an nvm file
  // are loaded by now.
  if (pairwiseMatches->match_files.find(index_pair) == pairwiseMatches->match_files.end()) {
    // Flag it as loaded
    pairwiseMatches->match_files[index_pair] = match_file;
    try {
      // Load it
      vw_out() << "Loading match file: " << match_file << std::endl;
      vw::ip::read_binary_match_file(match_file, left_ip, right_ip);
      vw_out() << "Read: " << left_ip.size() << " matches.\n";
    } catch(...)
 {
      // Having this pop-up for a large number of images is annoying
      vw_out() << "Cannot find the match file with given images and output prefix.\n";
      return;
    }
  }
  
  // These will be read when interest points are drawn
  pairwiseMatches->ip_to_show[left_index] = left_ip;
  pairwiseMatches->ip_to_show[right_index] = right_ip;
}

} // End namespace asp