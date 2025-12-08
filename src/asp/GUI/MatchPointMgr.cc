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

#include <vw/Math/Vector.h>

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


} // End namespace asp
