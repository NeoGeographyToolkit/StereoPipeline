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

#ifndef __ASP_GUI_MATCH_POINT_MGR_H__
#define __ASP_GUI_MATCH_POINT_MGR_H__

// ASP
#include <asp/Core/MatchList.h>
#include <asp/GUI/GuiUtilities.h>

// Vision Workbench
#include <vw/InterestPoint/InterestPoint.h> // For pairwiseMatchList
#include <vw/BundleAdjustment/ControlNetwork.h> // For ControlNetwork

// Standard
#include <string>
#include <vector>
#include <memory>

class QPainter;


namespace asp {

// Forward declarations
class AppData;
class MainWidget;

// Structures to keep track of all interest point matches.
class MatchPointMgr {
public:
  MatchPointMgr(asp::AppData & app_data): // Constructor now takes AppData
    m_matches_exist(false),
    m_cnet("ASP_control_network"),
    m_editMatchPointVecIndex(-1),
    m_editingMatches(false),
    m_app_data(app_data) {} // Initialize m_app_data

  void init(int num_images) {
    m_matchlist.resize(num_images);
  }

  void drawInterestPoints(QPainter* paint,
                          MainWidget* main_widget, // New argument
                          int beg_image_id, int base_image_id,
                          int window_width, int window_height);

  void loadPairwiseMatches(int left_index, int right_index,
                           std::string const& output_prefix);

  // Member variables
  bool     m_matches_exist;
  asp::MatchList    m_matchlist;
  pairwiseMatchList m_pairwiseMatches;
  pairwiseMatchList m_pairwiseCleanMatches;
  vw::ba::ControlNetwork m_cnet;
  int m_editMatchPointVecIndex; // Point being edited
  bool m_editingMatches;        // If we are in the middle of editing match points
  asp::AppData & m_app_data;   // Reference to AppData

}; // End class MatchPointMgr

} // End namespace asp

#endif // __ASP_GUI_MATCH_POINT_MGR_H__