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

#ifndef __ASP_GUI_GUI_GEOM_H__
#define __ASP_GUI_GUI_GEOM_H__

#include <asp/GUI/AppData.h>
#include <vw/Geometry/dPoly.h>
#include <limits>
#include <vector>

namespace asp {

// Result of searching for the closest polygon vertex or edge
struct PolySearchResult {
  int clipIndex = -1;
  int polyVecIndex = -1;
  int polyIndexInCurrPoly = -1;
  int vertIndexInCurrPoly = -1;
  double minX = 0;
  double minY = 0;
  double minDist = std::numeric_limits<double>::max();
};

void findClosestPolyVertex(double world_x0, double world_y0,
                           asp::AppData const& app_data,
                           int beg_image_id, int end_image_id,
                           PolySearchResult & result);

// Find the closest point in a given vector of polygons to a given point.
void findClosestPolyVertex(double x0, double y0,
                           std::vector<vw::geometry::dPoly> const& polyVec,
                           PolySearchResult & result);

// Find the closest edge in a given set of polygons to a given point.
void findClosestPolyEdge(double world_x0, double world_y0,
                         asp::AppData const& app_data,
                         int beg_image_id, int end_image_id,
                         PolySearchResult & result);

// Merge some polygons and save them in app_data.images[outIndex]
void mergePolys(asp::AppData & app_data, int beg_image_id, int end_image_id, int outIndex);

// Delete vertices in a given box
void deleteVerticesInBox(asp::AppData & app_data, vw::BBox2 const& box,
                         int beg_image_id, int end_image_id);

// Assemble the polygon structure
void formPoly(std::string              const& override_color,
              std::vector<int>         const& contiguous_blocks,
              std::vector<std::string> const& colors,
              std::vector<vw::geometry::anno> const& annotations,
              std::vector<vw::Vector3> const& vertices,
              std::vector<vw::geometry::dPoly> & polyVec);

// Find the closest edge in a given vector of polygons to a given point.
void findClosestPolyEdge(double x0, double y0,
                         std::vector<vw::geometry::dPoly> const& polyVec,
                         PolySearchResult & result);
}
#endif // __ASP_GUI_GUI_GEOM_H__
