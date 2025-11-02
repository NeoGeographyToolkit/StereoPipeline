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

#include <asp/GUI/GuiGeom.h>

namespace asp {

void findClosestPolyVertex(// inputs
                           double world_x0, double world_y0,
                           asp::AppData const& app_data,
                           int beg_image_id, int end_image_id,
                           // outputs
                           int & clipIndex,
                           int & polyVecIndex,
                           int & polyIndexInCurrPoly,
                           int & vertIndexInCurrPoly,
                           double & minX, double & minY,
                           double & minDist) {
  clipIndex           = -1;
  polyVecIndex        = -1;
  polyIndexInCurrPoly = -1;
  vertIndexInCurrPoly = -1;
  minX                = world_x0;
  minY                = world_y0;
  minDist             = std::numeric_limits<double>::max();

  vw::Vector2 world_P(world_x0, world_y0);

  for (int clipIter = beg_image_id; clipIter < end_image_id; clipIter++) {

    double minX0, minY0, minDist0;
    int polyVecIndex0, polyIndexInCurrPoly0, vertIndexInCurrPoly0;

    // Convert from world coordinates to given clip coordinates
    vw::Vector2 clip_P = app_data.world2proj(world_P, clipIter);

    findClosestPolyVertex(// inputs
                          clip_P.x(), clip_P.y(),
                          app_data.images[clipIter].polyVec,
                          // outputs
                          polyVecIndex0,
                          polyIndexInCurrPoly0,
                          vertIndexInCurrPoly0,
                          minX0, minY0,
                          minDist0);

    // Unless the polygon is empty, convert back to world
    // coordinates, and see if the current distance is smaller than
    // the previous one.
    if (polyVecIndex0 >= 0 && polyIndexInCurrPoly0 >= 0 &&
        vertIndexInCurrPoly0 >= 0) {

      vw::Vector2 closest_P = app_data.proj2world(vw::Vector2(minX0, minY0),
                                                  clipIter);
      minDist0 = norm_2(closest_P - world_P);

      if (minDist0 <= minDist) {
        clipIndex           = clipIter;
        polyVecIndex        = polyVecIndex0;
        polyIndexInCurrPoly = polyIndexInCurrPoly0;
        vertIndexInCurrPoly = vertIndexInCurrPoly0;
        minDist             = minDist0;
        minX                = closest_P.x();
        minY                = closest_P.y();
      }
    }
  }

  return;
}

// Find the closest point in a given vector of polygons to a given point.
void findClosestPolyVertex(// inputs
                           double x0, double y0,
                           std::vector<vw::geometry::dPoly> const& polyVec,
                           // outputs
                           int & polyVecIndex,
                           int & polyIndexInCurrPoly,
                           int & vertIndexInCurrPoly,
                           double & minX, double & minY,
                           double & minDist) {
  
  polyVecIndex = -1; polyIndexInCurrPoly = -1; vertIndexInCurrPoly = -1;
  minX = x0; minY = y0; minDist = std::numeric_limits<double>::max();
  
  for (int s = 0; s < (int)polyVec.size(); s++){
    
    double minX0, minY0, minDist0;
    int polyIndex, vertIndex;
    polyVec[s].findClosestPolyVertex(// inputs
                                     x0, y0,
                                     // outputs
                                     polyIndex, vertIndex, minX0, minY0, minDist0);
    
    if (minDist0 <= minDist){
      polyVecIndex  = s;
      polyIndexInCurrPoly = polyIndex;
      vertIndexInCurrPoly = vertIndex;
      minDist       = minDist0;
      minX          = minX0;
      minY          = minY0;
    }

  }

  return;
}

}
