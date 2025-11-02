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
#include <vw/Core/Log.h>
#include <vw/Geometry/geomUtils.h>
#include <vw/Cartography/shapeFile.h>
#include <ogrsf_frmts.h>

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

void findClosestPolyEdge(// inputs
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

    asp::findClosestPolyEdge(// inputs
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

      vw::Vector2 closest_P = app_data.proj2world(vw::Vector2(minX0, minY0), clipIter);
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

// Find the closest edge in a given vector of polygons to a given point.
void findClosestPolyEdge(// inputs
                         double x0, double y0,
                         std::vector<vw::geometry::dPoly> const& polyVec,
                         // outputs
                         int & polyVecIndex,
                         int & polyIndexInCurrPoly,
                         int & vertIndexInCurrPoly,
                         double & minX, double & minY,
                         double & minDist){
  
  polyVecIndex = -1; polyIndexInCurrPoly = -1; vertIndexInCurrPoly = -1;
  minX = x0; minY = y0; minDist = std::numeric_limits<double>::max();
  
  for (int s = 0; s < (int)polyVec.size(); s++){
    
    double minX0, minY0, minDist0;
    int polyIndex, vertIndex;
    polyVec[s].findClosestPolyEdge(x0, y0,
                                   polyIndex, vertIndex, minX0, minY0, minDist0); // outputs
    
    if (minDist0 <= minDist) {
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

// Assemble the polygon structure
void formPoly(std::string              const& override_color,
              std::vector<int>         const& contiguous_blocks,
              std::vector<std::string> const& colors,
              std::vector<vw::Vector3> const& scattered_data, // input vertices
              std::vector<vw::geometry::dPoly> & polyVec) {

  // Wipe the output
  polyVec.clear();
  polyVec.resize(1);

  if (colors.size() != contiguous_blocks.size()) 
    vw::vw_throw(vw::ArgumentErr() << "There must be as many polygons as colors for them.\n");
  
  size_t vertexCount = 0;
  for (size_t polyIt = 0; polyIt < contiguous_blocks.size(); polyIt++) {
    
    std::vector<double> x, y;
    for (int vertexIt = 0; vertexIt < contiguous_blocks[polyIt]; vertexIt++) {
      
      if (vertexCount >= scattered_data.size())
        vw::vw_throw(vw::ArgumentErr() << "Book-keeping error in reading polygons.\n");
      
      x.push_back(scattered_data[vertexCount].x());
      y.push_back(scattered_data[vertexCount].y());
      vertexCount++;
    }
    
    std::string curr_color = colors[polyIt]; // use color from file if it exists
    
    // The command line color overrides what is in the file
    if (override_color != "default" && override_color != "")
      curr_color = override_color;
    
    bool isPolyClosed = true;
    std::string layer;
    polyVec[0].appendPolygon(x.size(),
                             vw::geometry::vecPtr(x), vw::geometry::vecPtr(y),
                             isPolyClosed, curr_color, layer);
  }
  
  if (vertexCount != scattered_data.size()) 
    vw::vw_throw(vw::ArgumentErr() << "The number of read vertices is not what is expected.\n");
  
  return;
}

void mergePolys(asp::AppData & app_data, int beg_image_id, int end_image_id, int outIndex) {

  std::vector<vw::geometry::dPoly> polyVec;

  try {
    // We will infer these from existing polygons
    std::string poly_color, layer_str;

    // We must first organize all those user-drawn curves into meaningful polygons.
    // This can flip orientations and order of polygons.
    std::vector<OGRGeometry*> ogr_polys;

    for (int clipIt = beg_image_id; clipIt < end_image_id; clipIt++) {

      auto & polyVec = app_data.images[clipIt].polyVec;
      for (size_t vecIter = 0; vecIter < polyVec.size(); vecIter++) {

        if (poly_color == "") {
          std::vector<std::string> colors = polyVec[vecIter].get_colors();
          if (!colors.empty())
            poly_color = colors[0];
        }

        if (layer_str == "") {
          std::vector<std::string> layers = polyVec[vecIter].get_layers();
          if (!layers.empty())
            layer_str = layers[0];
        }

        // Make a copy of the polygons
        vw::geometry::dPoly poly = polyVec[vecIter];

        double * xv         = poly.get_xv();
        double * yv         = poly.get_yv();
        const int* numVerts = poly.get_numVerts();
        int numPolys        = poly.get_numPolys();
        int  totalNumVerts  = poly.get_totalNumVerts();

        // Convert from the coordinate system of each layer
        // to the one of the output layer
        for (int vIter = 0; vIter < totalNumVerts; vIter++) {
          vw::Vector2 P = app_data.proj2world(vw::Vector2(xv[vIter], yv[vIter]), clipIt);
          P = app_data.world2proj(P, outIndex);
          xv[vIter] = P.x();
          yv[vIter] = P.y();
        }

        // Iterate over polygon rings in the given polygon set
        int startPos = 0;
        for (int pIter = 0; pIter < numPolys; pIter++) {

          if (pIter > 0) startPos += numVerts[pIter - 1];
          int numCurrPolyVerts = numVerts[pIter];

          OGRLinearRing R;
          vw::geometry::toOGR(xv, yv, startPos, numCurrPolyVerts, R);

          OGRPolygon * P = new OGRPolygon;
          if (P->addRing(&R) != OGRERR_NONE)
            vw_throw(vw::ArgumentErr() << "Failed add ring to polygon.\n");

          ogr_polys.push_back(P);
        }

      }
    }

    vw::geometry::mergeOGRPolygons(poly_color, layer_str, ogr_polys, polyVec);
    
  } catch(std::exception &e) {
    vw::vw_out() << "OGR failed at " << e.what() << std::endl;
  }

  // Wipe all existing polygons and replace with this one
  for (int clipIt = beg_image_id; clipIt < end_image_id; clipIt++)
    app_data.images[clipIt].polyVec.clear();

  app_data.images[outIndex].polyVec = polyVec;
}

// Delete vertices in a given box
void deleteVerticesInBox(asp::AppData & app_data, vw::BBox2 const& box,
                         int beg_image_id, int end_image_id) {

  for (int clipIt = beg_image_id; clipIt < end_image_id; clipIt++) {

    for (size_t layerIt = 0; layerIt < app_data.images[clipIt].polyVec.size(); layerIt++) {

      vw::geometry::dPoly & poly     = app_data.images[clipIt].polyVec[layerIt]; // alias
      int                   numPolys = poly.get_numPolys();
      const int           * numVerts = poly.get_numVerts();
      const double        * xv       = poly.get_xv();
      const double        * yv       = poly.get_yv();
      std::vector<std::string>   colors   = poly.get_colors();
      std::vector<std::string>   layers   = poly.get_layers();

      vw::geometry::dPoly poly_out;
      int start = 0;
      for (int polyIter = 0; polyIter < numPolys; polyIter++) {

        if (polyIter > 0) start += numVerts[polyIter - 1];
        int pSize = numVerts[polyIter];

        std::vector<double> out_xv, out_yv;
        for (int vIter = 0; vIter < pSize; vIter++) {
          double  x = xv[start + vIter];
          double  y = yv[start + vIter];
          vw::Vector2 P = app_data.proj2world(vw::Vector2(x, y), clipIt);

          // This vertex will be deleted
          if (box.contains(P))
            continue;

          out_xv.push_back(x);
          out_yv.push_back(y);
        }

        bool isPolyClosed = true;
        poly_out.appendPolygon(out_xv.size(),
                               vw::geometry::vecPtr(out_xv),
                               vw::geometry::vecPtr(out_yv),
                               isPolyClosed, colors[polyIter], layers[polyIter]);
      }

      // Overwrite the polygon
      app_data.images[clipIt].polyVec[layerIt] = poly_out;
    }
  }
}

} // end namespace asp
