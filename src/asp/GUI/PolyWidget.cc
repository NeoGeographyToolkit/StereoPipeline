// __BEGIN_LICENSE__
//  Copyright (c) 2006-2024, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  https://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

/// \file PolyWidget.cc
///
/// Member functions of MainWidget that have to do with polygons.

#include <asp/GUI/MainWidget.h>
#include <asp/GUI/GuiGeom.h>
#include <asp/GUI/chooseFilesDlg.h>
#include <asp/Core/StereoSettings.h>

#include <vw/Cartography/shapeFile.h>

#include <qwt_point_data.h>
#include <QtGui>
#include <QtWidgets>
#include <QMenu>
#include <string>
#include <vector>

using namespace vw;

namespace asp {

// All functions here are member functions of MainWidget, but are moved
// to this file to reduce the size of MainWidget.cc.

// Change the color of given layer of polygons
void MainWidget::changePolyColor() {
  std::string polyColor;
  bool ans = getStringFromGui(this,
                              "Polygonal line color",
                              "Polygonal line color",
                              polyColor, polyColor);
  if (!ans)
    return;

  if (polyColor == "") {
    popUp("The polygonal line color must be set.");
    return;
  }

  for (auto it = m_indicesWithAction.begin(); it != m_indicesWithAction.end(); it++) {

    // We will assume the user wants to see this on top
    m_perImagePolyColor[*it] = polyColor;
    bringImageOnTop(*it);
  }

  // This is no longer needed
  m_indicesWithAction.clear();

  // Redraw everything, which will change the color
  refreshPixmap();
}

void MainWidget::setPolyEditMode(bool polyEditMode, bool refresh) {
  m_polyEditMode = polyEditMode;

  // Turn off moving vertices any time we turn on or off poly editing
  m_moveVertex->setChecked(false);
  m_showIndices->setChecked(false);

  if (!m_polyEditMode) {
    // Clean up any unfinished polygon
    // Need to put here pop-up asking to save
    m_currPolyX.clear();
    m_currPolyY.clear();

    // Call back to the main window and tell it to uncheck the polyEditMode checkbox
    // mode checkbox.
    emit uncheckPolyEditModeCheckbox();
    return;
  } else {
    setProfileMode(false);
  }

  // On occasions we don't want to force a refresh prematurely, the
  // GUI will take care of it when the layout is created
  if (refresh)
    refreshPixmap();
}

// TODO(oalexan1): Move out this non-gui function.
void MainWidget::appendToPolyVec(vw::geometry::dPoly const& P) {

  // Append the new polygon to the list of polygons. If we have several
  // clips already, append it to the last clip. If we have no clips,
  // create a new clip.
  if (app_data.images[m_polyLayerIndex].polyVec.size() == 0) {
    app_data.images[m_polyLayerIndex].polyVec.push_back(P);
  } else {
    app_data.images[m_polyLayerIndex].polyVec.back().appendPolygons(P);
  }

  return;
}

// Add a point to the polygon being drawn or stop drawing and append
// the drawn polygon to the list of polygons. This polygon is in the
// world coordinate system. When we append it, we will convert it to
// points in the desired geodetic projection.
void MainWidget::addPolyVert(double px, double py) {

  Vector2 S(px, py); // current point in screen pixels
  int pSize = m_currPolyX.size();

  // This is a bugfix. Before starting drawing, update
  // m_polyLayerIndex to point to a currently visible layer,
  // otherwise it looks as if polygons are invisible.
  if (pSize == 0 && m_chooseFiles && 
      m_chooseFiles->isHidden(app_data.images[m_polyLayerIndex].name)) {
    for (int j = m_beg_image_id; j < m_end_image_id; j++) {
      int i = m_filesOrder[j]; // image index

      if (m_chooseFiles && m_chooseFiles->isHidden(app_data.images[i].name))
        continue;
      m_polyLayerIndex = i; // not hidden
    }
  }

  // Starting point in this polygon. It is absolutely essential that we
  // keep it in world units. Otherwise, if we zoom while the polygon
  // is being drawn, we will not be able to close it properly.
  if (pSize == 0)
    m_startPix = screen2world(S);

  if (pSize <= 0 || norm_2(world2screen(m_startPix) - S) > m_pixelTol) {

    // We did not arrive yet at the starting point of the polygon being
    // drawn. Add the current point.

    S = screen2world(S);                    // world coordinates
    m_world_box.grow(S); // to not cut when plotting later
    S = app_data.world2proj(S, m_polyLayerIndex); // projected units

    m_currPolyX.push_back(S.x());
    m_currPolyY.push_back(S.y());
    pSize = m_currPolyX.size();

    // This will call paintEvent which will draw the current poly line
    update();

    return;
  }

  // Form the newly finished polygon
  vw::geometry::dPoly poly;
  poly.reset();
  bool isPolyClosed = true;
  std::string color = app_data.images[m_polyLayerIndex].color;
  if (color == "default")
    color = m_polyColor; // if no color was set from the command line

  // If the user set a custom color
  auto color_it = m_perImagePolyColor.find(m_polyLayerIndex);
  if (color_it != m_perImagePolyColor.end()) {
    color = color_it->second;
    app_data.images[m_polyLayerIndex].color = color; // save for the future
  }
  std::string layer = "";
  poly.appendPolygon(pSize,
      vw::geometry::vecPtr(m_currPolyX), vw::geometry::vecPtr(m_currPolyY),
                  isPolyClosed, color, layer);
  appendToPolyVec(poly);
  m_currPolyX.clear();
  m_currPolyY.clear();

  update(); // redraw the just polygons, not the underlying images
  //refreshPixmap();

  return;
}

// Delete a vertex closest to where the user clicked.
// TODO(oalexan1): This will fail when different polygons have
// different georeferences.
void MainWidget::deleteVertex() {

  Vector2 P = screen2world(Vector2(m_mousePrsX, m_mousePrsY));

  double min_x, min_y, min_dist;
  int clipIndex, polyVecIndex, polyIndexInCurrPoly, vertIndexInCurrPoly;
  asp::findClosestPolyVertex(// inputs
                             P.x(), P.y(), app_data,
                             m_beg_image_id, m_end_image_id,
                             // outputs
                             clipIndex,
                             polyVecIndex,
                             polyIndexInCurrPoly,
                             vertIndexInCurrPoly,
                             min_x, min_y, min_dist);

  if (clipIndex           < 0 ||
      polyVecIndex        < 0 ||
      polyIndexInCurrPoly < 0 ||
      vertIndexInCurrPoly < 0)
    return;

  app_data.images[clipIndex].polyVec[polyVecIndex].eraseVertex
    (polyIndexInCurrPoly, vertIndexInCurrPoly);

  // This will redraw just the polygons, not the pixmap
  update();

  return;
}

// TODO(oalexan1): Move out this non-gui function.
void MainWidget::deleteVertices() {

  if (m_stereoCropWin.empty()) {
    popUp("No region is selected.");
    return;
  }

  asp::deleteVerticesInBox(app_data, m_stereoCropWin, m_beg_image_id, m_end_image_id);
  
  // The selection has done its job, wipe it now
  m_stereoCropWin = BBox2();

  // This will redraw just the polygons, not the pixmap
  update();
}

// Insert intermediate vertex where the mouse right-clicks.
// TODO(oalexan1): This will fail when different polygons have
// different georeferences.
// TODO(oalexan1): Move out this non-gui function.
void MainWidget::insertVertex() {

  Vector2 P = screen2world(Vector2(m_mousePrsX, m_mousePrsY));
  m_world_box.grow(P); // to not cut when plotting later

  // If there is absolutely no polygon, start by creating one
  // with just one point.
  bool allEmpty = true;
  for (size_t clipIt = m_beg_image_id; clipIt < m_end_image_id; clipIt++) {
    if (app_data.images[clipIt].polyVec.size() > 0 &&
        app_data.images[clipIt].polyVec[0].get_totalNumVerts() > 0) {
      allEmpty = false;
      break;
    }
  }

  if (allEmpty) {
    addPolyVert(m_mousePrsX, m_mousePrsY); // init the polygon
    addPolyVert(m_mousePrsX, m_mousePrsY); // declare the polygon finished
    return;
  }

  // The location of the point to be inserted looks more reasonable
  // when one searches for closest edge, not vertex.
  double min_x, min_y, min_dist;
  int clipIndex, polyVecIndex, polyIndexInCurrPoly, vertIndexInCurrPoly;
  asp::findClosestPolyEdge(// inputs
                           P.x(), P.y(), app_data,
                           m_beg_image_id, m_end_image_id,
                           // outputs
                           clipIndex,
                           polyVecIndex,
                           polyIndexInCurrPoly,
                           vertIndexInCurrPoly,
                           min_x, min_y, min_dist);

  if (clipIndex           < 0 ||
      polyVecIndex        < 0 ||
      polyIndexInCurrPoly < 0 ||
      vertIndexInCurrPoly < 0) return;

  // Convert to coordinates of the desired clip
  P = app_data.world2proj(P, clipIndex);

  // Need +1 below as we insert AFTER current vertex
  app_data.images[clipIndex].polyVec[polyVecIndex].insertVertex(polyIndexInCurrPoly,
                                                          vertIndexInCurrPoly + 1,
                                                          P.x(), P.y());

  // This will redraw just the polygons, not the pixmap
  update();

  return;
}

// Merge existing polygons
void MainWidget::mergePolys() {
  asp::mergePolys(app_data, m_beg_image_id, m_end_image_id, m_polyLayerIndex);
}


// Save the currently created vector layer
void MainWidget::saveVectorLayerAsShapeFile() {

  if (m_polyLayerIndex < m_beg_image_id || m_polyLayerIndex >= m_end_image_id) {
    popUp("Images are inconsistent. Cannot save vector layer.");
    return;
  }

  std::string shapeFile = app_data.images[m_polyLayerIndex].name;
  shapeFile =  boost::filesystem::path(shapeFile).replace_extension(".shp").string();
  QString qShapeFile = QFileDialog::getSaveFileName(this,
                                                  tr("Save shapefile"), shapeFile.c_str(),
                                                  tr("(*.shp)"));


  shapeFile = qShapeFile.toStdString();
  if (shapeFile == "")
    return;

  bool has_geo = app_data.images[m_polyLayerIndex].has_georef;
  vw::cartography::GeoReference const& geo = app_data.images[m_polyLayerIndex].georef;

  // Save only polygons in the given layer. Polygons in other layers
  // can have individual georeferences.
  vw_out() << "Writing: " << shapeFile << std::endl;
  write_shapefile(shapeFile, has_geo, geo, app_data.images[m_polyLayerIndex].polyVec);
}

// Save the currently created vector layer. Its index is m_polyLayerIndex.
// Other layers are not saved. They may have their own georeferences.
void MainWidget::saveVectorLayerAsTextFile() {

  if (m_polyLayerIndex < m_beg_image_id || m_polyLayerIndex >= m_end_image_id) {
    popUp("Images are inconsistent. Cannot save vector layer.");
    return;
  }

  std::string textFile = app_data.images[m_polyLayerIndex].name;
  textFile =  boost::filesystem::path(textFile).replace_extension(".txt").string();
  QString qtextFile = QFileDialog::getSaveFileName(this,
                                                  tr("Save text file"), textFile.c_str(),
                                                  tr("(*.txt)"));
  textFile = qtextFile.toStdString();
  if (textFile == "")
    return;

  app_data.images[m_polyLayerIndex].writePoly(textFile);
}


// Contour the current image
bool MainWidget::contourImage() {

  int non_poly_image = -1;
  int num_non_poly_images = 0;
  for (int image_iter = m_beg_image_id; image_iter < m_end_image_id; image_iter++) {
    if (!app_data.images[image_iter].m_isPoly && !app_data.images[image_iter].m_isCsv)
      num_non_poly_images++;
    non_poly_image = image_iter;
  }

  if (num_non_poly_images > 1) {
    popUp("Must have just one image in window to contour an image.");
    return false;
  }

  if (non_poly_image < 0)
    return true; // Will quietly skip this

  m_polyLayerIndex = non_poly_image;

  int num_channels = app_data.images[m_polyLayerIndex].img.planes();
  if (num_channels > 1) {
    popUp("Contouring images makes sense only for single-channel images.");
    return false;
  }

  if (num_channels == 1)
    contour_image(app_data.images[m_polyLayerIndex].img, app_data.images[m_polyLayerIndex].georef,
                  m_thresh, app_data.images[m_polyLayerIndex].polyVec);

  // This will call paintEvent which will draw the contour
  update();

  return true;
}


void MainWidget::drawOneVertex(int x0, int y0, QColor color, int lineWidth,
                               int drawVertIndex, QPainter &paint) {

  // Draw a vertex as a small shape (a circle, rectangle, triangle)

  // Use variable size shapes to distinguish better points on top of
  // each other
  int len = 2*(drawVertIndex+1);
  len = std::min(len, 8); // limit how big this can get

  paint.setPen(QPen(color, lineWidth));

  int numTypes = 4;
  if (drawVertIndex < 0) {

    // This will be reached only for the case when a polygon
    // is so small that it collapses into a point.
    len = lineWidth;
    paint.setBrush(color);
    paint.drawRect(x0 - len, y0 - len, 2*len, 2*len);

  } else if (drawVertIndex%numTypes == 0) {

    // Draw a small empty ellipse
    paint.setBrush(Qt::NoBrush);
    paint.drawEllipse(x0 - len, y0 - len, 2*len, 2*len);

  } else if (drawVertIndex%numTypes == 1) {

    // Draw an empty square
    paint.setBrush(Qt::NoBrush);
    paint.drawRect(x0 - len, y0 - len, 2*len, 2*len);

  } else if (drawVertIndex%numTypes == 2) {

    // Draw an empty triangle
    paint.setBrush(Qt::NoBrush);
    paint.drawLine(x0 - len, y0 - len, x0 + len, y0 - len);
    paint.drawLine(x0 - len, y0 - len, x0 + 0,   y0 + len);
    paint.drawLine(x0 + len, y0 - len, x0 + 0,   y0 + len);

  } else {

    // Draw an empty reversed triangle
    paint.setBrush(Qt::NoBrush);
    paint.drawLine(x0 - len, y0 + len, x0 + len, y0 + len);
    paint.drawLine(x0 - len, y0 + len, x0 + 0,   y0 - len);
    paint.drawLine(x0 + len, y0 + len, x0 + 0,   y0 - len);

  }

  return;
}


void MainWidget::plotPoly(bool plotPoints, bool plotEdges,
                          bool plotFilled, bool showIndices,
                          int lineWidth,
                          int drawVertIndex, // 0 is a good choice here
                          QColor const& color,
                          QPainter & paint,
                          // Make a local copy of the poly on purpose
                          vw::geometry::dPoly currPoly) {

  using namespace vw::geometry;

  if (m_world_box.empty())
    return;

  // The box in world coordinates
  double x_min = m_world_box.min().x();
  double y_min = m_world_box.min().y();
  double x_max = m_world_box.max().x();
  double y_max = m_world_box.max().y();

  double screen_min_x = 0, screen_min_y = 0;

  // When polys are filled, plot largest polys first
  if (plotFilled) {
    // What on screen looks counter-clockwise, internally is clockwise,
    // because the screen y axis is in fact pointing down.
    // We need to reverse the orientation for the logic below to work properly
    //currPoly.reverse();
    //currPoly.sortBySizeAndMaybeAddBigContainingRect(x_min,  y_min, x_max, y_max);
    //currPoly.reverse();
  }

  // Clip the polygon a bit beyond the viewing window, as to not see
  // the edges where the cut took place. It is a bit tricky to
  // decide how much the extra should be.
  double tol = 1e-12;
  double pixelSize = std::max(m_world_box.width()/m_window_width,
                              m_world_box.height()/m_window_height);

  double extra  = 2*pixelSize*lineWidth;
  double extraX = extra + tol * std::max(std::abs(x_min), std::abs(x_max));
  double extraY = extra + tol * std::max(std::abs(y_min), std::abs(y_max));

  // Will try to use the color from polygons if they exist. Otherwise
  // use the default color.
  QColor local_color = color;

  dPoly clippedPoly;
  currPoly.clipPoly(x_min - extraX, y_min - extraY, x_max + extraX, y_max + extraY,
                    clippedPoly); // output

  std::vector<vw::geometry::anno> annotations;
  if (showIndices) {
    clippedPoly.compVertIndexAnno();
    clippedPoly.get_vertIndexAnno(annotations);
  }

  const double * xv       = clippedPoly.get_xv();
  const double * yv       = clippedPoly.get_yv();
  const int    * numVerts = clippedPoly.get_numVerts();
  int numPolys            = clippedPoly.get_numPolys();

  // Aliases
  const std::vector<char> & isPolyClosed = clippedPoly.get_isPolyClosed();
  const std::vector<std::string> & colors = clippedPoly.get_colors(); // we ignore these

  int start = 0;
  for (int pIter = 0; pIter < numPolys; pIter++) {

    if (pIter > 0) start += numVerts[pIter - 1];
    int pSize = numVerts[pIter];
    // Use the corresponding color if it exists and is valid
    if (colors.size() > pIter) {
      QColor curr_color = QColor(colors[pIter].c_str());
      if (curr_color.isValid())
        local_color = curr_color;
    }

    // Determine the orientation of polygons
    double signedArea = 0.0;
    if (plotFilled && isPolyClosed[pIter]) {
      bool counter_cc = true;
      signedArea = signedPolyArea(pSize, xv + start, yv + start, counter_cc);
    }

    QPolygon pa(pSize);
    for (int vIter = 0; vIter < pSize; vIter++) {

      Vector2 P = world2screen(Vector2(xv[start + vIter], yv[start + vIter]));
      pa[vIter] = QPoint(P.x(), P.y());

      // Qt's built in points are too small. Instead of drawing a point
      // draw a small shape.
      int tol = 4; // This is a bug fix for missing points. I don't understand
      //           // why this is necessary and why the number 4 is right.
      if (plotPoints                                                                 &&
          P.x() > screen_min_x - tol && P.x() < screen_min_x + m_window_width  + tol &&
          P.y() > screen_min_y - tol && P.y() < screen_min_y + m_window_height + tol) {
        drawOneVertex(P.x(), P.y(), local_color, lineWidth, drawVertIndex, paint);
      }
    }

    if (pa.size() <= 0) continue;

    if (plotEdges) {

      if (plotFilled && isPolyClosed[pIter]) {
        // Notice that we fill clockwise polygons, those with
        // negative area. That because on screen they in fact
        // appear counter-clockwise, since the screen y axis is
        // always down, and because the ESRI shapefile format
        // expects an outer polygon to be clockwise.
        if (signedArea < 0.0)  paint.setBrush(local_color);
        else                   paint.setBrush(m_backgroundColor);
        paint.setPen(Qt::NoPen);
      } else {
        paint.setBrush(Qt::NoBrush);
        paint.setPen(QPen(local_color, lineWidth));
      }

      if (isPolyZeroDim(pa)) {
        // Treat the case of polygons which are made up of just one point
        int l_drawVertIndex = -1;
        drawOneVertex(pa[0].x(), pa[0].y(), local_color, lineWidth, l_drawVertIndex,
                      paint);
      } else if (isPolyClosed[pIter]) {

        if (plotFilled) {
          paint.drawPolygon(pa);
        } else {
          // In some versions of Qt, drawPolygon is buggy when not
          // called to fill polygons. Don't use it, just draw the
          // edges one by one.
          int n = pa.size();
          for (int k = 0; k < n; k++) {
            QPolygon pb;
            int x0, y0; pa.point(k, &x0, &y0);       pb << QPoint(x0, y0);
            int x1, y1; pa.point((k+1)%n, &x1, &y1); pb << QPoint(x1, y1);
            paint.drawPolyline(pb);
          }
        }
      } else {
        paint.drawPolyline(pa); // don't join the last vertex to the first
      }
    }
  }

  // Plot the annotations
  int numAnno = annotations.size();
  for (int aIter = 0; aIter < numAnno; aIter++) {
    const anno & A = annotations[aIter];
    // Avoid points close to boundary, as were we clipped artificially
    if (! (A.x >= x_min && A.x <= x_max && A.y >= y_min && A.y <= y_max)) continue;
    Vector2 P = world2screen(Vector2(A.x, A.y));
    paint.setPen(QPen(QColor("gold"), lineWidth));
    paint.drawText(P.x(), P.y(), (A.label).c_str());
  } // End plotting annotations

  return;
}

// Go to the pixel locations on screen, and draw the polygonal line.
// This is robust to zooming in the middle of profiling.
// TODO: This will function badly when zooming.
void MainWidget::plotProfilePolyLine(QPainter & paint,
                                     std::vector<double> const& profileX,
                                     std::vector<double> const& profileY) {

  if (profileX.empty()) return;

  paint.setPen(QColor("red"));
  std::vector<QPoint> profilePixels;
  for (size_t it = 0; it < profileX.size(); it++) {
    Vector2 P = world2screen(Vector2(profileX[it], profileY[it]));
    QPoint Q(P.x(), P.y());
    paint.drawEllipse(Q, 2, 2); // Draw the point, and make it a little large
    profilePixels.push_back(Q);
  }
  paint.drawPolyline(&profilePixels[0], profilePixels.size());
}

void MainWidget::setPolyColor(std::string const& polyColor) {
  m_polyColor = polyColor;

  // When the color is set from the top menu rather than right-clicking
  // on an individual layer in the table on the left, it applies to all polygons
  for (int image_iter = m_beg_image_id; image_iter < m_end_image_id; image_iter++)
    m_perImagePolyColor[image_iter] = polyColor;

  refreshPixmap();
}

std::string MainWidget::getPolyColor() {
  return m_polyColor;
}

void MainWidget::setLineWidth(int lineWidth) {
  m_lineWidth = lineWidth;
  update();
}

int MainWidget::getLineWidth() {
  return m_lineWidth;
}

} // end namespace asp
