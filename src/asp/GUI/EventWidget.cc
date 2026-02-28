// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

// \file EventWidget.cc
//
// Member functions of MainWidget that handle mouse, keyboard,
// and other input events.

#include <asp/GUI/MainWidget.h>
#include <asp/GUI/GuiConstants.h>
#include <asp/GUI/GuiGeom.h>
#include <asp/GUI/ChooseFilesDlg.h>
#include <asp/Core/StereoSettings.h>
#include <asp/GUI/WidgetMenuMgr.h>

#include <vw/Geometry/geomUtils.h>

#include <QtGui>
#include <QtWidgets>

using namespace vw;

namespace asp {

// All functions here are member functions of MainWidget, but are moved
// to this file to reduce the size of MainWidget.cc.

void MainWidget::mousePressEvent(QMouseEvent *event) {

  // For rubberband
  m_mousePrsX  = event->pos().x();
  m_mousePrsY  = event->pos().y();
  m_rubberBand = m_emptyRubberBand;

  m_curr_pixel_pos = QPoint2Vec(QPoint(m_mousePrsX, m_mousePrsY)); // Record where we clicked
  updateCurrentMousePosition();

  // Need this for panning
  m_last_view = m_current_view;

  // Check if the user is holding down the crop window key.
  m_cropWinMode = ((event->buttons  () & Qt::LeftButton) &&
                    (event->modifiers() & Qt::ControlModifier));

  m_match_mgr.m_editMatchPointVecIndex = -1; // Keep this initialized

  // If the user is currently editing match points
  if (!m_polyEditMode && m_wid_menu_mgr->m_moveMatchPoint->isChecked()
      && !m_cropWinMode && asp::stereo_settings().view_matches) {

    m_match_mgr.m_editingMatches = true;

    Vector2 P = screen2world(Vector2(m_mousePrsX, m_mousePrsY));
    P = app_data.world2image_trans(P, m_base_image_id);

    // Find the match point we want to move
    double dist_limit = MATCH_POINT_DISTANCE_LIMIT;
    m_match_mgr.m_editMatchPointVecIndex
      = m_match_mgr.m_matchlist.findNearestMatchPoint(m_beg_image_id, P, dist_limit);

    if (asp::stereo_settings().view_matches) {
      // Update IP draw color
      // Will keep the zoom level
      emit updateMatchesSignal();
    } else {
      // Will reset the layout before continuing with matches
      asp::stereo_settings().view_matches = true;
      emit toggleViewMatchesSignal();
    }
  } // End match point update case

  // If the user is currently editing polygons
  if (m_polyEditMode && m_wid_menu_mgr->m_moveVertex->isChecked() && !m_cropWinMode) {

    Vector2 P = screen2world(Vector2(m_mousePrsX, m_mousePrsY));
    m_world_box.grow(P); // to not cut when plotting later

    // Find the vertex we want to move
    asp::PolySearchResult sr;
    asp::findClosestPolyVertex(P.x(), P.y(), app_data,
                               m_beg_image_id, m_end_image_id, sr);
    m_editClipIndex           = sr.clipIndex;
    m_editPolyVecIndex        = sr.polyVecIndex;
    m_editIndexInCurrPoly     = sr.polyIndexInCurrPoly;
    m_editVertIndexInCurrPoly = sr.vertIndexInCurrPoly;

    // This will redraw just the polygons, not the pixmap
    update();

    // The action will continue in mouseMoveEvent()

  } // End polygon update case

} // End function mousePressEvent()

void MainWidget::mouseMoveEvent(QMouseEvent *event) {

  QPoint Q = event->pos();
  int mouseMoveX = Q.x(), mouseMoveY = Q.y();

  m_curr_pixel_pos = QPoint2Vec(Q);
  updateCurrentMousePosition();

  if (!(event->buttons() & Qt::LeftButton))
    return;

  // The mouse is pressed and moving
  m_cropWinMode = ((event->buttons  () & Qt::LeftButton) &&
                   (event->modifiers() & Qt::ControlModifier));

  if (handleMatchPointMove(mouseMoveX, mouseMoveY))
    return;
  if (handlePolyVertexMove(mouseMoveX, mouseMoveY))
    return;
  handleRubberBandDrag(mouseMoveX, mouseMoveY);
} // End function mouseMoveEvent()

// Handle match point dragging during mouse move
bool MainWidget::handleMatchPointMove(int mouseMoveX, int mouseMoveY) {

  if (m_polyEditMode || !m_wid_menu_mgr->m_moveMatchPoint->isChecked() ||
      m_cropWinMode)
    return false;

  m_match_mgr.m_editingMatches = true;

  // Error checking
  if ((m_beg_image_id < 0) || (m_match_mgr.m_editMatchPointVecIndex < 0) ||
        (!m_match_mgr.m_matchlist.pointExists(m_beg_image_id,
          m_match_mgr.m_editMatchPointVecIndex)))
    return true;

  Vector2 P = screen2world(Vector2(mouseMoveX, mouseMoveY));
  P = app_data.world2image_trans(P, m_base_image_id);

  // Update the IP location
  m_match_mgr.m_matchlist.setPointPosition(m_beg_image_id,
    m_match_mgr.m_editMatchPointVecIndex, P.x(), P.y());

  if (asp::stereo_settings().view_matches) {
    // Update IP draw color
    // Will keep the zoom level
    emit updateMatchesSignal();
  } else {
    // Will reset the layout before continuing with matches
    asp::stereo_settings().view_matches = true;
    emit toggleViewMatchesSignal();
  }
  return true;
}

// Handle polygon vertex dragging during mouse move
bool MainWidget::handlePolyVertexMove(int mouseMoveX, int mouseMoveY) {

  if (!m_polyEditMode || !m_wid_menu_mgr->m_moveVertex->isChecked() ||
      m_cropWinMode)
    return false;

  // If moving vertices
  if (m_editClipIndex < 0 ||
      m_editPolyVecIndex        < 0 ||
      m_editIndexInCurrPoly     < 0 ||
      m_editVertIndexInCurrPoly < 0)
    return true;

  Vector2 P = screen2world(Vector2(mouseMoveX, mouseMoveY));

  m_world_box.grow(P); // to not cut when plotting later
  P = app_data.world2proj(P, m_editClipIndex); // projected units
  app_data.images[m_editClipIndex].polyVec[m_editPolyVecIndex]
    .changeVertexValue(m_editIndexInCurrPoly,
                       m_editVertIndexInCurrPoly, P.x(), P.y());
  // This will redraw just the polygons, not the pixmap
  update();
  return true;
}

// Handle rubberband rectangle dragging during mouse move
void MainWidget::handleRubberBandDrag(int mouseMoveX, int mouseMoveY) {

  // Standard Qt rubberband trick. This is highly confusing.  The
  // explanation for what is going on is the following.  We need to
  // wipe the old rubberband, and draw a new one.  Hence just the
  // perimeters of these two rectangles need to be re-painted,
  // nothing else changes. The first updateRubberBand() call below
  // schedules that the perimeter of the current rubberband be
  // repainted, but the actual repainting, and this is the key, WILL
  // HAPPEN LATER! Then we change m_rubberBand to the new value,
  // then we schedule the repaint event on the new rubberband.
  // Continued below.
  updateRubberBand(m_rubberBand);
  m_rubberBand = QRect(std::min(m_mousePrsX, mouseMoveX),
                        std::min(m_mousePrsY, mouseMoveY),
                        std::abs(mouseMoveX - m_mousePrsX),
                        std::abs(mouseMoveY - m_mousePrsY));
  updateRubberBand(m_rubberBand);
  // Only now, a single call to MainWidget::PaintEvent() happens,
  // even though it appears from above that two calls could happen
  // since we requested two updates. This call updates the perimeter
  // of the old rubberband, in effect wiping it, since the region
  // occupied by the old rubberband is scheduled to be repainted,
  // but the rubberband itself is already changed.  It also updates
  // the perimeter of the new rubberband, and as can be seen in
  // MainWidget::PaintEvent() the effect is to draw the rubberband.

  if (m_cropWinMode && !m_allowMultipleSelections) {
    // If there is on screen already a crop window, wipe it, as
    // we are now in the process of creating a new one.
    QRect R = bbox2qrect(world2screen(m_stereoCropWin));
    updateRubberBand(R);
    m_stereoCropWin = BBox2();
    R = bbox2qrect(world2screen(m_stereoCropWin));
    updateRubberBand(R);
  }
}

// The action to take when a user releases the mouse close to where it was pressed
void MainWidget::handlePixelClick(int mouseRelX, int mouseRelY) {

  // Threshold mode: early return
  if (m_threshold.calcMode) {
    handleThresholdClick(mouseRelX, mouseRelY);
    return;
  }

  Vector2 p = screen2world(Vector2(mouseRelX, mouseRelY));
  if (!m_profile.mode && !m_polyEditMode) {
    QPainter paint;
    paint.begin(&m_pixmap);
    QPoint Q(mouseRelX, mouseRelY);
    paint.setPen(QColor("red"));
    paint.drawEllipse(Q, 2, 2); // Draw the point, and make it a little larger
    paint.end();  // Make sure to end the painting session
  }

  bool can_profile = m_profile.mode;

  // Print pixel coordinates and image value.
  for (int j = m_beg_image_id; j < m_end_image_id; j++) {

    int it = m_filesOrder[j];

    // Don't show files the user wants hidden
    std::string fileName = app_data.images[it].name;
    if (m_chooseFiles && m_chooseFiles->isHidden(fileName))
      continue;

    std::string val = "none";
    Vector2 q = app_data.world2image_trans(p, it);

    int col = floor(q[0]), row = floor(q[1]);

    if (col >= 0 && row >= 0 && col < app_data.images[it].img().cols() &&
        row < app_data.images[it].img().rows())
      val = app_data.images[it].img().get_value_as_str(col, row);

    vw_out() << "Pixel and value: " << app_data.images[it].name << " ("
              << col << ", " << row << ") " << val << "\n";

    update();

    if (m_profile.mode) {

      // Sanity checks
      if (m_end_image_id - m_beg_image_id != 1) {
        popUp("A profile can be shown only when a single image is present.");
        can_profile = false;
      }
      int num_channels = app_data.images[it].img().planes();
      if (num_channels != 1) {
        popUp("A profile can be shown only when the image has a single channel.");
        can_profile = false;
      }

      if (!can_profile) {
        MainWidget::setProfileMode(can_profile);
        return;
      }

    } // End if m_profile.mode

  } // end iterating over images

  if (can_profile) {
    // Save the current point the user clicked onto in the
    // world coordinate system.
    m_profile.x.push_back(p.x());
    m_profile.y.push_back(p.y());

    // PaintEvent() will be called, which will call
    // plotProfilePolyLine() to show the polygonal line.

    // Now show the profile.
    MainWidget::plotProfile(app_data.images, m_profile.x, m_profile.y);

    // TODO(oalexan1): Why is this buried in the short distance check?
  } else if (m_polyEditMode) {
    handlePolyEditClick(mouseRelX, mouseRelY);
  }
}

// Handle poly edit mode click: move or add vertex
void MainWidget::handlePolyEditClick(int mouseRelX, int mouseRelY) {

  if (m_wid_menu_mgr->m_moveVertex->isChecked() && !m_cropWinMode) {
    // Move vertex

    if (m_editPolyVecIndex        < 0 ||
        m_editIndexInCurrPoly     < 0 ||
        m_editVertIndexInCurrPoly < 0)
      return;

    Vector2 P = screen2world(Vector2(mouseRelX, mouseRelY));
    m_world_box.grow(P); // to not cut when plotting later
    P = app_data.world2proj(P, m_polyLayerIndex); // projected units
    app_data.images[m_polyLayerIndex].polyVec[m_editPolyVecIndex]
      .changeVertexValue(m_editIndexInCurrPoly, m_editVertIndexInCurrPoly,
                          P.x(), P.y());

    // These are no longer needed for the time being
    m_editPolyVecIndex        = -1;
    m_editIndexInCurrPoly     = -1;
    m_editVertIndexInCurrPoly = -1;

    // This will redraw just the polygons, not the pixmap
    update();
    return;
  }

  // Add vertex
  addPolyVert(mouseRelX, mouseRelY);
}

// Handle threshold mode pixel click
void MainWidget::handleThresholdClick(int mouseRelX, int mouseRelY) {

  // Image threshold mode. If we released the mouse where we
  // pressed it, that means we want the current pixel value
  // to be the threshold if larger than the existing threshold.
  if (m_end_image_id - m_beg_image_id != 1) {
    popUp("Must have just one image in each window to do "
          "image threshold detection.");
    m_threshold.calcMode = false;
    refreshPixmap();
    return;
  }

  if (app_data.images[m_beg_image_id].img().planes() != 1) {
    popUp("Thresholding makes sense only for single-channel images.");
    m_threshold.calcMode = false;
    return;
  }

  if (app_data.use_georef) {
    popUp("Thresholding is not supported when using georeference "
          "information to show images.");
    m_threshold.calcMode = false;
    return;
  }

  Vector2 p = screen2world(Vector2(mouseRelX, mouseRelY));
  Vector2 q = app_data.world2image_trans(p, m_beg_image_id);

  int col = round(q[0]), row = round(q[1]);
  vw_out() << "Clicked on pixel: " << col << ' ' << row << std::endl;

  if (col >= 0 && row >= 0 &&
      col < app_data.images[m_beg_image_id].img().cols() &&
      row < app_data.images[m_beg_image_id].img().rows()) {
    double val = app_data.images[m_beg_image_id].img().get_value_as_double(
      col, row);
    m_threshold.value = std::max(m_threshold.value, val);
  }

  vw_out() << "Image threshold for "
            << app_data.images[m_beg_image_id].name
            << ": " << m_threshold.value << std::endl;
}

// Handle crop win in mouse release event
void MainWidget::handleCropWin() {

  // If now we allow multiple selected regions, but we did not allow at
  // the time the crop win was formed, save the crop win before it
  // will be overwritten.
  if (m_allowMultipleSelections && !m_stereoCropWin.empty()) {
    if (m_selectionRectangles.empty() ||
        m_selectionRectangles.back() != m_stereoCropWin) {
      m_selectionRectangles.push_back(m_stereoCropWin);
    }
  }

  // User selects the region to use for stereo.  Convert it to world
  // coordinates, and round to integer.  If we use georeferences,
  // the crop win is in projected units for the first image,
  // so we must convert to pixels.
  m_stereoCropWin = screen2world(qrect2bbox(m_rubberBand));

  if (m_allowMultipleSelections && !m_stereoCropWin.empty())
    m_selectionRectangles.push_back(m_stereoCropWin);

  for (int j = m_beg_image_id; j < m_end_image_id; j++) {

    int image_it = m_filesOrder[j];

    // Don't show files the user wants hidden
    std::string fileName = app_data.images[image_it].name;
    if (m_chooseFiles && m_chooseFiles->isHidden(fileName))
      continue;

    BBox2 image_box = app_data.world2image_trans(m_stereoCropWin, image_it);
    vw_out() << std::setprecision(8)
              << "src win for    " << app_data.images[image_it].name << ": "
              << round(image_box.min().x()) << ' ' << round(image_box.min().y()) << ' '
              << round(image_box.width())   << ' ' << round(image_box.height())  << std::endl;

    if (app_data.images[image_it].has_georef) {
      Vector2 proj_min, proj_max;
      // Convert pixels to projected coordinates
      BBox2 point_box;
      if (app_data.images[image_it].isPolyOrCsv())
        point_box = image_box;
      else
        point_box = app_data.images[image_it].georef.pixel_to_point_bbox(image_box);

      proj_min = point_box.min();
      proj_max = point_box.max();
      // Below we flip in y to make gdal happy
      vw_out() << std::setprecision(17)
                << "proj win for   "
                << app_data.images[image_it].name << ": "
                << proj_min.x() << ' ' << proj_max.y() << ' '
                << proj_max.x() << ' ' << proj_min.y() << std::endl;

      Vector2 lonlat_min, lonlat_max;
      BBox2 lonlat_box = app_data.images[image_it].georef.point_to_lonlat_bbox(point_box);

      lonlat_min = lonlat_box.min();
      lonlat_max = lonlat_box.max();
      // Again, miny and maxy are flipped on purpose
      vw_out() << std::setprecision(17)
                << "lonlat win for "
                << app_data.images[image_it].name << ": "
                << lonlat_min.x() << ' ' << lonlat_max.y() << ' '
                << lonlat_max.x() << ' ' << lonlat_min.y() << std::endl;
    }
  }

  // Wipe the rubberband, no longer needed.
  updateRubberBand(m_rubberBand);
  m_rubberBand = m_emptyRubberBand;
  updateRubberBand(m_rubberBand);

  // Draw the crop window. This may not be precisely the rubberband
  // since there is some loss of precision in conversion from
  // Qrect to BBox2 and back. Note actually that we are not drawing
  // here, we are scheduling this area to be updated, the drawing
  // has to happen (with precisely this formula) in PaintEvent().
  QRect R = bbox2qrect(world2screen(m_stereoCropWin));
  updateRubberBand(R);

  return;
}

// Zoom in or out after the user releases the mouse, depending on
// the rubberband direction
void MainWidget::zoomInOut(int mouseRelX, int mouseRelY) {

  // Wipe the rubberband
  updateRubberBand(m_rubberBand);
  m_rubberBand = m_emptyRubberBand;
  updateRubberBand(m_rubberBand);

  m_can_emit_zoom_all_signal = true;

  if (mouseRelX > m_mousePrsX && mouseRelY > m_mousePrsY) {

    // Dragging the mouse from upper-left to lower-right zooms in

    // The window selected with the mouse in world coordinates
    Vector2 A = screen2world(Vector2(m_mousePrsX, m_mousePrsY));
    Vector2 B = screen2world(Vector2(mouseRelX, mouseRelY));
    BBox2 view = BBox2(A, B);

    // Zoom to this window. Don't zoom so much that the view box
    // ends up having size 0 to numerical precision.
    if (!view.empty()) {
      double ratio = double(m_window_width) / double(m_window_height);
      m_current_view = vw::geometry::expandBoxToRatio(view, ratio);
    }

    // Must redraw the entire image
    refreshPixmap();

  } else if (mouseRelX < m_mousePrsX && mouseRelY < m_mousePrsY) {
    // Dragging the mouse in reverse zooms out
    double scale = 0.8;
    zoom(scale);
  }

}

// If a point was being moved, reset the ID and color. Points that are moved
// are also set to valid.
void MainWidget::adjustForEditMatchPoint() {
  m_match_mgr.m_matchlist.setPointValid(m_beg_image_id, m_match_mgr.m_editMatchPointVecIndex, true);
  m_match_mgr.m_editMatchPointVecIndex = -1;

  if (asp::stereo_settings().view_matches) {
    // Update IP draw color
    // Will keep the zoom level
    emit updateMatchesSignal();
  } else {
    // Will reset the layout before continuing with matches
    asp::stereo_settings().view_matches = true;
    emit toggleViewMatchesSignal();
  }
}

// Handle mouse release events
void MainWidget::mouseReleaseEvent(QMouseEvent *event) {

  QPoint mouse_rel_pos = event->pos();
  int mouseRelX = mouse_rel_pos.x();
  int mouseRelY = mouse_rel_pos.y();

  // Ctrl + left mouse button triggers crop window mode
  if ((event->buttons() & Qt::LeftButton) &&
      (event->modifiers() & Qt::ControlModifier)) {
    m_cropWinMode = true;
  }

  if (app_data.images.empty())
    return;

  // If a point was being moved, reset the ID and color. Points that are moved
  // are also set to valid.
  if (m_match_mgr.m_editMatchPointVecIndex >= 0)
    adjustForEditMatchPoint();

  // If the mouse was released close to where it was pressed
  if (std::abs(m_mousePrsX - mouseRelX) < m_pixelTol &&
      std::abs(m_mousePrsY - mouseRelY) < m_pixelTol) {
    handlePixelClick(mouseRelX, mouseRelY);
    return;
  }

  // Do not zoom or do other funny stuff if we are moving IP or vertices
  if (!m_polyEditMode && m_wid_menu_mgr->m_moveMatchPoint->isChecked() && !m_cropWinMode)
    return;

  if (m_polyEditMode && m_wid_menu_mgr->m_moveVertex->isChecked() && !m_cropWinMode)
    return;

  if (event->buttons() & Qt::RightButton) {
    // Drag the image along the mouse movement
    m_current_view -= (screen2world(QPoint2Vec(mouse_rel_pos)) -
                        screen2world(QPoint2Vec(QPoint(m_mousePrsX, m_mousePrsY))));
    refreshPixmap(); // will call paintEvent()
    return;
  }
  if (m_cropWinMode) {
    handleCropWin();
    return;
  }

  if (Qt::LeftButton) {
    zoomInOut(mouseRelX, mouseRelY);
    return;
  }

  // At this stage the user is supposed to release the control key, so
  // we are no longer in crop win mode, even if we were so far.
  m_cropWinMode = false;

  return;
} // End mouseReleaseEvent()

void MainWidget::mouseDoubleClickEvent(QMouseEvent *event) {
  m_curr_pixel_pos = QPoint2Vec(event->pos());
  updateCurrentMousePosition();
}

void MainWidget::wheelEvent(QWheelEvent *event) {
  int num_degrees = event->angleDelta().y() / 8;
  double num_ticks = double(num_degrees) / 360;

  // Accumulate ticks
  m_accumulatedZoomTicks += num_ticks;

  // Merge all mouse wheel movements within this time frame. This prevents many
  // incremental zoom actions.
  m_zoomTimer->start(250);

  m_curr_pixel_pos = QPointF2Vec(event->position());
  updateCurrentMousePosition();
}

void MainWidget::enterEvent(QEvent *event) {
}

void MainWidget::leaveEvent(QEvent */*event*/) {
}

void MainWidget::keyPressEvent(QKeyEvent *event) {

  // Save these before we modify the box
  double width  = m_current_view.width();
  double height = m_current_view.height();

  double factor = 0.2;  // We will pan by moving by 20%.
  switch (event->key()) {

    // Pan
  case Qt::Key_Left: // Pan left
    m_current_view.min().x() -= width*factor;
    m_current_view.max().x() -= width*factor;
    m_can_emit_zoom_all_signal = true;
    refreshPixmap();
    break;
  case Qt::Key_Right: // Pan right
    m_current_view.min().x() += width*factor;
    m_current_view.max().x() += width*factor;
    m_can_emit_zoom_all_signal = true;
    refreshPixmap();
    break;
  case Qt::Key_Up: // Pan up
    m_current_view.min().y() -= height*factor;
    m_current_view.max().y() -= height*factor;
    m_can_emit_zoom_all_signal = true;
    refreshPixmap();
    break;
  case Qt::Key_Down: // Pan down
    m_current_view.min().y() += height*factor;
    m_current_view.max().y() += height*factor;
    m_can_emit_zoom_all_signal = true;
    refreshPixmap();
    break;

  default:
    QWidget::keyPressEvent(event);
  }
}

// Apply the zoom after a delay to accumulate mouse wheel events
void MainWidget::handleZoomTimeout() {

  if (m_accumulatedZoomTicks == 0.0)
    return;

  // Multiply by 2.0 to make zooming more responsive
  double mag = std::abs(2.0 * m_accumulatedZoomTicks);

  double scale = 1;
  if (m_accumulatedZoomTicks > 0)
    scale = 1 + mag;
  else if (m_accumulatedZoomTicks < 0)
    scale = std::max(1 - mag, 0.5);

  zoom(scale);

  // Reset accumulated ticks after applying zoom
  m_accumulatedZoomTicks = 0.0;
}

void MainWidget::contextMenuEvent(QContextMenuEvent *event) {

  int x = event->x(), y = event->y();
  m_mousePrsX = x;
  m_mousePrsY = y;

  m_wid_menu_mgr->setupContextMenu(this);

  m_wid_menu_mgr->m_contextMenu->popup(mapToGlobal(QPoint(x,y)));
  return;
}

} // namespace asp
