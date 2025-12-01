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

/// \file MenuMgr.cc
///
/// Handles menu creation and management for MainWidget.
///
#include <asp/GUI/MenuMgr.h>
#include <asp/GUI/MainWidget.h>
#include <asp/GUI/GuiUtilities.h>

namespace asp {

// Right-click context menu
MenuMgr::MenuMgr(MainWidget* parent_widget) {
  
  m_contextMenu = new QMenu(parent_widget);
  
  // Polygon editing mode, they will be visible only when editing happens
  m_insertVertex   = m_contextMenu->addAction("Insert vertex");
  m_deleteVertex   = m_contextMenu->addAction("Delete vertex");
  m_deleteVertices = m_contextMenu->addAction("Delete vertices in selected region");
  m_moveVertex     = m_contextMenu->addAction("Move vertices");
  m_moveVertex->setCheckable(true);
  m_moveVertex->setChecked(false);

  m_showPolysFilled = m_contextMenu->addAction("Show polygons filled");
  m_showPolysFilled->setCheckable(true);
  m_showPolysFilled->setChecked(false);

  m_showIndices = m_contextMenu->addAction("Show vertex indices");
  m_showIndices->setCheckable(true);
  m_showIndices->setChecked(false);

  m_mergePolys = m_contextMenu->addAction("Merge polygons");

  // Other options
  m_addMatchPoint      = m_contextMenu->addAction("Add match point");
  m_deleteMatchPoint   = m_contextMenu->addAction("Delete match point");
  m_moveMatchPoint     = m_contextMenu->addAction("Move match point");
  m_moveMatchPoint->setCheckable(true);
  m_moveMatchPoint->setChecked(false);
  m_toggleHillshadeImageRightClick  = m_contextMenu->addAction("Toggle hillshaded display");
  m_setHillshadeParams = m_contextMenu->addAction("View/set hillshade azimuth and elevation");
  m_saveVectorLayerAsShapeFile = m_contextMenu->addAction("Save vector layer as shape file");
  m_saveVectorLayerAsTextFile = m_contextMenu->addAction("Save vector layer as text file");

  m_saveScreenshot     = m_contextMenu->addAction("Save screenshot");
  m_setThreshold       = m_contextMenu->addAction("View/set threshold");
  m_allowMultipleSelections_action
    = m_contextMenu->addAction("Allow multiple selected regions");
  m_allowMultipleSelections_action->setCheckable(true);
  m_allowMultipleSelections_action->setChecked(parent_widget->m_allowMultipleSelections);

  m_deleteSelection = m_contextMenu->addAction("Delete selected regions around this point");
  m_hideImagesNotInRegion
    = m_contextMenu->addAction("Hide images not intersecting selected region");

  // Connect signals to slots in the parent_widget
  QObject::connect(m_addMatchPoint, SIGNAL(triggered()), 
                   parent_widget, SLOT(addMatchPoint()));
  QObject::connect(m_deleteMatchPoint, SIGNAL(triggered()), 
                   parent_widget, SLOT(deleteMatchPoint()));
  QObject::connect(m_toggleHillshadeImageRightClick, SIGNAL(triggered()), 
                   parent_widget, SLOT(toggleHillshadeImageRightClick()));
  QObject::connect(m_setHillshadeParams, SIGNAL(triggered()), 
                   parent_widget, SLOT(setHillshadeParams()));
  QObject::connect(m_setThreshold, SIGNAL(triggered()), 
                   parent_widget, SLOT(setThreshold()));
  QObject::connect(m_saveScreenshot, SIGNAL(triggered()), 
                   parent_widget, SLOT(saveScreenshot()));
  QObject::connect(m_allowMultipleSelections_action, SIGNAL(triggered()), 
                   parent_widget, SLOT(allowMultipleSelections()));
  QObject::connect(m_deleteSelection, SIGNAL(triggered()), 
                   parent_widget, SLOT(deleteSelection()));
  QObject::connect(m_hideImagesNotInRegion, SIGNAL(triggered()), 
                   parent_widget, SLOT(hideImagesNotInRegion()));
  QObject::connect(m_saveVectorLayerAsShapeFile, SIGNAL(triggered()), 
                   parent_widget, SLOT(saveVectorLayerAsShapeFile()));
  QObject::connect(m_saveVectorLayerAsTextFile, SIGNAL(triggered()), 
                   parent_widget, SLOT(saveVectorLayerAsTextFile()));
  QObject::connect(m_deleteVertex, SIGNAL(triggered()), 
                   parent_widget, SLOT(deleteVertex()));
  QObject::connect(m_deleteVertices, SIGNAL(triggered()), 
                   parent_widget, SLOT(deleteVertices()));
  QObject::connect(m_insertVertex, SIGNAL(triggered()), 
                   parent_widget, SLOT(insertVertex()));
  QObject::connect(m_mergePolys, SIGNAL(triggered()), 
                   parent_widget, SLOT(mergePolys()));
}

void MenuMgr::setupContextMenu(MainWidget* parent_widget) {
  
  // If in poly edit mode, turn on these items.
  m_deleteVertex->setVisible(parent_widget->m_polyEditMode);
  m_deleteVertices->setVisible(parent_widget->m_polyEditMode);
  m_insertVertex->setVisible(parent_widget->m_polyEditMode);
  m_moveVertex->setVisible(parent_widget->m_polyEditMode);
  m_showIndices->setVisible(parent_widget->m_polyEditMode);
  m_showPolysFilled->setVisible(parent_widget->m_polyEditMode);

  // Add the saving polygon option even when not editing
  m_saveVectorLayerAsShapeFile->setVisible(true);
  m_saveVectorLayerAsTextFile->setVisible(true);

  m_mergePolys->setVisible(parent_widget->m_polyEditMode);

  // Refresh this from the variable, before popping up the menu
  m_allowMultipleSelections_action->setChecked(parent_widget->m_allowMultipleSelections);

  // Turn on these items if we are NOT in poly edit mode. Also turn some off
  // in sideBySideWithDialog() mode, as then we draw the interest points
  // only with refreshPixmap(), which is rare, so user's editing
  // choices won't be reflected in the GUI.
  m_addMatchPoint->setVisible(!parent_widget->m_polyEditMode && !sideBySideWithDialog());
  m_deleteMatchPoint->setVisible(!parent_widget->m_polyEditMode && !sideBySideWithDialog());
  m_moveMatchPoint->setVisible(!parent_widget->m_polyEditMode && !sideBySideWithDialog());
  m_toggleHillshadeImageRightClick->setVisible(!parent_widget->m_polyEditMode);
  m_setHillshadeParams->setVisible(!parent_widget->m_polyEditMode);
  m_setThreshold->setVisible(!parent_widget->m_polyEditMode);
  m_allowMultipleSelections_action->setVisible(!parent_widget->m_polyEditMode);
  m_deleteSelection->setVisible(!sideBySideWithDialog());
  m_hideImagesNotInRegion->setVisible(!sideBySideWithDialog());

  m_saveScreenshot->setVisible(true); // always visible
}

QMenu* MenuMgr::formCustomMenu(MainWidget* parent_widget) {
  
m_customMenu = new QMenu(parent_widget);

m_toggleHillshadeFromImageList = m_customMenu->addAction("Toggle hillshade display");
  QObject::connect(m_toggleHillshadeFromImageList, SIGNAL(triggered()),
                   parent_widget, SLOT(toggleHillshadeFromImageList()));

  if (!sideBySideWithDialog()) {
    // Do not offer these options when the images are side-by-side,
    // as that will just mess up with their order.

    m_bringImageOnTopFromTable = m_customMenu->addAction("Bring image on top");
    QObject::connect(m_bringImageOnTopFromTable, SIGNAL(triggered()),
                     parent_widget, SLOT(bringImageOnTopSlot()));

    m_pushImageToBottomFromTable = m_customMenu->addAction("Push image to bottom");
    QObject::connect(m_pushImageToBottomFromTable, SIGNAL(triggered()),
                     parent_widget, SLOT(pushImageToBottomSlot()));
  }

  m_zoomToImageFromTable = m_customMenu->addAction("Zoom to image");
  QObject::connect(m_zoomToImageFromTable, SIGNAL(triggered()),
                   parent_widget, SLOT(zoomToImage()));

  // If having polygons, make it possible to change their colors
  bool hasPoly = false;
  for (int image_iter = parent_widget->m_beg_image_id;
       image_iter < parent_widget->m_end_image_id; image_iter++) {
    if (parent_widget->app_data.images[image_iter].m_isPoly)
      hasPoly = true;
  }
  if (hasPoly) {
    m_changePolyColor = m_customMenu->addAction("Change colors of polygons");
    QObject::connect(m_changePolyColor, SIGNAL(triggered()), parent_widget, SLOT(changePolyColor()));
  }

  return m_customMenu;
}
} // End namespace asp
