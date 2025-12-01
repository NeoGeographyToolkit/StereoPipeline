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

namespace asp {

// Right-click context menu
MenuMgr::MenuMgr(MainWidget* parent_widget) {
  
  m_ContextMenu = new QMenu(parent_widget);
  
  // Polygon editing mode, they will be visible only when editing happens
  m_insertVertex   = m_ContextMenu->addAction("Insert vertex");
  m_deleteVertex   = m_ContextMenu->addAction("Delete vertex");
  m_deleteVertices = m_ContextMenu->addAction("Delete vertices in selected region");
  m_moveVertex     = m_ContextMenu->addAction("Move vertices");
  m_moveVertex->setCheckable(true);
  m_moveVertex->setChecked(false);

  m_showPolysFilled = m_ContextMenu->addAction("Show polygons filled");
  m_showPolysFilled->setCheckable(true);
  m_showPolysFilled->setChecked(false);

  m_showIndices = m_ContextMenu->addAction("Show vertex indices");
  m_showIndices->setCheckable(true);
  m_showIndices->setChecked(false);

  m_mergePolys = m_ContextMenu->addAction("Merge polygons");

  // Other options
  m_addMatchPoint      = m_ContextMenu->addAction("Add match point");
  m_deleteMatchPoint   = m_ContextMenu->addAction("Delete match point");
  m_moveMatchPoint     = m_ContextMenu->addAction("Move match point");
  m_moveMatchPoint->setCheckable(true);
  m_moveMatchPoint->setChecked(false);
  m_toggleHillshadeImageRightClick  = m_ContextMenu->addAction("Toggle hillshaded display");
  m_setHillshadeParams = m_ContextMenu->addAction("View/set hillshade azimuth and elevation");
  m_saveVectorLayerAsShapeFile = m_ContextMenu->addAction("Save vector layer as shape file");
  m_saveVectorLayerAsTextFile = m_ContextMenu->addAction("Save vector layer as text file");

  m_saveScreenshot     = m_ContextMenu->addAction("Save screenshot");
  m_setThreshold       = m_ContextMenu->addAction("View/set threshold");
  m_allowMultipleSelections_action
    = m_ContextMenu->addAction("Allow multiple selected regions");
  m_allowMultipleSelections_action->setCheckable(true);
  m_allowMultipleSelections_action->setChecked(parent_widget->m_allowMultipleSelections);

  m_deleteSelection = m_ContextMenu->addAction("Delete selected regions around this point");
  m_hideImagesNotInRegion
    = m_ContextMenu->addAction("Hide images not intersecting selected region");

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

} // End namespace asp
