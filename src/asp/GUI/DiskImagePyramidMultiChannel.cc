// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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

// TODO(oalexan1): This is very slow to compile. Need to figure out
// why. Could be the bundle adjustment logic. Or contouring, or
// shapefile logic, or DiskImagePyramid read from GuiUtilities.h.
// These may need to be moved to separate files.

// TODO(oalexan1): Move chooseFilesDlg to its own .h/.cc file.

#include <string>
#include <vector>
#include <QPolygon>
#include <QtGui>
#include <QtWidgets>
#include <ogrsf_frmts.h>

// For contours
#include <opencv2/imgproc.hpp>

#include <vw/Math/EulerAngles.h>
#include <vw/Image/Algorithms.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/Hillshade.h>
#include <vw/Core/RunOnce.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/InterestPoint/Matcher.h> // Needed for vw::ip::match_filename
#include <vw/Geometry/dPoly.h>
#include <vw/Cartography/shapeFile.h>
#include <vw/Core/Stopwatch.h>

#include <asp/GUI/GuiUtilities.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>

using namespace vw;
using namespace vw::gui;
using namespace vw::geometry;

namespace vw { namespace gui {

  
}} // namespace vw::gui
