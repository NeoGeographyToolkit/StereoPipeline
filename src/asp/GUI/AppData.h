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


/// \file AppData.h
///
/// Data structures for the GUI.
///
#ifndef __ASP_GUI_APP_DATA_H__
#define __ASP_GUI_APP_DATA_H__

// Standard library includes
#include <string>
#include <vector>

// Vision Workbench includes
#include <vw/Cartography/GeoTransform.h> // For vw::cartography::GeoTransform

// ASP includes
#include <asp/GUI/GuiUtilities.h> // For imageData

namespace asp { // User requested namespace 'asp'

// See MainWidget.h for what this id does
const int BASE_IMAGE_ID = 0;

struct AppData {

    AppData();
    AppData(vw::GdalWriteOptions const& opt, 
            bool use_georef,
            std::vector<std::map<std::string, std::string>> const& properties,
            std::vector<std::string>const& image_files);
    
    vw::GdalWriteOptions                       opt;
    bool                                       use_georef;
    asp::DisplayMode                       display_mode;
    std::vector<std::string>                   image_files;
    std::vector<asp::imageData>            images;
    std::vector<vw::cartography::GeoTransform> world2image;
    std::vector<vw::cartography::GeoTransform> image2world;
};

} // namespace asp

#endif  // __ASP_GUI_APP_DATA_H__
