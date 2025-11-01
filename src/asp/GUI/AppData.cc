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

/// \file AppData.cc
///
/// Data structures for the GUI.
///

#include <asp/GUI/AppData.h>
#include <asp/GUI/GuiArgs.h>
#include <asp/GUI/GuiUtilities.h>
#include <asp/Core/StereoSettings.h>

namespace asp {

// Set up the gui data
AppData::AppData(vw::GdalWriteOptions const& opt,
                 bool use_georef,
                 std::vector<std::map<std::string, std::string>> const& properties,
                 std::vector<std::string> const& image_files): 
  m_opt(opt), m_use_georef(use_georef), 
  m_image_files(image_files) {

  m_display_mode 
    = asp::stereo_settings().hillshade ? vw::gui::HILLSHADED_VIEW : vw::gui::REGULAR_VIEW;

  if (!stereo_settings().zoom_proj_win.empty())
    m_use_georef = true;
  
  size_t num_images = m_image_files.size();
  m_images.resize(num_images);

  std::vector<int> propertyIndices;
  asp::lookupPropertyIndices(properties, m_image_files, propertyIndices);

  // Read the images. If there is a delay, we read only the georef, deferring
  // for later loading the image pixels.
  bool delay = asp::stereo_settings().preview;
  bool has_georef = true;
  for (size_t i = 0; i < num_images; i++) {
    m_images[i].read(m_image_files[i], m_opt, vw::gui::REGULAR_VIEW,
                     properties[propertyIndices[i]],
                     delay);
    
    // Above we read the image in regular mode. If plan to display hillshade,
    // for now set the flag for that, and the hillshaded image will be created
    // and set later. (Something more straightforward could be done.)
    m_images[i].m_display_mode = m_display_mode;
    has_georef = has_georef && m_images[i].has_georef;
  }

  // Use georef if all images have it. This may be turned off later if it is desired
  // to show matches.
  if (has_georef)
    m_use_georef = true;

  // It is tricky to set up a layout for georeferenced images if they are loaded
  // one or a few at a time.
  if (delay) 
    m_use_georef = false;
  
  // If the user explicitly asked to not use georef, do not use it on startup
  if (asp::stereo_settings().no_georef) {
    m_use_georef = false; 
    // Further control of georef is from the gui menu
    asp::stereo_settings().no_georef = false; 
  }

  // Create the coordinate transforms
  m_world2image_trans.resize(num_images);
  m_image2world_trans.resize(num_images);
  if (m_use_georef) {
    for (int i = 0; i < num_images; i++) {  
      m_world2image_trans[i]
        = vw::cartography::GeoTransform(m_images[BASE_IMAGE_ID].georef,
                                        m_images[i].georef);
      m_image2world_trans[i]
        = vw::cartography::GeoTransform(m_images[i].georef,
                                        m_images[BASE_IMAGE_ID].georef);
    }
  }
  
}

} // namespace asp
