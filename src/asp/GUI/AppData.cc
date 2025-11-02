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
/// Data structures for the GUI
///

#include <asp/GUI/AppData.h>
#include <asp/GUI/GuiArgs.h>
#include <asp/GUI/GuiUtilities.h>
#include <asp/Core/StereoSettings.h>

namespace asp {

// Empty constructor
AppData::AppData(): use_georef(false),
                    display_mode(asp::REGULAR_VIEW) {}
                     
// Set up the gui data
AppData::AppData(vw::GdalWriteOptions const& opt_in,
                 bool use_georef_in,
                 std::vector<std::map<std::string, std::string>> const& properties,
                 std::vector<std::string> const& image_files_in): 
  opt(opt_in), use_georef(use_georef_in), image_files(image_files_in) {

  display_mode 
    = asp::stereo_settings().hillshade?asp::HILLSHADED_VIEW:asp::REGULAR_VIEW;

  if (!stereo_settings().zoom_proj_win.empty())
    use_georef = true;
  
  size_t num_images = image_files.size();
  images.resize(num_images);

  std::vector<int> propertyIndices;
  asp::lookupPropertyIndices(properties, image_files, propertyIndices);

  // Read the images. If there is a delay, we read only the georef, deferring
  // for later loading the image pixels.
  bool delay = asp::stereo_settings().preview;
  bool has_georef = true;
  for (size_t i = 0; i < num_images; i++) {
    images[i].read(image_files[i], opt, asp::REGULAR_VIEW,
                   properties[propertyIndices[i]], delay);
    
    // Above we read the image in regular mode. If plan to display hillshade,
    // for now set the flag for that, and the hillshaded image will be created
    // and set later. (Something more straightforward could be done.)
    images[i].m_display_mode = display_mode;
    has_georef = has_georef && images[i].has_georef;
  }

  // Use georef if all images have it. This may be turned off later if it is desired
  // to show matches.
  if (has_georef)
    use_georef = true;

  // It is tricky to set up a layout for georeferenced images if they are loaded
  // one or a few at a time.
  if (delay) 
    use_georef = false;
  
  // If the user explicitly asked to not use georef, do not use it on startup
  if (asp::stereo_settings().no_georef) {
    use_georef = false; 
    // Further control of georef is from the gui menu
    asp::stereo_settings().no_georef = false; 
  }

  // Create the coordinate transforms
  world2image.resize(num_images);
  image2world.resize(num_images);
  if (use_georef) {
    for (int i = 0; i < num_images; i++) {  
      world2image[i]
        = vw::cartography::GeoTransform(images[BASE_IMAGE_ID].georef,
                                        images[i].georef);
      image2world[i]
        = vw::cartography::GeoTransform(images[i].georef,
                                        images[BASE_IMAGE_ID].georef);
    }
  }
  
}

// If we use georef, the world is in projected point units of the
// first image, with y replaced with -y, to keep the y axis downward,
// for consistency with how images are plotted.  Convert a world box
// to a pixel box for the given image.
vw::Vector2 AppData::world2image_trans(vw::Vector2 const& P, int imageIndex) const{
  bool poly_or_xyz = (images[imageIndex].m_isPoly || images[imageIndex].m_isCsv);

  if (poly_or_xyz) {
    // Poly or points. There is no pixel concept in that case.
    if (!use_georef)
      return flip_in_y(P);
    return world2image[imageIndex].point_to_point(flip_in_y(P));
  }

  // Image
  if (!use_georef)
    return P;
  return world2image[imageIndex].point_to_pixel(flip_in_y(P));
}

vw::BBox2 AppData::world2image_trans(vw::BBox2 const& R, int imageIndex) const {

  bool poly_or_xyz = (images[imageIndex].m_isPoly || images[imageIndex].m_isCsv);

  if (R.empty())
    return R;
  if (images.empty())
    return R;

  if (poly_or_xyz) {
    // Poly or points. There is no pixel concept in that case.
    if (!use_georef)
      return flip_in_y(R);
    return world2image[imageIndex].point_to_point_bbox(flip_in_y(R));
  }

  // Image
  if (!use_georef)
    return R;
  return world2image[imageIndex].point_to_pixel_bbox(flip_in_y(R));
}

// The reverse of world2image_trans()
vw::Vector2 AppData::image2world_trans(vw::Vector2 const& P, int imageIndex) const {

  bool poly_or_xyz = (images[imageIndex].m_isPoly || images[imageIndex].m_isCsv);

  if (poly_or_xyz) {
    if (!use_georef)
      return flip_in_y(P);

    return flip_in_y(image2world[imageIndex].point_to_point(P));
  }

  if (!use_georef)
    return P;
  return flip_in_y(image2world[imageIndex].pixel_to_point(P));
}

// The reverse of world2image_trans()
vw::BBox2 AppData::image2world_trans(vw::BBox2 const& R, int imageIndex) const {

  if (R.empty()) return R;
  if (images.empty()) return R;

  bool poly_or_xyz = (images[imageIndex].m_isPoly || images[imageIndex].m_isCsv);

  // Consider the case when the current layer is a polygon.
  // TODO(oalexan1): What if a layer has both an image and a polygon?

  if (poly_or_xyz) {
    if (!use_georef)
      return flip_in_y(R);
    return flip_in_y(image2world[imageIndex].point_to_point_bbox(R));
  }

  if (!use_georef)
    return R;
  return flip_in_y(image2world[imageIndex].pixel_to_point_bbox(R));
}

// Convert from world coordinates to projected coordinates in given geospatial
// projection
vw::Vector2 AppData::world2proj(vw::Vector2 P, int imageIndex) const {
  if (!use_georef)
      return flip_in_y(P);
  return world2image[imageIndex].point_to_point(flip_in_y(P)); 
}

// The reverse of world2proj
vw::Vector2 AppData::proj2world(vw::Vector2 P, int imageIndex) const {
  if (!use_georef)
    return flip_in_y(P);
  return flip_in_y(image2world[imageIndex].point_to_point(P));
}

} // namespace asp
