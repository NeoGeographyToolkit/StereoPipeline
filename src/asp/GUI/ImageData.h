// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

/// \file ImageData.h
///
/// A class to keep all data associated with an image file

#ifndef __STEREO_GUI_IMAGE_DATA_H__
#define __STEREO_GUI_IMAGE_DATA_H__

// ASP
#include <asp/GUI/DiskImagePyramidMultiChannel.h>

// VW
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Geometry/dPoly.h>

#include <string>
#include <vector>
#include <map>

namespace asp {

  enum DisplayMode {REGULAR_VIEW, HILLSHADED_VIEW, COLORIZED_VIEW,
                    HILLSHADE_COLORIZED_VIEW,
                    THRESHOLDED_VIEW};

  // Return true if the extension is .csv or .txt
  bool hasCsv(std::string const& fileName);

  /// A class to keep all data associated with an image file
  class imageData {
  public:
    std::string      name, hillshaded_name, thresholded_name, colorized_name;
    vw::GdalWriteOptions m_opt;
    bool             has_georef;
    vw::cartography::GeoReference georef;
    vw::BBox2        image_bbox;
    vw::Vector2      val_range;
    bool             loaded_regular, loaded_hillshaded,
      loaded_thresholded, loaded_colorized; // if the image was loaded
    // There are several display modes. The one being shown is
    // determined by m_display_mode. Store the corresponding
    // image in one of the structures below
    DisplayMode m_display_mode;
    DiskImagePyramidMultiChannel img;
    DiskImagePyramidMultiChannel hillshaded_img;
    DiskImagePyramidMultiChannel thresholded_img;
    DiskImagePyramidMultiChannel colorized_img;

    std::vector<vw::geometry::dPoly> polyVec; // a shapefile
    std::string color; // poly color
    std::string style; // plotting style
    std::string colormap; // colormap style
    bool colorbar; // if a given image must be colorized

    // Scattered data to be plotted at (x, y) location with z giving
    // the intensity. May be colorized.
    std::vector<vw::Vector3> scattered_data;

    imageData(): m_display_mode(REGULAR_VIEW), has_georef(false),
                 loaded_regular(false), loaded_hillshaded(false),
                 loaded_thresholded(false), loaded_colorized(false),
                 m_isPoly(false), m_isCsv(false), colorbar(false) {}

    /// Read an image from disk into img and set the other variables.
    void read(std::string const& image, vw::GdalWriteOptions const& opt,
              DisplayMode display_mode = REGULAR_VIEW,
              std::map<std::string, std::string> const& properties =
              std::map<std::string, std::string>(), bool delay_loading = false);

    // The actual loading happens here
    void load();

    // Save the polygons to a plain text file
    void writePoly(std::string const& polyFile);

    bool m_isPoly, m_isCsv;
    bool isPolyOrCsv() const { return m_isPoly || m_isCsv; }

private:
    // These are very slow if used per pixel, so we cache their results in
    // member variables. Never call these directly.
    bool isPolyInternal(std::string const& name, std::string const& style) const;
    bool isCsvInternal(std::string const& name, std::string const& style) const;

  };

} // namespace asp

#endif  // __STEREO_GUI_IMAGE_DATA_H__
