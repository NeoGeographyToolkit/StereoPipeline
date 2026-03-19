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

// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).
//
// Lightweight replacements for sfm::SfmCameraInfo, sfm::View, and sfm::Scene
// used by sfm_view. These avoid pulling in the full MVE library for types
// that are simple data containers in this application.

#ifndef __ASP_SFMVIEW_SFM_UTILS_H__
#define __ASP_SFMVIEW_SFM_UTILS_H__

#include <memory>
#include <string>
#include <vector>

namespace sfm {

// SfmCamera intrinsic and extrinsic parameters.
// Stores world-to-camera rotation and translation.
struct SfmCameraInfo {
  SfmCameraInfo();

  // Compute camera position: pos = -R^T * t. Output is 3 floats.
  void fill_camera_pos(float* pos) const;

  // Store the viewing direction (third row of rotation). Output is 3 floats.
  void fill_viewing_direction(float* viewdir) const;

  // Store the 4x4 camera-to-world matrix. Output is 16 floats.
  void fill_cam_to_world(float* mat) const;

  // Read camera parameters from a .tsai pinhole model file.
  void read_tsai(std::string const& filename);

  // Intrinsic parameters
  float flen;

  // Extrinsic parameters (double for large satellite orbit values)
  double trans[3]; // SfmCamera translation: pos = -R^T * trans
  double rot[9];   // World-to-camera rotation (row-major 3x3)
};

// A single camera view: holds a name and camera parameters.
class View {
public:
  using Ptr = std::shared_ptr<View>;
  static Ptr create();

  // Load camera from a .tsai file. The view name is set to the
  // basename of camera_path.
  void load_view(std::string const& image_path,
                 std::string const& camera_path);

  std::string const& get_name() const;

  SfmCameraInfo const& get_camera() const;
  void set_camera(SfmCameraInfo const& cam);

  // No-op. Kept for API compatibility with code that prevents save prompts.
  void set_dirty(bool dirty);

private:
  View() = default;
  std::string name_;
  SfmCameraInfo camera_;
};

// A collection of camera views.
class Scene {
public:
  using Ptr = std::shared_ptr<Scene>;
  using ViewList = std::vector<View::Ptr>;

  // Create a scene from paired image and .tsai camera files.
  static Ptr create(std::vector<std::string> const& image_files,
                    std::vector<std::string> const& camera_files);

  ViewList const& get_views() const;
  ViewList& get_views();
  View::Ptr get_view_by_id(std::size_t id);

private:
  Scene() = default;
  ViewList views_;
};

} // namespace sfm

#endif // __ASP_SFMVIEW_SFM_UTILS_H__
