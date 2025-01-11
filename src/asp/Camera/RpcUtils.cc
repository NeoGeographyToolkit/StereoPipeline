// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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

// RPC model utilities

#include <asp/Camera/RpcUtils.h>
#include <asp/Camera/RPCModel.h>

#include <vw/Cartography/GeoReference.h>

namespace asp {

// Sample the llh box and shoot the 3D points into the camera.
// Filter by image box.
void sample_llh_pix_bbox(vw::BBox2 const& ll_box,
                         vw::Vector2 const& h_range,
                         int num_samples,
                         vw::cartography::Datum const& datum,
                         vw::CamPtr cam,
                         vw::BBox2 const& image_box,
                         // Outputs
                         std::vector<vw::Vector3> & all_llh,
                         std::vector<vw::Vector2> & all_pixels) {

  // Wipe the outputs
  all_llh.clear();
  all_pixels.clear();

  double delta_lon = (ll_box.max()[0] - ll_box.min()[0])/double(num_samples);
  double delta_lat = (ll_box.max()[1] - ll_box.min()[1])/double(num_samples);
  double delta_ht  = (h_range[1] - h_range[0])/double(num_samples);

  for (double lon = ll_box.min()[0]; lon <= ll_box.max()[0]; lon += delta_lon) {
    for (double lat = ll_box.min()[1]; lat <= ll_box.max()[1]; lat += delta_lat) {
      for (double ht = h_range[0]; ht <= h_range[1]; ht += delta_ht) {

        vw::Vector3 llh(lon, lat, ht);
        vw::Vector3 xyz = datum.geodetic_to_cartesian(llh);

        // Go back to llh. This is a bugfix for the 360 deg offset problem.
        llh = datum.cartesian_to_geodetic(xyz);

        vw::Vector2 cam_pix;
        try {
          // The point_to_pixel function can be capricious
          cam_pix = cam->point_to_pixel(xyz);
        } catch(...) {
          continue;
        }
        if (image_box.contains(cam_pix)) {
          all_llh.push_back(llh);
          all_pixels.push_back(cam_pix);
        }

      }
    }
  }
  
  return;
}

// Add pixel and llh samples along the perimeter and diagonals of image_box.
// Constrain by the ll box.
void add_perimeter_diag_points(vw::BBox2 const& image_box, 
                               vw::cartography::Datum const& datum,
                               vw::CamPtr cam,
                               vw::BBox2 const& ll, // lon-lat box
                               vw::Vector2 const& H, // height range
                               // Outputs (append to these)
                               std::vector<vw::Vector3> & all_llh,
                               std::vector<vw::Vector2> & all_pixels) {

  // Reduce the max by 1, as sample_float_box() assumes the max is not exclusive
  vw::BBox2 b = image_box;
  b.max() -= vw::Vector2(1, 1);

  // Calc samples on box perimeter and diagonals
  // Add on the perimeter a non-small portion of points we added so far throughout
  int num_steps = std::max(100, int(all_llh.size()/10));
  std::vector<vw::Vector2> points;
  vw::cartography::sample_float_box(b, points, num_steps);
  
  // Use only the min and max heights
  for (size_t i = 0; i < H.size(); i++) {
    double h = H[i];

    for (size_t j = 0; j < points.size(); j++) {
      vw::Vector2 pix = points[j];
      
      double semi_major = datum.semi_major_axis() + h;
      double semi_minor = datum.semi_minor_axis() + h;
      vw::Vector3 intersection;
      try {
        intersection 
          = vw::cartography::datum_intersection(semi_major, semi_minor, 
                                                cam->camera_center(pix), 
                                                cam->pixel_to_vector(pix));
        if (intersection == vw::Vector3())
          continue;
      } catch (...) {
        continue;
      }

      vw::Vector3 llh = datum.cartesian_to_geodetic(intersection);
      
      // Must be contained in the lon-lat box
      if (!ll.contains(vw::Vector2(llh[0], llh[1])))
        continue;
        
      all_llh.push_back(llh);
      all_pixels.push_back(pix);
    }
  }

  return;
}

} // end namespace asp
