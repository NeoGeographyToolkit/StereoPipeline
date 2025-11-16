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

/// \file BaParams.cc
///

#include <asp/Camera/BaParams.h>

#include <vw/Cartography/Datum.h>
#include <vw/FileIO/KML.h>
#include <vw/Core/Log.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <fstream>

namespace {
   // A global random generator, to avoid having this as class member
   boost::random::mt19937 g_rand_gen;
}

namespace asp {

IntrinsicOptions::IntrinsicOptions(): 
  center_shared(true), focus_shared(true), distortion_shared(true),
  share_intrinsics_per_sensor(false), num_sensors(0) {}

// Control per each group of cameras or for all cameras which intrinsics
// should be floated.
bool IntrinsicOptions::float_optical_center(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_center[sensor_id];
}

bool IntrinsicOptions::float_focal_length(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_focus[sensor_id];
}

bool IntrinsicOptions::float_distortion_params(int cam_index) const {
  // When sharing intrinsics per sensor, each sensor's float behavior is independent
  int sensor_id = 0;
  if (share_intrinsics_per_sensor) 
    sensor_id = cam2sensor.at(cam_index);

  return float_distortion[sensor_id];
}

// BaParams constructor
BaParams::BaParams(int num_points, int num_cameras,
          // Parameters below here only apply to pinhole models.
          bool using_intrinsics,
          int max_num_dist_params,
          IntrinsicOptions intrinsics_opts): 
    m_num_points        (num_points),
    m_num_cameras       (num_cameras),
    m_params_per_point  (PARAMS_PER_POINT),
    m_num_pose_params   (NUM_CAMERA_PARAMS),
    m_num_shared_intrinsics    (0),
    m_num_intrinsics_per_camera(0),
    m_max_num_dist_params(max_num_dist_params),
    m_focus_offset(0),
    m_distortion_offset(0),
    m_intrinsics_opts    (intrinsics_opts),
    m_points_vec        (num_points * PARAMS_PER_POINT,  0),
    m_cameras_vec       (num_cameras * NUM_CAMERA_PARAMS, 0),
    m_intrinsics_vec    (0),
    m_outlier_points_vec(num_points, false) {

    if (!using_intrinsics)
      return; // If we are not using intrinsics, nothing else to do.

    // Calculate how many values are stored per-camera, and
    //  what the offset is to a particular intrinsic value.
    // - The start of the array is always an entry for each intrinsic in case
    //   it is shared.
    // The logic is very different when intrinsics are shared per sensor,
    // rather than not shared at all or all shared.
    // TODO(oalexan1): Integrate this logic.
    if (!intrinsics_opts.share_intrinsics_per_sensor) {
      if (!intrinsics_opts.center_shared)
        m_num_intrinsics_per_camera += NUM_CENTER_PARAMS;
      if (intrinsics_opts.focus_shared)
        m_focus_offset = NUM_CENTER_PARAMS;
      else {
        m_num_intrinsics_per_camera += NUM_FOCUS_PARAMS;
        if (!intrinsics_opts.center_shared)
          m_focus_offset = NUM_CENTER_PARAMS;
      }
      if (intrinsics_opts.distortion_shared)
        m_distortion_offset = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS;
      else {
        m_num_intrinsics_per_camera += max_num_dist_params;
        if (!intrinsics_opts.center_shared)
          m_distortion_offset += NUM_CENTER_PARAMS;
        if (!intrinsics_opts.focus_shared)
          m_distortion_offset += NUM_FOCUS_PARAMS;
      }

      // For simplicity, we always set this to the same size and allocate
      // the same storage even if none of the parameters are shared.
      m_num_shared_intrinsics = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS
                                + max_num_dist_params;
      m_intrinsics_vec.resize(m_num_shared_intrinsics +
                              num_cameras*m_num_intrinsics_per_camera,
                              0); // Initialize to 0
    } else {
      m_num_intrinsics_per_camera = NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS
                                    + max_num_dist_params;
      m_intrinsics_vec.resize(intrinsics_opts.num_sensors * m_num_intrinsics_per_camera,
                              0); // Initialize to 0
    }
  }

// Copy constructor
BaParams::BaParams(BaParams const& other):
      m_num_points        (other.m_num_points),
      m_num_cameras       (other.m_num_cameras),
      m_params_per_point  (other.m_params_per_point),
      m_num_pose_params   (other.m_num_pose_params),
      m_num_shared_intrinsics    (other.m_num_shared_intrinsics),
      m_num_intrinsics_per_camera(other.m_num_intrinsics_per_camera),
      m_max_num_dist_params(other.m_max_num_dist_params),
      m_focus_offset      (other.m_focus_offset),
      m_distortion_offset (other.m_distortion_offset),
      m_intrinsics_opts    (other.m_intrinsics_opts),
      m_points_vec        (other.m_points_vec.size()),
      m_cameras_vec       (other.m_cameras_vec.size()),
      m_intrinsics_vec    (other.m_intrinsics_vec.size()),
      m_outlier_points_vec(other.m_outlier_points_vec.size()) {
    copy_points    (other);
    copy_cameras   (other);
    copy_intrinsics(other);
    copy_outliers  (other);
  }

// Set all camera position and pose values to zero.
void BaParams::init_cams_as_zero() {
  for (int i=0; i < m_cameras_vec.size(); i++)
    m_cameras_vec[i] = 0.0;
}

// When using the copy functions, the sizes must match!
/// Copy one set of values from another instance.
void BaParams::copy_points(BaParams const& other) {
  for (size_t i = 0; i < m_points_vec.size(); i++)
    m_points_vec[i] = other.m_points_vec[i];
}
void BaParams::copy_cameras(BaParams const& other) {
  for (size_t i = 0; i < m_cameras_vec.size(); i++)
    m_cameras_vec[i] = other.m_cameras_vec[i];
}
void BaParams::copy_intrinsics(BaParams const& other) {
  for (size_t i = 0; i < m_intrinsics_vec.size(); i++)
    m_intrinsics_vec[i] = other.m_intrinsics_vec[i];
}
void BaParams::copy_outliers(BaParams const& other) {
  for (size_t i = 0; i < m_outlier_points_vec.size(); i++)
    m_outlier_points_vec[i] = other.m_outlier_points_vec[i];
}

// Compute the offset in m_intrinsics_vec to the requested data
size_t BaParams::get_center_offset(int cam_index) const {
  if (!m_intrinsics_opts.share_intrinsics_per_sensor) {
    // Share all or none of the intrinsics
    if (m_intrinsics_opts.center_shared)
      return 0;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera;
  }

  // Share intrinsics per sensor
  int sensor_id = m_intrinsics_opts.cam2sensor.at(cam_index);
  return sensor_id * m_num_intrinsics_per_camera;
}

size_t BaParams::get_focus_offset(int cam_index) const {
  if (!m_intrinsics_opts.share_intrinsics_per_sensor) {
    // Share all or none of the intrinsics
    if (m_intrinsics_opts.focus_shared)
      return m_focus_offset;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera + m_focus_offset;
  } 

  // Share intrinsics per sensor
  int sensor_id = m_intrinsics_opts.cam2sensor.at(cam_index);
  return sensor_id * m_num_intrinsics_per_camera + NUM_CENTER_PARAMS;
}

size_t BaParams::get_distortion_offset(int cam_index) const {
  if (!m_intrinsics_opts.share_intrinsics_per_sensor) {
    // Share all or none of the intrinsics
    if (m_intrinsics_opts.distortion_shared)
      return m_distortion_offset;
    else
      return m_num_shared_intrinsics + cam_index*m_num_intrinsics_per_camera + 
              m_distortion_offset;
  }

  // Share intrinsics per sensor
  int sensor_id = m_intrinsics_opts.cam2sensor.at(cam_index);
  return sensor_id * m_num_intrinsics_per_camera + NUM_CENTER_PARAMS + NUM_FOCUS_PARAMS;
}

/// Apply a random offset to each camera position.
void BaParams::randomize_cameras() {
  // These are stored as x,y,z, axis_angle.
  // - We move the position +/- 5 meters.
  // - Currently we don't adjust the angle.
  boost::random::uniform_int_distribution<> xyz_dist(0, 10);
  const size_t NUM_CAMERA_PARAMS = 6;
  VW_ASSERT((m_cameras_vec.size() % NUM_CAMERA_PARAMS) == 0,
            vw::LogicErr() << "Camera parameter length is not a multiple of 6!");
  const size_t num_cameras = m_cameras_vec.size() / NUM_CAMERA_PARAMS;
  for (size_t c = 0; c < num_cameras; c++) {
    double* ptr = get_camera_ptr(c);
    for (size_t i = 0; i < 3; i++) {
      int diff = xyz_dist(g_rand_gen) - 5;
      ptr[i] += diff;
    }
  }
}

/// Randomly scale each intrinsic value.
void BaParams::randomize_intrinsics(std::vector<double> const& intrinsic_limits) {
  // Intrinsic values are stored as multipliers, here we 
  //  multiply from 0.5 to 1.5, being careful about shared and constant values.
  // - If intrinsic limits are specified, use that range instead.
  boost::random::uniform_int_distribution<> dist(0, 100);
  const double DENOM = 100.0;
  const double DEFAULT_MIN   = 0.5;
  const double DEFAULT_MAX   = 1.5;
  const double DEFAULT_RANGE = DEFAULT_MAX - DEFAULT_MIN;

  const size_t num_intrinsics = intrinsic_limits.size() / 2;
  float percent, scale, range = 0;
  // Iterate over cameras
  for (size_t c = 0; c < num_cameras(); c++) {
    size_t intrinsics_index = 0;
    if (m_intrinsics_opts.float_focal_length(c) &&
        !(m_intrinsics_opts.focus_shared && (c>0))) {
      double* ptr = get_intrinsic_focus_ptr(c);
      for (int i=0; i<NUM_FOCUS_PARAMS; i++) {
        percent = static_cast<double>(dist(g_rand_gen))/DENOM;
        if (intrinsics_index < num_intrinsics) {
          range = intrinsic_limits[2*intrinsics_index+1] - intrinsic_limits[2*intrinsics_index];
          scale = percent*range + intrinsic_limits[2*intrinsics_index];
        } else
          scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
        ptr[i] *= scale;
        ++intrinsics_index;
      }
    } // End focus case
    intrinsics_index = NUM_FOCUS_PARAMS; // In case we did not go through the loop
    if (m_intrinsics_opts.float_optical_center(c) && 
        !(m_intrinsics_opts.center_shared && (c>0))) {
      double* ptr = get_intrinsic_center_ptr(c);
      for (int i = 0; i < NUM_CENTER_PARAMS; i++) {
        percent = static_cast<double>(dist(g_rand_gen))/DENOM;
        if (intrinsics_index < num_intrinsics) {
          range = intrinsic_limits[2*intrinsics_index+1] 
            - intrinsic_limits[2*intrinsics_index];
          scale = percent*range + intrinsic_limits[2*intrinsics_index];
        } else
          scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
        ptr[i] *= scale;
        ++intrinsics_index;
      }
    } // End center case
    intrinsics_index = NUM_FOCUS_PARAMS+NUM_CENTER_PARAMS; // In case we did not go through the loops
    if (m_intrinsics_opts.float_distortion_params(c) && 
        !(m_intrinsics_opts.distortion_shared && (c > 0))) {
      double* ptr = get_intrinsic_distortion_ptr(c);
      for (int i = 0; i < m_max_num_dist_params; i++) {
        percent = static_cast<double>(dist(g_rand_gen))/DENOM;
        if (intrinsics_index < num_intrinsics) {
          range = intrinsic_limits[2*intrinsics_index+1] - intrinsic_limits[2*intrinsics_index];
          scale = percent*range + intrinsic_limits[2*intrinsics_index];
        } else
          scale = percent*DEFAULT_RANGE + DEFAULT_MIN;
        ptr[i] *= scale;
        intrinsics_index++;
      }
    } // End distortion case
  } // End camera loop
}

/// Print stats for optimized ground control points.
void BaParams::print_gcp_stats(std::string const& out_prefix, 
                       vw::ba::ControlNetwork const& cnet,
                       vw::cartography::Datum const& d) const {

  std::string gcp_report = out_prefix + "-gcp-report.txt";
  vw::vw_out() << "Writing: " << gcp_report << std::endl;
  
  std::ofstream gfs(gcp_report.c_str());
  gfs.precision(17); 

  gfs << "# Ground control point report\n";
  // TODO(oalexan1): Print this to report file! Compare with existing report!
  gfs << "# input_gcp optimized_gcp diff\n";
  
  int gcp_count = 0;
  for (int ipt = 0; ipt < num_points(); ipt++) {
  
    if (cnet[ipt].type() != vw::ba::ControlPoint::GroundControlPoint)
      continue;
  
    if (get_point_outlier(ipt))
      continue; // skip outliers

    vw::Vector3 input_gcp = cnet[ipt].position();
    vw::Vector3 opt_gcp   = get_point(ipt);
  
    gfs << "GCP count: " << gcp_count << std::endl;
    gfs << "ECEF: " << input_gcp << ' ' << opt_gcp << ' '
        << input_gcp - opt_gcp << std::endl;

    // Now convert to llh
    input_gcp = d.cartesian_to_geodetic(input_gcp);
    opt_gcp   = d.cartesian_to_geodetic(opt_gcp);

    gfs << "Lon-lat-height: " << input_gcp << ' ' << opt_gcp << ' '
       << input_gcp - opt_gcp << std::endl;
    gfs << "\n";
    gcp_count++;
  }
  gfs.close();
}

void BaParams::record_points_to_kml(const std::string &kml_path,
                            const vw::cartography::Datum& datum,
                            size_t skip, const std::string name) {

  if (datum.name() == asp::UNSPECIFIED_DATUM) {
    vw::vw_out(vw::WarningMessage) << "No datum specified. Cannot write file: "
                                   << kml_path << std::endl;
    return;
  }
  
  // Open the file
  vw::vw_out() << "Writing: " << kml_path << std::endl;
  vw::KMLFile kml(kml_path, name);
  
  // Set up a simple point icon with no labels
  const std::string icon 
    = "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png";
  const bool hide_labels = true;
  kml.append_style( "point", "", 1.0, icon, hide_labels);
  kml.append_style( "point_highlight", "", 1.1, icon, hide_labels);
  kml.append_stylemap( "point_placemark", "point",
                       "point_highlight");
  
  // Loop through the points
  const bool extrude = true;
  for (size_t i=0; i<num_points(); i+=skip) {
    
    if (get_point_outlier(i))
      continue; // skip outliers
    
    // Convert the point to GDC coords
    vw::Vector3 xyz         = get_point(i);
    vw::Vector3 lon_lat_alt = datum.cartesian_to_geodetic(xyz);

    // Add this to the output file
    kml.append_placemark(lon_lat_alt.x(), lon_lat_alt.y(), "", "", "point_placemark",
                         lon_lat_alt[2], extrude);
  }
  kml.close_kml();
}

} // end namespace asp
