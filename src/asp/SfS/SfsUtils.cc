// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// \file SfsUtils.cc
// Basic utilities for SfS

#include <asp/SfS/SfsUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/Core/CmdUtils.h>

#include <string>
#include <map>

namespace asp {

using namespace vw;

// Find lon-lat-height in the center of the DEM
void calcDemCenterLonLatHeight(vw::ImageView<double> const& dem,
                               double nodata_val,
                               vw::cartography::GeoReference const& georef,
                               vw::Vector3 & llh) {

  int cols = dem.cols(), rows = dem.rows();
  if (cols <= 0 || rows <= 0)
    vw_throw(ArgumentErr() << "Expecting a non-empty DEM.\n");

  vw::Vector2 ll = georef.pixel_to_lonlat(Vector2(cols/2.0, rows/2.0));
  double height = dem(cols/2.0, rows/2.0);
  if (height == nodata_val)
    height = 0.0;

  llh = vw::Vector3(ll[0], ll[1], height);
}

// Find the sun azimuth and elevation at the lon-lat position of the
// center of the DEM. The result can change depending on the DEM.
void sunAngles(ImageView<double> const& dem,
               double nodata_val, vw::cartography::GeoReference const& georef,
               vw::Vector3 const& sun_pos,
               double & azimuth, double & elevation) {

  // Find lon-lat-height in the center of the DEM
  vw::Vector3 llh;
  calcDemCenterLonLatHeight(dem, nodata_val, georef, llh);

  vw::Vector3 xyz = georef.datum().geodetic_to_cartesian(llh); // point on the planet

  vw::Vector3 sun_dir = sun_pos - xyz;
  sun_dir = sun_dir / norm_2(sun_dir); // normalize

  // Find the sun direction in the North-East-Down coordinate system
  vw::Matrix3x3 Ned2Ecef = georef.datum().lonlat_to_ned_matrix(llh);
  vw::Vector3 sun_dir_ned = inverse(Ned2Ecef) * sun_dir;

  if (sun_dir_ned[0] == 0 && sun_dir_ned[1] == 0)
    azimuth = 0;
  else
    azimuth = (180.0/M_PI) * atan2(sun_dir_ned[1], sun_dir_ned[0]);

  // azimuth = atan(E/N) in the NED system
  // So, when N = 1 and E = 0, azimuth is 0.

  double L = norm_2(subvector(sun_dir_ned, 0, 2));
  elevation = (180.0/M_PI) * atan2(-sun_dir_ned[2], L);
}

// Convert azimuth and elevation in degrees to Sun position in meters
vw::Vector3 angelsToSunPosition(double azimuth, double elevation,
                                const vw::Vector3& xyz,
                                const vw::Matrix3x3& Ned2Ecef) {
    // Convert to radians
    azimuth   *= M_PI/180.0;
    elevation *= M_PI/180.0;

    // Find the Sun direction in NED
    double n = cos(azimuth) * cos(elevation);
    double e = sin(azimuth) * cos(elevation);
    double d = -sin(elevation);

    // Convert the direction to ECEF
    vw::Vector3 sun_dir = Ned2Ecef * vw::Vector3(n, e, d);

    // The distance to the Sun
    double sun_dist = 149597870700.0; // meters

    // Add to xyz the direction multiplied by the distance
    return xyz + sun_dist * sun_dir;
}

// Read sun positions from a file
void readSunPositions(std::string const& sun_positions_list,
                      std::vector<std::string> const& input_images,
                      vw::ImageView<double> const& dem,
                      double nodata_val,
                      vw::cartography::GeoReference const& georef,
                      std::vector<vw::Vector3> & sun_positions) {

  // Initialize the sun position with something (the planet center)
  int num_images = input_images.size();
  sun_positions.resize(num_images);
  for (int it = 0; it < num_images; it++)
    sun_positions[it] = vw::Vector3();

  // First read the positions in a map, as they may be out of order
  std::map<std::string, vw::Vector3> sun_positions_map;
  std::ifstream ifs(sun_positions_list.c_str());
  std::string filename;
  double x, y, z;
  while (ifs >> filename >> x >> y >> z)
    sun_positions_map[filename] = vw::Vector3(x, y, z);

  // Put the sun positions in sun_positions.
  for (int it = 0; it < num_images; it++) {
    auto map_it = sun_positions_map.find(input_images[it]);
    if (map_it == sun_positions_map.end())
      vw_throw(ArgumentErr() << "Could not read the Sun position from file: "
               << sun_positions_list << " for image: " << input_images[it] << ".\n");

    sun_positions[it] = map_it->second;
  }
}

// Read the sun angles (azimuth and elevation) and convert them to sun positions.
void readSunAngles(std::string const& sun_positions_list,
                   std::vector<std::string> const& input_images,
                   vw::ImageView<double> const& dem,
                   double nodata_val,
                   vw::cartography::GeoReference const& georef,
                   std::vector<vw::Vector3> & sun_positions) {

  // Find lon-lat-height in the center of the DEM
  vw::Vector3 llh;
  calcDemCenterLonLatHeight(dem, nodata_val, georef, llh);

  // Point on the planet
  vw::Vector3 xyz = georef.datum().geodetic_to_cartesian(llh);

  // Find the sun direction in the North-East-Down coordinate system
  vw::Matrix3x3 Ned2Ecef = georef.datum().lonlat_to_ned_matrix(llh);

  // Initialize the sun position with something (the planet center)
  int num_images = input_images.size();
  sun_positions.resize(num_images);
  for (int it = 0; it < num_images; it++)
    sun_positions[it] = vw::Vector3();

  // First read the positions in a map, as they may be out of order
  std::map<std::string, vw::Vector3> sun_positions_map;
  std::ifstream ifs(sun_positions_list.c_str());
  std::string filename;
  double azimuth, elevation;
  while (ifs >> filename >> azimuth >> elevation)
    sun_positions_map[filename] = angelsToSunPosition(azimuth, elevation, xyz, Ned2Ecef);

  // Put the sun positions in sun_positions.
  for (int it = 0; it < num_images; it++) {
    auto map_it = sun_positions_map.find(input_images[it]);
    if (map_it == sun_positions_map.end())
      vw::vw_throw(vw::ArgumentErr() << "Could not read the Sun position from file: "
               << sun_positions_list << " for image: " << input_images[it] << ".\n");

    sun_positions[it] = map_it->second;
  }
}

// Query an ISIS or CSM camera to find the Sun position
vw::Vector3 sunPositionFromCamera(vw::CamPtr camera) {

  // Remove any adjustment to get to the camera proper
  vw::CamPtr ucam = unadjusted_model(camera);

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  // Try isis
  vw::camera::IsisCameraModel* isis_cam
    = dynamic_cast<vw::camera::IsisCameraModel*>(ucam.get());
  if (isis_cam != NULL)
    return isis_cam->sun_position();
#endif // ASP_HAVE_PKG_ISIS

  // Try csm
  asp::CsmModel* csm_cam = dynamic_cast<asp::CsmModel*>(ucam.get());
  if (csm_cam != NULL)
    return csm_cam->sun_position();

  // No luck. Later there will be a complaint.
  return vw::Vector3();
}

std::string exposureFileName(std::string const& prefix) {
  return prefix + "-exposures.txt";
}

std::string hazeFileName(std::string const& prefix) {
  return prefix + "-haze.txt";
}

std::string modelCoeffsFileName(std::string const& prefix) {
  return prefix + "-model_coeffs.txt";
}

std::string skippedImagesFileName(std::string const& prefix) {
  return prefix + "-skipped_images.txt";
}

std::string usedImagesFileName(std::string const& prefix) {
  return prefix + "-used_images.txt";
}

// Save the exposures to a file
void saveExposures(std::string const& out_prefix,
                   std::vector<std::string> const& input_images,
                   std::vector<double> const& exposures) {
  std::string exposure_file = exposureFileName(out_prefix);
  vw_out() << "Writing: " << exposure_file << std::endl;
  std::ofstream exf(exposure_file.c_str());
  exf.precision(17);
  for (size_t image_iter = 0; image_iter < exposures.size(); image_iter++)
    exf << input_images[image_iter] << " " << exposures[image_iter] << "\n";
  exf.close();
}

// Save the haze to a file
void saveHaze(std::string const& out_prefix,
              std::vector<std::string> const& input_images,
              std::vector<std::vector<double>> const& haze) {

  std::string haze_file = hazeFileName(out_prefix);
  vw_out() << "Writing: " << haze_file << std::endl;
  std::ofstream hzf(haze_file.c_str());
  hzf.precision(17);
  for (size_t image_iter = 0; image_iter < haze.size(); image_iter++) {
    hzf << input_images[image_iter];
    for (size_t hiter = 0; hiter < haze[image_iter].size(); hiter++)
      hzf << " " << haze[image_iter][hiter];

    hzf << "\n";
  }
  hzf.close();
}

// Save the skipped images list, these images had no/bad exposure
//  so no good data for the DEM in them
void saveSkippedImages(std::string const& out_prefix,
                       std::vector<std::string> const& input_images) {
  std::string skipped_file = skippedImagesFileName(out_prefix);
  vw_out() << "Writing: " << skipped_file << std::endl;
  std::ofstream skippedfile(skipped_file.c_str());
  for (const auto& image : input_images)
    skippedfile << image << "\n";
}

// Save the used images list, these images had good exposure
void saveUsedImages(std::string const& out_prefix,
                    std::vector<std::string> const& input_images) {
  std::string used_file = usedImagesFileName(out_prefix);
  vw_out() << "Writing: " << used_file << std::endl;
  std::ofstream usedfile(used_file.c_str());
  for (const auto& image : input_images)
    usedfile << image << "\n";
}

// Get the memory usage for the given process. This is for debugging, not used
// in production code. It does not work on OSX.
void callTop() {

  std::ostringstream os;
  int pid = getpid();
  os << pid;

  std::string cmd = "top -b -n 1 | grep -i ' sfs' | grep -i '" + os.str() + "'";
  std::string ans = vw::exec_cmd(cmd.c_str());
  vw_out() << "Memory usage: " << cmd << " " << ans << "\n";
}

} // end namespace asp
