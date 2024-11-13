// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

// Parse PRISM data and produce CSM camera files

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Camera/PRISM_XML.h>

// TODO(oalexan1): Many of these need to go when the code is modularized
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/CsmModelFit.h>
#include <asp/Camera/SatSim.h>
#include <asp/Core/SatSimBase.h>
#include <asp/Core/CameraTransforms.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options: public vw::GdalWriteOptions {
  std::string dim_file, csm_file;
  
  // Constructor
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add_options()
    ("dim", po::value(&opt.dim_file)->default_value(""),
     "The input PRISM .DIMA file.")
    ("csm", po::value(&opt.csm_file)->default_value(""),
     "The output CSM camera file.")
  ;
  
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // The dim file is required
  if (opt.dim_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing the input .DIMA file.\n" 
                 << usage << general_options);

  // The output file is required
  if (opt.csm_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing the output CSM file.\n");
      
  // TODO(oalexan1): Add logic to log to file     

} // End function handle_arguments

// Given a vector of values and a spacing, check that the values have
// that spacing, with given tolerance.
void checkSpacing(std::vector<double> const& vals, double spacing, double tol, 
                  std::string const& tag) {

  for (size_t i = 1; i < vals.size(); i++) {
    double diff = vals[i] - vals[i-1];
    double err = std::abs(diff - spacing);
    if (err > tol)
      vw::vw_throw(vw::ArgumentErr() << "Expecting all " << tag 
        << " values to be spaced by " << spacing << ". Found a discrepancy of " 
        << err << " seconds at index " << i << ".\n");
  }
  
  return;
}

int main(int argc, char * argv[]) {
    
  Options opt;
  try {
    
    // TODO(oalexan1): This needs to be modularized.
    // TODO(oalexan1): Use the input rol-pitch-yaw values, rather than the
    // cooked-up ones below.
    handle_arguments(argc, argv, opt);
    int ncols = -1, nrows = -1;
    double first_line_time = -1, last_line_time = -1;
    std::vector<vw::Vector3> positions, velocities, rpy; // rpy = roll-pitch-yaw
    std::vector<double> position_times, rpy_times;
    asp::parsePrismXml(opt.dim_file, ncols, nrows, first_line_time, last_line_time,
                       positions, velocities, position_times, rpy, rpy_times);
    
    // TODO(oalexan1): Modularize the block below
    
    // Shift all values closer to the origin to avoid numerical issues with interpolation
    double time_shift = first_line_time;
    first_line_time -= time_shift;
    last_line_time  -= time_shift;
    for (size_t i = 0; i < position_times.size(); i++)
      position_times[i] -= time_shift;
    for (size_t i = 0; i < rpy_times.size(); i++)
      rpy_times[i] -= time_shift;
      
   // WGS84 datum
   vw::cartography::Datum datum("WGS84");
   
   // Position sampling 
   double t0_ephem = position_times[0];
   double dt_ephem = position_times[1] - position_times[0];
   
   // The tolerance should not be too small as the times in seconds can be large.
   // A satellite velocity under 10 km / s would result in movement of less
   // than 1e-2 m in 1e-6 seconds.
   double tol = 1e-6;
   checkSpacing(position_times, dt_ephem, tol, "position");

   // Roll-pitch-yaw sampling   
   double t0_quat = rpy_times[0];
   double dt_quat = rpy_times[1] - rpy_times[0];
   checkSpacing(rpy_times, dt_quat, tol, "roll-pitch-yaw");
   
   double dt_line = (last_line_time - first_line_time) / (nrows - 1.0);
   vw::Vector2 image_size(ncols, nrows);

   // Sanity check to ensure interpolation works later
   if (position_times[0] > first_line_time + tol || 
       position_times.back() < last_line_time - tol)
     vw::vw_throw(vw::ArgumentErr() 
       << "The position time range must encompass the image lines time range.\n");
  if (rpy_times[0] > first_line_time + tol || 
      rpy_times.back() < last_line_time - tol)
    vw::vw_throw(vw::ArgumentErr() 
      << "The roll-pitch-yaw time range must encompass the image lines time range.\n");
   
    // They report 1.939 m focal length
    // <THEORETICAL_RESOLUTION unit="M">2.50</THEORETICAL_RESOLUTION>

    // 7060880 meters from Earth center is = 689,880 meters in elevation
    //        <ELLIPSE_AXIS unit="M">7060880.000000</ELLIPSE_AXIS>
    //  <ELLIPSE_ECCENTRICITY>0.000536</ELLIPSE_ECCENTRICITY>
    //     <ELLIPSE_INCLINATION>1.712797</ELLIPSE_INCLINATION>
    double ht = 689880; // height above Earth's surface, in meters
    double dx = 2.5; // resolution in meters
    double focal_length = ht / dx; // focal length in pixels
   
    // Lon and lat at last position
    vw::Vector3 last_pos = positions.back();
    vw::Vector3 llh = datum.cartesian_to_geodetic(last_pos);
    
    // Create a georeference 
    vw::cartography::GeoReference georef;
    georef.set_datum(datum);
    double scale  = 1.0, false_easting = 0.0, false_northing = 0.0;
    georef.set_stereographic(llh[1], llh[0], scale, false_easting, false_northing);

    double roll = 0.0, pitch = 23.8, yaw = 0.0;    
    std::vector<vw::Matrix3x3> cam2world(positions.size());

    // Iterate over positions
    for (size_t i = 0; i < positions.size(); i++) {
      vw::Vector3 beg_pos = positions[i];
      // Normalized velocity
      vw::Vector3 vel = velocities[i];
      vel = vel / vw::math::norm_2(vel);
      vw::Vector3 end_pos = beg_pos + asp::satSimDelta() * vel;
      
      vw::Vector3 llh = georef.datum().cartesian_to_geodetic(beg_pos);
      
       vw::Vector3 beg_proj 
       = georef.geodetic_to_point(georef.datum().cartesian_to_geodetic(beg_pos));
      vw::Vector3 end_proj
        = georef.geodetic_to_point(georef.datum().cartesian_to_geodetic(end_pos));
        
      vw::Vector3 proj_along, proj_across;
      asp::calcProjAlongAcross(beg_proj, end_proj, proj_along, proj_across);

      vw::Vector3 along, across; // ECEF
      asp::calcEcefAlongAcross(georef, asp::satSimDelta(),
                               proj_along, proj_across, beg_proj,
                               along, across);

      vw::Vector3 down = vw::math::cross_prod(along, across);
      down = down / vw::math::norm_2(down);
      
      // The camera to world rotation has these vectors as the columns
      asp::assembleCam2WorldMatrix(along, across, down, cam2world[i]);
      vw::Matrix3x3 R = asp::rollPitchYaw(roll, pitch, yaw);
      // TODO(oalexan1): It is not clear why below need to take the inverse of the rotation
      // In sat_sim that is not done.
      cam2world[i] = cam2world[i] * R * vw::math::inverse(asp::rotationXY());
        
    }
    
    // Let optical center be half the image size
    vw::Vector2 optical_center = image_size / 2.0;
    // TODO(oalexan1): sensor_id must come from the .DIMA file, must say at least Fwd or Bwd
    std::string sensor_id = "PRISM"; 

    // TODO(oalexan1): Must fill in an additional position based on the last
    // position and velocity, if not enough samples, as otherwise cannot
    // interpolate in time at the last line.  

    asp::CsmModel model;
    asp::populateCsmLinescan(first_line_time, dt_line, 
                             t0_ephem, dt_ephem, 
                             t0_ephem, dt_ephem,  // for poses
                             //t0_quat, dt_quat, // for later
                             focal_length, optical_center, image_size, datum, sensor_id,
                             positions, velocities, cam2world, model);
    
    // Write the camera
    vw::vw_out() << "Writing: " << opt.csm_file << "\n";
    model.saveState(opt.csm_file);
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
