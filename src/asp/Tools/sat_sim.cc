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

// Tool to create simulated satellite images and/or pinhole cameras for them.
// See the manual for details.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Sessions/StereoSession.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Geometry/baseUtils.h>
#include <vw/Cartography/CameraBBox.h>

using namespace vw::cartography;
using namespace vw::math;
using namespace vw::geometry;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::GdalWriteOptions {
  std::string dem_file, ortho_file, out_prefix, camera_list;
  vw::Vector3 first, last; // dem pixel and height above dem datum
  int num_cameras;
  vw::Vector2 optical_center, image_size, first_ground_pos, last_ground_pos;
  double focal_length, dem_height_error_tol;
  double roll, pitch, yaw, velocity, jitter_frequency;
  vw::Vector3 horizontal_uncertainty; // for roll, pitch, yaw
  bool no_images;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  double NaN = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("General options");
  general_options.add_options()
    ("dem", po::value(&opt.dem_file)->default_value(""), "Input DEM file.")
    ("ortho", po::value(&opt.ortho_file)->default_value(""), "Input georeferenced image file.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix. All the "
    "files that are saved will start with this prefix.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
     "A file containing the list of pinhole cameras to create synthetic images for. "
     "Then these cameras will be used instead of generating them. Specify one file "
     "per line. The options --first, --last, --num, --focal-length, "
     "and --optical-center will be ignored.")
    ("first", po::value(&opt.first)->default_value(vw::Vector3(), ""),
    "First camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum.")
    ("last", po::value(&opt.last)->default_value(vw::Vector3(), ""),
    "Last camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum.")
    ("num", po::value(&opt.num_cameras)->default_value(0),
    "Number of cameras to generate, including the first and last ones. Must be positive. "
    "The cameras are uniformly distributed along the straight edge from first to last (in "
    "projected coordinates).")
    ("first-ground-pos", po::value(&opt.first_ground_pos)->default_value(vw::Vector2(NaN, NaN), ""),
    "Coordinates of first camera ground footprint center (DEM column and row). "
    "If not set, the cameras will look straight down (perpendicular to along "
    "and across track directions).")
    ("last-ground-pos", po::value(&opt.last_ground_pos)->default_value(vw::Vector2(NaN, NaN), ""),
    "Coordinates of last camera ground footprint center (DEM column and row). "
    "If not set, the cameras will look straight down (perpendicular to along "
    "and across track directions).")
    ("focal-length", po::value(&opt.focal_length)->default_value(NaN),
     "Output camera focal length in units of pixel.")
    ("optical-center", po::value(&opt.optical_center)->default_value(vw::Vector2(NaN, NaN),"NaN NaN"),
     "Output camera optical center (image column and row).")
    ("image-size", po::value(&opt.image_size)->default_value(vw::Vector2(NaN, NaN),
      "NaN NaN"),
      "Output camera image size (width and height).")
    ("roll", po::value(&opt.roll)->default_value(NaN),
    "Camera roll angle, in degrees. See the documentation for more details.")
    ("pitch", po::value(&opt.pitch)->default_value(NaN),
    "Camera pitch angle, in degrees.")
    ("yaw", po::value(&opt.yaw)->default_value(NaN),
    "Camera yaw angle, in degrees.")
    ("velocity", po::value(&opt.velocity)->default_value(NaN),
    "Satellite velocity, in meters per second. Used for modeling jitter. A value of "
    "around 8000 m/s is typical for a satellite like SkySat in Sun-synchronous orbit "
    "(90 minute period) at an altitude of about 450 km. For WorldView, the velocity "
    "is around 7500 m/s, with a higher altitude and longer period.")
    ("horizontal-uncertainty", po::value(&opt.horizontal_uncertainty)->default_value(vw::Vector3(NaN, NaN, NaN)),
    "Camera horizontal uncertainty on the ground, in meters, at nadir orientation. "
    "Specify as three numbers, used for roll, pitch, and yaw. The "
    "angular uncertainty in the camera orientation for each of these angles "
    "is found as tan(angular_uncertainty) "
    "= horizontal_uncertainty / satellite_elevation_above_datum, then converted to degrees.")
    ("jitter-frequency", po::value(&opt.jitter_frequency)->default_value(NaN),
    "Jitter frequency, in Hz. Used for modeling jitter (satellite vibration). "
    "The jitter amplitude will be the angular horizontal uncertainty (see "
    "--horizontal-uncertainty).")
    ("no-images", po::bool_switch(&opt.no_images)->default_value(false)->implicit_value(true),
     "Create only cameras, and no images. Cannot be used with --camera-list.")
    ("dem-height-error-tol", po::value(&opt.dem_height_error_tol)->default_value(0.001),
     "When intersecting a ray with a DEM, use this as the height error tolerance "
     "(measured in meters). It is expected that the default will be always good enough.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("--dem <dem file> --ortho <ortho image file> "
                    "[other options]");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  if (opt.dem_file == "" || opt.ortho_file == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing input DEM and/or ortho image.\n");
  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing output prefix.\n");
  if (std::isnan(opt.image_size[0]) || std::isnan(opt.image_size[1]))
    vw::vw_throw(vw::ArgumentErr() << "The image size must be specified.\n");

  if (opt.camera_list != "" && opt.no_images)
    vw::vw_throw(vw::ArgumentErr() << "The --camera-list and --no-images options "
      "cannot be used together.\n");

  if (opt.camera_list == "") {
    if (opt.first == vw::Vector3() || opt.last == vw::Vector3())
      vw::vw_throw(vw::ArgumentErr() << "The first and last camera positions must be "
        "specified.\n");

    if (opt.first[2] != opt.last[2])
      vw::vw_out() << "Warning: The first and last camera positions have different "
                   << "heights above the datum. This is supported but is not usual. "
                   << "Check your inputs.\n";

    if (opt.num_cameras < 2)
      vw::vw_throw(vw::ArgumentErr() << "The number of cameras must be at least 2.\n");

    // Validate focal length, optical center, and image size
    if (std::isnan(opt.focal_length))
      vw::vw_throw(vw::ArgumentErr() << "The focal length must be positive.\n");
    if (std::isnan(opt.optical_center[0]) || std::isnan(opt.optical_center[1]))
      vw::vw_throw(vw::ArgumentErr() << "The optical center must be specified.\n");

    // Either both first and last ground positions are specified, or none.
    if (std::isnan(norm_2(opt.first_ground_pos)) != 
      std::isnan(norm_2(opt.last_ground_pos)))
      vw::vw_throw(vw::ArgumentErr() << "Either both first and last ground positions "
        "must be specified, or none.\n");

    // Check that either all of roll, pitch, and yaw are specified, or none.
    int ans = int(std::isnan(opt.roll)) +
              int(std::isnan(opt.pitch)) +
              int(std::isnan(opt.yaw));
    if (ans != 0 && ans != 3)
      vw::vw_throw(vw::ArgumentErr() << "Either all of roll, pitch, and yaw must be "
        "specified, or none.\n");
  }

  int ans = int(!std::isnan(opt.jitter_frequency)) +
            int(!std::isnan(opt.velocity)) +
            int(!std::isnan(opt.horizontal_uncertainty[0])) +
            int(!std::isnan(opt.horizontal_uncertainty[1])) +
            int(!std::isnan(opt.horizontal_uncertainty[2]));
  if (ans != 0 && ans != 5)
    vw::vw_throw(vw::ArgumentErr() << "Either all of jitter-frequency, velocity, and "
      "horizontal uncertainty must be specified, or none.\n");
  
  bool have_roll_pitch_yaw = !std::isnan(opt.roll) && !std::isnan(opt.pitch) &&
      !std::isnan(opt.yaw);
  if (ans != 0 && !have_roll_pitch_yaw)
    vw::vw_throw(vw::ArgumentErr() << "Modelling jitter requires specifying --roll, --pitch, and --yaw.\n");

  if (opt.camera_list != "" && ans != 0) 
    vw::vw_throw(vw::ArgumentErr() << "The --camera-list, --jitter-frequency, "
      "--velocity, and --horizontal-uncertainty options cannot be used together.\n");

  if (opt.velocity <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The satellite velocity must be positive.\n");

  if (opt.horizontal_uncertainty[0] < 0 || opt.horizontal_uncertainty[1] < 0 ||
      opt.horizontal_uncertainty[2] < 0)
    vw::vw_throw(vw::ArgumentErr() << "The horizontal uncertainty must be non-negative.\n");

  if (opt.jitter_frequency <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The jitter frequency must be positive.\n");

  // Create the output directory based on the output prefix
  vw::create_out_dir(opt.out_prefix);

  return;
}

// A function that will read a geo-referenced image, its nodata value,
// and the georeference, and will return a PixelMasked image, the nodata
// value, and the georeference.
// TODO(oalexan1): May need to move this to a more general place.
void readGeorefImage(std::string const& image_file, 
  float & nodata_val, vw::cartography::GeoReference & georef,
  vw::ImageViewRef<vw::PixelMask<float>> & masked_image) {

  // Initial value, in case the image has no nodata field
  nodata_val = std::numeric_limits<float>::quiet_NaN();
  if (!vw::read_nodata_val(image_file, nodata_val))
        vw::vw_out() << "Warning: Could not read the nodata value for: "
                      << image_file << "\nUsing: " << nodata_val << ".\n";

    // Read the image
    vw::vw_out() << "Reading: " << image_file << std::endl;
    vw::DiskImageView<float> image(image_file);
    // Create the masked image
    masked_image = vw::create_mask(image, nodata_val);

    // Read the georeference, and throw an exception if it is missing
    bool has_georef = vw::cartography::read_georeference(georef, image_file);
    if (!has_georef)
      vw::vw_throw(vw::ArgumentErr() << "Missing georeference in: "
                                     << image_file << ".\n");
}

// A function to convert from projected coordinates to ECEF
vw::Vector3 projToEcef(vw::cartography::GeoReference const& georef,
                vw::Vector3                          const& proj) {
  vw::Vector3 llh = georef.point_to_geodetic(proj);
  vw::Vector3 ecef = georef.datum().geodetic_to_cartesian(llh);
  return ecef;
}

// Compute point on trajectory and along and across track normalized vectors in
// ECEF coordinates, given the first and last proj points and a value t giving
// the position along this line
void calcTrajPtAlongAcross(vw::Vector3 const& first_proj,
                           vw::Vector3 const& last_proj,
                           vw::cartography::GeoReference const& dem_georef,
                           double t,
                           double delta,
                           vw::Vector3 const& proj_along,
                           vw::Vector3 const& proj_across,
                           // Outputs
                           vw::Vector3 & P,
                           vw::Vector3 & along,
                           vw::Vector3 & across) {

    P = first_proj * (1.0 - t) + last_proj * t; // traj

    // Use centered diffrerence to compute the along and across track points
    // This achieves higher quality results

    vw::Vector3 L1 = P - delta * proj_along; // along track point
    vw::Vector3 C1 = P - delta * proj_across; // across track point
    vw::Vector3 L2 = P + delta * proj_along; // along track point
    vw::Vector3 C2 = P + delta * proj_across; // across track point

    // Convert to cartesian
    P = projToEcef(dem_georef, P);
    L1 = projToEcef(dem_georef, L1);
    C1 = projToEcef(dem_georef, C1);
    L2 = projToEcef(dem_georef, L2);
    C2 = projToEcef(dem_georef, C2);

    // Create the along track and across track vectors
    along = L2 - L1;
    across = C2 - C1;

    // Normalize
    along = along/norm_2(along);
    across = across/norm_2(across);

    // Ensure that across is perpendicular to along
    across = across - dot_prod(along, across) * along;

    // Normalize again
    across = across / norm_2(across);
}

// Assemble the cam2world matrix
void assembleCam2WorldMatrix(vw::Vector3 const& along, 
                             vw::Vector3 const& across, 
                             vw::Vector3 const& down,
                             // Output
                             vw::Matrix3x3 & cam2world) {

  // The camera to world rotation has these vectors as the columns
  for (int row = 0; row < 3; row++) {
    cam2world(row, 0) = along[row];
    cam2world(row, 1) = across[row];
    cam2world(row, 2) = down[row];
  }
 return;
}

// Given an orbit given by the first and last camera center positions in
// projected coordinates, a real number t describing the position along this
// line, roll, pitch, and yaw for the camera (relative to nadir), find the z
// direction for the camera (camera look), intersect it with the ground, find
// the DEM pixel location, and return the distance from this location to a given
// pixel location.
double demPixelErr(Options const& opt,
                   vw::cartography::GeoReference const& dem_georef,
                   vw::ImageViewRef<vw::PixelMask<float>> dem,
                   vw::Vector3 const& first_proj,
                   vw::Vector3 const& last_proj,
                   vw::Vector3 const& proj_along,
                   vw::Vector3 const& proj_across,
                   double t,
                   double delta, // a small number to move along track
                   double roll, double pitch, double yaw,
                   vw::Vector2 const& pixel_loc) {

    // Calc position along the trajectory and normalized along and across vectors
    // in ECEF
    vw::Vector3 P, along, across;
    calcTrajPtAlongAcross(first_proj, last_proj, dem_georef, t, delta,
                          proj_along, proj_across, 
                          // Outputs
                          P, along, across);

    // Find the z vector as perpendicular to both along and across
    vw::Vector3 down = vw::math::cross_prod(along, across);
    down = down / norm_2(down);

    // The camera to world rotation has these vectors as the columns
    vw::Matrix3x3 cam2world;
    assembleCam2WorldMatrix(along, across, down, cam2world);
    // Apply the roll-pitch-yaw rotation
    vw::Matrix3x3 R = asp::rollPitchYaw(roll, pitch, yaw);
    cam2world = cam2world * R;

    // Ray from camera to ground going through image center
    vw::Vector3 cam_dir = cam2world * vw::Vector3(0, 0, 1);

    // Find the intersection of this ray with the ground
    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double max_abs_tol = std::min(opt.dem_height_error_tol, 1e-14);
    double max_rel_tol = max_abs_tol;
    int num_max_iter = 100;
    vw::Vector3 xyz;
    xyz = vw::cartography::camera_pixel_to_dem_xyz
      (P, cam_dir, dem,
        dem_georef, treat_nodata_as_zero,
        has_intersection, opt.dem_height_error_tol, 
        max_abs_tol, max_rel_tol, 
        num_max_iter, xyz);

    // Convert to llh
    vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);

    // Find pixel location 
    vw::Vector2 pixel_loc2 = dem_georef.lonlat_to_pixel
      (subvector(llh, 0, 2));

    // Return norm of difference to input pixel location
    return norm_2(pixel_loc - pixel_loc2);
}

// A model with the error given by demPixelErr(). The variable will be t,
// which will give the position along the trajectory.
class RayDemPixelLMA : public vw::math::LeastSquaresModelBase<RayDemPixelLMA> {

  Options const& m_opt;
  vw::cartography::GeoReference const& m_dem_georef;
  vw::ImageViewRef<vw::PixelMask<float>> m_dem;
  vw::Vector3 m_first_proj;
  vw::Vector3 m_last_proj;
  vw::Vector3 m_proj_along;
  vw::Vector3 m_proj_across;
  double m_delta;
  double m_roll, m_pitch, m_yaw;
  vw::Vector2 m_pixel_loc;

public:
  typedef vw::Vector<double, 1> result_type;
  typedef vw::Vector<double, 1> domain_type;
  typedef vw::Matrix<double>    jacobian_type; ///< Jacobian form. Auto.

  /// Constructor
  RayDemPixelLMA(Options const& opt,
                 vw::cartography::GeoReference const& dem_georef,
                 vw::ImageViewRef<vw::PixelMask<float>> dem,
                 vw::Vector3 const& first_proj,
                 vw::Vector3 const& last_proj,
                 vw::Vector3 const& proj_along,
                 vw::Vector3 const& proj_across,
                 double delta, // a small number to move along track
                 double roll, double pitch, double yaw,
                 vw::Vector2 const& pixel_loc):
    m_opt(opt), m_dem_georef(dem_georef), m_dem(dem),
    m_first_proj(first_proj), m_last_proj(last_proj),
    m_proj_along(proj_along), m_proj_across(proj_across),
    m_delta(delta), m_roll(roll), m_pitch(pitch), m_yaw(yaw),
    m_pixel_loc(pixel_loc){}

  // Evaluator operator. The goal is described earlier.
  inline result_type operator()(domain_type const& t) const {

    double err = demPixelErr(m_opt, m_dem_georef, m_dem,
                             m_first_proj, m_last_proj,
                             m_proj_along, m_proj_across,
                             t[0], m_delta, m_roll, m_pitch, m_yaw,
                             m_pixel_loc);

    result_type result;
    result[0] = err;
    return result;
  }
};

// Find the location of camera center along the trajectory, in projected
// coordinates, so that the ray from the camera center to the ground goes
// closest to given ground point.
void findBestProjCamLocation
  (Options const& opt,
   vw::cartography::GeoReference const& dem_georef,
   vw::ImageViewRef<vw::PixelMask<float>> dem,
   vw::Vector3 const& first_proj, vw::Vector3 const& last_proj,
   vw::Vector3 const& proj_along, vw::Vector3 const& proj_across,
   double delta, double roll, double pitch, double yaw,
   vw::Vector2 const& pixel_loc,
   // Outputs
   vw::Vector3 & best_proj) {

  // Set up the LMA problem
  RayDemPixelLMA model(opt, dem_georef, dem, first_proj, last_proj,
                       proj_along, proj_across, delta, roll, pitch, yaw,
                       pixel_loc);

  // Solve for the best location
  int status = -1;
  double max_abs_tol = 1e-14;
  double max_rel_tol = max_abs_tol;
  int num_max_iter = 100;
  vw::Vector<double, 1> observation; 
  observation[0] = 0; // because we want to minimize the error

  Vector<double, 1> len; len[0] = 0; // initial guess 
    len = vw::math::levenberg_marquardt(model, len, observation, status, 
      max_abs_tol, max_rel_tol, num_max_iter);

  // Note: The status is ignored here. We will just take whatever the solver
  // outputs, as it may not converge within tolerance. May need to do a 
  // sanity check on the output, maybe.

  // Compute the best location given the just-found position on the segment
  double t = len[0];
  best_proj = first_proj * (1.0 - t) + last_proj * t;
}

// A function to compute orbit length in ECEF given its endpoints in projected
// coordinates. Use 100,000 samples along the orbit. Should be enough.
double calcOrbitLength(vw::Vector3 const& first_proj,
                       vw::Vector3 const& last_proj,
                       vw::cartography::GeoReference const& dem_georef) {

  // Number of samples along the orbit and corresponding segments                     
  int num = 100000;  

  // Start of each segment
  vw::Vector3 beg = projToEcef(dem_georef, first_proj);
  // End of each segment
  vw::Vector3 end = beg;
  double orbitLength = 0.0;

  for (int i = 1; i < num; i++) { // note we start at 1

    double t = double(i) / double(num - 1); 
    // Find the projected position of the current point
    vw::Vector3 curr_proj = first_proj + t * (last_proj - first_proj);
    // Find the ECEF position of the current point
    end = projToEcef(dem_georef, curr_proj);

    // Add the length of the segment
    orbitLength += norm_2(end - beg);
    // Move to the next segment
    beg = end;
  }

  return orbitLength;
}

// A function that will take as input the endpoints and will compute the
// satellite trajectory and along track/across track/down directions in ECEF,
// which will give the camera to world rotation matrix.
// The key observation is that the trajectory will be a straight edge in
// projected coordinates so will be computed there first. In some usage
// modes we will adjust the end points of the trajectory along the way.
void calcTrajectory(Options & opt,
                    vw::cartography::GeoReference const& dem_georef,
                    vw::ImageViewRef<vw::PixelMask<float>> dem,
                    // Outputs
                    std::vector<vw::Vector3> & trajectory,
                    // the vector of camera to world rotation matrices
                    std::vector<vw::Matrix3x3> & cam2world) {

  // Convert the first and last camera center positions to projected coordinates
  vw::Vector3 first_proj, last_proj;
  subvector(first_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::math::subvector(opt.first, 0, 2)); // x and y
  first_proj[2] = opt.first[2]; // z
  subvector(last_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::math::subvector(opt.last,  0, 2)); // x and y
  last_proj[2] = opt.last[2]; // z

  // Validate one more time that we have at least two cameras
  if (opt.num_cameras < 2)
    vw::vw_throw(vw::ArgumentErr() << "The number of cameras must be at least 2.\n");

  // Create interpolated DEM with bilinear interpolation with invalid pixel 
  // edge extension
  vw::PixelMask<float> nodata_mask = vw::PixelMask<float>(); // invalid value
  nodata_mask.invalidate();
  auto interp_dem = vw::interpolate(dem, vw::BilinearInterpolation(),
    vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_mask));

  // Direction along the edge in proj coords (along track direction)
  vw::Vector3 proj_along = last_proj - first_proj;
  // Sanity check
  if (proj_along == vw::Vector3())
    vw::vw_throw(vw::ArgumentErr()
      << "The first and last camera positions are the same.\n");
  // Normalize
  proj_along = proj_along / norm_2(proj_along);
  // One more sanity check
  if (std::max(std::abs(proj_along[0]), std::abs(proj_along[1])) < 1e-6)
    vw::vw_throw(vw::ArgumentErr()
      << "It appears that the satellite is aiming for the ground. "
      <<  "Correct the orbit end points.\n");

  // Find the across-track direction, parallel to the ground, in projected coords
  vw::Vector3 proj_across = vw::math::cross_prod(proj_along, vw::Vector3(0, 0, 1));
  proj_across = proj_across / norm_2(proj_across);

  // A small number to help convert directions from being in projected space to
  // ECEF (the transform between these is nonlinear). Do not use a small value,
  // as in ECEF these will be large numbers and we may have precision issues.
  // The value 0.01 was tested well.
  double delta = 0.01; // in meters

  bool have_ground_pos = !std::isnan(norm_2(opt.first_ground_pos)) &&  
      !std::isnan(norm_2(opt.last_ground_pos));
  bool have_roll_pitch_yaw = !std::isnan(opt.roll) && !std::isnan(opt.pitch) &&
      !std::isnan(opt.yaw);

  // Starting point of orbit before we adjust it to match the desired
  // ground locations and roll/pitch/yaw angles.
  vw::Vector3 orig_first_proj = first_proj;

  if (have_ground_pos && have_roll_pitch_yaw) {
    // Find best starting and ending points for the orbit given desired
    // ground locations and roll/pitch/yaw angles.
    vw::Vector3 first_best_cam_loc_proj;
    findBestProjCamLocation(opt, dem_georef, dem, first_proj, last_proj,
                            proj_along, proj_across, delta, 
                            opt.roll, opt.pitch, opt.yaw,
                            opt.first_ground_pos, first_best_cam_loc_proj);
    // Same thing for the last camera
    vw::Vector3 last_best_cam_loc_proj;
    findBestProjCamLocation(opt, dem_georef, dem, first_proj, last_proj,
                            proj_along, proj_across, delta, 
                            opt.roll, opt.pitch, opt.yaw,
                            opt.last_ground_pos, last_best_cam_loc_proj);
    // Overwrite the first and last camera locations in projected coordinates
    // with the best ones
    first_proj = first_best_cam_loc_proj;
    last_proj  = last_best_cam_loc_proj;
  }                  

  // We did a sanity check to ensure that when opt.jitter_frequency is set,
  // opt.velocity and and opt.horizontal_uncertainty are also set and not NaN.
  bool model_jitter = (!std::isnan(opt.jitter_frequency));

  // Find the trajectory, as well as points in the along track and across track 
  // directions in the projected space
  std::vector<vw::Vector3> along_track(opt.num_cameras), across_track(opt.num_cameras);
  trajectory.resize(opt.num_cameras);
  cam2world.resize(opt.num_cameras);
  for (int i = 0; i < opt.num_cameras; i++) {
    double t = double(i) / double(opt.num_cameras - 1);

    // Calc position along the trajectory and normalized along and across vectors
    // in ECEF
    vw::Vector3 P, along, across;
    calcTrajPtAlongAcross(first_proj, last_proj, dem_georef, t, delta,
                          proj_along, proj_across, 
                          // Outputs
                          P, along, across);

    if (have_ground_pos && !have_roll_pitch_yaw) {
      // The camera will be constrained by the ground, but not by the roll/pitch/yaw,
      // then the orientation will change along the trajectory.
      vw::Vector2 ground_pix = opt.first_ground_pos * (1.0 - t) + opt.last_ground_pos * t;

      // Find the projected position along the ground path
      vw::Vector3 ground_proj_pos;
      subvector(ground_proj_pos, 0, 2) = dem_georef.pixel_to_point(ground_pix); // x and y
      auto val = interp_dem(ground_pix[0], ground_pix[1]);
      if (!is_valid(val))
        vw::vw_throw(vw::ArgumentErr() 
          << "Could not interpolate into the DEM along the ground path.\n");
      ground_proj_pos[2] = val.child(); // z

      // Convert the ground point to ECEF
      vw::Vector3 G = projToEcef(dem_georef, ground_proj_pos);

      // Find the ground direction
      vw::Vector3 ground_dir = G - P;
      if (norm_2(ground_dir) < 1e-6)
        vw::vw_throw(vw::ArgumentErr()
          << "The ground position is too close to the camera.\n");

      // Normalize      
      along = along / norm_2(along);
      ground_dir = ground_dir / norm_2(ground_dir);

      // Adjust the along-track direction to make it perpendicular to ground dir
      along = along - dot_prod(ground_dir, along) * ground_dir;

      // Find 'across' as y direction, given that 'along' is x, and 'ground_dir' is z
      across = -vw::math::cross_prod(along, ground_dir);
    }

    // Normalize
    along = along / norm_2(along);
    across = across / norm_2(across);
    // Ensure that across is perpendicular to along
    across = across - dot_prod(along, across) * along;
    // Normalize again
    across = across / norm_2(across);

    // Find the z vector as perpendicular to both along and across
    vw::Vector3 down = vw::math::cross_prod(along, across);
    down = down / norm_2(down);

    // Trajectory
    trajectory[i] = P;

    // The camera to world rotation has these vectors as the columns
    assembleCam2WorldMatrix(along, across, down, cam2world[i]);

    vw::Vector3 amp(0, 0, 0);
    if (model_jitter) {
      // Model the jitter as a sinusoidal motion in the along-track direction
      // Use a different amplitude for roll, pitch, and yaw.

      // Current postion in projected coordinates and height above datum for it
      vw::Vector3 curr_proj = first_proj * (1.0 - t) + last_proj * t;
      double height_above_datum = curr_proj[2];

      // Length of the orbit from starting point, before adjustment for roll,
      // pitch, and yaw. This way when different orbital segments are used, for
      // different roll, pitch, and yaw, d will not always start as 0 at
      // the beginning of each segment.
      double dist = calcOrbitLength(orig_first_proj, curr_proj, dem_georef);

      for (int c = 0; c < 3; c++) {
        // jitter amplitude as angular uncertainty given ground uncertainty
        double a = atan(opt.horizontal_uncertainty[c] / height_above_datum);
        // Covert to degrees
        a = a * 180.0 / M_PI;

        // Distance traveled in orbit from starting point
        double v = opt.velocity; 
        double f = opt.jitter_frequency;
        double T = v / f; // period in meters
        amp[c] = a * sin(dist * 2 * M_PI / T);
      }
    }

    // if to apply a roll, pitch, yaw rotation
    if (have_roll_pitch_yaw) {
      vw::Matrix3x3 R = asp::rollPitchYaw(opt.roll  + amp[0], 
                                          opt.pitch + amp[1], 
                                          opt.yaw   + amp[2]);
      cam2world[i] = cam2world[i] * R;
    }
  }
  return;
}

// Generate a prefix that will be used for image names and camera names
std::string genPrefix(Options const& opt, int i) {
  return opt.out_prefix + "-" + num2str(10000 + i);
}

// A function to read the pinhole cameras from disk
void readCameras(Options const& opt, 
    std::vector<std::string> & cam_names,
    std::vector<vw::camera::PinholeModel> & cams) {

  // Read the camera names
  vw::vw_out() << "Reading: " << opt.camera_list << std::endl;
  asp::read_list(opt.camera_list, cam_names);

  // Sanity check
  if (cam_names.empty())
    vw::vw_throw(vw::ArgumentErr() << "No cameras were found.\n");

  cams.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++)
    cams[i].read(cam_names[i]);
  
  return;
}

// A function to create and save the cameras. Assume no distortion, and pixel
// pitch = 1.
void genCameras(Options const& opt, std::vector<vw::Vector3> const & trajectory,
                  std::vector<vw::Matrix3x3> const & cam2world,
                  // outputs
                  std::vector<std::string> & cam_names,
                  std::vector<vw::camera::PinholeModel> & cams) {

  // Ensure we have as many camera positions as we have camera orientations
  if (trajectory.size() != cam2world.size())
    vw::vw_throw(vw::ArgumentErr()
      << "Expecting as many camera positions as camera orientations.\n");

    cams.resize(trajectory.size());
    cam_names.resize(trajectory.size());
    for (int i = 0; i < int(trajectory.size()); i++) {

      cams[i] = vw::camera::PinholeModel(trajectory[i], cam2world[i],
                                   opt.focal_length, opt.focal_length,
                                   opt.image_size[0], opt.image_size[1]);
      
      std::string camName = genPrefix(opt, i) + ".tsai";
      cam_names[i] = camName;
      vw::vw_out() << "Writing: " << camName << std::endl;
      cams[i].write(camName);
    }
  
  return;
}

// Generate images by projecting rays from the sensor to the ground
void genImages(Options const& opt,
    bool external_cameras,
    std::vector<std::string> const& cam_names,
    std::vector<vw::camera::PinholeModel> const& cams,
    vw::cartography::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> dem,
    vw::cartography::GeoReference const& ortho_georef,
    vw::ImageViewRef<vw::PixelMask<float>> ortho,
    float ortho_nodata_val) {

    // Generate image names from camera names by replacing the extension
    std::vector<std::string> image_names;
    image_names.resize(cam_names.size());
    for (int i = 0; i < int(cam_names.size()); i++) {
      if (external_cameras)
       image_names[i] = opt.out_prefix + "-" 
        + fs::path(cam_names[i]).filename().replace_extension(".tif").string();
      else
        image_names[i] = genPrefix(opt, i) + ".tif";

      vw::vw_out() << "Writing: " << image_names[i] << std::endl;
    }

  // Create interpolated image with bicubic interpolation with invalid pixel 
  // edge extension
  vw::PixelMask<float> nodata_mask = vw::PixelMask<float>(); // invalid value
  nodata_mask.invalidate();
  auto interp_ortho = vw::interpolate(ortho, vw::BicubicInterpolation(),
                                      vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_mask));
  int cols = opt.image_size[0];
  int rows = opt.image_size[1];
  vw::vw_out() << "Generating images.\n";

  // The location where the ray intersects the ground. We will use each obtained
  // location as initial guess for the next ray. This may not be always a great
  // guess, but it is better than starting nowhere. It should work decently
  // if the camera is high, and with a small footprint on the ground.
  vw::Vector3 xyz(0, 0, 0);

  for (size_t i = 0; i < cams.size(); i++) {
    vw::ImageView<vw::PixelMask<float>> image(cols, rows);

    vw::TerminalProgressCallback tpc("Computing: ", image_names[i] + ": ");
    tpc.report_progress(0);
    double inc_amount = 1.0 / double(cols);

    for (int col = 0; col < cols; col++) {
      for (int row = 0; row < rows; row++) {

        // Start with an invalid pixel
        image(col, row) = vw::PixelMask<float>();
        image(col, row).invalidate();
        vw::Vector2 pix(col, row);

        vw::Vector3 cam_ctr = cams[i].camera_center(pix);
        vw::Vector3 cam_dir = cams[i].pixel_to_vector(pix);

        // Intersect the ray going from the given camera pixel with a DEM
        // Use xyz as initial guess and overwrite it with the new value
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double max_abs_tol = std::min(opt.dem_height_error_tol, 1e-14);
        double max_rel_tol = max_abs_tol;
        int num_max_iter = 100;
        xyz = vw::cartography::camera_pixel_to_dem_xyz
          (cam_ctr, cam_dir, dem,
           dem_georef, treat_nodata_as_zero,
           has_intersection, opt.dem_height_error_tol, 
           max_abs_tol, max_rel_tol, 
           num_max_iter, xyz);

        if (!has_intersection) 
          continue; // will result in nodata pixels

        // Find the texture value at the intersection point by interpolation.
        // This will result in an invalid value if if out of range or if the
        // image itself has invalid pixels.
        vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);
        vw::Vector2 ortho_pix = ortho_georef.lonlat_to_pixel(vw::Vector2(llh[0], llh[1]));
        image(col, row) = interp_ortho(ortho_pix[0], ortho_pix[1]);
      }

      tpc.report_incremental_progress(inc_amount);
    } 
    tpc.report_finished();

    // Save the image using the block write function
    vw::vw_out() << "Writing: " << image_names[i] << std::endl;
    bool has_georef = false; // the produced image is raw, it has no georef
    bool has_nodata = true;

    block_write_gdal_image(image_names[i], 
      vw::apply_mask(image, ortho_nodata_val), 
      has_georef, ortho_georef, // the ortho georef will not be used
      has_nodata, ortho_nodata_val, // borrow the nodata from ortho
      opt, vw::TerminalProgressCallback("", "\t--> "));
  }  
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Read the DEM
    vw::ImageViewRef<vw::PixelMask<float>> dem;
    float dem_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference dem_georef;
    readGeorefImage(opt.dem_file, dem_nodata_val, dem_georef, dem);

    // Read the ortho image
    vw::ImageViewRef<vw::PixelMask<float>> ortho;
    float ortho_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference ortho_georef;
    readGeorefImage(opt.ortho_file, ortho_nodata_val, ortho_georef, ortho);

    std::vector<std::string> cam_names;
    std::vector<vw::camera::PinholeModel> cams;
    bool external_cameras = false;
    if (!opt.camera_list.empty()) {
      // Read the cameras
      readCameras(opt, cam_names, cams);
      external_cameras = true;
    } else {
     // Generate the cameras   
      std::vector<vw::Vector3> trajectory(opt.num_cameras);
      // vector of rot matrices
      std::vector<vw::Matrix3x3> cam2world(opt.num_cameras);
      calcTrajectory(opt, dem_georef, dem,
        // Outputs
        trajectory, cam2world);
        // Generate cameras
        genCameras(opt, trajectory, cam2world, cam_names, cams);
    }

    // Generate images
    if (!opt.no_images)
      genImages(opt, external_cameras, cam_names, cams, dem_georef, dem, 
        ortho_georef, ortho, ortho_nodata_val);

  } ASP_STANDARD_CATCHES;

  return 0;
}
