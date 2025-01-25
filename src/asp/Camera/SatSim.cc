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

// Functions used for the sat_sim.cc tool that are not general enough to put
// somewhere else.

#include <asp/Core/SatSimBase.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/Common.h>
#include <asp/Core/DemUtils.h>
#include <asp/Camera/SatSim.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/SyntheticLinescan.h>
#include <asp/Rig/rig_config.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Geometry/baseUtils.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Core/StringUtils.h>
#include <vw/Image/Filter.h>

#include <iomanip>

using namespace vw::cartography;
using namespace vw::math;
using namespace vw::geometry;

namespace fs = boost::filesystem;

namespace asp {

// Compute satellite position, and along / across track normalized vectors in
// ECEF coordinates, given the first and last proj points and a value t giving
// the position along this line. Produced along and across vectors are
// normalized and perpendicular to each other.
void calcEcefTrajPtAlongAcross(vw::Vector3 const& curr_proj,
                               vw::cartography::GeoReference const& dem_georef,
                               double delta,
                               vw::Vector3 const& proj_along,
                               vw::Vector3 const& proj_across,
                               // Outputs
                               vw::Vector3 & P,
                               vw::Vector3 & along,
                               vw::Vector3 & across) {

  // Compute the point on the trajectory, in projected coordinates
  asp::calcEcefAlongAcross(dem_georef, delta, proj_along, proj_across, curr_proj,
                          // Outputs, as vectors in ECEF
                          along, across);

  // Convert the point along trajectory to ECEF
  P  = vw::cartography::projToEcef(dem_georef, curr_proj);
}

// This is used to signify when the algorithm below fails to find a solution
const double g_big_val = 1e+100;

// Calc the camera center, orientation, and ground point for the given projected
// orbit position, roll, pitch, and yaw. The camera center is in ECEF.
// The guess for the xyz ground point will be updated if the solver converges.
bool calcCamPoseAndGroundPt(SatSimOptions const& opt,
                            vw::Vector3 const& curr_proj,
                            vw::cartography::GeoReference const& dem_georef,
                            vw::ImageViewRef<vw::PixelMask<float>> dem,
                            double delta,
                            vw::Vector3 const& proj_along,
                            vw::Vector3 const& proj_across,
                            double roll, double pitch, double yaw,
                            double height_guess,
                            // Outputs
                            vw::Matrix3x3 & cam2world,
                            vw::Vector3 & cam_ctr, 
                            vw::Vector3 & xyz,
                            vw::Vector3 & xyz_guess) {

  vw::Vector3 along, across;
  calcEcefTrajPtAlongAcross(curr_proj, dem_georef, delta,
                            proj_along, proj_across, 
                            // Outputs, perpendicular and normal vectors
                            cam_ctr, along, across);

  // Find the z vector as perpendicular to both along and across
  vw::Vector3 down = vw::math::cross_prod(along, across);
  down = down / norm_2(down);

  // The camera to world rotation
  asp::assembleCam2WorldMatrix(along, across, down, cam2world);
  // Apply the roll-pitch-yaw rotation
  vw::Matrix3x3 R = asp::rollPitchYaw(roll, pitch, yaw);
  cam2world = cam2world * R * asp::rotationXY();

  // Ray from camera to ground going through image center
  // TODO(oalexan1): This must go through the optical center, not the image center.
  vw::Vector3 cam_dir = cam2world * vw::Vector3(0, 0, 1);

  // Find the intersection of this ray with the ground
  bool treat_nodata_as_zero = false;
  bool has_intersection = false;
  double max_abs_tol = std::min(opt.dem_height_error_tol, 1e-14);
  double max_rel_tol = max_abs_tol;
  int num_max_iter = 100;
  xyz = vw::cartography::camera_pixel_to_dem_xyz
    (cam_ctr, cam_dir, dem,
      dem_georef, treat_nodata_as_zero,
      has_intersection, 
      // Below we use a prudent approach. Try to make the solver work
      // hard. It is not clear if this is needed.
      std::min(opt.dem_height_error_tol, 1e-8),
      max_abs_tol, max_rel_tol, 
      num_max_iter, xyz_guess, height_guess);

  // Update the guess if we found a solution
  if (has_intersection) 
    xyz_guess = xyz;

  return has_intersection;
}
  
// Given an orbit given by the first and last camera center positions in
// projected coordinates, a real number t describing the position along this
// line, roll, pitch, and yaw for the camera (relative to nadir), find the z
// direction for the camera (camera look), intersect it with the ground, find
// the DEM pixel location, and return the distance from this location to a given
// pixel location.
double demPixelErr(SatSimOptions const& opt,
                   vw::cartography::GeoReference const& dem_georef,
                   vw::ImageViewRef<vw::PixelMask<float>> dem,
                   vw::Vector3 const& first_proj,
                   vw::Vector3 const& last_proj,
                   vw::Vector3 const& proj_along,
                   vw::Vector3 const& proj_across,
                   double t,
                   double delta, // a small number to move along track
                   double roll, double pitch, double yaw,
                   vw::Vector2 const& pixel_loc,
                   double height_guess,
                   vw::Vector3 & xyz_guess) {

  // Calc position along the trajectory and normalized along and across vectors
  // in ECEF
  vw::Vector3 curr_proj = first_proj * (1.0 - t) + last_proj * t;

  vw::Matrix3x3 cam2world;
  vw::Vector3 cam_ctr, xyz;
  bool success = calcCamPoseAndGroundPt(opt, curr_proj, dem_georef, dem, delta,
                                        proj_along, proj_across, roll, pitch, yaw,
                                        height_guess, cam2world, cam_ctr, xyz, xyz_guess);
  
  if (!success)
    return g_big_val;

  // Convert to llh
  vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);

  // Find pixel location 
  vw::Vector2 pixel_loc2 = dem_georef.lonlat_to_pixel(subvector(llh, 0, 2));

  // If the pixel is outside the DEM, return a big value
  if (!vw::bounding_box(dem).contains(pixel_loc2))
    return g_big_val;

  return norm_2(pixel_loc - pixel_loc2);
}

// A model with the error given by demPixelErr(). The variable will be t,
// which will give the position along the trajectory. 
class RayDemPixelLMA : public vw::math::LeastSquaresModelBase<RayDemPixelLMA> {

  SatSimOptions const& m_opt;
  vw::cartography::GeoReference const& m_dem_georef;
  vw::ImageViewRef<vw::PixelMask<float>> m_dem;
  double m_height_guess;
  vw::Vector3 m_first_proj;
  vw::Vector3 m_last_proj;
  vw::Vector3 m_proj_along;
  vw::Vector3 m_proj_across;
  double m_delta, m_param_scale_factor;
  double m_roll, m_pitch, m_yaw;
  vw::Vector2 m_pixel_loc;
  mutable vw::Vector3 m_xyz_guess; // used to speed up the solver, not thread-safe

public:
  typedef vw::Vector<double, 1> result_type;
  typedef vw::Vector<double, 1> domain_type;
  typedef vw::Matrix<double>    jacobian_type; ///< Jacobian form. Auto.

  /// Constructor
  RayDemPixelLMA(SatSimOptions const& opt,
                 vw::cartography::GeoReference const& dem_georef,
                 vw::ImageViewRef<vw::PixelMask<float>> dem,
                 double height_guess,
                 vw::Vector3 const& first_proj,
                 vw::Vector3 const& last_proj,
                 vw::Vector3 const& proj_along,
                 vw::Vector3 const& proj_across,
                 double delta, // a small number to move along track
                 double param_scale_factor, // to go from optimizer units to t in [0, 1]
                 double roll, double pitch, double yaw,
                 vw::Vector2 const& pixel_loc):
    m_opt(opt), m_dem_georef(dem_georef), m_dem(dem), m_height_guess(height_guess),
    m_first_proj(first_proj), m_last_proj(last_proj),
    m_proj_along(proj_along), m_proj_across(proj_across),
    m_delta(delta), m_param_scale_factor(param_scale_factor),
    m_roll(roll), m_pitch(pitch), m_yaw(yaw),
    m_pixel_loc(pixel_loc), m_xyz_guess(vw::Vector3(0, 0, 0)) {}

  // Evaluator operator. The goal is described earlier.
  inline result_type operator()(domain_type const& len) const {

    // See note where param_scale_factor is defined.
    double t = len[0] * m_param_scale_factor;
    double err = demPixelErr(m_opt, m_dem_georef, m_dem, 
                             m_first_proj, m_last_proj,
                             m_proj_along, m_proj_across,
                             t, m_delta, m_roll, m_pitch, m_yaw, m_pixel_loc,
                             m_height_guess,
                             m_xyz_guess); // will change

    result_type result;
    result[0] = err;
    // vw::vw_out() << "t = " << t << ", err = " << err << "\n";
    return result;
  }
};

// Find the location of camera center along the trajectory, in projected
// coordinates, so that the ray from the camera center to the ground goes
// closest to given ground point.
void findBestProjCamLocation
  (SatSimOptions const& opt,
   vw::cartography::GeoReference const& dem_georef,
   vw::ImageViewRef<vw::PixelMask<float>> dem,
   double height_guess, 
   vw::Vector3 const& first_proj, vw::Vector3 const& last_proj,
   vw::Vector3 const& proj_along, vw::Vector3 const& proj_across,
   double delta, double roll, double pitch, double yaw,
   vw::Vector2 const& pixel_loc,
   // Outputs
   vw::Vector3 & best_proj) {

  // Note(oalexan1): This algorithm had issues with convergence. Let eps = 1e-7.
  // This is used in LevenbergMarquardt.h for numerical differentiation. Need to
  // ensure model(len) and model(len + eps) are sufficiently different. For
  // that, ensure that len and len + eps correspond to points in orbit separated
  // by about 1 meter. That is why, we start with t in [0, 1], which
  // parametrizes the orbital segment between first_proj and last_proj, and
  // parametrize using value len, with t = len * param_scale_factor. 
  double eps = 1e-7;
  vw::Vector3 P1 = vw::cartography::projToEcef(dem_georef, first_proj); // t = 0
  vw::Vector3 P2 = vw::cartography::projToEcef(dem_georef, last_proj);  // t = 1
  double d = norm_2(P2 - P1);
  if (d < 1.0)
    vw::vw_throw(vw::ArgumentErr() 
      << "Ensure that the input orbit end points are at least 1 m apart.\n");
  double param_scale_factor = 1.0 / (eps * d);
#if 0
  // Verification that param_scale_factor is correct
  {
    double l1 = 0, l2 = eps;
    double t1 = param_scale_factor * l1; 
    double t2 = param_scale_factor * l2;
    P1 = vw::cartography::projToEcef(dem_georef, first_proj * (1.0 - t1) + last_proj * t1);
    P2 = vw::cartography::projToEcef(dem_georef, first_proj * (1.0 - t2) + last_proj * t2);
    vw::vw_out() << "Param scale factor is " << param_scale_factor << "\n";
    vw::vw_out() << "Distance must be 1 meter: " << norm_2(P1 - P2) << "\n";
  }
#endif

  // Find a spacing in t that corresponds to 1 km movement in orbit.
  // We will use this to find a good initial guess.
  // This is very fragile code.
  // TODO(oalexan1): Find a robust way of finding an initial guess. Sometimes this fails.
  double dt = 1e-3;
  double t1 = -dt, t2 = dt;
  P1 = vw::cartography::projToEcef(dem_georef, first_proj * (1.0 - t1) + last_proj * t1);
  P2 = vw::cartography::projToEcef(dem_georef, first_proj * (1.0 - t2) + last_proj * t2);
  double slope = norm_2(P2 - P1) / (2*dt);
  double spacing = 1000.0 / slope;
#if 0
  // Verification that spacing is correct
  vw::vw_out() << "Spacing is " << spacing << "\n";
  {
    double t1 = 0, t2 = spacing;
    P1 = vw::cartography::projToEcef(dem_georef, first_proj * (1.0 - t1) + last_proj * t1);
    P2 = vw::cartography::projToEcef(dem_georef, first_proj * (1.0 - t2) + last_proj * t2);
    vw::vw_out() << "Distance must be 100 meters: " << norm_2(P2 - P1) << "\n";
  }
#endif

  // Set up the LMA problem
  RayDemPixelLMA model(opt, dem_georef, dem, height_guess, first_proj, last_proj,
                       proj_along, proj_across, delta, param_scale_factor,
                       roll, pitch, yaw, pixel_loc);
  vw::Vector<double, 1> best_len; best_len[0] = 0; // initial guess 

  // First need to search around for a good initial guess. This is a bug fix.
  // Number of attempts times spacing in m is 1e+8 m, which is 100,000 km. 
  // Enough for any orbit length.
  // vw::vw_out() << "Searching for a good initial guess.\n";
  int attempts = int(1e+8);
  double best_val = g_big_val;
  double best_i = 0;
  for (int i = 0; i < attempts; i++) {
    
    // Move towards the positive direction then the negative one
    double curr_best_val = best_val;
    for (int j = -1; j <= 1; j += 2) {
      double t = spacing * i * j;
      vw::Vector<double, 1> curr_len; 
      curr_len[0] = t / param_scale_factor;
      double val = model(curr_len)[0];
      //vw::vw_out() << "len, val = " << curr_len[0] << ' ' << val << "\n";
      if (val < best_val) {
        best_val = val;
        best_len = curr_len;
        best_i = i * j; // take into account the sign
      }
    }
    
    if (curr_best_val == best_val && curr_best_val < g_big_val) {
      // We are not improving anymore, so so stop here, as otherwise
      // we may be going too far.
      break;
    }

  } // end doing attempts
  
  // Do local refinement. This is necessary as the initial guess may be far from
  // the minimum and the function may be noisy. Will do i with increments of 1,
  // then 0.1, etc., around current best_i. Start with increment of 1 to peek
  // ahead beyond the values where we stopped before, while revisiting some of
  // the values of i as well. This was tested on a bug, so do not modify here
  // lightly.
  attempts = 8;
  for (int attempt = 0; attempt < attempts; attempt++) {
    double delta = 1.0 / pow(10.0, attempt);
    double best_i_init = best_i; // So that loop end points do not change mid-loop
    for (double i = best_i_init - 50*delta; i <= best_i_init + 50*delta; i += delta) {
      double t = spacing * i;
      vw::Vector<double, 1> curr_len; 
      curr_len[0] = t / param_scale_factor;
      double val = model(curr_len)[0];
      //vw::vw_out() << "len, val = " << curr_len[0] << ' ' << val << "\n";
      if (val < best_val) {
        best_val = val;
        best_len = curr_len;
        best_i = i;
      }
    }
  }
  
  // Run the optimization with the just-found initial guess
  // vw::vw_out() << "Running the solver.\n";
  int status = -1;
  double max_abs_tol = 1e-14;
  double max_rel_tol = max_abs_tol;
  int num_max_iter = 100;
  vw::Vector<double, 1> observation; 
  observation[0] = 0; // because we want to minimize the error
  best_len = vw::math::levenberg_marquardt(model, best_len, observation, status, 
                                      max_abs_tol, max_rel_tol, num_max_iter);
  // Note: The status is ignored here. We will just take whatever the solver
  // outputs, as it may not converge within tolerance. 
  double val = model(best_len)[0];
 
#if 0
// Turning this off, as the minimum cost function may be far from zero.
// May need to add some other check here.
  if (std::abs(model(best_len)[0]) > 1.0) {
    vw::vw_out() << "Abs of model value is " << std::abs(model(best_len)[0]) << "\n";
    // vw::vw_throw(vw::ArgumentErr() << "Error: The solver for finding correct ends of "
    //   << "orbital segment did not converge to a good solution. Check your DEM, " 
    //   << "roll, pitch, yaw, and ground path endpoints.\n");
  }
#endif

  // Compute the best location given the just-found position on the segment
  double t = best_len[0] * param_scale_factor;
  best_proj = first_proj * (1.0 - t) + last_proj * t;
}

// A function to compute orbit length in ECEF given its endpoints in projected
// coordinates. 1e+5 points are used to approximate the orbit length. Should be
// enough. This gets slow when using 1e+6 points.
// TODO(oalexan1): This may not be accurate enough for very long orbit segments.
// TODO(oalexan1): The number of samples better depend on orbit length.
double calcOrbitLength(vw::Vector3 const& first_proj, vw::Vector3 const& last_proj,
                       vw::cartography::GeoReference const& dem_georef) {

  int num = 1.0e+5;

  // Start of each segment
  vw::Vector3 beg = vw::cartography::projToEcef(dem_georef, first_proj);
  // End of each segment
  vw::Vector3 end = beg;
  double orbitLength = 0.0;

  for (int i = 1; i < num; i++) { // note we start at 1

    double t = double(i) / double(num - 1); 
    // Find the projected position of the current point
    vw::Vector3 curr_proj = first_proj + t * (last_proj - first_proj);
    // Find the ECEF position of the current point
    end = vw::cartography::projToEcef(dem_georef, curr_proj);

    // Add the length of the segment
    orbitLength += norm_2(end - beg);
    // Move to the next segment
    beg = end;
  }

  return orbitLength;
}

// The camera will be constrained by the ground, but not by the roll/pitch/yaw,
// then the orientation will change along the trajectory. Then adjust the along
// and across directions to reflect this. This will adjust the camera direction
// as well.
void cameraAdjustment(vw::Vector2 const& first_ground_pos,
                      vw::Vector2 const& last_ground_pos,
                      double t,
                      vw::cartography::GeoReference const& dem_georef,
                      vw::ImageViewRef<vw::PixelMask<float>> dem,
                      vw::Vector3 const& P, // camera center
                      // Outputs
                      vw::Vector3 & along, vw::Vector3 & across) {

  // Create interpolated DEM with bilinear interpolation with invalid pixel 
  // edge extension
  vw::PixelMask<float> nodata_mask = vw::PixelMask<float>(); // invalid value
  nodata_mask.invalidate();
  auto interp_dem = vw::interpolate(dem, vw::BilinearInterpolation(),
  vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_mask));

  // The camera will be constrained by the ground, but not by the roll/pitch/yaw,
  // then the orientation will change along the trajectory.
  vw::Vector2 ground_pix = first_ground_pos * (1.0 - t) + last_ground_pos * t;

  // Find the projected position along the ground path
  vw::Vector3 ground_proj_pos;
  subvector(ground_proj_pos, 0, 2) = dem_georef.pixel_to_point(ground_pix); // x and y

  auto val = interp_dem(ground_pix[0], ground_pix[1]);
  if (!is_valid(val))
    vw::vw_throw(vw::ArgumentErr() 
      << "Could not interpolate into the DEM along the ground path.\n");
  ground_proj_pos[2] = val.child(); // z

  // Convert the ground point to ECEF
  vw::Vector3 G = vw::cartography::projToEcef(dem_georef, ground_proj_pos);

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

  // Make these vectors have norm 1, and make across perpendicular to along
  // Should already be that way by now, but do it just in case
  asp::normalizeOrthogonalizeAlongAcross(along, across);
}

// Adjust the orbit end point and set the number of cameras given the frame rate
// This is a bit tricky because need to sample finely the orbit
void adjustForFrameRate(SatSimOptions                  const& opt,
                        vw::cartography::GeoReference const& dem_georef,
                        vw::Vector3                   const& first_proj,
                        // Outputs
                        vw::Vector3                        & last_proj, // in/out
                        int                                & num_cameras) {

  // Initialize the outputs, this value will change
  num_cameras = 0;
  
  // Orbit length in meters. Throw an error if getting an orbit of length 0,
  // as that suggests there was a failure in finding in orbit end points.
  double orbit_len = calcOrbitLength(first_proj, last_proj, dem_georef);
  if (orbit_len <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "Failure in computing orbit end points.\n");
  double period = opt.velocity / opt.frame_rate;

  // It is important to let the user know this
  vw::vw_out() << std::setprecision(17) 
      << "Distance between successive cameras = velocity / frame_rate = "
      << period << " meters.\n";

  // Number of cameras. Add 1 because we need to include the last camera.
  num_cameras = int(orbit_len / period) + 1;

  // Cannot have one camera sample for linescan regardless of what the user wants
  bool is_linescan = (opt.sensor_type.find("linescan") != std::string::npos);
  
  // Sanity checks. It is fine to have one single camera, but that is not usual.
  if (num_cameras < 1)
    vw::vw_throw(vw::ArgumentErr() << "The number of cameras must be at least 1.\n");
  if (num_cameras == 1) {
    if (is_linescan) {
      vw::vw_out(vw::WarningMessage) << "For linescan cameras, must have at least "
      "two camera samples. Adding a second one.\n";
      num_cameras = 2;
    } else {
      vw::vw_out(vw::WarningMessage) << "Creating only one frame camera sample.\n";
    }
  }

  // Update the orbit length. Handle gracefully the case of one camera.
  orbit_len = period * std::max(num_cameras - 1.0, 1.0);
  
  // Sanity check, important for the work below. It is fine for first and last 
  // proj to be in the same location, but that is not usual.
  if (norm_2(last_proj - first_proj) < 1e-6)
    vw::vw_out(vw::WarningMessage) << "Warning: The first and last camera positions are "
      <<"too close. Check your inputs.\n";

  // Travel along the orbit in very small increments. Return the last point
  // before exceeding the orbit length. 
  int num = 1000000; // one million samples along the orbit should be enough
  // TODO(oalexan1): Something more clever and faster should be done.
  vw::Vector3 beg = vw::cartography::projToEcef(dem_georef, first_proj);
  vw::Vector3 end = beg;
  vw::Vector3 out_proj = first_proj; // will keep the result here
  double curr_len = 0.0;
  int i = 1;
  while (1) {

    // Find the projected position of the current point
    double t = double(i) / double(num - 1); 
    vw::Vector3 curr_proj = first_proj + t * (last_proj - first_proj);

    // Find the ECEF position of the current point and distance from previous
    end = vw::cartography::projToEcef(dem_georef, curr_proj);
    double curr_dist = norm_2(end - beg);

    if (curr_len + curr_dist > orbit_len)
      break; // done, exceeded orbit length, will keep the previous point in out_proj
    
    curr_len += curr_dist;
    beg = end;
    i++;
    out_proj = curr_proj;

    // Sanity check
    if (i >= 100 * num) {
      vw::vw_out() << "Warning: Could not find the last camera along the orbit. Perhaps the frame rate is too low.\n";
      break;
    }

  }

  // Update the last orbit point, in projected coords. Handle gracefully
  // the case of one camera.
  if (norm_2(last_proj - first_proj) > 1e-6)
    last_proj = out_proj;
}

// Given the direction going from first_proj to last_proj, and knowing
// that we are at curr_proj, find if we are before or after ref_proj.
// Return 1 if after, -1 if before, 0 if at that point. 
double findDirectionAlongOrbit(vw::Vector3 const& ref_proj,
                                vw::Vector3 const& first_proj,
                                vw::Vector3 const& last_proj,
                                vw::Vector3 const& curr_proj) {
  double sign = 0.0;
  vw::Vector3 dir1 = last_proj - first_proj;
  double len1 = norm_2(dir1);
  if (len1 != 0)
    dir1 = dir1 / len1;

  // Handle the case when we are at ref_proj
  vw::Vector3 dir2 = curr_proj - ref_proj;
  double len2 = norm_2(dir2);
  if (len2 == 0)
    return 0;

   dir2 = dir2 / len2;
   if (norm_2(dir1 - dir2) <= norm_2(dir1 + dir2))
     sign = 1.0; // curr_proj is after ref_proj
   else  
    sign = -1.0; // curr_proj is before ref_proj

  return sign;
}

// Given the direction in orbit from first_proj to last_proj (these determine
// the orbit geometry), find the signed distance along the orbit (in ECEF) from
// ref_proj to curr_proj.
double signedOrbitDistance(vw::Vector3 const& curr_proj,
                           vw::Vector3 const& ref_proj,
                           vw::Vector3 const& first_proj,
                           vw::Vector3 const& last_proj,
                           vw::cartography::GeoReference const& dem_georef) {
  double dist = calcOrbitLength(ref_proj, curr_proj, dem_georef);
  double sign = findDirectionAlongOrbit(ref_proj, first_proj, last_proj, curr_proj);
  return sign * dist;
}

// Calc the jitter amplitude at a given location along the orbit. We will
// accumulate over all frequencies. The signed distance is from the reference
// orbit point. Use a different amplitude and phase shift for roll, pitch, and
// yaw. But all these share the same set of frequencies.
vw::Vector3 calcJitterAmplitude(SatSimOptions const& opt,
                                vw::Vector3   const& curr_proj, // curr proj camera pos
                                double signed_dist) { // distance from ref_proj

  double height_above_datum = curr_proj[2]; // curr satellite height
  vw::Vector3 amp(0, 0, 0);

  for (size_t freq_iter = 0; freq_iter < opt.jitter_frequency.size(); freq_iter++) {
    double f = opt.jitter_frequency[freq_iter];
    double v = opt.velocity;
    double T = v / f; // period in meters
    
    // Iterate over roll, pitch, and yaw
    for (int c = 0; c < 3; c++) {
      int index = 3 * freq_iter + c;
      double a = 0.0;
      // We have either horizontal uncertainty or jitter amplitude.
      if (!opt.horizontal_uncertainty.empty()) {
        // jitter amplitude as angular uncertainty given ground uncertainty
        a = atan(opt.horizontal_uncertainty[c] / height_above_datum);
        // Covert to degrees
        a = a * 180.0 / M_PI;
      } else {
        // Amplitude in micro radians
        a = opt.jitter_amplitude[index];
        // Convert to radians
        a = a * 1e-6;
        // Convert to degrees
        a = a * 180.0 / M_PI;
      }
      
      // Compute the jitter, in degrees. Add the phase.
      amp[c] += a * sin(signed_dist * 2.0 * M_PI / T + opt.jitter_phase[index]);
    }

  } // End loop through frequencies

  return amp;
}

// Parse --rig-sensor-ground-offsets. 
void parseSensorGroundOffsets(std::string const& offsets_str,
                              int num_sensors,
                              std::vector<double> & offsets) {
 
 std::string sep = ", \t\n"; // separators: comma, space, tab, newline
 offsets = vw::str_to_std_vec(offsets_str, sep);
 
 // If empty, set to 0.
 if (offsets.empty())
   offsets.resize(4 * num_sensors, 0.0);

 // Must have 4 offsets (2 sensor and 2 ground) per sensor
 if (offsets.size() != 4 * num_sensors)
   vw::vw_throw(vw::ArgumentErr() << "Expecting " << 4 * num_sensors 
     << " offsets, got " << offsets.size() << ".\n"); 
}

// Parse --rig-sensor-rotation-angles.
void parseSensorRotationAngles(std::string const& angles_str,
                               int num_sensors,
                               std::vector<double> & angles) {
 
  std::string sep = ", \t\n"; // separators: comma, space, tab, newline
  angles = vw::str_to_std_vec(angles_str, sep);

  // If empty, set to 0.
  if (angles.empty())
    angles.resize(num_sensors, 0.0);
       
  // Must have 1 angle per sensor
  if (angles.size() != num_sensors)
    vw::vw_throw(vw::ArgumentErr() << "Expecting " << num_sensors 
      << " angles, got " << angles.size() << ".\n");
}

// Change the rig to have desired offsets in the sensor plane and ground plane.
// These will determine the orientations. The offsets are in meters.
void adjustRigForOffsets(SatSimOptions const& opt,
                         vw::cartography::GeoReference const& dem_georef,
                         vw::ImageViewRef<vw::PixelMask<float>> dem,
                         double height_guess,
                         vw::Vector3 const& ref_proj,
                         vw::Vector3 const& proj_along,
                         vw::Vector3 const& proj_across,
                         double delta,
                         rig::RigSet & rig) {

  int num_rig_sensors = rig.cam_names.size();
  std::vector<double> offsets, angles;
  parseSensorGroundOffsets(opt.rig_sensor_ground_offsets, num_rig_sensors,
                           offsets); // output
  parseSensorRotationAngles(opt.rig_sensor_rotation_angles, num_rig_sensors,
                            angles); // output

  // Find the transform from the sensor to the ground at nadir, and the ground point
  vw::Matrix3x3 cam2world;
  vw::Vector3 cam_ctr;
  double roll = 0, pitch = 0, yaw = 0;
  vw::Vector3 xyz = vw::Vector3(0, 0, 0), xyz_guess = vw::Vector3(0, 0, 0);
  bool success = calcCamPoseAndGroundPt(opt, ref_proj, dem_georef, dem, delta,
                                        proj_along, proj_across, roll, pitch, yaw,
                                        height_guess, cam2world, cam_ctr, xyz, xyz_guess);
  if (!success)
   vw::vw_throw(vw::ArgumentErr() << "Could not compute the ground point at nadir.\n");     
  
  // Convert the ground point to camera coordinates
  xyz = inverse(cam2world) * (xyz - cam_ctr);
  
  // Iterate over sensors. For each sensor compute desired sensor center
  // and orientation
  for (int s = 0; s < num_rig_sensors; s++) {
    int offset_index = 4 * s;
    double sensor_offset_x = offsets[offset_index + 0];
    double sensor_offset_y = offsets[offset_index + 1];
    double ground_offset_x = offsets[offset_index + 2];
    double ground_offset_y = offsets[offset_index + 3];
    
    vw::Vector3 sensor_ctr = vw::Vector3(sensor_offset_x, sensor_offset_y, 0);
    vw::Vector3 ground_pt =  xyz + vw::Vector3(ground_offset_x, ground_offset_y, 0);

    // Must offset the ground point relative to the sensor center
    // to have it in sensor coordinates.
    ground_pt = ground_pt - sensor_ctr;
    
    // Need a rotation for the sensor to reflect the direction from sensor to ground
    vw::Vector3 z = normalize(ground_pt);
    
    // This is a minor bugfix. For the sensor that has the trivial orientation,
    // eliminate the numerical noise in the z axis. This way, the results with
    // no rig and with a trivial rig are the same.
    if (sensor_offset_x == 0 && sensor_offset_y == 0 && 
        ground_offset_x == 0 && ground_offset_y == 0 ||
        norm_2(z - vw::Vector3(0, 0, 1.0)) < 1e-10) 
      z = vw::Vector3(0, 0, 1.0);
    
    // Adjust x to be perpendicular to z
    vw::Vector3 x(1, 0, 0);
    x = x - dot_prod(x, z) * z;
    x = normalize(x);
    // y must be perpendicular to x and z
    vw::Vector3 y = cross_prod(z, x);
    
    // Compute the rotation due to offsets
    Eigen::Matrix<double, 4, 4> offset_mat = Eigen::Matrix<double, 4, 4>::Identity();
    for (int r = 0; r < 3; r++) {
      offset_mat(r, 0) = x[r];
      offset_mat(r, 1) = y[r];
      offset_mat(r, 2) = z[r];  
      offset_mat(r, 3) = sensor_ctr[r];
    }

    // Compute the rotation due to angles
    Eigen::Matrix<double, 4, 4> rot_mat = Eigen::Matrix<double, 4, 4>::Identity();
    if (!angles.empty()) {
      double th = angles[s];
      // Convert to radians
      th = th * M_PI / 180.0;
      // Create a rotation by th around the z axis
      Eigen::Matrix3d R;
      R(0, 0) = cos(th); R(0, 1) = -sin(th); R(0, 2) = 0;
      R(1, 0) = sin(th); R(1, 1) = cos(th);  R(1, 2) = 0;
      R(2, 0) = 0;       R(2, 1) = 0;        R(2, 2) = 1;
      rot_mat.block<3, 3>(0, 0) = R;
    }
    
    // Update the rig. Rotations get applied first.
    rig.ref_to_cam_trans[s].matrix() = offset_mat * rot_mat;
  }
  
}

// A function that will take as input the endpoints and will compute the
// positions and orientations in ECEF. The key observation is that the positions
// will form a straight edge in projected coordinates. In some usage modes we
// will adjust the end points of the produced path. The rig may get adjusted
// if --rig-sensor-ground-offsets is set.
void genCamPoses(SatSimOptions & opt,
                 vw::cartography::GeoReference const& dem_georef,
                 vw::ImageViewRef<vw::PixelMask<float>> dem,
                 double height_guess,
                 bool have_rig,
                 // Outputs
                 int                        & first_pos,
                 double                     & first_line_time,
                 double                     & orbit_len,
                 std::vector<vw::Vector3>   & positions,
                 std::vector<vw::Matrix3x3> & cam2world,
                 std::vector<vw::Matrix3x3> & cam2world_no_jitter,
                 std::vector<vw::Matrix3x3> & ref_cam2world,
                 std::vector<double>        & cam_times,
                 rig::RigSet                & rig) {


  // Initialize the outputs. They may change later.
  first_pos = 0;
  orbit_len = 0.0;
  first_line_time = 0.0;
  positions.clear();
  cam2world.clear();
  cam2world_no_jitter.clear();
  ref_cam2world.clear();
  cam_times.clear();

  // Convert the first and last camera center positions to projected coordinates
  vw::Vector3 first_proj, last_proj;
  subvector(first_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::math::subvector(opt.first, 0, 2)); // x and y
  first_proj[2] = opt.first[2]; // z
  subvector(last_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::math::subvector(opt.last,  0, 2)); // x and y
  last_proj[2] = opt.last[2]; // z

  // Direction along the edge in proj coords (along track direction),
  // and then the across track direction
  vw::Vector3 proj_along, proj_across;
  asp::calcProjAlongAcross(first_proj, last_proj, proj_along, proj_across);
  
  // A small number to help convert directions from being in projected space to
  // ECEF (the transform between these is nonlinear). Do not use a small value,
  // as in ECEF these will be large numbers and we may have precision issues.
  // The value 0.01 was tested well.
  double delta = asp::satSimDelta(); // in meters

  bool have_ground_pos = !std::isnan(norm_2(opt.first_ground_pos)) &&  
      !std::isnan(norm_2(opt.last_ground_pos));
  bool have_roll_pitch_yaw = !std::isnan(opt.roll) && !std::isnan(opt.pitch) &&
      !std::isnan(opt.yaw);

  // Projected orbital location when looking straight down at the first ground point.
  // This is the reference location for measuring time and jitter from.
  vw::Vector3 ref_proj = first_proj; // will change
  if (have_ground_pos)
    findBestProjCamLocation(opt, dem_georef, dem, height_guess, first_proj, last_proj,
                            proj_along, proj_across, delta, 
                            0, 0, 0, // roll, pitch, yaw
                            opt.first_ground_pos, 
                            ref_proj); // output

  // Adjust the rig given --rig-sensor-ground-offsets
  if (have_rig && 
      (!opt.rig_sensor_ground_offsets.empty() || !opt.rig_sensor_rotation_angles.empty()))
    adjustRigForOffsets(opt, dem_georef, dem, height_guess, ref_proj,
                        proj_along, proj_across, delta, 
                        rig);

  if (have_ground_pos && have_roll_pitch_yaw) {
    // Find best starting and ending points for the orbit given desired
    // ground locations and roll/pitch/yaw angles.
    // Print a message as this step can take a while
    vw::vw_out() << "Estimating orbital segment endpoints given ground constraints.\n";
    vw::Vector3 first_best_cam_loc_proj;
    vw::Stopwatch sw1;
    sw1.start();
    findBestProjCamLocation(opt, dem_georef, dem, height_guess, first_proj, last_proj,
                            proj_along, proj_across, delta, 
                            opt.roll, opt.pitch, opt.yaw,
                            opt.first_ground_pos, first_best_cam_loc_proj);
    sw1.stop();
    vw::vw_out() << "Elapsed time for starting endpoint: " 
                 << sw1.elapsed_seconds() << " s.\n";
                           
    // Same thing for the last camera
    vw::Stopwatch sw2;
    sw2.start();
    vw::Vector3 last_best_cam_loc_proj;
    findBestProjCamLocation(opt, dem_georef, dem, height_guess, first_proj, last_proj,
                            proj_along, proj_across, delta, 
                            opt.roll, opt.pitch, opt.yaw,
                            opt.last_ground_pos, last_best_cam_loc_proj);
    sw2.stop();
    vw::vw_out() << "Elapsed time for ending endpoint: " 
                 << sw2.elapsed_seconds() << " s.\n";
                           
    // Overwrite the first and last camera locations in projected coordinates
    // with the best ones
    first_proj = first_best_cam_loc_proj;
    last_proj  = last_best_cam_loc_proj;
  }                  

  if (!std::isnan(opt.frame_rate)) {
    // Adjust the orbit end point and set the number of cameras given the frame rate
    adjustForFrameRate(opt, dem_georef, first_proj, 
                      // outputs
                      last_proj, opt.num_cameras);                      
  }

  orbit_len = calcOrbitLength(first_proj, last_proj, dem_georef);

  // Good to print these
  vw::vw_out() << "Orbit length between the first and last inclination-adjusted cameras: " 
     << orbit_len << " meters.\n"; 
  vw::vw_out() << "Number of camera samples: " << opt.num_cameras << "." << "\n";

  // We did a sanity check to ensure that when opt.jitter_frequency is set,
  // opt.velocity and and opt.horizontal_uncertainty are also set and not NaN.
  bool model_jitter = (!std::isnan(opt.jitter_frequency[0]));

  // Find the trajectory, as well as points in the along track and across track 
  // directions in the projected space
  vw::vw_out() << "Computing the camera poses.\n"; 

  // For linescan cameras we want to go beyond the positions and orientations
  // needed for the first and last image line, to have room for interpolation
  // and jitter. For Pinhole cameras we do not need this.
  first_pos = 0;
  int last_pos = opt.num_cameras; // stop before last
  if (opt.sensor_type == "linescan") {
    // Double the number of cameras, half of extra ones going beyond image lines
    if (opt.num_cameras < 2)
      vw::vw_throw(vw::ArgumentErr() << "For linescan cameras, must have at least "
        "two camera samples.\n");
    first_pos = -opt.num_cameras/2;
    last_pos  = 2 * opt.num_cameras + first_pos;
  }

  int total = last_pos - first_pos;
  positions.resize(total);
  cam2world.resize(total);
  cam2world_no_jitter.resize(total);
  ref_cam2world.resize(total);
  cam_times.resize(total, 0.0);

  // Print progress
  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / double(total);
  tpc.report_progress(0);

  // The signed distance from ref_proj to first_proj. Later will add to it the
  // distance from first_proj to the current point. This allows measuring
  // distance and time from ref_proj. Based on this, set the first line time.
  double ref_to_first_signed_dist 
    = signedOrbitDistance(first_proj, ref_proj, first_proj, last_proj, dem_georef);
  if (opt.model_time)
    first_line_time = opt.ref_time + ref_to_first_signed_dist / opt.velocity;
  
  for (int i = 0; i < total; i++) {

    // Parametrize the orbital segment    
    double t = double(i + first_pos) / std::max(double(opt.num_cameras - 1.0), 1.0);

    // Current satellite postion in projected coordinates
    vw::Vector3 curr_proj = first_proj * (1.0 - t) + last_proj * t;

    // Signed distance from ref_proj to curr_proj. 
    double signed_dist = ref_to_first_signed_dist + t * orbit_len;
    
    // Record the time at which the camera is at this position. Must
    // be kept consistent with logic in genLinearCameras().
    cam_times[i] = first_line_time + t * orbit_len / opt.velocity;
    
    // Time better be positive, otherwise it may be tricky to interpret the timestamp
    // with a dash in front.
    if (opt.model_time) {
      if (cam_times[i] <= 0)
       vw::vw_throw(vw::ArgumentErr() << "Time must be positive. Check --reference-time.\n");
      if (cam_times[i] >= 1e+6)
       vw::vw_throw(vw::ArgumentErr() << "Time must be less than 1e+6. Check "
                    << "--reference-time.\n");
    }
      
    // Calc position along the trajectory and normalized along and across vectors
    // in ECEF. Produced along and across vectors are normalized and perpendicular
    // to each other.
    vw::Vector3 P, along, across;
    calcEcefTrajPtAlongAcross(curr_proj, dem_georef, delta,
                              proj_along, proj_across, 
                              // Outputs, in ECEF
                              P, along, across);

    // Adjust the camera if constrained by the ground but not by roll/pitch/yaw
    if (have_ground_pos && !have_roll_pitch_yaw)
      cameraAdjustment(opt.first_ground_pos, opt.last_ground_pos, t, dem_georef, dem, P, 
                       // outputs, will be normalized and perpendicular to each other
                       along, across);
    
    // Find the z vector as perpendicular to both along and across
    vw::Vector3 down = vw::math::cross_prod(along, across);
    down = down / norm_2(down);

    // Camera position in ECEF
    positions[i] = P;
    
    // The camera to world rotation has these vectors as the columns.
    // For now the camera is pointing down.
    asp::assembleCam2WorldMatrix(along, across, down, cam2world[i]);

    // Save this before applying adjustments as below. These two 
    // have some important differences, as can be seen below.
    ref_cam2world[i] = cam2world[i];
    cam2world_no_jitter[i] = cam2world[i];

    vw::Vector3 jitter_amp(0, 0, 0);
    if (model_jitter)
      jitter_amp = calcJitterAmplitude(opt, curr_proj, signed_dist);
    
    // If to apply a roll, pitch, yaw rotation
    if (have_roll_pitch_yaw) {
      vw::Matrix3x3 R = asp::rollPitchYaw(opt.roll  + jitter_amp[0], 
                                          opt.pitch + jitter_amp[1], 
                                          opt.yaw   + jitter_amp[2]);
      cam2world[i] = cam2world[i] * R;
    }
    
    // The rotation without jitter
    vw::Matrix3x3 R0 = vw::math::identity_matrix<3>();
    if (have_roll_pitch_yaw)
      R0 = asp::rollPitchYaw(opt.roll, opt.pitch, opt.yaw);
    cam2world_no_jitter[i] = cam2world_no_jitter[i] * R0;

    // In either case apply the in-plane rotation from camera to satellite frame
    cam2world[i] = cam2world[i] * asp::rotationXY();
    cam2world_no_jitter[i] = cam2world_no_jitter[i] * asp::rotationXY();

    tpc.report_incremental_progress(inc_amount);
  }
  tpc.report_finished();

  return;
}

// Generate a prefix for produced image and camera names.
std::string camPrefix(SatSimOptions const& opt, int iFrame, double timestamp, bool isRef,
                      bool isFrame, std::string const& suffix) {

  std::string ref = ""; 
  if (isRef) 
    ref = "-ref";
  
  // Prepare the timestamp string. If modeling time, will do sprintf with 7
  // digits before dot and 9 after (with the dot, there will be 17 characters in
  // total). This is to ensure that the time is unique. Use fixed precision. Use
  // leading zeros to ensure that the string is always the same length and will
  // be sorted correctly.
  char time[256];
  if (!isFrame) {
    time[0] = '\0'; // empty string
  } else {
    if (!opt.model_time) 
      snprintf(time, sizeof(time), "-%d", 10000 + iFrame); // 5 characters, starts with 1
    else
      snprintf(time, sizeof(time), "-%017.9f", timestamp); // 17 characters, 9 after dot
  }
  
  return opt.out_prefix + time + suffix + ref;
}

// A function to read Pinhole cameras from disk
void readPinholeCameras(SatSimOptions const& opt, 
                        std::vector<std::string> & cam_names,
                        std::vector<vw::CamPtr> & cams) {

  // Read the camera names
  vw::vw_out() << "Reading: " << opt.camera_list << "\n";
  asp::read_list(opt.camera_list, cam_names);

  // Sanity check
  if (cam_names.empty())
    vw::vw_throw(vw::ArgumentErr() << "No cameras were found.\n");

  cams.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++)
    cams[i] = vw::CamPtr(new vw::camera::PinholeModel(cam_names[i]));

  return;
}

// A function to perturb the cameras. A lot of logic is in asp::genCamPoses().
void perturbCameras(SatSimOptions const& opt, 
                    std::string const& suffix,
                    vw::cartography::GeoReference const& georef,
                    std::vector<std::string> const& cam_names,
                    std::vector<vw::CamPtr> & cams) {

  // Must have as many cam names as cameras
  if (cam_names.size() != cams.size())
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many camera names as cameras.\n");
  
  // Previous and current projected camera centers
  vw::Vector3 prev_proj_ctr(0, 0, 0), curr_proj_ctr(0, 0, 0);
  
  // Length along orbit from first to current camera
  double orbitLen = 0.0;
  
  for (int i = 0; i < int(cam_names.size()); i++) {
    
    // Camera must be pinhole
    vw::camera::PinholeModel * pin 
      = dynamic_cast<vw::camera::PinholeModel*>(cams[i].get());
    if (pin == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Expecting a Pinhole camera for: " 
                   << cam_names[i] << "\n");

    vw::Vector3 cam_ctr = pin->camera_center(vw::Vector2());
    vw::Matrix<double,3,3> cam2world = pin->get_rotation_matrix();
    
    vw::Vector3 llh = georef.datum().cartesian_to_geodetic(cam_ctr);

    // It is assume that the satellite points nadir. The camera is attached
    // to the satellite body at potentially an angle.
    vw::Matrix3x3 sat2world = georef.datum().lonlat_to_ned_matrix(llh);
    vw::Matrix3x3 cam2sat = inverse(sat2world) * cam2world;
    
    // The camera center in projected coordinates
    subvector(curr_proj_ctr, 0, 2) = georef.lonlat_to_point(subvector(llh, 0, 2)); 
    curr_proj_ctr[2] = llh[2];
    
    // The orbit length from the first to the current camera
    if (i > 0)
      orbitLen += calcOrbitLength(prev_proj_ctr, curr_proj_ctr, georef);
    
    // Jitter amplitude at the current location
    vw::Vector3 jitter_amp = calcJitterAmplitude(opt, curr_proj_ctr, orbitLen);

    // The jitter vibration is applied to the camera in the satellite frame
    vw::Matrix3x3 R_rpy = asp::rollPitchYaw(jitter_amp[0], jitter_amp[1], jitter_amp[2]);
    cam2world = sat2world * R_rpy * cam2sat;
    
    // Replace the camera rotation
    pin->set_camera_pose(cam2world);
    
    // Will save the camera with the output prefix
    std::string camName = opt.out_prefix + "-" + fs::path(cam_names[i]).filename().string();
    vw::vw_out() << "Writing: " << camName << "\n";
    pin->write(camName);
 
    // The current camera becomes the previous one for next time
    prev_proj_ctr = curr_proj_ctr;
  }

  // No rig
  bool have_rig = false;
  Eigen::Affine3d ref2sensor = Eigen::Affine3d::Identity();

  return;
}

// Check if we do a range
bool skipCamera(int i, SatSimOptions const& opt) {

  if (opt.first_index >= 0 && opt.last_index >= 0 &&
     (i < opt.first_index || i >= opt.last_index))
       return true;
  return false;
}

// Given a transform from ref sensor to world, the ref sensor to current sensor,
// create the transform from current sensor to world. Do it in-place. 
void applyRigTransform(Eigen::Affine3d const & ref_to_sensor,
                       vw::Vector3 & ctr, vw::Matrix3x3 & cam2world) {

  // Create 4x4 transform matrix
  Eigen::Matrix<double, 4, 4> cam2world4x4 = Eigen::Matrix<double, 4, 4>::Identity();
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      cam2world4x4(r, c) = cam2world(r, c);
    }
  }
  for (int r = 0; r < 3; r++) 
    cam2world4x4(r, 3) = ctr[r];

  // Apply the rig transform 
  cam2world4x4 = cam2world4x4 * ref_to_sensor.matrix().inverse(); 
  
  // Extract the rotation and translation
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      cam2world(r, c) = cam2world4x4(r, c);
    }
    ctr[r] = cam2world4x4(r, 3);
  }
  
}

// A function to create and save Pinhole cameras. Assume no distortion, and pixel
// pitch = 1.
void genPinholeCameras(SatSimOptions          const& opt,
                vw::cartography::GeoReference const& dem_georef,
                std::vector<vw::Vector3>      const& positions,
                std::vector<vw::Matrix3x3>    const& cam2world,
                std::vector<vw::Matrix3x3>    const& ref_cam2world,
                std::vector<double>           const& cam_times,
                bool                                 have_rig,
                Eigen::Affine3d               const& ref2sensor,
                std::string                   const& suffix, 
                // Outputs
                std::vector<std::string> & cam_names,
                std::vector<vw::CamPtr>  & cams) {

  // Ensure we have as many camera positions as we have camera orientations
  if (positions.size() != cam2world.size() || positions.size() != ref_cam2world.size()
      || positions.size() != cam_times.size())
    vw::vw_throw(vw::ArgumentErr() 
                 << "Expecting as many camera positions as camera orientations.\n");
  
  cams.resize(positions.size());
  cam_names.resize(positions.size());
  for (int i = 0; i < int(positions.size()); i++) {

    vw::Vector3 P    = positions[i];
    vw::Matrix3x3 R  = cam2world[i];
    vw::Vector3 P0   = positions[i];
    vw::Matrix3x3 R0 = ref_cam2world[i];

    if (have_rig) {
      // Must adjust for the rig
      applyRigTransform(ref2sensor, P, R);
      applyRigTransform(ref2sensor, P0, R0);
    }
    
    // Always create the cameras, but only save them if we are not skipping
    asp::CsmModel * csmPtr = NULL;
    vw::camera::PinholeModel *pinPtr = NULL; 
    vw::cartography::Datum d = dem_georef.datum();
    if (opt.save_as_csm) {
      csmPtr = new asp::CsmModel;
      csmPtr->createFrameModel(opt.image_size[0], opt.image_size[1],
                              opt.optical_center[0], opt.optical_center[1],
                              opt.focal_length, 
                              d.semi_major_axis(), d.semi_minor_axis(),
                              P, R);
      cams[i] = vw::CamPtr(csmPtr); // will own this pointer
    } else {
      pinPtr = new vw::camera::PinholeModel(P, R,
                                            opt.focal_length, opt.focal_length,
                                            opt.optical_center[0], opt.optical_center[1]);
      cams[i] = vw::CamPtr(pinPtr); // will own this pointer
    }

    // This is useful for understanding things in the satellite frame
    vw::camera::PinholeModel pinRefCam;
    asp::CsmModel csmRefCam;
    if (opt.save_ref_cams) {
      if (opt.save_as_csm) 
        csmRefCam.createFrameModel(opt.image_size[0], opt.image_size[1],
                                   opt.optical_center[0], opt.optical_center[1],
                                   opt.focal_length, 
                                   d.semi_major_axis(), d.semi_minor_axis(),
                                   P0, R0);
      else
        pinRefCam = vw::camera::PinholeModel(P0, R0,
                                             opt.focal_length, opt.focal_length,
                                             opt.optical_center[0], opt.optical_center[1]);
    }

    std::string ext;
    if (opt.save_as_csm)
      ext = ".json";
    else
      ext = ".tsai"; 

    // The suffix is used with the rig
    bool isRef = false, isFrame = true;
    std::string camName = camPrefix(opt, i, cam_times[i], isRef, isFrame, suffix) + ext;
    cam_names[i] = camName;

    // Check if we do a range
    if (skipCamera(i, opt)) continue;

    vw::vw_out() << "Writing: " << camName << "\n";
    if (opt.save_as_csm) 
      csmPtr->saveState(camName);
    else
      pinPtr->write(camName);

    if (opt.save_ref_cams) {
      bool isRef = true;
      std::string refCamName = camPrefix(opt, i, cam_times[i], isRef, isFrame, suffix) + ext;
      vw::vw_out() << "Writing: " << refCamName << "\n";
      if (opt.save_as_csm)
        csmRefCam.saveState(refCamName);
      else
        pinRefCam.write(refCamName);
    }
  }

  // Write the list of cameras only if we are not skipping the first camera
  // Otherwise the same file may be written by multiple processes. 
  if (!skipCamera(0, opt)) {
    std::string cam_list = opt.out_prefix + "-cameras" + suffix + ".txt"; 
    vw::vw_out() << "Writing: " << cam_list << "\n";
    asp::write_list(cam_list, cam_names);
  } else {
     // Print a warning message that the list won't be saved
     vw::vw_out(vw::WarningMessage) << "The camera list is saved only when " 
        << "--first-index is 0, to avoid a race condition.\n";
  }

  return;
}

// Bring crops in memory. It greatly helps with multi-threading speed.  
// This function is used only for small tiles, to avoid running out of memory
// (which did happen).
void setupCroppedDemAndOrtho(vw::Vector2 const& image_size,
    vw::CamPtr const& cam,
    vw::ImageViewRef<vw::PixelMask<float>> const& dem,
    vw::cartography::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> const& ortho,
    vw::cartography::GeoReference const& ortho_georef,
    double blur_sigma,
    // Outputs
    vw::ImageView<vw::PixelMask<float>> & crop_dem,
    vw::cartography::GeoReference & crop_dem_georef,
    vw::ImageView<vw::PixelMask<float>> & crop_ortho,
    vw::cartography::GeoReference & crop_ortho_georef) {

    // Find the bounding box of the dem and ortho portions seen in the camera,
    // in projected coordinates
    float mean_gsd = 0.0;    
    bool quick = true; // Assumes a big DEM fully containing the image    
    vw::BBox2 dem_box;
    try {
      // This is a bugfix. The camera_bbox function can fail if we want an
      // extent that does not fit fully with the input DEM/orthoimage.
      // We could use above quick = false, but then we'd get only partial
      // synthetic images, which isn't good.
      dem_box = vw::cartography::camera_bbox(dem, dem_georef, dem_georef,
        cam, image_size[0], image_size[1], mean_gsd, quick);
    } catch (const vw::Exception& e) {
       vw::vw_throw(vw::ArgumentErr() << "sat_sim: Failed to compute a synthetic image. "
       << "The most likely cause is that the desired image is out of bounds given "
       << "the input DEM and orthoimage.\n");
    }

    vw::cartography::GeoTransform d2o(dem_georef, ortho_georef);
    vw::BBox2 ortho_box = d2o.point_to_point_bbox(dem_box);

    // Find the DEM pixel box and expand it in case there was some inaccuracies
    // in finding the box
    vw::BBox2i dem_pixel_box = dem_georef.point_to_pixel_bbox(dem_box);
    int expand = 50;
    dem_pixel_box.expand(expand);
    dem_pixel_box.crop(vw::bounding_box(dem));

    // Same for the ortho
    vw::BBox2i ortho_pixel_box = ortho_georef.point_to_pixel_bbox(ortho_box);
    ortho_pixel_box.expand(expand);
    
    // Adjust for any blur
    int kernel_size = 0;
    if (blur_sigma > 0) {
      kernel_size = vw::compute_kernel_size(blur_sigma);
      ortho_pixel_box.expand(kernel_size); // to avoid edge effects
    }

    ortho_pixel_box.crop(vw::bounding_box(ortho));

    // Crop
    crop_dem = vw::crop(dem, dem_pixel_box);
    crop_dem_georef = crop(dem_georef, dem_pixel_box);

    if (blur_sigma > 0) 
      crop_ortho = vw::crop(vw::gaussian_filter(ortho, blur_sigma), ortho_pixel_box);
    else
      crop_ortho = vw::crop(ortho, ortho_pixel_box);
      
    crop_ortho_georef = crop(ortho_georef, ortho_pixel_box);
}

// Create a synthetic image with multiple threads
typedef vw::ImageView<vw::PixelMask<float>> ImageT;
class SynImageView: public vw::ImageViewBase<SynImageView> {
  
  typedef typename ImageT::pixel_type PixelT;
  SatSimOptions const& m_opt;
  vw::CamPtr m_cam;
  vw::cartography::GeoReference m_dem_georef; // make a copy to be thread-safe
  vw::ImageViewRef<vw::PixelMask<float>> const& m_dem;
  double m_height_guess;
  vw::cartography::GeoReference m_ortho_georef; // make a copy to be thread-safe
  vw::ImageViewRef<vw::PixelMask<float>> const& m_ortho;
  float m_ortho_nodata_val;

public:
  SynImageView(SatSimOptions const& opt,
               vw::CamPtr    const& cam,
               vw::cartography::GeoReference   const& dem_georef,
               vw::ImageViewRef<vw::PixelMask<float>> dem,
               double height_guess,
               vw::cartography::GeoReference   const& ortho_georef,
               vw::ImageViewRef<vw::PixelMask<float>> ortho,
               float ortho_nodata_val):
                m_opt(opt), m_cam(cam), 
                m_dem_georef(dem_georef), m_dem(dem),
                m_height_guess(height_guess),
                m_ortho_georef(ortho_georef), m_ortho(ortho),
                m_ortho_nodata_val(ortho_nodata_val) {}

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef vw::ProceduralPixelAccessor<SynImageView> pixel_accessor;

  inline vw::int32 cols() const { return m_opt.image_size[0]; }
  inline vw::int32 rows() const { return m_opt.image_size[1]; }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()( double/*i*/, double/*j*/, vw::int32/*p*/ = 0 ) const {
    vw::vw_throw(vw::NoImplErr() 
      << "SynImageView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef vw::CropView<vw::ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(vw::BBox2i const& bbox) const {

    // Expand the box a bit, to help with interpolation in the ortho image later
    vw::BBox2i extra_bbox = bbox;
    extra_bbox.expand(10);
    extra_bbox.crop(vw::BBox2i(0, 0, cols(), rows()));
    vw::Vector2 crop_start = extra_bbox.min();
    vw::Vector2 crop_image_size = extra_bbox.max() - extra_bbox.min();

    // Must adjust the camera to be able work with the current box, which 
    // does not necessarily start at (0,0). We only adjust the starting
    // position, and not any other params
    vw::Vector3 translation(0, 0, 0);
    vw::Quat    rotation(vw::math::identity_matrix<3>());
    vw::Vector2 pixel_offset = crop_start;
    double      scale = 1.0;
    vw::CamPtr crop_cam(new vw::camera::AdjustedCameraModel(m_cam, translation, rotation, 
                        pixel_offset, scale));

    // Bring crops in memory. It greatly helps with multi-threading speed.  
    vw::cartography::GeoReference crop_dem_georef; // make a copy to be thread-safe
    vw::ImageView<vw::PixelMask<float>> crop_dem;
    vw::cartography::GeoReference crop_ortho_georef; // make a copy to be thread-safe
    vw::ImageView<vw::PixelMask<float>> crop_ortho;
    setupCroppedDemAndOrtho(crop_image_size,
      crop_cam, m_dem, m_dem_georef, m_ortho, m_ortho_georef, m_opt.blur_sigma,
      // Outputs
      crop_dem, crop_dem_georef, crop_ortho, crop_ortho_georef);

    // Create interpolated image with bicubic interpolation with invalid pixel 
    // edge extension
    vw::PixelMask<float> nodata_mask = vw::PixelMask<float>(); // invalid value
    nodata_mask.invalidate();
    auto interp_ortho = vw::interpolate(crop_ortho, vw::BicubicInterpolation(),
                                        vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_mask));

    // The location where the ray intersects the ground. We will use each obtained
    // location as initial guess for the next ray. This may not be always a great
    // guess, but it is better than starting nowhere. It should work decently
    // if the camera is high, and with a small footprint on the ground.
    vw::Vector3 xyz_guess(0, 0, 0);

    vw::ImageView<result_type> tile(bbox.width(), bbox.height());

    for (int col = bbox.min().x(); col < bbox.max().x(); col++) {
      for (int row = bbox.min().y(); row < bbox.max().y(); row++) {

        // These will use to index into the tile 
        int c = col - bbox.min().x();
        int r = row - bbox.min().y();

        // Start with an invalid pixel
        tile(c, r) = vw::PixelMask<float>();
        tile(c, r).invalidate();

        // Here use the full image pixel indices
        vw::Vector2 pix(col, row);
        // Also use original camera
        vw::Vector3 cam_ctr = m_cam->camera_center(pix);
        vw::Vector3 cam_dir = m_cam->pixel_to_vector(pix);

        // Intersect the ray going from the given camera pixel with a DEM
        // Use xyz_guess as initial guess and overwrite it with the new value
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double max_abs_tol = std::min(m_opt.dem_height_error_tol, 1e-14);
        double max_rel_tol = max_abs_tol;
        int num_max_iter = 100;
        vw::Vector3 xyz = vw::cartography::camera_pixel_to_dem_xyz
          (cam_ctr, cam_dir, crop_dem,
            crop_dem_georef, treat_nodata_as_zero,
            has_intersection, m_opt.dem_height_error_tol, 
            max_abs_tol, max_rel_tol, 
            num_max_iter, xyz_guess, m_height_guess);

        if (!has_intersection)
          continue; // will result in nodata pixels

        // Update the guess for nex time, now that we have a valid intersection point
        xyz_guess = xyz;

        // Find the texture value at the intersection point by interpolation.
        // This will result in an invalid value if if out of range or if the
        // image itself has invalid pixels.
        vw::Vector3 llh = crop_dem_georef.datum().cartesian_to_geodetic(xyz);
        vw::Vector2 ortho_pix = crop_ortho_georef.lonlat_to_pixel
                                 (vw::Vector2(llh[0], llh[1]));
        tile(c, r) = interp_ortho(ortho_pix[0], ortho_pix[1]);
      }
    }

    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, vw::BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

// Generate images by projecting rays from the sensor to the ground
void genImages(SatSimOptions      const& opt,
    bool external_cameras,
    std::vector<std::string>      const& cam_names,
    std::vector<vw::CamPtr>       const& cams,
    std::string                   const& suffix, 
    vw::cartography::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> dem,
    double height_guess,
    vw::cartography::GeoReference const& ortho_georef,
    vw::ImageViewRef<vw::PixelMask<float>> ortho,
    float ortho_nodata_val) {

  vw::vw_out() << "Generating images.\n";

  // Generate image names from camera names by replacing the extension
  std::vector<std::string> image_names;
  image_names.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++) {
    if (external_cameras)
      image_names[i] = opt.out_prefix + "-" 
      + fs::path(cam_names[i]).filename().replace_extension(".tif").string();
    else // just replace the extension
     image_names[i] = fs::path(cam_names[i]).replace_extension(".tif").string();
  }

  for (size_t i = 0; i < cams.size(); i++) {

    // Check if we do a range
    if (skipCamera(i, opt)) continue;

    // Save the image using the block write function with multiple threads
    // Increase the tile size, as otherwise this code becomes very slow
    // since a time-consuming camera box calculation happens in each tile.
    // TODO(oalexan1): Figure out why the slowdown.
    vw::vw_out() << "Writing: " << image_names[i] << "\n";
    bool has_georef = false; // the produced image is raw, it has no georef
    bool has_nodata = true;
    SatSimOptions local_opt = opt;
    local_opt.raster_tile_size = vw::Vector2i(512, 512);
    block_write_gdal_image(image_names[i], 
                           vw::apply_mask(SynImageView(opt, cams[i], dem_georef, dem, 
                                                       height_guess, ortho_georef, 
                                                       ortho, ortho_nodata_val), 
                                          ortho_nodata_val),
                           has_georef, ortho_georef, has_nodata, ortho_nodata_val, 
                           local_opt, vw::TerminalProgressCallback("", "\t--> "));
  }  

  // Write the list of images only if we are not skipping the first camera
  // Otherwise the same file may be written by multiple processes. Also
  // do this only for pinhole cameras, as for linescan there is only one 
  // image/camera rather than a set.
  if (opt.sensor_type == "pinhole") {
    if (!skipCamera(0, opt)) {
      std::string image_list = opt.out_prefix + "-images" + suffix + ".txt"; 
      vw::vw_out() << "Writing: " << image_list << "\n";
      asp::write_list(image_list, image_names);
    } else {
      // Print a warning message that the list won't be saved
      vw::vw_out(vw::WarningMessage) << "The image list is saved only when " 
        << "--first-index is 0, to avoid a race condition.\n";
    }
  }

  return;
}

// If sensor type is one value, make a vector of that value with
// as many entries as sensor names. Otherwise split by comma,
// and there must be as many entries as sensors.
void handleSensorType(int num_sensors, 
                      std::string const& sensor_type, 
                      std::vector<std::string> & sensor_types) { 
  boost::split(sensor_types, sensor_type, boost::is_any_of(","));
  
  // If only one, fill to make num_sensors
  if (sensor_types.size() == 1) {
    std::string val = sensor_types[0];
    sensor_types.clear();
    for (int i = 0; i < num_sensors; i++)
      sensor_types.push_back(val);
  }
  
  // There must be as many as sensors
  if (int(sensor_types.size()) != num_sensors)
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many sensor types as sensors.\n");
  
  // Each must be linescan or pinhole
  for (int i = 0; i < int(sensor_types.size()); i++) {
    if (sensor_types[i] == "frame")
      sensor_types[i] = "pinhole"; // frame is synonymous with pinhole
    if (sensor_types[i] != "linescan" && sensor_types[i] != "pinhole")
      vw::vw_throw(vw::ArgumentErr() << "Expecting sensor type to be linescan "
                   << "or pinhole/frame.\n");
  }
}

// Generate cameras and images for a sensor
void genCamerasImages(float ortho_nodata_val,
            bool have_rig,
            int rig_sensor_index,
            vw::ImageViewRef<vw::PixelMask<float>> dem,
            double height_guess,
            vw::cartography::GeoReference const& ortho_georef,
            vw::ImageViewRef<vw::PixelMask<float>> ortho,
            SatSimOptions                      & opt,
            rig::RigSet                        & rig,
            vw::cartography::GeoReference const& dem_georef,
            std::string                   const& suffix) {

  std::vector<vw::Vector3> positions;
  std::vector<vw::Matrix3x3> cam2world, cam2world_no_jitter, ref_cam2world;
  std::vector<double> cam_times;
  int first_pos = 0; // used with linescan poses, which start before first image line
  double orbit_len = 0.0, first_line_time = 0.0; // will change
  
  // Compute the camera poses. Some logic here is different for linescan and frame
  // cameras, that's why this logic is called individually for each rig sensor,
  // even though if all sensors are of the same time it would be enough to call
  // it once.
  asp::genCamPoses(opt, dem_georef, dem, height_guess, have_rig,
                   // Outputs
                   first_pos, first_line_time, orbit_len, positions, cam2world, 
                   cam2world_no_jitter, ref_cam2world, cam_times, rig);

  // In genCamPoses() the rig may have been updated. Fetch the latest.
  Eigen::Affine3d ref2sensor = Eigen::Affine3d::Identity();
  if (have_rig)
    ref2sensor = rig.ref_to_cam_trans[rig_sensor_index];

  // Sequence of camera names and cameras for one sensor
  std::vector<std::string> cam_names; 
  std::vector<vw::CamPtr> cams;
  // The suffix is needed to distinguish the cameras and images for each rig sensor
  if (opt.sensor_type == "pinhole")
    asp::genPinholeCameras(opt, dem_georef, positions, cam2world, ref_cam2world,
                           cam_times, have_rig, ref2sensor, suffix, cam_names, cams);
  else
    asp::genLinescanCameras(first_line_time, orbit_len, dem_georef, dem, first_pos, 
                            positions, cam2world, cam2world_no_jitter, ref_cam2world, 
                            cam_times, height_guess, have_rig, ref2sensor, suffix, 
                            opt, cam_names, cams);

  bool external_cameras = false;
  if (!opt.no_images)
    asp::genImages(opt, external_cameras, cam_names, cams, suffix, dem_georef, dem, 
        height_guess, ortho_georef, ortho, ortho_nodata_val);

}

// Generate the cameras and images for a rig
void genRigCamerasImages(SatSimOptions          & opt,
            rig::RigSet                         & rig,
            vw::cartography::GeoReference  const& dem_georef,
            vw::ImageViewRef<vw::PixelMask<float>> dem,
            double height_guess,
            vw::cartography::GeoReference  const& ortho_georef,
            vw::ImageViewRef<vw::PixelMask<float>> ortho,
            float ortho_nodata_val) {

  // Sanity checks
  if (opt.rig_config == "")            
    vw::vw_throw(vw::ArgumentErr() << "The rig configuration file must be set.\n");
  
  // Handle sensor types
  std::vector<std::string> sensor_types;
  handleSensorType(rig.cam_names.size(), opt.sensor_type, sensor_types);
  
  std::vector<std::string> sensor_names;
  if (opt.sensor_name == "all")
    sensor_names = rig.cam_names; 
  else
    boost::split(sensor_names, opt.sensor_name, boost::is_any_of(","));
  
  // Map from each sensor in rig.cam_names to index
  std::map<std::string, int> sensor_name2index;
  for (int i = 0; i < int(rig.cam_names.size()); i++)
    sensor_name2index[rig.cam_names[i]] = i;
  
  // Iterate over sensor_names. Check if it is sensor_name2index
  for (size_t sensor_it = 0; sensor_it < sensor_names.size(); sensor_it++) {
    auto it = sensor_name2index.find(sensor_names[sensor_it]);
    if (it == sensor_name2index.end())
      vw::vw_throw(vw::ArgumentErr() << "Sensor: " << sensor_names[sensor_it] 
        << " not found in the rig.\n");
    
    // Pass the intrinsics to a local copy of the options
    SatSimOptions local_opt     = opt;
    int sensor_index            = it->second;
    auto params                 = rig.cam_params[sensor_index];
    local_opt.focal_length      = params.GetFocalLength();
    local_opt.optical_center[0] = params.GetOpticalOffset()[0];
    local_opt.optical_center[1] = params.GetOpticalOffset()[1];
    local_opt.image_size[0]     = params.GetDistortedSize()[0];
    local_opt.image_size[1]     = params.GetDistortedSize()[1];
    local_opt.sensor_type       = sensor_types[sensor_index];
    
    // The transform from the reference sensor to the current sensor
    Eigen::Affine3d ref2sensor = rig.ref_to_cam_trans[sensor_index];
    std::string suffix = "-" + rig.cam_names[sensor_index];
    bool have_rig = true;
    genCamerasImages(ortho_nodata_val, have_rig, sensor_index, dem, height_guess, 
                     ortho_georef, ortho, local_opt, rig, dem_georef, suffix); 
    
  }              
}

// Modify the rig that was used to produce the images to make the sensor transforms
// relative to the first sensor.or. 
void writeRelRig(std::string const& out_prefix, rig::RigSet const& rig) {
  
  rig::RigSet ref_rig = rig;
  for (size_t i = 0; i < rig.ref_to_cam_trans.size(); i++)
    ref_rig.ref_to_cam_trans[i] 
        = rig.ref_to_cam_trans[i] * rig.ref_to_cam_trans[0].inverse();
  
  // Ensure the first transform is identity. Now it may not be precisely it
  // because of numerical errors.
  ref_rig.ref_to_cam_trans[0] = Eigen::Affine3d::Identity();
  
  std::string ref_rig_config = out_prefix + "-rig_config.txt";
  bool have_rig = true;
  rig::writeRigConfig(ref_rig_config, have_rig, ref_rig);
  
}
            
} // end namespace asp
