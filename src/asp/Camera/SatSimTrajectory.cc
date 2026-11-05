// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file SatSimTrajectory.cc
/// Synthetic trajectory generation and orbit/attitude calculation for sat_sim.

#include <asp/Camera/SatSimTrajectory.h>
#include <asp/Camera/SatSim.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Core/CamPoseUtils.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/DemUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Rig/RigConfig.h>

#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Image/Interpolation.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/RandomSet.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Core/Log.h>

using namespace vw;

namespace asp {

void calcEcefTrajPtAlongAcross(vw::Vector3 const& curr_proj,
                               vw::GeoReference const& dem_georef,
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
  P  = vw::projToEcef(dem_georef, curr_proj);
}

// This is used to signify when the algorithm below fails to find a solution
const double g_big_val = 1e+100;

// Calc the camera center, orientation, and ground point for the given projected
// orbit position, roll, pitch, and yaw. The camera center is in ECEF.
// The guess for the xyz ground point will be updated if the solver converges.
bool calcCamPoseAndGroundPt(SatSimOptions const& opt,
                            vw::Vector3 const& curr_proj,
                            vw::GeoReference const& dem_georef,
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
  vw::Vector3 down = vw::cross_prod(along, across);
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
  xyz = vw::camera_pixel_to_dem_xyz
    (cam_ctr, cam_dir, dem,
      dem_georef, treat_nodata_as_zero,
      has_intersection,
      // Below we use a prudent approach. Try to make the solver work
      // hard. It is not clear if this is needed.
      std::min(opt.dem_height_error_tol, 1e-8),
      xyz_guess, height_guess);

  // Update the guess if we found a solution
  if (has_intersection) 
    xyz_guess = xyz;

  return has_intersection;
}
  
// DEM-pixel error from camera center to pixel_loc. Offset s absorbs roll.
vw::Vector2 demPixelErr(SatSimOptions const& opt,
                   vw::GeoReference const& dem_georef,
                   vw::ImageViewRef<vw::PixelMask<float>> dem,
                   vw::Vector3 const& first_proj,
                   vw::Vector3 const& last_proj,
                   vw::Vector3 const& proj_along,
                   vw::Vector3 const& proj_across,
                   double t,
                   double s, // across-track offset (meters)
                   double delta,
                   double roll, double pitch, double yaw,
                   vw::Vector2 const& pixel_loc,
                   double height_guess,
                   vw::Vector3 & xyz_guess) {

  vw::Vector3 curr_proj = first_proj * (1.0 - t) + last_proj * t + s * proj_across;

  vw::Matrix3x3 cam2world;
  vw::Vector3 cam_ctr, xyz;
  bool success = calcCamPoseAndGroundPt(opt, curr_proj, dem_georef, dem, delta,
                                        proj_along, proj_across, roll, pitch, yaw,
                                        height_guess, cam2world, cam_ctr, xyz, xyz_guess);

  if (!success)
    return vw::Vector2(g_big_val, g_big_val);

  vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);
  vw::Vector2 pixel_loc2 = dem_georef.lonlat_to_pixel(subvector(llh, 0, 2));

  if (!vw::bounding_box(dem).contains(pixel_loc2))
    return vw::Vector2(g_big_val, g_big_val);

  return pixel_loc - pixel_loc2;
}

// LMA model for orbital camera placement. Dim 1 optimizes along-track only (s = 0);
// Dim 2 optimizes along-track and across-track (to compensate roll).
template <size_t Dim>
class RayDemPixelLMA: public vw::LeastSquaresModelBase<RayDemPixelLMA<Dim>> {

  SatSimOptions const& m_opt;
  vw::GeoReference const& m_dem_georef;
  vw::ImageViewRef<vw::PixelMask<float>> m_dem;
  double m_height_guess;
  vw::Vector3 m_first_proj;
  vw::Vector3 m_last_proj;
  vw::Vector3 m_proj_along;
  vw::Vector3 m_proj_across;
  double m_delta, m_param_scale_factor, m_across_scale_factor;
  double m_roll, m_pitch, m_yaw;
  vw::Vector2 m_pixel_loc;
  mutable vw::Vector3 m_xyz_guess;

public:
  typedef vw::Vector<double, Dim> result_type;
  typedef vw::Vector<double, Dim> domain_type;
  typedef vw::Matrix<double>      jacobian_type;

  RayDemPixelLMA(SatSimOptions const& opt,
                 vw::GeoReference const& dem_georef,
                 vw::ImageViewRef<vw::PixelMask<float>> dem,
                 double height_guess,
                 vw::Vector3 const& first_proj,
                 vw::Vector3 const& last_proj,
                 vw::Vector3 const& proj_along,
                 vw::Vector3 const& proj_across,
                 double delta,
                 double param_scale_factor,
                 double across_scale_factor,
                 double roll, double pitch, double yaw,
                 vw::Vector2 const& pixel_loc):
    m_opt(opt), m_dem_georef(dem_georef), m_dem(dem), m_height_guess(height_guess),
    m_first_proj(first_proj), m_last_proj(last_proj),
    m_proj_along(proj_along), m_proj_across(proj_across),
    m_delta(delta), m_param_scale_factor(param_scale_factor),
    m_across_scale_factor(across_scale_factor),
    m_roll(roll), m_pitch(pitch), m_yaw(yaw),
    m_pixel_loc(pixel_loc), m_xyz_guess(vw::Vector3(0, 0, 0)) {}

  inline result_type operator()(domain_type const& len) const {
    double t = len[0] * m_param_scale_factor;
    double s = (Dim == 2) ? (len[1] * m_across_scale_factor) : 0.0;
    vw::Vector2 err = demPixelErr(m_opt, m_dem_georef, m_dem,
                                  m_first_proj, m_last_proj,
                                  m_proj_along, m_proj_across,
                                  t, s, m_delta, m_roll, m_pitch, m_yaw, m_pixel_loc,
                                  m_height_guess,
                                  m_xyz_guess);
    result_type result;
    if (Dim == 1) {
      result[0] = norm_2(err);
    } else {
      result[0] = err[0];
      result[1] = err[1];
    }
    return result;
  }
};

// Find initial along-track guess and refine with Levenberg-Marquardt.
template <size_t Dim>
void solveCamLocation(RayDemPixelLMA<Dim> const& model, double spacing, 
                      double param_scale_factor,
                      vw::Vector<double, Dim> const& observation,
                      vw::Vector<double, Dim> & best_len) {

  int attempts = 10000;
  double best_val = g_big_val;
  double best_i = 0;
  for (int i = 0; i < attempts; i++) {
    double curr_best_val = best_val;
    for (int j = -1; j <= 1; j += 2) {
      double t = spacing * i * j;
      vw::Vector<double, Dim> curr_len = best_len;
      curr_len[0] = t / param_scale_factor;
      double val = norm_2(model(curr_len));
      if (val < best_val) {
        best_val = val;
        best_len[0] = curr_len[0];
        best_i = i * j;
      }
    }
    if (curr_best_val == best_val && curr_best_val < g_big_val)
      break;
  }

  attempts = 8;
  for (int attempt = 0; attempt < attempts; attempt++) {
    double delta_step = 1.0 / pow(10.0, attempt);
    double best_i_init = best_i;
    for (double i = best_i_init - 50 * delta_step;
         i <= best_i_init + 50 * delta_step; i += delta_step) {
      double t = spacing * i;
      vw::Vector<double, Dim> curr_len = best_len;
      curr_len[0] = t / param_scale_factor;
      double val = norm_2(model(curr_len));
      if (val < best_val) {
        best_val = val;
        best_len[0] = curr_len[0];
        best_i = i;
      }
    }
  }

  int status = -1;
  double max_abs_tol = 1e-14;
  double max_rel_tol = max_abs_tol;
  int num_max_iter = 100;
  best_len = vw::levenberg_marquardt(model, best_len, observation, status,
                                    max_abs_tol, max_rel_tol, num_max_iter);
}

// Find camera center in projected coordinates that best matches a ground point.
// If roll is 0, optimizes along-track only. If roll != 0, also optimizes across-track.
void findBestProjCamLocation(SatSimOptions const& opt,
                             vw::GeoReference const& dem_georef,
                             vw::ImageViewRef<vw::PixelMask<float>> dem,
                             double height_guess,
                             vw::Vector3 const& first_proj,
                             vw::Vector3 const& last_proj,
                             vw::Vector3 const& proj_along,
                             vw::Vector3 const& proj_across,
                             double delta, double roll, double pitch, double yaw,
                             vw::Vector2 const& pixel_loc,
                             // Outputs
                             vw::Vector3 & best_proj) {

  if (!std::isnan(opt.yaw) && opt.yaw != 0)
    vw::vw_throw(vw::ArgumentErr() << "When having both ground and orbital constraints, "
                                   << "the yaw must be zero.\n");

  double eps = 1e-7;
  vw::Vector3 P1 = vw::projToEcef(dem_georef, first_proj);
  vw::Vector3 P2 = vw::projToEcef(dem_georef, last_proj);
  double d = norm_2(P2 - P1);
  if (d < 1.0)
    vw::vw_throw(vw::ArgumentErr() 
      << "Ensure that the input orbit end points are at least 1 m apart.\n");
  double param_scale_factor = 1.0 / (eps * d);

  double dt = 1e-3;
  double t1 = -dt, t2 = dt;
  P1 = vw::projToEcef(dem_georef, first_proj * (1.0 - t1) + last_proj * t1);
  P2 = vw::projToEcef(dem_georef, first_proj * (1.0 - t2) + last_proj * t2);
  double slope = norm_2(P2 - P1) / (2*dt);
  double spacing = 1000.0 / slope;

  bool solve_roll = (!std::isnan(roll) && roll != 0.0);

  if (!solve_roll) {
    RayDemPixelLMA<1> model(opt, dem_georef, dem, height_guess, first_proj, last_proj,
                            proj_along, proj_across, delta, param_scale_factor, 0.0,
                            roll, pitch, yaw, pixel_loc);
    vw::Vector<double, 1> best_len, observation;
    best_len[0] = 0;
    observation[0] = 0;
    solveCamLocation(model, spacing, param_scale_factor, observation, best_len);
    double t = best_len[0] * param_scale_factor;
    best_proj = first_proj * (1.0 - t) + last_proj * t;
    return;
  }

  // 2D solve (roll != 0)
  P1 = vw::projToEcef(dem_georef, first_proj - dt * proj_across);
  P2 = vw::projToEcef(dem_georef, first_proj + dt * proj_across);
  double across_slope = norm_2(P2 - P1) / (2 * dt);
  double across_scale_factor = 1.0 / (eps * across_slope);

  RayDemPixelLMA<2> model(opt, dem_georef, dem, height_guess, first_proj, last_proj,
                          proj_along, proj_across, delta, param_scale_factor,
                          across_scale_factor, roll, pitch, yaw, pixel_loc);
  double H = std::max(first_proj[2] - height_guess, 1000.0);
  double s_guess = H * tan(roll * M_PI / 180.0);
  double u_guess = s_guess / across_slope;

  vw::Vector<double, 2> best_len_pos, best_len_neg;
  best_len_pos[0] = 0; best_len_pos[1] =  u_guess / across_scale_factor;
  best_len_neg[0] = 0; best_len_neg[1] = -u_guess / across_scale_factor;

  vw::Vector<double, 2> observation;
  observation[0] = 0; observation[1] = 0;

  solveCamLocation(model, spacing, param_scale_factor, observation, best_len_pos);
  solveCamLocation(model, spacing, param_scale_factor, observation, best_len_neg);

  vw::Vector<double, 2> best_len = 
    (norm_2(model(best_len_pos)) < norm_2(model(best_len_neg)))
    ? best_len_pos : best_len_neg;

  double t = best_len[0] * param_scale_factor;
  double u = best_len[1] * across_scale_factor;
  best_proj = first_proj * (1.0 - t) + last_proj * t + u * proj_across;
}

// A function to compute orbit length in ECEF given its endpoints in projected
// coordinates. 1e+5 points are used to approximate the orbit length. Should be
// enough. This gets slow when using 1e+6 points.
// TODO(oalexan1): This may not be accurate enough for very long orbit segments.
// TODO(oalexan1): The number of samples better depend on orbit length.
double calcOrbitLength(vw::Vector3 const& first_proj, vw::Vector3 const& last_proj,
                       vw::GeoReference const& dem_georef) {

  int num = 1.0e+5;

  // Start of each segment
  vw::Vector3 beg = vw::projToEcef(dem_georef, first_proj);
  // End of each segment
  vw::Vector3 end = beg;
  double orbitLength = 0.0;

  for (int i = 1; i < num; i++) { // note we start at 1

    double t = double(i) / double(num - 1); 
    // Find the projected position of the current point
    vw::Vector3 curr_proj = first_proj + t * (last_proj - first_proj);
    // Find the ECEF position of the current point
    end = vw::projToEcef(dem_georef, curr_proj);

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
                      vw::GeoReference const& dem_georef,
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
  vw::Vector3 G = vw::projToEcef(dem_georef, ground_proj_pos);

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
  across = -vw::cross_prod(along, ground_dir);

  // Make these vectors have norm 1, and make across perpendicular to along
  // Should already be that way by now, but do it just in case
  asp::normalizeOrthogonalizeAlongAcross(along, across);
}

// Adjust the orbit end point and set the number of cameras given the frame rate
// This is a bit tricky because need to sample finely the orbit
void adjustForFrameRate(SatSimOptions                  const& opt,
                        vw::GeoReference const& dem_georef,
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
  vw::Vector3 beg = vw::projToEcef(dem_georef, first_proj);
  vw::Vector3 end = beg;
  vw::Vector3 out_proj = first_proj; // will keep the result here
  double curr_len = 0.0;
  int i = 1;
  while (1) {

    // Find the projected position of the current point
    double t = double(i) / double(num - 1); 
    vw::Vector3 curr_proj = first_proj + t * (last_proj - first_proj);

    // Find the ECEF position of the current point and distance from previous
    end = vw::projToEcef(dem_georef, curr_proj);
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
                           vw::GeoReference const& dem_georef) {
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

  // Seed the random number generator. This will be reproducible each time the
  // program is run, but will be different for each set of input cameras.
  if (opt.random_pose_perturbation)
    srand(vw::norm_2(curr_proj));
  
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
      
      // Compute the jitter, in degrees. Add the phase. For a random perturbation
      // make it between -a and a.
      if (!opt.random_pose_perturbation)
        amp[c] += a * sin(signed_dist * 2.0 * M_PI / T + opt.jitter_phase[index]);
      else 
        amp[c] += a * (2 * double(rand())/double(RAND_MAX) - 1.0);
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
                         vw::GeoReference const& dem_georef,
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
                 vw::GeoReference const& dem_georef,
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

  // Validate that --first, --last, and ground-pos (if given) are DEM
  // pixel coordinates inside the DEM extent. Out-of-range values
  // previously caused sat_sim to spin indefinitely while the optimizer
  // searched in unreachable regions.
  auto checkPixInDem = [&](std::string const& name,
                           vw::Vector2 const& pix) {
    double col = pix[0], row = pix[1];
    if (std::isnan(col) || std::isnan(row)) return;
    int ncols = dem.cols(), nrows = dem.rows();
    if (col < 0 || col > ncols - 1 || row < 0 || row > nrows - 1)
      vw::vw_throw(vw::ArgumentErr()
          << name << " DEM pixel (" << col << ", " << row
          << ") is outside DEM extent [0.." << (ncols - 1)
          << ", 0.." << (nrows - 1) << "]. "
          << "sat_sim expects --" << name
          << " in DEM pixel coordinates, not lon/lat.\n");
  };
  checkPixInDem("first",
                vw::subvector(opt.first, 0, 2));
  checkPixInDem("last",
                vw::subvector(opt.last,  0, 2));
  checkPixInDem("first-ground-pos", opt.first_ground_pos);
  checkPixInDem("last-ground-pos",  opt.last_ground_pos);

  // Convert the first and last camera center positions to projected coordinates
  vw::Vector3 first_proj, last_proj;
  subvector(first_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::subvector(opt.first, 0, 2)); // x and y
  first_proj[2] = opt.first[2]; // z
  subvector(last_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::subvector(opt.last,  0, 2)); // x and y
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
    vw::Vector3 down = vw::cross_prod(along, across);
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
    vw::Matrix3x3 R0 = vw::identity_matrix<3>();
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

} // end namespace asp
