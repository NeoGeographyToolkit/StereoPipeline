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

#include <asp/Camera/CsmModel.h>
#include <asp/Camera/CsmModelFit.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/LinescanSpotModel.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Core/CameraTransforms.h>

#include <vw/Cartography/Datum.h>
#include <vw/Math/QuatInterp.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/Distortion.h>

namespace asp {

using vw::Vector3;
using vw::Matrix3x3;

// Compute the local orbital frame from satellite position and velocity.
// From the SPOT 123-4-5 Geometry Handbook (GAEL-P135-DOC-001), Eq. 7a-c.
//   Z2 = normalize(pos), X2 = normalize(vel x Z2), Y2 = Z2 x X2
// Returns the matrix [X2 | Y2 | Z2] that converts from orbital to GCC.
Matrix3x3 get_local_orbital_frame(Vector3 const& position, Vector3 const& velocity) {
  Vector3 Z2 = vw::math::normalize(position);
  Vector3 X2 = vw::math::normalize(vw::math::cross_prod(velocity, Z2));
  Vector3 Y2 = vw::math::cross_prod(Z2, X2);
  Matrix3x3 out;
  for (int r = 0; r < 3; r++) {
    out(r,0) = X2[r];
    out(r,1) = Y2[r];
    out(r,2) = Z2[r];
  }
  return out;
}

// Compute the look rotation matrix Mp*Mr*My from yaw, pitch, roll.
// From the SPOT 123-4-5 Geometry Handbook (GAEL-P135-DOC-001), Eq. 11a-e.
// Converts a look vector from the navigation frame (O1) to the orbital frame (O2).
Matrix3x3 get_look_rotation_matrix(double yaw, double pitch, double roll) {
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);
  double cy = cos(yaw);
  double sy = sin(yaw);

  Matrix3x3 M;
  M(0,0) = (cr*cy);            M(0,1) = (-cr*sy);           M(0,2) = (-sr);
  M(1,0) = (cp*sy+sp*sr*cy);   M(1,1) = (cp*cy-sp*sr*sy);   M(1,2) = (sp*cr);
  M(2,0) = (-sp*sy+cp*sr*cy);  M(2,1) = (-sp*cy-cp*sr*sy);  M(2,2) = cp*cr;
  return M;
}

// Load a SPOT5 CSM camera model. Populates CSM with vendor extrinsics
// directly (positions, velocities, quaternions from XML), then fits
// only intrinsics (focal length, optical center, distortion) to match
// the vendor's per-column look angle table.
boost::shared_ptr<CsmModel>
load_spot5_csm_camera_model_from_xml(std::string const& path) {

  // Parse the SPOT5 XML file
  SpotXML xml_reader;
  xml_reader.read_xml(path);

  // Get the interpolation functors for position/velocity (for quaternion conversion)
  vw::LagrangianInterpolation position_func = xml_reader.setup_position_func();
  vw::LagrangianInterpolation velocity_func = xml_reader.setup_velocity_func();
  vw::LinearTimeInterpolation time_func     = xml_reader.setup_time_func();

  // Extract raw position/velocity data with times
  std::vector<vw::Vector3> positions, velocities;
  std::vector<double> pos_times;
  for (auto iter = xml_reader.position_logs.begin();
       iter != xml_reader.position_logs.end(); iter++) {
    double t = xml_reader.convert_time(iter->first);
    pos_times.push_back(t);
    positions.push_back(iter->second);
  }
  for (auto iter = xml_reader.velocity_logs.begin();
       iter != xml_reader.velocity_logs.end(); iter++)
    velocities.push_back(iter->second);

  // Compute uniform ephemeris time grid
  double t0_ephem = pos_times.front();
  double dt_ephem = (pos_times.back() - pos_times.front()) / (pos_times.size() - 1.0);

  // Warn if ephemeris time grid is not uniform (1e-3 relative tolerance)
  for (size_t i = 1; i < pos_times.size(); i++) {
    double dt_i = pos_times[i] - pos_times[i-1];
    if (std::abs(dt_i - dt_ephem) > 1e-3 * std::abs(dt_ephem))
      vw::vw_out(vw::WarningMessage) << "SPOT5: non-uniform ephemeris spacing at "
        << "sample " << i << ": dt = " << dt_i << " vs expected " << dt_ephem << "\n";
  }

  // Extract raw pose data and convert yaw/pitch/roll to GCC quaternions.
  vw::LinearPiecewisePositionInterpolation spot_pose_func
    = xml_reader.setup_pose_func(time_func);

  // Get pose sample times
  std::vector<double> pose_times;
  for (auto iter = xml_reader.pose_logs.begin();
       iter != xml_reader.pose_logs.end(); iter++) {
    double t = xml_reader.convert_time(iter->first);
    pose_times.push_back(t);
  }

  // Warn if attitude time grid is not uniform (1e-3 relative tolerance)
  if (pose_times.size() > 1) {
    double dt_pose = (pose_times.back() - pose_times.front()) /
                     (pose_times.size() - 1.0);
    for (size_t i = 1; i < pose_times.size(); i++) {
      double dt_i = pose_times[i] - pose_times[i-1];
      if (std::abs(dt_i - dt_pose) > 1e-3 * std::abs(dt_pose))
        vw::vw_out(vw::WarningMessage) << "SPOT5: non-uniform attitude spacing at "
          << "sample " << i << ": dt = " << dt_i << " vs expected " << dt_pose << "\n";
    }
  }

  // Pad pose times to match what setup_pose_func does (it pads at boundaries)
  double t0_quat = spot_pose_func.get_t0();
  double dt_quat = spot_pose_func.get_dt();
  double tend_quat = spot_pose_func.get_tend();
  size_t num_quat = static_cast<size_t>(round((tend_quat - t0_quat) / dt_quat));

  // Axis reorientation matrix: R = diag(-1, 1, -1)
  // Converts from vendor's navigation frame to the frame used by CSM.
  Matrix3x3 R;
  R(0,0) = -1.0; R(0,1) = 0.0; R(0,2) =  0.0;
  R(1,0) =  0.0; R(1,1) = 1.0; R(1,2) =  0.0;
  R(2,0) =  0.0; R(2,1) = 0.0; R(2,2) = -1.0;

  // Convert each pose sample to a GCC rotation matrix
  std::vector<vw::Matrix<double,3,3>> cam2world_vec(num_quat);
  for (size_t i = 0; i < num_quat; i++) {
    double time = t0_quat + dt_quat * static_cast<double>(i);
    Vector3 position       = position_func(time);
    Vector3 velocity       = velocity_func(time);
    Vector3 yaw_pitch_roll = spot_pose_func(time);

    Matrix3x3 lo_frame      = get_local_orbital_frame(position, velocity);
    Matrix3x3 look_rotation = get_look_rotation_matrix(
                                yaw_pitch_roll[0], yaw_pitch_roll[1], yaw_pitch_roll[2]);
    cam2world_vec[i] = lo_frame * look_rotation * R;
  }

  // Compute intrinsics from the look angle table using the PeruSat approach:
  // focal_length = 1.0, detector params in angle units.
  //
  // The local look vector is: normalize(tan(PSI_Y), tan(PSI_X), 1.0)
  // CSM's look vector is: normalize(undist_x, undist_y, focal_length)
  // With f=1.0: look = normalize(undist_x, undist_y, 1.0)
  // So we need: undist_x = tan(PSI_Y(col)), undist_y = tan(PSI_X(col))
  //
  // The detector model computes:
  //   distortedY = col * sampleSumming - sampleOrigin  (across-track)
  //   distortedX = 0 * lineSumming - lineOrigin         (along-track)
  //
  // Fit a linear model to tan(PSI_Y) vs column. The nonlinear residual
  // (degree 3+ curvature) is captured later by TRANSVERSE distortion.
  //   tan(PSI_Y(col)) ~ slope_y * col + intercept_y
  // Then: sampleSumming = slope_y, sampleOrigin = -intercept_y
  //
  // For PSI_X (nearly constant): lineOrigin = -tan(PSI_X_center)
  auto const& look_angles = xml_reader.look_angles;
  int num_cols = xml_reader.image_size[0];

  // Linear fit to tan(PSI_Y) vs column index (0-based)
  double sum_c = 0, sum_ty = 0, sum_c2 = 0, sum_cty = 0;
  for (int c = 0; c < num_cols; c++) {
    double ty = tan(look_angles[c].second[1]);
    sum_c   += c;
    sum_ty  += ty;
    sum_c2  += c * (double)c;
    sum_cty += c * ty;
  }
  double n_cols = num_cols;
  double slope_y     = (n_cols * sum_cty - sum_c * sum_ty) /
                       (n_cols * sum_c2 - sum_c * sum_c);
  double intercept_y = (sum_ty - slope_y * sum_c) / n_cols;

  // PSI_X is nearly constant - use center value
  int mid_col = num_cols / 2;
  double psi_x_center = look_angles[mid_col].second[0];

  double focal_length = 1.0;
  // optical_center is not used directly - detector origin encodes it
  vw::Vector2 optical_center(0.0, 0.0);

  // Line dating
  double first_line_time = time_func(0);
  double dt_line = xml_reader.line_period;

  vw::cartography::Datum datum("WGS84");
  std::string sensor_id = "SPOT5";

  // Populate CSM model fields directly. We do this instead of calling
  // populateCsmLinescan() because positions (12 pts at 30s) and quaternions
  // (515 pts at 0.125s) have different counts and time grids.
  boost::shared_ptr<CsmModel> csm_model(new CsmModel());
  csm_model->m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  csm_model->m_semi_major_axis = datum.semi_major_axis();
  csm_model->m_semi_minor_axis = datum.semi_minor_axis();

  csm_model->m_gm_model = boost::make_shared<UsgsAstroLsSensorModel>();
  UsgsAstroLsSensorModel* ls =
    dynamic_cast<UsgsAstroLsSensorModel*>(csm_model->m_gm_model.get());
  ls->reset();

  ls->m_nSamples         = xml_reader.image_size[0];
  ls->m_nLines           = xml_reader.image_size[1];
  ls->m_platformFlag     = 1; // order 8 Lagrange (matches vendor prescription)
  ls->m_minElevation     = -10000.0;
  ls->m_maxElevation     =  10000.0;
  ls->m_focalLength      = focal_length;
  ls->m_zDirection       = 1.0;
  ls->m_halfSwath        = 1.0;
  ls->m_sensorIdentifier = sensor_id;
  ls->m_majorAxis        = datum.semi_major_axis();
  ls->m_minorAxis        = datum.semi_minor_axis();

  // Detector model: PeruSat-style, mapping pixels to angle-like values.
  // distortedY = col * sampleSumming - sampleOrigin = slope_y * col + intercept_y
  // distortedX = 0 - lineOrigin = tan(PSI_X_center)
  ls->m_iTransL[0] = 0.0; ls->m_iTransL[1] = 0.0; ls->m_iTransL[2] = 1.0;
  ls->m_iTransS[0] = 0.0; ls->m_iTransS[1] = 1.0; ls->m_iTransS[2] = 0.0;
  ls->m_detectorSampleSumming  = slope_y;
  ls->m_detectorSampleOrigin   = -intercept_y + 0.5 * slope_y;
  ls->m_detectorLineSumming    = 1.0;
  ls->m_detectorLineOrigin     = -tan(psi_x_center);
  ls->m_startingDetectorLine   = 0.0;
  ls->m_startingDetectorSample = 0.0;

  // Line timing
  ls->m_intTimeLines.push_back(1.0);
  ls->m_intTimeStartTimes.push_back(first_line_time);
  ls->m_intTimes.push_back(dt_line);

  // Positions and velocities (12 pts at 30s)
  ls->m_t0Ephem = t0_ephem;
  ls->m_dtEphem = dt_ephem;
  ls->m_numPositions = 3 * positions.size();
  ls->m_positions.resize(ls->m_numPositions);
  ls->m_velocities.resize(ls->m_numPositions);
  for (size_t i = 0; i < positions.size(); i++)
    for (int c = 0; c < 3; c++) {
      ls->m_positions [3*i + c] = positions[i][c];
      ls->m_velocities[3*i + c] = velocities[i][c];
    }

  // Convert rotation matrices to VW quaternions (w, x, y, z order)
  std::vector<vw::Quaternion<double>> in_quats(cam2world_vec.size());
  std::vector<double> in_quat_times(cam2world_vec.size());
  for (size_t i = 0; i < cam2world_vec.size(); i++) {
    double x, y, z, w;
    asp::matrixToQuaternion(cam2world_vec[i], x, y, z, w);
    in_quats[i] = vw::Quaternion<double>(w, x, y, z);
    in_quat_times[i] = t0_quat + dt_quat * static_cast<double>(i);
  }

  // SLERP resample 5x to honor vendor's linear interpolation prescription.
  // CSM uses order-8 Lagrange on quaternion components, but the vendor
  // prescribes linear interpolation of attitude. A 5x denser grid makes
  // Lagrange converge to linear behavior between the original samples.
  int quat_resample_factor = 5;
  std::vector<double> out_quat_times;
  std::vector<vw::Quaternion<double>> out_quats;
  asp::resampleQuatSlerp(in_quat_times, in_quats, quat_resample_factor,
                         out_quat_times, out_quats);

  double quat_t0_out = out_quat_times.front();
  double quat_dt_out = (out_quat_times.back() - out_quat_times.front()) /
                       (out_quat_times.size() - 1.0);

  // Populate CSM quaternions (USGSCSM stores as x, y, z, w)
  ls->m_t0Quat = quat_t0_out;
  ls->m_dtQuat = quat_dt_out;
  ls->m_numQuaternions = 4 * out_quats.size();
  ls->m_quaternions.resize(ls->m_numQuaternions);
  for (size_t i = 0; i < out_quats.size(); i++) {
    ls->m_quaternions[4*i + 0] = out_quats[i].x();
    ls->m_quaternions[4*i + 1] = out_quats[i].y();
    ls->m_quaternions[4*i + 2] = out_quats[i].z();
    ls->m_quaternions[4*i + 3] = out_quats[i].w();
  }
  asp::normalizeQuaternions(ls);

  // TRANSVERSE distortion starting at identity
  ls->m_distortionType = TRANSVERSE;
  ls->m_opticalDistCoeffs.resize(20, 0.0);
  ls->m_opticalDistCoeffs[1]  = 1.0; // x: identity for ux
  ls->m_opticalDistCoeffs[12] = 1.0; // y: identity for uy

  // Re-create from state to finalize internal bookkeeping
  std::string modelState = ls->getModelState();
  ls->replaceModelState(modelState);

  // Fit intrinsics (focal_length, optical_center, distortion) to match
  // the vendor's per-column look angle table. Rotations are pinned constant
  // since extrinsics come from the vendor. Generate world sight vectors
  // directly from the look angle table and SLERP'd quaternions.
  {
    // SLERP interpolator on the resampled quaternions (same data as CSM has)
    vw::SLERPPoseInterpolation slerp_interp(out_quats, quat_t0_out, quat_dt_out);

    int num_rows = xml_reader.image_size[1];
    int d_col_fit = std::max(1, num_cols / 500); // ~500 column samples
    int d_row_fit = std::max(1, num_rows / 3);   // ~3 rows (intrinsics same per row)

    std::vector<int> col_idx, row_idx;
    for (int c = 0; c < num_cols; c += d_col_fit)
      col_idx.push_back(c);
    if (col_idx.back() != num_cols - 1)
      col_idx.push_back(num_cols - 1);
    for (int r = 0; r < num_rows; r += d_row_fit)
      row_idx.push_back(r);
    if (row_idx.back() != num_rows - 1)
      row_idx.push_back(num_rows - 1);

    // Vendor look direction (Eq. 5a) is u1 = normalize(-tan(PSI_Y), tan(PSI_X), -1).
    // The rotation matrices already include the R=diag(-1,1,-1) axis flip,
    // so the local look vector in the rotated frame is:
    //   u_local = normalize(tan(PSI_Y), tan(PSI_X), 1.0)
    SightMatT world_sight_mat(row_idx.size());
    for (size_t ri = 0; ri < row_idx.size(); ri++) {
      world_sight_mat[ri].resize(col_idx.size());
      double time = first_line_time + dt_line * row_idx[ri];
      vw::Quaternion<double> q = slerp_interp(time);
      vw::Matrix3x3 rot = q.rotation_matrix();
      for (size_t ci = 0; ci < col_idx.size(); ci++) {
        int c = col_idx[ci];
        double psi_x = look_angles[c].second[0];
        double psi_y = look_angles[c].second[1];
        vw::Vector3 local_look = normalize(vw::Vector3(tan(psi_y), tan(psi_x), 1.0));
        world_sight_mat[ri][ci] = rot * local_look;
      }
    }

    int min_col = col_idx.front();
    int min_row = row_idx.front();
    bool fix_rotations = true;
    asp::refineCsmLinescanFit(world_sight_mat, min_col, min_row,
                              d_col_fit, d_row_fit, *csm_model,
                              fix_rotations, TRANSVERSE);
  }

  return csm_model;
}

} // end namespace asp
