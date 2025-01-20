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

// Functions used for handling CSM camera models.

#include <asp/Camera/CsmModel.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/BaseCameraUtils.h>

#include <vw/Camera/OrbitalCorrections.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/Utilities.h>

namespace asp {

// Ensure that quaternions don't suddenly change sign. This is a bugfix.
void fixQuaternionSigns(UsgsAstroLsSensorModel * ls_model) {
  
  // Find the largest magnitude quaternion coefficient and its coordinate
  int numQuats = ls_model->m_numQuaternions / NUM_QUAT_PARAMS;
  double max_q = 0.0;
  int max_j = 0;
  for (int i = 0; i < numQuats; i++) {
    double * quat = &ls_model->m_quaternions[NUM_QUAT_PARAMS * i];
    for (int j = 0; j < 4; j++) {
      if (std::abs(quat[j]) > std::abs(max_q)) {
        max_q = quat[j];
        max_j = j;
      }
    }
  }
    
  // Ensure the signs are consistent
  for (int i = 0; i < numQuats; i++) {
    double * quat = &ls_model->m_quaternions[NUM_QUAT_PARAMS * i];
    if (quat[max_j] * max_q < 0) {
      for (int j = 0; j < 4; j++) {
        quat[j] *= -1.0;
      }  
    }
  }
}

// Normalize quaternions in UsgsAstroLsSensorModel.
void normalizeQuaternions(UsgsAstroLsSensorModel * ls_model) {

  for (int qit = 0; qit < ls_model->m_numQuaternions / 4; qit++) {

    double norm = 0.0;
    for (int coord = 0; coord < 4; coord++)
      norm += ls_model->m_quaternions[4 * qit + coord] * ls_model->m_quaternions[4 * qit + coord];

    norm = sqrt(norm);
    if (norm == 0)
      continue;
   
    for (int coord = 0; coord < 4; coord++)
      ls_model->m_quaternions[4 * qit + coord] /= norm;
  }

  // Fix any sign issues. This is a bugfix.
  asp::fixQuaternionSigns(ls_model);
  
  return;
}

// Normalize quaternions in UsgsAstroFrameSensorModel.
void normalizeQuaternions(UsgsAstroFrameSensorModel * frame_model) {

  // Fetch the quaternions. In the model, the positions are stored first, then
  // the quaternions. 
  double q[4];
  double norm = 0.0;
  for (size_t i = 0; i < 4; i++) {
    q[i] = frame_model->getParameterValue(i + 3); 
    norm += q[i]*q[i];
  }
  norm = sqrt(norm);

  if (norm == 0)
    return;
    
  // Normalize the quaternions. Put them back in the model.
  for (size_t i = 0; i < 4; i++) {
    q[i] /= norm;
    frame_model->setParameterValue(i + 3, q[i]);
  }

  return;
}

// Get quaternions. This duplicates the UsgsAstroLsSensorModel function as that one is private
void interpQuaternions(UsgsAstroLsSensorModel const* ls_model, double time,
                      double q[4]) {
  int nOrder = 8;
  if (ls_model->m_platformFlag == 0)
    nOrder = 4;
  int nOrderQuat = nOrder;
  if (ls_model->m_numQuaternions/4 < 6 && nOrder == 8)
    nOrderQuat = 4;
  
  lagrangeInterp(ls_model->m_numQuaternions / 4, &ls_model->m_quaternions[0],
                 ls_model->m_t0Quat, ls_model->m_dtQuat, time, 4, nOrderQuat, q);
}

// Get positions. Based on the UsgsAstroLsSensorModel code.
void interpPositions(std::vector<double> const& positions,
                     double t0Ephem, double dtEphem, int platformFlag,
                     double time, double pos[3]) {
  int nOrder = 8;
  if (platformFlag == 0)
    nOrder = 4;
  
  // This seems to handle gracefully when the number of positions is very small.
  lagrangeInterp(positions.size() / 3, &positions[0],
                 t0Ephem, dtEphem,
                 time, 3, nOrder, pos);
}

// Get positions. Based on the UsgsAstroLsSensorModel code.
void interpPositions(UsgsAstroLsSensorModel const* ls_model, double time,
                     double pos[3]) {
  // Call the above wrapper
  interpPositions(ls_model->m_positions, ls_model->m_t0Ephem, ls_model->m_dtEphem,
                  ls_model->m_platformFlag, time, pos);
}

// Get positions. Based on the UsgsAstroLsSensorModel code.
// TODO(oalexan1): Move this to a new CsmModelUtils.cc file and
void interpVelocities(UsgsAstroLsSensorModel const* ls_model, double time,
                  double vel[3]) {
  int nOrder = 8;
  if (ls_model->m_platformFlag == 0)
    nOrder = 4;
  double sensPosNom[3];
  lagrangeInterp(ls_model->m_numPositions / 3, &ls_model->m_velocities[0],
                 ls_model->m_t0Ephem, ls_model->m_dtEphem,
                 time, 3, nOrder, vel);
}

// Nearest neighbor interpolation into a sequence of vectors of length
// vectorLength, stored one after another in valueArray. The result
// goes in valueVector. Analogous to lagrangeInterp() in CSM.
void nearestNeibInterp(const int &numTimes, const double *valueArray,
                       const double &startTime, const double &delTime,
                       const double &time, const int &vectorLength,
                       double *valueVector) {
  
  if (numTimes < 1)
    vw::vw_throw(vw::ArgumentErr() << "Cannot interpolate into a vector of zero length.\n");
  
  // Compute index
  int index = round((time - startTime) / delTime);
  if (index < 0) 
    index = 0;
  if (index >= numTimes)
    index = numTimes - 1;

  int start = index * vectorLength;
  for (int i = 0; i < vectorLength; i++)
    valueVector[i] = valueArray[start + i];

  return;
}

// Given two values, double t1, t2, and to points, vw::Vector3 P1, P2, at those
// values, find the value at t using linear interpolation.
vw::Vector3 linearInterp(double t1, double t2, vw::Vector3 const& P1,
                         vw::Vector3 const& P2, double t) {
    if (t1 == t2)
      vw::vw_throw(vw::ArgumentErr() << "Expecting t1 != t2 in interpolation.\n");

  double alpha = (t - t1)/(t2 - t1);
  return P1 + alpha*(P2 - P1);
}

// - Given a vector of positions with starting time and spacing, and a new
//   starting time, spacing, and number of points, interpolate the positions at
//   the new locations. If outside the range, use linear extrapolation. For
//   that, assume the points are in ECEF, giving an orbit, so first convert to
//   projected coordinates to make the extrapolated trajectory still go around
//   the planet rather than go on a tangent. 
// - Will return the original value at points at which the new time is the same as old
//   time (within 1e-8 time tolerance).
void orbitInterpExtrap(double t0_in, double dt_in, int platformFlag,
  std::vector<double> const& positions_in, vw::cartography::GeoReference const& geo, 
  double t0_out, double dt_out, int num_out, std::vector<double> & positions_out) {

  // Wipe the output
  positions_out.clear();

  // Sanity checks
  if (positions_in.size() < 2 || num_out < 2)
    vw::vw_throw(vw::ArgumentErr() << "Expecting at least two positions in interpolation.\n");
  if (dt_in <= 0.0 || dt_out <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting positive time step in interpolation.\n");

  // May need to create new orbital points for interpolation/extrapolation. Start
  // by putting the existing ones in a map.
  std::map<double, vw::Vector3> time_to_pos;
  for (size_t i = 0; i < positions_in.size()/NUM_XYZ_PARAMS; i++) {
    double t = t0_in + i * dt_in;
    int start = i * NUM_XYZ_PARAMS;
    vw::Vector3 pos(positions_in[start + 0], positions_in[start + 1], 
                    positions_in[start + 2]);
    time_to_pos[t] = pos;
  }

  // Add left extrapolated points. Use first two positions for linear extrapolation.
  // Extrapolation is done in projected coordinates, to stay in orbit,
  // and then converted back to ECEF. Add at least 8 points to help with Lagrange
  // interpolation later.
  auto it = time_to_pos.begin();
  vw::Vector3 P0 = it->second;
  it++; 
  vw::Vector3 P1 = it->second;
  vw::Vector3 proj0 = vw::cartography::ecefToProj(geo, P0);
  vw::Vector3 proj1 = vw::cartography::ecefToProj(geo, P1);
  double t = t0_in;
  while (t + 8 * dt_in >= t0_out) {
    t -= dt_in;
    vw::Vector3 proj = linearInterp(t0_in, t0_in + dt_in, proj0, proj1, t);
    vw::Vector3 P = vw::cartography::projToEcef(geo, proj);
    time_to_pos[t] = P;
  }

  // Now do the same at the end. Use (t1, P1) for the last point, and (t0, P0) for
  // the second to last point. Extrapolate to the right of t1.
  it = time_to_pos.end(); 
  it--;
  P1 = it->second;
  double t1 = it->first;
  it--;
  double t0 = it->first;
  P0 = it->second;
  proj0 = vw::cartography::ecefToProj(geo, P0);
  proj1 = vw::cartography::ecefToProj(geo, P1);

  // Add right extrapolated points. Need to have at least 8 to be able to
  // interpolate using Lagrange.
  double t_out_end = t0_out + (num_out - 1) * dt_out;
  t = t1; 
  while (t - 8 * dt_in <= t_out_end) {
    t += dt_in;
    vw::Vector3 proj = linearInterp(t0, t1, proj0, proj1, t);
    vw::Vector3 P = vw::cartography::projToEcef(geo, proj);
    time_to_pos[t] = P;
  }

  // Put all the produced values in the same vector, in order of time
  int num_extra = time_to_pos.size();
  double t0_extra = time_to_pos.begin()->first;
  double dt_extra = dt_in;
  std::vector<double> extra_positions(num_extra * NUM_XYZ_PARAMS);
  int count = 0;
  for (auto it = time_to_pos.begin(); it != time_to_pos.end(); it++) {
    vw::Vector3 P = it->second;
    extra_positions[count*NUM_XYZ_PARAMS + 0] = P[0];
    extra_positions[count*NUM_XYZ_PARAMS + 1] = P[1];
    extra_positions[count*NUM_XYZ_PARAMS + 2] = P[2];
    count++;
  }

  // Now we have enough positions to interpolate at
  positions_out.resize(num_out * NUM_XYZ_PARAMS);
  for (int i = 0; i < num_out; i++) {
    double t = t0_out + i * dt_out;

    // If this is in the input data, just copy the value
    double i_in_float = (t - t0_in)/dt_in;
    int i_in = round(i_in_float);
    // TODO(oalexan1): The check below is sensitive to large t values
    // and to dt being on the order of 1e-8, which does not happen in practice,
    // but should be improved, somehow.
    if (i_in >= 0 && i_in < positions_in.size()/NUM_XYZ_PARAMS &&
        std::abs(t - (t0_in + i_in*dt_in)) < 1e-8) {
      positions_out[i*NUM_XYZ_PARAMS + 0] = positions_in[i_in*NUM_XYZ_PARAMS + 0];
      positions_out[i*NUM_XYZ_PARAMS + 1] = positions_in[i_in*NUM_XYZ_PARAMS + 1];
      positions_out[i*NUM_XYZ_PARAMS + 2] = positions_in[i_in*NUM_XYZ_PARAMS + 2];
      continue;
    }

    double P[3];
    interpPositions(extra_positions, t0_extra, dt_extra, platformFlag, t, P);
    positions_out[i*NUM_XYZ_PARAMS + 0] = P[0];
    positions_out[i*NUM_XYZ_PARAMS + 1] = P[1];
    positions_out[i*NUM_XYZ_PARAMS + 2] = P[2];
  }

  return;
}

// Find interpolated/extrapolated positions at all camera pose times.
// See the function being called below for more details.
void orbitInterpExtrap(UsgsAstroLsSensorModel const * ls_model,
                       vw::cartography::GeoReference const& geo,
                       std::vector<double> & positions_out) {

  orbitInterpExtrap(ls_model->m_t0Ephem, ls_model->m_dtEphem, ls_model->m_platformFlag,  
                    ls_model->m_positions,
                    geo, ls_model->m_t0Quat, ls_model->m_dtQuat, 
                    ls_model->m_quaternions.size()/NUM_QUAT_PARAMS,
                    positions_out); // output
}

// See documentation in CsmUtils.h
void populateCsmLinescan(double first_line_time, double dt_line, 
                         double t0_ephem, double dt_ephem,
                         double t0_quat, double dt_quat, 
                         double focal_length,
                         vw::Vector2                const & optical_center,
                         vw::Vector2i               const & image_size,
                         vw::cartography::Datum     const & datum, 
                         std::string                const & sensor_id, 
                         std::vector<vw::Vector3>   const & positions,
                         std::vector<vw::Vector3>   const & velocities,
                         std::vector<vw::Matrix3x3> const & cam2world,
                         // Outputs
                         asp::CsmModel                    & model) {
  
  // Sanity checks
  if (positions.size() != cam2world.size())
    vw_throw(vw::ArgumentErr() << "Expecting as many positions as orientations.\n");
  if (velocities.size() != positions.size())
    vw_throw(vw::ArgumentErr() << "Expecting as many velocities as positions.\n");
      
  // Do not use a precision below 1.0-e8 as then the linescan model will return junk.
  model.m_desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  model.m_semi_major_axis = datum.semi_major_axis();
  model.m_semi_minor_axis = datum.semi_minor_axis();

  // Create the linescan model. Memory is managed by m_gm_model.
  model.m_gm_model.reset(new UsgsAstroLsSensorModel);
  UsgsAstroLsSensorModel* ls_model
    = dynamic_cast<UsgsAstroLsSensorModel*>(model.m_gm_model.get());
  if (ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");

  // This performs many initializations apart from the above
  ls_model->reset();

  // Override some initializations
  ls_model->m_nSamples         = image_size[0]; 
  ls_model->m_nLines           = image_size[1];
  ls_model->m_platformFlag     = 1; // Use 1, for order 8 Lagrange interpolation
  ls_model->m_minElevation     = -10000.0; // -10 km
  ls_model->m_maxElevation     =  10000.0; //  10 km
  ls_model->m_focalLength      = focal_length;
  ls_model->m_zDirection       = 1.0;
  ls_model->m_halfSwath        = 1.0;
  ls_model->m_sensorIdentifier = sensor_id;
  ls_model->m_majorAxis        = model.m_semi_major_axis;
  ls_model->m_minorAxis        = model.m_semi_minor_axis;
  
  // The choices below are copied from the DigitalGlobe CSM linescan model.
  // Better to keep same convention than dig deep inside UsAstroLsSensorModel.
  // Also keep in mind that a CSM pixel has extra 0.5 added to it.
  ls_model->m_iTransL[0]             = 0.0;  
  ls_model->m_iTransL[1]             = 0.0;
  ls_model->m_iTransL[2]             = 1.0;
  ls_model->m_iTransS[0]             = 0.0;
  ls_model->m_iTransS[1]             = 1.0;
  ls_model->m_iTransS[2]             = 0.0;
  ls_model->m_detectorLineSumming    = 1.0;
  ls_model->m_detectorSampleSumming  = 1.0;
  
  // Keep these as is. Modify instead m_detectorLineOrigin and
  // m_detectorSampleOrigin. The effect is same as all USGSCSM code uses
  // m_detectorLineOrigin - m_startingDetectorLine, and the same for the sample.
  ls_model->m_startingDetectorLine   = 0.0;
  ls_model->m_startingDetectorSample = 0.0;
  
  // Optical center. There is an inconsistency below, but this is what works.
  ls_model->m_detectorLineOrigin     = optical_center[1];
  ls_model->m_detectorSampleOrigin   = optical_center[0] + 0.5;
  
  // Set the time 
  ls_model->m_intTimeLines.push_back(1.0); // to offset CSM's quirky 0.5 additions in places
  ls_model->m_intTimeStartTimes.push_back(first_line_time);
  ls_model->m_intTimes.push_back(dt_line); // time between lines

  // Copy positions and velocities
  ls_model->m_t0Ephem = t0_ephem;
  ls_model->m_dtEphem = dt_ephem;
  ls_model->m_numPositions = 3 * positions.size(); // concatenate all coordinates
  ls_model->m_positions.resize(ls_model->m_numPositions);
  ls_model->m_velocities.resize(ls_model->m_numPositions);
  for (size_t index = 0; index < positions.size(); index++) {
    vw::Vector3 ctr = positions[index];
    vw::Vector3 vel = velocities[index];
    for (int coord = 0; coord < 3; coord++) {
      ls_model->m_positions [3*index + coord] = ctr[coord];
      ls_model->m_velocities[3*index + coord] = vel[coord];
    }
  }
  
  // Copy orientations
  ls_model->m_numQuaternions = 4 * cam2world.size();
  ls_model->m_t0Quat = t0_quat;
  ls_model->m_dtQuat = dt_quat;
  ls_model->m_quaternions.resize(ls_model->m_numQuaternions);
  for (size_t index = 0; index < cam2world.size(); index++) {
    auto c2w = cam2world[index];
    double x, y, z, w;
    asp::matrixToQuaternion(c2w, x, y, z, w);

    // Note how we store the quaternions in the order x, y, z, w, not w, x, y, z.
    int coord = 0;
    ls_model->m_quaternions[4*index + coord] = x; coord++;
    ls_model->m_quaternions[4*index + coord] = y; coord++;
    ls_model->m_quaternions[4*index + coord] = z; coord++;
    ls_model->m_quaternions[4*index + coord] = w; coord++;
  }
  
  // Quaternions must always be normalized and not change suddenly in sign.
  asp::normalizeQuaternions(ls_model);

  // Use the radtan distortion model with zero distortion
  ls_model->m_distortionType = RADTAN;
  ls_model->m_opticalDistCoeffs.resize(5, 0.0);
  
  // Re-creating the model from the state forces some operations to
  // take place which are inaccessible otherwise.
  std::string modelState = ls_model->getModelState();
  ls_model->replaceModelState(modelState);
}

// Apply the given adjustment to the given CSM camera.
// The camera is passed twice, once as a CSM model, and once as a
// CamPtr, as the latter may have the CSM model as a member or 
// as a base class, depending on the implementation.
// TODO(oalexan1): This needs to be made uniform.
void applyAdjustmentToCsmCamera(std::string const& image_file,
                                std::string const& camera_file,
                                std::string const& adjust_prefix,
                                vw::CamPtr  const& cam,
                                asp::CsmModel    * csm_cam) {

  std::string adjust_file = asp::bundle_adjust_file_name(adjust_prefix, image_file, 
                                                         camera_file);
  vw::vw_out() << "Reading input adjustment: " << adjust_file << std::endl;
  vw::camera::AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(cam));
  adj_cam.read(adjust_file);
  vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
  csm_cam->applyTransform(ecef_transform);
}

// Calc the time of first image line, last image line, elapsed time
// between these lines, and elapsed time per line.  This assumes a
// linear relationship between lines and time.
// TODO(oalexan1): This is fragile. Maybe it can be avoided.
void calcTimes(UsgsAstroLsSensorModel const* ls_model,
               double & earlier_line_time, double & later_line_time,
               double & elapsed_time, double & dt_per_line) {

  int numLines = ls_model->m_nLines;
  csm::ImageCoord imagePt;

  asp::toCsmPixel(vw::Vector2(0, 0), imagePt);
  earlier_line_time = ls_model->getImageTime(imagePt);

  asp::toCsmPixel(vw::Vector2(0, numLines - 1), imagePt);
  later_line_time = ls_model->getImageTime(imagePt);

  // See note in resampleModel().
  if (earlier_line_time > later_line_time)
    std::swap(earlier_line_time, later_line_time);
  
  elapsed_time = later_line_time - earlier_line_time;
  dt_per_line = elapsed_time / (numLines - 1.0);

  if (later_line_time <= earlier_line_time)
    vw::vw_throw(vw::ArgumentErr()
                 << "The time of the last line (in scanning order) must be larger than "
                 << "first line time.\n");
  
  return;
}

// Calculate the line index for first and last tabulated position.
// We always expect these to be less than first line index (0), and no less
// than last valid image line index (numLines - 1), respectively.
// TODO(oalexan1): This assumes a linear relationship between time and lines,
// which is fragile. At least need to check that this assumption is satisfied.
void calcFirstLastPositionLines(UsgsAstroLsSensorModel const* ls_model, 
                                double & beg_position_line, double & end_position_line) {

  double earlier_line_time = -1.0, later_line_time = -1.0, 
         elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
               dt_per_line);
  
  // Find time of first and last tabulated position.
  double bt = ls_model->m_t0Ephem;
  double et = bt + (ls_model->m_positions.size()/NUM_XYZ_PARAMS - 1) * ls_model->m_dtEphem;

  // Use the equation: time = earlier_line_time + line * dt_per_line.
  // See note in resampleModel() about scan direction.
  beg_position_line = (bt - earlier_line_time) / dt_per_line;
  end_position_line = (et - earlier_line_time) / dt_per_line;

  // Sanity checks
  if (beg_position_line > 1e-3) // allow for rounding errors 
    vw::vw_throw(vw::ArgumentErr() << "Line of first tabulated position is "
                 << beg_position_line << ", which is after first image line, which is "
                 << 0 << ".\n");
  int numLines = ls_model->m_nLines;
  if (end_position_line < numLines - 1 - 1e-3)  // allow for rounding errors
    vw::vw_throw(vw::ArgumentErr() << "Line of last tabulated position is "
                 << end_position_line << ", which is before last image line, which is "
                 << numLines - 1 << ".\n");
}
  
// Calculate the line index for first and last tabulated orientation.
// We always expect these to be less than first line index (0), and no less
// than last valid image line index (numLines - 1), respectively.
void calcFirstLastOrientationLines(UsgsAstroLsSensorModel const* ls_model, 
                                   double & beg_orientation_line, double & end_orientation_line) {

  double earlier_line_time = -1.0, later_line_time = -1.0, 
         elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
               dt_per_line);
  
  // Find time of first and last tabulated orientation.
  double bt = ls_model->m_t0Quat;
  double et = bt + (ls_model->m_quaternions.size()/NUM_QUAT_PARAMS - 1) * ls_model->m_dtQuat;
  
  // Use the equation: time = earlier_line_time + line * dt_per_line.
  beg_orientation_line = (bt - earlier_line_time) / dt_per_line;
  end_orientation_line = (et - earlier_line_time) / dt_per_line;

  // Sanity checks
  if (beg_orientation_line > 1e-3) // allow for rounding errors 
    vw::vw_throw(vw::ArgumentErr() << "Line of first tabulated orientation is "
                 << beg_orientation_line << ", which is after first image line, which is "
                   << 0 << ".\n");
  int numLines = ls_model->m_nLines;
  if (end_orientation_line < numLines - 1 - 1e-3)  // allow for rounding errors
    vw::vw_throw(vw::ArgumentErr() << "Line of last tabulated orientation is "
                 << end_orientation_line << ", which is before last image line, which is "
                   << numLines - 1 << ".\n");
}

// The provided tabulated positions, velocities and quaternions may be too few,
// so resample them with --num-lines-per-position and --num-lines-per-orientation,
// if those are set. Throughout this function the lines are indexed in the order
// they are acquired, which can be the reverse of the order they are eventually
// stored in the file if the scan direction is reverse.
// This function assumes the quaternions have been normalized.
void resampleModel(int num_lines_per_position, int num_lines_per_orientation,
                   UsgsAstroLsSensorModel * ls_model) {
  
  // The positions and quaternions can go way beyond the valid range of image lines,
  // so need to estimate how many of them are within the range.
  int numLines = ls_model->m_nLines;
  vw::vw_out() << "Number of lines: " << numLines << ".\n";

  double earlier_line_time = -1.0, later_line_time = -1.0, 
         elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
            dt_per_line);

  // Line index of first and last tabulated position
  double beg_position_line = -1.0, end_position_line = -1.0;
  calcFirstLastPositionLines(ls_model, beg_position_line, end_position_line);
  vw::vw_out() << std::setprecision (17) << "Line of first and last tabulated position: "
           << beg_position_line << ' ' << end_position_line << "\n";

  // Line index of first and last tabulated orientation
  double beg_orientation_line = -1.0, end_orientation_line = -1.0;
  calcFirstLastOrientationLines(ls_model, beg_orientation_line, end_orientation_line);
  vw::vw_out() << std::setprecision (17) << "Line of first and last tabulated orientation: "
           << beg_orientation_line << ' ' << end_orientation_line << "\n";

  double numInputLinesPerPosition = (numLines - 1) * ls_model->m_dtEphem / elapsed_time;
  double numInputLinesPerOrientation = (numLines - 1) * ls_model->m_dtQuat / elapsed_time;
  vw::vw_out() << "Number of image lines per input position: "
           << round(numInputLinesPerPosition) << "\n";
  vw::vw_out() << "Number of image lines per input orientation: "
           << round(numInputLinesPerOrientation) << "\n";

  if (num_lines_per_position > 0) {
    // Resample in such a way that first and last samples are preserved. This is tricky.
    double posFactor = double(numInputLinesPerPosition) / double(num_lines_per_position);
    if (posFactor <= 0.0)
      vw::vw_throw(vw::ArgumentErr() << "Invalid image.\n");

    int numOldMeas = ls_model->m_numPositions / NUM_XYZ_PARAMS;
    int numNewMeas = round(posFactor * (numOldMeas - 1.0)) + 1; // careful here
    numNewMeas = std::max(numNewMeas, 2);

    posFactor = double(numNewMeas - 1.0) / double(numOldMeas - 1.0);
    double currDtEphem = ls_model->m_dtEphem / posFactor;
    double numLinesPerPosition = (numLines - 1.0) * currDtEphem / elapsed_time;
    vw::vw_out() << "Resampled number of lines per position: "
             << numLinesPerPosition << "\n";
    std::vector<double> positions(NUM_XYZ_PARAMS * numNewMeas, 0);
    std::vector<double> velocities(NUM_XYZ_PARAMS * numNewMeas, 0);
    for (int ipos = 0; ipos < numNewMeas; ipos++) {
      double time = ls_model->m_t0Ephem + ipos * currDtEphem;
      asp::interpPositions(ls_model, time, &positions[NUM_XYZ_PARAMS * ipos]);
      asp::interpVelocities(ls_model, time, &velocities[NUM_XYZ_PARAMS * ipos]);
    }
    
    // Overwrite in the model. Time of first tabulated position does not change.
    ls_model->m_dtEphem = currDtEphem;
    ls_model->m_numPositions = positions.size();
    ls_model->m_positions = positions;
    ls_model->m_velocities = velocities;

    // Sanity check
    double new_beg_position_line = -1.0, new_end_position_line = -1.0;
    calcFirstLastPositionLines(ls_model, new_beg_position_line, new_end_position_line);
    if (std::abs(beg_position_line - new_beg_position_line) > 1.0e-3 ||
        std::abs(end_position_line - new_end_position_line) > 1.0e-3)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Resampling was done "
                   << "without preserving first and last tabulated position time.\n");
  }

  if (num_lines_per_orientation > 0) {
    // Resample in such a way that first and last samples are preserved. This is tricky.
    double posFactor = double(numInputLinesPerOrientation) / double(num_lines_per_orientation);
    if (posFactor <= 0.0)
      vw::vw_throw(vw::ArgumentErr() << "Invalid image.\n");

    int numOldMeas = ls_model->m_numQuaternions / NUM_QUAT_PARAMS;
    int numNewMeas = round(posFactor * (numOldMeas - 1.0)) + 1; // careful here
    numNewMeas = std::max(numNewMeas, 2);

    posFactor = double(numNewMeas - 1.0) / double(numOldMeas - 1.0);
    double currDtQuat = ls_model->m_dtQuat / posFactor;
    double numLinesPerOrientation = (numLines - 1.0) * currDtQuat / elapsed_time;
    vw::vw_out() << "Resampled number of lines per orientation: "
             << numLinesPerOrientation << "\n";
    std::vector<double> quaternions(NUM_QUAT_PARAMS * numNewMeas, 0);
    for (int ipos = 0; ipos < numNewMeas; ipos++) {
      double time = ls_model->m_t0Quat + ipos * currDtQuat;
      asp::interpQuaternions(ls_model, time, &quaternions[NUM_QUAT_PARAMS * ipos]);
    }
    
    // Overwrite in the model. Time of first tabulated orientation does not change.
    ls_model->m_dtQuat = currDtQuat;
    ls_model->m_numQuaternions = quaternions.size();
    ls_model->m_quaternions = quaternions;

    // Sanity check
    double new_beg_orientation_line = -1.0, new_end_orientation_line = -1.0;
    calcFirstLastOrientationLines(ls_model, new_beg_orientation_line, new_end_orientation_line);
    if (std::abs(beg_orientation_line - new_beg_orientation_line) > 1.0e-3 ||
        std::abs(end_orientation_line - new_end_orientation_line) > 1.0e-3)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Resampling was done "
                   << "without preserving first and last tabulated orientation time.\n");
  }

  return;
}

// Get the time at a given line. 
double get_time_at_line(double line, UsgsAstroLsSensorModel const* ls_model) {

  csm::ImageCoord csm_pix;
  vw::Vector2 pix(0, line);
  asp::toCsmPixel(pix, csm_pix);
  return ls_model->getImageTime(csm_pix);
}

// Get the line number at a given time. This assumes a linear relationship
// between them (rather than piecewise linear).
double get_line_at_time(double time, UsgsAstroLsSensorModel const* ls_model) {

  // All dt values in m_ls_model->m_intTimes must be equal, or else
  // the model is not linear in time.
  for (size_t i = 1; i < ls_model->m_intTimeLines.size(); i++) {
    if (std::abs(ls_model->m_intTimes[i] - ls_model->m_intTimes[0]) > 1e-10)
      vw::vw_throw(vw::ArgumentErr() 
                    << "Expecting a linear relation between time and image lines.\n");
  }
  
  int line0 = 0;
  int line1 = ls_model->m_nLines - 1;
  double time0 = get_time_at_line(line0, ls_model);
  double time1 = get_time_at_line(line1, ls_model);
    
  return line0 + (line1 - line0) * (time - time0) / (time1 - time0);
}

// Get camera center at a given time
vw::Vector3 get_camera_center_at_time(double time, UsgsAstroLsSensorModel const* ls_model) {
  csm::EcefCoord ecef = ls_model->getSensorPosition(time);
  return vw::Vector3(ecef.x, ecef.y, ecef.z);
}
  
// Get camera velocity at a given time
vw::Vector3 get_camera_velocity_at_time(double time, UsgsAstroLsSensorModel const* ls_model) {
  csm::EcefVector ecef = ls_model->getSensorVelocity(time);
  return vw::Vector3(ecef.x, ecef.y, ecef.z);
}

// Re implementation of the CSM wrapper around the USGS function for going from
// a pixel to a ray. Need it because cannot access the CSM model given the ls model.
vw::Vector3 pixel_to_vector(vw::Vector2 const& pix, UsgsAstroLsSensorModel const* ls_model,
                            double desired_precision) {

  csm::ImageCoord csm_pix;
  asp::toCsmPixel(pix, csm_pix);

  double achievedPrecision = -1.0; // will be modified in the function
  csm::EcefLocus locus = ls_model->imageToRemoteImagingLocus(csm_pix,
                                                             desired_precision,
                                                             &achievedPrecision);
  vw::Vector3 dir = ecefVectorToVector(locus.direction);
  return dir;
}

// Adjust the linescan model to correct for velocity aberration and/or
// atmospheric refraction.
void orbitalCorrections(asp::CsmModel * csm_model, 
                        bool correct_velocity_aberration,
                        bool correct_atmospheric_refraction,
                        double local_earth_radius, 
                        double mean_ground_elevation) {
  
  UsgsAstroLsSensorModel *ls_model 
    = dynamic_cast<UsgsAstroLsSensorModel*>(csm_model->m_gm_model.get());

  if (ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");

  // Collect the updated quaternions in a separate vector, to not interfere
  // with using the current ones during the correction process.
  std::vector<double> updated_quats(ls_model->m_quaternions.size());
  
  auto & qv = ls_model->m_quaternions; // shorthand
  for (int pos_it = 0; pos_it < ls_model->m_numQuaternions / 4; pos_it++) {

    double t = ls_model->m_t0Quat + pos_it * ls_model->m_dtQuat;
    vw::Vector3 cam_ctr = asp::get_camera_center_at_time(t, ls_model);
    vw::Vector3 vel = asp::get_camera_velocity_at_time(t, ls_model);

    // Get back to VW quaternion format
    vw::Quat q(qv[4*pos_it + 3], qv[4*pos_it + 0], qv[4*pos_it + 1], qv[4*pos_it + 2]);

    // Find the line at the given time based on a linear fit
    double line = asp::get_line_at_time(t, ls_model);

    // Find the cam_direction at the center of the line
    vw::Vector2 pix(ls_model->m_nSamples/2.0, line);
    vw::Vector3 cam_dir = csm_model->pixel_to_vector(pix);
    vw::Quaternion<double> corr_rot;
    
    if (correct_atmospheric_refraction) {
      // Find and apply the atmospheric refraction correction
      cam_dir = vw::camera::apply_atmospheric_refraction_correction
                      (cam_ctr, local_earth_radius, mean_ground_elevation, cam_dir, 
                       corr_rot); // output
      q = corr_rot * q;
    }

    if (correct_velocity_aberration) {
      // Find and apply the velocity aberration correction
      cam_dir = vw::camera::apply_velocity_aberration_correction
                      (cam_ctr, vel, local_earth_radius, cam_dir, 
                      corr_rot); // output
      q = corr_rot * q;
    }
    
    // Create the updated quaternions. ASP stores the quaternions as (w, x, y,
    // z). CSM wants them as x, y, z, w.
    updated_quats[4*pos_it + 0] = q.x();
    updated_quats[4*pos_it + 1] = q.y();
    updated_quats[4*pos_it + 2] = q.z();
    updated_quats[4*pos_it + 3] = q.w();
  }

  // Replace with the updated quaternions
  ls_model->m_quaternions = updated_quats;
}

} // end namespace asp
