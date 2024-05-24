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

} // end namespace asp
