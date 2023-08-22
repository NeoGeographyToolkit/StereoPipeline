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
#include <asp/Camera/JitterSolveUtils.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoReferenceUtils.h>

#include <usgscsm/Utilities.h>

namespace asp {

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

  // Normalize the quaternions. Put them back in the model.
  for (size_t i = 0; i < 4; i++) {
    q[i] /= norm;
    frame_model->setParameterValue(i + 3, q[i]);
  }

  return;
}

// Get quaternions. This duplicates the UsgsAstroLsSensorModel function as that one is private
// TODO(oalexan1): Call this from LinescanDGModel.cc.
void interpQuaternions(UsgsAstroLsSensorModel * ls_model, double time,
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
void interpPositions(UsgsAstroLsSensorModel * ls_model, double time,
                     double pos[3]) {
  // Call the above wrapper
  interpPositions(ls_model->m_positions, ls_model->m_t0Ephem, ls_model->m_dtEphem,
                  ls_model->m_platformFlag, time, pos);
}

// Get positions. Based on the UsgsAstroLsSensorModel code.
// TODO(oalexan1): Move this to a new CsmModelUtils.cc file and
void interpVelocities(UsgsAstroLsSensorModel * ls_model, double time,
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
vw::Vector3 linearInterp(double t1, double t2, vw::Vector3 const& P1, vw::Vector3 const& P2, 
                         double t) {
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
    vw::Vector3 pos(positions_in[start + 0], positions_in[start + 1], positions_in[start + 2]);
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

} // end namespace asp
