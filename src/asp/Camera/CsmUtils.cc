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
// TODO(oalexan1): Move this to a new CsmModelUtils.cc file and
// call it from here and from LinescanDGModel.cc.
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
// TODO(oalexan1): Move this to a new CsmModelUtils.cc.
void interpPositions(UsgsAstroLsSensorModel * ls_model, double time,
                     double pos[3]) {
  int nOrder = 8;
  if (ls_model->m_platformFlag == 0)
    nOrder = 4;
  
  // TODO(oalexan1): What if the number of positions is < 4.
  lagrangeInterp(ls_model->m_numPositions / 3, &ls_model->m_positions[0],
                 ls_model->m_t0Ephem, ls_model->m_dtEphem,
                 time, 3, nOrder, pos);
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

} // end namespace asp
