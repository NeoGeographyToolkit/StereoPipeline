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

#include <Rig/camera_image.h>
#include <Rig/rig_config.h>
#include <asp/Rig/RigCostFunction.h>
#include <Rig/transform_utils.h>
#include <iostream>

namespace rig {

// If applicable, set up the parameters block to fix the rig translations and/or rotations
void setUpFixRigOptions(bool no_rig, bool fix_rig_translations, bool fix_rig_rotations,
                        ceres::SubsetManifold*& constant_transform_manifold) {
  
  constant_transform_manifold = NULL;
  
  int beg = 0, end = 0;
  if (!no_rig && fix_rig_translations) {
    beg = 0; 
    end = 3;
  }
  
  if (!no_rig && fix_rig_rotations) {
    if (!fix_rig_translations)
      beg = 3; // only fix rotation
    end = rig::NUM_RIGID_PARAMS;
  }
  
  // Make a vector that goes from beg to end with increment 1
  std::vector<int> fixed_indices;
  for (int it = beg; it < end; it++)
    fixed_indices.push_back(it);
  
  if (!fixed_indices.empty())
    constant_transform_manifold = new ceres::SubsetManifold(rig::NUM_RIGID_PARAMS,
                                                            fixed_indices);
}

// TODO(oalexan1): Move the rig cost functions here.

}  // end namespace rig
