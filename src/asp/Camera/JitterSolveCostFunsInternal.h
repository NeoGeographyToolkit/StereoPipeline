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

// Internal helpers shared between the jitter cost-function translation units.
// Not part of the public JitterSolveCostFuns.h API.

#ifndef __ASP_CAMERA_JITTER_SOLVE_COST_FUNS_INTERNAL_H__
#define __ASP_CAMERA_JITTER_SOLVE_COST_FUNS_INTERNAL_H__

#include <vw/Math/Vector.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>

namespace asp {

// Find the positions and orientations in the current linescan model
// that can affect the given pixel.
void calcPosQuatIndexBounds(double line_extra,
                            UsgsAstroLsSensorModel * ls_model,
                            vw::Vector2 const& observation,
                            // Outputs
                            int & begPosIndex,  int & endPosIndex,
                            int & begQuatIndex, int & endQuatIndex);

} // end namespace asp

#endif //__ASP_CAMERA_JITTER_SOLVE_COST_FUNS_INTERNAL_H__
