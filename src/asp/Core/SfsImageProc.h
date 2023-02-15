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

/// \file SfsImageProc.h
/// Image processing routines for SfS

#ifndef __SFS_IMAGE_PROC_H__
#define __SFS_IMAGE_PROC_H__

#include <vw/Image/ImageView.h>

#include <vector>
namespace asp{

  // Find the unsigned distance to the perimeter of max lit
  // region. Add portions of this to the blending weights, in
  // proportion to how relevant the images are likely to
  // contribute. This improves the level of detail in borderline
  // areas, but does not scale up, where likely the portions are
  // sliced up too thinly.
void adjustBorderlineDataWeights(int cols, int rows,
                                 int blending_dist, double blending_power,
                                 std::vector<vw::ImageView<double>> & ground_weights);
  
} // end namespace asp

#endif // __SFS_IMAGE_PROC_H__

