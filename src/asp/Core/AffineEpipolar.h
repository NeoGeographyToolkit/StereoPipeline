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


#ifndef __ASP_CORE_AFFINEEPIPOLAR_H__
#define __ASP_CORE_AFFINEEPIPOLAR_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>

namespace vw {
  namespace ip {
    struct InterestPoint;
  }
}

namespace asp {

  // Takes matched interest points and returns ideal rectifcation
  // transforms in left_matrix and right_matrix. This function returns
  // the ideal image size. (It reduces the image sizes to only the
  // intersect of the two images.)
  vw::Vector2i
  affine_epipolar_rectification( vw::Vector2i const& left_size,
                                 vw::Vector2i const& right_size,
                                 std::vector<vw::ip::InterestPoint> const& left,
                                 std::vector<vw::ip::InterestPoint> const& right,
                                 vw::Matrix<double>& left_matrix,
                                 vw::Matrix<double>& right_matrix );

}

#endif//__ASP_CORE_AFFINEEPIPOLAR_H__
