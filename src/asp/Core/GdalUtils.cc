// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

/// \file GdalUtils.cc
///

#include <asp/Core/GdalUtils.h>

using namespace vw;
using namespace vw::cartography;

// Unless user-specified, compute the rounding error for a given
// planet (a point on whose surface is given by 'shift'). Return an
// inverse power of 2, 1/2^10 for Earth and proportionally less for
// smaller bodies.
double asp::get_rounding_error(vw::Vector3 const& shift, double rounding_error) {

  // Do nothing if the user specified it.
  if (rounding_error > 0.0) return rounding_error;

  double len = norm_2(shift);
  VW_ASSERT(len > 0,  vw::ArgumentErr()
            << "Expecting positive length in get_rounding_error().");
  rounding_error = 1.5e-10*len;
    rounding_error = pow(2.0, round(log(rounding_error)/log(2.0)));
    return rounding_error;
}
