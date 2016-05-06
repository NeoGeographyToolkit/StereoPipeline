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


/// \file PhotometricOutlier.h
///
/// Warning: This code was written with only the Apollo Metric data in mind

#ifndef __STEREO_CORE_PHOTOMETRIC_OUTLIER_H__
#define __STEREO_CORE_PHOTOMETRIC_OUTLIER_H__

#include <string>

// Forward declaration
namespace vw{
  namespace cartography{
    class GdalWriteOptions;
  }
}

namespace asp {
  void photometric_outlier_rejection( vw::cartography::GdalWriteOptions const& opt,
                                      std::string const& prefix,
                                      std::string const& input_disparity,
                                      std::string & output_disparity,
                                      int kernel_size );
}

#endif //__STEREO_CORE_PHOTOMETRIC_OUTLIER_H__
