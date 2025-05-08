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

/// \file PointCloudAlignment.h
///

// Utilities used in cloud alignment that make use of PDAL. 

#ifndef __ASP_CORE_POINT_CLOUD_ALIGNMENT_H__
#define __ASP_CORE_POINT_CLOUD_ALIGNMENT_H__

#include <asp/asp_config.h>
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1

#include <asp/Core/EigenUtils.h> // defines DoubleMatrix
#include <pointmatcher/PointMatcher.h> // defines PointMatcher

#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>

#include <string>
#include <vector>

namespace vw {
  
  class GdalWriteOptions;
  namespace cartography {
    class GeoReference;
  }
}

namespace asp {
 
std::int64_t load_las(std::string const& file_name,
                      std::int64_t num_points_to_load,
                      vw::BBox2 const& lonlat_box,
                      vw::cartography::GeoReference const& geo,
                      bool verbose,
                      bool calc_shift,
                      // Outputs
                      vw::Vector3 & shift,
                      DoubleMatrix & data);

// Apply a given transform to a LAS file and save it.
void apply_transform_to_las(std::string const& input_file,
                            std::string const& output_file,
                            PointMatcher<asp::RealT>::Matrix const& T);

} // End namespace asp

#endif // ASP_HAVE_PKG_ISISIO

#endif // __ASP_CORE_POINT_CLOUD_ALIGNMENT_H__
