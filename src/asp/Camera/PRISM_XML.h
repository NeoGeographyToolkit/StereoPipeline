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

// Utilities for parsing PRISM .DIMA files in XML format.

#ifndef __STEREO_CAMERA_PRISM_XML_H__
#define __STEREO_CAMERA_PRISM_XML_H__

#include <vw/Math/Vector.h>

#include <vector>
#include <string>

namespace asp {

void parsePrismXml(std::string const& dim_file,
                   int & ncols, int & nrows, std::string & view, 
                   double & first_line_time, double & last_line_time,
                   std::vector<vw::Vector3> & positions,
                   std::vector<vw::Vector3> & velocities,
                   std::vector<double> & position_times,
                   std::vector<vw::Vector3> & rpy,
                   std::vector<double> & rpy_times);

} //end namespace asp

#endif//__STEREO_CAMERA_PRISM_XML_H__
