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


#ifndef __ASP_TOOLS_JITTERADJUST_H__
#define __ASP_TOOLS_JITTERADJUST_H__

#include <vw/Camera/CameraModel.h>

namespace asp{

  void jitter_adjust(std::vector<std::string> const& image_files,
                     std::vector<std::string> const& camera_files,
                     std::vector< boost::shared_ptr<vw::camera::CameraModel> > const& camera_models,
                     std::string const& out_prefix,
		     std::string const& session,
                     std::string const& match_file,
                     int num_threads);
}

#endif // __ASP_TOOLS_JITTERADJUST_H__
