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


/// \file StereoSessionFactory.h
///

#ifndef __STEREO_SESSION_FACTORY_H__
#define __STEREO_SESSION_FACTORY_H__

#include <asp/Sessions/StereoSession.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Cartography/GeoReference.h>

namespace asp {

class StereoSessionFactory {
  public:
    /// Given the input arguments, selects the correct type of StereoSession and initializes it.
    static StereoSession* create(std::string & session_type, // in-out variable
                                 vw::cartography::GdalWriteOptions const& options,
                                 std::string const& left_image_file   = "",
                                 std::string const& right_image_file  = "",
                                 std::string const& left_camera_file  = "",
                                 std::string const& right_camera_file = "",
                                 std::string const& out_prefix        = "",
                                 std::string const& input_dem         = "",
                                 const bool allow_map_promote=true); // If true, allow isis to become isismapisis based on dem availability

  private:
    StereoSessionFactory() {} // Prevent construction of static-only class

}; // End class StereoSessionFactory

} // end namespace asp

#include <asp/Sessions/StereoSessionFactory.tcc>

#endif // __STEREO_SESSION_FACTORY_H__
