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

#include <vw/FileIO/DiskImageResource.h>
#include <vw/Core/Log.h>

#include <asp/Core/ImageNormalization.h>
#include <asp/Core/StereoSettings.h>

#include <limits>

using namespace vw;

namespace asp {
  
  /// Returns the correct nodata value from the input images or the input options
  void get_nodata_values(boost::shared_ptr<vw::DiskImageResource> left_rsrc,
                         boost::shared_ptr<vw::DiskImageResource> right_rsrc,
                         float & left_nodata_value,
                         float & right_nodata_value){
    
    // The no-data value read from options overrides the value present in the image files.
    left_nodata_value  = std::numeric_limits<float>::quiet_NaN();
    right_nodata_value = std::numeric_limits<float>::quiet_NaN();
    if (left_rsrc->has_nodata_read ()) left_nodata_value  = left_rsrc->nodata_read();
    if (right_rsrc->has_nodata_read()) right_nodata_value = right_rsrc->nodata_read();
    
    float user_nodata = stereo_settings().nodata_value;
    if (!std::isnan(user_nodata)){
      
      if (user_nodata < left_nodata_value)
        vw_out(WarningMessage) << "It appears that the user-supplied no-data value is less than "
                               << "the no-data value of left image. This may not be what was "
                               << "intended.\n";
      if (user_nodata < right_nodata_value)
        vw_out(WarningMessage) << "It appears that the user-supplied no-data value is less than "
                               << "the no-data value of right image. This may not be what was "
                               << "intended.\n";

      left_nodata_value  = user_nodata;
      right_nodata_value = user_nodata;
    }

    return;
  }
  
}
