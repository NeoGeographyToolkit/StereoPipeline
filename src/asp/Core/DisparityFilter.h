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


#ifndef __ASP_CORE_DISPARITY_FILTER_H__
#define __ASP_CORE_DISPARITY_FILTER_H__

#include <vw/Image/ImageView.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>

namespace asp {

  class ASPGlobalOptions; // forward declaration
  
  // Filter D_sub. Must be called only for alignment method affineepipolar, homography,
  // and local_epipolar. For now this is not in use as a dataset where this would help
  // was not found. It was tested though.
  void filter_D_sub(ASPGlobalOptions const& opt,
                    boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                    boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                    vw::cartography::Datum const& datum,
                    std::string const& d_sub_file,
                    vw::Vector2 const& outlier_removal_params);
  

} // End namespace asp

#endif//__ASP_CORE_DISPARITY_FILTER_H__
