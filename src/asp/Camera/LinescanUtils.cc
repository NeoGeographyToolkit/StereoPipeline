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

// Functions that know about the many kinds of linescan cameras

#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Camera/LinescanPleiadesModel.h>

namespace asp {

// Find the underlying CSM camera. Applies only to CSM, Pleiades, and DG.
asp::CsmModel * csm_model(boost::shared_ptr<vw::camera::CameraModel> cam,
                            std::string const& stereo_session) {

  asp::CsmModel * csm_model = NULL;
  
  // If we have a DG model, then the CSM model is a member of it.
  // TODO(oalexan1): This is temporary. Need to move wholesale
  // to CSM model in DG, as done for Pleiades. 
  // TODO(oalexan1): Then remove the stereo_session argument.
  if (stereo_session != "dg") {
    csm_model = dynamic_cast<asp::CsmModel*>
      (vw::camera::unadjusted_model(cam.get()));
  } else {
    DGCameraModel * dg_model = dynamic_cast<asp::DGCameraModel*>
      (vw::camera::unadjusted_model(cam.get()));
    if (dg_model == NULL) 
      vw::vw_throw(vw::ArgumentErr() << "Expected a DG camera model.");
    csm_model = dg_model->m_csm_model.get();
  }

  if (csm_model == NULL) 
      vw::vw_throw(vw::ArgumentErr() << "Expected a CSM camera model.");
    
  return csm_model;
}

} // end namespace asp