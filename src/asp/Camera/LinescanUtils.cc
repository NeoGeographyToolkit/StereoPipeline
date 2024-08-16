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
#include <asp/Camera/LinescanASTERModel.h>

namespace asp {

// Find the underlying CSM camera. Applies only to CSM, Pleiades, ASTER, and DG.
asp::CsmModel * csm_model(boost::shared_ptr<vw::camera::CameraModel> cam,
                            std::string const& stereo_session) {
  
  asp::CsmModel * csm_model = NULL;
  
  // TODO(oalexan1): Remove the stereo_session when ASTER inherits from CSM.
  if (stereo_session == "aster") {
    // TODO(oalexan1): The ASTER model must inherit from CSM.
    ASTERCameraModel * aster_model = dynamic_cast<asp::ASTERCameraModel*>
      (vw::camera::unadjusted_model(cam.get()));
    if (aster_model == NULL) 
       vw::vw_throw(vw::ArgumentErr() << "Expected an ASTER camera model.");
    csm_model = &aster_model->m_csm_model;
  } else {
    // The Pleiades and DG models will come here as there is direct inheritance from CSM
    csm_model = dynamic_cast<asp::CsmModel*>(vw::camera::unadjusted_model(cam.get()));
  }

  if (csm_model == NULL) 
      vw::vw_throw(vw::ArgumentErr() << "Expected a CSM camera model.");
    
  return csm_model;
}

} // end namespace asp