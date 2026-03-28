// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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


/// \file LinescanASTERModel.h
///
/// Linescan model for ASTER. Fits a CSM linescan model to the ASTER
/// lattice of sight vectors and satellite positions.
///
#ifndef __STEREO_CAMERA_LINESCAN_ASTER_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_ASTER_MODEL_H__

#include <asp/Camera/CsmModel.h>

#include <vw/Math/Matrix.h>

namespace asp {

  // References:
  // -----------
  // ASTER User Handbook Version 2
  // https://asterweb.jpl.nasa.gov/content/03_data/04_Documents/aster_user_guide_v2.pdf

  // https://lpdaac.usgs.gov/documents/175/ASTER_L1_Product_Specifications.pdf

  // The following paper has a blurb on the ASTER linescan camera.
  // Quantifying ice loss in the eastern Himalayas since 1974 using
  // declassified spy satellite imagery
  // Joshua M. Maurer, Summer B. Rupper, Joerg M. Schaefer

  /// CSM-based linescan camera model for ASTER satellites.
  /// Inherits from CsmModel, following the same pattern as Pleiades,
  /// DG, SPOT, and PeruSat.
  class ASTERCameraModel : public asp::CsmModel {

  public:
    ASTERCameraModel(std::vector<std::vector<vw::Vector2>> const& lattice_mat,
		     std::vector<std::vector<vw::Vector3>> const& sight_mat,
		     std::vector<std::vector<vw::Vector3>> const& world_sight_mat,
		     std::vector<vw::Vector3>               const& sat_pos,
		     vw::Vector2i                            const& image_size,
		     boost::shared_ptr<vw::camera::CameraModel>     rpc_model);

    virtual ~ASTERCameraModel() {}
    virtual std::string type() const { return "LinescanASTER"; }

    boost::shared_ptr<vw::camera::CameraModel> get_rpc_model() { return m_rpc_model; }

  protected:
    boost::shared_ptr<vw::camera::CameraModel> m_rpc_model; // rpc approx, for initial guess

  }; // End class ASTERCameraModel


  /// Load a ASTER camera model from an XML file.
  /// - This function does not take care of Xerces XML init/de-init, the caller must
  ///   make sure this is done before/after this function is called!
  boost::shared_ptr<ASTERCameraModel>
  load_ASTER_camera_model_from_xml(std::string const& path,
				   boost::shared_ptr<vw::camera::CameraModel> rpc_model);
  
} // end namespace asp

#endif//__STEREO_CAMERA_LINESCAN_ASTER_MODEL_H__
