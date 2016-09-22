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


/// \file LinescanASTERModel.h
///
/// Linescan model for ASTER. Don't bother with time, just interpolate
/// between pointing vectors.
///
///
#ifndef __STEREO_CAMERA_LINESCAN_ASTER_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_ASTER_MODEL_H__

#include <vw/Math/Matrix.h>
#include <vw/Camera/LinescanModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/Extrinsics.h>


namespace asp {
  
  // References:
  // -----------
  // ASTER User Handbook Version 2
  // https://asterweb.jpl.nasa.gov/content/03_data/04_Documents/aster_user_guide_v2.pdf
  
  // The following paper has a blurb on the ASTER linescan camera.
  // Quantifying ice loss in the eastern Himalayas since 1974 using
  // declassified spy satellite imagery
  // Joshua M. Maurer, Summer B. Rupper, Joerg M. Schaefer

  // We do linear interpolation to find at each pixel the camera center
  // and pointing vector, and use a solver to back-project into the camera.
  // The useful load_ASTER_camera_model() function is at the end of the file.

  /// Specialization of the generic LinescanModel for ASTER satellites.
  class ASTERCameraModel : public vw::camera::CameraModel {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    ASTERCameraModel(std::vector< std::vector<vw::Vector2> > const& lattice_mat,
		     std::vector< std::vector<vw::Vector3> > const& sight_mat,
		     std::vector< std::vector<vw::Vector3> > const& world_sight_mat,
		     std::vector<vw::Vector3>                const& sat_pos,
		     vw::Vector2                             const& image_size,
		     boost::shared_ptr<vw::camera::CameraModel>     rpc_model);
    
    virtual ~ASTERCameraModel() {}
    virtual std::string type() const { return "LinescanASTER"; }

    // -- This set of functions implements virtual functions from LinescanModel.h --

    // TODO: See if we can port these local changes to the parent class
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point, vw::Vector2 const& start) const;
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point, double starty) const;
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point) const {
      return point_to_pixel(point, -1); // Redirect to the function with no initial guess
    }

    virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const;
    
    vw::Vector3 pixel_to_vector(vw::Vector2 const& pixel) const;

    boost::shared_ptr<vw::camera::CameraModel> get_rpc_model() { return m_rpc_model;}
    
  protected:

    std::vector< std::vector<vw::Vector2> > m_lattice_mat;
    std::vector< std::vector<vw::Vector3> > m_sight_mat;
    std::vector< std::vector<vw::Vector3> > m_world_sight_mat;
    std::vector<vw::Vector3>                m_sat_pos;
    vw::Vector2                             m_image_size;
    vw::camera::LinearPiecewisePositionInterpolation m_interp_sat_pos;
    vw::camera::SlerpGridPointingInterpolation m_interp_sight_mat;
    boost::shared_ptr<vw::camera::CameraModel> m_rpc_model; // rpc approx, for initial guess
  }; // End class ASTERCameraModel


  /// Load a ASTER camera model from an XML file.
  /// - This function does not take care of Xerces XML init/de-init, the caller must
  ///   make sure this is done before/after this function is called!
  boost::shared_ptr<ASTERCameraModel>
  load_ASTER_camera_model_from_xml(std::string const& path,
				   boost::shared_ptr<vw::camera::CameraModel> rpc_model);

}      // namespace asp


#endif//__STEREO_CAMERA_LINESCAN_ASTER_MODEL_H__
