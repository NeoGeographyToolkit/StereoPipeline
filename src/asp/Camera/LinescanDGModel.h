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


/// \file LinescanDGModel.h
///
/// A generic linescan camera model object
///
///
#ifndef __STEREO_CAMERA_LINESCAN_DG_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_DG_MODEL_H__

#include <vw/Camera/LinescanModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/Extrinsics.h>

namespace asp {


  

  // The intrinisic model expects +Z to be point out the camera. +X is
  // the column direction of the image and is perpendicular to
  // direction of flight. +Y is the row direction of the image (down
  // the image); it is also the flight direction. This is different
  // from Digital Globe model, but you can rotate pose beforehand.

  // The standard class variant is:
  //      typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation,
  //                      			  vw::camera::SLERPPoseInterpolation> DGCameraModel;

  // The useful load_dg_camera_model() function is at the end of the file.

  /// Specialization of the generic LinescanModel for Digital Globe satellites.
  /// - Two template types are left floating so that AdjustedLinescanDGModel can modify them.
  template <class PositionFuncT, class PoseFuncT>
  class LinescanDGModel : public vw::camera::LinescanModel<PositionFuncT,
                                                           vw::camera::LinearPiecewisePositionInterpolation,
                                                           PoseFuncT,
                                                           vw::camera::TLCTimeInterpolation> {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    LinescanDGModel(PositionFuncT const& position,
  		              vw::camera::LinearPiecewisePositionInterpolation const& velocity,
		                PoseFuncT     const& pose,
		                vw::camera::TLCTimeInterpolation                 const& time,
		                vw::Vector2i  const& image_size,
		                vw::Vector2   const& detector_origin,
		                double        const  focal_length
		    ) : vw::camera::LinescanModel<PositionFuncT,
                                      vw::camera::LinearPiecewisePositionInterpolation,
                                      PoseFuncT,
                                      vw::camera::TLCTimeInterpolation>(position, velocity, pose, 
                                                                        time, image_size, detector_origin, 
                                        		                            focal_length, true) {} // Always correct velocity aberration
    virtual ~LinescanDGModel() {}
    virtual std::string type() const { return "LinescanDG"; }

    /// Create a fake pinhole model. It will return the same results
    /// as the linescan camera at current line y, but we will use it
    /// by extension at neighboring lines as well.
    vw::camera::PinholeModel linescan_to_pinhole(double y) const;

  }; // End class LinescanDGModel
  
  /// This is the standard DG implementation
  typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation,
                  			  vw::camera::SLERPPoseInterpolation> DGCameraModel;

  /// Load a DG camera model from an XML file.
  /// - This function does not take care of Xerces XML init/de-init, the caller must
  ///   make sure this is done before/after this function is called!
  boost::shared_ptr<DGCameraModel> load_dg_camera_model_from_xml(std::string const& path);

}      // namespace asp

#include <asp/Camera/LinescanDGModel.tcc>


#endif//__STEREO_CAMERA_LINESCAN_DG_MODEL_H__
