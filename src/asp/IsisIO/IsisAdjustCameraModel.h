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


// IsisAdjustCameraModel
//
// IsisAdjustCameraModel is a clean break from IsisCameraModel
// as it only supports Frame Cameras and LineScan Cameras. No Map
// projection is supported.

#ifndef __VW_CAMERAMODEL_ISISADJUST_H__
#define __VW_CAMERAMODEL_ISISADJUST_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Camera/CameraModel.h>

#include <string>

#include <Pvl.h>
#include <Camera.h>

#include <boost/smart_ptr/shared_ptr.hpp>

namespace Isis {
  class AlphaCube;
  class CameraDetectorMap;
  class CameraDistortionMap;
  class CameraFocalPlaneMap;
}

namespace asp {
  class BaseEquation;
}

namespace vw {
namespace camera {

  class IsisAdjustCameraModel : public CameraModel {

  public:
    //-------------------------------------------------------------------
    //  Constructors / Deconstructor
    //-------------------------------------------------------------------
    IsisAdjustCameraModel( std::string cube_filename,
                           boost::shared_ptr<asp::BaseEquation> position_func,
                           boost::shared_ptr<asp::BaseEquation> pose_func );

    virtual ~IsisAdjustCameraModel(){}

    virtual std::string type() const { return "IsisAdjust"; }

    //-------------------------------------------------------------------
    //  Modification of Traditional Camera Routines
    //-------------------------------------------------------------------

    virtual Vector2 point_to_pixel( Vector3 const& point) const;
    virtual Vector3 pixel_to_vector( Vector2 const& pix ) const;
    virtual Vector3 camera_center( Vector2 const& pix = Vector2() ) const;
    virtual Quat camera_pose( Vector2 const& pix = Vector2() ) const;

    int lines() const { return m_camera->Lines(); }
    int samples() const { return m_camera->Samples(); }
    Vector3 sun_position( Vector2 const& pix = Vector2() ) const;

    std::string serial_number() const;

    //-------------------------------------------------------------------
    // Interfacing with equations
    //-------------------------------------------------------------------

    double ephemeris_time( Vector2 const& pix ) const;

    boost::shared_ptr<asp::BaseEquation>
      position_func() { return m_position_f; }

    boost::shared_ptr<asp::BaseEquation>
      pose_func() { return m_pose_f; }

  protected:
    boost::shared_ptr<asp::BaseEquation> m_position_f, m_pose_f;
    Isis::Pvl m_label;
    boost::shared_ptr<Isis::Camera> m_camera;
    boost::shared_ptr<Isis::AlphaCube> m_alphacube;
    Isis::CameraDistortionMap *m_distortmap;  // m_camera has ownership
    Isis::CameraFocalPlaneMap *m_focalmap;
    Isis::CameraDetectorMap   *m_detectmap;

  private:
    mutable Vector2 m_c_location; // Current pixel location (Prior to funcs)
    mutable Vector3 m_center;
    mutable Quat m_pose;
    void SetTime( Vector2 const& px, bool calc=true ) const;

    // These algorithms are different from IsisCameraModel in that
    // they use the adjustment functions in m_position_f and m_pose_f.
    class EphemerisLMA : public math::LeastSquaresModelBase<EphemerisLMA> {
      Vector3 m_point;
      boost::shared_ptr<Isis::Camera> m_camera;
      Isis::CameraDistortionMap *m_distortmap;
      Isis::CameraFocalPlaneMap *m_focalmap;
      boost::shared_ptr<asp::BaseEquation> m_position_f, m_pose_f;
    public:
      typedef Vector<double> result_type; // Back project result
      typedef Vector<double> domain_type; // Ephemeris time
      typedef Matrix<double> jacobian_type;

      inline EphemerisLMA( Vector3 const& point,
                           boost::shared_ptr<Isis::Camera> camera,
                           Isis::CameraDistortionMap* distortmap,
                           Isis::CameraFocalPlaneMap* focalmap,
                           boost::shared_ptr<asp::BaseEquation> position,
                           boost::shared_ptr<asp::BaseEquation> pose ) : m_point(point), m_camera(camera), m_distortmap(distortmap), m_focalmap(focalmap), m_position_f(position), m_pose_f(pose) {}

      inline result_type operator()( domain_type const& x ) const;
    };
  };

}}

#endif//__VW_CAMERAMODEL_ISISADJUST_H__
