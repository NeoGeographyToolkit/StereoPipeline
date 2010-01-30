// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __VW_CAMERAMODEL_ISISADJUST_H__
#define __VW_CAMERAMODEL_ISISADJUST_H__

// ASP
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/Equation.h>

namespace vw {
namespace camera {

  class IsisAdjustCameraModel : public IsisCameraModel {

  public:
    //-------------------------------------------------------------------
    //  Constructors / Deconstructor
    //-------------------------------------------------------------------
    IsisAdjustCameraModel( std::string cube_filename,
                           boost::shared_ptr<BaseEquation> position_func,
                           boost::shared_ptr<BaseEquation> pose_func );

    virtual ~IsisAdjustCameraModel();

    virtual std::string type() const { return "IsisAdjust"; }

    //-------------------------------------------------------------------
    //  Modification of Traditional Camera Routines
    //-------------------------------------------------------------------

    virtual Vector2 point_to_pixel( Vector3 const& point) const;

    virtual Vector3 pixel_to_vector( Vector2 const& pix ) const;

    virtual Vector3 camera_center( Vector2 const& pix = Vector2() ) const;

    virtual Quaternion<double> camera_pose( Vector2 const& pix = Vector2() ) const;

    //-------------------------------------------------------------------
    // Interfacing with equations
    //-------------------------------------------------------------------

    boost::shared_ptr<BaseEquation>
      position_func ( void ) { return m_position_f; }

    boost::shared_ptr<BaseEquation>
      pose_func ( void ) { return m_pose_f; }

  protected:
    boost::shared_ptr<BaseEquation> m_position_f, m_pose_f;

  private:
    // These algorithms are different from IsisCameraModel in that
    // they use the adjustment functions in m_position_f and m_pose_f.
    class EphemerisLMA : public math::LeastSquaresModelBase<EphemerisLMA> {
      Vector3 m_point;
      Isis::Camera* m_camera;
      Isis::CameraDistortionMap *m_distortmap;
      Isis::CameraFocalPlaneMap *m_focalmap;
      boost::shared_ptr<BaseEquation> m_position_f, m_pose_f;
    public:
      typedef Vector<double> result_type; // Back project result
      typedef Vector<double> domain_type; // Ephemeris time
      typedef Matrix<double> jacobian_type;

      inline EphemerisLMA( Vector3 const& point,
                           Isis::Camera* camera,
                           Isis::CameraDistortionMap* distortmap,
                           Isis::CameraFocalPlaneMap* focalmap,
                           boost::shared_ptr<BaseEquation> position,
                           boost::shared_ptr<BaseEquation> pose ) : m_point(point), m_camera(camera), m_distortmap(distortmap), m_focalmap(focalmap), m_position_f(position), m_pose_f(pose) {}

      inline result_type operator()( domain_type const& x ) const;
    };

    inline Vector2 project_using_current( Vector3 const& point ) const;
    Vector2 optimized_linescan_point_to_pixel( Vector3 const& point ) const;
  };

}}

#endif//__VW_CAMERAMODEL_ISISADJUST_H__
