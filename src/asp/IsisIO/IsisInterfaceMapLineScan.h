// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file IsisInterfaceMapLineScane.h
///
/// Map Projected Line Scan Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
#define __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__

// ASP & VW
#include <asp/IsisIO/IsisInterface.h>
#include <vw/Math/LevenbergMarquardt.h>

// Isis
#include <Projection.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraGroundMap.h>
#include <CameraFocalPlaneMap.h>
#include <AlphaCube.h>

namespace asp {
namespace isis {

  class IsisInterfaceMapLineScan : public IsisInterface {

  public:
    IsisInterfaceMapLineScan( std::string const& file );

    virtual ~IsisInterfaceMapLineScan() {
      if ( m_projection )
        delete m_projection;
      if ( m_alphacube )
        delete m_alphacube;
    }

    virtual std::string type()  { return "MapLineScan"; }

    // Standard Methods
    //-------------------------------------------------

    virtual vw::Vector2
      point_to_pixel( vw::Vector3 const& point ) const;
    virtual vw::Vector3
      pixel_to_vector( vw::Vector2 const& pix ) const;
    virtual vw::Vector3
      camera_center( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;
    virtual vw::Quaternion<double>
      camera_pose( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;

  protected:

    // Custom Variables
    Isis::Projection          *m_projection;
    Isis::CameraDetectorMap   *m_detectmap;
    Isis::CameraDistortionMap *m_distortmap;
    Isis::CameraGroundMap     *m_groundmap;
    Isis::CameraFocalPlaneMap *m_focalmap;
    Isis::AlphaCube           *m_alphacube;

  private:

    // Custom Functions
    mutable vw::Vector2 m_c_location;
    mutable vw::Vector3 m_center;
    mutable vw::Quaternion<double> m_pose;
    double m_radii[3];
    void SetTime( vw::Vector2 const& px,
                  bool calc=false ) const;
    class EphemerisLMA : public vw::math::LeastSquaresModelBase<EphemerisLMA> {
      vw::Vector3 m_point;
      Isis::Camera* m_camera;
      Isis::CameraDistortionMap *m_distortmap;
      Isis::CameraFocalPlaneMap *m_focalmap;
    public:
      typedef vw::Vector<double> result_type; // Back project result
      typedef vw::Vector<double> domain_type; // Ephemeris time
      typedef vw::Matrix<double> jacobian_type;

      inline EphemerisLMA( vw::Vector3 const& point,
                           Isis::Camera* camera,
                           Isis::CameraDistortionMap* distortmap,
                           Isis::CameraFocalPlaneMap* focalmap ) : m_point(point), m_camera(camera), m_distortmap(distortmap), m_focalmap(focalmap) {}

      inline result_type operator()( domain_type const& x ) const;
    };
  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
