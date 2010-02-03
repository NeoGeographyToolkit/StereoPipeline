// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file IsisInterfaceLineScane.h
///
/// Line Scan Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_LINESCAN_H__
#define __ASP_ISIS_INTERFACE_LINESCAN_H__

// VW & ASP
#include <vw/Math/LevenbergMarquardt.h>
#include <asp/IsisIO/IsisInterface.h>

// Isis
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <AlphaCube.h>

namespace asp {
namespace isis {

  class IsisInterfaceLineScan : public IsisInterface {

  public:
    IsisInterfaceLineScan( std::string const& file );

    virtual ~IsisInterfaceLineScan() {}

    virtual std::string type()  { return "LineScan"; }

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
    Isis::CameraDistortionMap *m_distortmap;
    Isis::CameraFocalPlaneMap *m_focalmap;
    Isis::CameraDetectorMap   *m_detectmap;
    Isis::AlphaCube           *m_alphacube;

  private:

    // Custom Fuctions
    mutable vw::Vector2 m_c_location;  // Clean away some consts
    mutable vw::Vector3 m_center;
    mutable vw::Quaternion<double> m_pose;
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

#endif//__ASP_ISIS_INTERFACE_LINESCAN_H__
