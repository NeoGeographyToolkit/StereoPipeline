// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file IsisCameraModel.h
///
/// This file contains the ISIS camera model.
///
#ifndef __VW_CAMERAMODEL_ISIS_H__
#define __VW_CAMERAMODEL_ISIS_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>

// VW
#include <vw/Camera/CameraModel.h>
#include <vw/Math/LevenbergMarquardt.h>

// ISIS
#include <Pvl.h>
#include <AlphaCube.h>
#include <Camera.h>
#include <CameraFactory.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <CameraGroundMap.h>

namespace vw {
namespace camera {

  // This is largely just a shortened reimplementation of ISIS's
  // Camera.cpp.
  class IsisCameraModel : public CameraModel {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    IsisCameraModel(std::string cube_filename);

    virtual ~IsisCameraModel();
    virtual std::string type() const { return "Isis"; }

    //------------------------------------------------------------------
    // Methods
    //------------------------------------------------------------------

    //  Computes the image of the point 'point' in 3D space on the
    //  image plane.  Returns a pixel location (col, row) where the
    //  point appears in the image.
    virtual Vector2 point_to_pixel(Vector3 const& point) const;

    // Returns a (normalized) pointing vector from the camera center
    //  through the position of the pixel 'pix' on the image plane.
    virtual Vector3 pixel_to_vector (Vector2 const& pix) const;


    // Returns the position of the focal point of the camera
    virtual Vector3 camera_center(Vector2 const& pix = Vector2() ) const;

    //  Pose is a rotation which moves a vector in camera coordinates into world coordinates.
    virtual Quaternion<double> camera_pose(Vector2 const& pix = Vector2() ) const;

    // Returns the number of lines is the ISIS cube
    virtual int lines(void) const { return m_camera->Lines(); }

    // Returns the number of samples in the ISIS cube
    virtual int samples(void) const{ return m_camera->Samples(); }

    // Returns the serial number of the ISIS cube
    virtual std::string serial_number(void) const;

  protected:
    Isis::Pvl m_label;
    Isis::Camera *m_camera;
    Isis::AlphaCube *m_alphacube;
    Isis::CameraDistortionMap *m_distortmap;
    Isis::CameraFocalPlaneMap *m_focalmap;
    Isis::CameraDetectorMap   *m_detectmap;
    Isis::CameraGroundMap     *m_groundmap;

  private:
    class EphemerisLMA : public math::LeastSquaresModelBase<EphemerisLMA> {
      Vector3 m_point;
      Isis::Camera* m_camera;
      Isis::CameraDistortionMap *m_distortmap;
      Isis::CameraFocalPlaneMap *m_focalmap;
    public:
      typedef Vector<double> result_type; // Back project result
      typedef Vector<double> domain_type; // Ephemeris time
      typedef Matrix<double> jacobian_type;

      inline EphemerisLMA( Vector3 const& point,
                           Isis::Camera* camera,
                           Isis::CameraDistortionMap* distortmap,
                           Isis::CameraFocalPlaneMap* focalmap ) : m_point(point), m_camera(camera), m_distortmap(distortmap), m_focalmap(focalmap) {}

      inline result_type operator()( domain_type const& x ) const;
    };

    inline Vector2 project_using_current( Vector3 const& point ) const;
    Vector2 optimized_linescan_point_to_pixel( Vector3 const& point) const;
  };

}}

#endif  //__VW_CAMERA_ISIS_H__
