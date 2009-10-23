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

#include <vw/Camera/CameraModel.h>

namespace vw {
namespace camera {

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
    virtual int getLines(void) const;

    // Returns the number of samples in the ISIS cube
    virtual int getSamples(void) const;

    // Returns the serial number of the ISIS cube
    virtual std::string serial_number(void) const;

  protected:
    mutable double m_current_line, m_current_sample;
    mutable double m_max_ephemeris, m_min_ephemeris;
    void set_image(double sample, double line) const;

    // A void pointer which is cast to Isis::Camera* in the
    // implentation of the methods below.
    void* m_isis_camera_ptr;
    void* m_isis_cube;
  };

}}      // namespace vw::camera

#endif  //__VW_CAMERA_ISIS_H__
