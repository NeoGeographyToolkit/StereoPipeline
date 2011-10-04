// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#ifndef __STEREO_SESSION_RPC_MODEL_H__
#define __STEREO_SESSION_RPC_MODEL_H__

#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>

namespace asp {

  // TODO: Need to work out a different way to triangulate. Our
  //   standard midpoint method doesn't seem to work.
  class RPCModel : public vw::camera::CameraModel {
    vw::cartography::Datum m_datum;

    // Scaling parameters
    vw::Vector<double,20> m_line_num_coeff, m_line_den_coeff,
      m_sample_num_coeff, m_sample_den_coeff;
    vw::Vector2 m_xy_offset;
    vw::Vector2 m_xy_scale;
    vw::Vector3 m_lonlatheight_offset;
    vw::Vector3 m_lonlatheight_scale;
    vw::BBox2 m_lonlat_bbox;

    void initialize( vw::DiskImageResourceGDAL* resource );
  public:
    RPCModel( std::string const& filename );
    RPCModel( vw::DiskImageResourceGDAL* resource );

    virtual std::string type() const { return "RPC"; }
    virtual ~RPCModel() {}

    // Standard Access Methods (Most of these will fail because they
    // don't apply well to RPC.)
    virtual vw::Vector2 point_to_pixel( vw::Vector3 const& point ) const;
    virtual vw::Vector3 pixel_to_vector( vw::Vector2 const& pix ) const {
      vw::vw_throw( vw::NoImplErr() << "RPCModel: Pixel to Vector not implemented" );
      return vw::Vector3();
    }
    virtual vw::Vector3 camera_center( vw::Vector2 const& pix ) const {
      vw::vw_throw( vw::NoImplErr() << "RPCModel: Camera center not implemented" );
      return vw::Vector3();
    }

    // Access to constants
    typedef vw::Vector<double,20> CoeffVec;
    vw::cartography::Datum const& datum() const { return m_datum; }
    CoeffVec const& line_num_coeff() const   { return m_line_num_coeff; }
    CoeffVec const& line_den_coeff() const   { return m_line_den_coeff; }
    CoeffVec const& sample_num_coeff() const { return m_sample_num_coeff; }
    CoeffVec const& sample_den_coeff() const { return m_sample_den_coeff; }
    vw::Vector2 const& xy_offset() const     { return m_xy_offset; }
    vw::Vector2 const& xy_scale() const      { return m_xy_scale; }
    vw::Vector3 const& lonlatheight_offset() const { return m_lonlatheight_offset; }
    vw::Vector3 const& lonlatheight_scale() const  { return m_lonlatheight_scale; }
    vw::BBox2 const& lonlat_bbox() const     { return m_lonlat_bbox; }

    static CoeffVec calculate_terms( vw::Vector3 const& v ) {
      CoeffVec result;
      result[0] = 1.0;
      result[1] = v.x();
      result[2] = v.y();
      result[3] = v.z();
      result[4] = v.x() * v.y();
      result[5] = v.x() * v.z();
      result[6] = v.y() * v.z();
      result[7] = v.x() * v.x();
      result[8] = v.y() * v.y();
      result[9] = v.z() * v.z();

      result[10] = v.x() * v.y() * v.z();
      result[11] = v.x() * v.x() * v.x();
      result[12] = v.x() * v.y() * v.y();
      result[13] = v.x() * v.z() * v.z();
      result[14] = v.x() * v.x() * v.y();
      result[15] = v.y() * v.y() * v.y();
      result[16] = v.y() * v.z() * v.z();
      result[17] = v.x() * v.x() * v.z();
      result[18] = v.y() * v.y() * v.z();
      result[19] = v.z() * v.z() * v.z();
      return result;
    }

  };

}

#endif//__STEREO_SESSION_RPC_CAMERA_MODEL_H__
