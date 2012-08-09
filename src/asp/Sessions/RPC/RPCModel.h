// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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

// RPC model and triangulation. Following the paper:

// Jacek Grodecki, Gene Dial and James Lutes, "Mathematical Model for
// 3D Feature Extraction from Multiple Satellite Images Described by
// RPCs." Proceedings of ASPRS 2004 Conference, Denver, Colorado, May,
// 2004.
  
#ifndef __STEREO_SESSION_RPC_MODEL_H__
#define __STEREO_SESSION_RPC_MODEL_H__

#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>

namespace asp {

  class RPCModel : public vw::camera::CameraModel {
    vw::cartography::Datum m_datum;

    // Scaling parameters
    vw::Vector<double,20> m_line_num_coeff, m_line_den_coeff,
      m_sample_num_coeff, m_sample_den_coeff;
    vw::Vector2 m_xy_offset;
    vw::Vector2 m_xy_scale;
    vw::Vector3 m_lonlatheight_offset;
    vw::Vector3 m_lonlatheight_scale;

    void initialize( vw::DiskImageResourceGDAL* resource );
  public:
    RPCModel( std::string const& filename );
    RPCModel( vw::DiskImageResourceGDAL* resource );
    RPCModel( vw::cartography::Datum const& datum,
              vw::Vector<double,20> const& line_num_coeff,
              vw::Vector<double,20> const& line_den_coeff,
              vw::Vector<double,20> const& samp_num_coeff,
              vw::Vector<double,20> const& samp_den_coeff,
              vw::Vector2 const& xy_offset,
              vw::Vector2 const& xy_scale,
              vw::Vector3 const& lonlatheight_offset,
              vw::Vector3 const& lonlatheight_scale );

    virtual std::string type() const { return "RPC"; }
    virtual ~RPCModel() {}

    // Standard Access Methods (Most of these will fail because they
    // don't apply well to RPC.)
    virtual vw::Vector2 point_to_pixel( vw::Vector3 const& point ) const;
    virtual vw::Vector3 pixel_to_vector( vw::Vector2 const& /*pix*/ ) const {
      vw::vw_throw( vw::NoImplErr() << "RPCModel: Pixel to Vector not implemented" );
      return vw::Vector3();
    }
    virtual vw::Vector3 camera_center( vw::Vector2 const& /*pix*/ ) const {
      vw::vw_throw( vw::NoImplErr() << "RPCModel: Camera center not implemented" );
      return vw::Vector3();
    }
    vw::Vector2 geodetic_to_pixel( vw::Vector3 const& point ) const;

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

    static CoeffVec calculate_terms( vw::Vector3 const& v ) {
      double x = v.x(), y = v.y(), z = v.z();
      CoeffVec result;
      result[ 0] = 1.0;
      result[ 1] = x;
      result[ 2] = y;
      result[ 3] = z;
      result[ 4] = x*y;
      result[ 5] = x*z;
      result[ 6] = y*z;
      result[ 7] = x*x;
      result[ 8] = y*y;
      result[ 9] = z*z;
      result[10] = x*y*z;
      result[11] = x*x*x;
      result[12] = x*y*y;
      result[13] = x*z*z;
      result[14] = x*x*y;
      result[15] = y*y*y;
      result[16] = y*z*z;
      result[17] = x*x*z;
      result[18] = y*y*z;
      result[19] = z*z*z;
      return result;
    }
    
    static vw::Matrix<double, 20, 3> terms_Jacobian( vw::Vector3 const& v ) {

      // Partial derivatives of the terms returned by the
      // calculate_terms() function.

      vw::Matrix<double, 20, 3> M;
      double x = v.x(), y = v.y(), z = v.z();
      
      // df/dx            df/dy               df/dz               // f 
      M[ 0][0] = 0.0;     M[ 0][1] = 0.0;     M[ 0][2] = 0.0;     // 1 
      M[ 1][0] = 1.0;     M[ 1][1] = 0.0;     M[ 1][2] = 0.0;     // x 
      M[ 2][0] = 0.0;     M[ 2][1] = 1.0;     M[ 2][2] = 0.0;     // y 
      M[ 3][0] = 0.0;     M[ 3][1] = 0.0;     M[ 3][2] = 1.0;     // z 
      M[ 4][0] = y;       M[ 4][1] = x;       M[ 4][2] = 0.0;     // xy 
      M[ 5][0] = z;       M[ 5][1] = 0.0;     M[ 5][2] = x;       // xz 
      M[ 6][0] = 0.0;     M[ 6][1] = z;       M[ 6][2] = y;       // yz 
      M[ 7][0] = 2.0*x;   M[ 7][1] = 0.0;     M[ 7][2] = 0.0;     // xx;
      M[ 8][0] = 0.0;     M[ 8][1] = 2.0*y;   M[ 8][2] = 0.0;     // yy;
      M[ 9][0] = 0.0;     M[ 9][1] = 0.0;     M[ 9][2] = 2.0*z;   // zz;
      M[10][0] = y*z;     M[10][1] = x*z;     M[10][2] = x*y;     // xyz;
      M[11][0] = 3.0*x*x; M[11][1] = 0.0;     M[11][2] = 0.0;     // xxx;
      M[12][0] = y*y;     M[12][1] = 2.0*x*y; M[12][2] = 0.0;     // xyy;
      M[13][0] = z*z;     M[13][1] = 0.0;     M[13][2] = 2.0*x*z; // xzz;
      M[14][0] = 2.0*x*y; M[14][1] = x*x;     M[14][2] = 0.0;     // xxy;
      M[15][0] = 0.0;     M[15][1] = 3.0*y*y; M[15][2] = 0.0;     // yyy;
      M[16][0] = 0.0;     M[16][1] = z*z;     M[16][2] = 2.0*y*z; // yzz;
      M[17][0] = 2.0*x*z; M[17][1] = 0.0;     M[17][2] = x*x;     // xxz;
      M[18][0] = 0.0;     M[18][1] = 2.0*y*z; M[18][2] = y*y;     // yyz;
      M[19][0] = 0.0;     M[19][1] = 0.0;     M[19][2] = 3.0*z*z; // zzz;
      
      return M;
    }
    
    static vw::Matrix<double, 1, 20> quotient_Jacobian( CoeffVec const& c, CoeffVec const& d, CoeffVec const& u ) {

      // Return the Jacobian of dot_prod(c, u) / dot_prod(d, u)
      // as a matrix with 1 row and 20 columns.
      
      double cu  = dot_prod(c, u);
      double du  = dot_prod(d, u);
      double den = du*du;
      
      vw::Matrix<double, 1, 20> R;
      for (int s = 0; s < 20; s++) R[0][s] = (du*c[s] - cu*d[s])/den;
      
      return R;
    }
    
    static vw::Matrix3x3 normalization_Jacobian(vw::Vector3 const& q){
      
      // Return the Jacobian of the function
      // f(x1, x2, x3) = ( (x1 - c1)/q1, (x2 - c2)/q2, (x3 - c3)/q3 )
      
      vw::Matrix3x3 M;
      M[0][0] = 1.0/q[0]; M[0][1] = 0.0;      M[0][2] = 0.0;
      M[1][0] = 0.0;      M[1][1] = 1.0/q[1]; M[1][2] = 0.0;
      M[2][0] = 0.0;      M[2][1] = 0.0;      M[2][2] = 1.0/q[2];
      return M;
    }

    vw::Matrix<double, 2, 3> geodetic_to_pixel_Jacobian( vw::Vector3 const& geodetic ) const;
    
  };
    
  inline std::ostream& operator<<(std::ostream& os, const RPCModel& rpc) {
    os << "RPC Model:" << std::endl
       << "Line Numerator: " << rpc.line_num_coeff() << std::endl
       << "Line Denominator: " << rpc.line_den_coeff() << std::endl
       << "Samp Numerator: " << rpc.sample_num_coeff() << std::endl
       << "Samp Denominator: " << rpc.sample_den_coeff() << std::endl
       << "XY Offset: " << rpc.xy_offset() << std::endl
       << "XY Scale: " << rpc.xy_scale() << std::endl
       << "Geodetic Offset: " << rpc.lonlatheight_offset() << std::endl
       << "Geodetic Scale: " << rpc.lonlatheight_scale();
    return os;
  }
}

#endif//__STEREO_SESSION_RPC_CAMERA_MODEL_H__
