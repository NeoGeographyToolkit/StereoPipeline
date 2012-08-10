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


#include <asp/Sessions/RPC/RPCModel.h>

#include <vw/Math/Vector.h>
#include <vw/FileIO/GdalIO.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Cartography.h>

using namespace vw;

namespace asp {

  void RPCModel::initialize( DiskImageResourceGDAL* resource ) {
    // Extract Datum (by means of GeoReference)
    cartography::GeoReference georef;
    cartography::read_georeference( georef, *resource );
    m_datum = georef.datum();

    // Extract RPC Info
    boost::shared_ptr<GDALDataset> dataset = resource->get_dataset_ptr();
    if ( !dataset )
      vw_throw( NotFoundErr() << "RPCModel: Could not read data. No file has been opened." );

    GDALRPCInfo gdal_rpc;
    if ( !GDALExtractRPCInfo( dataset->GetMetadata("RPC"),
                              &gdal_rpc) )
      vw_throw( NotFoundErr() << "RPCModel: GDAL resource appears not to have RPC metadata." );

    // Copy information over to our data structures.
    m_lonlatheight_offset = Vector3(gdal_rpc.dfLONG_OFF,
                                    gdal_rpc.dfLAT_OFF,
                                    gdal_rpc.dfHEIGHT_OFF);
    m_lonlatheight_scale = Vector3(gdal_rpc.dfLONG_SCALE,
                                   gdal_rpc.dfLAT_SCALE,
                                   gdal_rpc.dfHEIGHT_SCALE);
    m_xy_offset = Vector2(gdal_rpc.dfSAMP_OFF,   gdal_rpc.dfLINE_OFF);
    m_xy_scale  = Vector2(gdal_rpc.dfSAMP_SCALE, gdal_rpc.dfLINE_SCALE);

    m_line_num_coeff =   CoeffVec(gdal_rpc.adfLINE_NUM_COEFF);
    m_line_den_coeff =   CoeffVec(gdal_rpc.adfLINE_DEN_COEFF);
    m_sample_num_coeff = CoeffVec(gdal_rpc.adfSAMP_NUM_COEFF);
    m_sample_den_coeff = CoeffVec(gdal_rpc.adfSAMP_DEN_COEFF);
  }

  RPCModel::RPCModel( std::string const& filename ) {
    boost::scoped_ptr<DiskImageResourceGDAL> s_ptr( new DiskImageResourceGDAL(filename) );
    initialize( s_ptr.get() );
  }

  RPCModel::RPCModel( DiskImageResourceGDAL* resource  ) {
    initialize( resource );
  }

  RPCModel::RPCModel( cartography::Datum const& datum,
                      Vector<double,20> const& line_num_coeff,
                      Vector<double,20> const& line_den_coeff,
                      Vector<double,20> const& samp_num_coeff,
                      Vector<double,20> const& samp_den_coeff,
                      Vector2 const& xy_offset,
                      Vector2 const& xy_scale,
                      Vector3 const& lonlatheight_offset,
                      Vector3 const& lonlatheight_scale ) :
    m_datum(datum), m_line_num_coeff(line_num_coeff),
    m_line_den_coeff(line_den_coeff), m_sample_num_coeff(samp_num_coeff),
    m_sample_den_coeff(samp_den_coeff), m_xy_offset(xy_offset),
    m_xy_scale(xy_scale), m_lonlatheight_offset(lonlatheight_offset),
    m_lonlatheight_scale(lonlatheight_scale) {}

  // All of these implementations are largely inspired by the GDAL
  // code. We don't use the GDAL code unfortunately because they don't
  // make that part of the API available. However I believe this is a
  // safe reinterpretation that is safe to distribute.
  Vector2 RPCModel::point_to_pixel( Vector3 const& point ) const {
    return geodetic_to_pixel( m_datum.cartesian_to_geodetic( point ) );
  }

  vw::Vector<double,20> RPCModel::calculate_terms( vw::Vector3 const& v ) {
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

  vw::Matrix<double, 20, 3> RPCModel::terms_Jacobian( vw::Vector3 const& v ) {
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

  RPCModel::CoeffVec
  RPCModel::quotient_Jacobian( RPCModel::CoeffVec const& c,
                               RPCModel::CoeffVec const& d,
                               RPCModel::CoeffVec const& u ) {

    // Return the Jacobian of dot_prod(c, u) / dot_prod(d, u)
    // as a matrix with 1 row and 20 columns.

    double cu  = dot_prod(c, u);
    double du  = dot_prod(d, u);
    double den = du*du;

    return elem_quot(du * c - cu * d, den);
  }

  vw::Matrix3x3 RPCModel::normalization_Jacobian( Vector3 const& q ) {

    // Return the Jacobian of the function
    // f(x1, x2, x3) = ( (x1 - c1)/q1, (x2 - c2)/q2, (x3 - c3)/q3 )

    vw::Matrix3x3 M;
    M[0][0] = 1.0/q[0]; M[0][1] = 0.0;      M[0][2] = 0.0;
    M[1][0] = 0.0;      M[1][1] = 1.0/q[1]; M[1][2] = 0.0;
    M[2][0] = 0.0;      M[2][1] = 0.0;      M[2][2] = 1.0/q[2];
    return M;
  }

  Vector2 RPCModel::geodetic_to_pixel( Vector3 const& geodetic ) const {
    CoeffVec term =
      calculate_terms( elem_quot(geodetic - m_lonlatheight_offset,
                                 m_lonlatheight_scale) );

    Vector2 normalized_proj( dot_prod(term,m_sample_num_coeff) /
                             dot_prod(term,m_sample_den_coeff),
                             dot_prod(term,m_line_num_coeff) /
                             dot_prod(term,m_line_den_coeff) );

    return elem_prod( normalized_proj, m_xy_scale ) + m_xy_offset;
  }

  Matrix<double, 2, 3> RPCModel::geodetic_to_pixel_Jacobian( Vector3 const& geodetic ) const {

    Vector3 normalized_geodetic = elem_quot(geodetic - m_lonlatheight_offset, m_lonlatheight_scale);

    CoeffVec term = calculate_terms( normalized_geodetic );

    CoeffVec Qs = quotient_Jacobian(sample_num_coeff(),
                                    sample_den_coeff(), term );
    CoeffVec Ql = quotient_Jacobian(line_num_coeff(),
                                    line_den_coeff(), term );
    Matrix<double, 20, 3> MN =
      terms_Jacobian( normalized_geodetic ) *
      normalization_Jacobian( m_lonlatheight_scale );

    Matrix<double, 2, 3> Jacobian;
    select_row( Jacobian, 0 ) = m_xy_scale[0] * transpose( Qs ) * MN;
    select_row( Jacobian, 1 ) = m_xy_scale[1] * transpose( Ql ) * MN;

    return Jacobian;
  }

#if 1
  
#if 0
  void RPCModel::futureUnitTest(Vector3 const& geodetic ) const {
    std::cout.precision(20);

    Vector2 O = RPCModel::geodetic_to_pixel(geodetic);
      
    std::cout << "now in geodetic_to_pixel." << std::endl;
    std::cout << "input is " << geodetic << std::endl;
    std::cout << "output is: " << O << std::endl;
      
    vw::Matrix<double, 2, 3> M = geodetic_to_pixel_Jacobian(geodetic);
      
    std::cout << "jacobian is: " << std::endl;
    std::cout << M[0][0] << ' ' << M[0][1] << ' ' << M[0][2] << std::endl;
    std::cout << M[1][0] << ' ' << M[1][1] << ' ' << M[1][2] << std::endl;

      
    double eps = 1e-3;
    Vector2 O1 = (RPCModel::geodetic_to_pixel(geodetic + Vector3(eps, 0,   0  )) - O)/eps;
    Vector2 O2 = (RPCModel::geodetic_to_pixel(geodetic + Vector3(0,   eps, 0  )) - O)/eps;
    Vector2 O3 = (RPCModel::geodetic_to_pixel(geodetic + Vector3(0,   0,   eps)) - O)/eps;
      
    std::cout << "numerical jacobian is " << std::endl;
    std::cout << O1[0] << ' ' << O2[0] << ' ' << O3[0] << std::endl;
    std::cout << O1[1] << ' ' << O2[1] << ' ' << O3[1] << std::endl;
      
  }
#endif
  
#if 0
  Vector3 RPCModel::pixel_to_vector(Vector2 const& pix ) const {

    double  height_up = m_lonlatheight_offset[2];
    double  height_dn = m_lonlatheight_offset[2] - m_lonlatheight_scale[2];

    Vector2 lonlat_up = image_to_ground(pix, height_up);
    Vector2 lonlat_dn = image_to_ground(pix, height_dn);

    Vector3 geo_up = Vector3(lonlat_up[0], lonlat_up[1], height_up);
    Vector3 geo_dn = Vector3(lonlat_dn[0], lonlat_dn[1], height_dn);

    Vector2 pix_up = geodetic_to_pixel(geo_up);
    Vector2 pix_dn = geodetic_to_pixel(geo_dn);

    //std::cout << "geo and error is " << geo_up << ' ' << geo_dn << ' ' << norm_2(pix - pix_up) << ' ' << norm_2(pix - pix_dn) << std::endl;
    
    Vector3 P_up = m_datum.geodetic_to_cartesian( geo_up );
    Vector3 P_dn = m_datum.geodetic_to_cartesian( geo_dn );

    return normalize(P_dn - P_up);
  }
#endif
  
  void RPCModel::ctr_and_dir(Vector2 const& pix, Vector3 & ctr, Vector3 & dir ) const {

    double  height_up = m_lonlatheight_offset[2] + m_lonlatheight_scale[2]*0.5;
    double  height_dn = m_lonlatheight_offset[2] - m_lonlatheight_scale[2]*0.5;

    Vector2 lonlat_up = image_to_ground(pix, height_up);
    Vector2 lonlat_dn = image_to_ground(pix, height_dn);

    Vector3 geo_up = Vector3(lonlat_up[0], lonlat_up[1], height_up);
    Vector3 geo_dn = Vector3(lonlat_dn[0], lonlat_dn[1], height_dn);

    Vector2 pix_up = geodetic_to_pixel(geo_up);
    Vector2 pix_dn = geodetic_to_pixel(geo_dn);

    //std::cout << "geo and error is " << geo_up << ' ' << geo_dn << ' ' << norm_2(pix - pix_up) << ' ' << norm_2(pix - pix_dn) << std::endl;
    
    Vector3 P_up = m_datum.geodetic_to_cartesian( geo_up );
    Vector3 P_dn = m_datum.geodetic_to_cartesian( geo_dn );

    ctr = P_up;
    dir = normalize(P_dn - P_up);
  }

  Vector2 RPCModel::image_to_ground( Vector2 const& observedPixel, double height ) const {

    // To do: Make this into a full blown solver!!!
    // To do: Use the existing formula for matrix inversion!!!
    
    // Given a pixel (the projection of a point onto the camera image)
    // and the value of the height of the point, find the longitude
    // and latitude of the point. We use Newton's method.

    // Initial guess for the lon and lat.
    Vector2 lonlat = subvector(m_lonlatheight_offset, 0, 2);

    //     std::cout << "observedPixel is " << observedPixel << std::endl;
    //     std::cout << "lonlat is " << lonlat << std::endl;
    
    for (int s = 0; s < 10; s++){

      Vector3 geodetic;
      geodetic[0] = lonlat[0];
      geodetic[1] = lonlat[1];
      geodetic[2] = height;
       
      Vector2              p    = geodetic_to_pixel(geodetic);
      Matrix<double, 2, 3> Jall = geodetic_to_pixel_Jacobian(geodetic);

      //       std::cout << "geodetic is " << geodetic << std::endl;
      //       std::cout << "pixel is " << p << std::endl;
      //       std::cout << "observed pixel is " << observedPixel << std::endl;
      
      // The Jacobian only in respect to lon and lat.
      Matrix<double, 2, 2> J = submatrix(Jall, 0, 0, 2, 2);

      double det = J[0][0]*J[1][1] - J[0][1]*J[1][0];
      Matrix<double, 2, 2> invJ;
      invJ[0][0] =  J[1][1];
      invJ[0][1] = -J[0][1];
      invJ[1][0] = -J[1][0];
      invJ[1][1] =  J[0][0];
      invJ /= det;

      //std::cout << "vals are " << J << ' ' << invJ << ' ' << J*invJ << std::endl;
      
      // Newton's method
      // lonlat = lonlat - J^{-1}( F(lonlat) - observedPixel )
      Vector2 diff = p - observedPixel;
      Matrix<double, 2, 1> diffM; diffM[0][0] = diff[0]; diffM[1][0] = diff[1];
      Matrix<double, 2, 1> dv = -invJ * diffM;
      lonlat += Vector2(dv[0][0], dv[1][0]);

      //std::cout << "lonlat and diff are: " << lonlat  << ' ' << diff << std::endl;
    }

    return lonlat;
    
  }
#endif
}
