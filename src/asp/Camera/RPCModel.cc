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




#include <vw/Math/Vector.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <asp/Camera/RPCModel.h>

#include <gdal.h>
#include <gdal_priv.h>

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

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

    m_line_num_coeff   = CoeffVec(gdal_rpc.adfLINE_NUM_COEFF);
    m_line_den_coeff   = CoeffVec(gdal_rpc.adfLINE_DEN_COEFF);
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

  // The constructor just copies all of the input data
  RPCModel::RPCModel( cartography::Datum const& datum,
                      Vector<double,20> const& line_num_coeff,
                      Vector<double,20> const& line_den_coeff,
                      Vector<double,20> const& samp_num_coeff,
                      Vector<double,20> const& samp_den_coeff,
                      Vector2           const& xy_offset,
                      Vector2           const& xy_scale,
                      Vector3           const& lonlatheight_offset,
                      Vector3           const& lonlatheight_scale ) :
    m_datum(datum), 
    m_line_num_coeff(line_num_coeff),
    m_line_den_coeff(line_den_coeff), 
    m_sample_num_coeff(samp_num_coeff),
    m_sample_den_coeff(samp_den_coeff), 
    m_xy_offset(xy_offset),
    m_xy_scale(xy_scale), 
    m_lonlatheight_offset(lonlatheight_offset),
    m_lonlatheight_scale(lonlatheight_scale) {}
    

  // All of these implementations are largely inspired by the GDAL
  // code. We don't use the GDAL code unfortunately because they don't
  // make that part of the API available. However I believe this is a
  // safe reinterpretation that is safe to distribute.
  Vector2 RPCModel::point_to_pixel( Vector3 const& point ) const {
    return geodetic_to_pixel( m_datum.cartesian_to_geodetic( point ) );
  }

  Vector2 RPCModel::geodetic_to_pixel( Vector3 const& geodetic ) const {

    // Should we verify that the  input geodetic is in the box?

    Vector3 normalized_geodetic =
      elem_quot(geodetic - m_lonlatheight_offset, m_lonlatheight_scale);

    Vector2 normalized_pixel = normalized_geodetic_to_normalized_pixel( normalized_geodetic );

    return elem_prod( normalized_pixel, m_xy_scale ) + m_xy_offset;
  }

  Vector2 RPCModel::normalized_geodetic_to_normalized_pixel
  (Vector3 const& normalized_geodetic,
   RPCModel::CoeffVec const& line_num_coeff,
   RPCModel::CoeffVec const& line_den_coeff,
   RPCModel::CoeffVec const& sample_num_coeff,
   RPCModel::CoeffVec const& sample_den_coeff){

    CoeffVec term = calculate_terms( normalized_geodetic );
    Vector2 normalized_pixel( dot_prod(term,sample_num_coeff) /
                              dot_prod(term,sample_den_coeff),
                              dot_prod(term,line_num_coeff) /
                              dot_prod(term,line_den_coeff) );

    return normalized_pixel;
  }

  Vector2 RPCModel::normalized_geodetic_to_normalized_pixel
  (Vector3 const& normalized_geodetic ) const {

    return normalized_geodetic_to_normalized_pixel(normalized_geodetic,
                                                   m_line_num_coeff,
                                                   m_line_den_coeff,
                                                   m_sample_num_coeff,
                                                   m_sample_den_coeff
                                                   );
  }

  RPCModel::CoeffVec RPCModel::calculate_terms( vw::Vector3 const& normalized_geodetic ) {

    double x = normalized_geodetic.x(); // normalized lon
    double y = normalized_geodetic.y(); // normalized lat
    double z = normalized_geodetic.z(); // normalized height
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

  vw::Vector<int,20> RPCModel::get_coeff_order() {
    vw::Vector<int,20> result;
    for (int i= 0; i< 3; ++i) result[i] = 1;
    for (int i= 3; i<10; ++i) result[i] = 2;
    for (int i=10; i<20; ++i) result[i] = 3;
    return result;
  }

  vw::Matrix<double, 20, 2> RPCModel::terms_Jacobian2( vw::Vector3 const& normalized_geodetic ) {
    // Partial derivatives of the terms returned by the
    // calculate_terms() function in respect to the first two
    // variables only (unlike the terms_Jacobian3() function).

    vw::Matrix<double, 20, 2> M;
    double x = normalized_geodetic.x(); // normalized lon
    double y = normalized_geodetic.y(); // normalized lat
    double z = normalized_geodetic.z(); // normalized height

    // df/dx            df/dy               // f
    M[ 0][0] = 0.0;     M[ 0][1] = 0.0;     // 1
    M[ 1][0] = 1.0;     M[ 1][1] = 0.0;     // x
    M[ 2][0] = 0.0;     M[ 2][1] = 1.0;     // y
    M[ 3][0] = 0.0;     M[ 3][1] = 0.0;     // z
    M[ 4][0] = y;       M[ 4][1] = x;       // xy
    M[ 5][0] = z;       M[ 5][1] = 0.0;     // xz
    M[ 6][0] = 0.0;     M[ 6][1] = z;       // yz
    M[ 7][0] = 2.0*x;   M[ 7][1] = 0.0;     // xx
    M[ 8][0] = 0.0;     M[ 8][1] = 2.0*y;   // yy
    M[ 9][0] = 0.0;     M[ 9][1] = 0.0;     // zz
    M[10][0] = y*z;     M[10][1] = x*z;     // xyz
    M[11][0] = 3.0*x*x; M[11][1] = 0.0;     // xxx
    M[12][0] = y*y;     M[12][1] = 2.0*x*y; // xyy
    M[13][0] = z*z;     M[13][1] = 0.0;     // xzz
    M[14][0] = 2.0*x*y; M[14][1] = x*x;     // xxy
    M[15][0] = 0.0;     M[15][1] = 3.0*y*y; // yyy
    M[16][0] = 0.0;     M[16][1] = z*z;     // yzz
    M[17][0] = 2.0*x*z; M[17][1] = 0.0;     // xxz
    M[18][0] = 0.0;     M[18][1] = 2.0*y*z; // yyz
    M[19][0] = 0.0;     M[19][1] = 0.0;     // zzz

    return M;
  }

  vw::Matrix<double, 20, 3> RPCModel::terms_Jacobian3( vw::Vector3 const& normalized_geodetic ) {
    // Partial derivatives of the terms returned by the
    // calculate_terms() function in respect to all three
    // variables only (unlike the terms_Jacobian2() function).

    vw::Matrix<double, 20, 3> M;
    double x = normalized_geodetic.x(); // normalized lon
    double y = normalized_geodetic.y(); // normalized lat
    double z = normalized_geodetic.z(); // normalized height

    // df/dx            df/dy               df/dz               // f
    M[ 0][0] = 0.0;     M[ 0][1] = 0.0;     M[ 0][2] = 0.0;     // 1
    M[ 1][0] = 1.0;     M[ 1][1] = 0.0;     M[ 1][2] = 0.0;     // x
    M[ 2][0] = 0.0;     M[ 2][1] = 1.0;     M[ 2][2] = 0.0;     // y
    M[ 3][0] = 0.0;     M[ 3][1] = 0.0;     M[ 3][2] = 1.0;     // z
    M[ 4][0] = y;       M[ 4][1] = x;       M[ 4][2] = 0.0;     // xy
    M[ 5][0] = z;       M[ 5][1] = 0.0;     M[ 5][2] = x;       // xz
    M[ 6][0] = 0.0;     M[ 6][1] = z;       M[ 6][2] = y;       // yz
    M[ 7][0] = 2.0*x;   M[ 7][1] = 0.0;     M[ 7][2] = 0.0;     // xx
    M[ 8][0] = 0.0;     M[ 8][1] = 2.0*y;   M[ 8][2] = 0.0;     // yy
    M[ 9][0] = 0.0;     M[ 9][1] = 0.0;     M[ 9][2] = 2.0*z;   // zz
    M[10][0] = y*z;     M[10][1] = x*z;     M[10][2] = x*y;     // xyz
    M[11][0] = 3.0*x*x; M[11][1] = 0.0;     M[11][2] = 0.0;     // xxx
    M[12][0] = y*y;     M[12][1] = 2.0*x*y; M[12][2] = 0.0;     // xyy
    M[13][0] = z*z;     M[13][1] = 0.0;     M[13][2] = 2.0*x*z; // xzz
    M[14][0] = 2.0*x*y; M[14][1] = x*x;     M[14][2] = 0.0;     // xxy
    M[15][0] = 0.0;     M[15][1] = 3.0*y*y; M[15][2] = 0.0;     // yyy
    M[16][0] = 0.0;     M[16][1] = z*z;     M[16][2] = 2.0*y*z; // yzz
    M[17][0] = 2.0*x*z; M[17][1] = 0.0;     M[17][2] = x*x;     // xxz
    M[18][0] = 0.0;     M[18][1] = 2.0*y*z; M[18][2] = y*y;     // yyz
    M[19][0] = 0.0;     M[19][1] = 0.0;     M[19][2] = 3.0*z*z; // zzz

    return M;
  }

  RPCModel::CoeffVec
  RPCModel::quotient_Jacobian( RPCModel::CoeffVec const& c,
                               RPCModel::CoeffVec const& d,
                               RPCModel::CoeffVec const& u ) {

    // Return the Jacobian of dot_prod(c, u) / dot_prod(d, u)
    // as a vector with 20 elements.

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

  Matrix<double, 2, 3> RPCModel::geodetic_to_pixel_Jacobian( Vector3 const& geodetic ) const {

    Vector3 normalized_geodetic = elem_quot(geodetic - m_lonlatheight_offset, m_lonlatheight_scale);

    CoeffVec term = calculate_terms( normalized_geodetic );

    CoeffVec Qs = quotient_Jacobian(sample_num_coeff(), sample_den_coeff(), term );
    CoeffVec Ql = quotient_Jacobian(line_num_coeff(),   line_den_coeff(),   term );
    Matrix<double, 20, 3> MN = terms_Jacobian3( normalized_geodetic ) *
                                normalization_Jacobian( m_lonlatheight_scale );

    Matrix<double, 2, 3> J;
    select_row( J, 0 ) = m_xy_scale[0] * transpose( Qs ) * MN;
    select_row( J, 1 ) = m_xy_scale[1] * transpose( Ql ) * MN;

    return J;
  }

  Matrix<double, 2, 2> RPCModel::normalized_geodetic_to_pixel_Jacobian( Vector3 const& normalized_geodetic ) const {

    // This function is different from geodetic_to_pixel_Jacobian() in several respects:

    // 1. The input is the normalized geodetic, and the derivatives
    //    are in respect to the normalized geodetic as well.

    // 2. The derivatives are taken only in respect to the first two
    //    variables (normalized lon and lat, no height).

    // 3. The output is in normalized pixels (see m_xy_scale and m_xy_offset).

    CoeffVec term = calculate_terms( normalized_geodetic );

    CoeffVec Qs = quotient_Jacobian(sample_num_coeff(), sample_den_coeff(), term );
    CoeffVec Ql = quotient_Jacobian(line_num_coeff(),   line_den_coeff(),   term );

    Matrix<double, 20, 2> Jt = terms_Jacobian2( normalized_geodetic );

    Matrix<double, 2, 2> J;
    select_row( J, 0 ) = transpose( Qs ) * Jt;
    select_row( J, 1 ) = transpose( Ql ) * Jt;

    return J;
  }

  Matrix<double, 2, 3> RPCModel::geodetic_to_pixel_numerical_Jacobian( Vector3 const& geodetic, double tol ) const {

    // Find the Jacobian of geodetic_to_pixel using numerical
    // differentiation. This is used for testing purposes.

    Matrix<double, 2, 3> J;

    Vector2 B  = geodetic_to_pixel(geodetic);

    Vector2 B0 = (geodetic_to_pixel(geodetic + Vector3(tol, 0,   0  )) - B)/tol;
    Vector2 B1 = (geodetic_to_pixel(geodetic + Vector3(0,   tol, 0  )) - B)/tol;
    Vector2 B2 = (geodetic_to_pixel(geodetic + Vector3(0,   0,   tol)) - B)/tol;

    select_col(J, 0) = B0;
    select_col(J, 1) = B1;
    select_col(J, 2) = B2;

    return J;
  }

  Vector2 RPCModel::image_to_ground( Vector2 const& pixel, double height, Vector2 lonlat_guess ) const {

    // The absolute tolerance is experimental, needs more investigation
    double abs_tolerance = 1e-6;

    Vector2 normalized_pixel = elem_quot(pixel - m_xy_offset, m_xy_scale);

    // Initial guess for the normalized lon and lat
    if (lonlat_guess == Vector2(0.0, 0.0)){
      lonlat_guess = subvector(m_lonlatheight_offset, 0, 2);
    }
    Vector2 normalized_lonlat = elem_quot(lonlat_guess - subvector(m_lonlatheight_offset, 0, 2),
                                          subvector(m_lonlatheight_scale, 0, 2)
                                          );
    double len = norm_2(normalized_lonlat);
    if (len != len || len > 1.5){
      // If the input guess is NaN or unreasonable, use 0 as initial guess
      normalized_lonlat = Vector2(0.0, 0.0);
    }

    // 10 iterations should be enough for Newton's method to converge
    for (int iter = 0; iter < 10; iter++){

      Vector3 normalized_geodetic;
      normalized_geodetic[0] = normalized_lonlat[0];
      normalized_geodetic[1] = normalized_lonlat[1];
      normalized_geodetic[2] = (height - m_lonlatheight_offset[2])/m_lonlatheight_scale[2];

      Vector2              p = normalized_geodetic_to_normalized_pixel(normalized_geodetic);
      Matrix<double, 2, 2> J = normalized_geodetic_to_pixel_Jacobian(normalized_geodetic);

      // The inverse matrix computed analytically
      double det = J[0][0]*J[1][1] - J[0][1]*J[1][0];
      Matrix<double, 2, 2> invJ;
      invJ[0][0] =  J[1][1];
      invJ[0][1] = -J[0][1];
      invJ[1][0] = -J[1][0];
      invJ[1][1] =  J[0][0];
      invJ /= det;

      // Newton's method for F(x) = y is
      // x = x - J^{-1}( F(x) - y )
      Vector2 error_try = p - normalized_pixel;
      normalized_lonlat -= invJ*error_try;

      // Absolute error convergence criterion
      double  norm_try = norm_2(error_try);
      if (norm_try < abs_tolerance) {
        break;
      }

    }

    Vector2 lonlat = elem_prod( normalized_lonlat, subvector(m_lonlatheight_scale, 0, 2) )
                    + subvector(m_lonlatheight_offset, 0, 2);

    return lonlat;

  }

  void RPCModel::point_and_dir(Vector2 const& pix, Vector3 & P, Vector3 & dir ) const {

    // For an RPC model there is no defined origin so it and the ray need to be computed.

    // Center of valid region to bottom of valid region (normalized)
    const double VERT_SCALE_FACTOR = 0.9; // - The virtual center should be above the terrain
    double  height_up = m_lonlatheight_offset[2] + m_lonlatheight_scale[2]*VERT_SCALE_FACTOR;
    double  height_dn = m_lonlatheight_offset[2] - m_lonlatheight_scale[2]*VERT_SCALE_FACTOR;

    //vw_out() << "m_lonlatheight_offset = " << m_lonlatheight_offset << std::endl;
    //vw_out() << "m_lonlatheight_scale = " << m_lonlatheight_scale << std::endl;

    //vw_out() << "Height up = " << height_up << std::endl;
    //vw_out() << "Height dn = " << height_dn << std::endl;

    // Given the pixel and elevation, estimate lon-lat.
    // Use m_lonlatheight_offset as initial guess for lonlat_up,
    // and then use lonlat_up as initial guess for lonlat_dn.
    Vector2 lonlat_up = image_to_ground(pix, height_up, subvector(m_lonlatheight_offset, 0, 2));
    Vector2 lonlat_dn = image_to_ground(pix, height_dn, lonlat_up);

    //vw_out() << "lonlat_up = " << lonlat_up << std::endl;
    //vw_out() << "lonlat_dn = " << lonlat_dn << std::endl;

    Vector3 geo_up = Vector3(lonlat_up[0], lonlat_up[1], height_up);
    Vector3 geo_dn = Vector3(lonlat_dn[0], lonlat_dn[1], height_dn);

    //vw_out() << "geo_up = " << geo_up << std::endl;
    //vw_out() << "geo_dn = " << geo_dn << std::endl;

    Vector3 P_up = m_datum.geodetic_to_cartesian( geo_up );
    Vector3 P_dn = m_datum.geodetic_to_cartesian( geo_dn );

    dir = normalize(P_dn - P_up);
    
    // Set the origin location very far in the opposite direction of the pointing vector,
    //  to put it high above the terrain.
    const double LONG_SCALE_UP = 10000; // This is a distance in meters approx from the top of the llh valid cube
    P = P_up - dir*LONG_SCALE_UP;
  }

  Vector3 RPCModel::camera_center(Vector2 const& pix ) const{
    // Return an arbitrarily chosen point on the ray back-projected
    // through the camera from the current pixel.
    Vector3 P;
    Vector3 dir;
    point_and_dir(pix, P, dir);
    return P;
  }

  Vector3 RPCModel::pixel_to_vector(Vector2 const& pix ) const {
    // Find the normalized direction of the ray back-projected through
    // the camera from the current pixel.
    Vector3 P;
    Vector3 dir;
    point_and_dir(pix, P, dir);
    return dir;
  }

  std::ostream& operator<<(std::ostream& os, const RPCModel& rpc) {
    os << "RPC Model:"         << std::endl
       << "Line Numerator: "   << rpc.line_num_coeff()      << std::endl
       << "Line Denominator: " << rpc.line_den_coeff()      << std::endl
       << "Samp Numerator: "   << rpc.sample_num_coeff()    << std::endl
       << "Samp Denominator: " << rpc.sample_den_coeff()    << std::endl
       << "XY Offset: "        << rpc.xy_offset()           << std::endl
       << "XY Scale: "         << rpc.xy_scale()            << std::endl
       << "Geodetic Offset: "  << rpc.lonlatheight_offset() << std::endl
       << "Geodetic Scale: "   << rpc.lonlatheight_scale();
    return os;
  }
}
