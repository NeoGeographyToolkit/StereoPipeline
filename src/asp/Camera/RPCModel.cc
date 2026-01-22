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

#include <asp/Camera/RPCModel.h>

#include <vw/Core/StringUtils.h>
#include <vw/Math/Vector.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/FileTypes.h>
#include <vw/Math/NewtonRaphson.h>

#include <gdal.h>
#include <gdal_priv.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

using namespace vw;

namespace asp {

void RPCModel::initialize(DiskImageResourceGDAL* resource) {
  
  // Extract the datum (by means of georeference)
  cartography::GeoReference georef;
  cartography::read_georeference(georef, *resource);
  m_datum = georef.datum();

  // Initialize to 0
  m_err_bias = 0.0;
  m_err_rand = 0.0;

  m_terrain_height = std::numeric_limits<double>::quiet_NaN();
  
  // Extract RPC Info
  boost::shared_ptr<GDALDataset> dataset = resource->get_dataset_ptr();
  if (!dataset)
    vw_throw(NotFoundErr() << "RPCModel: Could not read data. No file has been opened.");

  GDALRPCInfo gdal_rpc;
  if (!GDALExtractRPCInfo(dataset->GetMetadata("RPC"),
                          &gdal_rpc))
    vw_throw(NotFoundErr() << "RPCModel: GDAL resource appears not to have RPC metadata.");

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

  m_err_bias = gdal_rpc.dfERR_BIAS;
  m_err_rand = gdal_rpc.dfERR_RAND;
  
  m_terrain_height = std::numeric_limits<double>::quiet_NaN();
}

RPCModel::RPCModel(std::string const& filename) {

  // Initialize to 0
  m_err_bias = 0.0;
  m_err_rand = 0.0;

  m_terrain_height = std::numeric_limits<double>::quiet_NaN();
  
  std::string ext = vw::get_extension(filename);
  if (ext == ".rpb") {
    load_rpb_file(filename);
    return;
  }

  // Must have this check, otherwise GDAL prints an error.
  if (vw::has_image_extension(filename)) {
    boost::shared_ptr<DiskImageResourceGDAL> s_ptr(new DiskImageResourceGDAL(filename));
    initialize(s_ptr.get());
  } else {
    // Throw an error. It will be caught, but it will get printed
    // only if no other approaches turn out to work later on.
    vw_throw(ArgumentErr() << "Not an image file: " << filename);
  }
  
}

RPCModel::RPCModel(DiskImageResourceGDAL* resource ) {
  initialize(resource);
}

// The constructor just copies all of the input data
RPCModel::RPCModel(cartography::Datum const& datum,
                    Vector<double,20> const& line_num_coeff,
                    Vector<double,20> const& line_den_coeff,
                    Vector<double,20> const& samp_num_coeff,
                    Vector<double,20> const& samp_den_coeff,
                    Vector2           const& xy_offset,
                    Vector2           const& xy_scale,
                    Vector3           const& lonlatheight_offset,
                    Vector3           const& lonlatheight_scale,
                    double err_bias, double err_rand,
                    double terrain_height):
  m_datum(datum), 
  m_line_num_coeff(line_num_coeff),
  m_line_den_coeff(line_den_coeff), 
  m_sample_num_coeff(samp_num_coeff),
  m_sample_den_coeff(samp_den_coeff), 
  m_xy_offset(xy_offset),
  m_xy_scale(xy_scale), 
  m_lonlatheight_offset(lonlatheight_offset),
  m_lonlatheight_scale(lonlatheight_scale),
  m_err_bias(err_bias), m_err_rand(err_rand),
  m_terrain_height(terrain_height) {}

void RPCModel::load_rpb_file(std::string const& filename) {
  //vw_out() << "Reading RPC model from RPB file, defaulting to WGS84 datum.\n";
  
  m_datum.set_well_known_datum("WGS84");
  std::ifstream f(filename.c_str());

  // Initialize to 0
  m_err_bias = 0.0;
  m_err_rand = 0.0;
  
  // Initialize to NaN
  m_terrain_height = std::numeric_limits<double>::quiet_NaN();
  
  std::string line;
  std::vector<std::string> tokens;
  bool lineNumCoeffs = false,
    lineDenCoeffs = false,
    sampNumCoeffs = false,
    sampDenCoeffs = false;
  int coeff_index = 0, max_coeff_index = 0;

  // Read through each line in the file    
  while (std::getline(f, line)) {
    
    try {
      // Break up the line
      boost::split(tokens, line, boost::is_any_of("=,;"));
      
      // Parse keywords
      if (line.find("lineOffset") != std::string::npos)
        m_xy_offset[1] = atof(tokens[1].c_str());
      if (line.find("sampOffset") != std::string::npos)
        m_xy_offset[0] = atof(tokens[1].c_str());
      if (line.find("latOffset") != std::string::npos)
        m_lonlatheight_offset[1] = atof(tokens[1].c_str());
      if (line.find("longOffset") != std::string::npos)
        m_lonlatheight_offset[0] = atof(tokens[1].c_str());
      if (line.find("heightOffset") != std::string::npos)
        m_lonlatheight_offset[2] = atof(tokens[1].c_str());
      if (line.find("lineScale") != std::string::npos)
        m_xy_scale[1] = atof(tokens[1].c_str());
      if (line.find("sampScale") != std::string::npos)
        m_xy_scale[0] = atof(tokens[1].c_str());        
      if (line.find("latScale") != std::string::npos)
        m_lonlatheight_scale[1] = atof(tokens[1].c_str());
      if (line.find("longScale") != std::string::npos)
        m_lonlatheight_scale[0] = atof(tokens[1].c_str());
      if (line.find("heightScale") != std::string::npos)
        m_lonlatheight_scale[2] = atof(tokens[1].c_str());
        
      if (line.find("errBias") != std::string::npos)
        m_err_bias = atof(tokens[1].c_str());
      if (line.find("errRand") != std::string::npos)
        m_err_rand = atof(tokens[1].c_str());

      // Start of a coefficient sequence
      if (line.find("lineNumCoef") != std::string::npos) {
        lineNumCoeffs = true;
        continue;
      } else if (line.find("lineDenCoef") != std::string::npos) {
        lineDenCoeffs = true;
        continue;
      } else if (line.find("sampNumCoef") != std::string::npos) {
        sampNumCoeffs = true;
        continue;
      } else if (line.find("sampDenCoef") != std::string::npos) {
        sampDenCoeffs = true;        
        continue;
      }

      // Handle the RPC coefficients
      if (lineNumCoeffs) {
        m_line_num_coeff[coeff_index] = atof(tokens[0].c_str());
        coeff_index++;
      }
      if (lineDenCoeffs) {
        m_line_den_coeff[coeff_index] = atof(tokens[0].c_str());
        coeff_index++;
      }
      if (sampNumCoeffs) {
        m_sample_num_coeff[coeff_index] = atof(tokens[0].c_str());
        coeff_index++;
      }
      if (sampDenCoeffs) {
        m_sample_den_coeff[coeff_index] = atof(tokens[0].c_str());
        coeff_index++;
      }

      // Done reading a coefficient sequence
      if (line.find(")") != std::string::npos) {
        lineNumCoeffs = false;
        lineDenCoeffs = false;
        sampNumCoeffs = false;
        sampDenCoeffs = false;
        
        if (coeff_index > max_coeff_index)
          max_coeff_index = coeff_index;
          
        coeff_index = 0;
      }

    } catch(...) {
      vw_throw(ArgumentErr() << "Error reading file " << filename
                << ", line = "  << line);
    }
  } // End loop through lines.
  f.close();
  
  // Basic error check
  if (max_coeff_index != 20)
    vw_throw(ArgumentErr() << "Error reading file " << filename
              << ", loaded wrong number of coefficients.");
}

// All of these implementations are largely inspired by the GDAL
// code. We don't use the GDAL code unfortunately because they don't
// make that part of the API available. However I believe this is a
// safe reinterpretation that is safe to distribute.
Vector2 RPCModel::point_to_pixel(Vector3 const& point) const {
  return geodetic_to_pixel(m_datum.cartesian_to_geodetic(point));
}

Vector2 RPCModel::geodetic_to_pixel(Vector3 const& geodetic) const {

  // Should we verify that the  input geodetic is in the box?

  Vector3 normalized_geodetic =
    elem_quot(geodetic - m_lonlatheight_offset, m_lonlatheight_scale);

  Vector2 normalized_pixel = normalizedLlhToPix(normalized_geodetic);

  return elem_prod(normalized_pixel, m_xy_scale) + m_xy_offset;
}

Vector2 RPCModel::normalizedLlhToPix(Vector3 const& normalized_geodetic,
                                     RPCModel::CoeffVec const& line_num_coeff,
                                     RPCModel::CoeffVec const& line_den_coeff,
                                     RPCModel::CoeffVec const& sample_num_coeff,
                                     RPCModel::CoeffVec const& sample_den_coeff){

  CoeffVec term = calculate_terms(normalized_geodetic);
  Vector2 normalized_pixel(dot_prod(term,sample_num_coeff) /
                            dot_prod(term,sample_den_coeff),
                            dot_prod(term,line_num_coeff) /
                            dot_prod(term,line_den_coeff));

  return normalized_pixel;
}

Vector2 RPCModel::normalizedLlhToPix(Vector3 const& normalized_geodetic) const {

  return normalizedLlhToPix(normalized_geodetic,
                            m_line_num_coeff, m_line_den_coeff,
                            m_sample_num_coeff, m_sample_den_coeff);
}

RPCModel::CoeffVec RPCModel::calculate_terms(vw::Vector3 const& normalized_geodetic) {

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
  for (int i = 0; i < 3; i++) result[i] = 1;
  for (int i = 3; i < 10; i++) result[i] = 2;
  for (int i = 10; i < 20; i++) result[i] = 3;
  return result;
}

vw::Matrix<double, 20, 2> RPCModel::terms_Jacobian2(vw::Vector3 const& normalized_geodetic) {
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

vw::Matrix<double, 20, 3> RPCModel::terms_Jacobian3(vw::Vector3 const& normalized_geodetic) {
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
RPCModel::quotient_Jacobian(RPCModel::CoeffVec const& c,
                            RPCModel::CoeffVec const& d,
                            RPCModel::CoeffVec const& u) {

  // Return the Jacobian of dot_prod(c, u) / dot_prod(d, u)
  // as a vector with 20 elements.

  double cu  = dot_prod(c, u);
  double du  = dot_prod(d, u);
  double den = du*du;

  return elem_quot(du * c - cu * d, den);
}

// Return the Jacobian of the function
// f(x1, x2, x3) = ((x1 - c1)/q1, (x2 - c2)/q2, (x3 - c3)/q3)
vw::Matrix3x3 RPCModel::normalization_Jacobian(Vector3 const& q) {

  vw::Matrix3x3 M;
  M[0][0] = 1.0/q[0]; M[0][1] = 0.0;      M[0][2] = 0.0;
  M[1][0] = 0.0;      M[1][1] = 1.0/q[1]; M[1][2] = 0.0;
  M[2][0] = 0.0;      M[2][1] = 0.0;      M[2][2] = 1.0/q[2];
  return M;
}

Matrix<double, 2, 3> 
RPCModel::geodetic_to_pixel_Jacobian(Vector3 const& geodetic) const {

  Vector3 normalized_geodetic = elem_quot(geodetic - m_lonlatheight_offset, 
                                          m_lonlatheight_scale);

  CoeffVec term = calculate_terms(normalized_geodetic);

  CoeffVec Qs = quotient_Jacobian(sample_num_coeff(), sample_den_coeff(), term);
  CoeffVec Ql = quotient_Jacobian(line_num_coeff(),   line_den_coeff(),   term);
  Matrix<double, 20, 3> MN = terms_Jacobian3(normalized_geodetic) *
    normalization_Jacobian(m_lonlatheight_scale);

  Matrix<double, 2, 3> J;
  select_row(J, 0) = m_xy_scale[0] * transpose(Qs) * MN;
  select_row(J, 1) = m_xy_scale[1] * transpose(Ql) * MN;

  return J;
}

// This function is different from geodetic_to_pixel_Jacobian() in several
// respects:
// 1. The input is the normalized geodetic, and the derivatives
//    are in respect to the normalized geodetic as well.
// 2. The derivatives are taken only in respect to the first two
//    variables (normalized lon and lat, no height).
// 3. The output is in normalized pixels (see m_xy_scale and m_xy_offset).
vw::Vector<double> 
RPCModel::normalizedLlhToPixJac(Vector3 const& normalized_geodetic) const {

    CoeffVec term = calculate_terms(normalized_geodetic);

    CoeffVec Qs = quotient_Jacobian(sample_num_coeff(), sample_den_coeff(), term);
    CoeffVec Ql = quotient_Jacobian(line_num_coeff(),   line_den_coeff(),   term);

    Matrix<double, 20, 2> Jt = terms_Jacobian2(normalized_geodetic);
    
    Vector<double> J(4);
    for (int i = 0; i < 4; i++) J[i] = 0;
    
    // Compute J = [Qs^T, Ql^T] * Jt
    for (int i = 0; i < 20; i++) {
      J[0] += Qs[i] * Jt[i][0];
      J[1] += Qs[i] * Jt[i][1];
      J[2] += Ql[i] * Jt[i][0];
      J[3] += Ql[i] * Jt[i][1];
    }
    
    return J;
  }

// Find the Jacobian of geodetic_to_pixel using numerical
// differentiation. This is used for testing purposes.
Matrix<double, 2, 3> 
RPCModel::geodetic_to_pixel_numerical_Jacobian(Vector3 const& geodetic, 
                                               double tol) const {

  Matrix<double, 2, 3> J;

  Vector2 B  = geodetic_to_pixel(geodetic);

  Vector2 B0 = (geodetic_to_pixel(geodetic + Vector3(tol, 0,   0 )) - B)/tol;
  Vector2 B1 = (geodetic_to_pixel(geodetic + Vector3(0,   tol, 0 )) - B)/tol;
  Vector2 B2 = (geodetic_to_pixel(geodetic + Vector3(0,   0,   tol)) - B)/tol;

  select_col(J, 0) = B0;
  select_col(J, 1) = B1;
  select_col(J, 2) = B2;

  return J;
}

// A class that computes the normalized pixel and Jacobian via two versions of
// operator(), in a way suitable to pass to the NewtonRaphson solver.
struct RpcFunJac {

  RpcFunJac(RPCModel const& rpc, double height): m_rpc(rpc) {

     m_normalized_height 
      = (height - m_rpc.m_lonlatheight_offset[2]) / m_rpc.m_lonlatheight_scale[2];
  }

  // The input is the normalized lon-lat. The output is the normalized pixel.
  vw::Vector2 operator()(vw::Vector2 const& normalized_lonlat) {
    return m_rpc.normalizedLlhToPix(Vector3(normalized_lonlat[0], 
                                            normalized_lonlat[1], 
                                            m_normalized_height));
  }
  
  // The input is the normalized lon-lat. The output is the Jacobian
  // of the normalized pixel in respect to the normalized lon-lat.
  vw::Vector<double> operator()(vw::Vector2 const& normalized_lonlat, double step) {
  
    return m_rpc.normalizedLlhToPixJac(Vector3(normalized_lonlat[0], 
                                               normalized_lonlat[1], 
                                               m_normalized_height));
  }
  
  RPCModel const& m_rpc;
  double m_normalized_height;
};

// Intersect the ray from the given pixel with surface at this
// height above the datum.  Use Newton's method. The obtained
// intersection point must project back into the pixel.
// TODO(oalexan1): Replace the solver here with the logic in
// vw/Math/NewtonRaphson.h.
Vector2 RPCModel::image_to_ground(Vector2 const& pixel, double height,
                                  Vector2 lonlat_guess) const {

  // Ensure the tolerance is rather tight, as later will trace rays
  // through pairs of such ground points for triangulation.
  double tol = 1e-10;

  Vector2 normalized_pixel = elem_quot(pixel - m_xy_offset, m_xy_scale);
  
  // The initial guess for the normalized lon-lat. 
  vw::Vector2 ll_off   = subvector(m_lonlatheight_offset, 0, 2);  
  vw::Vector2 ll_scale = subvector(m_lonlatheight_scale, 0, 2);
  if (lonlat_guess == Vector2(0.0, 0.0))
    lonlat_guess = ll_off; // initial guess
  vw::Vector2 normalized_lonlat = elem_quot(lonlat_guess - ll_off, ll_scale);

  // Set up Newton-Raphson. The RpcFunJac class computes both the function
  // and its Jacobian via two versions of operator().
  RpcFunJac rpc_fun_jac(*this, height);
  vw::math::NewtonRaphson nr(rpc_fun_jac, rpc_fun_jac);
  vw::Vector2 guess = normalized_lonlat;
  double step = 1e-6; // Not used with analytic Jacobian, but part of the API.
  
  // Find with normalized_lonlat such that
  // normalized_pixel = rpc_fun_jac(normalized_lonlat).
  normalized_lonlat = nr.solve(guess, normalized_pixel, step, tol);

  // Undo the normalization
  Vector2 lonlat = elem_prod(normalized_lonlat, ll_scale) + ll_off;
  
  return lonlat;
}

void RPCModel::point_and_dir(Vector2 const& pix, Vector3 & P, Vector3 & dir) const {

  // For an RPC model there is no defined origin so it and the ray need to be computed.
  // Try to have the ray end points not too far from the center of the valid region.
  // This can make a difference if the region is tall and the rays are curved.
  const double VERT_SCALE_FACTOR = 0.9; // - The virtual center should be above the terrain
  double delta = m_lonlatheight_scale[2]*VERT_SCALE_FACTOR; // measured in meters
  delta = std::min(delta, 50.0);
  delta = std::max(delta, 0.1);

  // Center of valid region to bottom of valid region (normalized)
  double  height_up = m_lonlatheight_offset[2] + delta;
  double  height_dn = m_lonlatheight_offset[2] - delta;

  // Given the pixel and elevation, estimate lon-lat.
  // Use m_lonlatheight_offset as initial guess for lonlat_up,
  // and then use lonlat_up as initial guess for lonlat_dn.
  vw::Vector2 ll_off = subvector(m_lonlatheight_offset, 0, 2);
  Vector2 lonlat_up = image_to_ground(pix, height_up, ll_off);
  Vector2 lonlat_dn = image_to_ground(pix, height_dn, lonlat_up);

  Vector3 geo_up = Vector3(lonlat_up[0], lonlat_up[1], height_up);
  Vector3 geo_dn = Vector3(lonlat_dn[0], lonlat_dn[1], height_dn);

  Vector3 P_up = m_datum.geodetic_to_cartesian(geo_up);
  Vector3 P_dn = m_datum.geodetic_to_cartesian(geo_dn);

  dir = normalize(P_dn - P_up);
  
  // Set the origin location very far in the opposite direction of the pointing vector,
  //  to put it high above the terrain. Normally the precise position along the ray
  // should not make any difference, except perhaps in error propagation
  // (see Covariance.cc).
  // TODO(oalexan1): Use the logic from cam_gen.cc to shoot rays from the ground
  // up and estimate where they intersect. That will give a better idea of the true
  // elevation of the camera above the ground.
  const double LONG_SCALE_UP = 100000.0; // 100 km above ground
  P = P_up - dir*LONG_SCALE_UP;
}

Vector3 RPCModel::camera_center(Vector2 const& pix) const{
  // Return an arbitrarily chosen point on the ray back-projected
  // through the camera from the current pixel.
  Vector3 P;
  Vector3 dir;
  point_and_dir(pix, P, dir);
  return P;
}

Vector3 RPCModel::pixel_to_vector(Vector2 const& pix) const {
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

// Save the RPC model to an XML file. This is similar to the WorldView format.
// GDAL can read it.
void RPCModel::saveXML(std::string const& filename) const {
  
  std::string lineoffset   = vw::num_to_str(xy_offset().y());
  std::string sampoffset   = vw::num_to_str(xy_offset().x());
  std::string latoffset    = vw::num_to_str(lonlatheight_offset().y());
  std::string longoffset   = vw::num_to_str(lonlatheight_offset().x());
  std::string heightoffset = vw::num_to_str(lonlatheight_offset().z());

  std::string linescale   = vw::num_to_str(xy_scale().y());
  std::string sampscale   = vw::num_to_str(xy_scale().x());
  std::string latscale    = vw::num_to_str(lonlatheight_scale().y());
  std::string longscale   = vw::num_to_str(lonlatheight_scale().x());
  std::string heightscale = vw::num_to_str(lonlatheight_scale().z());

  std::string linenumcoef = vw::vec_to_str(line_num_coeff());
  std::string linedencoef = vw::vec_to_str(line_den_coeff());
  std::string sampnumcoef = vw::vec_to_str(sample_num_coeff());
  std::string sampdencoef = vw::vec_to_str(sample_den_coeff());

  std::string datum_wkt = m_datum.get_wkt();
  
  std::ofstream ofs(filename.c_str());
  ofs.precision(17);

  ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n";
  ofs << "<isd>\n";
  ofs << "    <RPB>\n";
  ofs << "        <IMAGE>\n";
  ofs << "            <RPC_DATUM>"       << datum_wkt << "</RPC_DATUM>\n";
  ofs << "            <ERRBIAS>"         << m_err_bias << "</ERRBIAS>\n";
  ofs << "            <ERRRAND>"         << m_err_rand << "</ERRRAND>\n";
  ofs << "            <LINEOFFSET>"      << lineoffset   << "</LINEOFFSET>\n";
  ofs << "            <SAMPOFFSET>"      << sampoffset   << "</SAMPOFFSET>\n";
  ofs << "            <LATOFFSET>"       << latoffset    << "</LATOFFSET>\n";
  ofs << "            <LONGOFFSET>"      << longoffset   << "</LONGOFFSET>\n";
  ofs << "            <HEIGHTOFFSET>"    << heightoffset << "</HEIGHTOFFSET>\n";
  ofs << "            <LINESCALE>"       << linescale    << "</LINESCALE>\n";
  ofs << "            <SAMPSCALE>"       << sampscale    << "</SAMPSCALE>\n";
  ofs << "            <LATSCALE>"        << latscale     << "</LATSCALE>\n";
  ofs << "            <LONGSCALE>"       << longscale    << "</LONGSCALE>\n";
  ofs << "            <HEIGHTSCALE>"     << heightscale  << "</HEIGHTSCALE>\n";
  ofs << "            <LINENUMCOEFList>\n";
  ofs << "                <LINENUMCOEF>" << linenumcoef  << "</LINENUMCOEF>\n";
  ofs << "            </LINENUMCOEFList>\n";
  ofs << "            <LINEDENCOEFList>\n";
  ofs << "                <LINEDENCOEF>" << linedencoef  << "</LINEDENCOEF>\n";
  ofs << "            </LINEDENCOEFList>\n";
  ofs << "            <SAMPNUMCOEFList>\n";
  ofs << "                <SAMPNUMCOEF>" << sampnumcoef  << "</SAMPNUMCOEF>\n";
  ofs << "            </SAMPNUMCOEFList>\n";
  ofs << "            <SAMPDENCOEFList>\n";
  ofs << "                <SAMPDENCOEF>" << sampdencoef  << "</SAMPDENCOEF>\n";
  ofs << "            </SAMPDENCOEFList>\n";
  ofs << "        </IMAGE>\n";
  ofs << "    </RPB>\n";
  ofs << "</isd>\n";
  ofs.close();
}

} // end namespace asp
