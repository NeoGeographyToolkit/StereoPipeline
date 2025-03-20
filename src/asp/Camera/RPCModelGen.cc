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

#include <asp/Camera/RPCModelGen.h>
#include <asp/Camera/RpcUtils.h>
#include <asp/Camera/RPCModel.h>

#include <vw/Math/Geometry.h>
#include <vw/Math/BBox.h>

using namespace vw;

namespace asp {

// There are 20 polynomial coefficients for each of lineNum,
// lineDen, sampNum, sampDen, as the RPC polynomials are of degree
// 3 in x and y.  However, the 0 degree coeffs for both
// denominators is 1, so only 78 coefficients vary. Extract the
// variable coefficients from a vector.
void unpackCoeffs(Vector<double> const& C,
                  RPCModel::CoeffVec& lineNum, RPCModel::CoeffVec& lineDen,
                  RPCModel::CoeffVec& sampNum, RPCModel::CoeffVec& sampDen) {

  VW_ASSERT(C.size() == 78,
            ArgumentErr() << "Must have 78 coefficients.\n");

  int start = 0;
  // Line
  subvector(lineNum, 0, 20) = subvector(C, start, 20); start += 20;
  lineDen[0] = 1;
  subvector(lineDen, 1, 19) = subvector(C, start, 19); start += 19;
  // Sample
  subvector(sampNum, 0, 20) = subvector(C, start, 20); start += 20;
  sampDen[0] = 1;
  subvector(sampDen, 1, 19) = subvector(C, start, 19); start += 19;

  VW_ASSERT(start == 78, ArgumentErr() << "Book-keeping error.\n");

  return;
}

// This function does the reverse of unpackCoeffs().
void packCoeffs(RPCModel::CoeffVec const& lineNum, RPCModel::CoeffVec const& lineDen,
                RPCModel::CoeffVec const& sampNum, RPCModel::CoeffVec const& sampDen,
                Vector<double> & C) {

  C.set_size(78);
  int start = 0;
  subvector(C, start, 20) = subvector(lineNum, 0, 20); start += 20;
  subvector(C, start, 19) = subvector(lineDen, 1, 19); start += 19;
  subvector(C, start, 20) = subvector(sampNum, 0, 20); start += 20;
  subvector(C, start, 19) = subvector(sampDen, 1, 19); start += 19;

  return;
}

// Given a set of RPC coefficients, compute the projected pixels. The input is
// the RPC coefficients, packed in a vector. For each normalized geodetic,
// compute the normalized pixel value. This will be the output. Add the
// penalization terms to the output. See the note later in the code.
RpcSolveLMA::result_type RpcSolveLMA::operator()(RpcSolveLMA::domain_type const& C) const {

  // Unpack all the RPC model coefficients from the input vector C
  RPCModel::CoeffVec lineNum, lineDen, sampNum, sampDen;
  unpackCoeffs(C, lineNum, lineDen, sampNum, sampDen);

  // Initialize the output vector
  int numPts = m_normalizedGeodetics.size()/RPCModel::GEODETIC_COORD_SIZE;
  result_type result;
  result.set_size(m_normalizedPixels.size());
  
  // Loop through each test point
  for (int i = 0; i < numPts; i++){
    // Unpack the normalized geodetic coordinates
    vw::Vector3 G = subvector(m_normalizedGeodetics, RPCModel::GEODETIC_COORD_SIZE*i, 
                              RPCModel::GEODETIC_COORD_SIZE);
    
    // Project the normalized geodetic coordinate into the RPC camera to get a normalized pixel
    vw::Vector2 pxn = RPCModel::normalizedLlhToPix(G, lineNum, lineDen,
                                                                        sampNum, sampDen);
          
    // Pack the normalized pixel into the output result vector
    subvector(result, RPCModel::IMAGE_COORD_SIZE*i, RPCModel::IMAGE_COORD_SIZE) = pxn;

  }

  // There are 4*20 - 2 = 78 coefficients we optimize. Of those, 2
  // are 0-th degree, 4*3 = 12 are 1st degree, and the rest, 78 - 12
  // - 2 = 64 are higher degree.  Per Hartley, we'll add for each
  // such coefficient c, a term K*c in the cost function vector,
  // where K is a large number. This will penalize large values in
  // the higher degree coefficients.
  // - These values are attached to the end of the output vector
  int count = RPCModel::IMAGE_COORD_SIZE*numPts; 
  vw::Vector<int,20> coeff_order = RPCModel::get_coeff_order(); // This ranges from 1 to 3
  for (int i = 4; i < (int)lineNum.size(); i++)
    result[count++] = m_wt*lineNum[i] * (coeff_order[i]-1);
  for (int i = 4; i < (int)lineDen.size(); i++)
    result[count++] = m_wt*lineDen[i] * (coeff_order[i]-1);
  for (int i = 4; i < (int)sampNum.size(); i++)
    result[count++] = m_wt*sampNum[i] * (coeff_order[i]-1);
  for (int i = 4; i < (int)sampDen.size(); i++)
    result[count++] = m_wt*sampDen[i] * (coeff_order[i]-1);

  VW_ASSERT((int)result.size() == count, vw::ArgumentErr() << "Book-keeping error.\n");

  return result;
}
  
// Print out a name followed by the vector of values
void print_vec(std::string const& name, Vector<double> const& vals) {
  std::cout.precision(16);
  std::cout << name << ",";
  int len = vals.size();
  for (int i = 0; i < len - 1; i++)
    std::cout << vals[i] << ",";
  if (len > 0)
    std::cout << vals[len-1];
  std::cout << "\n";
}

// Dump a vector to a text file, one value per row.
void print_vec_to_file(std::string const& path, Vector<double> const& vals) {
  std::ofstream outFile(path.c_str());
  outFile.precision(16);
  int len = vals.size();
  for (int i = 0; i < len - 1; i++)
    outFile << vals[i] << "\n";
  if (len > 0)
    outFile << vals[len-1];
  outFile.close();

}

// Computes a system solution from a seed and returns the final error number.
int find_solution_from_seed(RpcSolveLMA    const& lma_model,
                            Vector<double> const& seed_params,
                            Vector<double> const& actual_observations,
                            Vector<double>      & final_params,
                            double              & norm_error) {

  // Initialize a zero vector of RPC model coefficients
  int status;

  // Use the L-M solver to optimize the RPC model coefficient values.
  double abs_tolerance  = 1e-10;
  double rel_tolerance  = 1e-10;
  int    max_iterations = 1000;
  final_params = math::levenberg_marquardt(lma_model, seed_params, actual_observations,
                                           status, abs_tolerance, rel_tolerance,
                                           max_iterations);

  if (status < 1) { // This means the solver failed to converge
    VW_OUT(DebugMessage, "asp") 
      << "rpc_gen: WARNING --> Levenberg-Marquardt solver status = " << status << "\n";
  }

  // Otherwise the solver converged, return the final error number.
  Vector<double> final_projected = lma_model(final_params);
  Vector<double> final_error     = lma_model.difference(final_projected, actual_observations);
  norm_error = norm_2(final_error);
  return status;
}

// Find the best-fit affine transform, that will serve as initial guess for RPC
void initRpcAsAffine(// Inputs
                     Vector<double> const& normalized_geodetics,
                     Vector<double> const& normalized_pixels,
                     // Outputs
                     RPCModel::CoeffVec & line_num,
                     RPCModel::CoeffVec & line_den,
                     RPCModel::CoeffVec & samp_num,
                     RPCModel::CoeffVec & samp_den) {
  
  int numPts = normalized_geodetics.size()/RPCModel::GEODETIC_COORD_SIZE;
  int numPts2 = (normalized_pixels.size() - asp::RpcSolveLMA::NUM_PENALTY_TERMS)
    / RPCModel::IMAGE_COORD_SIZE;
  
  if (numPts != numPts2) 
    vw_throw( ArgumentErr() << "Error in " << __FILE__
              << ". Number of inputs and outputs do not agree.\n");
  std::vector< Vector<double, RPCModel::GEODETIC_COORD_SIZE+1> > in(numPts), out(numPts);
  for (int p = 0; p < numPts; p++) {

    Vector<double, RPCModel::GEODETIC_COORD_SIZE+1> P;
    for (int q = 0; q < RPCModel::GEODETIC_COORD_SIZE; q++) 
      P[q] = normalized_geodetics[p*RPCModel::GEODETIC_COORD_SIZE + q]; // P[0], P[1], P[2]
    P[RPCModel::GEODETIC_COORD_SIZE] = 1; // P[3]
    in[p] = P;
    
    for (int q = 0; q < RPCModel::IMAGE_COORD_SIZE; q++) 
      P[q] = normalized_pixels[p*RPCModel::IMAGE_COORD_SIZE + q]; // P[0], P[1]
    P[RPCModel::IMAGE_COORD_SIZE]   = 0; // P[2]
    P[RPCModel::IMAGE_COORD_SIZE+1] = 1; // P[3]
    out[p] = P;
  }
  Matrix4x4 T = math::AffineFittingFunctorN<RPCModel::GEODETIC_COORD_SIZE>()(in, out);

  // Put this matrix in the format acceptable for the RPC solver
  for (int p = 0; p < int(line_num.size()); p++) {
    samp_num[p] = 0; samp_den[p] = 0; // first coordinate of output is sample
    line_num[p] = 0; line_den[p] = 0; // second coordinate of output is line
  }

  // The first coordinate of the output
  samp_num[0] = T(0, 3); // the d value, the translation, in a*x + b*y + c*z + d
  samp_num[1] = T(0, 0); samp_num[2] = T(0, 1); samp_num[3] = T(0, 2); // linear part, a, b, c
  
  // The second coordinate of the output
  line_num[0] = T(1, 3); // the d value, the translation, in a*x + b*y + c*z + d
  line_num[1] = T(1, 0); line_num[2] = T(1, 1); line_num[3] = T(1, 2); // linear part, a, b, c

  // The denominator is just 1 to start
  samp_den[0] = 1.0;
  line_den[0] = 1.0;
}

// Form the normalized llh and pixel arrays that will be as inputs to the RPC solver
// Form the normalized llh and pixel arrays that will be as inputs to the RPC solver
void normalizeLlhPix(std::vector<vw::Vector3> const& llh_vec,
                     std::vector<vw::Vector2> const& pix_vec,
                     vw::Vector3 const& llh_offset, vw::Vector3 const& llh_scale, 
                     vw::Vector2 const& pixel_offset, vw::Vector2 const& pixel_scale, 
                     // Outputs
                     vw::Vector<double> & normalized_llh, 
                     vw::Vector<double> & normalized_pixels) {

  // Allocate the inputs, including with space for the penalty terms in the
  // normalized pixels, that are zero for now.
  int num_total_pts = llh_vec.size();
  normalized_llh.set_size(asp::RPCModel::GEODETIC_COORD_SIZE*num_total_pts);
  normalized_pixels.set_size(asp::RPCModel::IMAGE_COORD_SIZE*num_total_pts
                              + asp::RpcSolveLMA::NUM_PENALTY_TERMS);

  for (size_t i = 0; i < normalized_pixels.size(); i++)
    normalized_pixels[i] = 0.0; 

  // Form the arrays of normalized pixels and normalized llh
  for (int pt = 0; pt < num_total_pts; pt++) {
    // Normalize the pixel to the [-1, 1] range
    vw::Vector3 llh_n   = elem_quot(llh_vec[pt] - llh_offset, llh_scale);
    vw::Vector2 pixel_n = elem_quot(pix_vec[pt] - pixel_offset, pixel_scale);
    vw::math::subvector(normalized_llh, asp::RPCModel::GEODETIC_COORD_SIZE*pt,
              asp::RPCModel::GEODETIC_COORD_SIZE) = llh_n;
    vw::math::subvector(normalized_pixels, asp::RPCModel::IMAGE_COORD_SIZE*pt,
              asp::RPCModel::IMAGE_COORD_SIZE) = pixel_n;
  }
  
}
  
// Find the best-fit RPC given the normalized geodetic and pixel values
void gen_rpc(// Inputs
             double penalty_weight,
             Vector<double> const& normalized_geodetics,
             Vector<double> const& normalized_pixels,
             bool refine_only,
             // Outputs
             RPCModel::CoeffVec & line_num,
             RPCModel::CoeffVec & line_den,
             RPCModel::CoeffVec & samp_num,
             RPCModel::CoeffVec & samp_den) {

  VW_ASSERT(penalty_weight >= 0, 
            ArgumentErr() << "The RPC penalty weight must be non-negative.\n" );

  // The percentage of the error that the penalty weights should represent
  double penalty_weight_fraction = penalty_weight;
  // Fraction with no adjustment
double native_penalty_fraction
    = (double)RpcSolveLMA::NUM_PENALTY_TERMS / (double)normalized_pixels.size();
  double penalty_adjustment = penalty_weight_fraction / native_penalty_fraction;

  // Initialize the RPC model with an affine transform, unless asked to refine only
  if (!refine_only)
    initRpcAsAffine(normalized_geodetics, normalized_pixels,
                    line_num, line_den, samp_num, samp_den);      
  
  // Initialize the model
  Vector<double> startGuess;
  startGuess.set_size(RPCModel::NUM_RPC_COEFFS);
  packCoeffs(line_num, line_den, samp_num, samp_den, startGuess);

  Vector<double> solution;
  double norm_error = -1.0;

  // Initialize a specialized least squares solver object and load the input data
  RpcSolveLMA lma_model(normalized_geodetics, normalized_pixels, penalty_adjustment);

  // Use the L-M solver to optimize the RPC model coefficient values.
  int status = find_solution_from_seed(lma_model, startGuess, normalized_pixels,
                                       solution, norm_error);
  VW_OUT(DebugMessage, "asp") << "Solved RPC coeffs: " << solution << "\n";
  VW_OUT(DebugMessage, "asp") << "rpc_gen: norm_error = " << norm_error << "\n";

  // If we ever want to improve our results further we should
  // experiment with multiple starting seeds
  unpackCoeffs(solution, line_num, line_den, samp_num, samp_den);
}

// Extract the 3x3 rotation matrix R and 3D translation vector T from a 4x4 ECEF
// transformation matrix.
void extractRotTrans(vw::Matrix4x4 const& ecef_transform,
                     vw::Matrix3x3& R, vw::Vector3& T) {

   // Extract the 3x3 rotation submatrix
   R = submatrix(ecef_transform, 0, 0, 3, 3);
   
   // Extract the translation vector from the last column 
   T[0] = ecef_transform(0, 3);
   T[1] = ecef_transform(1, 3); 
   T[2] = ecef_transform(2, 3);
}

// Transform a vector of lon-lat-height coordinates in place using a rotation matrix and translation vector
void transformLlhVec(std::vector<vw::Vector3> const& llh,
                     vw::Matrix3x3 const& R, vw::Vector3 const& T,
                     vw::cartography::Datum const& datum,
                     std::vector<vw::Vector3> & trans_llh) {

  // Resize the output
  trans_llh.resize(llh.size());                          

  // Iterate in i over llh
  for (size_t i = 0; i < llh.size(); i++) {
    
    // Convert from LLH to ECEF XYZ
    Vector3 xyz = datum.geodetic_to_cartesian(llh[i]);
    
    // Apply rotation and translation
    Vector3 transformed_xyz = R * xyz + T;
    
    // Convert back to LLH and store in trans_llh
    trans_llh[i] = datum.cartesian_to_geodetic(transformed_xyz);
  }
}

// Apply a transform to a box in lon-lat-height coordinates
void transformLlhBox(vw::Vector3 const& llh_offset_in,
                              Vector3 const& llh_scale_in, 
                              vw::Matrix3x3 const& R,
                              vw::Vector3 const& T,
                              vw::cartography::Datum const& datum,
                              // Outputs
                              vw::Vector3& llh_offset_out,
                              vw::Vector3& llh_scale_out) {

   // First reconstruct the min/max bounds from offset and scale
   Vector3 llh_min = llh_offset_in - llh_scale_in;
   Vector3 llh_max = llh_offset_in + llh_scale_in;

   // Convert min/max to cartesian
   Vector3 xyz_min = datum.geodetic_to_cartesian(llh_min);
   Vector3 xyz_max = datum.geodetic_to_cartesian(llh_max);

   // Apply transform to both points
   Vector3 xyz_min_trans = R * xyz_min + T;
   Vector3 xyz_max_trans = R * xyz_max + T;

   // Convert transformed points back to llh
   Vector3 llh_min_trans = datum.cartesian_to_geodetic(xyz_min_trans);
   Vector3 llh_max_trans = datum.cartesian_to_geodetic(xyz_max_trans);

   // Calculate new offset and scale from transformed min/max
   llh_offset_out = (llh_max_trans + llh_min_trans) / 2.0;
   llh_scale_out = (llh_max_trans - llh_min_trans) / 2.0;
}

// Produce a transformed RPC model
asp::RPCModel transformRpc(asp::RPCModel const& rpc_model, 
                           vw::Matrix4x4 const& transform,
                           vw::BBox2 const& image_box,
                           double & pixel_err) {

  // Initialize the output pixel error
  pixel_err = 0.0;
  
  vw::Vector3 llh_offset       = rpc_model.lonlatheight_offset();
  vw::Vector3 llh_scale        = rpc_model.lonlatheight_scale();
  vw::Vector2 pixel_offset     = rpc_model.xy_offset();
  vw::Vector2 pixel_scale      = rpc_model.xy_scale();
  vw::cartography::Datum datum = rpc_model.datum();
  
  vw::BBox2 lon_lat_range;
  lon_lat_range.min() = subvector(llh_offset - llh_scale, 0, 2);
  lon_lat_range.max() = subvector(llh_offset + llh_scale, 0, 2);
  vw::Vector2 height_range;
  height_range[0] = llh_offset[2] - llh_scale[2];
  height_range[1] = llh_offset[2] + llh_scale[2];

  // Generate point pairs and add pixels along image perimeter and diagonals
  std::vector<Vector3> llh_vec;
  std::vector<Vector2> pix_vec;
  vw::CamPtr cam(new asp::RPCModel(rpc_model)); // a copy as required by the api
  int num_samples = 20; // TODO(oalexan1): Think about this
  asp::sample_llh_pix_bbox(lon_lat_range, height_range, num_samples, datum, cam, image_box,
                           llh_vec, pix_vec); // outputs
  asp::add_perimeter_diag_points(image_box, datum, cam, lon_lat_range, height_range,
                                 llh_vec, pix_vec); // outputs
  
  // Apply the transform to the llh vector and extent
  vw::Matrix3x3 R;
  vw::Vector3 T;
  extractRotTrans(transform, R, T);
  std::vector<Vector3> llh_vec_trans;
  transformLlhVec(llh_vec, R, T, datum, llh_vec_trans);
  vw::Vector3 llh_offset_trans, llh_scale_trans;
  transformLlhBox(llh_offset, llh_scale, R, T, datum,
                  llh_offset_trans, llh_scale_trans); // outputs

  // This penalty weight is rather small. In practice it was not a problem. The
  // input camera should be sampled well and then an overfit should not be a
  // concern.
  double penalty_weight = 1e-4;
  
  // Form the vectors of transformed normalized llh and pixel values
  Vector<double> normalized_llh_trans;
  Vector<double> normalized_pixels;
  asp::normalizeLlhPix(llh_vec_trans, pix_vec, 
                       llh_offset_trans, llh_scale_trans, pixel_offset, pixel_scale, 
                       normalized_llh_trans, normalized_pixels); // outputs

  // Form the transformed RPC model by fitting to the transformed normalized llh
  asp::RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
  line_num = rpc_model.line_num_coeff();
  line_den = rpc_model.line_den_coeff();
  samp_num = rpc_model.sample_num_coeff();
  samp_den = rpc_model.sample_den_coeff();
  bool refine_only = true;
  asp::gen_rpc(penalty_weight, normalized_llh_trans, normalized_pixels, refine_only,
               line_num, line_den, samp_num, samp_den); // outputs
  asp::RPCModel rpc_trans(datum, line_num, line_den, samp_num, samp_den,
                          pixel_offset, pixel_scale, llh_offset_trans, llh_scale_trans,
                          rpc_model.m_err_bias, rpc_model.m_err_rand);

  // Find the error of applying the adjustments inline as opposed to externally
  for (size_t i = 0; i < llh_vec.size(); i++) {
    Vector3 llh = llh_vec[i];
    Vector3 xyz = datum.geodetic_to_cartesian(llh);
    Vector3 llh_trans = llh_vec_trans[i];
    Vector3 xyz_trans = datum.geodetic_to_cartesian(llh_trans);
    Vector2 cam_pix1, cam_pix2;
    try {
      cam_pix1 = rpc_model.point_to_pixel(xyz);
      cam_pix2 = rpc_trans.point_to_pixel(xyz_trans);
    } catch (...) {
      continue;
    }
    pixel_err = std::max(pixel_err, norm_2(cam_pix1 - cam_pix2));
  }
  
  return rpc_trans;
}

} // end namespace asp
