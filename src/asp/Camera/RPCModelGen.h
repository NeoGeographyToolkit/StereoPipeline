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

// RPC model generation. See rpc_gen.cc for details.

#ifndef __STEREO_CAMERA_RPC_MODEL_GEN_H__
#define __STEREO_CAMERA_RPC_MODEL_GEN_H__

#include <asp/Camera/RPCModel.h>
#include <vw/Math/LevenbergMarquardt.h>

namespace asp {

/// Unpack the 78 RPC coefficients from one long vector into four separate vectors.
void unpackCoeffs(vw::Vector<double> const& C,
                  RPCModel::CoeffVec& lineNum, RPCModel::CoeffVec& lineDen,
                  RPCModel::CoeffVec& sampNum, RPCModel::CoeffVec& sampDen);

/// Pack the 78 RPC coefficients from four separate vectors into one long vector.
void packCoeffs(RPCModel::CoeffVec const& lineNum, RPCModel::CoeffVec const& lineDen,
                RPCModel::CoeffVec const& sampNum, RPCModel::CoeffVec const& sampDen,
                vw::Vector<double> & C);

/// Find the best-fitting RPC coefficients for the camera transform
/// mapping a set of normalized geodetics to a set of normalized pixel values.
class RpcSolveLMA: public vw::math::LeastSquaresModelBase<RpcSolveLMA> {
  
  /// The normalized values are in the -1 to 1 range.
  vw::Vector<double> m_normalizedGeodetics; 
  vw::Vector<double> m_normalizedPixels; // Contains the extra penalty terms
  double             m_wt; // The penalty weight, k in the reference paper.
  
public:
  
  // For applying penalty weighting per the Hartley paper.
  static const int NUM_PENALTY_TERMS = 64;

  typedef vw::Vector<double> result_type;   // normalized pixels
  typedef result_type        domain_type;   // RPC coefficients
  typedef vw::Matrix<double> jacobian_type;

  /// Instantiate the solver with a set of GDC <--> Pixel pairs.
  RpcSolveLMA(const vw::Vector<double>& normalizedGeodetics,
              const vw::Vector<double>& normalizedPixels,
              double penaltyWeight):
    m_normalizedGeodetics(normalizedGeodetics),
    m_normalizedPixels(normalizedPixels),
    m_wt(penaltyWeight) {}

  // Given a set of RPC coefficients, compute the projected pixels.
  result_type operator()(domain_type const& C) const;
};

/// Print out a name followed by the vector of values
void print_vec(std::string const& name, vw::Vector<double> const& vals);

/// Dump a vector to a text file, one value per row.
void print_vec_to_file(std::string const& path, vw::Vector<double> const& vals);

// Form the normalized llh and pixel arrays that will be as inputs to the RPC solver
void normalizeLlhPix(std::vector<vw::Vector3> const& all_llh,
                     std::vector<vw::Vector2> const& all_pixels,
                     vw::Vector3 const& llh_scale, vw::Vector3 const& llh_offset,
                     vw::Vector2 const& pixel_scale, vw::Vector2 const& pixel_offset,
                     // Outputs
                     vw::Vector<double> & normalized_llh, 
                     vw::Vector<double> & normalized_pixels);

int find_solution_from_seed(RpcSolveLMA        const& lma_model,
                            vw::Vector<double> const& seed_params,
                            vw::Vector<double> const& actual_observations,
                            vw::Vector<double>      & final_params,
                            double                  & norm_error);

void gen_rpc(// Inputs
              double penalty_weight,
              vw::Vector<double> const& normalized_geodetics,
              vw::Vector<double> const& normalized_pixels,
              vw::Vector3 const& llh_scale,
              vw::Vector3 const& llh_offset,
              vw::Vector2 const& uv_scale,
              vw::Vector2 const& uv_offset,
              bool refine_only,
              // Outputs
              RPCModel::CoeffVec & line_num,
              RPCModel::CoeffVec & line_den,
              RPCModel::CoeffVec & samp_num,
              RPCModel::CoeffVec & samp_den);

} // end namespace asp

#endif //__STEREO_CAMERA_RPC_MODEL_GEN_H__
