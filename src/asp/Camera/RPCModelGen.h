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

  /// Unpack the 78 RPC coefficients from one long vector into four seperate vectors.
  void unpackCoeffs(vw::Vector<double> const& C,
                    RPCModel::CoeffVec& lineNum, RPCModel::CoeffVec& lineDen,
                    RPCModel::CoeffVec& sampNum, RPCModel::CoeffVec& sampDen
                    );

  /// Pack the 78 RPC coefficients from four seperate vectors into one long vector.
  void packCoeffs( RPCModel::CoeffVec const& lineNum, RPCModel::CoeffVec const& lineDen,
                   RPCModel::CoeffVec const& sampNum, RPCModel::CoeffVec const& sampDen,
                   vw::Vector<double> & C
                   );

  /// Find the best-fitting RPC coefficients for the camera transform
  /// mapping a set of normalized geodetics to a set of normalized pixel values.
  class RpcSolveLMA : public vw::math::LeastSquaresModelBase<RpcSolveLMA> {
    
    /// The normalized values are in the -1 to 1 range.
    vw::Vector<double> m_normalizedGeodetics, 
                       m_normalizedPixels; ///< Also contains the extra penalty terms
    double             m_wt; ///< The penalty weight, k in the reference paper.
    
  public:
   
    // For applying penalty weighting per the Hartley paper.
    static const int NUM_PENALTY_TERMS = 64;
  
    typedef vw::Vector<double> result_type;   // normalized pixels
    typedef result_type        domain_type;   // RPC coefficients
    typedef vw::Matrix<double> jacobian_type;

    /// Instantiate the solver with a set of GDC <--> Pixel pairs.
    RpcSolveLMA( const vw::Vector<double>& normalizedGeodetics,
                 const vw::Vector<double>& normalizedPixels,
                 double penaltyWeight
                 ) :
      m_normalizedGeodetics(normalizedGeodetics),
      m_normalizedPixels(normalizedPixels),
      m_wt(penaltyWeight){}

    /// Given a set of RPC coefficients, compute the projected pixels.
    inline result_type operator()( domain_type const& C ) const {

      // The input is the RPC coefficients, packed in a vector.
      // For each normalized geodetic, compute the normalized
      // pixel value. This will be the output.

      // Add the penalization terms to the output,
      // see the note later in the code.

      // Unpack all the RPC model coefficients from the input vector C
      RPCModel::CoeffVec lineNum, lineDen, sampNum, sampDen;
      unpackCoeffs(C, lineNum, lineDen, sampNum, sampDen);

      // Initialize the output vector
      int numPts = m_normalizedGeodetics.size()/RPCModel::GEODETIC_COORD_SIZE;
      result_type result;
      result.set_size(m_normalizedPixels.size());
      
      // Loop through each test point
      for (int i = 0; i < numPts; i++){
        // Unpack the normalized Geodetic coordinate
        vw::Vector3 G = subvector(m_normalizedGeodetics, RPCModel::GEODETIC_COORD_SIZE*i, RPCModel::GEODETIC_COORD_SIZE);
        
        // Project the normalized geodetic coordinate into the RPC camera to get a normalized pixel
        vw::Vector2 pxn = RPCModel::normalized_geodetic_to_normalized_pixel(G, lineNum, lineDen, sampNum, sampDen);
             
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
      for (int i = 4; i < (int)lineNum.size(); i++)  result[count++] = m_wt*lineNum[i] * (coeff_order[i]-1);
      for (int i = 4; i < (int)lineDen.size(); i++)  result[count++] = m_wt*lineDen[i] * (coeff_order[i]-1);
      for (int i = 4; i < (int)sampNum.size(); i++)  result[count++] = m_wt*sampNum[i] * (coeff_order[i]-1);
      for (int i = 4; i < (int)sampDen.size(); i++)  result[count++] = m_wt*sampDen[i] * (coeff_order[i]-1);

      VW_ASSERT((int)result.size() == count, vw::ArgumentErr() << "Book-keeping error.\n");

      return result;
    }

  };

  /// Print out a name followed by the vector of values
  void print_vec(std::string const& name, vw::Vector<double> const& vals);

  /// Dump a vector to a text file, one value per row.
  void print_vec_to_file(std::string const& path, vw::Vector<double> const& vals);
  
  void write_levmar_solver_results(std::string const& output_prefix, int status,
                                   vw::Vector<double> const& initial_params,
                                   vw::Vector<double> const& final_params,
                                   vw::Vector<double> const& actual_observation,
                                   RpcSolveLMA const& lma_model);
  
  int find_solution_from_seed(RpcSolveLMA    const& lma_model,
                              vw::Vector<double> const& seed_params,
                              vw::Vector<double> const& actual_observations,
                              vw::Vector<double>      & final_params,
                              double              & norm_error);
  
  void gen_rpc(// Inputs
               double penalty_weight,
               std::string    const& output_prefix,
               vw::Vector<double> const& normalized_geodetics,
               vw::Vector<double> const& normalized_pixels,
               vw::Vector3 const& llh_scale,
               vw::Vector3 const& llh_offset,
               vw::Vector2 const& uv_scale,
               vw::Vector2 const& uv_offset,
               // Outputs
               RPCModel::CoeffVec & line_num,
               RPCModel::CoeffVec & line_den,
               RPCModel::CoeffVec & samp_num,
               RPCModel::CoeffVec & samp_den);
}

#endif //__STEREO_CAMERA_RPC_MODEL_GEN_H__
