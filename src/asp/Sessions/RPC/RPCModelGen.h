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

#ifndef __STEREO_SESSION_RPC_MODEL_GEN_H__
#define __STEREO_SESSION_RPC_MODEL_GEN_H__

#include <asp/Sessions/RPC/RPCModel.h>
#include <vw/Math/LevenbergMarquardt.h>

namespace asp {

  void unpackCoeffs(vw::Vector<double> const& C,
                    RPCModel::CoeffVec& lineNum, RPCModel::CoeffVec& lineDen,
                    RPCModel::CoeffVec& sampNum, RPCModel::CoeffVec& sampDen
                    );

  void packCoeffs( RPCModel::CoeffVec const& lineNum, RPCModel::CoeffVec const& lineDen,
                   RPCModel::CoeffVec const& sampNum, RPCModel::CoeffVec const& sampDen,
                   vw::Vector<double> & C
                   );

  // Find the best-fitting RPC coefficients for the camera transform
  // mapping a set of normalized geodetics to a set of normalized
  // pixel values.
  class RpcSolveLMA : public vw::math::LeastSquaresModelBase<RpcSolveLMA> {
    vw::Vector<double> m_normalizedGeodetics, m_normalizedPixels;
    double m_wt;
  public:
    typedef vw::Vector<double> result_type; // normalized pixels
    typedef result_type domain_type;        // RPC coefficients
    typedef vw::Matrix<double> jacobian_type;

    RpcSolveLMA( const vw::Vector<double>& normalizedGeodetics,
                 const vw::Vector<double>& normalizedPixels,
                 double penaltyWeight
                 ) :
      m_normalizedGeodetics(normalizedGeodetics),
      m_normalizedPixels(normalizedPixels),
      m_wt(penaltyWeight){}

    inline result_type operator()( domain_type const& C ) const {

      // The input is the RPC coefficients, packed in a vector.
      // For each normalized geodetic, compute the normalized
      // pixel value. This will be the output.

      // Add the penalization terms to the output,
      // see the note later in the code.

      RPCModel::CoeffVec lineNum, lineDen, sampNum, sampDen;
      unpackCoeffs(C, lineNum, lineDen, sampNum, sampDen);

      int numPts = m_normalizedGeodetics.size()/3;

      result_type result;
      result.set_size(m_normalizedPixels.size());
      for (int i = 0; i < numPts; i++){
        vw::Vector3 G = subvector(m_normalizedGeodetics, 3*i, 3);
        // Note that we normalize the cost function by numPts.
        subvector(result, 2*i, 2)
          = RPCModel::normalized_geodetic_to_normalized_pixel
          (G, lineNum, lineDen, sampNum, sampDen)/numPts;
      }

      // There are 4*20 - 2 = 78 coefficients we optimize. Of those, 2
      // are 0-th degree, 4*3 = 12 are 1st degree, and the rest, 78 - 12
      // - 2 = 64 are higher degree.  Per Hartley, we'll add for each
      // such coefficient c, a term K*c in the cost function vector,
      // where K is a large number. This will penalize large values in
      // the higher degree coefficients.
      int count = 2*numPts;
      for (int i = 4; i < (int)lineNum.size(); i++) result[count++] = m_wt*lineNum[i];
      for (int i = 4; i < (int)lineDen.size(); i++) result[count++] = m_wt*lineDen[i];
      for (int i = 4; i < (int)sampNum.size(); i++) result[count++] = m_wt*sampNum[i];
      for (int i = 4; i < (int)sampDen.size(); i++) result[count++] = m_wt*sampDen[i];

      VW_ASSERT((int)result.size() == count,
                vw::ArgumentErr() << "Book-keeping error.\n");

      return result;
    }

  };


}

#endif //__STEREO_SESSION_RPC_MODEL_GEN_H__
