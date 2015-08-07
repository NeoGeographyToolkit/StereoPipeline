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
#include <asp/Camera/RPCModel.h>

using namespace vw;

namespace asp {

  void unpackCoeffs(Vector<double> const& C,
                    RPCModel::CoeffVec& lineNum, RPCModel::CoeffVec& lineDen,
                    RPCModel::CoeffVec& sampNum, RPCModel::CoeffVec& sampDen
                    ){

    // There are 20 polynomial coefficients for each of lineNum,
    // lineDen, sampNum, sampDen, as the RPC polynomials are of degree
    // 3 in x and y.  However, the 0 degree coeffs for both
    // denominators is 1, so only 78 coefficients vary. Extract the
    // variable coefficients from a vector.

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

  void packCoeffs( RPCModel::CoeffVec const& lineNum, RPCModel::CoeffVec const& lineDen,
                   RPCModel::CoeffVec const& sampNum, RPCModel::CoeffVec const& sampDen,
                   Vector<double> & C
                   ){

    // This function does the reverse of unpackCoeffs().

    C.set_size(78);
    int start = 0;
    subvector(C, start, 20) = subvector(lineNum, 0, 20); start += 20;
    subvector(C, start, 19) = subvector(lineDen, 1, 19); start += 19;
    subvector(C, start, 20) = subvector(sampNum, 0, 20); start += 20;
    subvector(C, start, 19) = subvector(sampDen, 1, 19); start += 19;

    return;
  }

}
