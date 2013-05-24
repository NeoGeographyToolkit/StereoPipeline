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


/// \file rpc_gen.cc
///
/// This program will generate an RPC model from a DG model.
/// It will work by creating a 3D grid in the provided lon-lat-height box,
/// at each of those points finding the corresponding pixel value, and
/// then finding the best-fitting RPC transform. For increased
/// accuracy, both lon-lat-height and pixel values are normalized.
///
/// Reference: The Cubic Rational Polynomial Camera Model, Hartley,
/// 2001.

#include <asp/Sessions/RPC/RPCModelGen.h>
#include <asp/Sessions/RPC/StereoSessionRPC.h>
#include <asp/Sessions/RPC/RPCStereoModel.h>
#include <asp/Sessions/DG/XML.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

using namespace vw;
using namespace asp;
using namespace xercesc;

typedef RPCModel::CoeffVec rmc;

void unpackCoeffs(Vector<double> const& C,
                  rmc& lineNum, rmc& lineDen, rmc& sampNum, rmc& sampDen
                  ){

  // There are 20 polynomial coefficients for each of lineNum,
  // lineDen, sampNum, sampDen, as the RPC polynomials are of degree 3
  // in x and y.  However, the 0 degree coeffs for both denominators
  // is 1, so only 78 coefficients vary.

  // Extract the variable coefficients from a vector.

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

void packCoeffs( rmc const& lineNum, rmc const& lineDen,
                 rmc const& sampNum, rmc const& sampDen,
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
      Vector3 G = subvector(m_normalizedGeodetics, 3*i, 3);
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
            ArgumentErr() << "Book-keeping error.\n");


    double e = norm_2(m_normalizedPixels - result);
    std::cout << "error is " << e << std::endl;

    return result;
  }

};


int main( int argc, char* argv[] ) {

  XMLPlatformUtils::Initialize();

  double penaltyWeight = 0.01;
  std::string xml_file = "WV01_11JAN131652222-P1BS-10200100104A0300.xml";

  StereoSessionDG session;
  boost::shared_ptr<camera::CameraModel>
    cam_dg( session.camera_model("", xml_file) );

  // Create an RPC Model
  RPCXML xml2;
  xml2.read_from_file( xml_file );
  RPCModel * model = new RPCModel( *xml2.rpc_ptr() );
  boost::shared_ptr<camera::CameraModel> cam_rp(model);

  GeometricXML geo;
  AttitudeXML att;
  EphemerisXML eph;
  ImageXML img;
  RPCXML rpc;
  asp::read_xml(xml_file, geo, att, eph, img, rpc);
  std::cout << "image size is " << img.image_size << std::endl;
  int sizeX = img.image_size[0];
  int sizeY = img.image_size[1];
  std::cout << "cols is " << sizeX << std::endl;
  std::cout << "rows is " << sizeY << std::endl;

  Vector3 llh_scale  = model->lonlatheight_scale();
  Vector3 llh_offset = model->lonlatheight_offset();
  std::cout << "llh scale off: " << llh_scale << ' ' << llh_offset << std::endl;

  Vector2 xy_scale  = model->xy_scale();
  Vector2 xy_offset = model->xy_offset();
  std::cout << "xy scale off: " << xy_scale << ' ' << xy_offset << std::endl;

  // Number of points in x and y at which we will optimize the RPC model
  int numPts = 10;

  int numTotalPts = numPts*numPts*numPts;

  std::cout << "numPts2 is " << numTotalPts << std::endl;
  // See comment about penalization in class RpcSolveLMA().
  int numExtraTerms = 64;

  Vector<double> normalizedGeodetics; normalizedGeodetics.set_size(3*numTotalPts);
  Vector<double> normalizedPixels; normalizedPixels.set_size(2*numTotalPts + numExtraTerms);
  for (int i = 0; i < (int)normalizedPixels.size(); i++) normalizedPixels[i] = 0.0;

  int count = 0;
  for (int x = 0; x < numPts; x++){
    for (int y = 0; y < numPts; y++){
      for (int z = 0; z < numPts; z++){

        Vector3 U( x/(numPts - 1.0), y/(numPts - 1.0), z/(numPts - 1.0) );
        U = 2*U - Vector3(1, 1, 1); // in the box [-1, 1]^3.

        Vector3 G = elem_prod(U, llh_scale) + llh_offset; // geodetic
        Vector3 P = model->datum().geodetic_to_cartesian(G); // xyz
        Vector2 pxg = cam_dg->point_to_pixel(P);
        Vector2 pxn = elem_quot(pxg - xy_offset, xy_scale);

        Vector2 pxr = cam_rp->point_to_pixel(P);
        //std::cout << U << ' ' << P << ' ' << pxg << ' ' << pxr  << ' '
        //          << norm_2(pxg-pxr)<< std::endl;
        //std::cout << U << ' ' << pxn << std::endl;

        subvector(normalizedGeodetics, 3*count, 3) = U;
        // Note that we normalize the error vector below
        subvector(normalizedPixels, 2*count, 2) = pxn/numTotalPts;
        count++;

      }
    }
  }

  RpcSolveLMA lma_model (normalizedGeodetics, normalizedPixels, penaltyWeight);
  int status;

  // Use the current model as an initial guess

  Vector<double> start;
  packCoeffs(model->line_num_coeff(), model->line_den_coeff(),
             model->sample_num_coeff(), model->sample_den_coeff(),
             start);

  for (int i = 0; i < (int)start.size(); i++) start[i] = 0.0;
  std::cout << "start is " << start << std::endl;
  //   double err1 = calcError(model, start, normalizedGeodetics, normalizedPixels);
//   std::cout << "start error is " << err1 << std::endl;

  RPCModel::CoeffVec lineNum, lineDen, sampNum, sampDen;
  unpackCoeffs(start, lineNum, lineDen, sampNum, sampDen);
  std::cout << "1 lineNum: " << lineNum << std::endl;
  std::cout << "1 lineDen: " << lineDen << std::endl;
  std::cout << "1 sampNum: " << sampNum << std::endl;
  std::cout << "1 sampDen: " << sampDen << std::endl;

  Vector<double> solution =
    math::levenberg_marquardt( lma_model, start, normalizedPixels, status,
                               1e-16, 1e-16, 1e3 );

  std::cout << "solution is " << solution << std::endl;
//   double err2 = calcError(model, solution, normalizedGeodetics, normalizedPixels);
//   std::cout << "stop error is " << err2 << std::endl;

  unpackCoeffs(solution, lineNum, lineDen, sampNum, sampDen);
  std::cout << "1 lineNum: " << lineNum << std::endl;
  std::cout << "1 lineDen: " << lineDen << std::endl;
  std::cout << "1 sampNum: " << sampNum << std::endl;
  std::cout << "1 sampDen: " << sampDen << std::endl;

  return 0;
}
