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
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace asp;
using namespace xercesc;

struct Options : asp::BaseOptions {
  // Input
  std::string camera_model;
  // Settings
  double penalty_weight;
  BBox3 lon_lat_height_box;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("penalty-weight", po::value(&opt.penalty_weight)->default_value(0.1),
     "Penalty weight to use to keep the higher-order RPC coefficients small. Higher penalty weight results in smaller such coefficients.")
    ("lon-lat-height-box", po::value(&opt.lon_lat_height_box)->default_value(BBox3(0,0,0,0,0,0)),
     "The 3D region in which to solve for the PRC model [lon_min lat_min height_min lon_max lat_max height_max].");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("camera-model", po::value(&opt.camera_model));

  po::positional_options_description positional_desc;
  positional_desc.add("camera-model", 1);

  std::string usage("[options] <camera_model>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( !vm.count("camera-model") )
    vw_throw( ArgumentErr() << "Requires <camera_model> input in order to proceed.\n\n"
              << usage << general_options );

}

int main( int argc, char* argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    std::cout << "penalty: " << opt.penalty_weight << std::endl;
    std::cout << "xml file: " << opt.camera_model << std::endl;
    std::cout << "box is " << opt.lon_lat_height_box << std::endl;

    XMLPlatformUtils::Initialize();

    StereoSessionDG session;
    boost::shared_ptr<camera::CameraModel>
      cam_dg( session.camera_model("", opt.camera_model) );

    RPCXML xml;
    xml.read_from_file( opt.camera_model );
    boost::shared_ptr<RPCModel> cam_rpc( new RPCModel( *xml.rpc_ptr() ) );

    // To do: Test with binary builder that this executable goes in the right place.

    // To do: Cleanup below.

    Vector3 llh_scale  = cam_rpc->lonlatheight_scale();
    Vector3 llh_offset = cam_rpc->lonlatheight_offset();
    std::cout << "llh scale off: " << llh_scale << ' ' << llh_offset << std::endl;

    Vector2 xy_scale  = cam_rpc->xy_scale();
    Vector2 xy_offset = cam_rpc->xy_offset();
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
          Vector3 P = cam_rpc->datum().geodetic_to_cartesian(G); // xyz
          Vector2 pxg = cam_dg->point_to_pixel(P);
          Vector2 pxn = elem_quot(pxg - xy_offset, xy_scale);

          Vector2 pxr = cam_rpc->point_to_pixel(P);
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

    RpcSolveLMA lma_model (normalizedGeodetics, normalizedPixels, opt.penalty_weight);
    int status;

    // Use the current model as an initial guess

    Vector<double> start;
    packCoeffs(cam_rpc->line_num_coeff(), cam_rpc->line_den_coeff(),
               cam_rpc->sample_num_coeff(), cam_rpc->sample_den_coeff(),
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

  } ASP_STANDARD_CATCHES;

  return 0;
}
