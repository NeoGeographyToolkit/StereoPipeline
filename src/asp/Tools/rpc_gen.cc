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

void print_vec(std::string const& name, Vector<double> const& vals){
  std::cout.precision(16);
  std::cout << name << ",";
  int len = vals.size();
  for (int i = 0; i < len - 1; i++) std::cout << vals[i] << ",";
  if (len > 0) std::cout << vals[len-1];
  std::cout << std::endl;
}

int main( int argc, char* argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    VW_ASSERT( opt.penalty_weight >= 0,
               ArgumentErr() << "The RPC penalty weight must be non-negative.\n" );

    XMLPlatformUtils::Initialize();
    StereoSessionDG session;
    boost::shared_ptr<camera::CameraModel>
      cam_dg( session.camera_model("", opt.camera_model) );

    RPCXML xml;
    xml.read_from_file( opt.camera_model );
    boost::shared_ptr<RPCModel> cam_rpc( new RPCModel( *xml.rpc_ptr() ) );
    Vector2 image_size = xml_image_size(opt.camera_model);

    // Normalization in the lon-lat-height and pixel domains
    Vector3 b = opt.lon_lat_height_box.min();
    Vector3 e = opt.lon_lat_height_box.max();
    Vector3 llh_scale  = (e - b)/2.0;
    Vector3 llh_offset = (e + b)/2.0;
    Vector2 xy_scale   = image_size/2.0;
    Vector2 xy_offset  = image_size/2.0;

    // Number of points in x and y at which we will optimize the RPC
    // model. Using 10 or 20 points gives roughly similar results.
    // 20 points result in 20^3 input data for optimization, with the
    // number of variable to optimize being just 78.
    int num_pts = 20;
    int num_total_pts = num_pts*num_pts*num_pts;

    // See comment about penalization in class RpcSolveLMA().
    int numExtraTerms = 64;

    Vector<double> normalizedGeodetics;
    normalizedGeodetics.set_size(3*num_total_pts);
    Vector<double> normalizedPixels;
    normalizedPixels.set_size(2*num_total_pts + numExtraTerms);
    for (int i = 0; i < (int)normalizedPixels.size(); i++)
      normalizedPixels[i] = 0.0;

    int count = 0;
    for (int x = 0; x < num_pts; x++){
      for (int y = 0; y < num_pts; y++){
        for (int z = 0; z < num_pts; z++){

          Vector3 U( x/(num_pts - 1.0), y/(num_pts - 1.0), z/(num_pts - 1.0) );
          U = 2*U - Vector3(1, 1, 1); // in the box [-1, 1]^3.

          Vector3 G = elem_prod(U, llh_scale) + llh_offset; // geodetic
          Vector3 P = cam_rpc->datum().geodetic_to_cartesian(G); // xyz
          Vector2 pxg = cam_dg->point_to_pixel(P);
          Vector2 pxn = elem_quot(pxg - xy_offset, xy_scale);

          // It is a useful exercise to compare DG and RPC cameras
          //Vector2 pxr = cam_rpc->point_to_pixel(P);
          //std::cout << U << ' ' << P << ' ' << pxg << ' ' << pxr  << ' '
          //          << norm_2(pxg-pxr)<< std::endl;

          subvector(normalizedGeodetics, 3*count, 3) = U;

          // Note that we normalize the error vector below
          subvector(normalizedPixels, 2*count, 2) = pxn/num_total_pts;

          count++;

        }
      }
    }

    RpcSolveLMA lma_model (normalizedGeodetics, normalizedPixels,
                           opt.penalty_weight);
    int status;
    Vector<double> start; start.set_size(78);
    for (int i = 0; i < (int)start.size(); i++) start[i] = 0.0;
    Vector<double> solution =
      math::levenberg_marquardt( lma_model, start, normalizedPixels, status,
                                 1e-16, 1e-16, 1e3 );

    // Dump the output to stdout, to be parsed by python
    RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
    unpackCoeffs(solution, line_num, line_den, samp_num, samp_den);
    print_vec("xy_scale",   xy_scale);
    print_vec("xy_offset",  xy_offset);
    print_vec("llh_scale",  llh_scale);
    print_vec("llh_offset", llh_offset);
    print_vec("line_num",   line_num);
    print_vec("line_den",   line_den);
    print_vec("samp_num",   samp_num);
    print_vec("samp_den",   samp_den);

  } ASP_STANDARD_CATCHES;

  return 0;
}
