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
/// Reference: The Cubic Rational Polynomial Camera Model, Hartley, 2001.

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

/// Structure for storing user options
struct RPC_gen_Options : asp::BaseOptions {
  // Input
  std::string camera_model;
  // Settings
  double penalty_weight;
  BBox3  lon_lat_height_box;
};

/// Parse input arguments
void handle_arguments( int argc, char *argv[], RPC_gen_Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("penalty-weight", po::value(&opt.penalty_weight)->default_value(0.1),
     "Penalty weight to use to keep the higher-order RPC coefficients small. Higher penalty weight results in smaller such coefficients.")
    ("lon-lat-height-box", po::value(&opt.lon_lat_height_box)->default_value(BBox3(0,0,0,0,0,0)),
     "The 3D region in which to solve for the RPC model [lon_min lat_min height_min lon_max lat_max height_max].");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("camera-model", po::value(&opt.camera_model));
  po::positional_options_description positional_desc;
  positional_desc.add("camera-model", 1);

  std::string usage("[options] <camera_model>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( !vm.count("camera-model") )
    vw_throw( ArgumentErr() << "Requires <camera_model> input in order to proceed.\n\n"
              << usage << general_options );

}

/// Print out a name followed by the vector of values
void print_vec(std::string const& name, Vector<double> const& vals){
  std::cout.precision(16);
  std::cout << name << ",";
  int len = vals.size();
  for (int i = 0; i < len - 1; i++) 
    std::cout << vals[i] << ",";
  if (len > 0) 
    std::cout << vals[len-1];
  std::cout << std::endl;
}

int main( int argc, char* argv[] ) {

  RPC_gen_Options opt;
  try {
    handle_arguments( argc, argv, opt );

    VW_ASSERT( opt.penalty_weight >= 0,
               ArgumentErr() << "The RPC penalty weight must be non-negative.\n" );

    // Load up the Digital Globe camera model from the camera file
    XMLPlatformUtils::Initialize();
    StereoSessionDG session;
    boost::shared_ptr<camera::CameraModel> cam_dg( session.camera_model("", opt.camera_model) );

    // Load up the RPC camera model from the camera file
    RPCXML xml;
    xml.read_from_file( opt.camera_model );
    boost::shared_ptr<RPCModel> cam_rpc( new RPCModel( *xml.rpc_ptr() ) );
    Vector2 image_size = xml_image_size(opt.camera_model);

    // Normalization in the lon-lat-height and pixel domains
    // - Compute an offset and scale to place all values in the
    //   zero to one range centered on the center coordinate/pixel.
    Vector3 min_llh_coord = opt.lon_lat_height_box.min();
    Vector3 max_llh_coord = opt.lon_lat_height_box.max();
    Vector3 llh_scale     = (max_llh_coord - min_llh_coord)/2.0; // half range
    Vector3 llh_offset    = (max_llh_coord + min_llh_coord)/2.0; // center point
    Vector2 uv_scale      = image_size/2.0; // half range
    Vector2 uv_offset     = image_size/2.0; // center point

    // Number of points in x and y at which we will optimize the RPC
    // model. Using 10 or 20 points gives roughly similar results.
    // 20 points result in 20^3 input data for optimization, with the
    // number of variable to optimize being just 78.
    int num_pts = 20; // The number of points per axis
    int num_total_pts = num_pts*num_pts*num_pts;

    // See comment about penalization in class RpcSolveLMA().
    int numExtraTerms = 64;

    // Initialize normalized data storage
    Vector<double> normalizedGeodetics;
    normalizedGeodetics.set_size(RPCModel::GEODETIC_COORD_SIZE*num_total_pts);
    Vector<double> normalizedPixels;
    normalizedPixels.set_size(RPCModel::IMAGE_COORD_SIZE*num_total_pts + numExtraTerms);
    for (int i = 0; i < (int)normalizedPixels.size(); i++)
      normalizedPixels[i] = 0.0;

    // Loop through all test points and generate the "correct" pairs / training data
    //  using the trusted DG camera model.
    int count = 0;
    for (int x = 0; x < num_pts; x++){
      for (int y = 0; y < num_pts; y++){
        for (int z = 0; z < num_pts; z++){

          // Test points are evenly spaced through the x/y/z -1 <> 1 range
          Vector3 U( x/(num_pts - 1.0), 
                     y/(num_pts - 1.0), 
                     z/(num_pts - 1.0) );
                  U = 2.0*U - Vector3(1, 1, 1); // in the box [-1, 1]^3.

          // Linear conversion from x/y/z to lat/lon/height
          Vector3 G   = elem_prod(U, llh_scale) + llh_offset; // geodetic
          
          // Convert from geodetic to geocentric coordinates
          Vector3 P   = cam_rpc->datum().geodetic_to_cartesian(G); // xyz
          
          // Project the GCC coordinate into the DG camera model
          Vector2 pxg = cam_dg->point_to_pixel(P);
          
          // Normalize the pixel to -1 <> 1 range
          Vector2 pxn = elem_quot(pxg - uv_offset, uv_scale);

          // It is a useful exercise to compare DG and RPC cameras
          //Vector2 pxr = cam_rpc->point_to_pixel(P);
          //std::cout << U << ' ' << P << ' ' << pxg << ' ' << pxr  << ' '
          //          << norm_2(pxg-pxr)<< std::endl;


          // Note that we normalize the error vector below          
          // TODO: Why is pxn divided here?
          subvector(normalizedGeodetics, RPCModel::GEODETIC_COORD_SIZE*count, RPCModel::GEODETIC_COORD_SIZE) = U;
          subvector(normalizedPixels,    RPCModel::IMAGE_COORD_SIZE   *count, RPCModel::IMAGE_COORD_SIZE   ) = pxn/num_total_pts;

          count++;

        } // End z loop
      } // End y loop
    } // End x loop

    // Initialize a specialized least squares solver object and load the input data
    RpcSolveLMA lma_model (normalizedGeodetics, normalizedPixels, opt.penalty_weight);
    
    // Initialize a zero vector of RPC model coefficients
    int status;
    Vector<double> start; 
    start.set_size(RPCModel::NUM_RPC_COEFFS);
    for (int i = 0; i < (int)start.size(); i++) 
      start[i] = 0.0;
    
    // Use the L-M solver to optimize the RPC model coefficient values.
    Vector<double> solution =
      math::levenberg_marquardt( lma_model, start, normalizedPixels, status,
                                 1e-16, 1e-16, 1e3 );

    // Dump the output to stdout, to be parsed by python
    RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
    unpackCoeffs(solution, line_num, line_den, samp_num, samp_den);
    print_vec("uv_scale",   uv_scale  );
    print_vec("uv_offset",  uv_offset );
    print_vec("llh_scale",  llh_scale );
    print_vec("llh_offset", llh_offset);
    print_vec("line_num",   line_num  );
    print_vec("line_den",   line_den  );
    print_vec("samp_num",   samp_num  );
    print_vec("samp_den",   samp_den  );

  } ASP_STANDARD_CATCHES;

  return 0;
}
