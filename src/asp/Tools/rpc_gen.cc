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

#include <fstream>

#include <asp/Camera/RPCModelGen.h>
#include <asp/Camera/DG_XML.h>
#include <asp/Sessions/StereoSessionFactory.h>
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
  std::string output_prefix;
};

/// Parse input arguments
void handle_arguments( int argc, char *argv[], RPC_gen_Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix",      po::value(&opt.output_prefix)->default_value(""),
             "Write diagnostic/report files to this location.  If not set then no files will be written.")
    ("penalty-weight",     po::value(&opt.penalty_weight)->default_value(0.1),
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

/// Dump a vector to a text file, one value per row.
void print_vec_to_file(std::string const& path, Vector<double> const& vals) {
  std::ofstream outFile(path.c_str());
  outFile.precision(16);
  int len = vals.size();
  for (int i = 0; i < len - 1; i++)
    outFile << vals[i] << std::endl;
  if (len > 0)
    outFile << vals[len-1];
  outFile.close();

}

void write_levmar_solver_results(std::string const& output_prefix, int status,
                                 Vector<double> const& initial_params,
                                 Vector<double> const& final_params,
                                 Vector<double> const& actual_observation,
                                 RpcSolveLMA const& lma_model) {

  // Compute initial and final numbers
  Vector<double> initial_projected = lma_model(initial_params);
  Vector<double> final_projected   = lma_model(final_params);
  Vector<double> initial_error     = lma_model.difference(initial_projected, actual_observation);
  Vector<double> final_error       = lma_model.difference(final_projected,   actual_observation);

  // Log the solver status
  VW_OUT(VerboseDebugMessage, "math") << "rpc_gen: levmar solver status = " << status << std::endl;
  VW_OUT(VerboseDebugMessage, "math") << "rpc_gen: levmar solver initial error norm_2 = " << norm_2(initial_error) << std::endl;
  VW_OUT(VerboseDebugMessage, "math") << "rpc_gen: levmar solver final   error norm_2 = " << norm_2(final_error  ) << std::endl;

  //// Dump the values to file
  //print_vec_to_file(output_prefix + "_initial_parameters.csv", initial_params);
  //print_vec_to_file(output_prefix + "_final_parameters.csv",   final_params);
  //print_vec_to_file(output_prefix + "_initial_projected.csv", initial_projected);
  //print_vec_to_file(output_prefix + "_final_projected.csv",   final_projected);
  //print_vec_to_file(output_prefix + "_initial_error.csv",     initial_error);
  //print_vec_to_file(output_prefix + "_final_error.csv",       final_error);

  //// Also add the results to the log
  //VW_OUT(VerboseDebugMessage, "math") << "LM: starting proj  " << initial_projected << std::endl;
  //VW_OUT(VerboseDebugMessage, "math") << "LM: final    proj  " << final_projected   << std::endl;
  //VW_OUT(VerboseDebugMessage, "math") << "LM: starting error " << initial_error     << std::endl;
  //VW_OUT(VerboseDebugMessage, "math") << "LM: final    error " << final_error       << std::endl;
}

/// Computes a system solution from a seed and returns the final error number.
int find_solution_from_seed(RpcSolveLMA    const& lma_model,
                            Vector<double> const& seed_params,
                            Vector<double> const& actual_observations,
                            Vector<double>      & final_params,
                            double              & norm_error) {

    // Initialize a zero vector of RPC model coefficients
    int status;

    // Use the L-M solver to optimize the RPC model coefficient values.
    const double abs_tolerance  = 1e-24;
    const double rel_tolerance  = 1e-24;
    const int    max_iterations = 2000;
    final_params = math::levenberg_marquardt( lma_model, seed_params, actual_observations, status,
                                              abs_tolerance, rel_tolerance, max_iterations );

    if (status < 1) { // This means the solver failed to converge!
      VW_OUT(DebugMessage, "math") << "rpc_gen: WARNING --> levmar solver status = " << status << std::endl;
    }

    // Otherwise the solver converged, return the final error number.
    Vector<double> final_projected = lma_model(final_params);
    Vector<double> final_error     = lma_model.difference(final_projected, actual_observations);
    norm_error = norm_2(final_error);
    return status;
}


void compute_scale_factors(vw::BBox3   const& gdc_box, 
                           vw::Vector2 const& image_size,
                           Vector3 &llh_scale, Vector3 &llh_offset,
                           Vector2 &uv_scale,  Vector2 &uv_offset) {

  Vector3 min_llh_coord = gdc_box.min();
  Vector3 max_llh_coord = gdc_box.max();

  // Use matched axis scaling for pixels so one axis does not get higher error weighting
  llh_scale  = (max_llh_coord - min_llh_coord)/2.0; // half range
  llh_offset = (max_llh_coord + min_llh_coord)/2.0; // center point
  double pixel_max = vw::math::max(image_size);
  uv_scale  = Vector2(pixel_max/2.0, pixel_max/2.0); // The long axis pixel is scaled to 1.0
  uv_offset = image_size/2.0; // center point 
}
  

/// Loads in the points from a text file
void load_pairs_from_file(std::string const& path,
                          Vector<double> &normalized_geodetics,
                          Vector<double> &normalized_pixels,
                          Vector3 &llh_scale, Vector3 &llh_offset,
                          Vector2 &uv_scale,  Vector2 &uv_offset) {
  // For now this only loads files from Geocam Space.
  
  const double HEIGHT = 0; // Might want to change this!
  
  vw_out() << "Reading point pairs from file...\n";
  
  // Read through the file and extract all the points
  
  std::ifstream infile(path.c_str());
  
  std::vector<Vector3> gdc_temp;
  std::vector<Vector2> pixel_temp;
  
  vw::BBox2 pixel_bbox;
  vw::BBox3 gdc_bbox;
  
  std::string line;
  while (std::getline(infile, line)) {
  
    size_t colon_pos = line.find(':');
    size_t start_pos = colon_pos + 1;
    size_t comma_pos = line.find(',');
    size_t num1_len = comma_pos - start_pos;
    float num1 = atof(line.substr(start_pos, num1_len).c_str());
    float num2 = atof(line.substr(comma_pos+1).c_str());    
    //printf("Read %lf, %lf\n", num1, num2);
  
    if (line[0] == 'u') { // = GDC
      Vector3 gdc(num1, num2, HEIGHT);
      gdc_temp.push_back(gdc);
      gdc_bbox.grow(gdc);
    }else { // v = pixel
      Vector2 pixel(num1, num2);
      pixel_temp.push_back(pixel);
      pixel_bbox.grow(pixel);
    }
  } // End loop through file
  
  // Check that we loaded the correct number of points
  size_t num_points = gdc_temp.size();
  if (pixel_temp.size() != num_points)
    vw_throw( ArgumentErr() << "Error reading input file: counts\n");
  
  // Automatically compute the scale factors
  
  compute_scale_factors(gdc_bbox, pixel_bbox.size(), llh_scale, llh_offset, uv_scale, uv_offset);
  
  // Currently the height is hardcoded to zero, resulting in zero scale and divide by zero!
  llh_scale [2] = 1.0;
  llh_offset[2] = 0.0;
  
  // Initialize normalized data storage
  normalized_geodetics.set_size(RPCModel::GEODETIC_COORD_SIZE*num_points);
  normalized_pixels.set_size   (RPCModel::IMAGE_COORD_SIZE   *num_points + RpcSolveLMA::NUM_PENALTY_TERMS);
  for (size_t i = 0; i < normalized_pixels.size(); i++)
    normalized_pixels[i] = 0.0; // Important: The extra penalty terms are all set to zero here.
      
  for (size_t i=0; i<num_points; ++i) {
  
    // Normalize the points
    Vector3 gdc   = elem_quot(gdc_temp  [i] - llh_offset, llh_scale);
    Vector2 pixel = elem_quot(pixel_temp[i] - uv_offset,  uv_scale );
  
    // Pack them in the output vectors
    subvector(normalized_geodetics, RPCModel::GEODETIC_COORD_SIZE*i, RPCModel::GEODETIC_COORD_SIZE) = gdc;
    subvector(normalized_pixels,    RPCModel::IMAGE_COORD_SIZE   *i, RPCModel::IMAGE_COORD_SIZE   ) = pixel;
   
  }
  vw_out() << "Loaded "<< num_points <<" processing point pairs from file\n";
}

/// Generates the set of GDC/pixel pairs that will be fed into the solver.
void generate_point_pairs(RPC_gen_Options opt,
                          Vector<double> &normalized_geodetics,
                          Vector<double> &normalized_pixels,
                          Vector3 &llh_scale, Vector3 &llh_offset,
                          Vector2 &uv_scale,  Vector2 &uv_offset) {

    // If the user provided a text file instead of a camera model, load the points
    // straight from the file.
    if ( (opt.camera_model.find(".csv") != std::string::npos) ||
         (opt.camera_model.find(".txt") != std::string::npos)   ) {
      load_pairs_from_file(opt.camera_model, normalized_geodetics, normalized_pixels,
                          llh_scale, llh_offset, uv_scale, uv_offset);
      return;
    }
    
    // Otherwise use the camera model information to generate the point pairs.

    // Load up the Digital Globe camera model from the camera file
    XMLPlatformUtils::Initialize();

    // Load the DG camera model. The API is kind of ugly.
    std::string session_name = "DG";
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session(asp::StereoSessionFactory::create
                       (session_name, opt, "", "", opt.camera_model, opt.camera_model, ""));
    boost::shared_ptr<camera::CameraModel> cam_dg(session->camera_model("", opt.camera_model));

    // Load up the RPC camera model from the camera file
    RPCXML xml;
    xml.read_from_file( opt.camera_model );
    boost::shared_ptr<RPCModel> cam_rpc( new RPCModel( *xml.rpc_ptr() ) );
    Vector2 image_size = xml_image_size(opt.camera_model);

    // Normalization in the lon-lat-height and pixel domains
    // - Compute an offset and scale to place all values in the
    //   zero to one range centered on the center coordinate/pixel.
    // - The input RPC model has similar numbers loaded but we will use different
    //   values unless the input options exactly match the input XML file.
    Vector3 min_llh_coord = opt.lon_lat_height_box.min();
    Vector3 max_llh_coord = opt.lon_lat_height_box.max();

    // Use matched axis scaling for pixels so one axis does not get higher error weighting
    llh_scale  = (max_llh_coord - min_llh_coord)/2.0; // half range
    llh_offset = (max_llh_coord + min_llh_coord)/2.0; // center point
    double pixel_max = vw::math::max(image_size);
    uv_scale  = Vector2(pixel_max/2.0, pixel_max/2.0); // The long axis pixel is scaled to 1.0
    uv_offset = image_size/2.0; // center point

    // Number of points in x and y at which we will optimize the RPC
    // model. Using 10 or 20 points gives roughly similar results.
    // 20 points result in 20^3 input data for optimization, with the
    // number of variable to optimize being just 78.
    int num_pts = 20; // The number of points per axis
    int num_total_pts = num_pts*num_pts*num_pts;

    // Initialize normalized data storage
    normalized_geodetics.set_size(RPCModel::GEODETIC_COORD_SIZE*num_total_pts);
    normalized_pixels.set_size(RPCModel::IMAGE_COORD_SIZE*num_total_pts + RpcSolveLMA::NUM_PENALTY_TERMS);
    for (size_t i = 0; i < normalized_pixels.size(); i++)
      normalized_pixels[i] = 0.0; // Important: The extra penalty terms are all set to zero here.

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
          //Vector2 pxg = cam_rpc->point_to_pixel(P); // DEBUG -> Try fitting to RPC!

          // Normalize the pixel to -1 <> 1 range
          Vector2 pxn = elem_quot(pxg - uv_offset, uv_scale);

          // It is a useful exercise to compare DG and RPC cameras
          //Vector2 pxr = cam_rpc->point_to_pixel(P);
          //std::cout << U << ' ' << P << ' ' << pxg << ' ' << pxr  << ' '
          //          << norm_2(pxg-pxr)<< std::endl;

          subvector(normalized_geodetics, RPCModel::GEODETIC_COORD_SIZE*count, RPCModel::GEODETIC_COORD_SIZE) = U;
          subvector(normalized_pixels,    RPCModel::IMAGE_COORD_SIZE   *count, RPCModel::IMAGE_COORD_SIZE   ) = pxn;
          count++;

        } // End z loop
      } // End y loop
    } // End x loop

}


int main( int argc, char* argv[] ) {

  RPC_gen_Options opt;
 // try {
    handle_arguments( argc, argv, opt );

    VW_ASSERT( opt.penalty_weight >= 0, ArgumentErr() << "The RPC penalty weight must be non-negative.\n" );

    // Generate all the point pairs using the input options
    Vector<double> normalized_geodetics;
    Vector<double> normalized_pixels;
    Vector3 llh_scale, llh_offset;
    Vector2 uv_scale,  uv_offset;
    generate_point_pairs(opt, normalized_geodetics, normalized_pixels,
                         llh_scale, llh_offset, uv_scale, uv_offset);

    double penalty_weight_fraction = opt.penalty_weight; // The percentage of the error that the penalty weights should represent
    double native_penalty_fraction = (double)RpcSolveLMA::NUM_PENALTY_TERMS / (double)normalized_pixels.size(); // Fraction with no adjustment
    double penalty_adjustment      = penalty_weight_fraction / native_penalty_fraction;

    VW_OUT(DebugMessage, "math") << "rpc_gen: Computed penalty weight: " << penalty_adjustment<< std::endl;

    // Initialize a specialized least squares solver object and load the input data
    RpcSolveLMA lma_model (normalized_geodetics, normalized_pixels, penalty_adjustment);

    int status;
    Vector<double> solution;
    double norm_error;

    // Initialize a zero vector of RPC model coefficients
    Vector<double> startZero;
    startZero.set_size(RPCModel::NUM_RPC_COEFFS);
    for (size_t i = 0; i < startZero.size(); i++)
      startZero[i] = 0.0;

    // Use the L-M solver to optimize the RPC model coefficient values.
    //VW_OUT(DebugMessage, "math") << "rpc_gen: Solving with zero seed" << std::endl;
    status = find_solution_from_seed(lma_model, startZero, normalized_pixels, solution, norm_error);
    VW_OUT(DebugMessage, "math") << "rpc_gen: norm_error = " << norm_error << std::endl;

    // If we ever want to improve our results further we should experiment with multiple starting seeds!

    // Dump all the results to disk if the user passed in an output prefix.
    if (opt.output_prefix != "")
      write_levmar_solver_results(opt.output_prefix, status, startZero, solution, normalized_pixels, lma_model);

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

 // } ASP_STANDARD_CATCHES;

  return 0;
}
