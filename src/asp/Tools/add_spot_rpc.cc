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


/// Generate an approximate RPC model for a SPOT5 image.
/// - The resulting model is appended to the end of the existing
///   SPOT5 metadata file.
/// - These files can then be used as input for stereo with -t rpc.


#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Core/StringUtils.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageResourceRaw.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Camera/LinescanSpotModel.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPCModelGen.h>
#include <xercesc/util/PlatformUtils.hpp>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;
using asp::RPCModel;

struct Options : public vw::cartography::GdalWriteOptions {
  string input_path, output_path; 
  double min_height, max_height;
  int    num_samples;
  double penalty_weight;
  Options(): min_height(-1), max_height(-1), num_samples(-1), penalty_weight(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-path,o",   po::value(&opt.output_path), "Specify the output path.")
    ("min-height",     po::value(&opt.min_height)->default_value(0),
     "The minimum height (in meters) above the WGS84 datum of the simulation box in which to compute the RPC approximation.")
    ("max-height",     po::value(&opt.max_height)->default_value(8000),
     "The maximum height (in meters) above the WGS84 datum of the simulation box in which to compute the RPC approximation.")
    ("num-samples",     po::value(&opt.num_samples)->default_value(100),
     "How many samples to use between the minimum and maximum heights.")
    ("penalty-weight",     po::value(&opt.penalty_weight)->default_value(0.1),
     "Penalty weight to use to keep the higher-order RPC coefficients small. Higher penalty weight results in smaller such coefficients.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input_path", po::value(&opt.input_path), "A SPOT5 Metadata file.");

  po::positional_options_description positional_desc;
  positional_desc.add("input_path", 1);

  string usage("<input path>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
			    positional, positional_desc, usage,
			    allow_unregistered, unregistered);
  
  if ( opt.input_path.empty() )
    vw_throw( ArgumentErr() << "Missing input path.\n" << usage << general_options );

  if ( opt.output_path.empty() ) {
    vw_out() << "Output path not provided, appending RPC model to the input metadata file.\n";
    opt.output_path = opt.input_path;
    
  }
}


/// Generates the set of GDC/pixel pairs that will be fed into the solver.
void generate_point_pairs(Options opt,
                          Vector<double> &normalized_geodetics,
                          Vector<double> &normalized_pixels,
                          Vector3 &llh_scale, Vector3 &llh_offset,
                          Vector2 &uv_scale,  Vector2 &uv_offset) {

    // Load up the camera model from the camera file
    xercesc::XMLPlatformUtils::Initialize();

    // Load the input camera model
    boost::shared_ptr<camera::CameraModel> cam_ptr
      = asp::load_spot5_camera_model_from_xml(opt.input_path);

    // Load some image info
    vw::ImageFormat format     = vw::DiskImageResourceRaw::image_format_from_spot5_DIM(opt.input_path);
    Vector2         image_size = Vector2(format.cols, format.rows);

    // Will have to change this if any SPOT5 data uses a different datum.
    vw::cartography::Datum datum("WGS84");

    // Only generate point pairs up to this far from the valid boundaries.
    // - There is no guarantee that a point right on the edge will safely project!
    const double CONTRACTION = 0.10;

    // Load the estimated image bounds from the XML file!
    std::vector<vw::Vector2> lonlat_corners = asp::SpotXML::get_lonlat_corners(opt.input_path);
    if (lonlat_corners.size() != 4)
      vw::vw_throw(ArgumentErr() << "Failed to parse lonlat corners of metadata file!");

    Vector2 top_left  = lonlat_corners[0]; // The corners should be loaded in this order.
    Vector2 top_right = lonlat_corners[1];
    //Vector2 bot_right = lonlat_corners[2];
    Vector2 bot_left  = lonlat_corners[3];
    
    // These vectors are aligned with the image projected on to the ground and are 
    //  used to iterate through the coverage region of the image.
    Vector2 col_axis = top_right - top_left;
    Vector2 row_axis = bot_left  - top_left;
    double  height_range = opt.max_height - opt.min_height;
    
    // Get a bounding box of the covered region (not all of the BBox has image coverage!)
    BBox2 bounding_box;
    for (size_t i=0; i<4; ++i)
      bounding_box.grow(lonlat_corners[i]);
    Vector3 min_llh_coord = Vector3(bounding_box.min()[0], bounding_box.min()[1], opt.min_height);
    Vector3 max_llh_coord = Vector3(bounding_box.max()[0], bounding_box.max()[1], opt.max_height);
    vw_out() << "Min lon/lat/height coord: " << min_llh_coord << std::endl;
    vw_out() << "Max lon/lat/height coord: " << max_llh_coord << std::endl;

    // Compute scale factors to describe the bounding box (aligned with ENU coordinate system)
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
    vw_out() << "Attempting to project " << num_total_pts << " locations...\n";

    // Initialize normalized data storage
    normalized_geodetics.set_size(RPCModel::GEODETIC_COORD_SIZE*num_total_pts);
    normalized_pixels.set_size(RPCModel::IMAGE_COORD_SIZE*num_total_pts
                               + asp::RpcSolveLMA::NUM_PENALTY_TERMS);
    for (size_t i = 0; i < normalized_pixels.size(); i++) {
      // Important: The extra penalty terms are all set to zero here.
      normalized_pixels[i] = 0.0; 
    }
    
    // Loop through the coverage area of the sattelite and generate pairs of 
    // normalized image/GDC training data using the SPOT5 camera model.
    int success_count = 0;
    int fail_count    = 0;
    for (int x = 0; x < num_pts; x++){
      for (int y = 0; y < num_pts; y++){
        for (int z = 0; z < num_pts; z++){

          // This is the test point location in our image normalized to 0 <> 1 range
          Vector3 u( x/(num_pts - 1.0),
                     y/(num_pts - 1.0),
                     z/(num_pts - 1.0) );
          // Shrink the point a little bit so we don't go all the way up to the valid boundaries.
          u = elem_sum(elem_prod(u,1.0-CONTRACTION), (CONTRACTION/2.0));

          // Obtain the lat/lon/height of this location
          Vector2 lonlat = top_left + u[0]*col_axis + u[1]*row_axis;
          Vector3 G      = Vector3(lonlat[0], lonlat[1], u[2]*height_range + opt.min_height);

          // Convert the actual lonlat point to vector U in the -1 <> 1 range.
          // - This vector is in a north-aligned coordinate system.
          // - RPC expects inputs in this range so it can use them with the scaling values we computed.
          Vector3 U = elem_quot((G - llh_offset), llh_scale);

          // Convert from geodetic to geocentric coordinates
          Vector3 P = datum.geodetic_to_cartesian(G); // xyz

          Vector2 pxg;
          try {
            // Project the GCC coordinate into the SPOT5 camera model
            pxg = cam_ptr->point_to_pixel(P);
            //vw_out() << "SUCESS location " << G << " === " << u << " === " << U << " into camera!  Skipping it.\n";
          } catch (vw::camera::PointToPixelErr) {
            //vw_out() << "Failed to project location " << G << " === " << u << " === " << U << " into camera!  Skipping it.\n";
            ++fail_count;
            continue;
          }

          // Normalize the pixel to -1 <> 1 range
          Vector2 pxn = elem_quot(pxg - uv_offset, uv_scale);

          subvector(normalized_geodetics, RPCModel::GEODETIC_COORD_SIZE*success_count,
                    RPCModel::GEODETIC_COORD_SIZE) = U;
          subvector(normalized_pixels,    RPCModel::IMAGE_COORD_SIZE   *success_count,
                    RPCModel::IMAGE_COORD_SIZE   ) = pxn;
          success_count++;

        } // End z loop
      } // End y loop
    } // End x loop

    vw_out() << "Successfully projected " << success_count << " locations.\n";
    
    // Update the sizes to reflect the number of successful point projections
    const bool preserve = true;
    normalized_geodetics.set_size(RPCModel::GEODETIC_COORD_SIZE*success_count, preserve);
    normalized_pixels.set_size(RPCModel::IMAGE_COORD_SIZE*success_count
                               + asp::RpcSolveLMA::NUM_PENALTY_TERMS, preserve);
}


// Save an XML file having all RPC information
void save_xml(Vector3 const& llh_scale,
              Vector3 const& llh_offset,
              Vector2 const& pixel_scale,
              Vector2 const& pixel_offset,
              asp::RPCModel::CoeffVec const& line_num,
              asp::RPCModel::CoeffVec const& line_den,
              asp::RPCModel::CoeffVec const& samp_num,
              asp::RPCModel::CoeffVec const& samp_den,
              std::string const& out_cam_file){

  std::string lineoffset   = vw::num_to_str(pixel_offset.y());
  std::string sampoffset   = vw::num_to_str(pixel_offset.x());
  std::string latoffset    = vw::num_to_str(llh_offset.y());
  std::string longoffset   = vw::num_to_str(llh_offset.x());
  std::string heightoffset = vw::num_to_str(llh_offset.z());
  
  std::string linescale   = vw::num_to_str(pixel_scale.y());
  std::string sampscale   = vw::num_to_str(pixel_scale.x());
  std::string latscale    = vw::num_to_str(llh_scale.y());
  std::string longscale   = vw::num_to_str(llh_scale.x());
  std::string heightscale = vw::num_to_str(llh_scale.z());

  std::string linenumcoef = vw::vec_to_str(line_num);
  std::string linedencoef = vw::vec_to_str(line_den);
  std::string sampnumcoef = vw::vec_to_str(samp_num);
  std::string sampdencoef = vw::vec_to_str(samp_den);

  const std::string RPC_START_STR = "<isd>";
  const std::string DIMAP_END_STR = "</Dimap_Document>";
  bool dimap_file = false;
  
  vw_out() << "Writing: " << out_cam_file << std::endl;
  // Try to open the file for read/write, if that fails just try
  // to open it for writing.
  std::fstream ofs;
  ofs.open(out_cam_file.c_str(), std::fstream::in | std::fstream::out);
  if (ofs.good()) {
    // Navigate to the either the end of the file or 
    //  the correct location to insert the RPC info
    std::string line;
    while (std::getline(ofs, line)) {
      if (line.find("Dimap_Document") != std::string::npos)
        dimap_file = true;
      if (line.find(RPC_START_STR) != std::string::npos) {
        ofs.seekg(-1*(RPC_START_STR.size()+1), std::ios_base::cur);
        break;
      }
      if (line.find(DIMAP_END_STR) != std::string::npos) {
        ofs.seekg(-1*(DIMAP_END_STR.size()+1), std::ios_base::cur);
        break;
      }
    }
  } else { // The file did not exist, create a new one.
    ofs.open(out_cam_file.c_str(), std::fstream::out);
  }
  if (!ofs.good())
    vw_throw( ArgumentErr() << "Error writing to file:" << out_cam_file);

  // Write RPC to file in Digital Globe format  
  ofs << "<isd>\n";
  ofs << "    <RPB>\n";
  ofs << "        <SATID>SPOT5_PAN</SATID>\n";
  ofs << "        <IMAGE>\n";
  ofs << "            <LINEOFFSET>"      << lineoffset   << "</LINEOFFSET>\n";
  ofs << "            <SAMPOFFSET>"      << sampoffset   << "</SAMPOFFSET>\n";
  ofs << "            <LATOFFSET>"       << latoffset    << "</LATOFFSET>\n";
  ofs << "            <LONGOFFSET>"      << longoffset   << "</LONGOFFSET>\n";
  ofs << "            <HEIGHTOFFSET>"    << heightoffset << "</HEIGHTOFFSET>\n";
  ofs << "            <LINESCALE>"       << linescale    << "</LINESCALE>\n";
  ofs << "            <SAMPSCALE>"       << sampscale    << "</SAMPSCALE>\n";
  ofs << "            <LATSCALE>"        << latscale     << "</LATSCALE>\n";
  ofs << "            <LONGSCALE>"       << longscale    << "</LONGSCALE>\n";
  ofs << "            <HEIGHTSCALE>"     << heightscale  << "</HEIGHTSCALE>\n";
  ofs << "            <LINENUMCOEFList>\n";
  ofs << "                <LINENUMCOEF>" << linenumcoef  << "</LINENUMCOEF>\n";
  ofs << "            </LINENUMCOEFList>\n";
  ofs << "            <LINEDENCOEFList>\n";
  ofs << "                <LINEDENCOEF>" << linedencoef  << "</LINEDENCOEF>\n";
  ofs << "            </LINEDENCOEFList>\n";
  ofs << "            <SAMPNUMCOEFList>\n";
  ofs << "                <SAMPNUMCOEF>" << sampnumcoef  << "</SAMPNUMCOEF>\n";
  ofs << "            </SAMPNUMCOEFList>\n";
  ofs << "            <SAMPDENCOEFList>\n";
  ofs << "                <SAMPDENCOEF>" << sampdencoef  << "</SAMPDENCOEF>\n";
  ofs << "            </SAMPDENCOEFList>\n";
  ofs << "        </IMAGE>\n";
  ofs << "    </RPB>\n";
  ofs << "</isd>\n";
  if (dimap_file) // Close of DIMAP files properly
    ofs << DIMAP_END_STR;
  ofs.close();
}


int main( int argc, char *argv[] ) {

  Options opt;
  //try {
    handle_arguments(argc, argv, opt);
   
    // Generate all the point pairs using the input options
    Vector<double> normalized_geodetics;
    Vector<double> normalized_pixels;
    Vector3 llh_scale, llh_offset;
    Vector2 uv_scale,  uv_offset;
    generate_point_pairs(opt, normalized_geodetics, normalized_pixels,
                         llh_scale, llh_offset, uv_scale, uv_offset);

    // Find the RPC coefficients
    RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
    asp::gen_rpc(// Inputs
                 opt.penalty_weight,
                 "", // Only need to pass in an output prefix to log solver output
                 normalized_geodetics, normalized_pixels,  
                 llh_scale, llh_offset, uv_scale, uv_offset,
                 // Outputs
                 line_num, line_den, samp_num, samp_den);
   
    save_xml(llh_scale, llh_offset, uv_scale, uv_offset,  
             line_num, line_den, samp_num, samp_den,  
             opt.output_path);
    
  //} ASP_STANDARD_CATCHES;

  return 0;
}
