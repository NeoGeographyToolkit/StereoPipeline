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


/// \file xyzi2csv.cc
///
// Convert a binary file in xyzi format to csv as lon,lat,
// height_above_datum_meters. Input data format: binary double
// precision floating point table with 4 columns: X, Y, Z, RDRid where
// X,Y,Z are the polar stereographic coordinates (in km) and RDRid is
// the LOLA RDR ID for each point with the format: YYDOYHHMM.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;

struct Options : public vw::GdalWriteOptions {
  std::string xyzi, dem, csv;
};
    
void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add_options()
    ("xyzi",  po::value(&opt.xyzi), "Input xyzi file.")
    ("dem",  po::value(&opt.dem), "DEM from which to read the projection used for xyzi.")
    ("csv",  po::value(&opt.csv), "Output csv file.")
    ;
    
  general_options.add(vw::GdalWriteOptionsDescription(opt));
    
  po::options_description positional("");
  
  po::positional_options_description positional_desc;

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.xyzi == "")
    vw_throw(ArgumentErr() << "Must specify the input xyzi file.\n");
    
  if (opt.dem == "")
    vw_throw(ArgumentErr() << "Must specify the input DEM file.\n");

  if (opt.csv == "") 
    vw_throw(ArgumentErr() << "Must specify the output csv file.\n");

  // Create the output directory
  vw::create_out_dir(opt.csv);

  return;
}

void run_xyzi2csv(int argc, char* argv[]) {

  // Parse arguments and perform validation
  Options opt;
  handle_arguments(argc, argv, opt);

  vw::cartography::GeoReference georef;
  bool has_georef = vw::cartography::read_georeference(georef, opt.dem);
  if (!has_georef)
    vw_throw(ArgumentErr() << "There is no georeference information in: "
              << opt.dem << ".\n");

  std::cout.precision(17);
  // Read the binary file
  std::ifstream ifs(opt.xyzi, std::ios::binary);

  std::cout << "--try to read" << std::endl;
  vw_out() << "Writing: " << opt.csv << "\n";
  std::ofstream ofs(opt.csv);
  ofs.precision(17);
  
  double v[4];
  while (1) {
    ifs.read(reinterpret_cast<char*>(&v[0]), 4 * sizeof(double));
    if (!ifs)
      break;
    
    Vector2 lonlat = georef.point_to_lonlat(Vector2(v[0] * 1000.0, v[1] * 1000.0));
    ofs << lonlat[0] << ", " << lonlat[1] << ", " << 1000.0 * v[2] << "\n";
  } 
    
}

int main(int argc, char* argv[]) {

  try {
    run_xyzi2csv(argc, argv);
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
