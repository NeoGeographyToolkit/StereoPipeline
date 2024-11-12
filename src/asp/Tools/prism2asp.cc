// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

// Parse PRISM data and produce CSM camera files

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/CsmModelFit.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Core/BitChecker.h>
#include <asp/Camera/XMLBase.h>
#include <asp/Camera/TimeProcessing.h>

#include <asp/Camera/PRISM_XML.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

// XML parsing
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <limits>
#include <cstring>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options: public vw::GdalWriteOptions {
  std::string dim_file, csm_file;
  
  // Constructor
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add_options()
    ("dim", po::value(&opt.dim_file)->default_value(""),
     "The input PRISM .DIMA file.")
    ("csm", po::value(&opt.csm_file)->default_value(""),
     "The output CSM camera file.")
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

  // The dim file is required
  if (opt.dim_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing the input .DIMA file.\n" 
                 << usage << general_options);

  // The output file is required
  if (opt.csm_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing the output CSM file.\n");
      
  // TODO(oalexan1): Add logic to log to file     

} // End function handle_arguments

int main(int argc, char * argv[]) {
    
  Options opt;
  try {
    
    // Mandatory initialization for Xerces
    xercesc::XMLPlatformUtils::Initialize();
    
    handle_arguments(argc, argv, opt);
    int ncols = -1, nrows = -1;
    double first_line_time = -1, last_line_time = -1;
    std::vector<vw::Vector3> positions, velocities, rph;
    std::vector<double> position_times, rph_times;
    asp::parsePrismXml(opt.dim_file, ncols, nrows, first_line_time, last_line_time,
                       positions, velocities, position_times, rph, rph_times);
    
  // Mandatory last line after invoking the xml reader  
  xercesc::XMLPlatformUtils::Terminate();
  
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
