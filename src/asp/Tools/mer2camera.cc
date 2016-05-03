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


#include <vw/Camera/CAHVModel.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/CAHVOREModel.h>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
namespace fs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/foreach.hpp>

using namespace vw;

struct Options : public vw::cartography::GdalWriteOptions {
  // Input
  std::string img_file;

  // Output
  std::string output_prefix;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o", po::value(&opt.output_prefix), "Output prefix for new control network.");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("img-file", po::value(&opt.img_file) );

  po::positional_options_description positional_desc;
  positional_desc.add("img-file", 1);

  std::string usage("[options] <img_file>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.img_file.empty() )
    vw_throw( ArgumentErr() << "Missing required input file.\n"
              << usage << general_options );
  if ( opt.output_prefix.empty() )
    opt.output_prefix = fs::path(opt.img_file).stem().string();
}

int main( int argc, char** argv ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    std::string camera_type;
    std::vector<std::string> camera_params;
    { // Extracting important keys I need
      std::ifstream image( opt.img_file.c_str(), std::ios::in );
      std::list<std::string> objects;

      std::string line;
      while ( image.good() ) {
        std::getline(image,line);
        if ( line.empty() )
          continue; // Skip blank lines
        if ( line[0] == '/' && line[1] == '*' )
          continue; // Skip comments

        // Splitting
        std::vector<std::string> tokens;
        boost::split( tokens, line, boost::is_any_of("="),
                      boost::token_compress_on );
        if ( tokens.size() != 2 )
          continue; // Handle only easy tokens
        boost::trim(tokens[0]);
        boost::trim(tokens[1]);
        boost::to_lower(tokens[0]);
        boost::to_lower(tokens[1]);
        if ( tokens[0] == "group" ) {
          objects.push_back(tokens[1]);
          continue;
        } else if ( tokens[0] == "end_group" ) {
          if ( objects.back() != tokens[1] )
            vw_throw( IOErr() << "Object parsing error!" );
          if ( objects.back() == "geometric_camera_model" )
            break; // I don't care to read anymore
          objects.pop_back();
          continue;
        }

        if ( objects.empty() )
          continue; // Labels not in a group is useless

        // For now .. only extracting camera model. However we could pull more.
        if ( objects.back() == "geometric_camera_model" ) {
          if ( tokens[0] == "model_type" )
            camera_type = tokens[1];
          if ( boost::starts_with(tokens[0],"model_component_") &&
               boost::is_digit()(tokens[0][tokens[0].size()-1]) )
            camera_params.push_back( tokens[1] );
        }
      }
      image.close();
    }

    vw_out() << "Found camera type: " << camera_type << std::endl;
    if ( camera_type != "cahv" && camera_type != "cahvor" &&
         camera_type != "cahvore" )
      vw_throw( IOErr() << "Unknown camera type: " << camera_type << "\n" );

    // Convert the camera properties to Vector3
    std::vector<Vector3> camera_vectors;
    BOOST_FOREACH( std::string const& str, camera_params ) {
      std::vector<std::string> tokens;
      boost::split( tokens, str, boost::is_any_of("(), "),
                    boost::token_compress_on );

      // Hack to remove starting empty token. I don't understand why
      // this happens.
      if ( tokens[0].empty() )
        tokens.erase(tokens.begin());
      if ( tokens[tokens.size()-1].empty() )
        tokens.erase(tokens.begin()+(tokens.size()-1));

      if ( tokens.size() == 3 ) {
        Vector3 temp;
        temp[0] = boost::lexical_cast<double>(tokens[0]);
        temp[1] = boost::lexical_cast<double>(tokens[1]);
        temp[2] = boost::lexical_cast<double>(tokens[2]);
        camera_vectors.push_back(temp);
      } else if ( tokens.size() == 1 ) {
        camera_vectors.push_back(Vector3(boost::lexical_cast<double>(tokens[0]),0,0));
      } else {
        vw_throw( IOErr() << "Error parsing vector: " << str << "\n" );
      }
    }

    // Writing out our results
    if ( camera_type == "cahv" ) {
      camera::CAHVModel camera( camera_vectors[0],
                                camera_vectors[1],
                                camera_vectors[2],
                                camera_vectors[3] );
      camera.write( opt.output_prefix + ".cahv" );
    } else if ( camera_type == "cahvor" ) {
      camera::CAHVORModel camera( camera_vectors[0],
                                  camera_vectors[1],
                                  camera_vectors[2],
                                  camera_vectors[3],
                                  camera_vectors[4],
                                  camera_vectors[5] );
      camera.write( opt.output_prefix + ".cahvor" );
    } else if ( camera_type == "cahvore" ) {
      camera::CAHVOREModel camera( camera_vectors[0],
                                   camera_vectors[1],
                                   camera_vectors[2],
                                   camera_vectors[3],
                                   camera_vectors[4],
                                   camera_vectors[5],
                                   camera_vectors[6],
                                   camera_vectors[8][0]);
      camera.write( opt.output_prefix + ".cahvore" );
    }
  } ASP_STANDARD_CATCHES;

  return 0;
}
