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


// Boost
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/foreach.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Math/Vector.h>
#include <vw/FileIO.h>
#include <vw/Image/Algorithms.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
using namespace vw;

// Support for ISIS image files
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

struct Options : public asp::BaseOptions {
  std::vector<std::pair<std::string,Vector2> > input_files;
  std::string gcp_file;
  std::string output_file;
  Vector2i grid_size;
  size_t kernel_size;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("kernel-size",po::value(&opt.kernel_size)->default_value(25),
     "Size of the individual crops thats make up the output image")
    ("output,o",po::value(&opt.output_file),
     "Output file for this process. If blank it will produce output with extension \"-render.tif\".");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("gcp-file", po::value(&opt.gcp_file));

  po::positional_options_description positional_desc;
  positional_desc.add("gcp-file", 1);

  std::string usage("[options] <gcp file>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.gcp_file.empty() )
    vw_throw( ArgumentErr() << "Missing input GCP file!\n"
              << usage << general_options );
  if ( opt.output_file.empty() )
    opt.output_file =
      fs::path( opt.gcp_file ).stem().string()+"-render.tif";
}

int main( int argc, char *argv[] ) {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif 

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Open GCP file
    vw_out() << " -> Opening \"" << opt.gcp_file << "\".\n";
    std::ifstream ifile( opt.gcp_file.c_str() );
    if ( !ifile.is_open() )
      vw_throw( ArgumentErr() << "Unable to open GCP file!\n" );
    size_t count = 0;
    while (!ifile.eof()) {
      if ( count == 0 ) {
        // Don't really care about this line
        Vector3 eh, ei;
        ifile >> eh[0] >> eh[1] >> eh[2] >> ei[0] >> ei[1] >> ei[2];
      } else {
        std::string file_name;
        Vector2 location;
        ifile >> file_name >> location[0] >> location[1];
        opt.input_files.push_back( std::make_pair(file_name,location) );
      }
      count++;
    }
    ifile.close();

    // Find and locate input images
    vw_out() << " -> Checking input files.\n";
    typedef std::pair<std::string,Vector2> ElementType;
    BOOST_FOREACH( ElementType const& input, opt.input_files ) {
      if ( !fs::exists(input.first) )
        vw_throw( IOErr() << "Unable to open \"" << input.first << "\". Is it in the current working directory?" );
    }
    opt.grid_size[0] = ceil( sqrt( float(opt.input_files.size() ) ) );
    opt.grid_size[1] = ceil( float(opt.input_files.size())/float(opt.grid_size[0] ) );

    // Render image in memory
    vw_out() << " -> Rasterizing \"" << opt.output_file << "\" in memory.\n";
    ImageView<PixelGray<uint8> > output(opt.grid_size[0]*opt.kernel_size,
                                        opt.grid_size[1]*opt.kernel_size);
    count = 0;
    BOOST_FOREACH( ElementType const& input, opt.input_files ) {
      // Working out write location
      Vector2i index;
      index[0] = count - floor(float(count)/float(opt.grid_size[0]))*opt.grid_size[0];
      index[1] = floor(float(count)/float(opt.grid_size[0]));

      // Applying to image
      DiskImageView<PixelGray<float> > image( input.first );
      crop( output, index[0]*opt.kernel_size, index[1]*opt.kernel_size,
            opt.kernel_size, opt.kernel_size ) =
        channel_cast_rescale<uint8>(normalize(crop( image, input.second[0]-opt.kernel_size/2,
                                                    input.second[1]-opt.kernel_size/2,
                                                    opt.kernel_size, opt.kernel_size )));

      count++;
    }
    asp::write_gdal_image( opt.output_file, output, opt,
                           TerminalProgressCallback("asp","") );

  } ASP_STANDARD_CATCHES;

  return 0;
}
