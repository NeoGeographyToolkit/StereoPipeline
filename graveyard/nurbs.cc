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


/// \file nurbs.cc
///

#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/vw.h>
using namespace vw;

#include "SurfaceNURBS.h"

using namespace vw;

int main( int argc, char *argv[] ) {

  std::string input_file_name, output_file_name;
  unsigned int num_iterations;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("input-file", po::value<std::string>(&input_file_name), "Explicitly specify the input file")
    ("output-file,o", po::value<std::string>(&output_file_name)->default_value("output.png"), "Specify the output file")
    ("iterations,i", po::value<unsigned int>(&num_iterations), "The number of iterations to perform")
    ("holes-only,h", "Only fill areas with no pixels (zero alpha) with interpolated data.")
    ("normalize", "Normalize the output image")
    ("verbose", "Verbose output");
  po::positional_options_description p;
  p.add("input-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  if( vm.count("help") ) {
    std::cout << desc << std::endl;
    return 1;
  }

  if( vm.count("input-file") != 1 ) {
    std::cout << "Error: Must specify exactly one input file!" << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  if( vm.count("verbose") ) {
    set_debug_level(VerboseDebugMessage);
  }

  try {
    ImageView<PixelGrayA<float> > image;
    read_image( image, input_file_name );

    ImageView<PixelGrayA<float> > output;
    output = MBASurfaceNURBS(image, num_iterations);
    
    if (vm.count("holes-only") ) {
      for (int j = 0; j < image.rows(); ++j) {
        for (int i = 0; i < image.cols(); ++i) {
          if (image(i,j).a() != 0) {
            output(i,j) = image(i,j);
          }
        }
      }
    }

    if( vm.count("normalize") ) {
      output = normalize(output);
    }

    write_image( output_file_name, output );
  }
  catch( Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  return 0;
}

