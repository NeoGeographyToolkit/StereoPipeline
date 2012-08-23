// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file point2las.cc
///

#include <fstream>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <liblas/liblas.hpp>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>

using namespace vw;
namespace po = boost::program_options;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
}

struct Options : asp::BaseOptions {

  // Input
  std::string pointcloud_filename;
  
  // Output
  std::string  out_prefix; // output_file_type;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix");
  //("output-filetype,t", po::value(&opt.output_file_type)->default_value("las"),
  // "Specify the output file [las laz]");
  
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.pointcloud_filename), "Input Point Cloud");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::string usage("<point-cloud> ...");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage );

  if ( opt.pointcloud_filename.empty() )
    vw_throw( ArgumentErr() << "Missing point cloud.\n"
              << usage << general_options );
  
  if ( opt.out_prefix.empty() )
    opt.out_prefix =
      asp::prefix_from_filename( opt.pointcloud_filename );

}

int main( int argc, char *argv[] ) {

  // To do: Test the performance when switching from rows to cols.
  // To do: Deal with the issue of scale and shift when converting
  //        to las to not lose precision.
  
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    DiskImageView<Vector4> point_disk_image(opt.pointcloud_filename);

    liblas::Header header;
    header.SetDataFormatId(liblas::ePointFormat1);
    
    std::string lasFile =  opt.out_prefix + ".las";
    std::ofstream ofs;
    ofs.open(lasFile.c_str(), std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);

    for (int row = 0; row < point_disk_image.rows(); row++){
      for (int col = 0; col < point_disk_image.cols(); col++){
        Vector4 const& point = point_disk_image(row, col);
        liblas::Point las_point;
        las_point.SetCoordinates(point[0], point[1], point[2]);
        writer.WritePoint(las_point);
      }
    }
    
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
