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
  bool compress;
  // Output
  std::string out_prefix;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("compress,c", "Compress using laszip.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.");
  
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

  opt.compress = vm.count("compress");

}

int main( int argc, char *argv[] ) {

  // To do: Test the performance when switching from rows to cols.
  // To do: Deal with the issue of scale and shift when converting
  //        to las to not lose precision.
  
  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    
    DiskImageView<Vector4> point_disk_image(opt.pointcloud_filename);

    // Find the bounding box of the points
    double big = std::numeric_limits<double>::max(); 
    Vector3 point_min = Vector3(big, big, big), point_max = -point_min;
    for (int row = 0; row < point_disk_image.rows(); row++){
      for (int col = 0; col < point_disk_image.cols(); col++){
        
        Vector3 const& point = subvector(point_disk_image(row, col), 0, 3);

        // Ignore no-data points
        if ( point == Vector3() ) continue;

        for (int k = 0; k < 3; k++){
          if (point[k] < point_min[k]) point_min[k] = point[k];
          if (point[k] > point_max[k]) point_max[k] = point[k];
        }

      }
    }

    // The las format stores the values as 32 bit integers. It takes a
    // double value x and stores round((x-offset)/scale). Here we
    // decide the values for offset and scale to lose minimum amount
    // of precision. We make the scale almost as large as it can be
    // without causing integer overflow.
    Vector3 offset = (point_min + point_max)/2.0;
    double maxInt = std::numeric_limits<int32>::max();
    maxInt *= 0.95; // Just in case stay a bit away 
    Vector3 scale = (point_max - point_min)/(2.0*maxInt);

    liblas::Header header;
    header.SetDataFormatId(liblas::ePointFormat1);
    header.SetScale(scale[0], scale[1], scale[2]);
    header.SetOffset(offset[0], offset[1], offset[2]);

    std::string lasFile;
    header.SetCompressed(opt.compress);
    if (opt.compress){
      lasFile = opt.out_prefix + ".laz";
    }else{
      lasFile = opt.out_prefix + ".las";
    }

    std::ofstream ofs;
    ofs.open(lasFile.c_str(), std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);
    
    for (int row = 0; row < point_disk_image.rows(); row++){
      for (int col = 0; col < point_disk_image.cols(); col++){

        Vector3 const& point = subvector(point_disk_image(row, col), 0, 3);
        if ( point == Vector3() ){
          // Ignore no-data points
          continue;
        }
        
        liblas::Point las_point;
        las_point.SetCoordinates(point[0], point[1], point[2]);
        writer.WritePoint(las_point);
      }
    }
    
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
