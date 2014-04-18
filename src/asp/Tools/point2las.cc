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


/// \file point2las.cc
///

#include <fstream>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <liblas/liblas.hpp>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Tools/point2dem.h> // We share common functions with point2dem

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>

using namespace vw;
namespace po = boost::program_options;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : asp::BaseOptions {
  // Input
  std::string pointcloud_filename;
  bool compressed;
  // Output
  std::string out_prefix;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("compressed,c", "Compress using laszip.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.pointcloud_filename), "Input Point Cloud");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::string usage("[options] <point-cloud>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.pointcloud_filename.empty() )
    vw_throw( ArgumentErr() << "Missing point cloud.\n"
              << usage << general_options );

  if ( opt.out_prefix.empty() )
    opt.out_prefix =
      asp::prefix_from_filename( opt.pointcloud_filename );

  // Create the output directory 
  asp::create_out_dir(opt.out_prefix);
  
  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  opt.compressed = vm.count("compressed");

}

int main( int argc, char *argv[] ) {

  // To do: need to understand what is the optimal strategy for
  // traversing the input point cloud file to minimize the reading
  // time.

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    ImageViewRef<Vector3> point_image = asp::read_cloud<3>(opt.pointcloud_filename);
    BBox3 cloud_bbox = pointcloud_bbox(point_image);

    // The las format stores the values as 32 bit integers. So, for a
    // given point, we store round((point-offset)/scale), as well as
    // the offset and scale values. Here we decide the values for
    // offset and scale to lose minimum amount of precision. We make
    // the scale almost as large as it can be without causing integer
    // overflow.
    Vector3 offset = (cloud_bbox.min() + cloud_bbox.max())/2.0;
    double maxInt = std::numeric_limits<int32>::max();
    maxInt *= 0.95; // Just in case stay a bit away
    Vector3 scale = cloud_bbox.size()/(2.0*maxInt);
    for (size_t i = 0; i < scale.size(); i++){
      if (scale[i] <= 0.0) scale[i] = 1.0e-16; // avoid degeneracy
    }

    liblas::Header header;
    // The line below causes trouble with compression in libLAS-1.7.0.
    //header.SetDataFormatId(liblas::ePointFormat1);
    header.SetScale(scale[0], scale[1], scale[2]);
    header.SetOffset(offset[0], offset[1], offset[2]);

    std::string lasFile;
    header.SetCompressed(opt.compressed);
    if (opt.compressed){
      lasFile = opt.out_prefix + ".laz";
    }else{
      lasFile = opt.out_prefix + ".las";
    }

    vw_out() << "Writing LAS file: " << lasFile + "\n";
    std::ofstream ofs;
    ofs.open(lasFile.c_str(), std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);

    TerminalProgressCallback progress_bar("asp","LAS: ");
    for (int row = 0; row < point_image.rows(); row++){
      progress_bar.report_fractional_progress(row, point_image.rows());
      for (int col = 0; col < point_image.cols(); col++){

        Vector3 point = point_image(col, row);
        if ( point == Vector3() ) continue; // skip no-data points

#if 0
        // For comparison later with las2txt.
        std::cout.precision(16);
        std::cout << "\npoint " << point[0] << ' ' << point[1] << ' '
                  << point[2] << std::endl;
#endif

        point = round( elem_quot((point - offset), scale) );

        liblas::Point las_point;
        las_point.SetCoordinates(point[0], point[1], point[2]);
        writer.WritePoint(las_point);

      }
    }
    progress_bar.report_finished();

  } ASP_STANDARD_CATCHES;

  return 0;
}
