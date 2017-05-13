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


/// \file s2p_fltr.cc
/// Filter the output disparity produced by S2P and create an F.tif file.
/// Not ready for general consumption!


#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>

#include <boost/filesystem.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::cartography;
using namespace std;

/// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector3             > { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<PixelMask<Vector2f> > { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

struct Options : vw::cartography::GdalWriteOptions {
  string input_dem, output_dem, output_datum, target_srs_string;
  double nodata_value;
  bool   use_double;
};

void handle_arguments( int argc, char *argv[], Options& opt ){

  po::options_description general_options("");
  general_options.add_options()
    ("output-datum", po::value(&opt.output_datum), " The datum to convert to. Supported options: WGS_1984, NAD83, WGS72, and NAD27.")
    ("t_srs",        po::value(&opt.target_srs_string)->default_value(""), "Specify the output datum via the PROJ.4 string.")
    ("nodata_value", po::value(&opt.nodata_value)->default_value(-32768),
     "The value of no-data pixels, unless specified in the DEM.")
    ("double", po::bool_switch(&opt.use_double)->default_value(false)->implicit_value(true),
     "Output using double precision (64 bit) instead of float (32 bit).");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-dem",    po::value(&opt.input_dem  ), "The path to the input DEM file.")
    ("output-dem",   po::value(&opt.output_dem ), "The path to the output DEM file.");

  po::positional_options_description positional_desc;
  positional_desc.add("input-dem",    1);
  positional_desc.add("output-dem",   1);

  string usage("[options] <input dem> <output dem>");
  bool allow_unregistered = false;
  vector<string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if ( opt.input_dem.empty() )
    vw_throw( ArgumentErr() << "Missing input arguments.\n\n"     << usage << general_options );
  if ( opt.output_dem.empty() )
    vw_throw( ArgumentErr() << "Requires <output dem> in order to proceed.\n\n"  << usage << general_options );
  if ( opt.output_datum.empty() && opt.target_srs_string.empty())
    vw_throw( ArgumentErr() << "Requires <output datum> or PROJ.4 string in order to proceed.\n\n" << usage << general_options );

  if ( !opt.output_datum.empty() && !opt.target_srs_string.empty())
    vw_out(WarningMessage) << "Both the output datum and the PROJ.4 string were specified. The former takes precedence.\n";

  boost::to_lower(opt.output_datum);

  vw::create_out_dir(opt.output_dem);

  //// Turn on logging to file
  //asp::log_to_file(argc, argv, "", opt.out_prefix);

}

// TODO: This tool is in need of a serious cleanup!
// Not ready for general usage.

int main( int argc, char *argv[] ) {

  try {

    Options opt;
    
    //handle_arguments( argc, argv, opt );

    std::string RD_file      = argv[1];
    std::string F_file       = argv[2];
    std::string disp_1d_file = argv[3];
    std::string mask_1d_file = argv[4];
    int left_crop_x          = atoi(argv[5]);
    int left_crop_y          = atoi(argv[6]);
    int right_crop_x         = atoi(argv[7]);
    int right_crop_y         = atoi(argv[8]);

    std::cout << "Input vals are " << RD_file << " " << F_file << ' '
              << disp_1d_file << ' ' << mask_1d_file << ' '
              << left_crop_x << ' '  << left_crop_y << " " 
              << right_crop_x << ' ' << right_crop_y << std::endl;

    if (left_crop_y != right_crop_y) 
      vw_throw(ArgumentErr() << "Left and right crop y must be the same\n");

    ImageView<PixelMask<Vector2f> > F;
    {
      // Peek inside of the RD file to find the size
      DiskImageView<PixelMask<Vector2f> > RD(RD_file);
      F.set_size(RD.cols(), RD.rows());
    }

    // Wipe F clean
    for (int col = 0; col < F.cols(); col++) {
      for (int row = 0; row < F.rows(); row++) {
        F(col, row) = PixelMask<Vector2f>();
        F(col, row).invalidate();
      }
    }

    ImageView<float> disp_1d = DiskImageView<float>( disp_1d_file );
    ImageView<uint8> mask_1d = DiskImageView<uint8>( mask_1d_file );

    int num_valid = 0;
    for (int col = 0; col < disp_1d.cols(); col++) {
      for (int row = 0; row < disp_1d.rows(); row++) {

        double disp_x   = disp_1d(col, row);
        int    is_valid = mask_1d(col, row);

        if (!is_valid) 
          continue;

        int left_pixel_x = left_crop_x + col;
        int left_pixel_y = left_crop_y + row;

        double right_pixel_x = right_crop_x + col + disp_x;
        double right_pixel_y = right_crop_y + row;
      
        if (left_pixel_x >= 0 && left_pixel_x < F.cols() &&
            left_pixel_y >= 0 && left_pixel_y < F.rows()){
          PixelMask<Vector2f> Q;
          Q.validate();
          Q.child() = Vector2f(right_pixel_x-left_pixel_x, right_pixel_y - left_pixel_y);

          // Watch for NaNs
          if (Q == Q) {
            F(left_pixel_x, left_pixel_y) = Q;
            F(left_pixel_x, left_pixel_y).validate();
            num_valid++;
          }
        }
      
      }
    }

    vw_out() << "num valid disparities" << num_valid << std::endl;
  
    bool has_georef = false;
    cartography::GeoReference georef;
    bool has_nodata = false;
    double nodata = -32768.0;

    vw_out() << "Writing: " << F_file << endl;
    vw::cartography::block_write_gdal_image(F_file, F,
                                            has_georef, georef,
                                            has_nodata, nodata, opt,
                                            TerminalProgressCallback
                                            ("asp","\t--> Filtering: ") );
  
  
  } ASP_STANDARD_CATCHES;

  return 0;
}
