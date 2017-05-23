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
  string input_RD, disp_1d_file, mask_1d_file, output_F, input_file, output_file;
  Vector2i left_crop, right_crop, shrink, grow;
  bool fill_with_nan;
};

void handle_arguments( int argc, char *argv[], Options& opt ){

  po::options_description general_options("");
  general_options.add_options()
    ("input-RD", po::value(&opt.input_RD),     "The ASP RD file to read the disparity size from.")
    ("output-F", po::value(&opt.output_F),     "The ASP F file to write the disparity size to.")
    ("disp-1d", po::value(&opt.disp_1d_file),  "The S2P 1D disparity file to read.")
    ("mask-1d", po::value(&opt.mask_1d_file),  "The S2P 1D mask file to read.")
    ("left-crop", po::value(&opt.left_crop),   "How much the S2P left image was cropped.")
    ("right-crop", po::value(&opt.right_crop), "How much the S2P right image was cropped.")
    ("shrink", po::value(&opt.shrink)->default_value(Vector2i(0, 0)),
     "Simply take an image, and shrink from all corners by this, if non-zero.")
    ("grow", po::value(&opt.grow)->default_value(Vector2i(0, 0)),
     "Simply take an image, and grow from all corners by this amount if nonzero, using the zero value.")
    ("input", po::value(&opt.input_file), "The file to read and shrink/grow.")
    ("output", po::value(&opt.output_file), "The file to write after shrinking/growing.")
    ("fill-with-nan", po::bool_switch(&opt.fill_with_nan)->default_value(false),
     "Fill expanded images with NaN.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  //positional.add_options();
  po::positional_options_description positional_desc;

  string usage("[options] <input dem> <output dem>");
  bool allow_unregistered = false;
  vector<string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);


   vw::create_out_dir(opt.output_F);

  //// Turn on logging to file
  //asp::log_to_file(argc, argv, "", opt.out_prefix);

}

int main( int argc, char *argv[] ) {

  try {

    Options opt;
    
    handle_arguments( argc, argv, opt );
    
    if (opt.input_file != "" && opt.output_file != "") {

      // In this mode we will either shrink or grow
      // an image at boundaries.

      ImageFormat fmt = vw::image_format(opt.input_file);
      DiskImageView<float> input_img(opt.input_file);
      int cols = input_img.cols();
      int rows = input_img.rows();

      float nan = std::numeric_limits<float>::quiet_NaN();
      float fill = 0;
      if (opt.fill_with_nan) fill = nan;

      ImageView<float> output_img = copy(input_img);

      if (opt.shrink != Vector2()) {
        vw_out() << "Will shrink by " << opt.shrink << std::endl;
        BBox2i box = bounding_box(input_img);
        box.min() += opt.shrink;
        box.max() -= opt.shrink;
        output_img = crop(input_img, box);
        
      } else if (opt.grow != Vector2()) {
        vw_out() << "Will grow by " << opt.grow << std::endl;
        int bx = opt.grow[0];
        int by = opt.grow[1];
        output_img.set_size(cols + 2*bx, rows + 2*by);
        for (int col = 0; col < output_img.cols(); col++) {
          for (int row = 0; row < output_img.rows(); row++) {
            output_img(col, row) = fill;
          }
        }
        for (int col = 0; col < cols; col++) {
          for (int row = 0; row < rows; row++) {
            output_img(col + bx, row + by) = input_img(col, row);
          }
        }

      }
      
      vw_out() << "Output size: " << output_img.cols() << ' ' << output_img.rows() << std::endl;
      vw_out() << "Writing: " << opt.output_file << std::endl;
      
      if (fmt.channel_type == VW_CHANNEL_UINT8) {
        write_image(opt.output_file, channel_cast<uint8>(output_img));
      }else if (fmt.channel_type== VW_CHANNEL_FLOAT32) {
        write_image(opt.output_file, output_img);
      }else{
        vw_throw(ArgumentErr() << "Unknown channel type!\n");
      }

      return 0;
    }
    
    int left_crop_x   = opt.left_crop[0];
    int left_crop_y   = opt.left_crop[1];
    int right_crop_x  = opt.right_crop[0];
    int right_crop_y  = opt.right_crop[1];

    if (left_crop_y != right_crop_y) 
      vw_throw(ArgumentErr() << "Left and right crop y must be the same\n");

    ImageView<PixelMask<Vector2f> > F;
    {
      // Peek inside of the RD file to find the size
      DiskImageView<PixelMask<Vector2f> > RD(opt.input_RD);
      F.set_size(RD.cols(), RD.rows());
    }

    // Wipe F clean
    for (int col = 0; col < F.cols(); col++) {
      for (int row = 0; row < F.rows(); row++) {
        F(col, row) = PixelMask<Vector2f>();
        F(col, row).invalidate();
      }
    }

    ImageView<float> disp_1d = DiskImageView<float>( opt.disp_1d_file );
    ImageView<uint8> mask_1d = DiskImageView<uint8>( opt.mask_1d_file );

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
          Q.child() = Vector2f(right_pixel_x - left_pixel_x, right_pixel_y - left_pixel_y);

          // Watch for NaNs
          if (Q == Q) {
            F(left_pixel_x, left_pixel_y) = Q;
            F(left_pixel_x, left_pixel_y).validate();
          }
        }
      
      }
    }

    bool has_georef = false;
    cartography::GeoReference georef;
    bool has_nodata = false;
    double nodata = -32768.0;

    vw_out() << "Writing: " << opt.output_F << endl;
    vw::cartography::block_write_gdal_image(opt.output_F, F,
                                            has_georef, georef,
                                            has_nodata, nodata, opt,
                                            TerminalProgressCallback
                                            ("asp","\t--> Filtering: ") );
  
  
  } ASP_STANDARD_CATCHES;

  return 0;
}
