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

// Take as input left and right images, and the min crop y heights
// them from larger images. Further crop them to a shared area with
// few or no no-data values. These images will be used by s2p.

#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/PointUtils.h>

#include <limits>
#include <cstring>
#include <ctime>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;

bool input_nodata_value_was_set = false;

struct Options : public vw::cartography::GdalWriteOptions {
  int crop_y_left, crop_y_right;
  double input_nodata_value; // override the nodata value from the input files
  std::string left_in, right_in, left_out, right_out;
  Options():crop_y_left(-1), crop_y_right(-1){}

};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("crop-y-left",   po::value(&opt.crop_y_left)->default_value(0),
     "How much the left image was cropped on top from the original")
    ("crop-y-right",   po::value(&opt.crop_y_right)->default_value(0),
     "How much the right image was cropped on top from the original")
    ("input-nodata-value",
     po::value(&opt.input_nodata_value)->default_value(-std::numeric_limits<float>::max()),
     "Override the nodata-value from the input files.");
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("left-in", po::value(&opt.left_in), "The left input image.")
    ("right-in", po::value(&opt.right_in), "The right input image.")
    ("left-out", po::value(&opt.left_out), "The left output image.")
    ("right-out", po::value(&opt.right_out), "The right output image.")
    ;
  
  po::positional_options_description positional_desc;
  positional_desc.add("left-in",   1);
  positional_desc.add("right-in",  1);
  positional_desc.add("left-out",  1);
  positional_desc.add("right-out", 1);

  string usage("[options] <left-in> <right-in> <left-out> <right-out>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if (opt.left_in == "" || opt.right_in == "" || opt.left_out == "" || opt.right_out == "")
    vw_throw(ArgumentErr()
             << "Not all inputs our outputs were specified.\n"
             << usage << general_options << "\n");

  if (opt.crop_y_left < 0 || opt.crop_y_right < 0)
    vw_throw(ArgumentErr()
             << "The crop heights must be non-negative.\n"
             << usage << general_options << "\n");

  input_nodata_value_was_set = vm.count("input-nodata-value");
}

// To find the corners, find 45 and -45 degree diagonal lines bounding
// the valid pixels.  This heuristic seems to work quite well for
// IceBridge. Here it is convenient to think of the (0, 0) image
// corner being in in the lower-left, row going up, and col going
// right.
void find_corners(ImageView<PixelMask<float> > const& img,
                  Vector2i & ll, Vector2i & lr, Vector2i & ul, Vector2i & ur){

  int cols = img.cols();
  int rows = img.rows();

  ll = Vector2(0, 0);
  lr = Vector2(cols-1, 0);
  ul = Vector2(0, rows-1);
  ur = Vector2(cols-1, rows-1);

  int ll_val = cols + rows;
  int lr_val = cols + rows;
  int ul_val = 0;
  int ur_val = 0;
  
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      if (!is_valid(img(col, row))) continue;

      int intercept045 = row - col; // We are on the line: y =  x + intercept045
      int intercept135 = row + col; // We are on the line: y = -x + intercept135

      Vector2i P(col, row);
      
      if (intercept135 < ll_val) { ll_val = intercept135; ll = P; }
      if (intercept045 < lr_val) { lr_val = intercept045; lr = P; }
      if (intercept045 > ul_val) { ul_val = intercept045; ul = P; }
      if (intercept135 > ur_val) { ur_val = intercept135; ur = P; }
    }
  }
}

void replace_nodata_with_mean(ImageView<PixelMask<float> > & img){
  
  double mean_val = 0.0;
  int num = 0;

  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (!is_valid(img(col, row))) continue;
      mean_val += img(col, row).child();
      num      += 1;
    }
  }
  
  if (num >= 1) 
    mean_val /= double(num);
  
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      if (!is_valid(img(col, row))){
        img(col, row) = mean_val;
        img(col, row).validate();
      }
    }
  }
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Test the code below!!!
    int delta_left = 0, delta_right = 0;
    if (opt.crop_y_left > opt.crop_y_right) {
      // More was cropped from the left, so crop now more from the right, to
      // have the image rows correspond.
      delta_right = opt.crop_y_left - opt.crop_y_right;
    }
    if (opt.crop_y_left < opt.crop_y_right) {
      // The reverse situation
      delta_left = opt.crop_y_right - opt.crop_y_left;
    }

    float left_nodata_value = -std::numeric_limits<float>::max(); 
    bool has_left_nodata = vw::read_nodata_val(opt.left_in, left_nodata_value);
    
    float right_nodata_value = -std::numeric_limits<float>::max(); 
    bool has_right_nodata = vw::read_nodata_val(opt.right_in, right_nodata_value);

    if (!has_left_nodata || !has_right_nodata)
      vw_throw(ArgumentErr()
             << "The input images are supposed to have nodata-values.\n");
      
    vw_out() << "Read no-data values: " << left_nodata_value << ' ' << right_nodata_value
             << std::endl;


    if (input_nodata_value_was_set) {
      vw_out() << "Over-riding nodata values with " << opt.input_nodata_value << std::endl;
      left_nodata_value = opt.input_nodata_value;
      right_nodata_value = opt.input_nodata_value;
    }
    
    ImageView<PixelMask<float> > left = create_mask_less_or_equal(DiskImageView<float>(opt.left_in),
                                                    left_nodata_value);
    ImageView<PixelMask<float> > right = create_mask_less_or_equal(DiskImageView<float>(opt.right_in),
                                                     right_nodata_value);

    //  Corners of valid data
    Vector2i ll_left, lr_left, ul_left, ur_left;
    Vector2i ll_right, lr_right, ul_right, ur_right;

    find_corners(left, ll_left, lr_left, ul_left, ur_left);
    find_corners(right, ll_right, lr_right, ul_right, ur_right);

    int left_crop_min_x = std::max(std::max(ll_left[0],  ul_left[0]),
                                   std::max(ll_right[0], ul_right[0]));
    int left_crop_max_x = std::min(std::min(lr_left[0],  ur_left[0]),
                                   std::min(lr_right[0], ur_right[0]));
    int left_crop_min_y = std::max(std::max(ll_left[1],  lr_left[1]),
                                   std::max(ll_right[1], lr_right[1]));
    int left_crop_max_y = std::min(std::min(ul_left[1],  ur_left[1]),
                                   std::min(ul_right[1], ur_right[1]));

    // Crop both to the same window
    int right_crop_min_x = left_crop_min_x; int right_crop_max_x = left_crop_max_x;
    int right_crop_min_y = left_crop_min_y; int right_crop_max_y = left_crop_max_y;
    
    // Adjust as explained earlier
    left_crop_min_y  += delta_left;
    right_crop_min_y += delta_right;

    vw_out() << "left_crop " << left_crop_min_x << ' ' << left_crop_min_y << ' '
             << left_crop_max_x << ' ' << left_crop_max_y << std::endl;
    
    vw_out() << "right_crop " << right_crop_min_x << ' ' << right_crop_min_y << ' '
             << right_crop_max_x << ' ' << right_crop_max_y << std::endl;
    
    BBox2 left_crop_box(left_crop_min_x, left_crop_min_y,
                    left_crop_max_x - left_crop_min_x + 1,
                    left_crop_max_y - left_crop_min_y + 1);
    
    BBox2 right_crop_box(right_crop_min_x, right_crop_min_y,
                     right_crop_max_x - right_crop_min_x + 1,
                     right_crop_max_y - right_crop_min_y + 1);

    // S2P does not handle well no-data values, so replace those with a valid value.
    //TODO: Better use mean of neighbors!
    
    ImageView<PixelMask<float> > left_crop_img = crop(left, left_crop_box);
    replace_nodata_with_mean(left_crop_img);

    ImageView<PixelMask<float> > right_crop_img = crop(right, right_crop_box);
    replace_nodata_with_mean(right_crop_img);
    
    bool has_georef = false;
    cartography::GeoReference georef;

    vw_out() << "Writing: " << opt.left_out << std::endl;
    TerminalProgressCallback tpc_left("vw", "");
    block_write_gdal_image(opt.left_out,
                           apply_mask(left_crop_img, left_nodata_value),
                           has_georef, georef,
                           has_left_nodata, left_nodata_value,
                           opt, tpc_left);
    
    vw_out() << "Writing: " << opt.right_out << std::endl;
    TerminalProgressCallback tpc_right("vw", "");
    block_write_gdal_image(opt.right_out,
                           apply_mask(right_crop_img, right_nodata_value),
                           has_georef, georef,
                           has_right_nodata, right_nodata_value,
                           opt, tpc_right);
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
