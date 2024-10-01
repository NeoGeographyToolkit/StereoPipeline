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

// Given a set of overlapping georeferenced images, find a small
// subset with almost the same coverage. This is used in SfS.

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>

#include <vw/Cartography/GeoTransform.h>

#include <vector>
#include <string>
#include <iostream>
#include <limits>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

struct Options : public vw::GdalWriteOptions {
  std::string image_list_file, out_list; 
  double threshold;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  
 double nan = std::numeric_limits<double>::quiet_NaN();

  po::options_description general_options("");
  general_options.add_options()
   ("image-list", po::value(&opt.image_list_file)->default_value(""),
    "The list of input images.")
   ("output-list,o", po::value(&opt.out_list)->default_value(""),
    "The file having the produced image subset.")
   ("threshold", po::value(&opt.threshold)->default_value(nan),
    "The image threshold. Pixels no less than this will contribute to the coverage.")
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

  // Validate inputs
  if (opt.image_list_file == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing image list.\n");
  if (opt.out_list == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing output list.\n");
  if (std::isnan(opt.threshold))
    vw::vw_throw(vw::ArgumentErr() << "The threshold must be set.\n");
    
  return;
}

// Calc the score of adding a new image. Apply the new value to the output image if
// requested.
double calc_score_apply(std::string const& image_file, 
                        double threshold, bool apply, 
                        vw::cartography::GeoReference const& out_georef,
                        // Output
                        vw::ImageView<vw::PixelMask<float>> & out_img) {
  
  // Read the image
  vw::DiskImageResourceGDAL in_rsrc(image_file);
  
  // Read the georef
  vw::cartography::GeoReference georef;
  bool is_good = read_georeference(georef, in_rsrc);
  if (!is_good)
    vw::vw_throw(vw::ArgumentErr() << "No georeference found in " << image_file << ".\n");
    
  // Read the no-data
  double nodata = -std::numeric_limits<float>::max();
  if (in_rsrc.has_nodata_read())
    nodata = in_rsrc.nodata_read();
  
  // Read the image and mask the no-data
  vw::ImageViewRef<vw::PixelMask<float>> img 
    = create_mask(vw::DiskImageView<float>(in_rsrc), nodata);
  
  // Bounds of the output and input images
  vw::BBox2 out_box = vw::bounding_box(out_img);
  vw::BBox2 img_box = vw::bounding_box(img);
  
  // Form the geo transform from the output to the input image
  vw::cartography::GeoTransform geotrans(out_georef, georef,
                                         out_box, img_box);
  
  // Find the current image bounding box in the output image coordinates
  std::cout << "--test this: " << vw::bounding_box(img) << std::endl;
  vw::BBox2 trans_box = geotrans.reverse_bbox(vw::bounding_box(img));
  std::cout << "--test trans box!\n";
  // Grow to int
  trans_box = vw::grow_bbox_to_int(trans_box);
  
  std::cout << "--out box is " << out_box << std::endl;
  std::cout << "--input box is " << img_box << std::endl;
  std::cout << "transformed input box is " << trans_box << std::endl;
  
  // trans_box must be contained within out_box, as that's how out_box was formed
  if (!out_box.contains(trans_box))
    vw::vw_throw(vw::ArgumentErr()
                  << "An image bounding box is not contained in output box.\n"); 
   

  // Prepare the image for interpolation   
  vw::PixelMask<float> no_data;
  no_data.invalidate();
  typedef vw::ValueEdgeExtension<vw::PixelMask<float>> NoDataType;
  NoDataType no_data_ext(no_data);
  vw::InterpolationView<vw::EdgeExtensionView<vw::ImageViewRef<vw::PixelMask<float>>, NoDataType>, vw::BilinearInterpolation> interp_img 
    = interpolate(img, vw::BilinearInterpolation(), no_data_ext);
 
  // Iterate over the transformed box and calculate the score or apply the
  // image
  double score = 0.0;
  for (int col = trans_box.min().x(); col < trans_box.max().x(); col++) {
    for (int row = trans_box.min().y(); row < trans_box.max().y(); row++) {
      
      vw::Vector2 out_pix(col, row);
      
      // If the value of out_pix is valid and no less than the threshold,
      // continue
      vw::PixelMask<float> out_val = out_img(col, row);
      if (is_valid(out_val) && out_val.child() >= threshold)
        continue;
        
      // Find the interpolated pixel value
      vw::Vector2 img_pix = geotrans.forward(out_pix);
      vw::PixelMask<float> val = interp_img(img_pix.x(), img_pix.y());
      
      // Skip invalid values and values less than the threshold
      if (!is_valid(val) || val.child() < threshold)
        continue;
      
      // This is a good value, increment the score
      score++;
      
      // Apply the value if requested
      if (apply)
        out_img(col, row) = val;
    }  
  }
   
   return score;
}
                           

void read_georefs(std::vector<std::string> const& image_files) {

  // TODO(oalexan1): Rename this function
  
  vw::vw_out() << "Reading input georeferences.\n";
  vw::cartography::GeoReference out_georef;

  // TODO(oalexan1): Must read nodata
  
  // Set up the progress bar
  int num_images = image_files.size();  
  vw::TerminalProgressCallback tpc("", "\t--> ");
  tpc.report_progress(0);
  double inc_amount = 1.0 / double(num_images);
   
  // Loop through all input images
  vw::BBox2 out_box;
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    std::cout << "\n";

    std::string image_file = image_files[image_iter]; 
    vw::DiskImageResourceGDAL in_rsrc(image_file);
    vw::DiskImageView<float> img(in_rsrc);
    vw::cartography::GeoReference georef;
    bool is_good = read_georeference(georef, in_rsrc);
    if (!is_good)
      vw::vw_throw(vw::ArgumentErr() << "No georeference found in " << image_file << ".\n");
    
    // Input image bounding box
    vw::BBox2 img_box = vw::bounding_box(img);
    std::cout << "--img box: " << img_box << std::endl;

    // Borrow the geo info from the first image
    if (image_iter == 0) {
      out_georef = georef;
      out_box = img_box;
    }
    
    // The georef transform from current to output image
    vw::cartography::GeoTransform geotrans(out_georef, georef,
                                           out_box, img_box);

    // Convert the bounding box of current image to output pixel coordinates
    vw::BBox2 trans_img_box = geotrans.reverse_bbox(img_box);
    std::cout << "--input and trans pix box: " << img_box << ' ' << trans_img_box << std::endl;
    out_box.grow(trans_img_box);
    
    tpc.report_incremental_progress(inc_amount);
    std::cout << "\n";
    
  } // End loop through DEM files
  tpc.report_finished();

  std::cout << "--final pixel box is " << out_box << std::endl;
  // grow this to int
  out_box = vw::grow_bbox_to_int(out_box);
  std::cout << "--final pixel box is " << out_box << std::endl;
  
  vw::Vector2 pix = out_box.min();
  std::cout << "--1pix and point is " << pix << ' ' << out_georef.pixel_to_point(pix) << std::endl;
  
  // Crop the georef to the output box
  out_georef = crop(out_georef, out_box);
  out_box -= out_box.min();
  std::cout << "--final pixel box is " << out_box << std::endl;
  
  pix = out_box.min();
  std::cout << "--2pix and point is " << pix << ' ' << out_georef.pixel_to_point(pix) << std::endl;
  
  // TODO(oalexan1): Must grow the output box to int. Then must crop
  // the georef to it and shift the box!

  // TODO(oalexan1): Must iterate over pixels. Then do forward(). Then convert
  // to point from forward pixel. Must also do multi-threaded.

  std::set<std::string> inspected_set;
  std::vector<std::string> inspected_images(num_images);
  std::vector<double> scores(num_images, 0.0);
  
  // Create the output image as a pixel mask, with all pixels being float and invalid
  vw::ImageView<vw::PixelMask<float>> out_img(out_box.width(), out_box.height());
  // invalidate all pixels
  for (int col = 0; col < out_img.cols(); col++) {
    for (int row = 0; row < out_img.rows(); row++) {
      out_img(col, row).invalidate();
    }
  }
  
  // Do as many passes as images, do nothing in each pass
  for (int pass = 0; pass < num_images; pass++) {
    
    // Skip inspected images
    if (inspected_set.find(image_files[pass]) != inspected_set.end())
      continue;
    
      
  }  
    
} // End function load_dem_bounding_boxes

void run_image_subset(Options const& opt) {
  
  std::vector<std::string> image_files;
  asp::read_list(opt.image_list_file, image_files);
  
  read_georefs(image_files);
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);
    run_image_subset(opt);
  } ASP_STANDARD_CATCHES;

  return 0;
}
