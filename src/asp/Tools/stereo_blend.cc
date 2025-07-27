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


// \file stereo_blend.cc

// The purpose of this tool is to blend the boundaries of extra-large
// stereo_corr tiles so that no seams are visible.  This is only
// required when running local alignment or SGM/MGM on large data sets
// using parallel_stereo.

// ROI means region of interest. Each tile has a central area, the
// ROI, extracted from the string, out-2048_0_1487_2048, and
// outer/padding areas, called buffers, that are not in the tile
// proper but rather in neighboring tiles. The buffer size is 0 at
// image boundary, and it can be smaller closer to the boundary,
// otherwise it is equal to the collar size, which is a fixed bias.

// Hence, what this tool does, is, take the outer areas of neighboring
// tiles which overlap with the inner area of the current tile, and
// blend the results.

#include <asp/Tools/stereo.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>

#include <vw/Image/ImageMath.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Stereo/DisparityMap.h>
#include <boost/filesystem.hpp>

using namespace vw;
using namespace vw::stereo;
using namespace asp;
using namespace std;

typedef ImageView<double> WeightsType;
typedef PixelMask<Vector2f> MaskedPixType;

// Every tile has 8 neighbors
const int NUM_NEIGHBORS = 8;

// Enum for the tiles. We count later on on the fact that the
// neighbors have indices in [0, 7]. TILE_M is the main tile and the
// others are its neighbors.
enum TilePosition {TILE_M = -1,
                   TILE_TL = 0, TILE_T = 1, TILE_TR = 2,
                   TILE_L  = 3,             TILE_R  = 4,
                   TILE_BL = 5, TILE_B = 6, TILE_BR = 7};

/// Given "out-2048_0_1487_2048" return "2048_0_1487_2048"
std::string extract_process_folder_bbox_string(std::string s, std::string const& in_file) {

  // If the filename was included, throw it out first.
  if (s.find("-" + in_file) != std::string::npos) {
    size_t pt = s.rfind("/");
    s = s.substr(0, pt);
  }

  size_t num_start = s.rfind("-");
  if (num_start == std::string::npos)
    vw_throw(ArgumentErr() << "Error parsing folder string: " << s);
  return s.substr(num_start+1);
}
  
/// Constructs a BBox2i from a parallel_stereo formatted folder.
BBox2i bbox_from_folder(std::string const& s, std::string const& in_file) {
  std::string cropped = extract_process_folder_bbox_string(s, in_file);

  int x, y, width, height;
  sscanf(cropped.c_str(), "%d_%d_%d_%d", &x, &y, &width, &height);
  return BBox2i(x, y, width, height);
}

// Load an image and form its weights
bool load_image_and_weights(std::string const& file_path,
                            ImageView<MaskedPixType> & image, WeightsType & weights,
                            int & num_channels, bool & has_nodata, float& nodata_value) {

  // Initialize the outputs to something
  num_channels = 1;
  has_nodata = false;
  nodata_value = -32768.0;
  
  // Verify image exists
  if (file_path == "")
    return false;

  vw_out() << "Reading: " << file_path << std::endl;

  // Verify that the disparity has float pixels, as expected. Blending
  // is done either after algorithms which are not the old ASP block
  // matching (ASP_BM), when the disparity pixels are always float, or
  // with local epipolar alignment, when the resulting disparity has
  // float pixels even when ASP_BM is used.
  boost::shared_ptr<DiskImageResource> rsrc(DiskImageResourcePtr(file_path));
  ChannelTypeEnum disp_data_type = rsrc->channel_type();
  if (disp_data_type != VW_CHANNEL_FLOAT32)
    vw_throw(ArgumentErr() << "Error: stereo_blend should only be called with float images.");

  num_channels = vw::get_num_channels(file_path);

  if (rsrc->has_nodata_read()) {
    has_nodata = true;
    nodata_value = rsrc->nodata_read();
  } else {
    has_nodata = false;
    if (num_channels == 1) {
      vw_throw(ArgumentErr() << "stereo_blend: For a single-channel image "
               << "expecting to have a no-data value in order to keep track of invalid pixels.");
    }
  }
  
  // Load the image from disk
  if (num_channels == 3) {
    // Load a disparity
    image = DiskImageView<MaskedPixType>(file_path);
  } else if (num_channels == 1) {
    // Load a float image with a nodata value, and create a disparity. It is simpler
    // to do it this way than to write some template-based logic.
    ImageView<float> curr_image = DiskImageView<float>(file_path);
    image.set_size(curr_image.cols(), curr_image.rows());
    for (int col = 0; col < curr_image.cols(); col++) {
      for (int row = 0; row < curr_image.rows(); row++) {
        float val = curr_image(col, row);
        if (val != nodata_value) {
          image(col, row) = MaskedPixType(vw::Vector2f(val, val)); 
          image(col, row).validate();
        } else{
          image(col, row) = MaskedPixType(vw::Vector2f(0, 0)); 
          image(col, row).invalidate();
        }
      } 
    }
  } else {
    vw_throw(ArgumentErr() << "stereo_blend: Expecting an image with 1 or 3 bands, but "
             << "image " << file_path << " has " << num_channels << " channels.\n");
  }
  
  // Compute the desired weights. Strictly speaking we need to compute the weights
  // only on the portion of the image that we will use for blending, but weights
  // computation is a cheap operation comparing to I/O time.
  centerline_weights(image, weights);

#if 0
  // For debugging
  std::string weights_file = file_path + "_weight.tif";
  
  vw_out() << "Writing: " << weights_file << std::endl;
  write_image(weights_file, weights);
#endif
  
  return true;
}

struct BlendOptions {
  std::string main_path;    // the path to the main disparity to blend
  BBox2i      main_roi;     // the main region of interest without padding
  BBox2i      padded_main;  // the main region of interest with padding
  std::string neib_path  [NUM_NEIGHBORS]; // the path to the neighbor disparity to blend
  BBox2i      neib_roi   [NUM_NEIGHBORS]; // neighbor region of interest without padding
  BBox2i      padded_neib[NUM_NEIGHBORS]; // neighbor region of interest with padding
  int         pad_size;
  BBox2i      full_box; // The box in which all tiles must fit, including their padding

  // Add padding to a box
  BBox2i add_padding(BBox2i box) {
    box.expand(pad_size);
    box.crop(full_box);
    return box;
  }
};

// Check if the dimensions of a tile agree with the box it is supposed to fit in
void check_size(BBox2i const& bbox, std::string const& image_file) {
  Vector2i image_size = file_image_size(image_file);

  bool is_good = (bbox.width() == image_size.x() && bbox.height() == image_size.y());
  if (!is_good) 
    vw_throw(ArgumentErr() << "stereo_blend: File: " << image_file
             << " is expected to fit in box: " << bbox);
}

void fill_blend_options(ASPGlobalOptions const& opt, std::string const& in_file,
                        BlendOptions & blend_opt) {

  std::string left_image = opt.out_prefix + "-L.tif";
  Vector2i full_image_size = file_image_size(left_image);
  blend_opt.full_box = BBox2i(0, 0, full_image_size.x(), full_image_size.y());
  blend_opt.pad_size = stereo_settings().sgm_collar_size;
  
  blend_opt.main_path = opt.out_prefix + "-" + in_file;

  // Get the top level output folder for parallel_stereo
  boost::filesystem::path parallel_stereo_folder(opt.out_prefix);

  // This wrong, as it assumes that out_prefix always looks like path1/path2.
  // What if it is path1/path2/path3. 
  parallel_stereo_folder = parallel_stereo_folder.parent_path().parent_path();

  // This must be sync-ed up with parallel_stereo. Read the list of
  // dirs that parallel_stereo made.
  std::vector<std::string> folder_list;
  std::string dir;
  std::string dirList = opt.out_prefix + "-dirList.txt";
  std::ifstream ifs(dirList.c_str());
  while (ifs >> dir)
    folder_list.push_back(dir);
  ifs.close();
  if (folder_list.empty()) 
    vw_throw(ArgumentErr() << "Something is corrupted. Found an empty file: "
             << dirList << ".\n");
  
  // Get the main tile bbox from the subfolder name
  boost::filesystem::path mpath(blend_opt.main_path);
  std::string mbb = mpath.parent_path().filename().string();

  blend_opt.main_roi    = bbox_from_folder(mbb, in_file);
  blend_opt.padded_main = blend_opt.add_padding(blend_opt.main_roi);
  check_size(blend_opt.padded_main, blend_opt.main_path);
  
  BBox2i main_bbox = blend_opt.main_roi;

  // Figure out where each folder goes
  for (size_t i = 0; i < folder_list.size(); i++) {
    BBox2i bbox = bbox_from_folder(folder_list[i], in_file);

    std::string bbox_string = extract_process_folder_bbox_string(folder_list[i], in_file);
    
    // Note that folder_list[i] already has the output prefix relative to the
    // directory parallel_stereo runs in.
    const std::string abs_path =
      folder_list[i] + "/" + bbox_string + "-" + in_file;

    if (bbox.max().x() == main_bbox.min().x()) { // Tiles one column to left
      if (bbox.max().y() == main_bbox.min().y()) { // Top left
        blend_opt.neib_path[TILE_TL] = abs_path;
        blend_opt.neib_roi [TILE_TL] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.min().y()) { // Left
        blend_opt.neib_path[TILE_L] = abs_path;
        blend_opt.neib_roi [TILE_L] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.max().y()) { // Bot left
        blend_opt.neib_path[TILE_BL] = abs_path;
        blend_opt.neib_roi [TILE_BL] = bbox;
        continue;
      }
    } // End left tiles
    
    if (bbox.min().x() == main_bbox.max().x()) { // Tiles one column to right
      if (bbox.max().y() == main_bbox.min().y()) { // Top right
        blend_opt.neib_path[TILE_TR] = abs_path;
        blend_opt.neib_roi [TILE_TR] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.min().y()) { // Right
        blend_opt.neib_path[TILE_R] = abs_path;
        blend_opt.neib_roi [TILE_R] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.max().y()) { // Bot right
        blend_opt.neib_path[TILE_BR] = abs_path;
        blend_opt.neib_roi [TILE_BR] = bbox;
        continue;
      }
    } // End right tiles
    
    if (bbox.min().x() == main_bbox.min().x()) { // Tiles in same column
      if (bbox.max().y() == main_bbox.min().y()) { // Top
        blend_opt.neib_path[TILE_T] = abs_path;
        blend_opt.neib_roi [TILE_T] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.max().y()) { // Bottom
        blend_opt.neib_path[TILE_B] = abs_path;
        blend_opt.neib_roi [TILE_B] = bbox;
        continue;
      }
    }
  }

  // Compute the padded box for each neighbor
  for (int i = 0; i < NUM_NEIGHBORS; i++) {
    
    if (blend_opt.neib_path[i] == "") 
      continue; // no neighbor in that direction

    blend_opt.padded_neib[i] = blend_opt.add_padding(blend_opt.neib_roi[i]);
    check_size(blend_opt.padded_neib[i], blend_opt.neib_path[i]);
  }

}

// See if a given image has only invalid pixels
bool invalid_image(ImageView<MaskedPixType> const& image) {
  for (int col = 0; col < image.cols(); col++) {
    for (int row = 0; row < image.rows(); row++) {
      if (is_valid(image(col, row))) {
        // Found a valid pixel
        return false;
      }
    }
  }

  return true;
}

/// Blend the borders of the main tile using the neighboring
/// tiles.
/// While all the main tile and neighbor tiles have padding, we will save
/// the blended main tile without padding.

ImageView<MaskedPixType> tile_blend(ASPGlobalOptions const& opt,
                                    BlendOptions & blend_opt,
                                    // These will change
                                    int & num_channels, bool & has_nodata, float & nodata_value) {

  // Initialize the outputs to something
  num_channels = 1;
  has_nodata = false;
  nodata_value = -32768.0;

  // Start the output image as invalid and zero. It will be used to
  // accumulate the weighted disparities.
  ImageView<MaskedPixType> output_image(blend_opt.main_roi.width(), blend_opt.main_roi.height()); 
  for (int col = 0; col < output_image.cols(); col++) {
    for (int row = 0; row < output_image.rows(); row++) {
      output_image(col, row) = MaskedPixType(); 
      output_image(col, row).invalidate();
    }
  }
  
  // Accumulate here the weights
  WeightsType output_weights(blend_opt.main_roi.width(), blend_opt.main_roi.height());
  for (int col = 0; col < output_weights.cols(); col++) {
    for (int row = 0; row < output_weights.rows(); row++) {
      output_weights(col, row) = 0.0; 
    }
  }

  // Add the contribution from the main tile and neighboring tiles. Note
  // that i = -1 corresponds to the main tile.
  for (int i = -1; i < NUM_NEIGHBORS; i++) {

    ImageView<MaskedPixType> image;
    WeightsType weights;
    BBox2i padded_box;
    
    if (i == -1) {
      // Main tile
      // The main tile better exist
      int curr_num_channels = 1;
      bool curr_has_nodata = false;
      float curr_nodata_value = -32768.0;
      bool ans = load_image_and_weights(blend_opt.main_path, image, weights,
                                        curr_num_channels, curr_has_nodata, curr_nodata_value);
      if (!ans) 
        vw_throw(ArgumentErr() << "stereo_blend: main tile is missing.");

      // Since the image was loaded successfully, copy its other info
      num_channels = curr_num_channels;
      has_nodata   = curr_has_nodata;
      nodata_value = curr_nodata_value;

      padded_box = blend_opt.padded_main;
      
      // If there are no valid pixels in the main tile without its padding,
      // return an invalid blended tile.
      if (invalid_image(crop(image, blend_opt.main_roi - blend_opt.padded_main.min())))
        return output_image;
      
    } else {
      // A neighboring tile
      int curr_num_channels = 1;
      bool curr_has_nodata = false;
      float curr_nodata_value = -32768.0;
      bool ans = load_image_and_weights(blend_opt.neib_path[i], image, weights,
                                        curr_num_channels, curr_has_nodata, curr_nodata_value);
      if (!ans)
        continue; // Nothing to blend

      // Since the image was loaded successfully, copy its other info. Note we assume
      // all inputs are consistent.
      num_channels = curr_num_channels;
      has_nodata   = curr_has_nodata;
      nodata_value = curr_nodata_value;

      padded_box = blend_opt.padded_neib[i];
    }

    // Do the blending, either with the main or neighboring tiles
    for (int col = 0; col < output_image.cols(); col++) {
      for (int row = 0; row < output_image.rows(); row++) {

        // Convert the given pixel to the coordinate system of the full image
        Vector2 pix = Vector2(col, row) + blend_opt.main_roi.min();

        // Convert the pixel to the coordinate system of the current padded tile
        pix = pix - padded_box.min();

        // Padded tiles can overlap only partially with the central region of the main
        // tile. If not in the overlap region, skip the work.
        if (!vw::bounding_box(image).contains(pix)) 
          continue;

        if (!is_valid(image(pix[0], pix[1])) || weights(pix[0], pix[1]) <= 0.0) 
          continue; // No useful info

        output_image(col, row).validate();
        output_image(col, row)   += weights(pix[0], pix[1]) * image(pix[0], pix[1]);
        output_weights(col, row) += weights(pix[0], pix[1]);
      }
    }
  }
  
  // Normalize
  for (int col = 0; col < output_image.cols(); col++) {
    for (int row = 0; row < output_image.rows(); row++) {
      
      if (!is_valid(output_image(col, row)))
        continue;
      
      if (output_weights(col, row) <= 0) {
        output_image(col, row).invalidate();
        continue;
      }
      
      output_image(col, row) /= output_weights(col, row);
    }
  }
  
  return output_image;
}

void stereo_blending(ASPGlobalOptions const& opt, std::string const& in_file,
                     std::string const& out_file) {

  BlendOptions blend_opt;
  fill_blend_options(opt, in_file, blend_opt);

  // Since this tool is only for follow-up processing of SGM results
  // in parallel_stereo, it can be safely assumed that the input
  // images are small enough to load entirely into memory.

  cartography::GeoReference left_georef;
  std::string left_image = opt.out_prefix + "-L.tif";
  bool   has_left_georef = read_georeference(left_georef, left_image);
  int num_channels       = 1;
  bool has_nodata        = false;
  float nodata           = -32768.0;

  ImageView<MaskedPixType> blended_disp = tile_blend(opt, blend_opt,
                                                     // These will change
                                                     num_channels, has_nodata, nodata);

  // Sanity check
  if (num_channels == 1 && !has_nodata) {
    vw_throw(ArgumentErr() << "stereo_blend: For a single-channel image "
             << "expecting to have a no-data value in order to keep track of invalid pixels.");
  }

  std::string full_out_file = opt.out_prefix + "-" + out_file;
  vw_out() << "Writing: " << full_out_file << "\n";
  if (num_channels == 3) {
    // Write the blended disparity
    vw::cartography::block_write_gdal_image(full_out_file, blended_disp,
                                            has_left_georef, left_georef,
                                            has_nodata, nodata, opt,
                                            TerminalProgressCallback("asp", "\t--> Blending :"));
  } else if (num_channels == 1) {
    // Write a single-channel image with no-data
    ImageView<float> image(blended_disp.cols(), blended_disp.rows());
    for (int col = 0; col < image.cols(); col++) {
      for (int row = 0; row < image.rows(); row++) {
        if (is_valid(blended_disp(col, row))) 
          image(col, row) = blended_disp(col, row).child()[0];
        else
          image(col, row) = nodata;
      }
    }
    vw::cartography::block_write_gdal_image(full_out_file, image,
                                            has_left_georef, left_georef,
                                            has_nodata, nodata, opt,
                                            TerminalProgressCallback("asp", "\t--> Blending:"));
  }
}

int main(int argc, char* argv[]) {

  try {

    vw_out() << "\n[ " << current_posix_time_string() << " ]: Stage 2 --> BLENDING\n";

    stereo_register_sessions();

    bool verbose = false;
    vector<ASPGlobalOptions> opt_vec;
    string output_prefix;
    asp::parse_multiview(argc, argv, SubpixelDescription(),
                         verbose, output_prefix, opt_vec);
    ASPGlobalOptions opt = opt_vec[0];

    // Subpixel refinement uses smaller tiles.
    //---------------------------------------------------------
    int ts = ASPGlobalOptions::rfne_tile_size();
    opt.raster_tile_size = Vector2i(ts, ts);

    // This tool is only intended to run as part of parallel_stereo, which
    //  renames the normal -D.tif file to -Dnosym.tif.
    std::string in_file =  "Dnosym.tif";

    string out_file = "B.tif";
    if (stereo_settings().subpixel_mode > 6){
      // No further subpixel refinement, skip to the -RD output.
      out_file = "RD.tif";
    }
    stereo_blending(opt, in_file, out_file);

    // See if to also blend L-R disp differences
    if (stereo_settings().save_lr_disp_diff) {
      in_file  = "L-R-disp-diff.tif";
      out_file = "L-R-disp-diff-blend.tif";
      stereo_blending(opt, in_file, out_file);
    }
    
    vw_out() << "\n[ " << current_posix_time_string() << " ]: BLENDING FINISHED\n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
