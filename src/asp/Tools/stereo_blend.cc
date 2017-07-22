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
// required when running the SGM algorithm on large data sets using
// parallel_stereo.

// ROI means region of interest. Each tile has a central area,
// extracted from the string, out-2048_0_1487_2048, and outer/padding
// areas, called buffers, that are not in the tile proper but rather
// in neighboring tiles. The buffer size is 0 at image boundary, and
// it can be smaller closer to the boundary, otherwise it is equal to
// the collar size, which is a fixed bias.

// Hence, what this tool does, is, take the outer areas of neighboring
// tiles which overlap with the inner area of the current tile, and
// blend the results.

#include <asp/Tools/stereo.h>
#include <vw/Stereo/DisparityMap.h>
#include <boost/filesystem.hpp>

using namespace vw;
using namespace vw::stereo;
using namespace asp;
using namespace std;

typedef DiskImageView<PixelMask<Vector2f> > DiskImageType;
typedef ImageView    <PixelMask<Vector2f> > DispImageType;
typedef ImageView    <double              > WeightsType;

const size_t NUM_NEIGHBORS = 8;
enum Position {TL = 0, T = 1, TR = 2,
                L = 3, M = 8,  R = 4,
               BL = 5, B = 6, BR = 7};
// M is the central non-buffered area.

/// Debugging aid
std::string position_string(int p) {
  switch(p) {
    case TL: return "TL";
    case T:  return "T";
    case TR: return "TR";
    case L:  return "L";
    case R:  return "R";
    case BL: return "BL";
    case B:  return "B";
    case BR: return "BR";
    default: return "M";
  };
}

/// Returns the opposite position (what it is in the neighbor)
Position get_opposed_position(Position p) {
  switch(p) {
    case TL: return BR;
    case T:  return B;
    case TR: return BL;
    case L:  return R;
    case R:  return L;
    case BL: return TR;
    case B:  return T;
    case BR: return TL;
    default: return M;
  };
}

/// Given "out-2048_0_1487_2048" return "2048_0_1487_2048"
std::string extract_process_folder_bbox_string(std::string  s) {
  // If the filename was included, throw it out first.
  if (s.find("-Dnosym.tif") != std::string::npos) {
    size_t pt = s.rfind("/");
    s = s.substr(0, pt);
  }
  size_t num_start = s.rfind("-");
  if (num_start == std::string::npos)
    vw_throw( ArgumentErr() << "Error parsing folder string: " << s );
  return s.substr(num_start+1);
}

/// Constructs a BBox2i from a parallel_stereo formatted folder.
BBox2i bbox_from_folder(std::string const& s) {
  std::string cropped = extract_process_folder_bbox_string(s);
  int x, y, width, height;
  sscanf(cropped.c_str(), "%d_%d_%d_%d", &x, &y, &width, &height);
  return BBox2i(x, y, width, height);
}


/// Returns one of eight possible ROI locations for the given tile.
/// - If get_buffer is set, fetch the ROI from the buffer region,
///   so from the outer region to the tile
///   Otherwise get it from the non-buffer region at that location,
///   that is, from the inner region.
/// - Some tiles do not have all buffers available, if one of these
///   is requested the function will return false.
/// - Set buffers_stripped if you want the output ROI in reference to
///   an image with the buffers removed.  This will always fail if combined
///   with get_bufer==true, as there is no buffer area to fetch
/// - Generally you would get the non-buffer region for the main tile,
///   and the opposed buffer region for the neighboring tile.
bool get_roi_from_tile(std::string const& tile_path, Position pos,
		       int buffer_size, bool get_buffer,
		       BBox2i &output_roi,
		       bool buffers_stripped=false) {
  
  // Initialize the output
  output_roi = BBox2i();
  
  // Get the ROI of this tile and of the entire processing job
  // The unbuffered roi is before we expand it with buffer on the sides.
  BBox2i unbuffered_roi = bbox_from_folder(tile_path); // extract from 2048_5120_1024_394
  
  Vector2i image_size = file_image_size(tile_path);
  if (buffers_stripped)
    image_size = Vector2i(unbuffered_roi.width(), unbuffered_roi.height());

  // Adjust the size of the output ROI according to the available buffer area
  int roi_width    = unbuffered_roi.width();   // Size with no buffers
  int roi_height   = unbuffered_roi.height();
  int image_width  = image_size[0];  // Size with buffers included
  int image_height = image_size[1];

  // Determine if this tile sits on the upper left image border.
  // We don't need to worry about the lower-right border,
  // we will infer what goes there based on tile size.
  bool left_edge  = (unbuffered_roi.min().x() == 0);
  bool top_edge   = (unbuffered_roi.min().y() == 0);
  if (buffers_stripped) {
    left_edge = top_edge = false; 
  }
  
  // Get the size of the buffers/padding  on each edge (no buffer if on the edge)
  int left_offset  = (left_edge ) ? 0 : buffer_size;
  int top_offset   = (top_edge  ) ? 0 : buffer_size;
  int right_offset = image_width  - left_offset - roi_width;
  int bot_offset   = image_height - top_offset  - roi_height;
  vw_out() << "Offsets: " << left_offset << ' ' << right_offset << ' '
           << top_offset << ' ' << bot_offset << std::endl;
  
  if (right_offset < 0 || bot_offset < 0) 
    vw_throw( ArgumentErr() << "Something is wrong with the current tile geometry.\n" );
  
  // The three sizes of bboxes that will be used.
  Vector2i corner_size         (buffer_size, buffer_size);
  Vector2i horizontal_edge_size(roi_width,   buffer_size);
  Vector2i vertical_edge_size  (buffer_size, roi_height);
  Vector2i dummy(0,0);
  BBox2i   image_box(0, 0, image_width, image_height);

  int right_diff = image_width  - right_offset;
  int bot_diff   = image_height - top_offset;
  switch(pos) {
  case TL: 
    if (get_buffer) {
      if (left_edge || top_edge)
        return false;
      output_roi = BBox2i(Vector2i(0, 0), dummy);
    } else
      output_roi = BBox2i(Vector2i(left_offset, top_offset), dummy);
    output_roi.set_size(corner_size);
    break;
  case T:
    if (get_buffer) {
      if (top_edge)
        return false;
      output_roi = BBox2i(Vector2i(left_offset, 0), dummy);
    } else
      output_roi = BBox2i(Vector2i(left_offset, top_offset), dummy);
    output_roi.set_size(horizontal_edge_size);
    break;             
  case TR:
    if (get_buffer) {
      if (top_edge)
          return false;
      output_roi = BBox2i(Vector2i(right_diff, 0), dummy);
    } else
      output_roi = BBox2i(Vector2i(right_diff - buffer_size, top_offset), dummy);
    output_roi.set_size(corner_size);
    break;
  case L:
    if (get_buffer) {
      if (left_edge)
        return false;
      output_roi = BBox2i(Vector2i(0, top_offset), dummy);
    } else
      output_roi = BBox2i(Vector2i(left_offset, top_offset), dummy);
    output_roi.set_size(vertical_edge_size);
    break;
  case R:
    if (get_buffer) {
      output_roi = BBox2i(Vector2i(right_diff, top_offset), dummy);
    } else
      output_roi = BBox2i(Vector2i(right_diff - buffer_size, top_offset), dummy);
    output_roi.set_size(vertical_edge_size);
    break;
  case BL:
    if (get_buffer) {
      if (left_edge)
        return false;
      output_roi = BBox2i(Vector2i(0, bot_diff), dummy);
    } else
      output_roi = BBox2i(Vector2i(left_offset, bot_diff - buffer_size), dummy);
    output_roi.set_size(corner_size);
    break;
  case B:
    if (get_buffer) {
      output_roi = BBox2i(Vector2i(left_offset, bot_diff), dummy);
    } else
      output_roi = BBox2i(Vector2i(left_offset, bot_diff - buffer_size), dummy);
    output_roi.set_size(horizontal_edge_size);
    break;
  case BR:
    if (get_buffer) {
      output_roi = BBox2i(Vector2i(right_diff, bot_diff), dummy);
    } else
      output_roi = BBox2i(Vector2i(right_diff - buffer_size, bot_diff - buffer_size), dummy);
    output_roi.set_size(corner_size);
    break;
  default: // M (central area, everything except for the buffers)
    if (get_buffer)
      return false; // Central area is never a buffer
    output_roi = BBox2i(Vector2i(left_offset, top_offset), dummy);
    output_roi.set_size(Vector2i(roi_width, roi_height));
    break;
  };

  // Ensure we never go across image boundary
  output_roi.crop(image_box);

  if (output_roi.empty()) return false;

  return true;
}

/// Load the desired portion of a disparity tile and associated image weights.
bool load_image_and_weights(std::string const& file_path, BBox2i const& roi,
                            DispImageType & image, WeightsType & weights) {
  // Verify image exists
  if (file_path == "")
    return false;
    
  // Load the image from disk
  DispImageType full_image = DiskImageType(file_path);
  
  // Compute the desired weights
  centerline_weights(full_image, weights, roi);
  
  // Extract the desired portion of the full image
  image = crop(full_image, roi);
  
  return true;
}

/// Perform the blending
void blend_tile_region(DispImageType      & main_image, WeightsType      & main_weights,
                       BBox2i const& main_roi,
                       DispImageType const& neighbor,   WeightsType const& neighbor_weights,
                       BBox2i const& neighbor_roi) {

  // Multiply-accumulate the image values, then accumulate the weights.
  // Careful when dealing with no-data values.
  DispImageType cropped_image   = crop(main_image,   main_roi);
  WeightsType   cropped_weights = crop(main_weights, main_roi);

  for (int col = 0; col < neighbor.cols(); col++) {
    for (int row = 0; row < neighbor.rows(); row++) {
      
      if (!is_valid(neighbor(col, row)) || neighbor_weights(col, row) <= 0) continue;

      // If there was no data before, there will be data now
      cropped_image(col, row).validate();
      cropped_image(col, row) += neighbor(col, row) * neighbor_weights(col, row);
      cropped_weights(col, row) += neighbor_weights(col, row);
    }
  }

  // Put the modified portion backs into the larger images
  crop(main_image,   main_roi) = cropped_image;
  crop(main_weights, main_roi) = cropped_weights;
}

struct BlendOptions {
  std::string main_path;
  BBox2i      main_roi;
  std::string tile_paths[NUM_NEIGHBORS];
  BBox2i      rois      [NUM_NEIGHBORS]; // TODO: Keep?
  int sgm_collar_size;
  
  // Helper functions to indicate which directions tiles are present in
  bool has_upper_tiles() const { 
    return ((tile_paths[TL] != "") || (tile_paths[T] != "") || (tile_paths[TR] != ""));
  };
  bool has_lower_tiles() const { 
    return ((tile_paths[BL] != "") || (tile_paths[B] != "") || (tile_paths[BR] != ""));
  };
  bool has_left_tiles() const { 
    return ((tile_paths[TL] != "") || (tile_paths[L] != "") || (tile_paths[BL] != ""));
  };
  bool has_right_tiles() const { 
    return ((tile_paths[TR] != "") || (tile_paths[R] != "") || (tile_paths[BR] != ""));
  };
};


/// Make sure the created ROIs are correct by design
void check_roi_bounds(BBox2i & input_roi, BBox2i & tile_roi, BBox2i const& image_box) {

  BBox2i original_input_roi = input_roi;
  input_roi.crop(image_box);

  BBox2i original_tile_roi = tile_roi;
  tile_roi.crop(image_box);

  if (original_input_roi != input_roi || original_tile_roi != tile_roi) 
    vw_throw(ArgumentErr() << "stereo_blend: Incorrect ROIs.");

  if (input_roi.size() != tile_roi.size()) 
    vw_throw(ArgumentErr() << "stereo_blend: Inner and outer ROI must be the same dimensions.");

  // The code below should not be reached
  
  Vector2i min_movement = input_roi.min() - original_input_roi.min();
  Vector2i max_movement = input_roi.max() - original_input_roi.max();

  // Shrink this one as well, in the same way
  //BBox2i original_tile_roi = tile_roi;
  tile_roi.min() += min_movement;
  tile_roi.max() += max_movement;
}


/// Blend the borders of an input disparity tile using the adjacent disparity tiles.
DispImageType
tile_blend( DispImageType const& input_image,
            BlendOptions       & opt) {

  const bool debug = false;

  // The amount of padding applied to each tile.
  int buff_size = opt.sgm_collar_size;

  const bool GET_BUFFER   = true;
  const bool NOT_BUFFER   = false;
  const bool BUFFERS_GONE = true;

  // Retrieve the output bounding box in the input image
  BBox2i output_bbox;
  bool ans = get_roi_from_tile(opt.main_path, M, buff_size, NOT_BUFFER, output_bbox);
  if (!ans) 
    vw_throw(ArgumentErr() << "stereo_blend: central region cannot be empty.");

  // The bbox of the input image and the bbox that will be written as
  // output, with the padding (buffer) removed.
  // BBox2i input_bbox  = bounding_box(input_image);  

  // Allocate the output image as a copy of the input image.
  // - This sets the non-blended portion of the image.
  DispImageType output_image = crop(input_image, output_bbox);

  // Compute weights for the main tile
  WeightsType main_weights;
  centerline_weights(input_image, main_weights, output_bbox);

  if (debug) {
    write_image("main_image.tif", output_image);
    write_image("main_weights.tif", main_weights);
  }

  // Load the neighboring eight tiles
  //Vector2i      tile_sizes [NUM_NEIGHBORS];
  BBox2i        tile_rois  [NUM_NEIGHBORS]; // ROIs in the neighbors
  BBox2i        input_rois [NUM_NEIGHBORS]; // ROIs in the main image
  DispImageType images     [NUM_NEIGHBORS]; // Contains cropped regions
  WeightsType   weights    [NUM_NEIGHBORS]; // Contains cropped regions

  // Figure out the ROIs, load images, and initialize output blend values.
  for (size_t i=0; i<NUM_NEIGHBORS; ++i) {
    if (opt.tile_paths[i] == "")
      continue;

    try {

      // Get the ROI from the cropped input image
      bool ans1 = get_roi_from_tile(opt.main_path, Position(i),
                                    buff_size, NOT_BUFFER, input_rois[i],
                                    BUFFERS_GONE);
      
      // Get the ROI from the neighboring tile
      bool ans2 = get_roi_from_tile(opt.tile_paths[i], get_opposed_position(Position(i)), 
                                    buff_size, GET_BUFFER, tile_rois[i]);
      
      if (!ans1 || !ans2) continue; // nothing to blend
      
      check_roi_bounds(input_rois[i], tile_rois[i], bounding_box(output_image));
      load_image_and_weights(opt.tile_paths[i], tile_rois[i], images[i], weights[i]);
      
      if (debug) {
        write_image("tile_image_"+position_string(i)+".tif", images[i]);
        write_image("tile_weights_"+position_string(i)+".tif", weights[i]);
      }
      
    } catch(...) {
      vw_out(WarningMessage) << "Error loading tile " << opt.tile_paths[i] 
                             << " but will proceed with the available tiles.\n";
      opt.tile_paths[i] = ""; // Mark this tile as invalid
    }

  }

  vw_out() << "Premultiply...\n";

  output_image *= main_weights;
  
  vw_out() << "Performing blending...\n";

  // Blend in the neighbors one section at a time.
  for (size_t i=0; i<NUM_NEIGHBORS; ++i) {
    if (opt.tile_paths[i] == "") // Check tile validity
      continue;
    // We discard the neighboring tile mask here because the weights will take 
    //  care of those pixels and we don't want to OR in the mask of the neighbor.
    blend_tile_region(output_image,             main_weights, input_rois[i],
                      validate_mask(images[i]), weights[i],   tile_rois[i]);
  }

  vw_out() << "Postmultiply...\n";
  
  // Normalize the main image values to account for the applied weighting.
  // Careful with zero weights. 
  for (int col = 0; col < output_image.cols(); col++) {
    for (int row = 0; row < output_image.rows(); row++) {
      if (main_weights(col, row) != 0) 
        output_image(col, row) /= main_weights(col, row);
      else
        output_image(col, row).invalidate();
    }
  }

  return output_image;
}


void fill_blend_options(ASPGlobalOptions const& opt, BlendOptions & blend_options) {

  blend_options.main_path = opt.out_prefix + "-Dnosym.tif";

  // Get the top level output folder for parallel_stereo
  boost::filesystem::path parallel_stereo_folder(opt.out_prefix);

  parallel_stereo_folder = parallel_stereo_folder.parent_path().parent_path();

  // This must be sync-ed up with parallel_stereo. Read the list of
  // dirs that parallel_stereo made.
  std::vector<std::string> folder_list;
  std::string dir;
  std::string dirList = opt.out_prefix + "-dirList.txt";
  std::ifstream ifs(dirList.c_str());
  while (ifs >> dir){
    folder_list.push_back(dir);
  }
  ifs.close();
  if (folder_list.empty()) 
    vw_throw( ArgumentErr() << "Something is corrupted. Found an empty file: " << dirList << ".\n" );
  
  // Get the main tile bbox from the subfolder name
  boost::filesystem::path mpath(blend_options.main_path);
  std::string mbb = mpath.parent_path().filename().string();
  blend_options.main_roi = bbox_from_folder(mbb);
  BBox2i main_bbox = blend_options.main_roi;
   
  // Figure out where each folder goes
  for (size_t i=0; i<folder_list.size(); ++i) {
    BBox2i bbox = bbox_from_folder(folder_list[i]);
    
    std::string bbox_string = extract_process_folder_bbox_string(folder_list[i]);
    
    const std::string abs_path = parallel_stereo_folder.string() + 
        "/" + folder_list[i] + "/" + bbox_string + "-Dnosym.tif";
      
    if (bbox.max().x() == main_bbox.min().x()) { // Tiles one column to left
      if (bbox.max().y() == main_bbox.min().y()) { // Top left
        blend_options.tile_paths[TL] = abs_path;
        blend_options.rois      [TL] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.min().y()) { // Left
        blend_options.tile_paths[L] = abs_path;
        blend_options.rois      [L] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.max().y()) { // Bot left
        blend_options.tile_paths[BL] = abs_path;
        blend_options.rois      [BL] = bbox;
        continue;
      }
    } // End left tiles
    if (bbox.min().x() == main_bbox.max().x()) { // Tiles one column to right
      if (bbox.max().y() == main_bbox.min().y()) { // Top right
        blend_options.tile_paths[TR] = abs_path;
        blend_options.rois      [TR] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.min().y()) { // Right
        blend_options.tile_paths[R] = abs_path;
        blend_options.rois      [R] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.max().y()) { // Bot right
        blend_options.tile_paths[BR] = abs_path;
        blend_options.rois      [BR] = bbox;
        continue;
      }
    } // End right tiles
    if (bbox.min().x() == main_bbox.min().x()) { // Tiles in same column
      if (bbox.max().y() == main_bbox.min().y()) { // Top
        blend_options.tile_paths[T] = abs_path;
        blend_options.rois      [T] = bbox;
        continue;
      }
      if (bbox.min().y() == main_bbox.max().y()) { // Bottom
        blend_options.tile_paths[B] = abs_path;
        blend_options.rois      [B] = bbox;
        continue;
      }
    }
    //vw_throw( ArgumentErr() << "Unrecognized folder location: " << folder_list[i] );
  }
  
  //vw_throw( ArgumentErr() << "DEBUG!" );
  
  blend_options.sgm_collar_size = stereo_settings().sgm_collar_size;
}

void stereo_blending( ASPGlobalOptions const& opt ) {

  BlendOptions blend_options;
  fill_blend_options(opt, blend_options);

  // This tool is only intended to run as part of parallel_stereo, which
  //  renames the normal -D.tif file to -Dnosym.tif.

  // Since this tool is only for follow-up processing of SGM results in
  //  parallel_stereo, it can be safely assumed that the input images are small enough
  //  to load entirely into memory.
  DispImageType integer_disp;

  try {

    // Verify that the input correlation file is float, indicating SGM processing.
    // - No need to run the blend operation on integer files!
    boost::shared_ptr<DiskImageResource> rsrc(DiskImageResourcePtr(blend_options.main_path));
    ChannelTypeEnum disp_data_type = rsrc->channel_type();
    if (disp_data_type == VW_CHANNEL_INT32)
      vw_throw(ArgumentErr() << "Error: stereo_blend should only be called after SGM correlation.");
    integer_disp = DiskImageType(blend_options.main_path);
    
  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at blending stage -- could not read input files.\n" 
                            << e.what() << "\nExiting.\n\n" );
  }
  
  cartography::GeoReference left_georef;
  bool   has_left_georef = read_georeference(left_georef,  opt.out_prefix + "-L.tif");
  bool   has_nodata      = false;
  double nodata          = -32768.0;

  DispImageType output = tile_blend(integer_disp, blend_options);

  string rd_file = opt.out_prefix + "-RD.tif";
  vw_out() << "Writing: " << rd_file << "\n";
  vw::cartography::block_write_gdal_image(rd_file, output,
                                          has_left_georef, left_georef,
                                          has_nodata, nodata, opt,
                                          TerminalProgressCallback("asp", "\t--> Blending :") );
}

int main(int argc, char* argv[]) {

  try {

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 2 --> BLENDING \n";

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

    // Internal Processes
    //---------------------------------------------------------
    stereo_blending( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : BLENDING FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
