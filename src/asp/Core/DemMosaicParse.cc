// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file DemMosaicParse.cc
/// Parse and validate dem_mosaic options

#include <asp/Core/DemMosaicParse.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/FileUtils.h>

#include <vw/FileIO/GdalWriteOptionsDesc.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;

namespace asp {

void handleDemMosaicArgs(int argc, char *argv[], asp::DemMosaicOptions& opt) {

  po::options_description general_options("Options");
  general_options.add_options()
    ("dem-list,l", po::value<std::string>(&opt.dem_list),
     "A text file listing the DEM files to mosaic, one per line.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix. One or more tiles will be written with this prefix. Alternatively, an exact output file can be specified, with a .tif extension.")
    ("tile-size",       po::value<int>(&opt.tile_size)->default_value(1000000),
     "The maximum size of output DEM tile files to write, in pixels.")
    ("tile-index",      po::value<int>(&opt.tile_index),
     "The index of the tile to save (starting from zero). When this program is invoked, it will print out how many tiles are there. Default: save all tiles.")
    ("tile-list",      po::value(&opt.tile_list_str)->default_value(""),
     "List of tile indices (in quotes) to save. A tile index starts from 0.")
    ("priority-blending-length", po::value<int>(&opt.priority_blending_len)->default_value(0),
     "If positive, keep unmodified values from the earliest available DEM except a band this wide measured in pixels inward of its boundary where blending with subsequent DEMs will happen.")
    ("no-border-blend", po::bool_switch(&opt.no_border_blend)->default_value(false),
     "Only apply blending around holes, don't blend at image borders.  Not compatible with centerline weights.")
    ("tr",              po::value(&opt.tr),
     "Output grid size, that is, the DEM resolution in target georeferenced units per pixel. Default: use the same resolution as the first DEM to be mosaicked.")
    ("t_srs",           po::value(&opt.target_srs_string)->default_value(""),
     "Specify the output projection as a GDAL projection string (WKT, GeoJSON, or PROJ). If not provided, use the one from the first DEM to be mosaicked.")
    ("t_projwin",       po::value(&opt.projwin),
     "Limit the mosaic to this region, with the corners given in georeferenced coordinates "
     "(xmin ymin xmax ymax). Max is exclusive. See the --gdal-tap and --tap "
     "options if desired to apply addition adjustments to this extent.")
    ("gdal-tap", po::bool_switch(&opt.gdal_tap)->default_value(false),
     "Ensure that the output mosaic bounds (as printed by gdalinfo) are integer "
     "multiples of the grid size (as set with --tr). When --t_projwin is set and its "
     "entries are integer multiples of the grid size, that precise extent will be produced "
     "on output. This functions as the GDAL -tap option.")
    ("tap",  po::bool_switch(&opt.tap)->default_value(false),
     "Let the output grid be at integer multiples of the grid size (like the default "
     "behavior of point2dem and mapproject. Then the mosaic extent is biased by an "
     "additional half a pixel. If this option is not set, the input grids determine the "
     "output grid. See also --gdal-tap.")
    ("first",   po::bool_switch(&opt.first)->default_value(false),
     "Keep the first encountered DEM value (in the input order).")
    ("last",    po::bool_switch(&opt.last)->default_value(false),
     "Keep the last encountered DEM value (in the input order).")
    ("min",     po::bool_switch(&opt.min)->default_value(false),
     "Keep the smallest encountered DEM value.")
    ("max",     po::bool_switch(&opt.max)->default_value(false),
     "Keep the largest encountered DEM value.")
    ("mean",    po::bool_switch(&opt.mean)->default_value(false),
     "Find the mean DEM value.")
    ("stddev",    po::bool_switch(&opt.stddev)->default_value(false),
     "Find the standard deviation of the DEM values.")
    ("median",  po::bool_switch(&opt.median)->default_value(false),
     "Find the median DEM value (this can be memory-intensive, fewer threads are suggested).")
    ("nmad",  po::bool_switch(&opt.nmad)->default_value(false),
      "Find the normalized median absolute deviation DEM value (this can be memory-intensive, fewer threads are suggested).")
    ("count",   po::bool_switch(&opt.count)->default_value(false),
     "Each pixel is set to the number of valid DEM heights at that pixel.")
    ("weight-list", po::value<std::string>(&opt.weight_list),
     "A text file having a list of external weight files to use in blending, one per line. "
     "These are multiplied by the internal weights to ensure seamless blending. The weights "
     "must be in one-to-one correspondence with the DEMs to be mosaicked.")
    ("invert-weights", po::bool_switch(&opt.invert_weights)->default_value(false),
     "Use 1/weight instead of weight in blending, with --weight-list.")
    ("min-weight", po::value<double>(&opt.min_weight)->default_value(0.0),
     "Limit from below with this value the weights provided with --weight-list.")
    ("hole-fill-length", po::value(&opt.hole_fill_len)->default_value(0),
     "Maximum dimensions of a hole in the DEM to fill, in pixels. See also "
     "--fill-search-radius.")
    ("fill-search-radius",   po::value(&opt.fill_search_radius)->default_value(0.0),
     "Fill an invalid pixel with a weighted average of pixel values within this radius in pixels. The weight is 1/(factor * dist^power + 1), where the distance is measured in pixels. See an example in the doc. See also --fill-power, --fill-percent and --fill-num-passes.")
    ("fill-power", po::value(&opt.fill_power)->default_value(8.0),
     "Power exponent to use when filling nodata values with --fill-search-radius.")
    ("fill-percent", po::value(&opt.fill_percent)->default_value(10.0),
     "Fill an invalid pixel using weighted values of neighbors only if the percentage of valid pixels within the radius given by --fill-search-radius is at least this")
    ("fill-num-passes", po::value(&opt.fill_num_passes)->default_value(0),
     "Fill invalid values using --fill-search-radius this many times.")
    ("erode-length", po::value<int>(&opt.erode_len)->default_value(0),
     "Erode the DEM by this many pixels at boundary.")
    ("block-max", po::bool_switch(&opt.block_max)->default_value(false),
     "For each block of size --block-size, keep the DEM with the largest sum of values in the block.")
    ("georef-tile-size",    po::value<double>(&opt.geo_tile_size),
     "Set the tile size in georeferenced (projected) units (e.g., degrees or meters).")
    ("output-nodata-value", po::value<double>(&opt.out_nodata_value),
     "No-data value to use on output. Default: use the one from the first DEM to be "
     "mosaicked.")
    ("ot",  po::value(&opt.output_type)->default_value("Float32"),
     "Output data type. Supported types: Byte, UInt16, Int16, UInt32, Int32, Float32. If "
     "the output type is a kind of integer, values are rounded and then clamped to the "
     "limits of that type.")
    ("weights-blur-sigma", po::value<double>(&opt.weights_blur_sigma)->default_value(5.0),
     "The standard deviation of the Gaussian used to blur the weights. Higher value results in smoother weights and blending. Set to 0 to not use blurring.")
    ("weights-exponent",   po::value<double>(&opt.weights_exp)->default_value(2.0),
     "The weights used to blend the DEMs should increase away from the boundary as a power with this exponent. Higher values will result in smoother but faster-growing weights.")
    ("use-centerline-weights",   po::bool_switch(&opt.use_centerline_weights)->default_value(false),
     "Compute weights based on a DEM centerline algorithm. Produces smoother weights if the input DEMs don't have holes or complicated boundary.")
    ("dem-blur-sigma", po::value<double>(&opt.dem_blur_sigma)->default_value(0.0),
     "Blur the DEM using a Gaussian with this value of sigma. A larger value will blur more. Default: No blur.")
    ("nodata-threshold", po::value(&opt.nodata_threshold)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Values no larger than this number will be interpreted as no-data.")
    ("propagate-nodata", po::bool_switch(&opt.propagate_nodata)->default_value(false),
     "Set a pixel to nodata if any input DEM is also nodata at that location.")
    ("extra-crop-length", po::value<int>(&opt.extra_crop_len)->default_value(200),
     "Crop the DEMs this far from the current tile (measured in pixels) before blending them (a small value may result in artifacts). This value also helps determine how to "
     "plateau the blending weights inwards, away from the DEM boundary.")
    ("block-size",      po::value<int>(&opt.block_size)->default_value(0),
     "Process the mosaic with this value of the block size. A large value can result in "
     "increased memory usage. Later, the mosaic will be saved with blocks given by "
     "--tif-tile-size. This is an advanced internal parameter.")
    ("save-dem-weight",      po::value<int>(&opt.save_dem_weight),
     "Save the weight image that tracks how much the input DEM with given index contributed to the output mosaic at each pixel (smallest index is 0).")
    ("first-dem-as-reference", po::bool_switch(&opt.first_dem_as_reference)->default_value(false),
     "The output DEM will have the same size, grid, and georeference as the first one, with the other DEMs blended within its perimeter.")
    ("this-dem-as-reference", po::value(&opt.this_dem_as_reference)->default_value(""),
     "The output DEM will have the same size, grid, and georeference as this one, but it will not be used in the mosaic.")
    ("force-projwin", po::bool_switch(&opt.force_projwin)->default_value(false),
     "Make the output mosaic fill precisely the specified projwin, by padding it if necessary and aligning the output grid to the region.")
    ("save-index-map",   po::bool_switch(&opt.save_index_map)->default_value(false),
     "For each output pixel, save the index of the input DEM it came from (applicable only for --first, --last, --min, --max, --median, and --nmad). A text file with the index assigned to each input DEM is saved as well.")
    ("dem-list-file", po::value<std::string>(&opt.dem_list_file),
     "Alias for --dem-list, kept for backward compatibility.")
    ;

  // Use in GdalWriteOptions '--tif-tile-size' rather than '--tile-size', to not conflict
  // with the '--tile-size' definition used by this tool.
  bool adjust_tile_size_opt = true;
  general_options.add(vw::GdalWriteOptionsDescription(opt, adjust_tile_size_opt));

  // The input DEM files are either specified as positional arguments or in a list,
  // via --dem-list.  
  po::options_description positional("");
  positional.add_options()
    ("dem-files", po::value<std::vector<std::string>>(), "Input DEM files");
  po::positional_options_description positional_desc;
  positional_desc.add("dem-files", -1);

  std::string usage("[options] <dem files or -l dem_list.txt> -o output.tif");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  opt.dem_files.clear();
  if (vm.count("dem-files") != 0)
    opt.dem_files = vm["dem-files"].as<std::vector<std::string>>();

  // Error checking
  if (opt.out_prefix == "")
    vw_throw(ArgumentErr() << "No output prefix was specified.\n"
                           << usage << general_options);
  if (opt.num_threads == 0)
    vw_throw(ArgumentErr() << "The number of threads must be set and positive.\n"
                           << usage << general_options);
  if (opt.erode_len < 0)
    vw_throw(ArgumentErr() << "The erode length must not be negative.\n"
                           << usage << general_options);
  if (opt.extra_crop_len < 0)
    vw_throw(ArgumentErr() << "The blending length must not be negative.\n"
                           << usage << general_options);
  if (opt.hole_fill_len < 0)
    vw_throw(ArgumentErr() << "The hole fill length must not be negative.\n"
                           << usage << general_options);
  if (opt.fill_search_radius < 0.0)
    vw_throw(ArgumentErr() << "The fill search radius must be non-negative.\n"
                           << usage << general_options);
  if (opt.fill_power < 0.0)
    vw_throw(ArgumentErr() << "The fill factor must be non-negative.\n"
                           << usage << general_options);
    if (opt.fill_percent <= 0.0 || opt.fill_percent > 100.0)
    vw_throw(ArgumentErr() << "The fill percent must be in the range (0, 100].\n"
                           << usage << general_options);
  if (opt.fill_num_passes < 0)
    vw_throw(ArgumentErr() << "The number of fill passes must not be negative.\n"
                           << usage << general_options);
  if (opt.tile_size <= 0)
    vw_throw(ArgumentErr() << "The size of a tile in pixels must be positive.\n"
                           << usage << general_options);
  if (opt.priority_blending_len < 0)
    vw_throw(ArgumentErr() << "The priority blending length must not be negative.\n"
                           << usage << general_options);

  // This is a bug fix. The user by mistake passed in an empty projection string.
  if (!vm["t_srs"].defaulted() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr()
             << "The value of --t_srs is empty. Then it must not be set at all.\n");

  // If priority blending is used, need to adjust extra_crop_len accordingly
  opt.extra_crop_len = std::max(opt.extra_crop_len, 3*opt.priority_blending_len);

  // Make sure no more than one of these options is enabled.
  int noblend = numNoBlendOptions(opt);
  if (noblend > 1)
    vw_throw(ArgumentErr() << "At most one of the options --first, --last, "
         << "--min, --max, -mean, --stddev, --median, --nmad, --count can be specified.\n"
         << usage << general_options);

  if (opt.geo_tile_size < 0)
    vw_throw(ArgumentErr()
             << "The size of a tile in georeferenced units must not be negative.\n"
             << usage << general_options);

  if (noblend && opt.priority_blending_len > 0) {
    vw_throw(ArgumentErr()
       << "Priority blending cannot happen if any of the statistics DEMs are computed.\n"
       << usage << general_options);
  }

  if (opt.priority_blending_len > 0 && opt.weights_exp == 2) {
    vw_out() << "Increasing --weights-exponent to 3 for smoother blending.\n";
    opt.weights_exp = 3;
  }

  if (noblend && !opt.first && !opt.last && !opt.min && !opt.max && !opt.mean
      && opt.save_dem_weight >= 0) {
    vw_throw(ArgumentErr() << "Cannot save the weights unless blending is on or one of "
                           << "--first, --last, --min, --max, --mean is invoked.\n"
                           << usage << general_options);
  }

  if (opt.save_index_map && !opt.first && !opt.last &&
                            !opt.min && !opt.max && !opt.median && !opt.nmad)
    vw_throw(ArgumentErr() << "Cannot save an index map unless one of "
                           << "--first, --last, --min, --max, --median, --nmad is invoked.\n"
                           << usage << general_options);

  if (opt.save_dem_weight >= 0 && opt.save_index_map)
    vw_throw(ArgumentErr()
       << "Cannot save both the index map and the DEM weights at the same time.\n"
       << usage << general_options);

  // For compatibility with the GDAL tools, allow the min and max to be reversed.
  if (opt.projwin != BBox2()) {
    if (opt.projwin.min().x() > opt.projwin.max().x())
      std::swap(opt.projwin.min().x(), opt.projwin.max().x());
    if (opt.projwin.min().y() > opt.projwin.max().y())
      std::swap(opt.projwin.min().y(), opt.projwin.max().y());
  }

  if (opt.weights_blur_sigma < 0.0)
    vw_throw(ArgumentErr() << "The value --weights-blur-sigma must be non-negative.\n"
             << usage << general_options);

  if (opt.dem_blur_sigma < 0.0)
    vw_throw(ArgumentErr() << "The value --dem-blur-sigma must be non-negative.\n"
             << usage << general_options);

  if (opt.weights_exp <= 0)
    vw_throw(ArgumentErr() << "The weights exponent must be positive.\n"
             << usage << general_options);

  if (opt.priority_blending_len > 0 && opt.use_centerline_weights)
    vw::vw_throw(vw::ArgumentErr()
             << "The --priority-blending-length and --use-centerline-weights options "
             << "cannot be used together, as the latter expects no holes in the DEM, "
             << "but the priority blending length works by internally hollowing out "
             << "the non-priority DEMs before blending.\n");

  // Cannot have both --dem-list and --dem-list-file. The latter is for backward
  // compatibility.
  if (opt.dem_list != "" && opt.dem_list_file != "")
    vw_throw(ArgumentErr() << "Cannot have both --dem-list and --dem-list-file.\n");
  if (opt.dem_list_file != "") {
    opt.dem_list = opt.dem_list_file;
    opt.dem_list_file = "";
  }

  // Read the DEMs
  if (opt.dem_list != "") { // Get them from a list

    if (!opt.dem_files.empty()) {
      // Concatenate all the options into a single string
      std::string extra_list = "";
      for (size_t s = 0; s < opt.dem_files.size(); s++)
        extra_list += opt.dem_files[s];
      vw::vw_throw(vw::ArgumentErr()
                   << "The DEMs were specified via a list. There were however "
                   << "extraneous files or options passed in: " << extra_list << ".\n");
    }

    asp::read_list(opt.dem_list, opt.dem_files);
    if (opt.dem_files.empty())
      vw_throw(ArgumentErr() << "No DEM files to mosaic.\n");

  } else {  // Get them from the command line
    if (opt.dem_files.empty())
      vw_throw(ArgumentErr() << "No input DEMs were specified.\n");
  }

  if (opt.this_dem_as_reference != "" && opt.first_dem_as_reference) {
    vw::vw_throw(vw::ArgumentErr()
             << "Cannot have both options --first-dem-as-reference "
             << "and --this-dem-as-reference.\n");
  }

  // We will co-opt the logic of first_dem_as_reference but won't blend the reference DEM
  if (opt.this_dem_as_reference != "") {
    opt.first_dem_as_reference = true;
    opt.dem_files.insert(opt.dem_files.begin(), opt.this_dem_as_reference);
  }

  if (int(opt.dem_files.size()) <= opt.save_dem_weight)
    vw::vw_throw(vw::ArgumentErr()
             << "Cannot save weights for given index as it is out of bounds.\n");

  // When too many things should be done at the same time it is tricky
  // to have them work correctly. So prohibit that. Let the user run
  // one operation at a time.
  int num_ops = (opt.dem_blur_sigma > 0) + (opt.hole_fill_len > 0) +
                (opt.fill_search_radius > 0) + (opt.erode_len > 0);
  if (num_ops > 1)
    vw_throw(ArgumentErr() << "Cannot fill holes (based on size or search radius), blur, "
                           << "and erode the input DEM at the same time.\n");

  if (num_ops > 0 &&
      (opt.target_srs_string != "" || opt.tr > 0 || opt.dem_files.size() > 1 ||
       opt.priority_blending_len > 0))
    vw::vw_throw(vw::ArgumentErr() << "Cannot fill holes (based on size or search radius), "
       << "blur, or erode, if there is more than one input DEM, or reprojection, "
       << "or priority blending is desired. These operations should be done "
       << "one at a time as there may be issues due to the fact each input DEM has "
       << "its own grid size and also the order of operations.\n");

  if (opt.priority_blending_len > 0 &&
      (opt.tap || opt.gdal_tap ||  opt.force_projwin || !opt.projwin.empty() ||
       !opt.weight_list.empty() || opt.invert_weights || opt.min_weight > 0 ||
       opt.propagate_nodata ||  opt.first_dem_as_reference ||
       !opt.this_dem_as_reference.empty()))
    vw::vw_throw(vw::ArgumentErr()
             << "The option --priority-blending-length should not be mixed with other "
             << "options. Do each operation individually.\n");

  if (opt.tap && opt.gdal_tap)
    vw::vw_throw(vw::ArgumentErr()
           << "The options --tap and --gdal-tap cannot be used together.\n");

  // print warning usign vw warning message
  if (opt.fill_search_radius > 30)
    vw_out(vw::WarningMessage) << "The fill search radius is large. "
                               << "This may result in slow execution time.\n";

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  if (!vm.count("output-nodata-value")) {
    // Set a default out_nodata_value, but remember that this is
    // set internally, not by the user.
    opt.has_out_nodata = false;
    opt.out_nodata_value = -std::numeric_limits<float>::max();
  } else {
    opt.has_out_nodata = true;
  }

  // Cast this to float. All our nodata are float.
  opt.nodata_threshold = float(opt.nodata_threshold);

  if (!boost::math::isnan(opt.nodata_threshold) && noblend)
    vw::vw_throw(vw::ArgumentErr()
      << "The option --nodata-threshold cannot be used together with "
      << "some other invoked options.\n");

  // Parse the list of tiles to save. First replace commas and semicolons by a space.
  std::replace(opt.tile_list_str.begin(), opt.tile_list_str.end(), ',', ' ');
  std::replace(opt.tile_list_str.begin(), opt.tile_list_str.end(), ';', ' ');
  opt.tile_list.clear();
  std::istringstream os(opt.tile_list_str);
  int val;
  while (os >> val)
    opt.tile_list.insert(val);

  // Sanity checks for --first-dem-as-reference
  if (opt.first_dem_as_reference) {
    if (opt.target_srs_string != "" || opt.tr > 0 || opt.projwin != BBox2())
      vw_throw(ArgumentErr()
                << "Cannot change the projection, spacing, or output box, if the first DEM "
                << "is to be used as reference.\n");
    if (opt.first  || opt.last || opt.min    || opt.max || opt.mean ||
        opt.median || opt.nmad || opt.stddev ||
        opt.priority_blending_len > 0 || //opt.save_dem_weight >= 0 ||
        !boost::math::isnan(opt.nodata_threshold)) {
      vw_throw(ArgumentErr()
                << "Cannot do anything except regular blending if the first DEM "
                << "is to be used as reference.\n");
    }
  }

  // The block size must be a multiple of 16. Handle this early.
  if (opt.block_size % 16 != 0)
    vw::vw_throw(vw::ArgumentErr() << "The block size must be a multiple of 16.\n");

  // Handle the option --weight-list.
  if (opt.weight_list != "") {

    if (noblend || opt.priority_blending_len > 0 ||
        !boost::math::isnan(opt.nodata_threshold))
      vw_throw(ArgumentErr()
                << "Cannot do anything except regular blending with the option "
                << "--weight-list.\n");

    asp::read_list(opt.weight_list, opt.weight_files);

    // Must have the same number of weights as DEMs
    if (opt.weight_files.size() != opt.dem_files.size())
      vw_throw(ArgumentErr() << "The number of weights in the file " << opt.weight_list
                             << " must match the number of DEMs.\n");

    // Read each DEM temporarily and each weight to get their sizes and georefs. Those
    // must agree.
    for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++) {
      DiskImageView<double> dem(opt.dem_files[dem_iter]);
      DiskImageView<double> weight(opt.weight_files[dem_iter]);

      // Check cols and rows
      if (dem.cols() != weight.cols() || dem.rows() != weight.rows())
        vw_throw(ArgumentErr() << "The DEM " << opt.dem_files[dem_iter] << " and its weight "
                 << opt.weight_files[dem_iter] << " have different dimensions.\n");

      vw::cartography::GeoReference dem_georef, weight_georef;
      bool has_dem_georef = vw::cartography::read_georeference(dem_georef,
                                                               opt.dem_files[dem_iter]);
      bool has_weight_georef = vw::cartography::read_georeference(weight_georef,
                                                                  opt.weight_files[dem_iter]);
      if (!has_dem_georef)
        vw_throw(ArgumentErr() << "The DEM " << opt.dem_files[dem_iter]
                 << " has no georeference.\n");
      if (!has_weight_georef)
        vw_throw(ArgumentErr() << "The weight " << opt.weight_files[dem_iter]
                 << " has no georeference.\n");

      // Must have the same wkt string
      if (dem_georef.get_wkt() != weight_georef.get_wkt())
        vw_throw(ArgumentErr() << "The DEM " << opt.dem_files[dem_iter]
                 << " and its weight " << opt.weight_files[dem_iter]
                 << " have different georeferences.\n");
    }
   }
} // End function handleDemMosaicArgs

} // end namespace asp
