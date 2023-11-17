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

#include <asp/Core/DemUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/PdalUtils.h>

#include <vw/Image/AntiAliasing.h>
#include <vw/Image/Filter.h>
#include <vw/Image/InpaintView.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace vw;
namespace asp {

DemOptions::DemOptions():
  nodata_value(-std::numeric_limits<float>::max()),
  semi_major(0), semi_minor(0), fsaa(1),
  dem_hole_fill_len(0), ortho_hole_fill_len(0), ortho_hole_fill_extra_len(0),
  remove_outliers_with_pct(true), use_tukey_outlier_removal(false),
  max_valid_triangulation_error(0),
  erode_len(0), search_radius_factor(0), sigma_factor(0),
  default_grid_size_multiplier(1.0), use_surface_sampling(false),
  has_las_or_csv_or_pcd(false), max_output_size(9999999, 9999999), 
  auto_proj_center(false), input_is_projected(false) {}

// Create an antialiased DEM. This is old code. Needs to be wiped at some point.
ImageViewRef<PixelGray<float>>
generate_fsaa_raster(asp::OrthoRasterizerView const& rasterizer, 
                     DemOptions const& opt) {
  // This probably needs a Lanczos filter. Sinc filter is the ideal since it is
  // the ideal brick filter. Or possibly apply the blur on a linear scale
  // (pow(0,2.2), blur, then exp).

  float fsaa_sigma  = 1.0f * float(opt.fsaa)/2.0f;
  int   kernel_size = vw::compute_kernel_size(fsaa_sigma);

  ImageViewRef<PixelGray<float>> rasterizer_fsaa;
  if (opt.fsaa > 1) {
    // subsample with antialiasing
    // TODO(oalexan1): Break up this sorry mess
    rasterizer_fsaa = apply_mask(vw::resample_aa(translate(gaussian_filter
                       (fill_nodata_with_avg(create_mask(rasterizer.impl(), opt.nodata_value),
                         kernel_size), fsaa_sigma),
                        -double(opt.fsaa-1)/2.0, double(opt.fsaa-1)/2.0,
                        ConstantEdgeExtension()), 1.0/opt.fsaa),
                        opt.nodata_value);
  } else {
    rasterizer_fsaa = rasterizer.impl();
  }
  return rasterizer_fsaa;
}

// The files will be input point clouds, and if opt.do_ortho is
// true, also texture files. If texture files are present, there
// must be one for each point cloud, and each cloud must have the
// same dimensions as its texture file.
void parse_input_clouds_textures(std::vector<std::string> const& files,
                                 DemOptions& opt) {

  int num = files.size();
  if (num == 0)
    vw_throw(ArgumentErr() << "Missing input point clouds.\n");

  // Ensure there were no unrecognized options
  for (int i = 0; i < num; i++){
    if (!files[i].empty() && files[i][0] == '-'){
      vw_throw(ArgumentErr() << "Unrecognized option: " << files[i] << ".\n");
    }
  }

  // Ensure that files exist
  for (int i = 0; i < num; i++){
    if (!fs::exists(files[i])){
      vw_throw(ArgumentErr() << "File does not exist: " << files[i] << ".\n");
    }
  }

  if (opt.do_ortho){
    if (num <= 1)
      vw_throw(ArgumentErr() << "Missing input texture files.\n");
    if (num%2 != 0)
      vw_throw(ArgumentErr()
                << "There must be as many texture files as input point clouds.\n");
  }

  // Separate the input point clouds from the textures
  opt.pointcloud_files.clear(); opt.texture_files.clear();
  for (int i = 0; i < num; i++){
    if (asp::is_las_or_csv_or_pcd(files[i]) || get_num_channels(files[i]) >= 3)
      opt.pointcloud_files.push_back(files[i]);
    else
      opt.texture_files.push_back(files[i]);
  }

  if (opt.pointcloud_files.empty())
    vw_throw(ArgumentErr() << "No valid point cloud files were provided.\n");

  if (!opt.do_ortho && !opt.texture_files.empty())
    vw_throw(ArgumentErr() << "No ortho image was requested, yet texture files were passed as inputs.\n");

  // Must have this check here before we start assuming all input files
  // are tif.
  opt.has_las_or_csv_or_pcd = false;
  for (int i = 0; i < (int)files.size(); i++)
    opt.has_las_or_csv_or_pcd = (opt.has_las_or_csv_or_pcd || 
                                 asp::is_las_or_csv_or_pcd(files[i]));
  if (opt.has_las_or_csv_or_pcd && opt.do_ortho)
    vw_throw(ArgumentErr() 
             << "Cannot create orthoimages if point clouds are LAS or CSV.\n");

  if (opt.do_ortho){

    if (opt.pointcloud_files.size() != opt.texture_files.size())
      vw_throw(ArgumentErr() << "There must be as many input point clouds "
                              << "as texture files to be able to create orthoimages.\n");

    for (int i = 0; i < (int)opt.pointcloud_files.size(); i++){
      // Here we ignore that a point cloud file may have many channels.
      // We just want to verify that the cloud file and texture file
      // have the same number of rows and columns.
      DiskImageView<float> cloud(opt.pointcloud_files[i]);
      DiskImageView<float> texture(opt.texture_files[i]);
      if (cloud.cols() != texture.cols() || cloud.rows() != texture.rows()) {
        vw_throw(ArgumentErr() << "Point cloud " << opt.pointcloud_files[i]
                                << " and texture file " << opt.texture_files[i]
                                << " do not have the same dimensions.\n");
      }
    }
  }

  return;
}

// Convert any LAS or CSV files to ASP tif files. We do some binning
// to make the spatial data more localized, to improve performance.
// - We will later wipe these temporary tifs.
void las_or_csv_or_pcd_to_tifs(DemOptions& opt, vw::cartography::Datum const& datum,
                               std::vector<std::string> & tmp_tifs) {

  if (!opt.has_las_or_csv_or_pcd)
    return;

  Stopwatch sw;
  sw.start();

  // Error checking for CSV
  int num_files = opt.pointcloud_files.size();
  for (int i = 0; i < num_files; i++){
    if (!asp::is_csv(opt.pointcloud_files[i]))
      continue;
    if (opt.csv_format_str == "")
      vw_throw(ArgumentErr() << "CSV files were passed in, but the "
                             << "CSV format string was not set.\n");
  }

  // Extract georef info from PC or las files.
  vw::cartography::GeoReference pc_georef;
  bool have_pc_georef = asp::georef_from_pc_files(opt.pointcloud_files, pc_georef);

  // Configure a CSV converter object according to the input parameters
  asp::CsvConv csv_conv;
  csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str); // Modifies csv_conv

  // Set the georef for CSV files, if user's csv_proj4_str if specified
  vw::cartography::GeoReference csv_georef;
  csv_conv.parse_georef(csv_georef);

  // TODO: This may be a bug. What if the csv-proj4 and t_srs strings use
  // datums with different radii? 
  csv_georef.set_datum(datum);

  if (!have_pc_georef) // if we have no georef so far, the csv georef is our best guess.
    pc_georef = csv_georef;

  // There are situations in which some files will already be tif, and
  // others will be LAS or CSV. When we convert the latter to tif,
  // we'd like to be able to match the number of rows of the existing
  // tif files, so later when we concatenate all these files from left
  // to right for the purpose of creating the DEM, we waste little space.
  std::int64_t num_rows = 0;
  for (int i = 0; i < num_files; i++){
    if (asp::is_las_or_csv_or_pcd(opt.pointcloud_files[i]))
      continue;
    DiskImageView<float> img(opt.pointcloud_files[i]);
    // Record the max number of rows across all input tifs
    num_rows = std::max(num_rows, std::int64_t(img.rows())); 
  }

  // No tif files exist. Find a reasonable value for the number of rows.
  if (num_rows == 0) {
    std::int64_t max_num_pts = 0;
    for (int i = 0; i < num_files; i++){
      std::string file = opt.pointcloud_files[i];
      if (asp::is_las(file))
        max_num_pts = std::max(max_num_pts, asp::las_file_size(file));
      if (asp::is_csv(file))
        max_num_pts = std::max(max_num_pts, asp::csv_file_size(file));
      // Note: PCD support needs to be tested!
      if (asp::is_pcd(file))
        max_num_pts = std::max(max_num_pts, asp::pcd_file_size(file)); 
      // No need to check for other cases: at least one file must be las or csv or pcd!
    }
    num_rows = std::max(std::int64_t(1), (std::int64_t)ceil(sqrt(double(max_num_pts))));
  }

  // This is very important. For efficiency later, we don't want to
  // create blocks smaller than what OrthoImageView will use later.
  int block_size = ASP_MAX_SUBBLOCK_SIZE;

  // For csv and las files, create temporary tif files. In those files
  // we'll have the points binned so that nearby points have nearby
  // indices.  This is key to fast rasterization later.
  for (int i = 0; i < num_files; i++){

    if (!asp::is_las_or_csv_or_pcd(opt.pointcloud_files[i])) // Skip tif files
      continue;
    std::string in_file = opt.pointcloud_files[i];
    std::string stem    = fs::path(in_file).stem().string();
    std::string suffix;
    if (opt.out_prefix.find(stem) != std::string::npos)
      suffix = ".tif";
    else
      suffix = "-" + stem + ".tif";
    std::string out_file = opt.out_prefix + "-tmp" + suffix;

    // Handle the case when the output file may exist
    const int NUM_TEMP_NAME_RETRIES = 1000;
    for (int count = 0; count < NUM_TEMP_NAME_RETRIES; count++){
      if (!fs::exists(out_file))
        break;
      // File exists, try a different name
      vw_out() << "File exists: " << out_file << std::endl;
      std::ostringstream os; os << count;
      out_file = opt.out_prefix + "-tmp-" + os.str() + suffix;
    }
    if (fs::exists(out_file))
      vw_throw(ArgumentErr() << "Too many attempts at creating a temporary file.\n");

    // TODO: This if statement should not be needed, the function should handle it!
    // Perform the actual conversion to a tif file
    if (asp::is_las(in_file))
      asp::las_or_csv_to_tif(in_file, out_file, num_rows, block_size, 
                             &opt, pc_georef, csv_conv);
    else // CSV
      asp::las_or_csv_to_tif(in_file, out_file, num_rows, block_size, 
                             &opt, csv_georef, csv_conv);

    opt.pointcloud_files[i] = out_file; // so we can use it instead of the las file
    tmp_tifs.push_back(out_file); // so we can wipe it later
  }

  sw.stop();
  vw_out(DebugMessage,"asp") << "LAS or CSV to TIF conversion time: "
                             << sw.elapsed_seconds() << " seconds.\n";

} // End function las_or_csv_or_pcd_to_tifs

} // end namespace asp
