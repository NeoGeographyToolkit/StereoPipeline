// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

#include <asp/Core/PointToDem.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/PointCloudProcessing.h>
#include <asp/Core/PdalUtils.h>
#include <asp/Core/CartographyUtils.h>

#include <vw/Image/AntiAliasing.h>
#include <vw/Image/Filter.h>
#include <vw/Image/InpaintView.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace vw;
namespace asp {

DemOptions::DemOptions():
  nodata_value(-std::numeric_limits<float>::max()),
  semi_major(0), semi_minor(0),
  dem_hole_fill_len(0), ortho_hole_fill_len(0), ortho_hole_fill_extra_len(0),
  remove_outliers_with_pct(true), use_tukey_outlier_removal(false),
  max_valid_triangulation_error(0),
  erode_len(0), search_radius_factor(0), sigma_factor(0),
  default_grid_size_multiplier(1.0),
  has_las_or_csv_or_pcd(false), max_output_size(9999999, 9999999),
  auto_proj_center(false), input_is_projected(false) {}

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
  for (int i = 0; i < num; i++) {
    if (!files[i].empty() && files[i][0] == '-') {
      vw_throw(ArgumentErr() << "Unrecognized option: " << files[i] << ".\n");
    }
  }

  // Ensure that files exist
  for (int i = 0; i < num; i++) {
    if (!fs::exists(files[i])) {
      vw_throw(ArgumentErr() << "File does not exist: " << files[i] << ".\n");
    }
  }

  if (opt.do_ortho) {
    if (num <= 1)
      vw_throw(ArgumentErr() << "Missing input texture files.\n");
    if (num%2 != 0)
      vw_throw(ArgumentErr()
                << "There must be as many texture files as input point clouds.\n");
  }

  // Separate the input point clouds from the textures
  opt.pointcloud_files.clear(); opt.texture_files.clear();
  for (int i = 0; i < num; i++) {
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
                                 opt.input_is_projected ||
                                 asp::is_las_or_csv_or_pcd(files[i]));
  if (opt.has_las_or_csv_or_pcd && opt.do_ortho)
    vw_throw(ArgumentErr()
             << "Cannot create orthoimages unless the inputs are PC.tif files.\n");

  if (opt.do_ortho) {

    if (opt.pointcloud_files.size() != opt.texture_files.size())
      vw_throw(ArgumentErr() << "There must be as many input point clouds "
                              << "as texture files to be able to create orthoimages.\n");

    for (int i = 0; i < (int)opt.pointcloud_files.size(); i++) {
      // Here we ignore that a point cloud file may have many channels.
      // We just want to verify that the cloud file and texture file
      // have the same number of rows and columns.
      DiskImageView<float> cloud(opt.pointcloud_files[i]);
      DiskImageView<float> texture(opt.texture_files[i]);
      if (cloud.cols() != texture.cols() || cloud.rows() != texture.rows())
        vw_throw(ArgumentErr() << "Point cloud " << opt.pointcloud_files[i]
                                << " and texture file " << opt.texture_files[i]
                                << " do not have the same dimensions.\n");
    }
  }

  return;
}

// Convert any LAS, CSV, PCD, or unorganized TIF files to ASP tif files. We do
// some binning to make the spatial data more localized, to improve performance.
// We will later wipe these temporary tif files.
void chip_convert_to_tif(DemOptions const& opt,
                         asp::CsvConv const& csv_conv,
                         vw::cartography::GeoReference const& csv_georef,
                         // Outputs
                         std::vector<std::string> & pc_files, 
                         std::vector<std::string> & conv_files) {

  if (!opt.has_las_or_csv_or_pcd)
    return;

  Stopwatch sw;
  sw.start();

  // Error checking for CSV
  int num_files = pc_files.size();
  for (int i = 0; i < num_files; i++) {
    if (!asp::is_csv(pc_files[i]))
      continue;
    if (opt.csv_format_str == "")
      vw_throw(ArgumentErr() 
               << "CSV files were passed in, but the option --csv-format was not set.\n");
  }

  // Extract georef info from PC or las files.
  vw::cartography::GeoReference pc_georef;
  bool have_pc_georef = asp::georef_from_pc_files(pc_files, pc_georef);

  if (!have_pc_georef) // if we have no georef so far, the csv georef is our best guess.
    pc_georef = csv_georef;

  // There are situations in which some files will already be tif, and
  // others will be LAS or CSV. When we convert the latter to tif,
  // we'd like to be able to match the number of rows of the existing
  // tif files, so later when we concatenate all these files from left
  // to right for the purpose of creating the DEM, we waste little space.
  std::int64_t num_rows = 0;
  for (int i = 0; i < num_files; i++) {
    if (asp::is_las_or_csv_or_pcd(pc_files[i]))
      continue;
    DiskImageView<float> img(pc_files[i]);
    // Record the max number of rows across all input tifs
    num_rows = std::max(num_rows, std::int64_t(img.rows()));
  }

  // No tif files exist. Find a reasonable value for the number of rows.
  if (num_rows == 0) {
    std::int64_t max_num_pts = 0;
    for (int i = 0; i < num_files; i++) {
      std::string file = pc_files[i];
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
  std::vector<std::string> all_out_files;
  for (int i = 0; i < num_files; i++) {

    if (!asp::is_las_or_csv_or_pcd(pc_files[i]) && !opt.input_is_projected) {
      // Skip organized tif files
      all_out_files.push_back(pc_files[i]);
      continue;
    }

    std::string in_file = pc_files[i];
    vw::vw_out() << "Processing file: " << in_file << "\n";
    std::string stem    = fs::path(in_file).stem().string();
    std::string suffix;
    if (opt.out_prefix.find(stem) != std::string::npos)
      suffix = "";
    else
      suffix = "-" + stem;
    std::string file_prefix = opt.out_prefix + "-tmp" + suffix;

    // TODO: This if statement should not be needed, the function should handle it!
    // Perform the actual conversion to a tif file
    std::vector<std::string> out_files;
    vw::GdalWriteOptions l_opt = opt; // Copy the write options
    if (asp::is_las(in_file))
      asp::las_or_csv_to_tif(in_file, file_prefix, num_rows, block_size,
                             l_opt, pc_georef, csv_conv, out_files);
    else // CSV, PCD, unordered projected TIF
      asp::las_or_csv_to_tif(in_file, file_prefix, num_rows, block_size,
                             l_opt, csv_georef, csv_conv, out_files);

    // Append out_files to all_out_files and to conv_files by inserting
    // Note that all_out_files will have both PC.tif files and outputs
    // of las_or_csv_to_tif, while conv_files will have only the latter.
    std::copy(out_files.begin(), out_files.end(), std::back_inserter(all_out_files));
    std::copy(out_files.begin(), out_files.end(), std::back_inserter(conv_files));
  }

  // Update the list of all files
  pc_files = all_out_files;

  sw.stop();
  vw_out(DebugMessage,"asp") << "LAS or CSV to TIF conversion time: "
                             << sw.elapsed_seconds() << " seconds.\n";

} // End function chip_convert_to_tif

// Set the projection based on options. By now opt.proj_lon and opt.proj_lat
// should have been set. 
void setProjection(DemOptions const& opt, cartography::GeoReference & output_georef) {
  
  // Can set a projection either via a string or via options
  if (!opt.target_srs_string.empty() && opt.target_srs_string != "auto")
    vw::vw_throw(ArgumentErr()
                << "Using a projection with --t_srs precludes settting it in other ways.");
  
  // Shorten notation
  double lon = opt.proj_lon, lat = opt.proj_lat, s = opt.proj_scale;
  double e = opt.false_easting, n = opt.false_northing;

  switch (opt.projection) {
    case SINUSOIDAL:           
      output_georef.set_sinusoidal(lon, e, n); 
      break;
    case MERCATOR:             
      output_georef.set_mercator(lat, lon, s, e, n); 
      break;
    case TRANSVERSEMERCATOR:   
      output_georef.set_transverse_mercator(lat, lon, s, e, n); 
      break;
    case ORTHOGRAPHIC:         
      output_georef.set_orthographic(lat, lon, e, n); 
      break;
    case STEREOGRAPHIC:        
      output_georef.set_stereographic(lat, lon, s, e, n); 
      break;
    case OSTEREOGRAPHIC:       
      output_georef.set_oblique_stereographic(lat, lon, s, e, n); 
      break;
    case GNOMONIC:             
      output_georef.set_gnomonic(lat, lon, s, e, n); 
      break;
    case LAMBERTAZIMUTHAL:     
      output_georef.set_lambert_azimuthal(lat, lon, e, n); 
      break;
    case UTM:                  
      output_georef.set_UTM(opt.utm_zone); 
      break;
    case GEOGRAPHIC:           
      output_georef.set_geographic(); 
      break;
    case AUTO_DETERMINED:   
      setAutoProj(lat, lon, output_georef);
      break;
    default:
      // Throw an error if the projection is not set
      vw::vw_throw(ArgumentErr() << "No projection was set.\n");
  }
  
  return;
} 

} // end namespace asp
