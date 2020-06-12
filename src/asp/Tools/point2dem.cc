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


/// \file point2dem.cc
///

#include <asp/Core/PointUtils.h>
#include <asp/Core/OrthoRasterizer.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Image/AntiAliasing.h>
#include <vw/Image/InpaintView.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Core/StringUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Cartography/PointImageManipulation.h>

#include <boost/math/special_functions/fpclassify.hpp>

#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;


// This is a list of types the user can specify for output with a dedicated command line flag.
enum ProjectionType {
  SINUSOIDAL,
  MERCATOR,
  TRANSVERSEMERCATOR,
  ORTHOGRAPHIC,
  STEREOGRAPHIC,
  OSTEREOGRAPHIC,
  GNOMONIC,
  LAMBERTAZIMUTHAL,
  UTM,
  PLATECARREE
};

struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::vector<std::string> pointcloud_files, texture_files;

  // Settings
  std::vector<double> dem_spacing;
  float       nodata_value;
  double      semi_major, semi_minor;
  std::string reference_spheroid, datum;
  double      phi_rot, omega_rot, kappa_rot;
  std::string rot_order;
  double      proj_lat, proj_lon, proj_scale, false_easting, false_northing;
  double      lon_offset, lat_offset, height_offset;
  size_t      utm_zone;
  ProjectionType projection;
  bool        has_alpha, do_normalize, do_ortho, do_error, no_dem;
  double      rounding_error;
  std::string target_srs_string;
  BBox2       target_projwin;
  int         fsaa, dem_hole_fill_len, ortho_hole_fill_len, ortho_hole_fill_extra_len;
  bool        remove_outliers_with_pct;
  Vector2     remove_outliers_params;
  double      max_valid_triangulation_error;
  Vector2     median_filter_params;
  int         erode_len;
  std::string csv_format_str, csv_proj4_str, filter;
  double      search_radius_factor, sigma_factor, default_grid_size_multiplier;
  bool        use_surface_sampling;
  bool        has_las_or_csv_or_pcd;
  Vector2i    max_output_size;

  // Output
  std::string out_prefix, output_file_type;

  // Defaults that the user doesn't need to see.
  Options() : nodata_value(-std::numeric_limits<float>::max()),
      semi_major(0), semi_minor(0), fsaa(1),
      dem_hole_fill_len(0), ortho_hole_fill_len(0), ortho_hole_fill_extra_len(0),
      remove_outliers_with_pct(true), max_valid_triangulation_error(0),
      erode_len(0), search_radius_factor(0), sigma_factor(0),
      default_grid_size_multiplier(1.0), use_surface_sampling(false),
      has_las_or_csv_or_pcd(false), max_output_size(9999999, 9999999){}
};

void parse_input_clouds_textures(std::vector<std::string> const& files,
                                 std::string const& usage,
                                 po::options_description const& general_options,
                                 Options& opt ) {

  // The files will be input point clouds, and if opt.do_ortho is
  // true, also texture files. If texture files are present, there
  // must be one for each point cloud, and each cloud must have the
  // same dimensions as its texture file.

  int num = files.size();
  if (num == 0)
    vw_throw( ArgumentErr() << "Missing input point clouds.\n"
                            << usage << general_options );

  // Ensure there were no unrecognized options
  for (int i = 0; i < num; i++){
    if (!files[i].empty() && files[i][0] == '-'){
      vw_throw( ArgumentErr() << "Unrecognized option: " << files[i] << ".\n"
                              << usage << general_options );
    }
  }

  // Ensure that files exist
  for (int i = 0; i < num; i++){
    if (!fs::exists(files[i])){
      vw_throw( ArgumentErr() << "File does not exist: " << files[i] << ".\n" );
    }
  }

  if (opt.do_ortho){
    if (num <= 1)
      vw_throw( ArgumentErr() << "Missing input texture files.\n"
                              << usage << general_options );
    if (num%2 != 0)
      vw_throw( ArgumentErr()
                << "There must be as many texture files as input point clouds.\n"
                << usage << general_options );
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
    vw_throw( ArgumentErr() << "No valid point cloud files were provided.\n");

  if (!opt.do_ortho && !opt.texture_files.empty())
    vw_throw( ArgumentErr() << "No ortho image was requested, yet texture files were passed as inputs.\n");

  // Must have this check here before we start assuming all input files
  // are tif.
  opt.has_las_or_csv_or_pcd = false;
  for (int i = 0; i < (int)files.size(); i++)
    opt.has_las_or_csv_or_pcd = opt.has_las_or_csv_or_pcd || asp::is_las_or_csv_or_pcd(files[i]);
  if (opt.has_las_or_csv_or_pcd && opt.do_ortho)
    vw_throw( ArgumentErr() << "Cannot create orthoimages if " << "point clouds are LAS or CSV.\n" );

  if (opt.do_ortho){

    if (opt.pointcloud_files.size() != opt.texture_files.size())
      vw_throw( ArgumentErr() << "There must be as many input point clouds "
                              << "as texture files to be able to create orthoimages.\n");

    for (int i = 0; i < (int)opt.pointcloud_files.size(); i++){
      // Here we ignore that a point cloud file may have many channels.
      // We just want to verify that the cloud file and texture file
      // have the same number of rows and columns.
      DiskImageView<float> cloud(opt.pointcloud_files[i]);
      DiskImageView<float> texture(opt.texture_files[i]);
      if ( cloud.cols() != texture.cols() || cloud.rows() != texture.rows() ){
        vw_throw( ArgumentErr() << "Point cloud " << opt.pointcloud_files[i]
                                << " and texture file " << opt.texture_files[i]
                                << " do not have the same dimensions.\n");
      }
    }
  }

}

/// Convert any LAS or CSV files to ASP tif files. We do some binning
/// to make the spatial data more localized, to improve performance.
/// - We will later wipe these temporary tifs.
void las_or_csv_or_pcd_to_tifs(Options& opt,
                        cartography::Datum const& datum,
                        std::vector<std::string> & tmp_tifs){

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
  GeoReference pc_georef;
  bool have_pc_georef = asp::georef_from_pc_files(opt.pointcloud_files, pc_georef);

  // Configure a CSV converter object according to the input parameters
  asp::CsvConv csv_conv;
  csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str); // Modifies csv_conv

  // Set the georef for CSV files, if user's csv_proj4_str if specified
  GeoReference csv_georef;
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
  int num_rows = 0;
  for (int i = 0; i < num_files; i++){
    if (asp::is_las_or_csv_or_pcd(opt.pointcloud_files[i]))
      continue;
    DiskImageView<float> img(opt.pointcloud_files[i]);
    num_rows = std::max(num_rows, img.rows()); // Record the max number of rows across all input tifs
  }

  // No tif files exist. Find a reasonable value for the number of rows.
  if (num_rows == 0){
    boost::uint64_t max_num_pts = 0;
    for (int i = 0; i < num_files; i++){
      std::string file = opt.pointcloud_files[i];
      if (asp::is_las(file))  max_num_pts = std::max(max_num_pts, asp::las_file_size(file));
      if (asp::is_csv(file))  max_num_pts = std::max(max_num_pts, asp::csv_file_size(file));
      if (asp::is_pcd(file))  max_num_pts = std::max(max_num_pts, asp::pcd_file_size(file)); // Note: PCD support needs to be tested!
      // No need to check for other cases; At least one file must be las or csv or pcd!
    }
    num_rows = std::max(1, (int)ceil(sqrt(double(max_num_pts))));
  }

  // This is very important. For efficiency later, we don't want to
  // create blocks smaller than what OrthoImageView will use later.
  int block_size = asp::OrthoRasterizerView::max_subblock_size();

  // For csv and las files, create temporary tif files. In those files
  // we'll have the points binned so that nearby points have nearby
  // indices.  This is key to fast rasterization later.
  for (int i = 0; i < num_files; i++){

    if (!asp::is_las_or_csv_or_pcd(opt.pointcloud_files[i])) // Skip tif files
      continue;
    std::string in_file = opt.pointcloud_files[i];
    std::string stem    = fs::path( in_file ).stem().string();
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
      vw_throw( ArgumentErr() << "Too many attempts at creating a temporary file.\n");

    // TODO: This if statement should not be needed, the function should handle it!
    // Perform the actual conversion to a tif file
    if (asp::is_las(in_file))
      asp::las_or_csv_to_tif(in_file, out_file, num_rows, block_size, &opt, pc_georef, csv_conv);
    else // CSV
      asp::las_or_csv_to_tif(in_file, out_file, num_rows, block_size, &opt, csv_georef, csv_conv);

    opt.pointcloud_files[i] = out_file; // so we can use it instead of the las file
    tmp_tifs.push_back(out_file); // so we can wipe it later
  }

  sw.stop();
  vw_out(DebugMessage,"asp") << "LAS or CSV to TIF conversion time: " << sw.elapsed_seconds() << std::endl;

}

// TODO: Move this somewhere?
/// Parses a string containing a list of numbers
void split_number_string(const std::string &input, std::vector<double> &output) {

  // Get a space delimited string
  std::string delimiter = " ";
  std::string s = input;
  std::replace( s.begin(), s.end(), ',', ' ');

  double val;
  std::stringstream stream(s);
  while (stream >> val) {
    output.push_back(val);
  }
}

void handle_arguments( int argc, char *argv[], Options& opt ) {

  std::string dem_spacing1, dem_spacing2;

  po::options_description manipulation_options("Manipulation options");
  manipulation_options.add_options()
    ("x-offset",       po::value(&opt.lon_offset )->default_value(0),    "Add a horizontal offset to the DEM.")
    ("y-offset",       po::value(&opt.lat_offset )->default_value(0),    "Add a horizontal offset to the DEM.")
    ("z-offset",       po::value(&opt.height_offset )->default_value(0),    "Add a vertical offset to the DEM.")
    ("rotation-order", po::value(&opt.rot_order)->default_value("xyz"),
         "Set the order of an Euler angle rotation applied to the 3D points prior to DEM rasterization.")
    ("phi-rotation",   po::value(&opt.phi_rot  )->default_value(0),"Set a rotation angle phi.")
    ("omega-rotation", po::value(&opt.omega_rot)->default_value(0),"Set a rotation angle omega.")
    ("kappa-rotation", po::value(&opt.kappa_rot)->default_value(0),"Set a rotation angle kappa.");

  po::options_description projection_options("Projection options");
  projection_options.add_options()
    ("t_srs",         po::value(&opt.target_srs_string)->default_value(""), "Specify the output projection (PROJ.4 string). Can also be an URL or in WKT format, as for GDAL.")
    ("t_projwin",     po::value(&opt.target_projwin),
     "The output DEM will have corners with these georeferenced coordinates.")
    ("dem-spacing,s", po::value(&dem_spacing1)->default_value(""),
             "Set output DEM resolution (in target georeferenced units per pixel). If not specified, it will be computed automatically (except for LAS and CSV files). Multiple spacings can be set (in quotes) to generate multiple output files. This is the same as the --tr option.")
    ("tr",            po::value(&dem_spacing2)->default_value(""), "This is identical to the --dem-spacing option.")
    ("datum",                    po::value(&opt.datum),
     "Set the datum. This will override the datum from the input images and also --t_srs, --semi-major-axis, and --semi-minor-axis. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("reference-spheroid,r", po::value(&opt.reference_spheroid),
     "This is identical to the datum option.")
    ("semi-major-axis",      po::value(&opt.semi_major)->default_value(0), "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis",      po::value(&opt.semi_minor)->default_value(0), "Explicitly set the datum semi-minor axis in meters.")
    ("sinusoidal",          "Save using a sinusoidal projection.")
    ("mercator",            "Save using a Mercator projection.")
    ("transverse-mercator", "Save using a transverse Mercator projection.")
    ("orthographic",        "Save using an orthographic projection.")
    ("stereographic",       "Save using a stereographic projection.")
    ("oblique-stereographic", "Save using an oblique stereographic projection.")
    ("gnomonic",            "Save using a gnomonic projection.")
    ("lambert-azimuthal",   "Save using a Lambert azimuthal projection.")
    ("utm",        po::value(&opt.utm_zone),                      "Save using a UTM projection with the given zone.")
    ("proj-lat",   po::value(&opt.proj_lat)->default_value(0),    "The center of projection latitude (if applicable).")
    ("proj-lon",   po::value(&opt.proj_lon)->default_value(0),    "The center of projection longitude (if applicable).")
    ("proj-scale", po::value(&opt.proj_scale)->default_value(1),  "The projection scale (if applicable).")
    ("false-easting", po::value(&opt.false_easting)->default_value(0),  "The projection false easting (if applicable).")
    ("false-northing", po::value(&opt.false_northing)->default_value(0),  "The projection false northing (if applicable).");

  po::options_description general_options("General Options");
  general_options.add_options()
    ("nodata-value",      po::value(&opt.nodata_value)->default_value(-std::numeric_limits<float>::max()),
             "Set the nodata value.")
    ("use-alpha",         po::bool_switch(&opt.has_alpha)->default_value(false),
             "Create images that have an alpha channel.")
    ("normalized,n",      po::bool_switch(&opt.do_normalize)->default_value(false),
             "Also write a normalized version of the DEM (for debugging).")
    ("orthoimage",        po::bool_switch(&opt.do_ortho)->default_value(false),
             "Write an orthoimage based on the texture files passed in as inputs (after the point clouds).")
    ("output-prefix,o",   po::value(&opt.out_prefix),                             "Specify the output prefix.")
    ("output-filetype,t", po::value(&opt.output_file_type)->default_value("tif"), "Specify the output file.")
    ("errorimage",        po::bool_switch(&opt.do_error)->default_value(false),   "Write an additional image whose values represent the triangulation ray intersection error in meters (the closest distance between the rays emanating from the two cameras corresponding to the same point on the ground).")
    ("dem-hole-fill-len", po::value(&opt.dem_hole_fill_len)->default_value(0),    "Maximum dimensions of a hole in the output DEM to fill in, in pixels.")
    ("orthoimage-hole-fill-len",      po::value(&opt.ortho_hole_fill_len)->default_value(0),
            "Maximum dimensions of a hole in the output orthoimage to fill in, in pixels.")
    ("orthoimage-hole-fill-extra-len",      po::value(&opt.ortho_hole_fill_extra_len)->default_value(0),
            "This value, in pixels, will make orthoimage hole filling more aggressive by first extrapolating the point cloud. A small value is suggested to avoid artifacts. Hole-filling also works better when less strict with outlier removal, such as in --remove-outliers-params, etc.")
    ("remove-outliers",               po::bool_switch(&opt.remove_outliers_with_pct)->default_value(true),
            "Turn on outlier removal based on percentage of triangulation error. Obsolete, as this is the default.")
    ("remove-outliers-params",        po::value(&opt.remove_outliers_params)->default_value(Vector2(75.0, 3.0), "pct factor"),
            "Outlier removal based on percentage. Points with triangulation error larger than pct-th percentile times factor will be removed as outliers. [default: pct=75.0, factor=3.0]")
    ("max-valid-triangulation-error", po::value(&opt.max_valid_triangulation_error)->default_value(0),
            "Outlier removal based on threshold. Points with triangulation error larger than this (in meters) will be removed from the cloud.")
    ("max-output-size",          po::value(&opt.max_output_size)->default_value(Vector2(9999999, 9999999)),
            "Don't write the output DEM if it is calculated to be this size or greater.")
    ("median-filter-params",          po::value(&opt.median_filter_params)->default_value(Vector2(0, 0),
            "window_size threshold"), "If the point cloud height at the current point differs by more than the given threshold from the median of heights in the window of given size centered at the point, remove it as an outlier. Use for example 11 and 40.0.")
    ("erode-length",   po::value<int>(&opt.erode_len)->default_value(0),
            "Erode input point clouds by this many pixels at boundary (after outliers are removed, but before filling in holes).")
    ("csv-format",     po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4",      po::value(&opt.csv_proj4_str)->default_value(""), "The PROJ.4 string to use to interpret the entries in input CSV files, if those files contain Easting and Northing fields. If not specified, --t_srs will be used.")
    ("filter",      po::value(&opt.filter)->default_value("weighted_average"), "The filter to apply to the heights of the cloud points within a given circular neighborhood when gridding (its radius is controlled via --search-radius-factor). Options: weighted_average (default), min, max, mean, median, stddev, count (number of points), nmad (= 1.4826 * median(abs(X - median(X)))), n-pct (where n is a real value between 0 and 100, for example, 80-pct, meaning, 80th percentile). Except for the default, the name of the filter will be added to the obtained DEM file name, e.g., output-min-DEM.tif.")
    ("rounding-error", po::value(&opt.rounding_error)->default_value(asp::APPROX_ONE_MM),
            "How much to round the output DEM and errors, in meters (more rounding means less precision but potentially smaller size on disk). The inverse of a power of 2 is suggested. [Default: 1/2^10]")
    ("search-radius-factor", po::value(&opt.search_radius_factor)->default_value(0.0),
     "Multiply this factor by dem-spacing to get the search radius. The DEM height at a given grid point is obtained as a weighted average of heights of all points in the cloud within search radius of the grid point, with the weights given by a Gaussian. Default search radius: max(dem-spacing, default_dem_spacing), so the default factor is about 1.")
    ("gaussian-sigma-factor", po::value(&opt.sigma_factor)->default_value(0.0),
     "The value s to be used in the Gaussian exp(-s*(x/grid_size)^2) when computing the DEM. The default is -log(0.25) = 1.3863. A smaller value will result in a smoother terrain.")
    ("default-grid-size-multiplier", po::value(&opt.default_grid_size_multiplier)->default_value(1.0),
     "If the output DEM grid size (--dem-spacing) is not specified, compute it automatically (as the mean ground sample distance), and then multiply it by this number. It is suggested that this number be set to 4 though the default is 1.")
    ("use-surface-sampling", po::bool_switch(&opt.use_surface_sampling)->default_value(false),
     "Use the older algorithm, interpret the point cloud as a surface made up of triangles and interpolate into it (prone to aliasing).")
    ("fsaa",   po::value<int>(&opt.fsaa)->default_value(1),            "Oversampling amount to perform antialiasing (obsolete).")
    ("no-dem", po::bool_switch(&opt.no_dem)->default_value(false), "Skip writing a DEM.");
  
  general_options.add( manipulation_options );
  general_options.add( projection_options );
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value< std::vector<std::string> >(), "Input files");

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <point-clouds> [ --orthoimage <textures> ]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if (vm.count("input-files") == 0)
    vw_throw( ArgumentErr() << "Missing input point clouds.\n" << usage << general_options );
  std::vector<std::string> input_files = vm["input-files"].as< std::vector<std::string> >();
  parse_input_clouds_textures(input_files, usage, general_options, opt);

  if (opt.median_filter_params[0] < 0 || opt.median_filter_params[1] < 0){
    vw_throw( ArgumentErr() << "The parameters for median-based filtering "
                            << "must be non-negative.\n" << usage << general_options );
  }

  if (opt.has_las_or_csv_or_pcd && opt.median_filter_params[0] > 0 &&
      opt.median_filter_params[1] > 0){
    vw_throw( ArgumentErr() << "Median-based filtering cannot handle CSV or LAS files.\n"
                            << usage << general_options );
  }

  if (opt.erode_len < 0){
    vw_throw( ArgumentErr() << "Erode length must be non-negative.\n"
                            << usage << general_options );
  }

  if ( (dem_spacing1.size() > 0) && (dem_spacing2.size() > 0) ){
    vw_throw( ArgumentErr() << "The DEM spacing was specified twice.\n"
                            << usage << general_options );
  }

  // Consolidate the dem_spacing and tr parameters
  if (dem_spacing1.size() < dem_spacing2.size())
    dem_spacing1 = dem_spacing2; // Now we can just use dem_spacing1

  // Extract the list of numbers from the input string
  split_number_string(dem_spacing1, opt.dem_spacing);
  if (opt.dem_spacing.size() == 0)
    opt.dem_spacing.push_back(0.0); // Make sure we have a number!

  bool spacing_provided = false;
  for (size_t i=0; i<opt.dem_spacing.size(); ++i) {
    if (opt.dem_spacing[i] < 0.0){
      // Note: Zero spacing means we'll set it internally.
      vw_throw( ArgumentErr() << "The DEM spacing must be non-negative.\n" << usage << general_options );
    }
    if (opt.dem_spacing[i] > 0)
      spacing_provided = true;
  }

  if (opt.has_las_or_csv_or_pcd && !spacing_provided){
    vw_throw( ArgumentErr() << "When inputs are LAS or CSV files, the "
                            << "output DEM resolution must be set.\n" );
  }

  if ( opt.out_prefix.empty() )
    opt.out_prefix = asp::prefix_from_pointcloud_filename( opt.pointcloud_files[0] );

  if (opt.use_surface_sampling){
    vw_out(WarningMessage) << "The --use-surface-sampling option invokes the old algorithm and "
                           << "is obsolete, it will be removed in future versions.\n";
  }

  if (opt.use_surface_sampling && opt.filter != "weighted_average")
    vw_throw( ArgumentErr() << "Cannot use surface "
                            << "sampling with any filter of point cloud points.\n" );

  if (opt.use_surface_sampling && opt.has_las_or_csv_or_pcd)
    vw_throw( ArgumentErr() << "Cannot use surface " << "sampling with LAS or CSV files.\n" );

  if (opt.fsaa != 1 && !opt.use_surface_sampling){
    vw_throw( ArgumentErr() << "The --fsaa option is obsolete. It can be used only with the "
              << "--use-surface-sampling option which invokes the old algorithm.\n" << usage << general_options );
  }

  if (opt.dem_hole_fill_len < 0)
    vw_throw( ArgumentErr() << "The value of --dem-hole-fill-len must be non-negative.\n");
  if (opt.ortho_hole_fill_len < 0)
    vw_throw( ArgumentErr() << "The value of --orthoimage-hole-fill-len must be non-negative.\n");
  if (opt.ortho_hole_fill_extra_len < 0)
    vw_throw( ArgumentErr() << "The value of --orthoimage-hole-fill-extra-len must be non-negative.\n");
  if ( !opt.do_ortho && opt.ortho_hole_fill_len > 0) {
    vw_throw( ArgumentErr() << "The value of --orthoimage-hole-fill-len"
                            << " is positive, but orthoimage generation was not requested.\n");
  }

  if (opt.ortho_hole_fill_len > 0) {
    // We do hole-filling before erosion and outlier removal, for performance reason,
    // and the two may not play nicely together.
    if (opt.erode_len > 0) 
      vw_out(WarningMessage) << "Erosion may interfere with filling "
                             << "holes in ortho images.\n";
    if (opt.median_filter_params[0] > 0 && opt.median_filter_params[1] > 0)
      vw_out(WarningMessage) << "Removing outliers using a median filter may interfere "
                             << "with filling holes in ortho images.\n";
    if (opt.remove_outliers_params[0] < 100)
      vw_out(WarningMessage) << "Removing outliers using a percentile filter may interfere "
                             << "with filling holes in ortho images.\n";
  }

  double pct = opt.remove_outliers_params[0], factor = opt.remove_outliers_params[1];
  if (pct <= 0 || pct > 100 || factor <= 0.0){
    vw_throw( ArgumentErr()
              << "Invalid values were provided for remove-outliers-params.\n");
  }

  if (opt.max_valid_triangulation_error > 0){
    // Since the user passed in a threshold, will use that to rm
    // outliers, instead of using the percentage.
    opt.remove_outliers_with_pct = false;
  }

  // For compatibility with GDAL, we allow the projwin y coordinate to be flipped.
  // Correct that here.
  if ( opt.target_projwin != BBox2() ) {
    if ( opt.target_projwin.min().y() > opt.target_projwin.max().y() ) {
      std::swap( opt.target_projwin.min().y(), opt.target_projwin.max().y() );
    }
    vw_out() << "Cropping to " << opt.target_projwin << " pt. " << std::endl;
  }

  // If the input PROJ.4 string is empty, use the output one. 
  if ( opt.csv_proj4_str.empty() && !opt.target_srs_string.empty()) {
    opt.csv_proj4_str = opt.target_srs_string;
  }

  // If the user specified a PROJ.4 string to use to interpret the
  // input in CSV files, use the same string to create output DEMs,
  // unless the user explicitly sets the output PROJ.4 string.
  if ( !opt.csv_proj4_str.empty() && opt.target_srs_string.empty()) {
    vw_out() << "The PROJ.4 string for reading CSV files was set. "
             << "Will use it for output as well.\n";
    opt.target_srs_string = opt.csv_proj4_str;
  }

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // reference_spheroid and datum are aliases.
  boost::to_lower(opt.reference_spheroid);
  boost::to_lower(opt.datum);
  if (opt.datum != "" && opt.reference_spheroid != "")
    vw_throw( ArgumentErr() << "Both --datum and --reference-spheroid were specified.\n");
  if (opt.datum == "")
    opt.datum = opt.reference_spheroid;

  if      ( vm.count("sinusoidal") )           opt.projection = SINUSOIDAL;
  else if ( vm.count("mercator") )             opt.projection = MERCATOR;
  else if ( vm.count("transverse-mercator") )  opt.projection = TRANSVERSEMERCATOR;
  else if ( vm.count("orthographic") )         opt.projection = ORTHOGRAPHIC;
  else if ( vm.count("stereographic") )        opt.projection = STEREOGRAPHIC;
  else if ( vm.count("oblique-stereographic")) opt.projection = OSTEREOGRAPHIC;
  else if ( vm.count("gnomonic") )             opt.projection = GNOMONIC;
  else if ( vm.count("lambert-azimuthal") )    opt.projection = LAMBERTAZIMUTHAL;
  else if ( vm.count("utm") )                  opt.projection = UTM;
  else                                         opt.projection = PLATECARREE; // Default output projection
}

template <class ImageT>
ImageViewRef< PixelGray<float> >
generate_fsaa_raster( ImageViewBase<ImageT> const& rasterizer, Options const& opt ) {
  // This probably needs a lanczos filter. Sinc filter is the ideal
  // since it is the ideal brick filter.
  // ... or ...
  // possibly apply the blur on a linear scale (pow(0,2.2), blur, then exp).

  float fsaa_sigma  = 1.0f * float(opt.fsaa)/2.0f;
  int   kernel_size = vw::compute_kernel_size(fsaa_sigma);

  ImageViewRef< PixelGray<float> > rasterizer_fsaa;
  if ( opt.fsaa > 1 ) {
    // subsample .. samples from the corner.
    rasterizer_fsaa =
      apply_mask
      (vw::resample_aa
       (translate
    (gaussian_filter
     (fill_nodata_with_avg(create_mask(rasterizer.impl(),opt.nodata_value),
                           kernel_size),
      fsaa_sigma),
     -double(opt.fsaa-1)/2., double(opt.fsaa-1)/2.,
     ConstantEdgeExtension()), 1.0/opt.fsaa),
       opt.nodata_value);
  } else {
    rasterizer_fsaa = rasterizer.impl();
  }
  return rasterizer_fsaa;
}

namespace asp{

  // If the third component of a vector is NaN, mask that vector as invalid
  template<class VectorT>
  struct NaN2Mask: public ReturnFixedType< PixelMask<VectorT> > {
    NaN2Mask(){}
    PixelMask<VectorT> operator() (VectorT const& vec) const {
      if (boost::math::isnan(vec.z()))
        return PixelMask<VectorT>(); // invalid
      else
        return PixelMask<VectorT>(vec); // valid
    }
  };

  // Reverse the operation of NaN2Mask
  template<class VectorT>
  struct Mask2NaN: public ReturnFixedType<VectorT> {
    Mask2NaN(){}
    VectorT operator() (PixelMask<VectorT> const& pvec) const {
      if (!is_valid(pvec))
        return VectorT(0, 0, std::numeric_limits<typename VectorT::value_type>::quiet_NaN());
      else
        return pvec.child();
    }
  };

  // If the third component of a vector is NaN, assign to it the given no-data value
  struct NaN2NoData: public ReturnFixedType<Vector3> {
    NaN2NoData(float nodata_val):m_nodata_val(nodata_val){}
    float m_nodata_val;
    Vector3 operator() (Vector3 const& vec) const {
      if (boost::math::isnan(vec.z()))
        return Vector3(m_nodata_val, m_nodata_val, m_nodata_val); // invalid
      else
        return vec; // valid
    }
  };

  // Take a given point xyz and the error at that point. Convert the
  // error to the NED (North-East-Down) coordinate system.
  struct ErrorToNED : public ReturnFixedType<Vector3> {
    GeoReference m_georef;
    ErrorToNED(GeoReference const& georef):m_georef(georef){}

    Vector3 operator() (Vector6 const& pt) const {

      Vector3 xyz = subvector(pt, 0, 3);
      if (xyz == Vector3()) return Vector3();

      Vector3   err     = subvector(pt, 3, 3);
      Vector3   geo     = m_georef.datum().cartesian_to_geodetic(xyz);
      Matrix3x3 M       = m_georef.datum().lonlat_to_ned_matrix(subvector(geo, 0, 2));
      Vector3   ned_err = inverse(M)*err;
      return ned_err;
    }
  };
  template <class ImageT>
  UnaryPerPixelView<ImageT, ErrorToNED>
  inline error_to_NED( ImageViewBase<ImageT> const& image, GeoReference const& georef ) {
    return UnaryPerPixelView<ImageT, ErrorToNED>( image.impl(),
                                                  ErrorToNED(georef) );
  }

  /// Write an image to disk while handling some common options.
  template<class ImageT>
  void save_image(Options& opt, ImageT img, GeoReference const& georef,
                  int hole_fill_len, std::string const& imgName){


    // When hole-filling is used, we need to look hole_fill_len beyond
    // the current block.  If the block size is 256, and hole fill len
    // is big, like 512 or 1024, we end up processing a huge block
    // only to save a small center block.  For that reason, save
    // temporarily with big blocks, and then re-save with small blocks.
    if (hole_fill_len > 512)
      vw_out(WarningMessage) << "Detected large hole-fill length. "
                             << "Memory usage and run-time may go up.\n";

    int block_size = nextpow2(2.0*hole_fill_len);
    block_size = std::max(256, block_size);

    // Append a tag if desired to compute the min, max, etc. Later on, in OrthoRasterizer
    // we do a full validation of opt.filter.
    std::string tag = "";
    if (opt.filter != "weighted_average")
      tag = "-" + opt.filter; 

    std::string output_file = opt.out_prefix + tag + "-" + imgName
      + "." + opt.output_file_type;
    vw_out() << "Writing: " << output_file << "\n";
    TerminalProgressCallback tpc("asp", imgName + ": ");
    if ( opt.output_file_type == "tif" ) {
      bool has_georef = true, has_nodata = true;
      asp::save_with_temp_big_blocks(block_size, output_file, img,
                                     has_georef, georef,
                                     has_nodata, opt.nodata_value, opt, tpc);
    }
    else
      vw::cartography::write_gdal_image(output_file, img, georef, opt, tpc);
  } // End function save_image


  /// A class for combining the three channels of errors and finding their absolute values.
  template <class ImageT>
  class CombinedView : public ImageViewBase<CombinedView<ImageT> >
  {
    double m_nodata_value;
    ImageT m_image1;
    ImageT m_image2;
    ImageT m_image3;
    typedef typename ImageT::pixel_type dpixel_type;

  public:

    typedef Vector3f pixel_type;
    typedef const Vector3f result_type;
    typedef ProceduralPixelAccessor<CombinedView> pixel_accessor;

    CombinedView(double nodata_value,
                 ImageViewBase<ImageT> const& image1,
                 ImageViewBase<ImageT> const& image2,
                 ImageViewBase<ImageT> const& image3):
      m_nodata_value(nodata_value),
      m_image1( image1.impl() ),
      m_image2( image2.impl() ),
      m_image3( image3.impl() ){}

    inline int32 cols  () const { return m_image1.cols(); }
    inline int32 rows  () const { return m_image1.rows(); }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {

      Vector3f error(m_image1(i, j), m_image2(i, j), m_image3(i, j));

      if (error[0] == m_nodata_value || error[1] == m_nodata_value || error[2] == m_nodata_value){
        return Vector3f(m_nodata_value, m_nodata_value, m_nodata_value);
      }

      return Vector3f(std::abs(error[0]), std::abs(error[1]), std::abs(error[2]));
    }

    /// \cond INTERNAL
    typedef CombinedView<typename ImageT::prerasterize_type> prerasterize_type;

    inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
      return prerasterize_type(m_nodata_value,
                               m_image1.prerasterize(bbox),
                               m_image2.prerasterize(bbox),
                               m_image3.prerasterize(bbox)
                              );
    }
    template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }
    /// \endcond
  };
  template <class ImageT>
  CombinedView<ImageT> combine_channels(double nodata_value,
                                        ImageViewBase<ImageT> const& image1,
                                        ImageViewBase<ImageT> const& image2,
                                        ImageViewBase<ImageT> const& image3){
    VW_ASSERT(image1.impl().cols() == image2.impl().cols() &&
              image2.impl().cols() == image3.impl().cols() &&
              image1.impl().rows() == image2.impl().rows() &&
              image2.impl().rows() == image3.impl().rows(),
              ArgumentErr() << "Expecting the error channels to have the same size.");

    return CombinedView<ImageT>(nodata_value, image1.impl(), image2.impl(), image3.impl());
  }

  /// Round pixels in given image to multiple of given scale.
  /// Don't round nodata values.
  template <class PixelT>
  struct RoundImagePixelsSkipNoData: public vw::ReturnFixedType<PixelT> {

    double m_scale, m_nodata;

    RoundImagePixelsSkipNoData(double scale, double nodata) : m_scale(scale), m_nodata(nodata){
    }

    PixelT operator() (PixelT const& pt) const {

      // We will pass in m_scale = 0 if we don't want rounding to happen.
      if (m_scale <= 0)
        return pt;

      // Skip given pixel if any channels are nodata
      int num_channels = PixelNumChannels<PixelT>::value;
      typedef typename CompoundChannelType<PixelT>::type channel_type;
      for (int c = 0; c < num_channels; c++){
        if ( (double)compound_select_channel<channel_type const&>(pt,c) == m_nodata )
          return pt;
      }

      return PixelT(m_scale*round(channel_cast<double>(pt)/m_scale));
    }

  };

  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, RoundImagePixelsSkipNoData<typename ImageT::pixel_type> >
  inline round_image_pixels_skip_nodata( vw::ImageViewBase<ImageT> const& image,
                                         double scale, double nodata ) {
    return vw::UnaryPerPixelView<ImageT, RoundImagePixelsSkipNoData<typename ImageT::pixel_type> >
      ( image.impl(), RoundImagePixelsSkipNoData<typename ImageT::pixel_type>(scale, nodata) );
  }

  template<class VectorT>
  struct VectorNorm: public ReturnFixedType<double> {
    VectorNorm(){}
    double operator() (VectorT const& vec) const {
      return norm_2(vec);
    }
  };

  class ErrorRangeEstimAccum : public ReturnFixedType<void> {
    typedef double accum_type;
    std::vector<accum_type> m_vals;
  public:
    typedef accum_type value_type;

    ErrorRangeEstimAccum() { m_vals.clear(); }

    void operator()( accum_type const& value ) {
      // Don't add zero errors, those most likely came from invalid points
      if (value > 0)
        m_vals.push_back(value);
    }

    int size(){
      return m_vals.size();
    }

    value_type value(Vector2 const& remove_outliers_params){
      VW_ASSERT(!m_vals.empty(), ArgumentErr() << "ErrorRangeEstimAccum: no valid samples");

      // How to pick a representative value for maximum error?  The
      // maximum error itself may be no good, as it could be very
      // huge, and then sampling the range of errors will be distorted
      // by that.  The solution adopted here: Find a percentile of the
      // range of errors, mulitply it by the outlier factor, and
      // multiply by another factor to ensure we don't underestimate
      // the maximum. This value may end up being larger than the
      // largest error, but at least it is is not grossly huge
      // if just a few of the errors are very large.
      std::sort(m_vals.begin(), m_vals.end());
      int    len    = m_vals.size();
      double pct    = remove_outliers_params[0]/100.0; // e.g., 0.75
      double factor = remove_outliers_params[1];
      int    k      = std::min(len-1, (int)(pct*len));
      double val    = m_vals[k]*factor*4.0;
      return val;
    }

  };

  template<int num_ch>
  ImageViewRef<double> error_norm(std::vector<std::string> const& pc_files){

    // Read the error channels from the point clouds, and take their norm

    VW_ASSERT(pc_files.size() >= 1, ArgumentErr() << "Expecting at least one file.\n");

    const int beg_ech = 3; // errors start at this channel
    const int num_ech = num_ch - beg_ech; // number of error channels
    ImageViewRef< Vector<double, num_ch> > point_disk_image
      = asp::form_point_cloud_composite<Vector<double, num_ch> >(pc_files,
          asp::OrthoRasterizerView::max_subblock_size());
    ImageViewRef< Vector<double, num_ech> > error_channels =
      select_channels<num_ech, num_ch, double>(point_disk_image, beg_ech);

    return per_pixel_filter(error_channels, asp::VectorNorm< Vector<double, num_ech> >());
  }

  int num_channels(std::vector<std::string> const& pc_files){

    // Find the number of channels in the point clouds.
    // If the point clouds have inconsistent number of channels,
    // return the minimum of 3 and the minimum number of channels.
    // This will be used to flag that we cannot reliable extract the
    // error channels, which start at channel 4.

    VW_ASSERT(pc_files.size() >= 1, ArgumentErr() << "Expecting at least one file.\n");

    int num_channels0 = get_num_channels(pc_files[0]);
    int min_num_channels = num_channels0;
    for (int i = 1; i < (int)pc_files.size(); i++){
      int num_channels = get_num_channels(pc_files[i]);
      min_num_channels = std::min(min_num_channels, num_channels);
      if (num_channels != num_channels0)
        min_num_channels = std::min(min_num_channels, 3);
    }
    return min_num_channels;
  }

} // end namespace asp


void do_software_rasterization(asp::OrthoRasterizerView& rasterizer,
                               Options& opt,
                               cartography::GeoReference& georef,
                               ImageViewRef<double> const& error_image,
                               double estim_max_error,
                               size_t *num_invalid_pixels) {

  vw_out() << "\t-- Starting DEM rasterization --\n";
  vw_out() << "\t--> DEM spacing: " <<     rasterizer.spacing() << " pt/px\n";
  vw_out() << "\t             or: " << 1.0/rasterizer.spacing() << " px/pt\n";

  // TODO: Maybe put a warning or check here if the size is too big

  // Now we are ready to specify the affine transform.
  georef.set_transform(rasterizer.geo_transform());

  // If the user requested FSAA, we temporarily increase the
  // resolution, apply a blur, then resample to the original
  // resolution. This results in a DEM with less antialiasing.  Note
  // that the georef above is set with the spacing before resolution
  // is increased, which will be the final spacing as well.
  if ( opt.fsaa > 1 )
    rasterizer.set_spacing( rasterizer.spacing() / double(opt.fsaa) );

  // If the user specified the ULLR .. update the georeference
  // transform here. The generate_fsaa_raster will be responsible
  // for making sure we have the correct pixel crop.
  if ( opt.target_projwin != BBox2() ) {
    Matrix3x3 transform = georef.transform();
    transform(0,2) = opt.target_projwin.min().x();
    transform(1,2) = opt.target_projwin.max().y();
    georef.set_transform( transform );
  }

  // Fix have pixel offset required if pixel_interpretation is
  // PixelAsArea. We could have done that earlier ... but it makes
  // the above easier to not think about it.
  if ( georef.pixel_interpretation() == cartography::GeoReference::PixelAsArea ) {
    Matrix3x3 transform = georef.transform();
    transform(0,2) -= 0.5 * transform(0,0);
    transform(1,2) -= 0.5 * transform(1,1);
    georef.set_transform( transform );
  }

  // Do not round the DEM heights for small bodies
  if (georef.datum().semi_major_axis() <= asp::MIN_RADIUS_FOR_ROUNDING ||
      georef.datum().semi_minor_axis() <= asp::MIN_RADIUS_FOR_ROUNDING){
    opt.rounding_error = 0.0;
  }

  ImageViewRef< PixelGray<float> > rasterizer_fsaa
    = generate_fsaa_raster( rasterizer, opt );

  // Write out the DEM. We've set the texture to be the height.
  Vector2 tile_size(vw_settings().default_tile_size(),
                    vw_settings().default_tile_size());
  if ( !opt.no_dem ){
    Stopwatch sw2;
    sw2.start();
    ImageViewRef< PixelGray<float> > dem
      = asp::round_image_pixels_skip_nodata(rasterizer_fsaa, opt.rounding_error,
                                            opt.nodata_value);

    int hole_fill_len = opt.dem_hole_fill_len;
    if (hole_fill_len > 0){
      // Note that we first cache the tiles of the rasterized DEM, and
      // fill holes later. This greatly improves the performance.
      dem = apply_mask
        (vw::fill_holes_grass(create_mask
                               (block_cache(dem, tile_size, opt.num_threads),
                                opt.nodata_value),
                               hole_fill_len),
         opt.nodata_value);
    }

    // Stop the program if it is going to create too large a DEM, this will cause a crash.
    Vector2i dem_size = bounding_box(dem).size();
    vw_out()<< "Creating output file that is " << dem_size << " px.\n";
    if ((dem_size[0] > opt.max_output_size[0]) || (dem_size[1] > opt.max_output_size[1]))
      vw_throw( ArgumentErr()
                << "Requested DEM size is too large, max allowed output size is "
                << opt.max_output_size << " pixels.\n" );

    asp::save_image(opt, dem, georef, hole_fill_len, "DEM");
    sw2.stop();
    vw_out(DebugMessage,"asp") << "DEM render time: " << sw2.elapsed_seconds() << ".\n";

    double num_invalid_pixelsD = static_cast<double>(*num_invalid_pixels);
    double num_total_pixels    = static_cast<double>(dem_size[0]*dem_size[1]);
    double invalid_ratio       = num_invalid_pixelsD / num_total_pixels;
    vw_out() << "Percentage of valid pixels = " << 1.0-invalid_ratio << "\n";
    *num_invalid_pixels = 0; // Reset this count
  }

  // Write triangulation error image if requested
  if ( opt.do_error ) {
    int num_channels = asp::num_channels(opt.pointcloud_files);

    int hole_fill_len = 0;
    if (num_channels == 4){
      // The error is a scalar.
      ImageViewRef<Vector4> point_disk_image
        = asp::form_point_cloud_composite<Vector4>
        (opt.pointcloud_files, asp::OrthoRasterizerView::max_subblock_size());
      ImageViewRef<double> error_channel = select_channel(point_disk_image,3);
      rasterizer.set_texture( error_channel );
      rasterizer_fsaa = generate_fsaa_raster( rasterizer, opt );
      save_image(opt, asp::round_image_pixels_skip_nodata(rasterizer_fsaa,
                                                          opt.rounding_error,
                                                          opt.nodata_value),
                 georef, hole_fill_len, "IntersectionErr");
    }else if (num_channels == 6){
      // The error is a 3D vector. Convert it to NED coordinate system, and rasterize it.
      ImageViewRef<Vector6> point_disk_image = asp::form_point_cloud_composite<Vector6>
        (opt.pointcloud_files, asp::OrthoRasterizerView::max_subblock_size());
      ImageViewRef<Vector3> ned_err = asp::error_to_NED(point_disk_image, georef);
      std::vector< ImageViewRef< PixelGray<float> > >  rasterized(3);
      for (int ch_index = 0; ch_index < 3; ch_index++){
        ImageViewRef<double> ch = select_channel(ned_err, ch_index);
        rasterizer.set_texture(ch);
        rasterizer_fsaa = generate_fsaa_raster( rasterizer, opt );
        rasterized[ch_index] =
          block_cache(rasterizer_fsaa, tile_size, opt.num_threads);
      }
      save_image(opt, asp::round_image_pixels_skip_nodata
                              (asp::combine_channels(opt.nodata_value, rasterized[0], 
                                                     rasterized[1], rasterized[2]),
                               opt.rounding_error, opt.nodata_value),
                 georef, hole_fill_len, "IntersectionErr");
    }else{
      // Note: We don't throw here. We still would like to write the
      // DRG (below) even if we can't write the error image.
      vw_out() << "The point cloud files must have an equal number of channels which "
               << "must be 4 or 6 to be able to process the intersection error.\n";
    }
  }

  // Write out a normalized version of the DEM, if requested (for debugging)
  if (opt.do_normalize) {
    int hole_fill_len = 0;
    DiskImageView< PixelGray<float> > dem_image(opt.out_prefix + "-DEM." + opt.output_file_type);
    asp::save_image(opt, apply_mask(channel_cast<uint8>(normalize(create_mask(dem_image,opt.nodata_value),
                                                                  rasterizer.bounding_box().min().z(),
                                                                  rasterizer.bounding_box().max().z(),
                                                                  0, 255)
                                                       )
                                   ),
                    georef, hole_fill_len, "DEM-normalized");
  }

  // Write DRG if the user requested and provided a texture file.
  // This must be at the end, as we may be messing with the point
  // image in irreversible ways.
  if (opt.do_ortho) {

    Stopwatch sw3;
    sw3.start();
    ImageViewRef< PixelGray<float> > texture
      = asp::form_point_cloud_composite< PixelGray<float> >
      (opt.texture_files, asp::OrthoRasterizerView::max_subblock_size());
    rasterizer.set_texture(texture);

    if (opt.ortho_hole_fill_len > 0) {

      // Need to convert the hole fill length from output image pixels
      // to input point cloud pixels as we will fill holes in the cloud
      // itself.
      int hole_fill_len = rasterizer.pc_hole_fill_len(opt.ortho_hole_fill_len);

      // Fetch a reference to the current point image
      ImageViewRef<Vector3> point_image = rasterizer.get_point_image();

      // Mask the NaNs
      ImageViewRef< PixelMask<Vector3> > point_image_mask
        = per_pixel_filter(point_image, asp::NaN2Mask<Vector3>());

      // If to grow the cloud a bit, to help hole-filling later. This should
      // not be large as it creates artifacts. The main work better
      // be done by grassfire later. 
      if (opt.ortho_hole_fill_extra_len > 0) {
        int hole_fill_mode = 2;
        int hole_fill_num_smooth_iter = 3;
        int hole_fill_extra_len
          = rasterizer.pc_hole_fill_len(opt.ortho_hole_fill_extra_len);
        point_image_mask = vw::fill_holes(point_image_mask, hole_fill_mode,
                                          hole_fill_num_smooth_iter,
                                          hole_fill_extra_len);

        // Use big tiles, to reduce the overhead
        // of expanding each tile by hole size. 
        int big_block_size = nextpow2(2.0*hole_fill_extra_len);
        big_block_size = std::max(256, big_block_size);

        // Cache each hole-filled point cloud tile as likely we will
        // need it again in the future when rasterizing a different
        // portion of the output ortho image.
        point_image_mask = block_cache(point_image_mask, 
                                       vw::Vector2(big_block_size, big_block_size),
                                       opt.num_threads);
      }

      // Fill the holes
      point_image_mask =vw::fill_holes_grass(point_image_mask, hole_fill_len);

      // back to NaNs
      point_image = per_pixel_filter(point_image_mask, asp::Mask2NaN<Vector3>());

      // When filling holes, use big tiles, to reduce the overhead
      // of expanding each tile by hole size. 
      int big_block_size = nextpow2(2.0*hole_fill_len);
      big_block_size = std::max(256, big_block_size);

      // Cache each hole-filled point cloud tile as likely we will
      // need it again in the future when rasterizing a different
      // portion of the output ortho image.
      point_image = block_cache(point_image, 
                                vw::Vector2(big_block_size, big_block_size),
                                opt.num_threads);

      // Pass to the rasterizer the point image with the holes filled
      rasterizer.set_point_image(point_image);
    }

    rasterizer_fsaa = generate_fsaa_raster(rasterizer, opt);
    asp::save_image(opt, rasterizer_fsaa, georef,
                    0, // no need for a buffer here, as we cache hole-filled tiles
                    "DRG");
    sw3.stop();
    vw_out(DebugMessage,"asp") << "DRG render time: " << sw3.elapsed_seconds() << "\n";
  }

} // End do_software_rasterization


// Wrapper for do_software_rasterization that goes through all spacing values
void do_software_rasterization_multi_spacing(const ImageViewRef<Vector3>& proj_points,
                                             Options& opt,
                                             cartography::GeoReference& georef,
                                             ImageViewRef<double> const& error_image,
                                             double estim_max_error,
                                             vw::BBox3 const& estim_proj_box) {
  // Perform the slow initialization that can be shared by all output resolutions
  Stopwatch sw1;
  sw1.start();
  vw::Mutex count_mutex; // Need to pass in by pointer due to C++ class restrictions
  size_t num_invalid_pixels = 0; // Need to pass in by pointer because we can't get back the number from
                                 //  the original rasterizer object otherwise for some reason.
  asp::OrthoRasterizerView
    rasterizer(proj_points.impl(), select_channel(proj_points.impl(),2),
               opt.search_radius_factor, opt.sigma_factor, opt.use_surface_sampling,
               asp::ASPGlobalOptions::tri_tile_size(), // to efficiently process the cloud
               opt.target_projwin,
               opt.remove_outliers_with_pct, opt.remove_outliers_params,
               error_image, estim_max_error, estim_proj_box, opt.max_valid_triangulation_error,
               opt.median_filter_params, opt.erode_len, opt.has_las_or_csv_or_pcd,
               opt.filter, opt.default_grid_size_multiplier,
               &num_invalid_pixels, &count_mutex,
               TerminalProgressCallback("asp","QuadTree: "));

  sw1.stop();
  vw_out(DebugMessage,"asp") << "Quad time: " << sw1.elapsed_seconds() << std::endl;

  // Perform other rasterizer configuration
  rasterizer.set_use_alpha(opt.has_alpha);
  rasterizer.set_use_minz_as_default(false);
  rasterizer.set_default_value(opt.nodata_value);

  std::string base_out_prefix = opt.out_prefix;

  // Call the function for each dem spacing
  for (size_t i = 0; i < opt.dem_spacing.size(); i++) {
    double this_spacing = opt.dem_spacing[i];

    // Required second init step for each spacing
    rasterizer.initialize_spacing(this_spacing);

    // Each spacing gets a variation of the output prefix
    if (i == 0)
      opt.out_prefix = base_out_prefix;
    else // Write later iterations to a different path.
      opt.out_prefix = base_out_prefix + "_" + vw::num_to_str(i);
    do_software_rasterization(rasterizer, opt, georef, error_image,
                              estim_max_error, &num_invalid_pixels);
  } // End loop through spacings

  opt.out_prefix = base_out_prefix; // Restore the original value
}

// Sample the image and get generous estimates (but without outliers)
// of the maximum triangulation error and of the 3D box containing the
// projected points. These will be tightened later.
double estim_max_tri_error_and_proj_box(ImageViewRef<Vector3> const& proj_points,
                                        ImageViewRef<double> const& error_image,
                                        Vector2 const& remove_outliers_params,
                                        BBox3 & estim_proj_box) {

  // Initialize the outputs
  double estim_max_error = 0.0;
  estim_proj_box = BBox3();

  // Start with a 256 (2^8) by 256 sampling of the cloud
  bool success = false;
  for (int attempt = 8; attempt <= 18; attempt++){
    
    double sample = (1 << attempt);
    int32 subsample_amt = int32(norm_2(Vector2(error_image.cols(), error_image.rows()))/sample);
    if (subsample_amt < 1 )
      subsample_amt = 1;
    
    Stopwatch sw2;
    sw2.start();
    PixelAccumulator<asp::ErrorRangeEstimAccum> error_accum;
    for_each_pixel(subsample(error_image, subsample_amt),
                   error_accum,
                   TerminalProgressCallback
                   ("asp","Bounding box and triangulation error range estimation: ") );
    if (error_accum.size() > 0){
      success = true;
      estim_max_error = error_accum.value(remove_outliers_params);
    }
    sw2.stop();

    asp::estimate_points_bdbox(subsample(proj_points, subsample_amt),  
                               remove_outliers_params,  
                               estim_proj_box);
    
    if (estim_proj_box.empty()) 
      success = false;
    
    vw_out(DebugMessage,"asp") << "Elapsed time: " << sw2.elapsed_seconds() << std::endl;
    if (success || subsample_amt == 1) break;
    vw_out() << "Estimation failed. Check if your cloud is valid. "
             << "Trying again with finer sampling.\n";
  }
  return estim_max_error;
}

//-----------------------------------------------------------------------------------

int main( int argc, char *argv[] ) {
  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Set up the georeferencing information.  We specify everything
    // here except for the affine transform, which is defined later once
    // we know the bounds of the orthorasterizer view.  However, we can
    // still reproject the points in the point image without the affine
    // transform because this projection never requires us to convert to
    // or from pixel space.
    GeoReference output_georef;

    // See if we can get a georef from any of the input pc files
    GeoReference pc_georef;
    bool has_pc_georef = asp::georef_from_pc_files(opt.pointcloud_files, pc_georef);
    if (has_pc_georef)
      output_georef = pc_georef;

    // See if the user specified the datum outside of the srs string
    cartography::Datum user_datum;
    bool have_user_datum = asp::read_user_datum(opt.semi_major, opt.semi_minor,
                                                opt.datum, user_datum);

    // If the data was left in cartesian coordinates, we need to give
    // the DEM a projection that uses some physical units (meters),
    // rather than lon, lat. Otherwise, we honor the user's requested
    // projection and convert the points if necessary.
    if (opt.target_srs_string.empty()) {

      if (have_user_datum)
        output_georef.set_datum( user_datum );

      switch( opt.projection ) {
      case SINUSOIDAL:           output_georef.set_sinusoidal           (opt.proj_lon,                               opt.false_easting, opt.false_northing); break;
      case MERCATOR:             output_georef.set_mercator             (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
      case TRANSVERSEMERCATOR:   output_georef.set_transverse_mercator  (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
      case ORTHOGRAPHIC:         output_georef.set_orthographic         (opt.proj_lat, opt.proj_lon,                 opt.false_easting, opt.false_northing); break;
      case STEREOGRAPHIC:        output_georef.set_stereographic        (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
      case OSTEREOGRAPHIC:       output_georef.set_oblique_stereographic(opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
      case GNOMONIC:             output_georef.set_gnomonic             (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
      case LAMBERTAZIMUTHAL:     output_georef.set_lambert_azimuthal    (opt.proj_lat, opt.proj_lon,                 opt.false_easting, opt.false_northing); break;
      case UTM:                  output_georef.set_UTM( opt.utm_zone ); break;
      default: // Handles plate carree
        break;
      }
    } else { // The user specified the target srs_string

      // Set the srs string into georef.
      asp::set_srs_string(opt.target_srs_string, have_user_datum, user_datum, output_georef);
    }

    // Convert any input LAS or CSV files to ASP's point cloud tif format
    // - The output and input datum will match unless the input data files
    //   themselves specify a different datum.
    // - Should all be XYZ format when finished
    std::vector<std::string> tmp_tifs;
    las_or_csv_or_pcd_to_tifs(opt, output_georef.datum(), tmp_tifs);

    // Generate a merged xyz point cloud consisting of all inputs
    // - By now, each input exists in xyz tif format.
    ImageViewRef<Vector3> point_image
      = asp::form_point_cloud_composite<Vector3>(opt.pointcloud_files,
                                                 asp::OrthoRasterizerView::max_subblock_size());
    
    // Apply an (optional) rotation to the 3D points before building the mesh.
    if (opt.phi_rot != 0 || opt.omega_rot != 0 || opt.kappa_rot != 0) {
      vw_out() << "\t--> Applying rotation sequence: " << opt.rot_order
               << "      Angles: " << opt.phi_rot << "   "
               << opt.omega_rot << "  " << opt.kappa_rot << "\n";
      point_image = asp::point_transform
        (point_image, math::euler_to_rotation_matrix(opt.phi_rot, opt.omega_rot,
                                                     opt.kappa_rot, opt.rot_order));
    }

    // Set up the error image
    ImageViewRef<double> error_image;
    if (opt.remove_outliers_with_pct || opt.max_valid_triangulation_error > 0.0){
      int num_channels = asp::num_channels(opt.pointcloud_files);

      if      (num_channels == 4) {
        error_image = asp::error_norm<4>(opt.pointcloud_files);
      } else if (num_channels == 6) {
        error_image = asp::error_norm<6>(opt.pointcloud_files);
      } else {
        vw_out() << "The point cloud files must have an equal number of channels which "
                 << "must be 4 or 6 to be able to remove outliers.\n";
        opt.remove_outliers_with_pct      = false;
        opt.max_valid_triangulation_error = 0.0;
      }
    }
    
    // Determine if we should be using a longitude range between
    // [-180, 180] or [0,360]. We determine this by looking at the
    // average location of the points. If the average location has a
    // negative x value (think in ECEF coordinates) then we should
    // be using [0,360].
    double avg_lon = asp::find_avg_lon(point_image);

    // TODO: Do we need the recenter code now that we have this?
    // TODO: Modify other code so we don't have to handle this one special case!
    // Forcing the georef object outside its comfort zone is not safe for all projections!
    if (output_georef.overall_proj4_str().find("+proj=aea") == std::string::npos)
      output_georef.set_lon_center(avg_lon < 100);

    // Convert xyz points to projected points
    // - The cartesian_to_geodetic call converts invalid (0,0,0,0) points to NaN,
    //   which is checked for in the OrthoRasterizer class.
    ImageViewRef<Vector3> proj_points;
    if (opt.lon_offset != 0 || opt.lat_offset != 0 || opt.height_offset != 0) {
      vw_out() << "\t--> Applying offset: " << opt.lon_offset
               << " " << opt.lat_offset << " " << opt.height_offset << "\n";
      proj_points
        = geodetic_to_point         // GDC to XYZ
        (asp::point_image_offset    // Add user coordinate offset
         (asp::recenter_longitude   // XYZ to GDC, then normalize longitude
          (cartesian_to_geodetic(point_image, output_georef), 
           avg_lon),
          Vector3(opt.lon_offset, opt.lat_offset, opt.height_offset)
          ),
         output_georef);
    } else {
      proj_points = geodetic_to_point(asp::recenter_longitude
                                      (cartesian_to_geodetic(point_image, output_georef),
                                       avg_lon),
                                      output_georef);
    }

    double estim_max_error = 0.0;
    BBox3 estim_proj_box;
    if (error_image.rows() > 0 && error_image.cols() > 0) {

      if (error_image.cols() != point_image.cols() || error_image.rows() != point_image.rows()) 
        vw_throw(ArgumentErr() << "The error image and point image must have the same size.");

      estim_max_error = estim_max_tri_error_and_proj_box(proj_points, error_image,
                                                         opt.remove_outliers_params,
                                                         estim_proj_box);
    }

    // Create the DEM
    do_software_rasterization_multi_spacing(proj_points, opt, output_georef, error_image,
                                            estim_max_error, estim_proj_box);
    
    // Wipe the temporary files
    for (int i = 0; i < (int)tmp_tifs.size(); i++)
      if (fs::exists(tmp_tifs[i])) fs::remove(tmp_tifs[i]);
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
