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

// \file point2dem.cc
//

#include <asp/Core/DemUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/OrthoRasterizer.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/OutlierProcessing.h>
#include <asp/Core/PointCloudProcessing.h>

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
using namespace asp;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// TODO: Move this somewhere?
// Parses a string containing a list of numbers
void split_number_string(const std::string &input, std::vector<double> &output) {

  // Get a space delimited string
  std::string delimiter = " ";
  std::string s = input;
  std::replace(s.begin(), s.end(), ',', ' ');

  double val;
  std::stringstream stream(s);
  while (stream >> val) {
    output.push_back(val);
  }
}

void handle_arguments(int argc, char *argv[], DemOptions& opt) {

  std::string dem_spacing1, dem_spacing2;

  po::options_description manipulation_options("Manipulation options");
  manipulation_options.add_options()
    ("x-offset",       po::value(&opt.lon_offset)->default_value(0), 
     "Add a longitude offset (in degrees) to the DEM.")
    ("y-offset",       po::value(&opt.lat_offset)->default_value(0), 
     "Add a latitude offset (in degrees) to the DEM.")
    ("z-offset",       po::value(&opt.height_offset)->default_value(0), 
     "Add a vertical offset (in meters) to the DEM.")
    ("rotation-order", po::value(&opt.rot_order)->default_value("xyz"),
      "Set the order of an Euler angle rotation applied to the 3D points prior to DEM rasterization.")
    ("phi-rotation",   po::value(&opt.phi_rot )->default_value(0),
     "Set a rotation angle phi.")
    ("omega-rotation", po::value(&opt.omega_rot)->default_value(0),
     "Set a rotation angle omega.")
    ("kappa-rotation", po::value(&opt.kappa_rot)->default_value(0),
     "Set a rotation angle kappa.");

  po::options_description projection_options("Projection options");
  projection_options.add_options()
    ("t_srs",         po::value(&opt.target_srs_string)->default_value(""), 
     "Specify the output projection as a GDAL projection string (WKT, GeoJSON, or PROJ.4). If not provided, will be read from the point cloud, if available.")
    ("t_projwin",     po::value(&opt.target_projwin),
     "The output DEM will have corners with these georeferenced coordinates. The actual spatial extent (ground footprint) is obtained by expanding this box by half the grid size.")
    ("dem-spacing,s", po::value(&dem_spacing1)->default_value(""),
      "Set output DEM resolution (in target georeferenced units per pixel). These units may be in degrees or meters, depending on your projection. If not specified, it will be computed automatically (except for LAS and CSV files). Multiple spacings can be set (in quotes) to generate multiple output files. This is the same as the --tr option.")
    ("tr", po::value(&dem_spacing2)->default_value(""), "This is identical to the --dem-spacing option.")
    ("datum", po::value(&opt.datum)->default_value(""),
     "Set the datum. This will override the datum from the input images and also --t_srs, --semi-major-axis, and --semi-minor-axis. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("reference-spheroid,r", po::value(&opt.reference_spheroid)->default_value(""),
     "This is identical to the datum option.")
    ("semi-major-axis",      po::value(&opt.semi_major)->default_value(0),
     "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis", po::value(&opt.semi_minor)->default_value(0),
     "Explicitly set the datum semi-minor axis in meters.")
    ("sinusoidal",        
       "Save using a sinusoidal projection.")
    ("mercator",            
     "Save using a Mercator projection.")
    ("transverse-mercator", 
     "Save using a transverse Mercator projection.")
    ("orthographic",        
     "Save using an orthographic projection.")
    ("stereographic",       
     "Save using a stereographic projection. See also --auto-proj-center.")
    ("oblique-stereographic", 
     "Save using an oblique stereographic projection.")
    ("gnomonic",            
     "Save using a gnomonic projection.")
    ("lambert-azimuthal",   
     "Save using a Lambert azimuthal projection.")
    ("utm",        po::value(&opt.utm_zone),
     "Save using a UTM projection with the given zone.")
    ("proj-lat",   po::value(&opt.proj_lat)->default_value(0),
     "The center of projection latitude (if applicable).")
    ("proj-lon",   po::value(&opt.proj_lon)->default_value(0),
     "The center of projection longitude (if applicable).")
    ("auto-proj-center", po::bool_switch(&opt.auto_proj_center)->default_value(false),
     "Automatically compute the projection center, when the projection is "
     "stereographic, etc. Use the median longitude and latitude of cloud "
     "points. This overrides the values of --proj-lon and --proj-lat.")
    ("proj-scale", po::value(&opt.proj_scale)->default_value(1),
     "The projection scale (if applicable).")
    ("false-easting", po::value(&opt.false_easting)->default_value(0),
     "The projection false easting (if applicable).")
    ("false-northing", po::value(&opt.false_northing)->default_value(0),
     "The projection false northing (if applicable).");

  po::options_description general_options("General options");
  general_options.add_options()
    ("nodata-value",      po::value(&opt.nodata_value)->default_value(-std::numeric_limits<float>::max()),
             "Set the nodata value.")
    ("use-alpha",         po::bool_switch(&opt.has_alpha)->default_value(false),
             "Create images that have an alpha channel.")
    ("normalized,n",      po::bool_switch(&opt.do_normalize)->default_value(false),
             "Also write a normalized version of the DEM (for debugging).")
    ("orthoimage",        po::bool_switch(&opt.do_ortho)->default_value(false),
             "Write an orthoimage based on the texture files passed in as inputs "
             "(after the point clouds).")
    ("output-prefix,o",   po::value(&opt.out_prefix),
     "Specify the output prefix.")
    ("output-filetype,t", po::value(&opt.output_file_type)->default_value("tif"), "Specify the output file.")
    ("errorimage",        po::bool_switch(&opt.do_error)->default_value(false),
    "Write an additional image, whose values represent the triangulation ray "
    "intersection error in meters (the closest distance between the rays "
    "emanating from the two cameras corresponding to the same point on the "
    "ground). Filename is <output prefix>-IntersectionErr.tif. If stereo "
    "triangulation was done with the option --compute-error-vector, this "
    "intersection error will instead have 3 bands, corresponding to the "
    "coordinates of that vector, unless the option --scalar-error is set.")
    ("scalar-error", po::bool_switch(&opt.scalar_error)->default_value(false),
     "If the point cloud has a vector triangulation error, ensure that "
     "the intersection error produced by this program is the rasterized "
     "norm of that vector. See also --error-image.")
    ("dem-hole-fill-len", po::value(&opt.dem_hole_fill_len)->default_value(0),
     "Maximum dimensions of a hole in the output DEM to fill in, in pixels.")
    ("orthoimage-hole-fill-len",      po::value(&opt.ortho_hole_fill_len)->default_value(0),
     "Maximum dimensions of a hole in the output orthoimage to fill in, in pixels.")
    ("orthoimage-hole-fill-extra-len", po::value(&opt.ortho_hole_fill_extra_len)->default_value(0),
     "This value, in pixels, will make orthoimage hole filling more aggressive by first extrapolating the point cloud. A small value is suggested to avoid artifacts. Hole-filling also works better when less strict with outlier removal, such as in --remove-outliers-params, etc.")
    ("remove-outliers", po::bool_switch(&opt.remove_outliers_with_pct)->default_value(true),
      "Turn on outlier removal based on percentage of triangulation error. Obsolete, as this is the default.")
    ("remove-outliers-params",        po::value(&opt.remove_outliers_params)->default_value(Vector2(75.0, 3.0), "pct factor"),
      "Outlier removal based on percentage. Points with triangulation error larger than pct-th percentile times factor and points too far from the cluster of most points will be removed as outliers. [default: pct=75.0, factor=3.0]")
    ("use-tukey-outlier-removal", po::bool_switch(&opt.use_tukey_outlier_removal)->default_value(false)->implicit_value(true),
     "Remove outliers above Q3 + 1.5*(Q3 - Q1). Takes precedence over --remove-outlier-params.")
    ("max-valid-triangulation-error", po::value(&opt.max_valid_triangulation_error)->default_value(0),
      "Outlier removal based on threshold. If positive, points with triangulation error larger than this will be removed from the cloud. Measured in meters. This option takes precedence over --remove-outliers-params and --use-tukey-outlier-removal.")
    ("max-output-size", po::value(&opt.max_output_size)->default_value(Vector2(9999999, 9999999)),
      "Don't write the output DEM if it is calculated to be this size or greater.")
    ("median-filter-params", po::value(&opt.median_filter_params)->default_value(Vector2(0, 0), "window_size threshold"), 
     "If the point cloud height at the current point differs by more than the given threshold from the median of heights in the window of given size centered at the point, remove it as an outlier. Use for example 11 and 40.0.")
    ("erode-length",   po::value<int>(&opt.erode_len)->default_value(0),
     "Erode input point clouds by this many pixels at boundary (after outliers are removed, but before filling in holes).")
    ("csv-format",     po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4",      po::value(&opt.csv_proj4_str)->default_value(""), 
     "The PROJ.4 string to use to interpret the entries in input CSV files, if those files contain Easting and Northing fields. If not specified, --t_srs will be used.")
    ("filter",      po::value(&opt.filter)->default_value("weighted_average"), 
     "The filter to apply to the heights of the cloud points within a given circular neighborhood when gridding (its radius is controlled via --search-radius-factor). Options: weighted_average (default), min, max, mean, median, stddev, count (number of points), nmad (= 1.4826 * median(abs(X - median(X)))), n-pct (where n is a real value between 0 and 100, for example, 80-pct, meaning, 80th percentile). Except for the default, the name of the filter will be added to the obtained DEM file name, e.g., output-min-DEM.tif.")
    ("rounding-error", po::value(&opt.rounding_error)->default_value(asp::APPROX_ONE_MM),
     "How much to round the output DEM and errors, in meters (more rounding means less precision but potentially smaller size on disk). The inverse of a power of 2 is suggested. Default: 1/2^10.")
    ("search-radius-factor", po::value(&opt.search_radius_factor)->default_value(0.0),
     "Multiply this factor by dem-spacing to get the search radius. The DEM height at a given grid point is obtained as a weighted average of heights of all points in the cloud within search radius of the grid point, with the weights given by a Gaussian. Default search radius: max(dem-spacing, default_dem_spacing), so the default factor is about 1.")
    ("propagate-errors", po::bool_switch(&opt.propagate_errors)->default_value(false),  
     "Write files with names {output-prefix}-HorizontalStdDev.tif and {output-prefix}-VerticalStdDev.tif having the gridded stddev produced from bands 5 and 6 of the input point cloud, if this cloud was created with the option --propagate-errors. The same gridding algorithm is used as for creating the DEM.")
    ("gaussian-sigma-factor", po::value(&opt.sigma_factor)->default_value(0.0),
     "The value s to be used in the Gaussian exp(-s*(x/grid_size)^2) when computing the DEM. The default is -log(0.25) = 1.3863. A smaller value will result in a smoother terrain.")
    ("default-grid-size-multiplier", po::value(&opt.default_grid_size_multiplier)->default_value(1.0),
     "If the output DEM grid size (--dem-spacing) is not specified, compute it automatically (as the mean ground sample distance), and then multiply it by this number. It is suggested that this number be set to 4 though the default is 1.")
    ("use-surface-sampling", po::bool_switch(&opt.use_surface_sampling)->default_value(false),
     "Use the older algorithm, interpret the point cloud as a surface made up of triangles and interpolate into it (prone to aliasing).")
    ("fsaa",   po::value<int>(&opt.fsaa)->default_value(1),
     "Oversampling amount to perform antialiasing (obsolete).")
    ("no-dem", po::bool_switch(&opt.no_dem)->default_value(false),
     "Skip writing a DEM.")
    ("input-is-projected", po::bool_switch(&opt.input_is_projected)->default_value(false), 
     "Input data is already in projected coordinates, or is a point cloud in Cartesian "
     "coordinates that is small in extent. Need not be spatially organized. "
     "If both a top and bottom surface exists, one of them must be cropped out. "
     "Point (0, 0, 0) is considered invalid. Must specify a projection to interpret "
     "the data and the output grid size.");

  general_options.add(manipulation_options);
  general_options.add(projection_options);
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value< std::vector<std::string> >(), "Input files");

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <point-clouds> [ --orthoimage <textures> ]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (vm.count("input-files") == 0)
    vw_throw(ArgumentErr() << "Missing input point clouds.\n" << usage << general_options);
  std::vector<std::string> input_files = vm["input-files"].as< std::vector<std::string> >();
  
  parse_input_clouds_textures(input_files, opt);

  if (opt.median_filter_params[0] < 0 || opt.median_filter_params[1] < 0){
    vw_throw(ArgumentErr() << "The parameters for median-based filtering "
                            << "must be non-negative.\n" << usage << general_options);
  }

  // This is a bug fix. The user by mistake passed in an empty projection string.
  if (!vm["t_srs"].defaulted() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr() 
             << "The value of --t_srs is empty. Then it must not be set at all.\n");
  
  if (opt.has_las_or_csv_or_pcd && opt.median_filter_params[0] > 0 &&
      opt.median_filter_params[1] > 0){
    vw_throw(ArgumentErr() 
             << "Median-based filtering cannot handle CSV or LAS files.\n"
             << usage << general_options);
  }

  if (opt.erode_len < 0){
    vw_throw(ArgumentErr() << "Erode length must be non-negative.\n"
                            << usage << general_options);
  }

  if ((dem_spacing1.size() > 0) && (dem_spacing2.size() > 0)){
    vw_throw(ArgumentErr() << "The DEM spacing was specified twice.\n"
                            << usage << general_options);
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
      vw_throw(ArgumentErr() << "The DEM spacing must be non-negative.\n" << usage << general_options);
    }
    if (opt.dem_spacing[i] > 0)
      spacing_provided = true;
  }

  if (opt.has_las_or_csv_or_pcd && !spacing_provided){
    vw_throw(ArgumentErr() << "When inputs are not PC.tif files, the "
                            << "output DEM resolution must be set.\n");
  }

  if (opt.out_prefix.empty())
    opt.out_prefix = asp::prefix_from_pointcloud_filename(opt.pointcloud_files[0]);

  if (opt.use_surface_sampling){
    vw_out(WarningMessage) 
          << "The --use-surface-sampling option invokes the old algorithm and "
          << "is obsolete, it will be removed in future versions.\n";
  }

  if (opt.use_surface_sampling && opt.filter != "weighted_average")
    vw_throw(ArgumentErr() 
             << "Cannot use surface "
             << "sampling with any filter of point cloud points.\n");

  if (opt.use_surface_sampling && opt.has_las_or_csv_or_pcd)
    vw_throw(ArgumentErr() << "Cannot use surface " << "sampling with LAS or CSV files.\n");

  if (opt.fsaa != 1 && !opt.use_surface_sampling){
    vw_throw(ArgumentErr() 
             << "The --fsaa option is obsolete. It can be used only with the "
             << "--use-surface-sampling option which invokes the old algorithm.\n" 
             << usage << general_options);
  }

  if (opt.dem_hole_fill_len < 0)
    vw_throw(ArgumentErr() << "The value of --dem-hole-fill-len must be non-negative.\n");
  if (opt.ortho_hole_fill_len < 0)
    vw_throw(ArgumentErr() << "The value of --orthoimage-hole-fill-len must be non-negative.\n");
  if (opt.ortho_hole_fill_extra_len < 0)
    vw_throw(ArgumentErr() << "The value of --orthoimage-hole-fill-extra-len must be non-negative.\n");
  if (!opt.do_ortho && opt.ortho_hole_fill_len > 0)
    vw_throw(ArgumentErr() << "The value of --orthoimage-hole-fill-len is positive, "
                           << "but orthoimage generation was not requested.\n");

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
  if (pct <= 0.0 || pct > 100.0 || factor <= 0.0){
    vw_throw(ArgumentErr()
              << "Invalid values were provided for outlier removal params.\n");
  }

  if (opt.input_is_projected && opt.auto_proj_center)
    vw_throw(ArgumentErr() << "Cannot use both --input-is-projected and "
             << "--auto-proj-center.\n");
  
  if (opt.max_valid_triangulation_error > 0) {
    // Since the user passed in a threshold, will use that to rm
    // outliers, instead of using the percentage.
    opt.remove_outliers_with_pct = false;
    opt.use_tukey_outlier_removal = false;
  }

  // For compatibility with GDAL, we allow the proj win y coordinate to be flipped.
  // Correct that here.
  if (opt.target_projwin != BBox2()) {
    if (opt.target_projwin.min().y() > opt.target_projwin.max().y()) {
      std::swap(opt.target_projwin.min().y(), opt.target_projwin.max().y());
    }
    vw_out() << "Cropping to " << opt.target_projwin << " pt. " << std::endl;
  }

  // If the user specified a PROJ.4 string to use to interpret the
  // input in CSV files, use the same string to create output DEMs,
  // unless the user explicitly sets the output PROJ.4 string.
  if (!opt.csv_proj4_str.empty() && opt.target_srs_string.empty()) {
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
    vw_throw(ArgumentErr() << "Both --datum and --reference-spheroid were specified.\n");
  if (opt.datum == "")
    opt.datum = opt.reference_spheroid;

  if      (vm.count("sinusoidal"))            opt.projection = SINUSOIDAL;
  else if (vm.count("mercator"))              opt.projection = MERCATOR;
  else if (vm.count("transverse-mercator"))   opt.projection = TRANSVERSEMERCATOR;
  else if (vm.count("orthographic"))          opt.projection = ORTHOGRAPHIC;
  else if (vm.count("stereographic"))         opt.projection = STEREOGRAPHIC;
  else if (vm.count("oblique-stereographic")) opt.projection = OSTEREOGRAPHIC;
  else if (vm.count("gnomonic"))              opt.projection = GNOMONIC;
  else if (vm.count("lambert-azimuthal"))     opt.projection = LAMBERTAZIMUTHAL;
  else if (vm.count("utm"))                   opt.projection = UTM;
  else                                        opt.projection = PLATECARREE; // Default

  // Sanity check
  if (opt.auto_proj_center && opt.projection == PLATECARREE)
    vw::vw_throw(ArgumentErr() 
                 << "No projection was set (such as --stereographic). "
                 << "Cannot use --auto-proj-center.\n");
} // end function handle_arguments()

// Wrapper for do_software_rasterization that goes through all spacing values
void do_software_rasterization_multi_spacing(const ImageViewRef<Vector3>& proj_points,
                                             DemOptions& opt,
                                             cartography::GeoReference& georef,
                                             ImageViewRef<double> const& error_image,
                                             double estim_max_error,
                                             vw::BBox3 const& estim_proj_box) {

  asp::OutlierRemovalMethod outlier_removal_method = asp::NO_OUTLIER_REMOVAL_METHOD;
  if (opt.remove_outliers_with_pct)
    outlier_removal_method = asp::PERCENTILE_OUTLIER_METHOD;
  if (opt.use_tukey_outlier_removal) 
    outlier_removal_method = asp::TUKEY_OUTLIER_METHOD; // takes precedence
  
  // Perform the slow initialization that can be shared by all output resolutions
  Stopwatch sw1;
  sw1.start();
  vw::Mutex count_mutex; // Need to pass in by pointer due to C++ class restrictions
  
  // Need to pass in by pointer because we can't get back the number from
  //  the original rasterizer object otherwise for some reason.
  std::int64_t num_invalid_pixels = 0;
  asp::OrthoRasterizerView
    rasterizer(proj_points.impl(), select_channel(proj_points.impl(),2),
               opt.search_radius_factor, opt.sigma_factor, opt.use_surface_sampling,
               asp::ASPGlobalOptions::tri_tile_size(), // to efficiently process the cloud
               opt.target_projwin,
               outlier_removal_method, opt.remove_outliers_params,
               error_image, estim_max_error, estim_proj_box, opt.max_valid_triangulation_error,
               opt.median_filter_params, opt.erode_len, opt.has_las_or_csv_or_pcd,
               opt.filter, opt.default_grid_size_multiplier,
               &num_invalid_pixels, &count_mutex,
               TerminalProgressCallback("asp", "Point cloud extent estimation: "));

  sw1.stop();
  vw_out(DebugMessage,"asp") << "Extent estimation time: " 
    << sw1.elapsed_seconds() << " s.\n";
  
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
    do_software_rasterization(rasterizer, opt, georef, 
                              &num_invalid_pixels);
  } // End loop through spacings

  opt.out_prefix = base_out_prefix; // Restore the original value
}

// Set the projection based on options
void set_projection(DemOptions const& opt, cartography::GeoReference & output_georef) {
  
  // Can set a projection either via a string or via options
  if (!opt.target_srs_string.empty())
    vw::vw_throw(ArgumentErr()
                << "The --t_srs option must not be used when setting a projection.\n");

  switch (opt.projection) {
    case SINUSOIDAL:           output_georef.set_sinusoidal           (opt.proj_lon,                               opt.false_easting, opt.false_northing); break;
    case MERCATOR:             output_georef.set_mercator             (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
    case TRANSVERSEMERCATOR:   output_georef.set_transverse_mercator  (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
    case ORTHOGRAPHIC:         output_georef.set_orthographic         (opt.proj_lat, opt.proj_lon,                 opt.false_easting, opt.false_northing); break;
    case STEREOGRAPHIC:        output_georef.set_stereographic        (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
    case OSTEREOGRAPHIC:       output_georef.set_oblique_stereographic(opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
    case GNOMONIC:             output_georef.set_gnomonic             (opt.proj_lat, opt.proj_lon, opt.proj_scale, opt.false_easting, opt.false_northing); break;
    case LAMBERTAZIMUTHAL:     output_georef.set_lambert_azimuthal    (opt.proj_lat, opt.proj_lon,                 opt.false_easting, opt.false_northing); break;
    case UTM:                  output_georef.set_UTM(opt.utm_zone); break;
    default: // lonlat projection
      break;
  }
  
  return;
} 

int main(int argc, char *argv[]) {
  DemOptions opt;
  try {
    handle_arguments(argc, argv, opt);

    // Set up the georeferencing information. We specify everything
    // here except for the affine transform, which is defined later once
    // we know the bounds of the orthorasterizer view.  However, we can
    // still reproject the points in the point image without the affine
    // transform because this projection never requires us to convert to
    // or from pixel space.
    GeoReference output_georef;

    // See if we can get a georef from any of the input pc files
    GeoReference pc_georef;
    bool have_input_georef = asp::georef_from_pc_files(opt.pointcloud_files, pc_georef);
    if (have_input_georef)
      output_georef = pc_georef;

    // See if the user specified the datum outside of the srs string
    cartography::Datum user_datum;
    bool have_user_datum = asp::read_user_datum(opt.semi_major, opt.semi_minor,
                                                opt.datum, user_datum);

    // If the data was left in cartesian coordinates, we need to give
    // the DEM a projection that uses some physical units (meters),
    // rather than lon, lat. Otherwise, we honor the user's requested
    // projection and convert the points if necessary.
    // Do not quietly assume an Earth datum, the user must set it.
    if (opt.target_srs_string.empty()) {
      if (have_user_datum)
        output_georef.set_datum(user_datum);
      else if (!have_input_georef && opt.datum == "")
        vw::vw_throw(vw::ArgumentErr()
                     << "A datum, projection, or semi-axes must be set.\n");
      set_projection(opt, output_georef);
    } else {
      // Set the user-specified the target srs_string into georef
      asp::set_srs_string(opt.target_srs_string, have_user_datum, user_datum,
                          have_input_georef, output_georef);
    }

    // If the input PROJ.4 string is empty, use the output one. 
    if (!opt.csv_format_str.empty() && opt.csv_proj4_str.empty()) {
      opt.csv_proj4_str = output_georef.get_wkt();
      vw_out() << "The option --csv-proj4 was not specified. Using the output projection "
               << "when interpreting csv files.\n";
    }
    
    // Convert any input LAS, CSV, PCD, or unorganized projected TIF files to
    // ASP's point cloud tif format
    // - The output and input datum will match unless the input data files
    //   themselves specify a different datum.
    // - Should all be XYZ format when finished, unless option 
    //  --input-is-projected is set.
    std::vector<std::string> tmp_tifs;
    chip_convert_to_tif(opt, output_georef.datum(), tmp_tifs);

    // Generate a merged xyz point cloud consisting of all inputs
    // - By now, each input exists in xyz tif format.
    ImageViewRef<Vector3> point_image
      = asp::form_point_cloud_composite<Vector3>(opt.pointcloud_files,
                                                 ASP_MAX_SUBBLOCK_SIZE);
    
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
    if (opt.remove_outliers_with_pct || opt.use_tukey_outlier_removal ||
        opt.max_valid_triangulation_error > 0.0){
      error_image = asp::point_cloud_error_image(opt.pointcloud_files);
      
      if (error_image.rows() == 0 || error_image.cols() == 0) {
        vw_out() << "The point cloud files must have an equal number of channels which "
                 << "must be 4 or 6 to be able to remove outliers.\n";
        opt.remove_outliers_with_pct      = false;
        opt.use_tukey_outlier_removal     = false;
        opt.max_valid_triangulation_error = 0.0;
      }
    }
    
    // TODO(oalexan1): Do we need anywhere here the lon-lat window?
    // E.g.: georef.ll_box_from_pix_box(image_bbox);
    
    // Determine if we should be using a longitude range between
    // [-180, 180] or [0,360]. The former is used, unless the latter
    // results in a tighter range of longitudes, such as when crossing
    // the international date line.
    vw::BBox2 lonlat_box = asp::estim_lonlat_box(point_image, output_georef.datum());
    output_georef.set_image_ll_box(lonlat_box);
    
    // TODO: Do we need the recenter code now that we have this?
    // TODO: Modify other code so we don't have to handle this one special case!
    // Forcing the georef object outside its comfort zone is not safe for all projections!

    if (opt.auto_proj_center) {
      // Find the median lon lat and reapply this to the georef. Must be done
      // after estimating the lonlat box.
      asp::median_lon_lat(point_image, output_georef, opt.proj_lon, opt.proj_lat);
      set_projection(opt, output_georef);
    }
    
    // Convert xyz points to projected points
    // - The cartesian_to_geodetic call converts invalid (0,0,0,0) points to NaN,
    //   which is checked for in the OrthoRasterizer class.
    ImageViewRef<Vector3> proj_points;

    if (opt.input_is_projected) {
      vw_out() << "\t--> Assuming the input cloud is already projected.\n";
      proj_points = point_image;
    } else {
      // TODO(oalexan1): Wipe the --x-offset, etc, not used.
      if (opt.lon_offset != 0 || opt.lat_offset != 0 || opt.height_offset != 0) {
        vw_out() << "\t--> Applying offset: " << opt.lon_offset
                << " " << opt.lat_offset << " " << opt.height_offset << "\n";
        proj_points
          = geodetic_to_point         // GDC to XYZ
          (asp::point_image_offset    // Add user coordinate offset
            (cartesian_to_geodetic(point_image, output_georef),
            Vector3(opt.lon_offset, opt.lat_offset, opt.height_offset)
          ),
          output_georef);
      } else {
        proj_points = geodetic_to_point(cartesian_to_geodetic(point_image, output_georef),
                                        output_georef);
      }
    }

    // Estimate the proj box size, and the max intersection error (if having an error iamge)
    double estim_max_error = 0.0;
    BBox3 estim_proj_box;
    estim_max_error = asp::estim_max_tri_error_and_proj_box(proj_points, error_image,
                                                         opt.remove_outliers_params,
                                                         estim_proj_box);
    // Create the DEM
    do_software_rasterization_multi_spacing(proj_points, opt, output_georef, error_image,
                                            estim_max_error, estim_proj_box);
    // Wipe the temporary files
    for (int i = 0; i < (int)tmp_tifs.size(); i++)
      if (fs::exists(tmp_tifs[i])) 
        fs::remove(tmp_tifs[i]);
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
