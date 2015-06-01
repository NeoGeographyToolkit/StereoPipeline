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
#include <asp/Core/AntiAliasing.h>
#include <asp/Core/InpaintView.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Cartography/PointImageManipulation.h>

#include <boost/math/special_functions/fpclassify.hpp>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

enum ProjectionType {
  SINUSOIDAL,
  MERCATOR,
  TRANSVERSEMERCATOR,
  ORTHOGRAPHIC,
  STEREOGRAPHIC,
  LAMBERTAZIMUTHAL,
  UTM,
  PLATECARREE
};

struct Options : asp::BaseOptions {
  // Input
  std::vector<std::string> pointcloud_files, texture_files;

  // Settings
  float dem_spacing, nodata_value;
  double semi_major, semi_minor;
  std::string reference_spheroid;
  double phi_rot, omega_rot, kappa_rot;
  std::string rot_order;
  double proj_lat, proj_lon, proj_scale;
  double x_offset, y_offset, z_offset;
  size_t utm_zone;
  ProjectionType projection;
  bool has_nodata_value, has_alpha, do_normalize, do_ortho, do_error, no_dem;
  double rounding_error;
  std::string target_srs_string;
  BBox2 target_projwin;
  BBox2i target_projwin_pixels;
  int fsaa, dem_hole_fill_len, ortho_hole_fill_len;
  bool remove_outliers_with_pct;
  Vector2 remove_outliers_params;
  double max_valid_triangulation_error;
  Vector2 median_filter_params;
  int erode_len;
  std::string csv_format_str, csv_proj4_str;
  double search_radius_factor;
  bool use_surface_sampling;
  bool has_las_or_csv;

  // Output
  std::string  out_prefix, output_file_type;

  // Defaults that the user doesn't need to see. (The Magic behind the
  // curtain).
  Options() : dem_spacing(0), nodata_value(std::numeric_limits<float>::quiet_NaN()),
              semi_major(0), semi_minor(0), fsaa(1),
              dem_hole_fill_len(0), ortho_hole_fill_len(0),
              remove_outliers_with_pct(true), max_valid_triangulation_error(0),
              erode_len(0), search_radius_factor(0), use_surface_sampling(false),
              has_las_or_csv(false){}
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
    if (asp::is_las_or_csv(files[i]) || get_num_channels(files[i]) >= 3)
      opt.pointcloud_files.push_back(files[i]);
    else
      opt.texture_files.push_back(files[i]);
  }

  // Must have this check here before we start assuming all input files
  // are tif.
  opt.has_las_or_csv = false;
  for (int i = 0; i < (int)files.size(); i++)
    opt.has_las_or_csv = opt.has_las_or_csv || asp::is_las_or_csv(files[i]);
  if (opt.has_las_or_csv && opt.do_ortho)
    vw_throw( ArgumentErr() << "Cannot create orthoimages if "
              << "point clouds are LAS or CSV.\n" );

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

void las_or_csv_to_tifs(Options& opt, vw::cartography::GeoReference const& georef,
                        std::vector<std::string> & tmp_tifs){

  // Convert any LAS or CSV files to ASP tif files. We do some binning
  // to make the spatial data more localized, to improve performance.
  // We will later wipe these temporary tifs.

  if (!opt.has_las_or_csv) return;

  Stopwatch sw;
  sw.start();

  // Error checking for CSV
  int num_files = opt.pointcloud_files.size();
  for (int i = 0; i < num_files; i++){
    if (!asp::is_csv(opt.pointcloud_files[i])) continue;
    if (opt.csv_format_str == "")
      vw_throw(ArgumentErr() << "CSV files were passed in, but the "
               << "CSV format string was not set.\n");
  }

  asp::CsvConv csv_conv;
  asp::parse_csv_format(opt.csv_format_str, opt.csv_proj4_str, csv_conv);

  // Set the georef for CSV files
  GeoReference csv_georef = georef;

  // Set user's csv_proj4_str if specified
  asp::handle_easting_northing(csv_conv, csv_georef);

  // There are situations in which some files will already be tif, and
  // others will be LAS or CSV. When we convert the latter to tif,
  // we'd like to be able to match the number of rows of the existing
  // tif files, so later when we concatenate all these files from left
  // to right for the purpose of creating the DEM, we waste little
  // space.
  int num_rows = 0;
  for (int i = 0; i < num_files; i++){
    if (asp::is_las_or_csv(opt.pointcloud_files[i])) continue;
    DiskImageView<float> img(opt.pointcloud_files[i]);
    num_rows = std::max(num_rows, img.rows());
  }

  // No tif files exist. Find a reasonable value for the number of rows.
  if (num_rows == 0){
    boost::uint64_t max_num_pts = 0;
    for (int i = 0; i < num_files; i++){
      std::string file = opt.pointcloud_files[i];
      if (asp::is_las(file))
        max_num_pts = std::max(max_num_pts, asp::las_file_size(file));
      if (asp::is_csv(file))
        max_num_pts = std::max(max_num_pts, asp::csv_file_size(file));
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

    if (!asp::is_las_or_csv(opt.pointcloud_files[i])) continue;
    std::string in_file = opt.pointcloud_files[i];
    std::string stem = fs::path( in_file ).stem().string();
    std::string suffix;
    if (opt.out_prefix.find(stem) != std::string::npos)
      suffix = ".tif";
    else
      suffix = "-" + stem + ".tif";
    std::string out_file = opt.out_prefix + "-tmp" + suffix;

    // Handle the case when the output file may exist
    for (int count = 0; count < 1000; count++){
      if (!fs::exists(out_file)) break;
      // File exists, try a different name
      vw_out() << "File exists: " << out_file << std::endl;
      std::ostringstream os; os << count;
      out_file = opt.out_prefix + "-tmp-" + os.str() + suffix;
    }
    if (fs::exists(out_file))
      vw_throw( ArgumentErr()
                << "Too many attempts at creating a temporary file.\n");

    asp::las_or_csv_to_tif(in_file, out_file, num_rows, block_size,
                           &opt, csv_georef, csv_conv);
    opt.pointcloud_files[i] = out_file; // so we can use it instead of the las file
    tmp_tifs.push_back(out_file); // so we can wipe it later
  }

  sw.stop();
  vw_out(DebugMessage,"asp")
    << "LAS or CSV to TIF conversion time: " << sw.elapsed_seconds() << std::endl;

}

void handle_arguments( int argc, char *argv[], Options& opt ) {

  float dem_spacing1, dem_spacing2;

  po::options_description manipulation_options("Manipulation options");
  manipulation_options.add_options()
    ("x-offset", po::value(&opt.x_offset)->default_value(0), "Add a horizontal offset to the DEM.")
    ("y-offset", po::value(&opt.y_offset)->default_value(0), "Add a horizontal offset to the DEM.")
    ("z-offset", po::value(&opt.z_offset)->default_value(0), "Add a vertical offset to the DEM.")
    ("rotation-order", po::value(&opt.rot_order)->default_value("xyz"),"Set the order of an Euler angle rotation applied to the 3D points prior to DEM rasterization.")
    ("phi-rotation", po::value(&opt.phi_rot)->default_value(0),"Set a rotation angle phi.")
    ("omega-rotation", po::value(&opt.omega_rot)->default_value(0),"Set a rotation angle omega.")
    ("kappa-rotation", po::value(&opt.kappa_rot)->default_value(0),"Set a rotation angle kappa.");

  po::options_description projection_options("Projection options");
  projection_options.add_options()
    ("t_srs", po::value(&opt.target_srs_string)->default_value(""),
     "Specify the output projection (PROJ.4 string).")
    ("t_projwin", po::value(&opt.target_projwin), "Selects a subwindow from the source image for copying but with the corners given in georeferenced coordinates. Max is exclusive.")
    ("dem-spacing,s", po::value(&dem_spacing1)->default_value(0.0),
     "Set output DEM resolution (in target georeferenced units per pixel). If not specified, it will be computed automatically (except for LAS and CSV files). This is the same as the --tr option.")
    ("tr", po::value(&dem_spacing2)->default_value(0.0),
     "Set output DEM resolution (in target georeferenced units per pixel). If not specified, it will be computed automatically (except for LAS and CSV files). This is the same as the --dem-spacing option.")
    ("reference-spheroid,r", po::value(&opt.reference_spheroid),"Set the reference spheroid [Earth, Moon, Mars]. This will override manually set datum information.")
    ("semi-major-axis", po::value(&opt.semi_major)->default_value(0), "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis", po::value(&opt.semi_minor)->default_value(0), "Explicitly set the datum semi-minor axis in meters.")
    ("sinusoidal", "Save using a sinusoidal projection.")
    ("mercator", "Save using a Mercator projection.")
    ("transverse-mercator", "Save using a transverse Mercator projection.")
    ("orthographic", "Save using an orthographic projection.")
    ("stereographic", "Save using a stereographic projection.")
    ("lambert-azimuthal", "Save using a Lambert azimuthal projection.")
    ("utm", po::value(&opt.utm_zone), "Save using a UTM projection with the given zone.")
    ("proj-lat", po::value(&opt.proj_lat)->default_value(0),
     "The center of projection latitude (if applicable).")
    ("proj-lon", po::value(&opt.proj_lon)->default_value(0),
     "The center of projection longitude (if applicable).")
    ("proj-scale", po::value(&opt.proj_scale)->default_value(1),
     "The projection scale (if applicable).");

  po::options_description general_options("General Options");
  general_options.add_options()
    ("nodata-value", po::value(&opt.nodata_value),
     "Nodata value to use on output. This is the same as default-value.")
    ("use-alpha", po::bool_switch(&opt.has_alpha)->default_value(false),
     "Create images that have an alpha channel.")
    ("normalized,n", po::bool_switch(&opt.do_normalize)->default_value(false),
     "Also write a normalized version of the DEM (for debugging).")
    ("orthoimage", po::bool_switch(&opt.do_ortho)->default_value(false), "Write an orthoimage based on the texture files passed in as inputs (after the point clouds).")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
    ("output-filetype,t", po::value(&opt.output_file_type)->default_value("tif"), "Specify the output file.")
    ("errorimage", po::bool_switch(&opt.do_error)->default_value(false), "Write a triangulation intersection error image.")
    ("dem-hole-fill-len", po::value(&opt.dem_hole_fill_len)->default_value(0), "Maximum dimensions of a hole in the output DEM to fill in, in pixels.")
    ("orthoimage-hole-fill-len", po::value(&opt.ortho_hole_fill_len)->default_value(0), "Maximum dimensions of a hole in the output orthoimage to fill in, in pixels.")
    ("remove-outliers", po::bool_switch(&opt.remove_outliers_with_pct)->default_value(true),
     "Turn on outlier removal based on percentage of triangulation error. Obsolete, as this is the default.")
    ("remove-outliers-params", po::value(&opt.remove_outliers_params)->default_value(Vector2(75.0, 3.0), "pct factor"), "Outlier removal based on percentage. Points with triangulation error larger than pct-th percentile times factor will be removed as outliers. [default: pct=75.0, factor=3.0]")
    ("max-valid-triangulation-error", po::value(&opt.max_valid_triangulation_error)->default_value(0), "Outlier removal based on threshold. Points with triangulation error larger than this (in meters) will be removed from the cloud.")
    ("median-filter-params", po::value(&opt.median_filter_params)->default_value(Vector2(0, 0), "window_size threshold"), "If the point cloud height at the current point differs by more than the given threshold from the median of heights in the window of given size centered at the point, remove it as an outlier. Use for example 11 and 40.0.")
    ("erode-length", po::value<int>(&opt.erode_len)->default_value(0),
     "Erode input point clouds by this many pixels at boundary (after outliers are removed, but before filling in holes).")
    ("csv-format", po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4", po::value(&opt.csv_proj4_str)->default_value(""), "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("rounding-error", po::value(&opt.rounding_error)->default_value(asp::APPROX_ONE_MM),
     "How much to round the output DEM and errors, in meters (more rounding means less precision but potentially smaller size on disk). The inverse of a power of 2 is suggested. [Default: 1/2^10]")
    ("search-radius-factor", po::value(&opt.search_radius_factor)->default_value(0.0),
     "Multiply this factor by dem-spacing to get the search radius. The DEM height at a given grid point is obtained as a weighted average of heights of all points in the cloud within search radius of the grid point, with the weights given by a Gaussian. Default search radius: max(dem-spacing, default_dem_spacing), so the default factor is about 1.")
    ("use-surface-sampling", po::bool_switch(&opt.use_surface_sampling)->default_value(false),
     "Use the older algorithm, interpret the point cloud as a surface made up of triangles and interpolate into it (prone to aliasing).")
    ("fsaa", po::value(&opt.fsaa)->implicit_value(3), "Oversampling amount to perform antialiasing (obsolete).")
    ("no-dem", po::bool_switch(&opt.no_dem)->default_value(false), "Skip writing a DEM.");

  general_options.add( manipulation_options );
  general_options.add( projection_options );
  general_options.add( asp::BaseOptionsDescription(opt) );

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
    vw_throw( ArgumentErr() << "Missing input point clouds.\n"
              << usage << general_options );
  std::vector<std::string> input_files = vm["input-files"].as< std::vector<std::string> >();
  parse_input_clouds_textures(input_files, usage, general_options, opt);

  if (opt.median_filter_params[0] < 0 || opt.median_filter_params[1] < 0){
    vw_throw( ArgumentErr() << "The parameters for median-based filtering "
              << "must be non-negative.\n"
              << usage << general_options );
  }

  if (opt.has_las_or_csv && opt.median_filter_params[0] > 0 &&
      opt.median_filter_params[1] > 0){
    vw_throw( ArgumentErr() << "Median-based filtering cannot handle CSV or LAS files.\n"
              << usage << general_options );
  }

  if (opt.erode_len < 0){
    vw_throw( ArgumentErr() << "Erode length must be non-negative.\n"
              << usage << general_options );
  }

  // A fix to the unfortunate fact that the user can specify the DEM
  // spacing in two ways on the command line.
  if (dem_spacing1 < 0.0 || dem_spacing2 < 0.0 ){
    // Note: Zero spacing means we'll set it internally.
    vw_throw( ArgumentErr() << "The DEM spacing must be non-negative.\n"
              << usage << general_options );
  }
  if (dem_spacing1 != 0 && dem_spacing2 != 0){
    vw_throw( ArgumentErr() << "The DEM spacing was specified twice.\n"
              << usage << general_options );
  }
  opt.dem_spacing = std::max(dem_spacing1, dem_spacing2);

  if (opt.has_las_or_csv && opt.dem_spacing <= 0){
    vw_throw( ArgumentErr() << "When inputs are LAS or CSV files, the "
              << "output DEM resolution must be set.\n" );
  }

  if ( opt.out_prefix.empty() )
    opt.out_prefix =
      asp::prefix_from_pointcloud_filename( opt.pointcloud_files[0] );

  if (opt.use_surface_sampling){
    vw_out(WarningMessage)
      << "The --use-surface-sampling option invokes the old algorithm and "
      << "is obsolete, it will be removed in future versions.\n";
  }

  if (opt.use_surface_sampling && opt.has_las_or_csv)
    vw_throw( ArgumentErr() << "Cannot use surface "
              << "sampling with LAS or CSV files.\n" );

  if (opt.fsaa != 1 && !opt.use_surface_sampling){
    vw_throw( ArgumentErr() << "The --fsaa option is obsolete. It can be used only with the --use-surface-sampling option which invokes the old algorithm.\n" << usage << general_options );
  }

  if (opt.dem_hole_fill_len < 0)
    vw_throw( ArgumentErr() << "The value of "
              << "--dem-hole-fill-len must be non-negative.\n");
  if (opt.ortho_hole_fill_len < 0)
    vw_throw( ArgumentErr() << "The value of "
              << "--orthoimage-hole-fill-len must be non-negative.\n");
  if ( !opt.do_ortho && opt.ortho_hole_fill_len > 0) {
    vw_throw( ArgumentErr() << "The value of --orthoimage-hole-fill-len"
              << " is positive, but orthoimage generation was not requested.\n");
  }

  double pct = opt.remove_outliers_params[0], factor = opt.remove_outliers_params[1];
  if (pct <= 0 || pct > 100 || factor <= 0.0){
    vw_throw( ArgumentErr() << "Invalid values were provided for remove-outliers-params.\n");
  }

  if (opt.max_valid_triangulation_error > 0){
    // Since the user passed in a threshold, will use that to rm
    // outliers, instead of using the percentage.
    opt.remove_outliers_with_pct = false;
  }

  // Create the output directory
  asp::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  boost::to_lower( opt.reference_spheroid );
  if ( vm.count("sinusoidal") )               opt.projection = SINUSOIDAL;
  else if ( vm.count("mercator") )            opt.projection = MERCATOR;
  else if ( vm.count("transverse-mercator") ) opt.projection = TRANSVERSEMERCATOR;
  else if ( vm.count("orthographic") )        opt.projection = ORTHOGRAPHIC;
  else if ( vm.count("stereographic") )       opt.projection = STEREOGRAPHIC;
  else if ( vm.count("lambert-azimuthal") )   opt.projection = LAMBERTAZIMUTHAL;
  else if ( vm.count("utm") )                 opt.projection = UTM;
  else                                        opt.projection = PLATECARREE;
  opt.has_nodata_value = vm.count("nodata-value");
}

// If a pixel has invalid data, fill its value with the average of
// valid pixel values within a given window around the pixel.
template <class ImageT>
class FillNoDataWithAvg:
  public ImageViewBase< FillNoDataWithAvg<ImageT> > {
  ImageT m_img;
  int m_kernel_size;
  typedef typename ImageT::pixel_type PixelT;

public:

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef ProceduralPixelAccessor<FillNoDataWithAvg> pixel_accessor;

  FillNoDataWithAvg( ImageViewBase<ImageT> const& img,
                     int kernel_size) :
    m_img(img.impl()), m_kernel_size(kernel_size) {
    VW_ASSERT(m_kernel_size%2 == 1 && m_kernel_size > 0,
              ArgumentErr() << "Expecting odd and positive kernel size.");
  }

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {
    if (is_valid(m_img(i, j)))
      return m_img(i, j);

    pixel_type val; val.validate();
    int nvalid = 0;
    int c0 = i, r0 = j; // convert to int from size_t
    int k2 = m_kernel_size/2, nc = m_img.cols(), nr = m_img.rows();
    for (int c = std::max(0, c0-k2); c <= std::min(nc-1, c0+k2); c++){
      for (int r = std::max(0, r0-k2); r <= std::min(nr-1, r0+k2); r++){
        if (!is_valid(m_img(c, r))) continue;
        val += m_img(c, r);
        nvalid++;
      }
    }

    if (nvalid == 0) return m_img(i, j); // could not find valid points
    return val/nvalid; // average of valid values within window
  }

  typedef FillNoDataWithAvg<CropView<ImageView<PixelT> > > prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {

    // Crop into an expanded box as to have enough pixels to do
    // averaging with given window at every pixel in the current box.
    BBox2i biased_box = bbox;
    biased_box.expand(m_kernel_size/2);
    biased_box.crop(bounding_box(m_img));
    ImageView<PixelT> dest( crop( m_img, biased_box ) );

    return prerasterize_type( crop( dest, -biased_box.min().x(), -biased_box.min().y(), cols(), rows() ), m_kernel_size );

}
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }

};
template <class ImgT>
FillNoDataWithAvg<ImgT>
fill_nodata_with_avg( ImageViewBase<ImgT> const& img,
                      int kernel_size) {
  typedef FillNoDataWithAvg<ImgT> result_type;
  return result_type( img.impl(), kernel_size);
}

template <class ImageT>
ImageViewRef< PixelGray<float> >
generate_fsaa_raster( ImageViewBase<ImageT> const& rasterizer,
                      Options const& opt ) {
  // This probably needs a lanczos filter. Sinc filter is the ideal
  // since it is the ideal brick filter.
  // ... or ...
  // possibly apply the blur on a linear scale (pow(0,2.2), blur, then exp).

  float fsaa_sigma = 1.0f * float(opt.fsaa)/2.0f;
  int kernel_size = vw::compute_kernel_size(fsaa_sigma);

  ImageViewRef< PixelGray<float> > rasterizer_fsaa;
  if ( opt.target_projwin != BBox2() ) {
    typedef ValueEdgeExtension< PixelGray<float> > ValExtend;
    if ( opt.fsaa > 1 ) {
      // subsample .. samples from the corner.
      rasterizer_fsaa =
        crop(edge_extend
             (apply_mask
              (asp::resample_aa
               (translate
                (gaussian_filter
                 (fill_nodata_with_avg
                  (create_mask(rasterizer.impl(),opt.nodata_value),
                   kernel_size),
                  fsaa_sigma),
                 -double(opt.fsaa-1)/2., double(opt.fsaa-1)/2.,
                 ConstantEdgeExtension()), 1.0/opt.fsaa),
               opt.nodata_value),
              ValExtend(opt.nodata_value)), opt.target_projwin_pixels );
    } else {
      rasterizer_fsaa =
        crop(edge_extend(rasterizer.impl(),
                         ValExtend(opt.nodata_value)), opt.target_projwin_pixels );
    }
  } else {
    if ( opt.fsaa > 1 ) {
      // subsample .. samples from the corner.
      rasterizer_fsaa =
        apply_mask
        (asp::resample_aa
         (translate
          (gaussian_filter
           (fill_nodata_with_avg
            (create_mask(rasterizer.impl(),opt.nodata_value),
             kernel_size),
            fsaa_sigma),
           -double(opt.fsaa-1)/2., double(opt.fsaa-1)/2.,
           ConstantEdgeExtension()), 1.0/opt.fsaa),
         opt.nodata_value);
    } else {
      rasterizer_fsaa = rasterizer.impl();
    }
  }
  return rasterizer_fsaa;
}

namespace asp{

  // If the third component of a vector is NaN, assign to it the given
  // no-data value
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

      Vector3 err = subvector(pt, 3, 3);
      Vector3 geo = m_georef.datum().cartesian_to_geodetic(xyz);
      Matrix3x3 M = m_georef.datum().lonlat_to_ned_matrix(subvector(geo, 0, 2));
      Vector3 ned_err = M*err;
      return ned_err;
    }
  };
  template <class ImageT>
  UnaryPerPixelView<ImageT, ErrorToNED>
  inline error_to_NED( ImageViewBase<ImageT> const& image, GeoReference const& georef ) {
    return UnaryPerPixelView<ImageT, ErrorToNED>( image.impl(),
                                                  ErrorToNED(georef) );
  }

  template<class ImageT>
  void save_image(Options& opt, ImageT img, GeoReference const& georef,
                  int hole_fill_len, std::string const& imgName){


    // When hole-filling is used, we need to look hole_fill_len beyond
    // the current block.  If the block size is 256, and hole fill len
    // is big, like 512 or 1024, we end up processing a huge block
    // only to save a small center block.  For that reason, save
    // temporarily with big blocks, and then re-save with small
    // blocks.
    if (hole_fill_len > 512)
      vw_out(WarningMessage)
        << "Detected large hole-fill length. Memory usage and run-time may go up.\n";

    int block_size = nextpow2(2.0*hole_fill_len);
    block_size = std::max(256, block_size);

    std::string output_file = opt.out_prefix + "-" + imgName + "."
      + opt.output_file_type;
    vw_out() << "Writing: " << output_file << "\n";
    TerminalProgressCallback tpc("asp", imgName + ": ");
    if ( opt.output_file_type == "tif" )
      asp::save_with_temp_big_blocks(block_size, output_file, img, georef, opt.nodata_value,
                                     opt, tpc);
    else
      asp::write_gdal_georeferenced_image(output_file, img, georef, opt, tpc);
  }

  // A class for combining the three channels of errors and finding
  // their absolute values.
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

    inline int32 cols() const { return m_image1.cols(); }
    inline int32 rows() const { return m_image1.rows(); }
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

  // Round pixels in given image to multiple of given scale.
  // Don't round nodata values.
  template <class PixelT>
  struct RoundImagePixelsSkipNoData: public vw::ReturnFixedType<PixelT> {

    double m_scale, m_nodata;

    RoundImagePixelsSkipNoData(double scale, double nodata):m_scale(scale),
                                                            m_nodata(nodata){
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
      int len = m_vals.size();
      double pct = remove_outliers_params[0]/100.0; // e.g., 0.75
      double factor = remove_outliers_params[1];
      int k = std::min(len-1, (int)(pct*len));
      double val = m_vals[k]*factor*4.0;
      return val;
    }

  };

  // Read a texture file
  template<class PixelT>
  typename boost::enable_if<boost::is_same<PixelT, PixelGray<float> >, ImageViewRef<PixelT> >::type
  read_file(std::string const& file){
    return DiskImageView<PixelT>(file);
  }

  // Read a point cloud file
  template<class PixelT>
  typename boost::disable_if<boost::is_same<PixelT, PixelGray<float> >, ImageViewRef<PixelT> >::type
  read_file(std::string const& file){
    return asp::read_cloud< math::VectorSize<PixelT>::value >(file);
  }

  // Read given files and form an image composite.
  template<class PixelT>
  ImageViewRef<PixelT> form_composite(std::vector<std::string> const & files){

    VW_ASSERT(files.size() >= 1,
              ArgumentErr() << "Expecting at least one file.\n");

    // We will use this composite in OrthoRasterizerView, where we
    // will break it into sub-blocks. We don't want a sub-block to see
    // portions of more than one image in the composite, as that will
    // be bad for performance (it will think the subblock has a huge
    // range of values). Hence, we will separate the images in the
    // composite by an amount which will sufficiently isolate them.
    int spacing = asp::OrthoRasterizerView::max_subblock_size();

    vw::mosaic::ImageComposite<PixelT> C;
    C.set_draft_mode(true); // images will be disjoint, no need for fancy stuff

    for (int i = 0; i < (int)files.size(); i++){

      ImageViewRef<PixelT> I = read_file<PixelT>(files[i]);

      // We will stack the images in the composite side by side. Images which
      // are wider than tall will be transposed.
      // To do: A more efficient approach would be to also stack one
      // image on top of each other, if some images are not too tall.
      if (I.rows() < I.cols()) I = transpose(I);

      int start = C.cols();
      if (i > 0){
        // Insert the spacing
        start = spacing*(int)ceil(double(start)/spacing) + spacing;
      }
      C.insert(I, start, 0);

    }

    return C;

  }

  template<int num_ch>
  ImageViewRef<double> error_norm(std::vector<std::string> const& pc_files){

    // Read the error channels from the point clouds, and take their norm

    VW_ASSERT(pc_files.size() >= 1,
              ArgumentErr() << "Expecting at least one file.\n");

    const int beg_ech = 3; // errors start at this channel
    const int num_ech = num_ch - beg_ech; // number of error channels
    ImageViewRef< Vector<double, num_ch> > point_disk_image
      = asp::form_composite<Vector<double, num_ch> >(pc_files);
    ImageViewRef< Vector<double, num_ech> > error_channels =
      select_channels<num_ech, num_ch, double>(point_disk_image, beg_ech);

    return per_pixel_filter(error_channels,
                            asp::VectorNorm< Vector<double, num_ech> >());
  }

  int num_channels(std::vector<std::string> const& pc_files){

    // Find the number of channels in the point clouds.
    // If the point clouds have inconsistent number of channels,
    // return the minimum of 3 and the minimum number of channels.
    // This will be used to flag that we cannot reliable extract the
    // error channels, which start at channel 4.

    VW_ASSERT(pc_files.size() >= 1,
              ArgumentErr() << "Expecting at least one file.\n");

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

void do_software_rasterization( const ImageViewRef<Vector3>& proj_point_input,
                                Options& opt,
                                cartography::GeoReference& georef,
                                ImageViewRef<double> const& error_image,
                                double estim_max_error) {
  Stopwatch sw1;
  sw1.start();
  asp::OrthoRasterizerView
    rasterizer(proj_point_input.impl(), select_channel(proj_point_input.impl(),2),
               opt.dem_spacing, opt.search_radius_factor, opt.use_surface_sampling,
               Options::tri_tile_size(), // to efficiently process the cloud
               opt.remove_outliers_with_pct, opt.remove_outliers_params,
               error_image, estim_max_error, opt.max_valid_triangulation_error,
               opt.median_filter_params, opt.erode_len, opt.has_las_or_csv,
               TerminalProgressCallback("asp","QuadTree: ") );

  sw1.stop();
  vw_out(DebugMessage,"asp") << "Quad time: " << sw1.elapsed_seconds()
                             << std::endl;

  if (!opt.has_nodata_value) {
    opt.nodata_value = std::floor(rasterizer.bounding_box().min().z() - 1);
  }
  rasterizer.set_use_minz_as_default(false);
  rasterizer.set_default_value(opt.nodata_value);

  vw_out() << "\t--> DEM spacing: " << rasterizer.spacing() << " pt/px\n";
  vw_out() << "\t             or: " << 1.0/rasterizer.spacing() << " px/pt\n";

  if (opt.has_alpha)
    rasterizer.set_use_alpha(true);

  // Now we are ready to specify the affine transform.
  georef.set_transform(rasterizer.geo_transform());

  // If the user requested FSAA .. tell the rasterer to increase
  // its sampling rate
  if ( opt.fsaa > 1 )
    rasterizer.set_spacing( rasterizer.spacing() / double(opt.fsaa) );

  // If the user specified the ULLR .. update the georeference
  // transform here. The generate_fsaa_raster will be responsible
  // for making sure we have the correct pixel crop.
  if ( opt.target_projwin != BBox2() ) {
    if ( opt.target_projwin.min().y() > opt.target_projwin.max().y() )
      std::swap( opt.target_projwin.min().y(),
                 opt.target_projwin.max().y() );
    vw_out() << "Cropping to " << opt.target_projwin << " pt. " << std::endl;
    Matrix3x3 transform = georef.transform();
    opt.target_projwin.max().x() -= fabs(transform(0,0));
    opt.target_projwin.min().y() += fabs(transform(1,1));

    // This math seems a little silly, but we need to fuzz the
    // values on target projwin so that it aligns with a pixel value.
    opt.target_projwin_pixels =
      georef.point_to_pixel_bbox( opt.target_projwin );
    opt.target_projwin_pixels.crop(bounding_box(rasterizer.impl()));
    opt.target_projwin =
      georef.pixel_to_point_bbox( opt.target_projwin_pixels );
    transform(0,2) = opt.target_projwin.min().x();
    transform(1,2) = opt.target_projwin.max().y();
    georef.set_transform( transform );
  }

  // Fix have pixel offset required if pixel_interpretation is
  // PixelAsArea. We could have done that earlier ... but it makes
  // the above easier to not think about it.
  if ( georef.pixel_interpretation() ==
       cartography::GeoReference::PixelAsArea ) {
    Matrix3x3 transform = georef.transform();
    transform(0,2) -= 0.5 * transform(0,0);
    transform(1,2) -= 0.5 * transform(1,1);
    georef.set_transform( transform );
  }

  vw_out() << "\nOutput georeference: \n\t" << georef << std::endl;

  // Do not round the DEM heights for small bodies
  if (georef.datum().semi_major_axis() <= asp::MIN_RADIUS_FOR_ROUNDING ||
      georef.datum().semi_minor_axis() <= asp::MIN_RADIUS_FOR_ROUNDING){
    opt.rounding_error = 0.0;
  }

  // We will first generate the DEM with holes, and then fill them later,
  // rather than filling holes in the cloud first. This is faster.
  rasterizer.set_hole_fill_len(0);

  ImageViewRef< PixelGray<float> > rasterizer_fsaa =
    generate_fsaa_raster( rasterizer, opt );

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
        (asp::fill_holes_grass(create_mask
                               (block_cache(dem, tile_size, opt.num_threads),
                                opt.nodata_value),
                               hole_fill_len),
         opt.nodata_value);
    }

    vw_out()<< "Creating output file that is " << bounding_box(dem).size()
            << " px.\n";

    save_image(opt, dem, georef, hole_fill_len, "DEM");
    sw2.stop();
    vw_out(DebugMessage,"asp") << "DEM render time: "
                               << sw2.elapsed_seconds() << std::endl;
  }

  // Write triangulation error image if requested
  if ( opt.do_error ) {
    int num_channels = asp::num_channels(opt.pointcloud_files);

    int hole_fill_len = 0;
    if (num_channels == 4){
      // The error is a scalar.
      ImageViewRef<Vector4> point_disk_image
        = asp::form_composite<Vector4>(opt.pointcloud_files);
      ImageViewRef<double> error_channel = select_channel(point_disk_image,3);
      rasterizer.set_texture( error_channel );
      rasterizer.set_hole_fill_len(hole_fill_len);
      rasterizer_fsaa = generate_fsaa_raster( rasterizer, opt );
      save_image(opt,
                 asp::round_image_pixels_skip_nodata(rasterizer_fsaa,
                                                     opt.rounding_error,
                                                     opt.nodata_value),
                 georef, hole_fill_len, "IntersectionErr");
    }else if (num_channels == 6){
      // The error is a 3D vector. Convert it to NED coordinate system,
      // and rasterize it.
      ImageViewRef<Vector6> point_disk_image
        = asp::form_composite<Vector6>(opt.pointcloud_files);
      ImageViewRef<Vector3> ned_err = asp::error_to_NED(point_disk_image, georef);
      std::vector< ImageViewRef< PixelGray<float> > >  rasterized(3);
      for (int ch_index = 0; ch_index < 3; ch_index++){
        ImageViewRef<double> ch = select_channel(ned_err, ch_index);
        rasterizer.set_texture(ch);
        rasterizer.set_hole_fill_len(hole_fill_len);
        rasterizer_fsaa = generate_fsaa_raster( rasterizer, opt );
        rasterized[ch_index] =
          block_cache(rasterizer_fsaa, tile_size, opt.num_threads);
      }
      save_image(opt,
                 asp::round_image_pixels_skip_nodata
                 (asp::combine_channels
                  (opt.nodata_value,
                   rasterized[0], rasterized[1], rasterized[2]),
                  opt.rounding_error, opt.nodata_value),
                 georef, hole_fill_len, "IntersectionErr");
    }else{
      // Note: We don't throw here. We still would like to write the
      // DRG (below) even if we can't write the error image.
      vw_out() << "The point cloud files must have an equal number of channels which "
               << "must be 4 or 6 to be able to process the intersection error.\n";
    }
  }

  // Write DRG if the user requested and provided a texture file
  if (opt.do_ortho) {
    int hole_fill_len = opt.ortho_hole_fill_len;
    Stopwatch sw3;
    sw3.start();
    ImageViewRef< PixelGray<float> > texture
      = asp::form_composite< PixelGray<float> >(opt.texture_files);
    rasterizer.set_texture(texture);
    rasterizer.set_hole_fill_len(hole_fill_len);
    rasterizer_fsaa = generate_fsaa_raster( rasterizer, opt );
    save_image(opt, rasterizer_fsaa, georef, hole_fill_len, "DRG");
    sw3.stop();
    vw_out(DebugMessage,"asp") << "DRG render time: "
                               << sw3.elapsed_seconds() << std::endl;
  }

  // Write out a normalized version of the DEM, if requested (for debugging)
  if (opt.do_normalize) {
    int hole_fill_len = 0;
    DiskImageView< PixelGray<float> >
      dem_image(opt.out_prefix + "-DEM." + opt.output_file_type);
    save_image(opt,
               apply_mask
               (channel_cast<uint8>
                (normalize(create_mask(dem_image,opt.nodata_value),
                           rasterizer.bounding_box().min().z(),
                           rasterizer.bounding_box().max().z(),
                           0, 255))),
               georef, hole_fill_len, "DEM-normalized");
  }
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Extract georef info from las files before converting them to ASP clouds.
    GeoReference las_georef;
    bool has_las_georef = asp::georef_from_las(opt.pointcloud_files, las_georef);

    // See if the user specified the datum outside of the srs string
    cartography::Datum user_datum;
    bool have_user_datum = asp::read_user_datum(opt.semi_major, opt.semi_minor,
                                                opt.reference_spheroid,
                                                user_datum);

    // Set up the georeferencing information.  We specify everything
    // here except for the affine transform, which is defined later once
    // we know the bounds of the orthorasterizer view.  However, we can
    // still reproject the points in the point image without the affine
    // transform because this projection never requires us to convert to
    // or from pixel space.
    GeoReference georef;

    // If the user specified a PROJ.4 string to use to interpret the
    // input in CSV files, use the same string to create output DEMs,
    // unless the user explicitly sets the output PROJ.4 string.
    if ( !opt.csv_proj4_str.empty() && opt.target_srs_string.empty()) {
      vw_out() << "The PROJ.4 string for reading CSV files was set. "
               << "Will use it for output as well.\n";
      opt.target_srs_string = opt.csv_proj4_str;
    }

    // If the data was left in cartesian coordinates, we need to give
    // the DEM a projection that uses some physical units (meters),
    // rather than lon, lat. Otherwise, we honor the user's requested
    // projection and convert the points if necessary.
    if (opt.target_srs_string.empty()) {

      if (have_user_datum)
        georef.set_datum( user_datum );

      switch( opt.projection ) {
      case SINUSOIDAL:
        georef.set_sinusoidal(opt.proj_lon); break;
      case MERCATOR:
        georef.set_mercator(opt.proj_lat,opt.proj_lon,opt.proj_scale); break;
      case TRANSVERSEMERCATOR:
        georef.set_transverse_mercator(opt.proj_lat,opt.proj_lon,opt.proj_scale); break;
      case ORTHOGRAPHIC:
        georef.set_orthographic(opt.proj_lat,opt.proj_lon); break;
      case STEREOGRAPHIC:
        georef.set_stereographic(opt.proj_lat,opt.proj_lon,opt.proj_scale); break;
      case LAMBERTAZIMUTHAL:
        georef.set_lambert_azimuthal(opt.proj_lat,opt.proj_lon); break;
      case UTM:
        georef.set_UTM( opt.utm_zone ); break;
      default: // Handles plate carree
        if (has_las_georef) georef = las_georef; // copy from las georef if present
        break;
      }

    } else {

      // Set the srs string into georef.
      asp::set_srs_string(opt.target_srs_string,
                          have_user_datum, user_datum, georef);
    }

    // Convert any input LAS or CSV files to ASP's point cloud tif format
    std::vector<std::string> tmp_tifs;
    las_or_csv_to_tifs(opt, georef, tmp_tifs);

    ImageViewRef<Vector3> point_image
      = asp::form_composite<Vector3>(opt.pointcloud_files);

    // Apply an (optional) rotation to the 3D points before building the mesh.
    if (opt.phi_rot != 0 || opt.omega_rot != 0 || opt.kappa_rot != 0) {
      vw_out() << "\t--> Applying rotation sequence: " << opt.rot_order
               << "      Angles: " << opt.phi_rot << "   "
               << opt.omega_rot << "  " << opt.kappa_rot << "\n";
      point_image =
        asp::point_transform(point_image, math::euler_to_rotation_matrix
                        (opt.phi_rot, opt.omega_rot,opt.kappa_rot, opt.rot_order));
    }

    // Determine if we should be using a longitude range between
    // [-180, 180] or [0,360]. We determine this by looking at the
    // average location of the points. If the average location has a
    // negative x value (think in ECEF coordinates) then we should
    // be using [0,360].
    Stopwatch sw1;
    sw1.start();
    int32 subsample_amt = int32(norm_2(Vector2(point_image.cols(),
                                               point_image.rows()))/32.0);
    if (subsample_amt < 1 ) subsample_amt = 1;
    PixelAccumulator<MeanAccumulator<Vector3> > mean_accum;
    for_each_pixel( subsample(point_image, subsample_amt),
                    mean_accum,
                    TerminalProgressCallback("asp","Statistics: ") );
    Vector3 avg_location = mean_accum.value();
    double avg_lon = avg_location.x() >= 0 ? 0 : 180;
    sw1.stop();
    vw_out(DebugMessage,"asp") << "Statistics time: " << sw1.elapsed_seconds()
                               << std::endl;

    // Estimate the maximum value of the error channel in case we would
    // like to remove outliers.
    ImageViewRef<double> error_image;
    double estim_max_error = 0.0;
    if (opt.remove_outliers_with_pct || opt.max_valid_triangulation_error > 0.0){
      int num_channels = asp::num_channels(opt.pointcloud_files);
      if (num_channels == 4)
        error_image = asp::error_norm<4>(opt.pointcloud_files);
      else if (num_channels == 6)
        error_image = asp::error_norm<6>(opt.pointcloud_files);
      else{
        vw_out() << "The point cloud files must have an equal number of channels which "
                 << "must be 4 or 6 to be able to remove outliers.\n";
        opt.remove_outliers_with_pct = false;
        opt.max_valid_triangulation_error = 0.0;
      }

      if (opt.remove_outliers_with_pct && opt.max_valid_triangulation_error == 0.0){
        // Get a somewhat dense sampling of the error image to get an idea
        // of what the distribution of errors is. This will be refined
        // later using a histogram approach and using all points. Do
        // several attempts if the sampling is too coarse.
        bool success = false;
        for (int count = 7; count <= 18; count++){

          double sample = (1 << count);
          int32 subsample_amt = int32(norm_2(Vector2(point_image.cols(),
                                                     point_image.rows()))/sample);
          if (subsample_amt < 1 ) subsample_amt = 1;

          Stopwatch sw2;
          sw2.start();
          PixelAccumulator<asp::ErrorRangeEstimAccum> error_accum;
          for_each_pixel( subsample(error_image, subsample_amt),
                          error_accum,
                          TerminalProgressCallback
                          ("asp","Triangulation error range estimation: ") );
          if (error_accum.size() > 0){
            success = true;
            estim_max_error = error_accum.value(opt.remove_outliers_params);
          }
          sw2.stop();
          vw_out(DebugMessage,"asp") << "Triangulation error range estimation time: "
                                     << sw2.elapsed_seconds() << std::endl;
          if (success || subsample_amt == 1) break;
          vw_out() << "Failed to estimate the triangulation range. "
                   << "Trying again with finer sampling.\n";
        }
      }
    }

    // We trade off readability here to avoid ImageViewRef dereferences
    if (opt.x_offset != 0 || opt.y_offset != 0 || opt.z_offset != 0) {
      vw_out() << "\t--> Applying offset: " << opt.x_offset
               << " " << opt.y_offset << " " << opt.z_offset << "\n";
      do_software_rasterization
        (geodetic_to_point
         (asp::point_image_offset
          (asp::recenter_longitude(cartesian_to_geodetic(point_image,georef),
                              avg_lon),
           Vector3(opt.x_offset,
                   opt.y_offset,
                   opt.z_offset)),georef),
         opt, georef, error_image, estim_max_error);
    } else {
      do_software_rasterization
        (geodetic_to_point
         (asp::recenter_longitude
          (cartesian_to_geodetic(point_image,georef), avg_lon),georef),
         opt, georef, error_image, estim_max_error);
    }

    // Wipe the temporary files
    for (int i = 0; i < (int)tmp_tifs.size(); i++)
      if (fs::exists(tmp_tifs[i])) fs::remove(tmp_tifs[i]);

  } ASP_STANDARD_CATCHES;

  return 0;
}
