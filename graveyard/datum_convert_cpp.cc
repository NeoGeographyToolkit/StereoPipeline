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


/// \file datum_convert.cc
///

#include <vw/Cartography/GeoTransform.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/PointUtils.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::cartography;
using namespace std;

template <class ImageT>
class DatumConvertView : public ImageViewBase<DatumConvertView<ImageT>>
{
  ImageT              m_input_dem;
  GeoReference const& m_input_georef;
  GeoReference const& m_output_georef;
  GeoTransform const  m_tf;
  double              m_nodata_val;
  bool                m_use_gcc_convert;
  bool                m_debug_mode;

public:

  typedef double pixel_type;
  typedef double result_type;
  typedef ProceduralPixelAccessor<DatumConvertView> pixel_accessor;

  /// Image view which replaces each input elevation with the elevation
  ///  value of the same location in the output georeference system.
  /// - This view does not do any horizontal movement.
  DatumConvertView(ImageT       const& input_dem,
                   GeoReference const& input_georef,
                   GeoReference const& output_georef,
                   double nodata_val,
                   bool debug_mode=false):
    m_input_dem(input_dem), m_input_georef(input_georef), m_output_georef(output_georef), 
    m_tf(input_georef, output_georef), m_nodata_val(nodata_val), m_use_gcc_convert(false), m_debug_mode(debug_mode) {
    
    // For simple datums with just the ellipsoid size specifed don't use Proj4 to do the
    //  datum conversions, use our ellipsoid calculations instead.  Proj4 does not seem to
    //  handle these properly at the moment.
    // - This handles the MOLA and D_MARS datums.
    std::string proj4_in (input_georef.overall_proj4_str ());
    std::string proj4_out(output_georef.overall_proj4_str());
    if ((!input_georef.is_projected()) && (!output_georef.is_projected()) &&   
         (proj4_in.find ("+ellps") == std::string::npos) &&
         (proj4_in.find ("+datum") == std::string::npos) &&
         (proj4_out.find("+ellps") == std::string::npos) &&
         (proj4_out.find("+datum") == std::string::npos)  ){
      m_use_gcc_convert = true;
    }
  }

  inline int32 cols  () const { return m_input_dem.cols(); }
  inline int32 rows  () const { return m_input_dem.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()(size_t col, size_t row, size_t p=0) const {

    // Handle nodata
    if (m_input_dem(col, row) == m_nodata_val)
      return m_nodata_val;

    // Compute the elevation in the output datum
    double  current_height  = m_input_dem(col, row);
    Vector2 input_pixel(col, row);
    Vector2 input_lonlat    = m_input_georef.pixel_to_lonlat(input_pixel);
    Vector3 input_llh(input_lonlat[0], input_lonlat[1], current_height);

    if (m_use_gcc_convert) {
      // Rely on our geodetic to GCC transforms.
      Vector3 gcc                 = m_input_georef.datum().geodetic_to_cartesian(input_llh);
      Vector3 output_llh_gccCheck = m_output_georef.datum().cartesian_to_geodetic(gcc);
      //Vector2 output_gcc_pixel    = m_output_georef.lonlat_to_pixel(Vector2(output_llh_gccCheck[0], output_llh_gccCheck[1]));
      return output_llh_gccCheck[2];
    }
    
    // Otherwise use Proj4 through the GeoTransform class.
    Vector3 output_llh = m_tf.lonlatalt_to_lonlatalt(input_llh);
    
    if (m_debug_mode) {
      std::cout.precision(12);
      std::cout << "input pixel        = " << input_pixel << std::endl;
      std::cout << "Dem lon/lat/alt    = " << input_llh     << std::endl;
      std::cout << "Output lon/lat/alt = " << output_llh << std::endl;
      double dist = haversine_circle_distance(input_lonlat, Vector2(output_llh[0], output_llh[1]));
      std::cout << "Haversine circle distance = " << dist << std::endl << std::endl;
   }
    
    return output_llh[2];
  }

  /// \cond INTERNAL
  typedef DatumConvertView<typename ImageT::prerasterize_type> prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {
    return prerasterize_type(m_input_dem.prerasterize(bbox), m_input_georef,
                              m_output_georef, m_nodata_val, m_debug_mode);
  }
  template <class DestT> inline void rasterize(DestT const& dest, BBox2i const& bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
  /// \endcond
};

// Helper function which uses the class above.
template <class ImageT>
DatumConvertView<ImageT>
datum_convert(ImageViewBase<ImageT> const& input_dem,
               GeoReference          const& input_georef,
               GeoReference          const& output_georef,
               double                       nodata_val,
               bool                         debug_mode) {
  return DatumConvertView<ImageT>(input_dem.impl(), input_georef, output_georef, nodata_val, debug_mode);
}


/// Convert input pixel location to output projection location
/// - Could allow the GeoTransform class to do this operation but it has been disabled
///   there to prevent misuse.
Vector2 get_output_loc(Vector2      const& input_pixel,
                       double       const  dem_height,
                       GeoReference const& input_georef,
                       GeoReference const& output_georef,
                       GeoTransform const& tf) {
  Vector2 input_lonlat     = input_georef.pixel_to_lonlat(input_pixel);
  Vector3 input_llh         (input_lonlat[0], input_lonlat[1], dem_height);
  Vector3 output_llh       = tf.lonlatalt_to_lonlatalt(input_llh);
  Vector2 output_lonlat     (output_llh[0], output_llh[1]);
  Vector2 output_projected = output_georef.lonlat_to_point(output_lonlat);

  return output_projected;
}


/// Find the bounding box in the output projected space that contains all the input data
template <typename T>
BBox2 get_output_projected_bbox(GeoReference     const& input_georef,
                                GeoReference     const& output_georef,
                                DiskImageView<T> const& input_dem,
                                double           const  nodata) {

  const int num_rows = input_dem.rows();
  const int num_cols = input_dem.cols();

  GeoTransform tf(input_georef, output_georef);

  // Expand along sides
  BBox2 output_bbox;
  for (int r = 0; r < num_rows; r++) {
    Vector2 pixel_left (0, r);
    Vector2 pixel_right(num_cols-1, r);
    double height_left  = input_dem(pixel_left[0],  pixel_left[1]);
    double height_right = input_dem(pixel_right[0], pixel_right[1]);

    // Don't allow nodata elevation values to be used in computations.
    // - Would be more accurate to use a mean elevation or something 
    //   instead of zero for the default value but this would increase the execution time.
    if (height_left  <= nodata) height_left  = 0;
    if (height_right <= nodata) height_right = 0;

    output_bbox.grow(get_output_loc(pixel_left,  height_left,  input_georef, output_georef, tf));
    output_bbox.grow(get_output_loc(pixel_right, height_right, input_georef, output_georef, tf));
  }

  // Expand along the top and bottom
  for (int c = 1; c < num_cols - 1; c++) {
    Vector2 pixel_top(c, 0);
    Vector2 pixel_bot(c, num_rows-1);
    double height_top = input_dem(pixel_top[0], pixel_top[1]);
    double height_bot = input_dem(pixel_bot[0], pixel_bot[1]);

    if (height_top <= nodata) height_top = 0;
    if (height_bot <= nodata) height_bot = 0;

    output_bbox.grow(get_output_loc(pixel_top, height_top, input_georef, output_georef, tf));
    output_bbox.grow(get_output_loc(pixel_bot, height_bot, input_georef, output_georef, tf));
  }

  return output_bbox;
}

struct Options: vw::GdalWriteOptions {
  string input_dem, output_dem, output_datum, input_datum,
         target_srs_string, input_grid, output_info_string;
  double nodata_value;
  bool   keep_bounds, use_double, debug_mode;
};

void handle_arguments(int argc, char *argv[], Options& opt){

  po::options_description general_options("");
  general_options.add_options()
    ("output-datum", po::value(&opt.output_datum), " The datum to convert to. Supported options: WGS_1984, D_MARS, MOLA.")
    ("input-datum", po::value(&opt.input_datum), " Override the value of the input datum. Supported options: WGS_1984, WGS72, NAD83, NAD27, D_MARS, MOLA.")
    ("t_srs",        po::value(&opt.target_srs_string)->default_value(""), "Specify the output datum via the PROJ.4 string.")
    ("output-info-string", po::value(&opt.output_info_string)->default_value(""), 
      "Write this additional text into a tag in the output file.")
    ("input-grid",   po::value(&opt.input_grid)->default_value(""), 
      "Specify a grid shift file that applies to the input image.")
    ("keep-bounds",  po::bool_switch(&opt.keep_bounds)->default_value(false)->implicit_value(true),
     "Don't recompute the image boundary.  This can help reduce changes caused by interpolation.")
    ("nodata_value", po::value(&opt.nodata_value)->default_value(-32768),
     "The value of no-data pixels, unless specified in the DEM.")
    ("debug-mode",  po::bool_switch(&opt.debug_mode)->default_value(false)->implicit_value(true),
     "Print conversion info for every pixel (to help verify output).")
    ("double", po::bool_switch(&opt.use_double)->default_value(false)->implicit_value(true),
     "Output using double precision (64 bit) instead of float (32 bit).");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("input-dem",    po::value(&opt.input_dem ), "The path to the input DEM file.")
    ("output-dem",   po::value(&opt.output_dem), "The path to the output DEM file.");
  po::positional_options_description positional_desc;
  positional_desc.add("input-dem",    1);
  positional_desc.add("output-dem",   1);

  string usage("[options] <input dem> <output dem>");
  bool allow_unregistered = false;
  vector<string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  
  if (opt.input_dem.empty())
    vw_throw(ArgumentErr() << "Missing <input dem>.\n\n" << usage << general_options);
  if (opt.output_dem.empty())
    vw_throw(ArgumentErr() << "Missing <output dem>.\n\n" << usage << general_options);
  if (opt.output_datum.empty() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr() << "Requires <output datum> or PROJ.4 string in order to proceed.\n\n" << usage << general_options);

  if (!opt.output_datum.empty() && !opt.target_srs_string.empty())
    vw_out(WarningMessage) << "Both the output datum and the PROJ.4 string were specified. The former takes precedence.\n";

  if (opt.debug_mode) {  // Debug output is unreadable with multiple threads.
    vw_out() << "Debug mode set, forcing thread count to 1.\n";
    opt.num_threads = 1;
  }

  boost::to_lower(opt.output_datum);

  vw::create_out_dir(opt.output_dem);

} // End function handle_arguments


/// Removes an field from a proj4 string if it is present.
/// - Returns true if the field was present.
bool strip_proj_entry(std::string &proj, std::string const& field) {
  size_t pos = proj.find(field);
  if (pos == std::string::npos)
    return false; // Field not found, nothing to do.

  size_t end = proj.find(' ', pos); // Find the next space.
  if (end == std::string::npos)
    end = proj.size(); // Handle end of the string.

  // Erase the field.
  size_t len = end - pos;
  proj.erase(pos, len);
  return true;
}


/// Set up the output georef which will be written to disk, and also a 
///  "working" output georef which will be used in the datum conversion computations.
/// - This is done because Proj.4 handles datum conversions in odd ways which sometimes
///   require that the output proj string be WGS84 even if that is not accurate.
void get_output_georef(Options      const& opt,
                       GeoReference const& input_georef,
                       GeoReference      & output_georef,
                       GeoReference      & output_working_georef) {

  // Create an output GeoReference object
  cartography::Datum user_datum;
  bool have_user_datum = asp::read_user_datum(0, 0, opt.output_datum, user_datum);
  if (opt.target_srs_string.empty()) {
    if (have_user_datum)
      output_georef.set_well_known_geogcs(opt.output_datum);
  } else {
    // The user specified the target srs_string
    // Set the srs string into georef.
    user_datum.set_datum_from_proj_str(opt.target_srs_string);
    output_georef.set_datum(user_datum);
  }
  
  output_working_georef = output_georef;
  
  // Use a different working proj4 if there is a grid shift being applied
  //  and the output datum is not WGS84.
  std::string input_proj = input_georef.overall_proj4_str();
  if (input_proj.find("+nadgrids") != std::string::npos){
    // Remove the existing datum name and replace with WGS84.
    std::string s = output_georef.overall_proj4_str();
    strip_proj_entry(s, "+datum");
    strip_proj_entry(s, "+ellps");
    s += " +datum=WGS84";
    Datum d;
    d.set_datum_from_proj_str(s);
    output_working_georef.set_datum(d);
  }
}

template <typename T>
void do_work(Options const& opt) {

  // Read the DEM to adjust
  DiskImageResourceGDAL dem_rsrc(opt.input_dem);
  double dem_nodata_val = opt.nodata_value;
  if (dem_rsrc.has_nodata_read()) {
    dem_nodata_val = dem_rsrc.nodata_read();
    vw_out() << "Found input nodata value for " << opt.input_dem << ": " << dem_nodata_val << endl;
  }
  DiskImageView<T> dem_img(dem_rsrc);
  GeoReference dem_georef;
  bool has_georef = read_georeference(dem_georef, dem_rsrc);
  if (!has_georef)
    vw_throw(ArgumentErr() << "Missing georeference for DEM: " << opt.input_dem << "\n");

  if (opt.input_datum != "")
    dem_georef.set_well_known_geogcs(opt.input_datum);

  if (opt.input_grid != "") {
    // Append the input grid option to the proj4 string we read in from disk.
    Datum d = dem_georef.datum();
    std::string s = dem_georef.overall_proj4_str();
    s += " " + opt.input_grid;
    d.set_datum_from_proj_str(s);
    dem_georef.set_datum(d);
  }

  GeoReference output_georef = dem_georef; // Default to the input image info
  BBox2i output_pixel_box(0, 0, dem_img.cols(), dem_img.rows());

  // Create an output GeoReference object
  
  GeoReference output_working_georef;
  get_output_georef(opt, dem_georef, output_georef, output_working_georef);

  if (!opt.keep_bounds) {
    BBox2 output_proj_box = get_output_projected_bbox(dem_georef, output_working_georef,
                                                      dem_img, dem_nodata_val);
    if (output_proj_box.area() <= 0) {
      vw_out() << "Error: No valid pixels found when computing output projected BBox!\n";
      return;
    }
    vw_out() << "Computed output projected box:\n" << output_proj_box << std::endl;

    // Overwrite the projected bounds in the output bounding box
    Matrix<double,3,3> affine = output_georef.transform();
    affine(0, 2) = output_proj_box.min()[0];
    if (affine(1, 1) > 0)
      affine(1, 2) = output_proj_box.min()[1];
    else // Decreasing Y as row increases
      affine(1, 2) = output_proj_box.max()[1];
    output_georef.set_transform(affine);
    output_working_georef.set_transform(affine);

    output_pixel_box = output_georef.point_to_pixel_bbox(output_proj_box);
    vw_out() << "Computed output pixel box:\n" << output_pixel_box << std::endl;

    // Check against an arbitrary size to 
    const int MAX_OUTPUT_SIZE = 999999;
    if ((output_pixel_box.width () > MAX_OUTPUT_SIZE) ||
        (output_pixel_box.height() > MAX_OUTPUT_SIZE)) {
      vw_out() << "Error: computed output size is too large!\n";
      return;
    }
    
  } // End boundary computation
  else
    vw_out() << "Output pixel box:\n" << output_pixel_box << std::endl;

  vw_out() << "Input georef:\n" << dem_georef << std::endl;
  vw_out() << "Output georef:\n" << output_georef << std::endl;

  // Update the elevation values in the image to account for the new datum.
  ImageViewRef<double> dem_new_heights = datum_convert(pixel_cast<double>(dem_img),
                                                       dem_georef,
                                                       output_working_georef,
                                                       dem_nodata_val, opt.debug_mode);

  // Apply the horizontal warping to the image on account of the new datum.
  ImageViewRef<double> output_dem = apply_mask(geo_transform(create_mask(dem_new_heights,
                                                                         dem_nodata_val),
                                                             dem_georef, output_working_georef,
                                                             output_pixel_box.width(),
                                                             output_pixel_box.height(),
                                                             ConstantEdgeExtension()
                                                           ),
                                               dem_nodata_val);

  vw_out() << "Writing adjusted DEM: " << opt.output_dem << endl;
  
  // Add extra info string if provided
  std::map<std::string, std::string> keywords;
  if (opt.output_info_string != "") {
    keywords["DATUM_INFO"] = opt.output_info_string;
  }

  if (opt.use_double) {
    // Output as double
    block_write_gdal_image(opt.output_dem, output_dem,
                           true, output_georef,
                           true, dem_nodata_val, opt,
                           TerminalProgressCallback("asp", "\t--> Converting datum: "),
                           keywords);
  } else {
    // Output as float
    block_write_gdal_image(opt.output_dem,  pixel_cast<float>(output_dem),
                           true, output_georef,
                           true, dem_nodata_val, opt,
                           TerminalProgressCallback("asp", "\t--> Converting datum: "),
                           keywords);
  }

}


int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);


    // Check the input data type
    DiskImageResourceGDAL dem_rsrc(opt.input_dem);
    ChannelTypeEnum input_data_type = dem_rsrc.channel_type();

    // Redirect to another function with the correct template type
    switch(input_data_type) {
      //case VW_CHANNEL_INT8   : do_work<vw::int8> (opt);  break;
      //case VW_CHANNEL_UINT8  : do_work<vw::uint8>(opt);  break;
      case VW_CHANNEL_INT16  : do_work<vw::int16>  (opt);  break;
      case VW_CHANNEL_UINT16 : do_work<vw::uint16> (opt);  break;
      case VW_CHANNEL_INT32  : do_work<vw::int32>  (opt);  break;
      case VW_CHANNEL_UINT32 : do_work<vw::uint32> (opt);  break;
      case VW_CHANNEL_FLOAT32: do_work<vw::float32>(opt);  break;
      case VW_CHANNEL_FLOAT64: do_work<vw::float64>(opt);  break;
      default: vw_throw(ArgumentErr() << "Input image format " << input_data_type << " is not supported!\n");
    };


  } ASP_STANDARD_CATCHES;

  return 0;
}
