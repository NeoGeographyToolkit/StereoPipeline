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


#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>

#include <boost/filesystem.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::cartography;
using namespace std;

template <class ImageT>
class DatumConvertView : public ImageViewBase<DatumConvertView<ImageT> >
{
  ImageT        m_input_dem;
  GeoReference const& m_input_georef;
  GeoReference const& m_output_georef;
  double              m_nodata_val;

public:

  typedef double pixel_type;
  typedef double result_type;
  typedef ProceduralPixelAccessor<DatumConvertView> pixel_accessor;

  /// Image view which adds or subtracts the ellipsoid/geoid difference
  ///  from elevations in a DEM image.
  DatumConvertView(ImageT       const& input_dem,
                   GeoReference const& input_georef,
                   GeoReference const& output_georef,
                   double nodata_val):
    m_input_dem(input_dem), m_input_georef(input_georef), m_output_georef(output_georef),
    m_nodata_val(nodata_val){}

  inline int32 cols  () const { return m_input_dem.cols(); }
  inline int32 rows  () const { return m_input_dem.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t col, size_t row, size_t p=0 ) const {

    // Handle nodata
    if ( m_input_dem(col, row) == m_nodata_val )
      return m_nodata_val;

    // Compute the elevation in the output datum
    double  current_height  = m_input_dem(col, row);
    Vector2 input_lonlat    = m_input_georef.pixel_to_lonlat(Vector2(col, row));
    Vector3 input_llh(input_lonlat[0], input_lonlat[1], current_height);
    Vector3 gcc_coord       = m_input_georef.datum().geodetic_to_cartesian(input_llh);
    Vector3 output_lonlat   = m_output_georef.datum().cartesian_to_geodetic(gcc_coord);
    double output_height = output_lonlat[2];
/*
    if ( (current_height < -50) || (current_height > 700))
    {
      std::cout << "pixel      = " << Vector2(col, row) << std::endl;
      std::cout << "height      = " << current_height << std::endl;
      //std::cout << m_input_dem(col, row)  << std::endl;
      //std::cout << "dem lonlat = " << input_llh     << std::endl;
      //std::cout << "gcc_coord  = " << gcc_coord     << std::endl;
      //std::cout << "out lonlat = " << output_lonlat << std::endl;
    }
*/
    return output_height;
  }

  /// \cond INTERNAL
  typedef DatumConvertView<typename ImageT::prerasterize_type> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return prerasterize_type( m_input_dem.prerasterize(bbox), m_input_georef,
                              m_output_georef, m_nodata_val );
  }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }
  /// \endcond
};

// Helper function which uses the class above.
template <class ImageT>
DatumConvertView<ImageT>
datum_convert( ImageViewBase<ImageT> const& input_dem,
               GeoReference          const& input_georef,
               GeoReference          const& output_georef,
               double                       nodata_val) {
  return DatumConvertView<ImageT>( input_dem.impl(), input_georef, output_georef, nodata_val );
}

/// Convert input pixel location to output projection location
Vector2 get_output_loc(Vector2      const& input_pixel,
                       double       const  dem_height,
                       GeoReference const& input_georef,
                       GeoReference const& output_georef) {
  Vector2 input_lonlat     = input_georef.pixel_to_lonlat(input_pixel);
  Vector3 input_llh(input_lonlat[0], input_lonlat[1], dem_height);
  Vector3 gcc_coord        = input_georef.datum().geodetic_to_cartesian(input_llh);
  Vector3 output_lonlat    = output_georef.datum().cartesian_to_geodetic(gcc_coord);
  Vector2 output_projected = output_georef.lonlat_to_point(Vector2(output_lonlat[0], output_lonlat[1]));
  return output_projected;
}

/// Find the bounding box in the output projected space that contains all the input data
template <typename T>
BBox2 get_output_projected_bbox(GeoReference     const& input_georef,
                                GeoReference     const& output_georef,
                                DiskImageView<T> const& input_dem) {

  const int num_rows = input_dem.rows();
  const int num_cols = input_dem.cols();

  vw_out() << "Image size = " << Vector2(num_cols, num_rows) << std::endl;

  // Expand along sides
  BBox2 output_bbox;
  for (int r=0; r<num_rows; ++r) {
    Vector2 pixel_left (0,          r);
    Vector2 pixel_right(num_cols-1, r);
    double height_left  = input_dem(pixel_left[0],  pixel_left[1]);
    double height_right = input_dem(pixel_right[0], pixel_right[1]);

    output_bbox.grow(get_output_loc(pixel_left,  height_left,  input_georef, output_georef));
    output_bbox.grow(get_output_loc(pixel_right, height_right, input_georef, output_georef));
  }

  // Expand along the top and bottom
  for (int c=1; c<num_cols-1; ++c) {
    Vector2 pixel_top(c, 0         );
    Vector2 pixel_bot(c, num_rows-1);
    double height_top = input_dem(pixel_top[0], pixel_top[1]);
    double height_bot = input_dem(pixel_bot[0], pixel_bot[1]);

    output_bbox.grow(get_output_loc(pixel_top, height_top, input_georef, output_georef));
    output_bbox.grow(get_output_loc(pixel_bot, height_bot, input_georef, output_georef));
  }
  return output_bbox;
}


struct Options : vw::cartography::GdalWriteOptions {
  string input_dem, output_dem, output_datum, target_srs_string;
  double nodata_value;
  bool   use_double;
};

void handle_arguments( int argc, char *argv[], Options& opt ){

  po::options_description general_options("");
  general_options.add_options()
    ("output-datum", po::value(&opt.output_datum), " The datum to convert to. Supported options: WGS_1984, NAD83, WGS72, and NAD27.")
    ("t_srs",        po::value(&opt.target_srs_string)->default_value(""), "Specify the output datum via the PROJ.4 string.")
    ("nodata_value", po::value(&opt.nodata_value)->default_value(-32768),
     "The value of no-data pixels, unless specified in the DEM.")
    ("double", po::bool_switch(&opt.use_double)->default_value(false)->implicit_value(true),
     "Output using double precision (64 bit) instead of float (32 bit).");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-dem",    po::value(&opt.input_dem  ), "The path to the input DEM file.")
    ("output-dem",   po::value(&opt.output_dem ), "The path to the output DEM file.");

  po::positional_options_description positional_desc;
  positional_desc.add("input-dem",    1);
  positional_desc.add("output-dem",   1);

  string usage("[options] <input dem> <output dem>");
  bool allow_unregistered = false;
  vector<string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if ( opt.input_dem.empty() )
    vw_throw( ArgumentErr() << "Missing input arguments.\n\n"     << usage << general_options );
  if ( opt.output_dem.empty() )
    vw_throw( ArgumentErr() << "Requires <output dem> in order to proceed.\n\n"  << usage << general_options );
  if ( opt.output_datum.empty() && opt.target_srs_string.empty())
    vw_throw( ArgumentErr() << "Requires <output datum> or PROJ.4 string in order to proceed.\n\n" << usage << general_options );

  if ( !opt.output_datum.empty() && !opt.target_srs_string.empty())
    vw_out(WarningMessage) << "Both the output datum and the PROJ.4 string were specified. The former takes precedence.\n";

  boost::to_lower(opt.output_datum);

  vw::create_out_dir(opt.output_dem);

  //// Turn on logging to file
  //asp::log_to_file(argc, argv, "", opt.out_prefix);

}

/// Perform vertical datum conversion.
/// - gdalwarp should be used after this tool to perform the horizontal conversion
/// - TODO: This tool does everything?

// --> Option 1: Run this tool before gdalwarp, all conversions handled.
// --> Option 2: This tool does everything, but only handles datum conversions
//               within the same projection type.

template <typename T>
void do_work(Options const& opt) {

  // Read the DEM to adjust
  DiskImageResourceGDAL dem_rsrc(opt.input_dem);
  double dem_nodata_val = opt.nodata_value;
  if ( dem_rsrc.has_nodata_read() ) {
    dem_nodata_val = dem_rsrc.nodata_read();
    vw_out() << "\tFound input nodata value for " << opt.input_dem << ": " << dem_nodata_val << endl;
  }
  DiskImageView<T> dem_img(dem_rsrc);
  GeoReference dem_georef;
  bool has_georef = read_georeference(dem_georef, dem_rsrc);
  if (!has_georef)
    vw_throw( ArgumentErr() << "Missing georeference for DEM: " << opt.input_dem << "\n" );

  // Handle the datums supported by our Datum class:
  // WGS84, WGS72, NAD83, NAD27, D_MOON, D_MARS

  // Create an output GeoReference object
  cartography::Datum user_datum;
  bool have_user_datum = asp::read_user_datum(0, 0, opt.output_datum, user_datum);
  GeoReference output_georef = dem_georef;
  if (opt.target_srs_string.empty()) {
    if (have_user_datum)
      output_georef.set_well_known_geogcs(opt.output_datum);
  } else {
    // The user specified the target srs_string
    // Set the srs string into georef.
    asp::set_srs_string(opt.target_srs_string, have_user_datum, user_datum, output_georef);
  }

  vw_out() << "Input georef:\n"  << dem_georef    << std::endl;

  BBox2 output_proj_box = get_output_projected_bbox(dem_georef, output_georef, dem_img);

  vw_out() << "Computed output projected box:\n" << output_proj_box << std::endl;

  // Overwrite the projected bounds in the output bounding box
  Matrix<double,3,3> affine = output_georef.transform();
  affine(0, 2) = output_proj_box.min()[0];
  if (affine(1, 1) > 0)
    affine(1, 2) = output_proj_box.min()[1];
  else // Decreasing Y as row increases
    affine(1, 2) = output_proj_box.max()[1];
  output_georef.set_transform(affine);

  vw_out() << "Output georef:\n" << output_georef << std::endl;

  BBox2i output_pixel_box = output_georef.point_to_pixel_bbox(output_proj_box);
  vw_out() << "Computed output pixel box:\n" << output_pixel_box << std::endl;

  // Update the elevation values in the image to account for the new datum.
  ImageViewRef<double> dem_new_heights = datum_convert(pixel_cast<double>(dem_img),
                                                       dem_georef,
                                                       output_georef,
                                                       dem_nodata_val
                                                      );

  // Apply the horizontal warping to the image on account of the new datum.
  ImageViewRef<double> output_dem = apply_mask(geo_transform(create_mask(dem_new_heights,
                                                                         dem_nodata_val),
                                                             dem_georef, output_georef,
                                                             output_pixel_box.width(),
                                                             output_pixel_box.height(),
                                                             ConstantEdgeExtension()
                                                            ),
                                               dem_nodata_val
                                              );

  vw_out() << "Writing adjusted DEM: " << opt.output_dem << endl;

  if ( opt.use_double ) {
    // Output as double
    block_write_gdal_image(opt.output_dem, output_dem,
                           true, output_georef,
                           true, dem_nodata_val, opt,
                           TerminalProgressCallback("asp", "\t--> Converting datum: ") );
  }else{
    // Output as float
    block_write_gdal_image(opt.output_dem,  pixel_cast<float>(output_dem),
                           true, output_georef,
                           true, dem_nodata_val, opt,
                           TerminalProgressCallback("asp", "\t--> Converting datum: ") );
  }

}


int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );


    // Check the input data type
    DiskImageResourceGDAL dem_rsrc(opt.input_dem);
    ChannelTypeEnum input_data_type = dem_rsrc.channel_type();

    // Redirect to another function with the correct template type
    switch(input_data_type) {
      //case VW_CHANNEL_INT8   : do_work<vw::int8   >(opt);  break;
      //case VW_CHANNEL_UINT8  : do_work<vw::uint8  >(opt);  break;
      case VW_CHANNEL_INT16  : do_work<vw::int16  >(opt);  break;
      case VW_CHANNEL_UINT16 : do_work<vw::uint16 >(opt);  break;
      case VW_CHANNEL_INT32  : do_work<vw::int32  >(opt);  break;
      case VW_CHANNEL_UINT32 : do_work<vw::uint32 >(opt);  break;
      case VW_CHANNEL_FLOAT32: do_work<vw::float32>(opt);  break;
      case VW_CHANNEL_FLOAT64: do_work<vw::float64>(opt);  break;
      default : vw_throw(ArgumentErr() << "Input image format " << input_data_type << " is not supported!\n");
    };


  } ASP_STANDARD_CATCHES;

  return 0;
}
