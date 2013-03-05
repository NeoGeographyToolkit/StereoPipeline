// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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

/// \file dem_geoid.cc
///

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using std::endl;
using std::string;
using namespace vw;
using namespace vw::cartography;

template <class ImageT>
class DemGeoidView : public ImageViewBase<DemGeoidView<ImageT> >
{
  ImageT m_img;
  GeoReference const& m_georef;
  ImageViewRef<PixelMask<double> > const& m_geoid;
  GeoReference const& m_geoid_georef;
  bool m_reverse_adjustment;
  double m_nodata_val;

public:

  typedef double pixel_type;
  typedef double result_type;
  typedef ProceduralPixelAccessor<DemGeoidView> pixel_accessor;

  DemGeoidView(ImageT const& img, GeoReference const& georef,
               ImageViewRef<PixelMask<double> > const& geoid,
               GeoReference const& geoid_georef, bool reverse_adjustment, double nodata_val):
    m_img(img), m_georef(georef),
    m_geoid(geoid), m_geoid_georef(geoid_georef),
    m_reverse_adjustment(reverse_adjustment),
    m_nodata_val(nodata_val){}

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t col, size_t row, size_t p=0 ) const {

    if ( m_img(col, row, p) == m_nodata_val ) return m_nodata_val;

    Vector2 lonlat = m_georef.pixel_to_lonlat(Vector2(col, row));

    // For testing (see the link to the reference web form belows).
    // lonlat[0] = -121; lonlat[1] = 37; // mainland US
    // lonlat[0] = -152; lonlat[1] = 66; // Alaska

    // Need to carefully wrap lonlat to the [0, 360) x [-90, 90) box.
    // Note that lon = 25, lat = 91 is the same as lon = 180 + 25, lat = 89
    // as we go through the North pole and show up on the other side.
    while ( std::abs(lonlat[1]) > 90.0 ){
      if ( lonlat[1] > 90.0 ){
        lonlat[1] = 180.0 - lonlat[1];
        lonlat[0] += 180.0;
      }
      if ( lonlat[1] < -90.0 ){
        lonlat[1] = -180.0 - lonlat[1];
        lonlat[0] += 180.0;
      }
    }
    while( lonlat[0] <   0.0  ) lonlat[0] += 360.0;
    while( lonlat[0] >= 360.0 ) lonlat[0] -= 360.0;

    result_type       geoid_height           = 0.0;
    Vector2           pix                    = m_geoid_georef.lonlat_to_pixel(lonlat);
    PixelMask<double> interp_val             = m_geoid(pix[0], pix[1]);
    if (!is_valid(interp_val)) return m_nodata_val;
    geoid_height                             = interp_val.child();
    result_type       height_above_ellipsoid = m_img(col, row, p);
    double            direction              = m_reverse_adjustment?-1:1;
    // See the note in the main program about the formula below
    result_type height_above_geoid           = height_above_ellipsoid - direction*geoid_height;

    return height_above_geoid;
  }

  /// \cond INTERNAL
  typedef DemGeoidView<typename ImageT::prerasterize_type> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return prerasterize_type( m_img.prerasterize(bbox), m_georef,
                              m_geoid, m_geoid_georef, m_reverse_adjustment, m_nodata_val );
  }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }
  /// \endcond
};

template <class ImageT>
DemGeoidView<ImageT>
dem_geoid( ImageViewBase<ImageT> const& img, GeoReference const& georef,
           ImageViewRef<PixelMask<double> > const& geoid,
           GeoReference const& geoid_georef, bool reverse_adjustment, double nodata_val) {
  return DemGeoidView<ImageT>( img.impl(), georef, geoid, geoid_georef,
                               reverse_adjustment, nodata_val );
}

struct Options : asp::BaseOptions {
  string geoid, geoid_file, dem_name, output_prefix;
  double nodata_value;
  bool use_double;
  bool reverse_adjustment;
};

void handle_arguments( int argc, char *argv[], Options& opt ){

  po::options_description general_options("");
  general_options.add_options()
    ("nodata_value", po::value(&opt.nodata_value)->default_value(-32767),
     "The value of no-data pixels, unless specified in the DEM.")
    ("geoid", po::value(&opt.geoid)->default_value("EGM96"), "The geoid to use [EGM96, NAVD88].")
    ("output-prefix,o", po::value(&opt.output_prefix), "Specify the output prefix.")
    ("double", po::bool_switch(&opt.use_double)->default_value(false)->implicit_value(true),
     "Output using double precision (64 bit) instead of float (32 bit).")
    ("reverse-adjustment",
     po::bool_switch(&opt.reverse_adjustment)->default_value(false)->implicit_value(true),
     "Go from DEM relative to the geoid to DEM relative to the ellipsoid.");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("dem", po::value(&opt.dem_name), "Explicitly specify the DEM.");

  po::positional_options_description positional_desc;
  positional_desc.add("dem", 1);

  std::string usage("[options] <dem>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  // The geoid containing the adjustments
#define STR_EXPAND(tok) #tok
#define STR_QUOTE(tok) STR_EXPAND(tok)
  std::string geoid_path = STR_QUOTE(GEOID_PATH);
  if (opt.geoid == "EGM96"){
    opt.geoid_file = geoid_path + "/" + "egm96-5.tif";
  }else if(opt.geoid == "NAVD88"){
    opt.geoid_file = geoid_path + "/" + "NAVD88.tif";
  }else{
    vw_throw( ArgumentErr() << "Unknown geoid: " << opt.geoid << ".\n\n"
              << usage << general_options );
  }

  if ( opt.dem_name.empty() )
    vw_throw( ArgumentErr() << "Requires <dem> in order to proceed.\n\n"
              << usage << general_options );

  if ( opt.output_prefix.empty() ) {
    opt.output_prefix = fs::path(opt.dem_name).stem().string();
  }
}

// Given a DEM, with each height value relative to the datum
// ellipsoid, convert the heights to be relative to the geoid.

// From: http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/intpthel.html
// Geoid heights can be used to convert between orthometric
// heights (approximately mean sea level) and ellipsoid heights
// according to the formula:
// h   =   H   +   N
// where,
// h = WGS 84 Ellipsoid height
// H = Orthometric height
// N = EGM96 Geoid height
// Therefore: H = h - N.

// We support two geoids: EGM96 and  NAVD88.
// Online tools for verification:
// EGM96:  http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/intpt.html
// NAVD88: http://www.ngs.noaa.gov/cgi-bin/GEOID_STUFF/geoid09_prompt1.prl
//
// Example, at lat = 37 and lon = -121 (Basalt Hills, CA), we have
// EGM96  geoid height = -32.69
// NAVD88 geoid height = -32.931

// To do: Do another test with the web form for Alaska for NAVD88!
// To do: Add Hawaii, etc.

int main( int argc, char *argv[] ) {

  std::cout << "see to do above!" << std::endl;

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    bool reverse_adjustment = opt.reverse_adjustment;

    // Read the DEM to adjust
    DiskImageResourceGDAL dem_rsrc(opt.dem_name);
    double dem_nodata_val = opt.nodata_value;
    if ( dem_rsrc.has_nodata_read() ) {
      dem_nodata_val = dem_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value for " << opt.dem_name << ": "
               << dem_nodata_val << endl;
    }
    DiskImageView<double> dem_img(dem_rsrc);
    GeoReference dem_georef;
    bool has_georef = read_georeference(dem_georef, dem_rsrc);
    if (!has_georef)
      vw_throw( ArgumentErr() << "Missing georeference for DEM: " << opt.dem_name << "\n" );

    // Read the geoid containing the adjustments. Read it in memory
    // entirely to dramatically speed up the computations.
    double geoid_nodata_val = std::numeric_limits<float>::quiet_NaN();
    DiskImageResourceGDAL geoid_rsrc(opt.geoid_file);
    if ( geoid_rsrc.has_nodata_read() ) {
      geoid_nodata_val = geoid_rsrc.nodata_read();
    }
    vw_out() << "\tAdjusting the DEM using the geoid: " << opt.geoid_file << endl;
    ImageView<float> geoid_img = DiskImageView<float>(geoid_rsrc);
    GeoReference geoid_georef;
    read_georeference(geoid_georef, geoid_rsrc);

    if ( dem_georef.datum().name() != geoid_georef.datum().name() ){
      vw_throw( ArgumentErr() << "Datum mismatch."
                << "\nDEM   datum: " << dem_georef.datum().name()
                << "\nGeoid datum: " << geoid_georef.datum().name()
                << "\n"
                );
    }

    ImageViewRef<PixelMask<double> > geoid
      = interpolate(create_mask( pixel_cast<double>(geoid_img), geoid_nodata_val ),
                    BicubicInterpolation(), ZeroEdgeExtension());

    ImageViewRef<double> adj_dem = dem_geoid(dem_img, dem_georef,
                                             geoid, geoid_georef,
                                             reverse_adjustment, dem_nodata_val);

    std::string adj_dem_file = opt.output_prefix + "-adj.tif";
    vw_out() << "Writing adjusted DEM: " << adj_dem_file << std::endl;

    if ( opt.use_double ) {
      // Output as double
      boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc(adj_dem_file,
                                                                          adj_dem, opt ) );
      rsrc->set_nodata_write( dem_nodata_val );
      write_georeference( *rsrc, dem_georef );
      block_write_image( *rsrc, adj_dem,
                         TerminalProgressCallback("asp", "\t--> Applying DEM adjustment: ") );
    }else{
      // Output as float
      ImageViewRef<float> adj_dem_float = channel_cast<float>( adj_dem );
      boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc(adj_dem_file,
                                                                          adj_dem_float, opt ) );
      rsrc->set_nodata_write( dem_nodata_val );
      write_georeference( *rsrc, dem_georef );
      block_write_image( *rsrc, adj_dem_float,
                         TerminalProgressCallback("asp", "\t--> Applying DEM adjustment: ") );
    }


  } ASP_STANDARD_CATCHES;

  return 0;
}
