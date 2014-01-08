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


/// \file dem_geoid.cc
///

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

#include <boost/filesystem.hpp>
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
  double m_correction;
  double m_nodata_val;

public:

  typedef double pixel_type;
  typedef double result_type;
  typedef ProceduralPixelAccessor<DemGeoidView> pixel_accessor;

  DemGeoidView(ImageT const& img, GeoReference const& georef,
               ImageViewRef<PixelMask<double> > const& geoid,
               GeoReference const& geoid_georef, bool reverse_adjustment, double correction, double nodata_val):
    m_img(img), m_georef(georef),
    m_geoid(geoid), m_geoid_georef(geoid_georef),
    m_reverse_adjustment(reverse_adjustment),
    m_correction(correction),
    m_nodata_val(nodata_val){}

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t col, size_t row, size_t p=0 ) const {

    if ( m_img(col, row, p) == m_nodata_val ) return m_nodata_val;

    Vector2 lonlat = m_georef.pixel_to_lonlat(Vector2(col, row));

    // For testing (see the link to the reference web form belows).
    //lonlat[0] = -121;   lonlat[1] = 37;   // mainland US
    //lonlat[0] = -152;   lonlat[1] = 66;   // Alaska
    //lonlat[0] = -155.5; lonlat[1] = 19.5; // Hawaii

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
    geoid_height                             = interp_val.child() + m_correction;
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
                              m_geoid, m_geoid_georef, m_reverse_adjustment, m_correction, m_nodata_val );
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
           GeoReference const& geoid_georef, bool reverse_adjustment, double correction, double nodata_val) {
  return DemGeoidView<ImageT>( img.impl(), georef, geoid, geoid_georef,
                               reverse_adjustment, correction, nodata_val );
}

struct Options : asp::BaseOptions {
  string dem_name, out_prefix;
  double nodata_value;
  bool use_double;
  bool reverse_adjustment;
};

std::string get_geoid_full_path(std::string geoid_file){

  // We try two ways of finding the path to the geoid file.

  // 1. Using the compile flag, from the GEOID_PATH macro.
#define STR_EXPAND(tok) #tok
#define STR_QUOTE(tok) STR_EXPAND(tok)
  std::string full_path = std::string(STR_QUOTE(GEOID_PATH)) + "/" + geoid_file;
  if (fs::exists(full_path)) return full_path;

  // 2. Using the ASP_DATA path.
  char * asp_data = getenv("ASP_DATA");
  if (asp_data == NULL){
    vw_throw( ArgumentErr() << "The environmental variable ASP_DATA was not set. "
              << "It should point to the 'share' directory of your ASP distribution.\n" );
  }

  // Must keep this version synchronized with Packages.py!
  full_path = std::string(asp_data) + "/geoids-1.1/" + geoid_file;
  if (!fs::exists(full_path)){
    vw_throw( ArgumentErr() << "Could not find geoid: " << full_path << ".\n"
              << "The value of environmental variable ASP_DATA is '" << asp_data << "'.\n");
  }

  return full_path;
}

void handle_arguments( int argc, char *argv[], Options& opt ){

  po::options_description general_options("");
  general_options.add_options()
    ("nodata_value", po::value(&opt.nodata_value)->default_value(-32768),
     "The value of no-data pixels, unless specified in the DEM.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
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

  if ( opt.dem_name.empty() )
    vw_throw( ArgumentErr() << "Requires <dem> in order to proceed.\n\n"
              << usage << general_options );

  if ( opt.out_prefix.empty() )
    opt.out_prefix = fs::path(opt.dem_name).stem().string();
  
  // Create the output directory 
  asp::create_out_dir(opt.out_prefix);
  
  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);
  
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

// We support three geoids: EGM96, NAVD88 (Earth) and MOLA MEGDR (Mars).
// Online tools for verification:
// EGM96:  http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/intpt.html
// NAVD88: http://www.ngs.noaa.gov/cgi-bin/GEOID_STUFF/geoid09_prompt1.prl

// MOLA MEGDR: http://geo.pds.nasa.gov/missions/mgs/megdr.html

// Examples: At lat = 37 and lon = -121 (Basalt Hills, CA), we have
// EGM96  geoid height = -32.69 m
// NAVD88 geoid height = -32.931 m
// At lat = 66 and lon = -152 (Alaska):
// EGM96 geoid height  = 9.51 m
// NAVD88 geoid height = 8.255 m
// At lat = 19.5 and lon = -155.5 (Hawaii)
// NAVD88 geoid height: 27.353
// EGM96 geoid height:  26.58

int main( int argc, char *argv[] ) {

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

    // Find out the datum from the DEM. If we fail, we do an educated guess.
    std::string datum_name = dem_georef.datum().name();
    std::string lname = boost::to_lower_copy(datum_name);
    std::string geoid_file;
    if ( lname == "wgs_1984" || lname == "wgs 1984" || lname == "wgs1984" ||
         lname == "wgs84" || lname == "world geodetic system 1984" ){
      geoid_file = "egm96-5.tif";
    }else if (lname == "north_american_datum_1983"){
      geoid_file = "navd88.tif";
    }else if (lname == "d_mars"){
      geoid_file = "mola_areoid.tif";
    }else if ( std::abs( dem_georef.datum().semi_major_axis() - 6378137.0 ) < 500.0){
      // Guess Earth
      vw_out(WarningMessage) << "Unknown datum: " << datum_name << ". Guessing: WGS_1984.\n";
      geoid_file = "egm96-5.tif";
    }else if ( std::abs( dem_georef.datum().semi_major_axis() - 3396190.0) < 500.0){
      // Guess Mars
      vw_out(WarningMessage) << "Unknown datum: " << datum_name << ". Guessing: D_MARS.\n";
      geoid_file = "mola_areoid.tif";
    }else{
      vw_throw( ArgumentErr() << "Cannot apply geoid adjustment to DEM relative to datum: "
                << datum_name << "\n");
    }
    geoid_file = get_geoid_full_path(geoid_file);
    vw_out() << "Adjusting the DEM using the geoid: " << geoid_file << endl;

    // Read the geoid containing the adjustments. Read it in memory
    // entirely to dramatically speed up the computations.
    double geoid_nodata_val = std::numeric_limits<float>::quiet_NaN();
    DiskImageResourceGDAL geoid_rsrc(geoid_file);
    if ( geoid_rsrc.has_nodata_read() ) {
      geoid_nodata_val = geoid_rsrc.nodata_read();
    }
    ImageView<float> geoid_img = DiskImageView<float>(geoid_rsrc);
    GeoReference geoid_georef;
    read_georeference(geoid_georef, geoid_rsrc);

    // Need to apply an extra correction if the datum radius of the geoid is different
    // than the datum radius of the DEM to correct. We do this only if the datum is
    // a sphere, such on mars, as otherwise a uniform correction won't work.
    double major_correction
      = geoid_georef.datum().semi_major_axis() - dem_georef.datum().semi_major_axis();
    double minor_correction
      = geoid_georef.datum().semi_minor_axis() - dem_georef.datum().semi_minor_axis();
    if (major_correction != 0){
      if (std::abs(1.0 - minor_correction/major_correction) > 1.0e-5){
        vw_throw( ArgumentErr() << "The input DEM and geoid datums are incompatible. "
                  << "Cannot apply geoid adjustment.\n" );
      }
      vw_out(WarningMessage) << "Will compensate for the fact that the input DEM and geoid datums "
                             << "axis lengths differ.\n";
    }

    ImageViewRef<PixelMask<double> > geoid
      = interpolate(create_mask( pixel_cast<double>(geoid_img), geoid_nodata_val ),
                    BicubicInterpolation(), ZeroEdgeExtension());

    ImageViewRef<double> adj_dem = dem_geoid(dem_img, dem_georef, geoid, geoid_georef,
                                             reverse_adjustment, major_correction, dem_nodata_val);

    std::string adj_dem_file = opt.out_prefix + "-adj.tif";
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
