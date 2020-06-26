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

/// Fortran function declaration from the "geoid" mini external library
/// compiled using Binary Builder.
/// TODO: Must put this into a C++ header file in that library, and include
/// that header file here. Must also make sure that header file gets installed.
extern "C" {
  void egm2008_call_interp_(int* nriw2, int* nciw2, double* grid,
                            double* flon, double* flat, double* val);
}

#include <vw/Image/Interpolation.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

#include <boost/filesystem.hpp>
#include <boost/dll.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::cartography;
using namespace std;

/// Image view which adds or subtracts the ellipsoid/geoid difference
///  from elevations in a DEM image.
template <class ImageT>
class DemGeoidView : public ImageViewBase<DemGeoidView<ImageT> >
{
  ImageT                m_img; ///< The DEM
  GeoReference   const& m_georef;
  bool                  m_is_egm2008;
  vector<double>                   const& m_egm2008_grid; ///< Special variable storing EGM2008 data
  ImageViewRef<PixelMask<double> > const& m_geoid; ///< Interpolation view of the geoid
  GeoReference                     const& m_geoid_georef;
  bool     m_reverse_adjustment; ///< If true, convert from orthometric height to geoid height
  double   m_correction;
  double   m_nodata_val;

public:

  typedef double pixel_type;
  typedef double result_type;
  typedef ProceduralPixelAccessor<DemGeoidView> pixel_accessor;


  DemGeoidView(ImageT const& img, GeoReference const& georef,
               bool is_egm2008, vector<double> const& egm2008_grid,
               ImageViewRef<PixelMask<double> > const& geoid,
               GeoReference const& geoid_georef, bool reverse_adjustment,
               double correction, double nodata_val):
    m_img(img), m_georef(georef),
    m_is_egm2008(is_egm2008), m_egm2008_grid(egm2008_grid),
    m_geoid(geoid), m_geoid_georef(geoid_georef),
    m_reverse_adjustment(reverse_adjustment),
    m_correction(correction),
    m_nodata_val(nodata_val){}

  inline int32 cols  () const { return m_img.cols(); }
  inline int32 rows  () const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t col, size_t row, size_t p=0 ) const {

    if ( m_img(col, row, p) == m_nodata_val )
      return m_nodata_val; // Skip invalid pixels

    Vector2 lonlat = m_georef.pixel_to_lonlat(Vector2(col, row));

    // For testing (see the link to the reference web form belows).
    //lonlat[0] = -121;   lonlat[1] = 37;   // mainland US
    //lonlat[0] = -152;   lonlat[1] = 66;   // Alaska
    //lonlat[0] = -155.5; lonlat[1] = 19.5; // Hawaii

    // TODO: Does the GeoRef class handle this now?

    // Need to carefully wrap lonlat to the [0, 360) x [-90, 90) box.
    // Note that lon = 25, lat = 91 is the same as lon = 180 + 25, lat = 89
    // as we go through the North pole and show up on the other side.
    while ( fabs(lonlat[1]) > 90.0 ){
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



    result_type geoid_height = 0.0;
    if (m_is_egm2008){
      int nr = m_geoid.rows(), 
          nc = m_geoid.cols();
      // Call fortran function from "geoid" mini external library
      egm2008_call_interp_(&nr, &nc, (double*)&m_egm2008_grid[0],
                           &lonlat[0], &lonlat[1], &geoid_height);
    }else{
      // Use our own interpolation into the geoid image
      Vector2  pix = m_geoid_georef.lonlat_to_pixel(lonlat);
      PixelMask<double> interp_val = m_geoid(pix[0], pix[1]);
      if (!is_valid(interp_val))
        return m_nodata_val;
      geoid_height = interp_val.child();
    }

    geoid_height += m_correction;

    result_type height_above_ellipsoid = m_img(col, row, p);

    // Compute height above the geoid
    // - See the note in the main program about the formula below
    if (m_reverse_adjustment)
      return height_above_ellipsoid + geoid_height;
    else
      return height_above_ellipsoid - geoid_height;
  }

  /// \cond INTERNAL
  typedef DemGeoidView<typename ImageT::prerasterize_type> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return prerasterize_type( m_img.prerasterize(bbox), m_georef,
                              m_is_egm2008, m_egm2008_grid,
                              m_geoid, m_geoid_georef, m_reverse_adjustment,
                              m_correction, m_nodata_val );
  }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }
  /// \endcond
};

// Helper function which uses the class above.
template <class ImageT>
DemGeoidView<ImageT>
dem_geoid( ImageViewBase<ImageT> const& img, GeoReference const& georef,
           bool is_egm2008, vector<double> & egm2008_grid,
           ImageViewRef<PixelMask<double> > const& geoid,
           GeoReference const& geoid_georef, bool reverse_adjustment,
           double correction, double nodata_val) {
  return DemGeoidView<ImageT>( img.impl(), georef,
                               is_egm2008, egm2008_grid,
                               geoid, geoid_georef,
                               reverse_adjustment, correction, nodata_val );
}

/// Parameters for this tool
struct Options : vw::cartography::GdalWriteOptions {
  string dem_path, geoid, out_prefix;
  double nodata_value;
  bool   use_double; // Otherwise use float
  bool   reverse_adjustment;
};

// Get the absolute path to the geoid. It is normally in the share/geoids
// directory of the ASP distribution, but in dev mode the directory
// having the geoids can be set via the ASP_GEOID_DIR env var.
string get_geoid_full_path(std::string const& prog_name, std::string const& geoid_file){

  // Convert to absolute path first, to avoid issues when prog_name
  // is simply "dem_geoid" with no path, when subsequent operations on
  // it will fail.
  char * geoid_dir_ptr = getenv("ASP_GEOID_DIR");
  fs::path geoid_dir;
  if (geoid_dir_ptr != NULL && std::string(geoid_dir_ptr) != "")
    geoid_dir = fs::path(std::string(geoid_dir_ptr));
  else
    geoid_dir = boost::dll::program_location().parent_path().parent_path() / fs::path("share")
      / fs::path("geoids");
  
  fs::path geoid_path = geoid_dir / fs::path(geoid_file);
  if (!fs::exists(geoid_path)) 
    vw_throw( ArgumentErr() << "Could not find the geoid: " << geoid_path.string()
              << ". Set export ASP_GEOID_DIR=/path/to/share/geoids\n");

  return geoid_path.string();
}

void handle_arguments( int argc, char *argv[], Options& opt ){

  po::options_description general_options("");
  general_options.add_options()
    ("nodata_value",    po::value(&opt.nodata_value)->default_value(-32768),
         "The value of no-data pixels, unless specified in the DEM.")
    ("geoid",           po::value(&opt.geoid), 
        "Specify the geoid to use for the given datum. For WGS84 use EGM96 or EGM2008. Default: EGM96. For Mars use MOLA or leave blank. For NAD83 use NAVD88 or leave blank. When not specified it will be auto-detected.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
    ("double",          po::bool_switch(&opt.use_double)->default_value(false)->implicit_value(true),
         "Output using double precision (64 bit) instead of float (32 bit).")
    ("reverse-adjustment",
                        po::bool_switch(&opt.reverse_adjustment)->default_value(false)->implicit_value(true),
        "Go from DEM relative to the geoid to DEM relative to the ellipsoid.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("dem", po::value(&opt.dem_path), "Explicitly specify the DEM.");

  po::positional_options_description positional_desc;
  positional_desc.add("dem", 1);

  string usage("[options] <dem>");
  bool allow_unregistered = false;
  vector<string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if ( opt.dem_path.empty() )
    vw_throw( ArgumentErr() << "Requires <dem> in order to proceed.\n\n"
              << usage << general_options );

  boost::to_lower(opt.geoid);

  if ( opt.out_prefix.empty() )
    opt.out_prefix = fs::path(opt.dem_path).stem().string();

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

}

/// Given a DEM, with each height value relative to the datum
/// ellipsoid, convert the heights to be relative to the geoid.

// From: http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/intpthel.html
// Geoid heights can be used to convert between orthometric
// heights (approximately mean sea level) and ellipsoid heights
// according to the formula:
// h   =   H   +   N
// where,
// h = WGS 84 Ellipsoid height
// H = Orthometric height
// N = EGM96 Geoid height (relative to the ellipsoid)
// Therefore: H = h - N.

// We support four geoids: EGM96, EGM2008, NAVD88 (Earth) and MOLA MEGDR (Mars).
// Online tools for verification:
// EGM96:  http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/intpt.html
// NAVD88: http://www.ngs.noaa.gov/cgi-bin/GEOID_STUFF/geoid09_prompt1.prl

// EGM2008: http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm2008/egm08_wgs84.html
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

// For EGM2008, we use the tabulated values at 2.5', and the provided routine
// for interpolation (interp_2p5min.f), as described at
// http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm2008/egm08_wgs84.html
// We store the tabulated values as a tif file, and modified
// the routine to be callable from this executable.






int main( int argc, char *argv[] ) {

  Options opt;
  try {
    std::string prog_name = argv[0];
    handle_arguments( argc, argv, opt );

    bool reverse_adjustment = opt.reverse_adjustment;

    // Read the DEM to adjust
    DiskImageResourceGDAL dem_rsrc(opt.dem_path);
    double dem_nodata_val = opt.nodata_value;
    if ( dem_rsrc.has_nodata_read() ) {
      dem_nodata_val = dem_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value for " << opt.dem_path << ": "
               << dem_nodata_val << endl;
    }
    DiskImageView<double> dem_img(dem_rsrc);
    GeoReference dem_georef;
    bool has_georef = read_georeference(dem_georef, dem_rsrc);
    if (!has_georef)
      vw_throw( ArgumentErr() << "Missing georeference for DEM: " << opt.dem_path << "\n" );

    
    // TODO: Improve this handling so it can read DEMS with relevant EPSG codes, etc.
    
    // Find out the datum from the DEM. If we fail, we do an educated guess.
    string datum_name = dem_georef.datum().name();
    string lname      = boost::to_lower_copy(datum_name);
    string geoid_file;
    bool is_wgs84 = false, is_mola = false;
    if ( lname == "wgs_1984" || lname == "wgs 1984" || lname == "wgs1984" ||
         lname == "wgs84"    || lname == "world geodetic system 1984" ){
      is_wgs84 = true;
    }else if (lname == "north_american_datum_1983"){
      geoid_file = "navd88.tif";
    }else if (lname == "d_mars"){
      is_mola = true;
    }else if ( fabs( dem_georef.datum().semi_major_axis() - 6378137.0 ) < 500.0){
      // Guess Earth
      vw_out(WarningMessage) << "Unknown datum: " << datum_name << ". Guessing: WGS_1984.\n";
      is_wgs84 = true;
    }else if ( fabs( dem_georef.datum().semi_major_axis() - 3396190.0) < 500.0){
      // Guess Mars
      vw_out(WarningMessage) << "Unknown datum: " << datum_name << ". Guessing: D_MARS.\n";
      is_mola = true;
    }else{
      vw_throw( ArgumentErr() << "Cannot apply geoid adjustment to DEM relative to datum: "
                << datum_name << "\n");
    }

    // Ensure that the value of --geoid is compatible with the datum from the DEM.
    // Only WGS_1984 datums can be used with EGM geoids, only the NAD83 datum
    // can be used with NAVD88, and only MOLA can be used with Mars.
    bool is_egm2008 = false;
    if (is_wgs84){
      if (opt.geoid == "egm2008"){
        is_egm2008 = true;
        geoid_file = "egm2008.jp2";
      }else if (opt.geoid == "egm96" || opt.geoid == "")
        geoid_file = "egm96-5.jp2"; // The default WGS84 geoid option
      else
        vw_throw( ArgumentErr() << "The datum is WGS84. The only supported options for the geoid are EGM96 and EGM2008. Got instead: " << opt.geoid << ".\n");
    }else if (lname == "north_american_datum_1983"){
      if (opt.geoid != "" && opt.geoid != "navd88")
        vw_throw( ArgumentErr() << "The datum is North_American_Datum_1983. "
                                << "Hence the value of the --geoid option must be either "
                                << "empty (auto-detected) or NAVD88. Got instead: "
                                << opt.geoid << ".\n");
    }else if (is_mola){
      if (opt.geoid != "" && opt.geoid != "mola")
        vw_throw( ArgumentErr() << "Detected a Mars DEM. In that case, the "
                                << "value of the --geoid option must be either empty "
                                << "(auto-detected) or MOLA. Got instead: " << opt.geoid << ".\n");

    }else if (opt.geoid != "")
      vw_throw( ArgumentErr() << "The geoid value: " << opt.geoid
                              << " is applicable only for the WGS_1984 datum.\n");

    if (is_mola)
      geoid_file = "mola_areoid.tif";

    // Find where we keep the information for this geoid
    geoid_file = get_geoid_full_path(prog_name, geoid_file);
    vw_out() << "Adjusting the DEM using the geoid: " << geoid_file << endl;

    // Read the geoid containing the adjustments. Read it in memory
    // entirely to dramatically speed up the computations.
    double geoid_nodata_val = numeric_limits<float>::quiet_NaN();
    DiskImageResourceGDAL geoid_rsrc(geoid_file);
    if ( geoid_rsrc.has_nodata_read() ) {
      geoid_nodata_val = geoid_rsrc.nodata_read();
    }
    ImageView<float> geoid_img = DiskImageView<float>(geoid_rsrc);
    GeoReference geoid_georef;
    read_georeference(geoid_georef, geoid_rsrc);

    if (is_wgs84 && !is_egm2008){
      // Convert the egm96 int16 JPEG2000-encoded geoid to float.
      double a =  0, 
             b =  65534, // TODO: What is this?
             c = -108, 
             d =  86, 
             s = (d-c)/(b-a);
      for (int col = 0; col < geoid_img.cols(); col++){
        for (int row = 0; row < geoid_img.rows(); row++){
          geoid_img(col, row) = s*(geoid_img(col, row) - a) + c;
        }
      }
    }

    // The EGM2008 case is special. Then, we don't do bicubic interpolation into
    // geoid_img, rather, we invoke some Fortran routine, which gives more accurate results.
    // And we scale the int16 JPEG2000-encoded geoid to float.
    vector<double> egm2008_grid;
    if (is_egm2008){
      double a  =  0,  
             b  =  65534, // TODO: What is this?
             c  = -107, 
             d  =  86, 
             s  = (d-c)/(b-a);
      int    nr = geoid_img.rows(), 
             nc = geoid_img.cols();
      egm2008_grid.resize(nr*nc);
      for (int col = 0; col < nc; col++){
        for (int row = 0; row < nr; row++){
          double val = geoid_img(col, row);
          val = s*(val - a) + c;
          egm2008_grid[row + col*nr] = val; // that is, egm2008_grid(row, col) = val;
        }
      }
    }

    // Need to apply an extra correction if the datum radius of the geoid is different
    // than the datum radius of the DEM to correct. We do this only if the datum is
    // a sphere, such on mars, as otherwise a uniform correction won't work.
    double major_correction = 0.0;
    double minor_correction = 0.0;
    if (!is_egm2008){
      major_correction = geoid_georef.datum().semi_major_axis() - dem_georef.datum().semi_major_axis();
      minor_correction = geoid_georef.datum().semi_minor_axis() - dem_georef.datum().semi_minor_axis();
    }
    if (major_correction != 0){
      if (fabs(1.0 - minor_correction/major_correction) > 1.0e-5){
        vw_throw( ArgumentErr() << "The input DEM and geoid datums are incompatible. "
                  << "Cannot apply geoid adjustment.\n" );
      }
      vw_out(WarningMessage) << "Will compensate for the fact that the input DEM and geoid datums "
                             << "axis lengths differ.\n";
    }

    // Put an interpolation and mask wrapper around the input geoid file
    ImageViewRef<PixelMask<double> > geoid
      = interpolate(create_mask( pixel_cast<double>(geoid_img), 
                                 geoid_nodata_val ),
                    BicubicInterpolation(), ZeroEdgeExtension());

    //vw_out() << "Input DEM georef: " << dem_georef << std::endl;
    //vw_out() << "Geoid georef: " << geoid_georef << std::endl;

    // Set up conversion image view
    ImageViewRef<double> adj_dem = dem_geoid(dem_img, dem_georef,
                                             is_egm2008, egm2008_grid,
                                             geoid, geoid_georef,
                                             reverse_adjustment, major_correction, dem_nodata_val);

    string adj_dem_file = opt.out_prefix + "-adj.tif";
    vw_out() << "Writing adjusted DEM: " << adj_dem_file << endl;

    std::map<std::string, std::string> keywords;
    keywords["GEOID"] = opt.geoid;
    GdalWriteOptions geo_opt;

    if ( opt.use_double ) {
      // Output as double
      block_write_gdal_image(adj_dem_file, adj_dem, true, dem_georef,true, dem_nodata_val, geo_opt,
                             TerminalProgressCallback("asp", "\t--> Applying DEM adjustment: "),
                             keywords);
    }else{
      // Output as float
      ImageViewRef<float> adj_dem_float = channel_cast<float>( adj_dem );

      block_write_gdal_image(adj_dem_file, adj_dem_float, true, dem_georef,true, dem_nodata_val, geo_opt,
                             TerminalProgressCallback("asp", "\t--> Applying DEM adjustment: "),
                             keywords);
    }


  } ASP_STANDARD_CATCHES;

  return 0;
}
