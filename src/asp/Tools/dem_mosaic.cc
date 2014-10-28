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

/// \file dem_mosaic.cc
///

// A tool to mosaic and blend DEMs, and output the mosaic as tiles.

// Note 1: In practice, the tool may be more efficient if the entire
// mosaic is written out as one single large image, rather than being
// broken up into tiles. To achieve that, just specify to the tool a
// very large tile size, and use 0 for the tile index in the command
// line options.

// Note 2: The tool can be high on memory usage, so processes for
// individual tiles may need to be run on separate machines.

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include <limits>
using namespace std;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

using namespace vw;
using namespace vw::cartography;

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/convenience.hpp>
namespace fs = boost::filesystem;

typedef float RealT; // Use double for debugging

// This is used for various tolerances
double g_tol = 1e-6;

// Function for highlighting spots of data
template<class PixelT>
class NotNoDataFunctor {
  typedef typename CompoundChannelType<PixelT>::type channel_type;
  channel_type m_nodata;
  typedef ChannelRange<channel_type> range_type;
public:
  NotNoDataFunctor( channel_type nodata ) : m_nodata(nodata) {}

  template <class Args> struct result {
    typedef channel_type type;
  };

  inline channel_type operator()( channel_type const& val ) const {
    return (val != m_nodata && !isnan(val))? range_type::max() : range_type::min();
  }
};

template <class ImageT, class NoDataT>
UnaryPerPixelView<ImageT,UnaryCompoundFunctor<NotNoDataFunctor<typename ImageT::pixel_type>, typename ImageT::pixel_type>  >
inline notnodata( ImageViewBase<ImageT> const& image, NoDataT nodata ) {
  typedef UnaryCompoundFunctor<NotNoDataFunctor<typename ImageT::pixel_type>, typename ImageT::pixel_type> func_type;
  func_type func( nodata );
  return UnaryPerPixelView<ImageT,func_type>( image.impl(), func );
}

// Set nodata pixels to 0 and valid data pixels to something big.
template<class PixelT>
struct BigOrZero: public ReturnFixedType<PixelT> {
  PixelT m_nodata;
  BigOrZero(PixelT nodata):m_nodata(nodata){}
  double operator() (PixelT const& pix) const {
    if (pix != m_nodata && !isnan(pix)) return 1e+8;
    return 0;
  }
};

BBox2 point_to_pixel_bbox_nogrow(GeoReference const& georef, BBox2 const& ptbox){

  // Given the corners in the projected space, find the pixel corners.
  // This differs from the point_to_pixel_bbox() function in
  // GeoReferenceBase.cc in that in the latter the box is grown to
  // int. Here we prefer finer control.
  
  BBox2 pix_box;
  Vector2 cr[] = {ptbox.min(), ptbox.max(),
                  Vector2(ptbox.min().x(), ptbox.max().y()),
                  Vector2(ptbox.max().x(), ptbox.min().y())};
  for (int icr = 0; icr < (int)(sizeof(cr)/sizeof(Vector2)); icr++)
    pix_box.grow( georef.point_to_pixel(cr[icr]) );

  return pix_box;
}

GeoReference read_georef(std::string const& file){
  // Read a georef, and check for success
  GeoReference geo;
  bool is_good = read_georeference(geo, file);
  if (!is_good)
    vw_throw(ArgumentErr() << "No georeference found in " << file << ".\n");
  return geo;
}

std::string processed_proj4(std::string const& srs){
  // Apparently functionally identical proj4 strings can differ in
  // subtle ways, such as an extra space, etc. For that reason, must
  // parse and process any srs string before comparing it with another
  // string.
  GeoReference georef;
  bool have_user_datum = false;
  Datum user_datum;
  asp::set_srs_string(srs, have_user_datum, user_datum, georef);
  return georef.overall_proj4_str();
}

struct Options : asp::BaseOptions {
  string dem_list_file, out_prefix, target_srs_string;
  vector<string> dem_files;
  double tr, geo_tile_size;
  bool has_out_nodata;
  RealT out_nodata_value;
  int tile_size, tile_index, erode_len, blending_len;
  bool first, last, min, max, mean, median, count;
  BBox2 target_projwin;
  Options(): tr(0), geo_tile_size(0), has_out_nodata(false), tile_index(-1),
             first(false), last(false), min(false), max(false), 
             mean(false), median(false), count(false){}
};

int no_blend(Options const& opt){
  return int(opt.first) + int(opt.last) + int(opt.min) + int(opt.max) 
    + int(opt.mean) + int(opt.median) + int(opt.count);
}

std::string tile_suffix(Options const& opt){
  if (opt.first) return "first-";
  if (opt.last) return "last-";
  if (opt.min) return "min-";
  if (opt.max) return "max-";
  if (opt.mean) return "mean-";
  if (opt.median) return "median-";
  if (opt.count) return "count-";
  return "";
}

class DemMosaicView: public ImageViewBase<DemMosaicView>{
  int m_cols, m_rows;
  Options const& m_opt;
  vector< ImageViewRef<RealT> > const& m_images;
  vector<GeoReference> const& m_georefs; 
  GeoReference m_out_georef;
  vector<RealT> m_nodata_values;
  
public:
  DemMosaicView(int cols, int rows, Options const& opt,
                vector< ImageViewRef<RealT> > const& images,
                vector<GeoReference> const& georefs,
                GeoReference const& out_georef,
                vector<RealT> const& nodata_values):
    m_cols(cols), m_rows(rows), m_opt(opt), 
    m_images(images), m_georefs(georefs),
    m_out_georef(out_georef), m_nodata_values(nodata_values){

    // Sanity check, see if datums differ, then the tool won't work
    for (int i = 0; i < (int)m_georefs.size(); i++){
      if (m_georefs[i].datum().name() != m_out_georef.datum().name()){
        vw_throw(NoImplErr() << "Mosaicking of DEMs with different datums "
                 << "is not implemented. Datums encountered:\n"
                 << m_georefs[i].datum() << "\n"
                 <<  m_out_georef.datum() << "\n");
      }
    }
  }
  
  typedef RealT pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<DemMosaicView> pixel_accessor;
  
  inline int cols() const { return m_cols; }
  inline int rows() const { return m_rows; }
  inline int planes() const { return 1; }
  
  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "DemMosaicView::operator()(...) is not implemented");
    return pixel_type();
  }
  
  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // We will do all computations in double precision, regardless
    // of the precision of the inputs, for increased accuracy.
    typedef PixelGrayA<double> RealGrayA;
    ImageView<double> tile(bbox.width(), bbox.height());
    ImageView<double> weights(bbox.width(), bbox.height());
    fill( tile, m_opt.out_nodata_value );
    fill( weights, 0.0 );

    int noblend = no_blend(m_opt);

    std::vector< ImageView<double> > tiles; // used for median calculation
    
    for (int dem_iter = 0; dem_iter < (int)m_images.size(); dem_iter++){

      GeoReference georef = m_georefs[dem_iter];
      ImageViewRef<double> disk_dem = pixel_cast<double>(m_images[dem_iter]);
      double nodata_value = m_nodata_values[dem_iter];

      // The GeoTransform will hide the messy details of conversions
      // from pixels to points and lon-lat.
      GeoTransform geotrans(georef, m_out_georef);

      BBox2 in_box = geotrans.reverse_bbox(bbox);
      
      // Grow to account for blending and erosion length, etc.
      in_box.expand(m_opt.erode_len + m_opt.blending_len
                    + BilinearInterpolation::pixel_buffer + 1);
      in_box.crop(bounding_box(disk_dem));
      if (in_box.width() == 1 || in_box.height() == 1){
        // Grassfire likes to have width of at least 2
        in_box.expand(1);
        in_box.crop(bounding_box(disk_dem));
      }
      if (in_box.width() <= 1 || in_box.height() <= 1) continue;

      if (m_opt.median){
        // Must use a blank tile each time
        fill( tile, m_opt.out_nodata_value );
        fill( weights, 0.0 );
      }
      
      // Crop the disk dem to a 2-channel in-memory image. First channel
      // is the image pixels, second will be the grassfire weights.
      ImageView<RealGrayA> dem = crop(disk_dem, in_box);

      // Use grassfire weights for smooth blending
      ImageView<double> local_wts = grassfire(notnodata(select_channel(dem, 0),
                                                        nodata_value));
      // Dump the weights
      //std::ostringstream os;
      //os << "weights_" << dem_iter << ".tif";
      //std::cout << "Writing: " << os.str() << std::endl;
      //block_write_gdal_image(os.str(), local_wts, georef, -100,
      //                       asp::BaseOptions(),
      //                       TerminalProgressCallback("asp", ""));

      int max_cutoff = max_pixel_value(local_wts);
      int min_cutoff = m_opt.erode_len;
      if (max_cutoff <= min_cutoff) max_cutoff = min_cutoff + 1; // precaution
      
      // Erode
      local_wts = clamp(local_wts - min_cutoff, 0.0, max_cutoff - min_cutoff);
    
      // Set the weights in the alpha channel
      for (int col = 0; col < dem.cols(); col++){
        for (int row = 0; row < dem.rows(); row++){
          dem(col, row).a() = local_wts(col, row);
        }
      }
      
      ImageViewRef<RealGrayA> interp_dem
        = interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());
      
      for (int c = 0; c < bbox.width(); c++){
        for (int r = 0; r < bbox.height(); r++){

          Vector2 out_pix(c +  bbox.min().x(), r +  bbox.min().y());
          Vector2 in_pix = geotrans.reverse(out_pix);

          double x = in_pix[0] - in_box.min().x();
          double y = in_pix[1] - in_box.min().y();
          RealGrayA pval;

          int i0 = round(x), j0 = round(y);
          if (fabs(x-i0) < g_tol && fabs(y-j0) < g_tol &&
              (i0 >= 0 && i0 <= dem.cols()-1 &&
               j0 >= 0 && j0 <= dem.rows()-1) ){

            // A lot of care is needed here. We are at an integer
            // pixel, save for numerical error. Just borrow pixel's
            // value, and don't interpolate. Interpolation can result
            // in invalid pixels if the current pixel is valid but its
            // neighbors are not. It can also make it appear is if the
            // current indices are out of bounds while in fact they
            // are barely so.
            pval = dem(i0, j0);
            
          }else{
            
            // below must use x <= cols()-1 as x is double
            bool is_good = (x >= 0 && x <= dem.cols()-1 &&
                            y >= 0 && y <= dem.rows()-1);
            if (!is_good) continue;

            // If we have weights of 0, that means there are invalid
            // pixels, so skip this point.
            int i = (int)floor(x), j = (int)floor(y);
            if (dem(i, j  ).a() <= 0 || dem(i+1, j  ).a() <= 0 ||
                dem(i, j+1).a() <= 0 || dem(i+1, j+1).a() <= 0) continue;
            pval = interp_dem(x, y);
          }
          double val = pval.v();
          double wt = pval.a();
          
          if (wt <= 0) continue;

          bool is_nodata = (tile(c, r) == m_opt.out_nodata_value);

          // Initialize the tile if not done already
          if (!m_opt.median && !m_opt.min && !m_opt.max){
            if ( is_nodata ){
              tile(c, r) = 0;
              weights(c, r) = 0.0;
            }
          }

          if ( ( m_opt.first && is_nodata)                        || 
               m_opt.last                                         ||
               ( m_opt.min && ( val < tile(c, r) || is_nodata ) ) ||
               ( m_opt.max && ( val > tile(c, r) || is_nodata ) ) ||
               m_opt.median ){
            tile(c, r) = val;
            weights(c, r) = wt;
          }else if (m_opt.mean){
            tile(c, r) += val;
            weights(c, r)++;
          }else if (m_opt.count){
            tile(c, r)++;
            weights(c, r) += wt;
          }else if (noblend == 0){
            // Weighted average
            tile(c, r) += wt*val;
            weights(c, r) += wt;
          }
        }
      }

      // This will be memory intensive
      if (m_opt.median)
        tiles.push_back(copy(tile));
      
    } // end iterating over DEMs
    
    // Divide by the weights
    if (noblend == 0 || m_opt.mean){
      for (int c = 0; c < bbox.width(); c++){
        for (int r = 0; r < bbox.height(); r++){
          if ( weights(c, r) > 0 ){
            tile(c, r) /= weights(c, r);
          }
        }
      }
    }

    if (m_opt.median){
      fill( tile, m_opt.out_nodata_value );
      vector<double> vals(tiles.size());
      for (int c = 0; c < bbox.width(); c++){
        for (int r = 0; r < bbox.height(); r++){
          vals.clear();
          for (int i = 0; i < (int)tiles.size(); i++){
            ImageView<double> & tile_ref = tiles[i];
            if ( tile_ref(c, r) == m_opt.out_nodata_value ) continue;
            vals.push_back(tile_ref(c, r));
          }
          if (!vals.empty()){
            tile(c, r) = math::destructive_median(vals);
          }
        }
      }
    }

    return prerasterize_type(pixel_cast<RealT>(tile),
                             -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("Options");
  general_options.add_options()
    ("dem-list-file,l", po::value<string>(&opt.dem_list_file),
     "Text file listing the DEM files to mosaic, one per line.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
    ("tile-size", po::value<int>(&opt.tile_size)->default_value(1000000),
     "The maximum size of output DEM tile files to write, in pixels.")
    ("tile-index", po::value<int>(&opt.tile_index),
     "The index of the tile to save (starting from zero). When this program is invoked, it will print  out how many tiles are there. Default: save all tiles.")
    ("erode-length", po::value<int>(&opt.erode_len)->default_value(0),
     "Erode input DEMs by this many pixels at boundary and hole edges before mosaicking them.")
    ("blending-length", po::value<int>(&opt.blending_len)->default_value(200),
     "Larger values of this number (measured in input DEM pixels) may result in smoother blending while using more memory and computing time.")
    ("tr", po::value(&opt.tr),
     "Output DEM resolution in target georeferenced units per pixel. Default: use the same resolution as the first DEM to be mosaicked.")
    ("t_srs", po::value(&opt.target_srs_string)->default_value(""),
     "Specify the output projection (PROJ.4 string). Default: use the one from the first DEM to be mosaicked.")
    ("t_projwin", po::value(&opt.target_projwin),
     "Limit the mosaic to this region, with the corners given in georeferenced coordinates (xmin ymin xmax ymax). Max is exclusive.")
    ("first", po::bool_switch(&opt.first)->default_value(false),
     "Keep the first encountered DEM value (in the input order).")
    ("last", po::bool_switch(&opt.last)->default_value(false),
     "Keep the last encountered DEM value (in the input order).")
    ("min", po::bool_switch(&opt.min)->default_value(false),
     "Keep the smallest encountered DEM value.")
    ("max", po::bool_switch(&opt.max)->default_value(false),
     "Keep the largest encountered DEM value.")
    ("mean", po::bool_switch(&opt.mean)->default_value(false),
     "Find the mean DEM value.")
    ("median", po::bool_switch(&opt.median)->default_value(false),
     "Find the median DEM value (this can be memory-intensive, fewer threads are suggested).")
    ("count", po::bool_switch(&opt.count)->default_value(false),
     "Each pixel is set to the number of valid DEM heights at that pixel.")
    ("georef-tile-size", po::value<double>(&opt.geo_tile_size),
     "Set the tile size in georeferenced (projected) units (e.g., degrees or meters).")
    ("output-nodata-value", po::value<RealT>(&opt.out_nodata_value),
     "No-data value to use on output. Default: use the one from the first DEM to be mosaicked.")
    ("threads", po::value<int>(&opt.num_threads)->default_value(4),
     "Number of threads to use.")
    ("help,h", "Display this help message.");

  // Parse options
  po::options_description options("Allowed Options");
  options.add(general_options);

  po::options_description positional("");
  po::positional_options_description positional_desc;
    
  std::string usage("[options] <dem files or -l dem_files_list.txt> -o output_file_prefix");
  bool allow_unregistered = true;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  // Error checking  
  if (opt.out_prefix == "")
    vw_throw(ArgumentErr() << "No output prefix was specified.\n"
              << usage << general_options );
  if (opt.num_threads == 0)
    vw_throw(ArgumentErr() << "The number of threads must be set and "
             << "positive.\n" << usage << general_options );
  if (opt.erode_len < 0)
    vw_throw(ArgumentErr() << "The erode length must not be negative.\n"
             << usage << general_options );
  if (opt.blending_len < 0)
    vw_throw(ArgumentErr() << "The blending length must not be negative.\n"
             << usage << general_options );
  if (opt.tile_size <= 0)
    vw_throw(ArgumentErr() << "The size of a tile in pixels must "
             << "be positive.\n"
             << usage << general_options );

  int noblend = no_blend(opt);
  
  if (noblend > 1)
    vw_throw(ArgumentErr() << "At most one of the options --first, --last, "
             << "--min, --max, -mean, --median, --count can be specified.\n"
             << usage << general_options );

  if (opt.geo_tile_size < 0)
    vw_throw(ArgumentErr() << "The size of a tile in georeferenced units must "
             << "not be negative.\n"
              << usage << general_options );

  // Read the DEMs
  if (opt.dem_list_file != ""){

    // Get them from a list
    
    if (!unregistered.empty())
      vw_throw(ArgumentErr() << "The DEMs were specified via a list. There were however "
               << "extraneous files or options passed in.\n"
               << usage << general_options );
    
    ifstream is(opt.dem_list_file.c_str());
    string file;
    while (is >> file) opt.dem_files.push_back(file);
    if (opt.dem_files.empty())
      vw_throw(ArgumentErr() << "No DEM files to mosaic.\n");
    is.close();

  }else{

    // Get them from the command line
    if (unregistered.empty())
      vw_throw(ArgumentErr() << "No input DEMs were specified..\n"
               << usage << general_options );
    opt.dem_files = unregistered;
  }
  
  // Create the output directory 
  asp::create_out_dir(opt.out_prefix);
  
  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  if (!vm.count("output-nodata-value")){
    // Set a default out_nodata_value, but remember that this is
    // set internally, not by the user.
    opt.has_out_nodata = false;
    opt.out_nodata_value= -numeric_limits<RealT>::max();
  }else
    opt.has_out_nodata = true;

}

int main( int argc, char *argv[] ) {

  Options opt;
  try{
    
    handle_arguments( argc, argv, opt );

    // Read nodata from first DEM, unless the user chooses to specify it.
    if (!opt.has_out_nodata){
      DiskImageResourceGDAL in_rsrc(opt.dem_files[0]);
      if (in_rsrc.has_nodata_read()) opt.out_nodata_value = in_rsrc.nodata_read();
    }
    vw_out() << "Using output no-data value: " << opt.out_nodata_value << endl;

    // Form the mosaic georef. The georef of the first DEM is used as
    // initial guess unless user wants to change the resolution and
    // projection.
    if (opt.target_srs_string != "")
      opt.target_srs_string = processed_proj4(opt.target_srs_string);
      
    GeoReference out_georef = read_georef(opt.dem_files[0]);
    double spacing = opt.tr;
    if (opt.target_srs_string != ""                                              &&
        opt.target_srs_string != processed_proj4(out_georef.overall_proj4_str()) &&
        spacing <= 0 ){
      vw_throw(ArgumentErr()
               << "Changing the projection was requested. The output DEM "
               << "resolution must be specified via the --tr option.\n");
    }

    if (opt.target_srs_string != ""){
      // Set the srs string into georef.
      bool have_user_datum = false;
      Datum user_datum;
      asp::set_srs_string(opt.target_srs_string,
                          have_user_datum, user_datum, out_georef);
    }

    // Bugfix: steal the datum and its name from the input, if the output
    // datum name is unknown.
    if (out_georef.datum().name() == "unknown"){
      GeoReference georef = read_georef(opt.dem_files[0]);
      if (out_georef.datum().semi_major_axis() == georef.datum().semi_major_axis() &&
          out_georef.datum().semi_minor_axis() == georef.datum().semi_minor_axis() ){
        vw_out() << "Using the datum: " << georef.datum() << std::endl;
        out_georef.set_datum(georef.datum());
      }
    }
    
    // Use desired spacing if user-specified
    if (spacing > 0.0){
      Matrix<double,3,3> transform = out_georef.transform();
      transform.set_identity();
      transform(0, 0) = spacing;
      transform(1, 1) = -spacing;
      out_georef.set_transform(transform);
    }else
      spacing = out_georef.transform()(0, 0);

    // if the user specified the tile size in georeferenced units.
    if (opt.geo_tile_size > 0){
      opt.tile_size = (int)round(opt.geo_tile_size/spacing);
      vw_out() << "Tile size in pixels: " << opt.tile_size << "\n";
    }
    opt.tile_size = std::max(opt.tile_size, 1);
    
    // Store the no-data values, pointers to images, and georeferences
    // (for speed). Find the bounding box of all DEMs in the projected
    // space.
    vw_out() << "Reading the input DEMs.\n";
    TerminalProgressCallback tpc("", "\t--> ");
    tpc.report_progress(0);
    double inc_amount = 1.0 / double(opt.dem_files.size() );
    vector<RealT> nodata_values;
    vector< ImageViewRef<RealT> > images;
    vector< GeoReference > georefs;
    BBox2 mosaic_bbox;
    for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++){
      
      double curr_nodata_value = opt.out_nodata_value;
      DiskImageResourceGDAL in_rsrc(opt.dem_files[dem_iter]);
      if ( in_rsrc.has_nodata_read() ) curr_nodata_value = in_rsrc.nodata_read();
      GeoReference georef = read_georef(opt.dem_files[dem_iter]);
      DiskImageView<RealT> img(opt.dem_files[dem_iter]);

      if (out_georef.overall_proj4_str() == georef.overall_proj4_str()){ 
        mosaic_bbox.grow(georef.bounding_box(img));
      }else{

        // Compute the bounding box of the current image
        // in projected coordinates of the mosaic.
        BBox2 imgbox = bounding_box(img);
        BBox2 lonlat_box = georef.pixel_to_lonlat_bbox(imgbox);

        // Must compensate for the fact that the lonlat
        // of the two images can differ by 360 degrees.
        Vector2 old_orgin = georef.pixel_to_lonlat(Vector2(0, 0));
        Vector2 new_orgin = out_georef.pixel_to_lonlat(Vector2(0, 0));
        Vector2 offset( 360.0*round( (new_orgin[0] - old_orgin[0])/360.0 ), 0.0 );
        lonlat_box += offset;
        BBox2 proj_box = out_georef.lonlat_to_point_bbox(lonlat_box);
        mosaic_bbox.grow(proj_box);
      }
      
      nodata_values.push_back(curr_nodata_value);
      images.push_back(img);
      georefs.push_back(georef);
      
      tpc.report_incremental_progress( inc_amount );
    }
    tpc.report_finished();

    // If to create the mosaic only in a given region
    if (opt.target_projwin != BBox2())
      mosaic_bbox.crop(opt.target_projwin);

    // Set the lower-left corner. Note: The position of the corner is
    // somewhat arbitrary. If the corner is actually very close to an
    // integer number, we assume it should in fact be integer but got
    // moved a bit due to numerical error. Then we set it to
    // integer. This ensures that when we mosaic a single DEM we get
    // its corners to be the same as the originals rather than moved
    // by a slight offset.
    BBox2 pixel_box = point_to_pixel_bbox_nogrow(out_georef, mosaic_bbox);
    Vector2 beg_pix = pixel_box.min();
    if (norm_2(beg_pix - round(beg_pix)) < g_tol ) beg_pix = round(beg_pix);
    out_georef = crop(out_georef, beg_pix[0], beg_pix[1]);

    // Image size
    pixel_box = point_to_pixel_bbox_nogrow(out_georef, mosaic_bbox);
    Vector2 end_pix = pixel_box.max();
    
    int cols = (int)round(end_pix[0]); // end_pix is the last pix in the image
    int rows = (int)round(end_pix[1]);
    
    // Form the mosaic and write it to disk
    vw_out()<< "The size of the mosaic is " << cols << " x " << rows
            << " pixels.\n";

    // The next power of 2 >= 4*(blending_len + erode_len). We want to
    // make the blocks big, to reduce overhead from blending_len and
    // erode_len, but not so big that it may not fit in memory.
    int block_size = nextpow2(4.0*(opt.erode_len + opt.blending_len));
    block_size = std::max(block_size, 256); // don't make them too small though

    int num_tiles_x = (int)ceil((double)cols/double(opt.tile_size));
    if (num_tiles_x <= 0) num_tiles_x = 1;
    int num_tiles_y = (int)ceil((double)rows/double(opt.tile_size));
    if (num_tiles_y <= 0) num_tiles_y = 1;
    int num_tiles = num_tiles_x*num_tiles_y;
    vw_out() << "Number of tiles: " << num_tiles_x << " x "
             << num_tiles_y << " = " << num_tiles << std::endl;

    if (opt.tile_index >= num_tiles){
      vw_out() << "Tile with index: " << opt.tile_index << " is out of bounds."
               << std::endl;
      return 0;
    }

    // See if to save all tiles, or an individual tile.
    int start_tile = opt.tile_index, end_tile = opt.tile_index + 1;
    if (opt.tile_index < 0){
      start_tile = 0;
      end_tile = num_tiles;
    }
    
    for (int tile_id = start_tile; tile_id < end_tile; tile_id++){
      
      int tile_index_y = tile_id / num_tiles_x;
      int tile_index_x = tile_id - tile_index_y*num_tiles_x;
      BBox2i tile_box(tile_index_x*opt.tile_size, tile_index_y*opt.tile_size,
                      opt.tile_size, opt.tile_size);
      tile_box.crop(BBox2i(0, 0, cols, rows));
      ostringstream os;
      os << opt.out_prefix << "-tile-" << tile_suffix(opt) << tile_id << ".tif";
      std::string dem_tile = os.str();
      
      // We use block_cache to rasterize tiles of size block_size.
      ImageViewRef<RealT> out_dem
        = crop(DemMosaicView(cols, rows, opt, images, georefs,
                             out_georef, nodata_values),
               tile_box);
      
      vw_out() << "Writing: " << dem_tile << std::endl;
      opt.raster_tile_size = Vector2(block_size, block_size); // disk block size
      GeoReference crop_georef
        = crop(out_georef, tile_box.min().x(), tile_box.min().y());
      block_write_gdal_image(dem_tile, out_dem, crop_georef, opt.out_nodata_value,
                             opt, TerminalProgressCallback("asp", "\t--> "));

      // We wrote big blocks, as then there's less overhead.
      // But those are hard to manipulate with gdal_translate.
      if (opt.raster_tile_size[0] != 256){
        std::string tmp_tile = fs::path(dem_tile).replace_extension(".tmp.tif").string();
        fs::rename(dem_tile, tmp_tile);
        DiskImageView<RealT> tmp_dem(tmp_tile);
        opt.raster_tile_size = Vector2(256, 256); // disk block size
        vw_out() << "Re-writing with blocks of size: " << opt.raster_tile_size[0] << std::endl;
        block_write_gdal_image(dem_tile, tmp_dem, crop_georef, opt.out_nodata_value,
                               opt, TerminalProgressCallback("asp", "\t--> "));
        fs::remove(tmp_tile);
      }
    }
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

