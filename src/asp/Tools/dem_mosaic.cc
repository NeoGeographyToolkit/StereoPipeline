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
typedef PixelGrayA<RealT> RealGrayA;

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

BBox2 point_to_pixel_bbox(GeoReference const& georef, BBox2 const& ptbox){

  // Given the corners in the projected space, find the pixel corners.

  BBox2 pix_box;
  Vector2 cr[] = {ptbox.min(), ptbox.max(),
                  Vector2(ptbox.min().x(), ptbox.max().y()),
                  Vector2(ptbox.max().x(), ptbox.min().y())};
  for (int icr = 0; icr < (int)(sizeof(cr)/sizeof(Vector2)); icr++)
    pix_box.grow( georef.point_to_pixel(cr[icr]) );

  return pix_box;
}
  
class DemMosaicView: public ImageViewBase<DemMosaicView>{
  int m_cols, m_rows, m_erode_len, m_blending_len;
  bool m_draft_mode;
  vector< DiskImageView<RealT> > const& m_images;
  vector<cartography::GeoReference> const& m_georefs; 
  cartography::GeoReference m_out_georef;
  vector<RealT> m_nodata_values;
  RealT m_out_nodata_value;

public:
  DemMosaicView(int cols, int rows, int erode_len, int blending_len,
                bool draft_mode,
                vector< DiskImageView<RealT> > const& images,
                vector<cartography::GeoReference> const& georefs,
                cartography::GeoReference const& out_georef,
                vector<RealT> const& nodata_values, RealT out_nodata_value):
    m_cols(cols), m_rows(rows), m_erode_len(erode_len),
    m_blending_len(blending_len), m_draft_mode(draft_mode),
    m_images(images), m_georefs(georefs),
    m_out_georef(out_georef), m_nodata_values(nodata_values),
    m_out_nodata_value(out_nodata_value){}
  
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

    ImageView<pixel_type> tile(bbox.width(), bbox.height());
    ImageView<RealT>      weights(bbox.width(), bbox.height());
    fill( tile, m_out_nodata_value );
    fill( weights, 0.0 );
    
    for (int dem_iter = 0; dem_iter < (int)m_images.size(); dem_iter++){
      
      cartography::GeoReference georef = m_georefs[dem_iter];
      ImageViewRef<RealT> disk_dem = m_images[dem_iter];
      double nodata_value = m_nodata_values[dem_iter];
      
      // The tile corners as pixels in curr_dem
      BBox2 point_box = m_out_georef.pixel_to_point_bbox(bbox);
      BBox2 pix_box = point_to_pixel_bbox(georef, point_box);
      pix_box.min() = floor(pix_box.min());
      pix_box.max() = ceil(pix_box.max());

      // Grow to account for blending and erosion length, etc.
      pix_box.expand(m_erode_len + m_blending_len + BilinearInterpolation::pixel_buffer + 1);
      pix_box.crop(bounding_box(disk_dem));
      if (pix_box.empty()) continue;

      // Crop the disk dem to a 2-channel in-memory image. First channel
      // is the image pixels, second will be the grassfire weights.
      ImageView<RealGrayA> dem = crop(disk_dem, pix_box);

      // Use grassfire weights for smooth blending
      ImageView<RealT> local_wts =
        grassfire(notnodata(select_channel(dem, 0), nodata_value));

      // Dump the weights
      //std::ostringstream os;
      //os << "weights_" << dem_iter << ".tif";
      //std::cout << "Writing: " << os.str() << std::endl;
      //block_write_gdal_image(os.str(), local_wts, georef, -100,
      //                       asp::BaseOptions(),
      //                       TerminalProgressCallback("asp", ""));

      int max_cutoff = max_pixel_value(local_wts);
      int min_cutoff = m_erode_len;
      if (max_cutoff <= min_cutoff) max_cutoff = min_cutoff + 1; // extra precaution
      
      // Erode
      local_wts = clamp(local_wts - min_cutoff, 0.0, max_cutoff - min_cutoff);
    
      // Create a crop of the DEM, and let the weights be the alpha channel.
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
          Vector2 in_pix = georef.point_to_pixel
            (m_out_georef.pixel_to_point(out_pix));
          double x = in_pix[0] - pix_box.min().x();
          double y = in_pix[1] - pix_box.min().y();
          // below must use x <= cols()-1 as x is double
          bool is_good = (x >= 0 && x <= dem.cols()-1 &&
                          y >= 0 && y <= dem.rows()-1 );
          if (!is_good) continue;

          int i0 = round(x), j0 = round(y);
          RealGrayA pval;
          if (fabs(x-i0) < g_tol && fabs(y-j0) < g_tol){
            // We are at an integer pixel, save for numerical error.
            // Just borrow pixel's value, and don't interpolate.
            // Interpolation can result in invalid pixels if the
            // current pixel is valid but its neighbors are not.
            pval = dem(i0, j0);
          }else{
            // If we have weights of 0, that means there are invalid
            // pixels, so skip this point.
            int i = (int)floor(x), j = (int)floor(y);
            if (dem(i,   j  ).a() <= 0 || dem(i+1, j  ).a() <= 0 ||
                dem(i,   j+1).a() <= 0 || dem(i+1, j+1).a() <= 0)continue;
            pval = interp_dem(x, y);
          }
          double val = pval.v();
          double wt = pval.a();
          
          if (wt <= 0) continue;

          // Initialize the tile if not done already
          if ( tile(c, r) == m_out_nodata_value || isnan(tile(c, r)) )
            tile(c, r) = 0;
          
          if (!m_draft_mode){
            // Combine the values
            tile(c, r) += wt*val;
            weights(c, r) += wt;
          }else{
            // Use just the last value
            tile(c, r) = val;
            weights(c, r) = 1;
          }
          
        }
      }
      
    } // end iterating over DEMs
    
    // Divide by the weights
    int num_valid_pixels = 0;
    for (int c = 0; c < bbox.width(); c++){
      for (int r = 0; r < bbox.height(); r++){
        if ( weights(c, r) > 0 ){
          tile(c, r) /= weights(c, r);
          num_valid_pixels++;
        }
      }
    }

    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

struct Options : asp::BaseOptions {
  bool draft_mode;
  string dem_list_file, out_prefix;
  double mpp, tr;
  bool has_out_nodata;
  RealT out_nodata_value;
  int tile_size, tile_index, erode_len, blending_len;
  Options():has_out_nodata(false){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("Options");
  general_options.add_options()
    ("dem-list-file,l", po::value<string>(&opt.dem_list_file),
     "Text file listing the DEM files to mosaic, one per line.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
    ("tile-size", po::value<int>(&opt.tile_size)->default_value(1000000),
     "The maximum size of output DEM tile files to write, in pixels.")
    ("tile-index", po::value<int>(&opt.tile_index)->default_value(0),
     "The index of the tile to save (starting from zero). When this program is invoked, it will print  out how many tiles are there.")
    ("erode-length", po::value<int>(&opt.erode_len)->default_value(0),
     "Erode input DEMs by this many pixels at boundary and hole edges before mosacking them.")
    ("blending-length", po::value<int>(&opt.blending_len)->default_value(200),
     "Larger values of this number (measured in input DEM pixels) may result in smoother blending while using more memory and computing time.")
    ("tr", po::value(&opt.tr)->default_value(0.0),
     "Output DEM resolution in target georeferenced units per pixel. If not specified, use the same resolution as the first DEM to be mosaicked.")
    ("output-nodata-value", po::value<RealT>(&opt.out_nodata_value),
     "No-data value to use on output. If not specified, use the one from the first DEM to be mosaicked.")
    ("draft-mode", po::bool_switch(&opt.draft_mode)->default_value(false),
     "Put the DEMs together without blending them (the result is less smooth).")
    ("threads", po::value<int>(&opt.num_threads),
     "Number of threads to use.")
    ("help,h", "Display this help message.");

  // Parse options
  po::options_description options("Allowed Options");
  options.add(general_options);

  po::options_description positional("");
  po::positional_options_description positional_desc;
    
  std::string usage("[options] -l dems_list.txt -o output_file_prefix");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  // Error checking
  if (opt.dem_list_file == "")
    vw_throw(ArgumentErr() << "No list of DEMs was specified.\n"
              << usage << general_options );
  if (opt.mpp > 0.0 && opt.tr > 0.0)
    vw_throw(ArgumentErr() << "Just one of the --mpp and --tr options needs to be set.\n"
              << usage << general_options );
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
             << "be set and positive.\n"
              << usage << general_options );
  if (opt.tile_index < 0)
    vw_throw(ArgumentErr() << "The index of the tile to save must be set "
             << "and non-negative.\n"
              << usage << general_options );

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

    // Read the DEMs to mosaic
    std::vector<std::string> dem_files;
    {
      ifstream is(opt.dem_list_file.c_str());
      string file;
      while (is >> file) dem_files.push_back(file);
      if (dem_files.empty())
        vw_throw(ArgumentErr() << "No DEM files to mosaic.\n");
      is.close();
    }
    
    // Read nodata from first DEM, unless the user chooses to specify it.
    if (!opt.has_out_nodata){
      DiskImageResourceGDAL in_rsrc(dem_files[0]);
      if (in_rsrc.has_nodata_read()) opt.out_nodata_value = in_rsrc.nodata_read();
    }
    vw_out() << "Using output no-data value: " << opt.out_nodata_value << endl;
    
    // Store the no-data values, pointers to images, and georeferences
    // (for speed). Find the bounding box of all DEMs in the projected
    // space.
    vw_out() << "Reading the input DEMs.\n";
    TerminalProgressCallback tpc("", "\t--> ");
    tpc.report_progress(0);
    double inc_amount = 1.0 / double(dem_files.size() );
    vector<RealT> nodata_values;
    vector< DiskImageView<RealT> > images;
    vector< cartography::GeoReference > georefs;
    BBox2 mosaic_bbox;
    for (int dem_iter = 0; dem_iter < (int)dem_files.size(); dem_iter++){
      images.push_back(DiskImageView<RealT>( dem_files[dem_iter] ));

      cartography::GeoReference geo;
      bool is_good = read_georeference(geo, dem_files[dem_iter]);
      if (!is_good)
        vw_throw(ArgumentErr() << "No georeference found in "
                 << dem_files[dem_iter] << ".\n");
      georefs.push_back(geo);
      
      double curr_nodata_value = opt.out_nodata_value;
      DiskImageResourceGDAL in_rsrc(dem_files[dem_iter]);
      if ( in_rsrc.has_nodata_read() ) curr_nodata_value = in_rsrc.nodata_read();
      nodata_values.push_back(curr_nodata_value);

      mosaic_bbox.grow(georefs[dem_iter].bounding_box
                       (DiskImageView<RealT>(dem_files[dem_iter])));
      tpc.report_incremental_progress( inc_amount );
    }
    tpc.report_finished();

    // Form the mosaic georef. The georef of the first DEM is used as
    // initial guess.
    cartography::GeoReference out_georef = georefs[0];
    double spacing = opt.tr;
    // Use desired spacing if user-specified
    if (spacing > 0.0){
      Matrix<double,3,3> transform = out_georef.transform();
      transform.set_identity();
      transform(0, 0) = spacing;
      transform(1, 1) = -spacing;
      out_georef.set_transform(transform);
    }

    // Set the lower-left corner.
    // Note: The position of the corner is somewhat arbitrary.
    // If the corner is actually very close to an integer number,
    // we assume it should in fact be integer but got moved a bit
    // due to numerical error. Then we set it to integer. This ensures
    // that when we mosaic a single DEM we get its corners to be the
    // same as the originals rather than moved by a slight offset.
    BBox2 pixel_box = point_to_pixel_bbox(out_georef, mosaic_bbox);
    Vector2 beg_pix = pixel_box.min();
    if (norm_2(beg_pix - round(beg_pix)) < g_tol ) beg_pix = round(beg_pix);
    out_georef = crop(out_georef, beg_pix[0], beg_pix[1]);

    // Image size
    pixel_box = point_to_pixel_bbox(out_georef, mosaic_bbox);
    Vector2 end_pix = pixel_box.max();
    
    int cols = (int)round(end_pix[0]); // end_pix is the last pix in the image
    int rows = (int)round(end_pix[1]);
    
    // Form the mosaic and write it to disk
    vw_out()<< "The size of the mosaic is " << cols << " x " << rows
            << " pixels.\n";

    int num_tiles_x = (int)ceil((double)cols/double(opt.tile_size));
    if (num_tiles_x <= 0) num_tiles_x = 1;
    int num_tiles_y = (int)ceil((double)rows/double(opt.tile_size));
    if (num_tiles_y <= 0) num_tiles_y = 1;
    int num_tiles = num_tiles_x*num_tiles_y;
    vw_out() << "Number of tiles: " << num_tiles_x << " x "
             << num_tiles_y << " = " << num_tiles << std::endl;

    if (opt.tile_index < 0 || opt.tile_index >= num_tiles){
      vw_out() << "Tile with index: " << opt.tile_index << " is out of bounds."
               << std::endl;
      return 0;
    }
    int tile_index_y = opt.tile_index / num_tiles_y;
    int tile_index_x = opt.tile_index - tile_index_y*num_tiles_y;
    BBox2i tile_box(tile_index_x*opt.tile_size, tile_index_y*opt.tile_size,
                    opt.tile_size, opt.tile_size);
    tile_box.crop(BBox2i(0, 0, cols, rows));
    ostringstream os; os << opt.out_prefix << "-tile-" << opt.tile_index << ".tif";
    std::string dem_tile = os.str();

    // The next power of 2 >= 4*(blending_len + erode_len). We want to
    // make the blocks big, to reduce overhead from blending_len and
    // erode_len, but not so big that it may not fit in memory.
    int block_size = ( 1 << (int)ceil(log(4.0*max(1, opt.erode_len
                                                  + opt.blending_len))/log(2.0)) );
    block_size = std::max(block_size, 256); // don't make them too small though
    
    // We use block_cache to rasterize tiles of size block_size.
    ImageViewRef<RealT> out_dem = 
      block_cache(crop(DemMosaicView(cols, rows, opt.erode_len, opt.blending_len,
                                     opt.draft_mode, images, georefs,
                                     out_georef, nodata_values,
                                     opt.out_nodata_value),
                       tile_box),
                  Vector2(block_size, block_size), opt.num_threads);
    
    vw_out() << "Writing: " << dem_tile << std::endl;
    cartography::GeoReference crop_georef
      = crop(out_georef, tile_box.min().x(), tile_box.min().y());
    block_write_gdal_image(dem_tile, out_dem, crop_georef, opt.out_nodata_value,
                           opt, TerminalProgressCallback("asp", "\t--> "));

  } ASP_STANDARD_CATCHES;
  
  return 0;
}

