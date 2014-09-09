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
// Unless USE_GRASSFIRE is set to 0 below, it expects DEMs to have an
// alpha channel with the grassfire weights, which are used for
// blending the DEMs. Such a DEM can be obtained from a regular DEM
// using the VisionWorkbench grassfirealpha command.

// Note 1: In practice, the tool may be more efficient if the entire
// mosaic is written out as one single large image, rather than being
// broken up into tiles. To achieve that, just specify to the tool a
// very large tile size, and use 0 for the tile index in the command
// line options.

// Note 2: The tool can be high on memory usage, so processes for
// individual tiles may need to be run on separate machines.

// To do:
// Add unit tests.
// To do: Get rid of GetImageCorners and ModelParams!

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

// To do: Wipe this!
struct ModelParams {
  std::string inputFilename; 
};

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

Vector4 ComputeGeoBoundary(cartography::GeoReference Geo, int width, int height){

  // Get the lonlat coordinates of the four pixels corners of the image.

  Vector4 corners;
  Vector2 leftTopPixel(0,0);
  Vector2 leftTopLonLat = Geo.pixel_to_lonlat(leftTopPixel);

  Vector2 rightBottomPixel(width-1, height-1);
  Vector2 rightBottomLonLat = Geo.pixel_to_lonlat(rightBottomPixel);

  double minLon = leftTopLonLat(0);
  double minLat = leftTopLonLat(1);
  double maxLon = rightBottomLonLat(0);
  double maxLat = rightBottomLonLat(1);

  if (maxLat<minLat){
    double temp = minLat;
    minLat = maxLat;
    maxLat = temp;
  }

  if (maxLon<minLon){
    double temp = minLon;
    minLon = maxLon;
    maxLon = temp;
  }

  corners(0) = minLon;
  corners(1) = maxLon;
  corners(2) = minLat;
  corners(3) = maxLat;

  return corners;
}


Vector4 getImageCorners(std::string imageFile){

  // Get the four corners of an image, that is the lon-lat coordinates of
  // the pixels in the image corners.

  // Note: Below we assume that the image is float. In fact, for the
  // purpose of calculation of corners, the type of the image being
  // read does not matter.
  DiskImageView<float> image(imageFile);

  GeoReference imageGeo;
  bool is_good = read_georeference(imageGeo, imageFile);
  if (!is_good){
    vw_throw(ArgumentErr() << "No georeference found in " << imageFile << ".\n");
  }
  Vector4 imageCorners = ComputeGeoBoundary(imageGeo, image.cols(), image.rows());
  return imageCorners;
}

class DemMosaicView: public ImageViewBase<DemMosaicView>{
  int m_cols, m_rows;
  bool m_draft_mode;
  vector<ModelParams> & m_modelParamsArray;
  vector< DiskImageView<RealT> > const& m_images;
  vector<cartography::GeoReference> const& m_georefs; 
  cartography::GeoReference m_out_georef;
  vector<RealT> m_nodata_values;
  RealT m_out_nodata_value;

public:
  DemMosaicView(int cols, int rows, bool draft_mode,
                vector<ModelParams> & modelParamsArray,
                vector< DiskImageView<RealT> > const& images,
                vector<cartography::GeoReference> const& georefs,
                cartography::GeoReference const& out_georef,
                vector<RealT> const& nodata_values, RealT out_nodata_value):
    m_cols(cols), m_rows(rows),
    m_draft_mode(draft_mode),
    m_modelParamsArray(modelParamsArray),
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
    
    for (int dem_iter = 0; dem_iter < (int)m_modelParamsArray.size(); dem_iter++){
      
      cartography::GeoReference georef = m_georefs[dem_iter];
      ImageViewRef<RealT> curr_disk_dem = m_images[dem_iter];
      double nodata_value = m_nodata_values[dem_iter];
      
      // The tile corners as pixels in curr_dem
      Vector2 b = georef.lonlat_to_pixel
        (m_out_georef.pixel_to_lonlat(bbox.min()));
      Vector2 e = georef.lonlat_to_pixel
        (m_out_georef.pixel_to_lonlat(bbox.max()));
      if (b[0] > e[0]) std::swap(b[0], e[0]);
      if (b[1] > e[1]) std::swap(b[1], e[1]);
      b = floor(b); e = ceil(e);
      BBox2i curr_box(b[0], b[1], e[0] - b[0], e[1] - b[1]);
      curr_box.expand(BilinearInterpolation::pixel_buffer + 1);
      curr_box.crop(bounding_box(curr_disk_dem));
      if (curr_box.empty()) continue;

      // Use grassfire weights for smooth blending
      ImageView<int> distance =
        grassfire(notnodata(curr_disk_dem, nodata_value));
                  
      int feather_max = max_pixel_value( distance );
      int feather_min = 0;
      if (feather_max < 1) feather_max = 1; // extra precaution
      
      // Make the weights have values between 0 and 1.
      ImageViewRef<RealT> norm_dist;
      norm_dist = pixel_cast<RealT>(1.0 / (feather_max - feather_min) *
                                    clamp(pixel_cast<RealT>(distance) - feather_min,
                                          0.0, feather_max - feather_min));
      
      // Create a crop of the DEM, and let the weights be the alpha channel.
      ImageView<RealGrayA> curr_dem = crop(curr_disk_dem, curr_box);
      ImageView<RealT> crop_dist = crop(norm_dist, curr_box);
      for (int col = 0; col < curr_dem.cols(); col++){
        for (int row = 0; row < curr_dem.rows(); row++){
          curr_dem(col, row).a() = crop_dist(col, row);
        }
      }
      
      ImageViewRef<RealGrayA> interp_dem
        = interpolate(curr_dem, BilinearInterpolation(), ConstantEdgeExtension());
      
      for (int c = 0; c < bbox.width(); c++){
        for (int r = 0; r < bbox.height(); r++){
          Vector2 out_pix(c +  bbox.min().x(), r +  bbox.min().y());
          Vector2 in_pix = georef.lonlat_to_pixel
            (m_out_georef.pixel_to_lonlat(out_pix));
          double x = in_pix[0] - curr_box.min().x();
          double y = in_pix[1] - curr_box.min().y();
          // below must use x <= cols()-1 as x is double
          bool is_good = (x >= 0 && x <= curr_dem.cols()-1 &&
                          y >= 0 && y <= curr_dem.rows()-1 );
          if (!is_good) continue;

          int i0 = round(x), j0 = round(y);
          double tol = 1e-6;
          RealGrayA pval;
          if (fabs(x-i0) < tol && fabs(y-j0) < tol){
            // We are at an integer pixel, save for numerical error.
            // Just borrow pixel's value, and don't interpolate.
            pval = curr_dem(i0, j0);
          }else{
            // If we have weights of 0, that means there are invalid pixels,
            // so skip this point.
            int i = (int)floor(x), j = (int)floor(y);
            if (curr_dem(i,   j  ).a() <= 0 ||
                curr_dem(i+1, j  ).a() <= 0 ||
                curr_dem(i,   j+1).a() <= 0 ||
                curr_dem(i+1, j+1).a() <= 0
                )continue;
            
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
  int tile_size, tile_index;
  Options():has_out_nodata(false){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("Options");
  general_options.add_options()
    ("dem-list-file,l", po::value<string>(&opt.dem_list_file),
     "List of DEM files to mosaic, one per line.")
    ("tr", po::value(&opt.tr)->default_value(0.0),
     "Output DEM resolution in target georeferenced units per pixel. If not specified, use the same resolution as the first DEM to be mosaicked.")
    ("mpp", po::value<double>(&opt.mpp)->default_value(0.0),
     "Output DEM resolution in meters per pixel (at the equator). If not specified, use the same resolution as the first DEM to be mosaicked.")
    ("tile-size", po::value<int>(&opt.tile_size)->default_value(1000000),
     "The maximum size of output DEM tile files to write, in pixels.")
    ("tile-index", po::value<int>(&opt.tile_index)->default_value(0),
     "The index of the tile to save in the list of tiles (starting from zero). If called with given tile size, and no tile index, the tool will print out how many tiles are there.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
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
    
    // Find the lon-lat bounding box of all DEMs
    double big = numeric_limits<double>::max();
    Vector4 ll_bbox(big, -big, big, -big);
    for (int dem_iter = 0; dem_iter < (int)dem_files.size(); dem_iter++){
      std::string curr_file = dem_files[dem_iter];
      Vector4 corners = getImageCorners(curr_file);
      ll_bbox[0] = std::min(ll_bbox[0], corners[0]);
      ll_bbox[1] = std::max(ll_bbox[1], corners[1]);
      ll_bbox[2] = std::min(ll_bbox[2], corners[2]);
      ll_bbox[3] = std::max(ll_bbox[3], corners[3]);
    }

    // Form the georef. The georef of the first DEM is used as initial guess.
    cartography::GeoReference out_georef;
    bool is_good = read_georeference(out_georef, dem_files[0]);
    if (!is_good)
      vw_throw(ArgumentErr() << "No georeference found in " << dem_files[0] << ".\n");

    double spacing = opt.tr;
    if (opt.mpp > 0.0){
      // First convert meters per pixel to degrees per pixel using
      // the datum radius.
      spacing = 360.0*opt.mpp/( 2*M_PI*out_georef.datum().semi_major_axis() );
      // Next, wipe the georef altogether, as we will use lon-lat.
      out_georef.set_geographic();
    }

    // Use desired spacing if user-specified
    if (spacing > 0.0){
      Matrix<double,3,3> transform = out_georef.transform();
      transform.set_identity();
      transform(0, 0) = spacing;
      transform(1, 1) = -spacing;
      out_georef.set_transform(transform);
    }
    
    // Set the lower-left corner
    Vector2 beg_pix = out_georef.lonlat_to_pixel(Vector2(ll_bbox[0], ll_bbox[3]));
    out_georef = crop(out_georef, beg_pix[0], beg_pix[1]);

    // Image size
    Vector2 end_pix = out_georef.lonlat_to_pixel(Vector2(ll_bbox[1], ll_bbox[2]));
    int cols = (int)round(end_pix[0]) + 1; // end_pix is the last pix in the image
    int rows = (int)round(end_pix[1]) + 1;
    
    // Compute the weights, and store the no-data values, pointers
    // to images, and georeferences (for speed).
    vector<ModelParams> modelParamsArray;
    vector<RealT> nodata_values;
    vector< DiskImageView<RealT> > images;
    vector< cartography::GeoReference > georefs;
    for (int dem_iter = 0; dem_iter < (int)dem_files.size(); dem_iter++){
      modelParamsArray.push_back(ModelParams());
      modelParamsArray[dem_iter].inputFilename = dem_files[dem_iter];
      images.push_back(DiskImageView<RealT>( dem_files[dem_iter] ));

      cartography::GeoReference geo;
      bool is_good = read_georeference(geo, dem_files[dem_iter]);
      if (!is_good)
        vw_throw(ArgumentErr() << "No georeference found in " << dem_files[dem_iter] << ".\n");
      georefs.push_back(geo);
      
      double curr_nodata_value = opt.out_nodata_value;
      DiskImageResourceGDAL in_rsrc(dem_files[dem_iter]);
      if ( in_rsrc.has_nodata_read() ) curr_nodata_value = in_rsrc.nodata_read();
      nodata_values.push_back(curr_nodata_value);
    }

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

    ImageViewRef<RealT> out_dem
      = crop(DemMosaicView(cols, rows, opt.draft_mode,
                           modelParamsArray, images, georefs,
                           out_georef, nodata_values, opt.out_nodata_value),
             tile_box);
    vw_out() << "Writing: " << dem_tile << std::endl;
    cartography::GeoReference crop_georef
      = crop(out_georef, tile_box.min().x(), tile_box.min().y());
    block_write_gdal_image(dem_tile, out_dem, crop_georef, opt.out_nodata_value,
                           opt, TerminalProgressCallback("asp", ""));

  } ASP_STANDARD_CATCHES;
  
  return 0;
}

