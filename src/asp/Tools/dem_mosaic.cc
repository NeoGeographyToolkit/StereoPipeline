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

#define USE_GRASSFIRE 1

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

#if USE_GRASSFIRE
typedef PixelGrayA<float> DemPixelT;
#else
typedef float DemPixelT;
#endif

struct ModelParams {
  std::string inputFilename; 
};

void listTifsInDir(const std::string & dirName,
                   std::vector<std::string> & tifsInDir
                   ){
  
  tifsInDir.clear();

  fs::path dir(dirName);
  if ( !fs::exists(dirName) || !fs::is_directory(dirName ) ) return;

  fs::directory_iterator end_iter; // default construction yields past-the-end
  for  ( fs::directory_iterator dir_iter(dirName); dir_iter != end_iter; ++dir_iter)
    {
      if (! fs::is_regular_file(dir_iter->status()) ) continue;
      std::string fileName = (*dir_iter).path().string();
      int len = fileName.size();
      if (len >= 4 && fileName.substr(len - 4, 4) == ".tif"){
        //std::cout << "Now adding " << fileName << std::endl;
        tifsInDir.push_back( fileName );
      }
    }

  // Sort the files in lexicographic order
  std::sort(tifsInDir.begin(), tifsInDir.end());

  return;
}

Vector4 ComputeGeoBoundary(cartography::GeoReference Geo, int width, int height){

  // Get the lonlat coordinates of the four pixels corners of the image.

  Vector4 corners;
  Vector2 leftTopPixel(0,0);
  Vector2 leftTopLonLat = Geo.pixel_to_lonlat(leftTopPixel);

  Vector2 rightBottomPixel(width-1, height-1);
  Vector2 rightBottomLonLat = Geo.pixel_to_lonlat(rightBottomPixel);

  float minLon = leftTopLonLat(0);
  float minLat = leftTopLonLat(1);
  float maxLon = rightBottomLonLat(0);
  float maxLat = rightBottomLonLat(1);

  if (maxLat<minLat){
    float temp = minLat;
    minLat = maxLat;
    maxLat = temp;
  }

  if (maxLon<minLon){
    float temp = minLon;
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

  // Note: Below we assume that the image is uint8. In fact, for the
  // purpose of calculation of corners the type of the image being
  // read does not matter.
  DiskImageView<PixelMask<PixelGray<uint8> > >  image(imageFile);

  GeoReference imageGeo;
  bool is_good = read_georeference(imageGeo, imageFile);
  if (!is_good){
    std::cerr << "No georeference found in " << imageFile << std::endl;
    exit(1);
  }
  Vector4 imageCorners = ComputeGeoBoundary(imageGeo, image.cols(), image.rows());
  return imageCorners;
}

bool boxesOverlap(const Vector4 & box1Corners, const Vector4 & box2Corners){

  int lonOverlap = 0;
  int latOverlap = 0;

  if (box1Corners(0) > box1Corners(1) || box2Corners(0) > box2Corners(1))
    {
      std::cout << "ERROR: Must never happen: " << __FILE__ << " at line " << __LINE__ << std::endl;
      exit(1);
    }

  if ( std::max(box1Corners(0), box2Corners(0)) < std::min(box1Corners(1), box2Corners(1)) )
    {
      lonOverlap = 1;
    }

  if (box1Corners(2) > box1Corners(3) || box2Corners(2) > box2Corners(3))
    {
      std::cout << "ERROR: Must never happen: " << __FILE__ << " at line " << __LINE__ << std::endl;
      exit(1);
    }

  if ( std::max(box1Corners(2), box2Corners(2)) < std::min(box1Corners(3), box2Corners(3)) )
    {
      latOverlap = 1;
    }

  return (lonOverlap == 1 && latOverlap == 1);

}

void listTifsInDirOverlappingWithBox(const std::string & dirName,
                                                 Vector4 & boxCorners,
                                                 const std::string & outputListName){

  std::vector<std::string> tifsInDir;
  listTifsInDir(dirName, tifsInDir);

  ofstream fh(outputListName.c_str());
  if (!fh) {
    std::cerr << "ERROR: listTifsInDirOverlappingWithBox: can't open " << outputListName
              << " for writing" << std::endl;
    exit(1);
  }

  fh.precision(20);
  for (int fileIter = 0; fileIter < (int)tifsInDir.size(); fileIter++){
    const std::string & currFile = tifsInDir[fileIter];
    Vector4 currCorners = getImageCorners(currFile);
    if (!boxesOverlap(currCorners, boxCorners)) continue;
    fh << 1 << " " << currFile << " " << currCorners(0) << " " << currCorners(1) << " " <<
      currCorners(2) << " " << currCorners(3) << std::endl;
  }
  fh.close();

  return;
}

class DemMosaicView: public ImageViewBase<DemMosaicView>{
  int m_cols, m_rows;
  bool m_use_no_weights, m_stack_dems;
  vector<ModelParams> & m_modelParamsArray;
  vector< DiskImageView<DemPixelT> > const& m_images;
  vector<cartography::GeoReference> const& m_georefs; 
  cartography::GeoReference m_out_georef;
  vector<double> m_nodata_values;
  double m_out_nodata_value;

public:
  DemMosaicView(int cols, int rows, bool use_no_weights, bool stack_dems,
                vector<ModelParams> & modelParamsArray,
                vector< DiskImageView<DemPixelT> > const& images,
                vector<cartography::GeoReference> const& georefs,
                cartography::GeoReference const& out_georef,
                vector<double> const& nodata_values, double out_nodata_value):
    m_cols(cols), m_rows(rows),
    m_use_no_weights(use_no_weights), m_stack_dems(stack_dems),
    m_modelParamsArray(modelParamsArray),
    m_images(images), m_georefs(georefs),
    m_out_georef(out_georef), m_nodata_values(nodata_values),
    m_out_nodata_value(out_nodata_value){}
  
  typedef float pixel_type;
  typedef pixel_type result_type;
  typedef PixelMask<float> masked_pixel_type;
  typedef ProceduralPixelAccessor<DemMosaicView> pixel_accessor;
  
  inline int32 cols() const { return m_cols; }
  inline int32 rows() const { return m_rows; }
  inline int32 planes() const { return 1; }
  
  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "DemMosaicView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    ImageView<pixel_type> tile(bbox.width(), bbox.height());
    ImageView<float>      weights(bbox.width(), bbox.height());
    fill( tile, m_out_nodata_value );
    fill( weights, 0.0 );

    for (int dem_iter = 0; dem_iter < (int)m_modelParamsArray.size(); dem_iter++){

      cartography::GeoReference georef = m_georefs[dem_iter];
      ImageViewRef<DemPixelT> curr_disk_dem = m_images[dem_iter];
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
            
      // Read the whole chunk into memory, to speed things up
      ImageView<DemPixelT> curr_dem = crop(curr_disk_dem, curr_box);
#if USE_GRASSFIRE
      ImageViewRef<DemPixelT> interp_dem
        = interpolate(curr_dem, BilinearInterpolation(), ConstantEdgeExtension());
#else
      ImageViewRef< PixelMask<DemPixelT> > interp_dem
      = interpolate(create_mask(curr_dem, nodata_value),
                    BilinearInterpolation(), ConstantEdgeExtension());
#endif
      
      for (int c = 0; c < bbox.width(); c++){
        for (int r = 0; r < bbox.height(); r++){
          Vector2 out_pix(c +  bbox.min().x(), r +  bbox.min().y());
          Vector2 in_pix = georef.lonlat_to_pixel
            (m_out_georef.pixel_to_lonlat(out_pix));
          double x = in_pix[0] - curr_box.min().x();
          double y = in_pix[1] - curr_box.min().y();
          // below must use x <= cols()-1 as x is double
          if (x >= 0 && x <= interp_dem.cols()-1 &&
              y >= 0 && y <= interp_dem.rows()-1 ){
#if USE_GRASSFIRE
            // If we have weights of 0, that means there are invalid pixels,
            // so skip this point.
            int i = (int)floor(x), j = (int)floor(y);
            if (curr_dem(i,   j  ).a() <= 0 ||
                curr_dem(i+1, j  ).a() <= 0 ||
                curr_dem(i,   j+1).a() <= 0 ||
                curr_dem(i+1, j+1).a() <= 0
                )continue;
            
            DemPixelT pval = interp_dem(x, y);
            double val = pval.v();
            double wt = pval.a();
#else
            PixelMask<DemPixelT> pval = interp_dem(x, y);
            if (! is_valid(pval)) continue;
            double val = pval.child();
            double wt = ComputeLineWeightsHV(in_pix,
                                             m_modelParamsArray[dem_iter]);
#endif
            if (wt <= 0) continue;
            if (m_use_no_weights || m_stack_dems) wt = 1.0; // so weights are only 0 or 1.
            if ( tile(c, r) == m_out_nodata_value || isnan(tile(c, r)) )
              tile(c, r) = 0;
            if (!m_stack_dems){
              // Combine the values
              tile(c, r) += wt*val;
              weights(c, r) += wt;
            }else{
              // Use just the last value
              tile(c, r) = wt*val;
              weights(c, r) = wt;
            }
            
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

//     vw_out() << "Num valid pixels in " << bbox  << ' ' << num_valid_pixels
//              << std::endl;
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

struct Options : asp::BaseOptions {
  bool use_no_weights, stack_dems;
  string dem_list_file, out_prefix;
  double mpp, tr, out_nodata_value;
  int tile_size, tile_index;
  Options():out_nodata_value(std::numeric_limits<float>::quiet_NaN()){}
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
    ("output-nodata-value", po::value<double>(&opt.out_nodata_value),
     "No-data value to use on output. If not specified, use the one from the first DEM to be mosaicked.")
    ("use-no-weights", po::bool_switch(&opt.use_no_weights)->default_value(false),
     "Average the DEMs to mosaic without using weights (the result is not as smooth).")
    ("stack-dems", po::bool_switch(&opt.stack_dems)->default_value(false),
     "Stack each DEM on top of the previous one, without attempting to combine their values (the result is less smooth).")
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
  if (opt.use_no_weights && opt.stack_dems){
    vw_throw(ArgumentErr() << "At most one of --use-no-weights and --stack-dems must be set.\n"
              << usage << general_options );
  }

  // Create the output directory 
  asp::create_out_dir(opt.out_prefix);
  
  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

}

int main( int argc, char *argv[] ) {

  Options opt;
  try{
    
    handle_arguments( argc, argv, opt );

    // Read the DEMs to mosaic
    std::vector<std::string> dem_files;
    {
      ifstream is(opt.dem_list_file.c_str());
      std::cout << "list is " << opt.dem_list_file << std::endl;
      string file;
      while (is >> file) dem_files.push_back(file);
      if (dem_files.empty())
        vw_throw(ArgumentErr() << "No DEM files to mosaic.\n");
      is.close();
    }
    
    // Read nodata from first DEM, unless the user chooses to specify it.
    if (std::isnan(opt.out_nodata_value)){
      DiskImageResourceGDAL in_rsrc(dem_files[0]);
      if ( in_rsrc.has_nodata_read() ) opt.out_nodata_value = in_rsrc.nodata_read();
    }
    
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
    if (!is_good){
      std::cerr << "No georeference found in " << dem_files[0] << std::endl;
      exit(1);
    }
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
    std::cout << "Output georeference:\n" << out_georef << std::endl;
    
    // Image size
    Vector2 end_pix = out_georef.lonlat_to_pixel(Vector2(ll_bbox[1], ll_bbox[2]));
    int cols = (int)ceil(end_pix[0]);
    int rows = (int)ceil(end_pix[1]);

    // Compute the weights, and store the no-data values, pointers
    // to images, and georeferences (for speed).
    vector<ModelParams> modelParamsArray;
    vector<double> nodata_values;
    vector< DiskImageView<DemPixelT> > images;
    vector< cartography::GeoReference > georefs;
    for (int dem_iter = 0; dem_iter < (int)dem_files.size(); dem_iter++){
      modelParamsArray.push_back(ModelParams());
      modelParamsArray[dem_iter].inputFilename = dem_files[dem_iter];
      images.push_back(DiskImageView<DemPixelT>( dem_files[dem_iter] ));

#if USE_GRASSFIRE
#else
      ComputeDEMCenterLines(modelParamsArray[dem_iter]);
#endif
      cartography::GeoReference geo;
      bool is_good = read_georeference(geo, dem_files[dem_iter]);
      if (!is_good){
        std::cerr << "No georeference found in " << dem_files[dem_iter]
                  << std::endl;
        exit(1);
      }
      georefs.push_back(geo);
      
      double curr_nodata_value = opt.out_nodata_value;
      DiskImageResourceGDAL in_rsrc(dem_files[dem_iter]);
      if ( in_rsrc.has_nodata_read() ) curr_nodata_value = in_rsrc.nodata_read();
      nodata_values.push_back(curr_nodata_value);

#if USE_GRASSFIRE
      boost::scoped_ptr<vw::SrcImageResource> src(vw::DiskImageResource::open(dem_files[dem_iter]));
      int num_channels = src->channels();
      int num_planes   = src->planes();
      if (num_channels*num_planes != 2){
        std::cerr << "Need to have an alpha channel with the grassfire weights in "
                  << dem_files[dem_iter] << std::endl;
        exit(1);
      }
#endif

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

    // fix here!!!!
    DiskImageResourceGDAL::Options gdal_options;
    gdal_options["COMPRESS"] = "LZW";
    ImageViewRef<float> out_dem
      = crop(DemMosaicView(cols, rows, opt.use_no_weights, opt.stack_dems,
                           modelParamsArray, images, georefs,
                           out_georef, nodata_values, opt.out_nodata_value),
             tile_box);
    vw_out() << "Writing: " << dem_tile << std::endl;
    DiskImageResourceGDAL rsrc(dem_tile, out_dem.format(), Vector2i(256, 256),
                               gdal_options);
    if (!isnan(opt.out_nodata_value))
      rsrc.set_nodata_write(opt.out_nodata_value);
    cartography::GeoReference crop_georef
      = crop(out_georef, tile_box.min().x(), tile_box.min().y());
    
    write_georeference(rsrc, crop_georef);
    block_write_image(rsrc, out_dem, TerminalProgressCallback("{Core}",
                                                              "Processing:"));

  } ASP_STANDARD_CATCHES;
  
  return 0;
}

