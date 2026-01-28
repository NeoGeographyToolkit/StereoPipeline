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


/// \file stereo_fltr.cc
///
#include <asp/Sessions/StereoSession.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Macros.h>
#include <asp/Core/ThreadedEdgeMask.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Gotcha/CBatchProc.h>
#include <asp/Tools/stereo.h>

#include <vw/Stereo/DisparityMap.h>
#include <vw/Stereo/Algorithms.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Image/BlobIndex.h>
#include <vw/Image/ErodeView.h>
#include <vw/Image/InpaintView.h>
#include <vw/Cartography/GeoReferenceUtils.h>

#include <xercesc/util/PlatformUtils.hpp>

#include <boost/dll.hpp>

using namespace vw;
using namespace asp;


/// Apply a set of smoothing filters to the subpixel disparity results.
template <class ImageT, class DispImageT>
class TextureAwareDisparityFilter: public ImageViewBase<TextureAwareDisparityFilter<ImageT, DispImageT> >{
  ImageT     m_img;
  DispImageT m_disp_img;
  
  int   m_median_filter_size;     ///< Step 1: Apply a median filter of this size
  int   m_texture_smooth_range;   ///< Step 2: Compute texture measure of input image with this kernel size
  float m_texture_max;            ///< Step 3: Perform texture-aware smoothing of the disparity.  m_texture_max
  int   m_max_smooth_kernel_size; ///<         smooths more pixels, and the smooth_kernel_size increases the smoothing intensity.
  
public:
  TextureAwareDisparityFilter( ImageViewBase<ImageT    > const& img,
                               ImageViewBase<DispImageT> const& disp_img,
                               int   median_filter_size,
                               int   texture_smooth_range,
                               float texture_max,
                               int   max_smooth_kernel_size):
    m_img(img.impl()), m_disp_img(disp_img.impl()),
    m_median_filter_size(median_filter_size),
    m_texture_smooth_range(texture_smooth_range),
    m_texture_max(texture_max),
    m_max_smooth_kernel_size(max_smooth_kernel_size)
     {}

  // Image View interface
  typedef typename DispImageT::pixel_type pixel_type;
  typedef pixel_type                      result_type;
  typedef ProceduralPixelAccessor<TextureAwareDisparityFilter> pixel_accessor;

  inline int32 cols  () const { return m_disp_img.cols(); }
  inline int32 rows  () const { return m_disp_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "TextureAwareDisparityFilter::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // Figure out the largest kernel expansion we need to support the filtering
    int max_half_kernel = m_texture_smooth_range;
    if (m_max_smooth_kernel_size > max_half_kernel)
      max_half_kernel = m_max_smooth_kernel_size;
    max_half_kernel += m_median_filter_size; // Don't forget we apply two kernels in succession
    max_half_kernel /= 2;

    // Rasterize both input image regions
    BBox2i bbox2 = bbox;
    bbox2.expand(max_half_kernel);
    bbox2.crop(bounding_box(m_img)); // Restrict to valid input area
    ImageView<typename ImageT::pixel_type> input_tile      = crop(m_img,      bbox2);
    ImageView<pixel_type                 > input_disp_tile = crop(m_disp_img, bbox2);

    ImageView<float> texture_image;
    vw::stereo::texture_measure(input_tile, texture_image, m_texture_smooth_range);
    //write_image( "texture_image.tif", texture_image );


    ImageView<pixel_type > disp_tile_median;
    vw::stereo::disparity_median_filter(input_disp_tile, disp_tile_median, m_median_filter_size);
    
    ImageView<pixel_type > disp_tile_filtered;
    vw::stereo::texture_preserving_disparity_filter(disp_tile_median, disp_tile_filtered, texture_image, 
                                                    m_texture_max, m_max_smooth_kernel_size);

    // Fake the bounds on the returned image region
    return prerasterize_type(disp_tile_filtered,
                             -bbox2.min().x(), -bbox2.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class ImageT, class DispImageT>
TextureAwareDisparityFilter<ImageT, DispImageT>
texture_aware_disparity_filter( ImageViewBase<ImageT    > const& img,
                                ImageViewBase<DispImageT> const& disp_img,
                                int   median_filter_size,
                                int   texture_smooth_range,
                                float texture_max,
                                int   max_smooth_kernel_size) {
  typedef TextureAwareDisparityFilter<ImageT, DispImageT> return_type;
  return return_type(img.impl(), disp_img.impl(), median_filter_size, 
                     texture_smooth_range, texture_max, max_smooth_kernel_size);
}

// Erode blobs from given image by iterating through tiles, biasing
// each tile by a factor of blob size, removing blobs in the tile,
// then shrinking the tile back. The bias is necessary to help avoid
// fragmenting (and then unnecessarily removing) blobs.
template <class ImageT>
class PerTileErode: public ImageViewBase<PerTileErode<ImageT> >{
  ImageT m_img;
public:
  PerTileErode( ImageViewBase<ImageT>   const& img):
    m_img(img.impl()){}

  // Image View interface
  typedef typename ImageT::pixel_type pixel_type;
  typedef pixel_type                  result_type;
  typedef ProceduralPixelAccessor<PerTileErode> pixel_accessor;

  inline int32 cols  () const { return m_img.cols(); }
  inline int32 rows  () const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "PerTileErode::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    int area = stereo_settings().erode_max_size;

    // We look a beyond the current tile, to avoid cutting blobs
    // if possible. Skinny blobs will be cut though.
    int bias = 2*int(std::ceil(sqrt(double(area))));

    BBox2i bbox2 = bbox;
    bbox2.expand(bias);
    bbox2.crop(bounding_box(m_img));
    ImageView<pixel_type> tile_img = crop(m_img, bbox2);

    int tile_size = std::max(bbox2.width(), bbox2.height()); // don't subsplit
    BlobIndexThreaded smallBlobIndex(tile_img, area, tile_size);
    ImageView<pixel_type> clean_tile_img = applyErodeView(tile_img,
                                                          smallBlobIndex);
    return prerasterize_type(clean_tile_img,
                             -bbox2.min().x(), -bbox2.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class ImageT>
PerTileErode<ImageT>
per_tile_erode( ImageViewBase<ImageT> const& img) {
  typedef PerTileErode<ImageT> return_type;
  return return_type( img.impl() );
}

// Run several cleanup passes with desired cleanup mode.
template <class ViewT>
struct MultipleDisparityCleanUp {
  typedef ImageViewRef< typename ViewT::pixel_type > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input, int N) {

    result_type out = input;
    for (int i = 0; i < N; i++){
      int mode = stereo_settings().filter_mode;
      if (mode == 1){
        out = stereo::disparity_cleanup_using_mean
          (out.impl(),
           stereo_settings().rm_half_kernel.x(),
           stereo_settings().rm_half_kernel.y(),
           stereo_settings().max_mean_diff);
      }else if (mode == 2){
        out = stereo::disparity_cleanup_using_thresh
          (out.impl(),
           stereo_settings().rm_half_kernel.x(),
           stereo_settings().rm_half_kernel.y(),
           stereo_settings().rm_threshold,
           stereo_settings().rm_min_matches/100.0);
      }else
        vw_throw( ArgumentErr() << "\nExpecting value of 1 or 2 for filter-mode. "
                  << "Got: " << mode << "\n" );
    }

    return out;
  }
};

template <class ImageT>
void write_good_pixel_and_filtered(ImageViewBase<ImageT> const& inputview,
                                   ASPGlobalOptions const& opt) {
  // Write Good Pixel Map
  // Sub-sampling so that the user can actually view it.
  double sub_scale = double( std::min( inputview.impl().cols(),
                                inputview.impl().rows() ) ) / 2048.0;
  if (sub_scale < 1) // Don't use a sub_scale less than one.
    sub_scale = 1;

  // Write out the good pixel map
  std::string goodPixelFile = opt.out_prefix + "-GoodPixelMap.tif";
  vw_out() << "Writing: " << goodPixelFile << std::endl;
  ImageViewRef<  PixelRGB<uint8> > goodPixelImage
    = subsample(apply_mask
                (copy_mask
                 (stereo::missing_pixel_image(inputview.impl()),
                  create_mask(DiskImageView<vw::uint8>(opt.out_prefix+"-lMask.tif"), 0)
                  )
                 ), sub_scale);

  // Determine if we can attach geo information to the output image
  cartography::GeoReference left_georef;
  bool has_left_georef = read_georeference(left_georef,  opt.out_prefix + "-L.tif");
  bool has_nodata = false;
  double nodata = -32768.0;

  vw::cartography::GeoReference good_pixel_georef;
  if (has_left_georef) {
    // Account for scale. Note that goodPixelImage is not guaranteed to respect
    // the sub_scale factor above, hence this calculation.
    double good_pixel_scale = 0.5*( double(goodPixelImage.cols())/inputview.impl().cols()
                                    + double(goodPixelImage.rows())/inputview.impl().rows());
    good_pixel_georef = resample(left_georef, good_pixel_scale);
  }

  vw::cartography::block_write_gdal_image
    ( goodPixelFile, goodPixelImage, has_left_georef, good_pixel_georef,
      has_nodata, nodata,
      opt, TerminalProgressCallback("asp", "\t--> Good pixel map: ") );

  bool removeSmallBlobs = (stereo_settings().erode_max_size > 0);

  std::string outF = opt.out_prefix + "-F.tif";

  // Fill holes
  if(stereo_settings().enable_fill_holes) {
    // Generate a list of blobs below a maximum size
    // - This requires the entire input image to be read in
    //    and produces a single blob list for the entire image.
    vw_out() << "\t--> Filling holes with inpainting method.\n";
    BlobIndexThreaded smallHoleIndex( invert_mask( inputview.impl() ),
                                      stereo_settings().fill_hole_max_size,
                                      vw::vw_settings().default_tile_size(),
                                      vw::vw_settings().default_num_threads()
                                      );
    vw_out() << "\t    * Identified " << smallHoleIndex.num_blobs() << " holes\n";
    bool use_grassfire = true;
    typename ImageT::pixel_type default_inpaint_val;


    if (!removeSmallBlobs) { // Skip small blob removal
      // Write out the image to disk, filling in the blobs in the process
      vw_out() << "Writing: " << outF << std::endl;
      vw::cartography::block_write_gdal_image( outF,
                                   inpaint(inputview.impl(), smallHoleIndex,
                                           use_grassfire, default_inpaint_val),
                                   has_left_georef, left_georef,
                                   has_nodata, nodata, opt,
                                   TerminalProgressCallback
                                   ("asp","\t--> Filtering: ") );
    }
    else { // Add small blob removal step
      // Write out the image to disk, filling in and removing blobs in the process
      // - Blob removal is done second to make sure inner-blob holes are removed.
      vw_out() << "Writing: " << outF << std::endl;
      vw::cartography::block_write_gdal_image( outF,
                                   per_tile_erode
                                   (inpaint(inputview.impl(),
                                            smallHoleIndex,
                                            use_grassfire,
                                            default_inpaint_val) ),
                                   has_left_georef, left_georef,
                                   has_nodata, nodata, opt,
                                   TerminalProgressCallback
                                   ("asp","\t--> Filtering: ") );
    }

  } else { // No hole filling
    if (!removeSmallBlobs) { // Skip small blob removal
      vw_out() << "Writing: " << outF << std::endl;
      vw::cartography::block_write_gdal_image( outF, inputview.impl(),
                                   has_left_georef, left_georef,
                                   has_nodata, nodata, opt,
                                   TerminalProgressCallback
                                   ("asp", "\t--> Filtering: ") );
    }
    else { // Add small blob removal step
      vw_out() << "\t--> Removing small blobs.\n";
      // Write out the image to disk, removing the blobs in the process
      vw_out() << "Writing: " << outF << std::endl;
      vw::cartography::block_write_gdal_image(outF, per_tile_erode(inputview.impl()),
                                  has_left_georef, left_georef,
                                  has_nodata, nodata, opt,
                                  TerminalProgressCallback
                                  ("asp","\t--> Filtering: ") );
    }

  } // End no hole filling case
} //end write_good_pixel_and_filtered

void stereo_filtering(ASPGlobalOptions& opt) {

  std::string post_correlation_fname = opt.out_prefix+"-RD.tif";

  try {

    // Rasterize the results so far to a temporary file on disk.
    // This file is deleted once we complete the second half of the
    // disparity map filtering process.

    // Apply filtering for high frequencies
    typedef DiskImageView<PixelMask<Vector2f> > input_type;
    input_type disparity_disk_image(post_correlation_fname);

    // Applying additional clipping from the edge. We make new
    // mask files to avoid a weird and tricky segfault due to ownership issues.
    DiskImageView<vw::uint8> left_mask ( opt.out_prefix+"-lMask.tif" );
    DiskImageView<vw::uint8> right_mask( opt.out_prefix+"-rMask.tif" );
    
    // Erode at the edge. If not set, apply some logic as below.
    int32 edge_buffer = stereo_settings().edge_buffer_size;
    if (edge_buffer < 0) {
      if (stereo_settings().subpixel_mode > 0 && stereo_settings().subpixel_mode <= 6)
        edge_buffer = max(stereo_settings().subpixel_kernel); // max of the two entries
      else 
        edge_buffer = 5; // should be enough
    }  

    DiskImageView<PixelGray<float>> left_disk_image (opt.out_prefix+"-L.tif");

    vw_out() << "\t--> Cleaning up disparity map prior to filtering processes ("
             << stereo_settings().rm_cleanup_passes << " pass).\n";

    // If the user wants to do no filtering at all, that amounts
    // to doing no passes.
    if (stereo_settings().filter_mode == 0)
      stereo_settings().rm_cleanup_passes = 0;

    // No Erosion step
    if ( stereo_settings().rm_cleanup_passes >= 1 ) {
      // Apply an outlier removal filter
      write_good_pixel_and_filtered
        (stereo::disparity_mask
          (MultipleDisparityCleanUp<input_type>()
            (disparity_disk_image, stereo_settings().rm_cleanup_passes),
              apply_mask(asp::threaded_edge_mask(left_mask, 0,edge_buffer,1024)),
              apply_mask(asp::threaded_edge_mask(right_mask,0,edge_buffer,1024))),
            opt);
    } else { // No cleanup passes
      write_good_pixel_and_filtered
        (stereo::disparity_mask
          (
            texture_aware_disparity_filter(left_disk_image, disparity_disk_image, 
                                          stereo_settings().median_filter_size,
                                          stereo_settings().disp_smooth_size+2, // Compute texture a little larger than smooth radius
                                          stereo_settings().disp_smooth_texture, 
                                          stereo_settings().disp_smooth_size),
            apply_mask(asp::threaded_edge_mask(left_mask, 0,edge_buffer,1024)),
            apply_mask(asp::threaded_edge_mask(right_mask,0,edge_buffer,1024))),
          opt);
    } // End cleanup passes check

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at filtering stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
} // end stereo_filtering()

void gotcha_disparity_refinement(ASPGlobalOptions& opt) {

  // Apply Gotcha on tiles of size 1024
  opt.raster_tile_size = Vector2i(ASPGlobalOptions::corr_tile_size(),
                                  ASPGlobalOptions::corr_tile_size());

  // Use this much padding to ensure that the tiles processed by Gotcha overlap,
  // to avoid help avoid seams.
  int padding = ASPGlobalOptions::corr_tile_size()/4;
  
  if (stereo_settings().casp_go_param_file == "") {
    // Use the default CASP-GO parameter file if not provided
    std::string base_dir = boost::dll::program_location().parent_path().parent_path().string();
    stereo_settings().casp_go_param_file = base_dir + "/share/CASP-GO_params.xml";
  }

  if (!fs::exists(stereo_settings().casp_go_param_file)) 
    vw_throw(ArgumentErr() << "Cannot read CASP-GO parameter file: "
             << stereo_settings().casp_go_param_file << "\n");
  else
    vw_out() << "Refining the disparity using the Gotcha algorithm and parameter file: "
             << stereo_settings().casp_go_param_file << "\n";

  // First move the current F file out of the way, as it is not possible to overwrite
  // it in place.
  std::string disp_file = opt.out_prefix + "-F.tif";
  std::string disp_file_nogotcha = opt.out_prefix + "-F-nogotcha.tif";
  std::string cmd = "mv " + disp_file + " " + disp_file_nogotcha;
  vw_out() << cmd << "\n";
  system(cmd.c_str());
  
  ImageViewRef<PixelMask<Vector2f>> filtered_disparity
    = opt.session->pre_pointcloud_hook(disp_file_nogotcha);

  // TODO(oalexan1): How about no-data pixels in the left and right images?
  std::string L_file = opt.out_prefix + "-L.tif";
  std::string R_file = opt.out_prefix + "-R.tif";
  DiskImageView<float> left_image (L_file);
  DiskImageView<float> right_image(R_file);
  
  // Determine if we can attach geo information to the output disparity
  cartography::GeoReference left_georef;
  bool has_left_georef = read_georeference(left_georef, L_file);
  bool has_nodata = false;
  double nodata = -32768.0;
  vw_out() << "Writing Gotcha-refined disparity: " << disp_file << std::endl;
  block_write_gdal_image(disp_file,
                         gotcha::gotcha_refine(filtered_disparity,  
                                               left_image, right_image,
                                               padding, stereo_settings().casp_go_param_file),
                         has_left_georef, left_georef,
                         has_nodata, nodata, opt,
                         TerminalProgressCallback("asp","\t  Gotcha:  "));
}

int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();

    vw_out() << "\n[ " << current_posix_time_string()
             << " ]: Stage 4 --> FILTERING\n";

    stereo_register_sessions();

    bool verbose = false;
    std::vector<ASPGlobalOptions> opt_vec;
    std::string output_prefix;
    asp::parseStereoArgs(argc, argv, FilteringDescription(),
                         verbose, output_prefix, opt_vec);
    ASPGlobalOptions opt = opt_vec[0];

    // Internal Processes
    //---------------------------------------------------------
    stereo_filtering(opt);

    if (stereo_settings().gotcha_disparity_refinement)
      gotcha_disparity_refinement(opt);
    
    vw_out() << "\n[ " << current_posix_time_string()
             << " ]: FILTERING FINISHED\n";

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
