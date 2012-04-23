// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <vw/Stereo/DisparityMap.h>

#include <asp/Core/BlobIndexThreaded.h>
#include <asp/Core/InpaintView.h>
#include <asp/Core/ErodeView.h>
#include <asp/Core/ThreadedEdgeMask.h>

using namespace vw;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

// This uses a struct as partial template specialization is not
// allowed for functions in C++.
template <class ViewT, int N>
struct MultipleDisparityCleanUp {

  MultipleDisparityCleanUp<ViewT,N-1> inner_func;

  typedef typename MultipleDisparityCleanUp<ViewT,N-1>::result_type inner_type;
  typedef UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<inner_type,ConstantEdgeExtension>, stereo::RemoveOutliersFunc<typename inner_type::pixel_type> >, stereo::RemoveOutliersFunc<typename inner_type::pixel_type> > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input,
                                 int const& h_half_kern,
                                 int const& v_half_kern,
                                 double const& pixel_thres,
                                 double const& rej_thres )  {
    return stereo::disparity_clean_up(
                                      inner_func(input.impl(), h_half_kern,
                                                 v_half_kern, pixel_thres,
                                                 rej_thres), h_half_kern,
                                      v_half_kern, pixel_thres, rej_thres);
  }
};

template <class ViewT>
struct MultipleDisparityCleanUp<ViewT,1> {

  typedef UnaryPerPixelAccessorView< UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ConstantEdgeExtension>, stereo::RemoveOutliersFunc<typename ViewT::pixel_type> >, stereo::RemoveOutliersFunc<typename ViewT::pixel_type> > result_type;

  inline result_type operator()( ImageViewBase<ViewT> const& input,
                                 int const& h_half_kern,
                                 int const& v_half_kern,
                                 double const& pixel_thres,
                                 double const& rej_thres ) {
    return stereo::disparity_clean_up(
                                      input.impl(), h_half_kern,
                                      v_half_kern, pixel_thres, rej_thres);
  }
};

template <class ImageT>
void write_resample_with_reduce_memory( ImageViewBase<ImageT> const& inputview,
                                        double sub_scale,
                                        std::string const& suffix,
                                        Options const& opt,
                                        uint32 copies = 1 ) {
  ImageT const& input = inputview.impl();

  if ( sub_scale > 1 ) sub_scale = 1;
  // Solving for the number of threads and the tile size to use for
  // subsampling while only using 500 MiB of memory. (The cache code
  // is a little slow on releasing so it will probably use 1.5GiB
  // memory during subsampling) Also tile size must be a power of 2
  // and greater than or equal to 64 px;
  uint32 previous_num_threads = vw_settings().default_num_threads();
  uint32 sub_threads = previous_num_threads + 1;
  uint32 tile_power = 0;
  const uint32 pixel_bytes = PixelNumBytes<typename ImageT::pixel_type>::value;
  const double scaling = ceil(1.0/sub_scale);
  while ( tile_power < 6 && sub_threads > 1) {
    sub_threads--;
    tile_power =
      boost::numeric_cast<uint32>( floor( ( log(512.0e6) -
                                            log(double(copies*sub_threads*pixel_bytes) *
                                                scaling*scaling) ) / log(4) ) );
  }
  uint32 sub_tile_size = 1 << tile_power;

  VW_OUT(DebugMessage, "asp") << "\t--> Creating previews. Subsampling by " << sub_scale
                              << " by using " << sub_tile_size << " tile size and "
                              << sub_threads << " threads.\n";

  if ( sub_tile_size > vw_settings().default_tile_size() )
    sub_tile_size = vw_settings().default_tile_size();

  TransformView<InterpolationView<EdgeExtensionView<ImageT, ConstantEdgeExtension>, BilinearInterpolation>, ResampleTransform> reduced =
    resample(input, sub_scale);
  vw_settings().set_default_num_threads(sub_threads);
  DiskImageResourceGDAL rsrc( opt.out_prefix + suffix + ".tif",
                              reduced.format(),
                              Vector2i(sub_tile_size,
                                       sub_tile_size ),
                              opt.gdal_options );
  block_write_image( rsrc, reduced,
                     TerminalProgressCallback("asp", "\t    Writing: "));
  vw_settings().set_default_num_threads(previous_num_threads);
}

template <class ImageT>
void write_good_pixel_and_filtered( ImageViewBase<ImageT> const& inputview,
                                    Options const& opt ) {
  { // Write Good Pixel Map
    vw_out() << "\t--> Creating \"Good Pixel\" image: "
             << (opt.out_prefix + "-GoodPixelMap.tif") << "\n";

    // Sub-sampling so that the user can actually view it.
    float sub_scale =
      2048.0 / float( std::min( inputview.impl().cols(),
                                inputview.impl().rows() ) );
    write_resample_with_reduce_memory(
      apply_mask(copy_mask(stereo::missing_pixel_image(inputview.impl()),
                           create_mask(DiskImageView<vw::uint8>(opt.out_prefix+"-lMask.tif"),0))),
      sub_scale, "-GoodPixelMap", opt, 4 /* it thinks it's working with uint8, but the input is float */ );
  }

  // Fill Holes
  if(stereo_settings().fill_holes) {
    vw_out() << "\t--> Filling holes with Inpainting method.\n";
    BlobIndexThreaded bindex( invert_mask( inputview.impl() ),
                              stereo_settings().fill_hole_max_size );
    vw_out() << "\t    * Identified " << bindex.num_blobs() << " holes\n";
    asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                 asp::InpaintView<ImageT >(inputview.impl(), bindex ),
                                 opt, TerminalProgressCallback("asp","\t--> Filtering: ") );

  } else {
    asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                 inputview.impl(), opt,
                                 TerminalProgressCallback("asp", "\t--> Filtering: ") );
  }
}

void stereo_filtering( Options& opt ) {
  vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 3 --> FILTERING \n";

  std::string post_correlation_fname;
  opt.session->pre_filtering_hook(opt.out_prefix+"-RD.tif",
                                  post_correlation_fname);

  try {

    // Rasterize the results so far to a temporary file on disk.
    // This file is deleted once we complete the second half of the
    // disparity map filtering process.
    {
      // Apply filtering for high frequencies
      typedef DiskImageView<PixelMask<Vector2f> > input_type;
      input_type disparity_disk_image(post_correlation_fname);

      // Applying additional clipping from the edge. We make new
      // mask files to avoid a weird and tricky segfault due to
      // ownership issues.
      DiskImageView<vw::uint8> left_mask( opt.out_prefix+"-lMask.tif" );
      DiskImageView<vw::uint8> right_mask( opt.out_prefix+"-rMask.tif" );
      int32 mask_buffer = max( stereo_settings().subpixel_kernel );

      vw_out() << "\t--> Cleaning up disparity map prior to filtering processes ("
               << stereo_settings().rm_cleanup_passes << " pass).\n";
      if ( stereo_settings().mask_flatfield ) {
        ImageViewRef<PixelMask<Vector2f> > filtered_disparity;
        if ( stereo_settings().rm_cleanup_passes == 1 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,1>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
        else if ( stereo_settings().rm_cleanup_passes == 2 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,2>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
        else if ( stereo_settings().rm_cleanup_passes == 3 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,3>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
        else if ( stereo_settings().rm_cleanup_passes >= 4 )
          filtered_disparity =
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,4>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
        else
          filtered_disparity =
            stereo::disparity_mask(disparity_disk_image,
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));

        // This is only turned on for apollo. Blob detection doesn't
        // work to great when tracking a whole lot of spots. HiRISE
        // seems to keep breaking this so I've keep it turned off.
        //
        // The crash happens inside Boost Graph when dealing with
        // large number of blobs.
        BlobIndexThreaded bindex( filtered_disparity,
                                  stereo_settings().erode_max_size );
        vw_out() << "\t    * Eroding " << bindex.num_blobs() << " islands\n";
        write_good_pixel_and_filtered( ErodeView<ImageViewRef<PixelMask<Vector2f> > >(filtered_disparity, bindex ),
                                       opt );
      } else {
        // No Erosion step
        if ( stereo_settings().rm_cleanup_passes == 1 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,1>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else if ( stereo_settings().rm_cleanup_passes == 2 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,2>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else if ( stereo_settings().rm_cleanup_passes == 3 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,3>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else if ( stereo_settings().rm_cleanup_passes >= 4 )
          write_good_pixel_and_filtered(
            stereo::disparity_mask(MultipleDisparityCleanUp<input_type,4>()(
                                                                            disparity_disk_image,stereo_settings().rm_h_half_kern,
                                                                            stereo_settings().rm_v_half_kern,
                                                                            stereo_settings().rm_threshold,
                                                                            stereo_settings().rm_min_matches/100.0),
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt);
        else
          write_good_pixel_and_filtered(
            stereo::disparity_mask(disparity_disk_image,
                                   apply_mask(asp::threaded_edge_mask(left_mask,0,mask_buffer,1024)),
                                   apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))), opt );
      }
    }
  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at filtering stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
}

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // user safety check
    //---------------------------------------------------------
    try {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1,camera_model2);

      // Do the camera's appear to be in the same location?
      if ( norm_2(camera_model1->camera_center(Vector2()) -
                  camera_model2->camera_center(Vector2())) < 1e-3 )
        vw_out(WarningMessage,"console")
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should be double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";
    } catch ( camera::PixelToRayErr const& e ) {
    } catch ( camera::PointToPixelErr const& e ) {
      // Silent. Top Left pixel might not be valid on a map
      // projected image.
    }

    // Internal Processes
    //---------------------------------------------------------
    stereo_filtering( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : FILTERING FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
