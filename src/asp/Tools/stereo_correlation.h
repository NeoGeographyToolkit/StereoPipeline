// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo_correlation.h

#ifndef __ASP_STEREO_CORRELATION_H__
#define __ASP_STEREO_CORRELATION_H__

#include <asp/Tools/stereo.h>

namespace vw {

  // Correlator View
  template <class FilterT>
    inline stereo::CorrelatorView<PixelGray<float>,vw::uint8,FilterT>
    correlator_helper( DiskImageView<PixelGray<float> > & left_disk_image,
                       DiskImageView<PixelGray<float> > & right_disk_image,
                       DiskImageView<vw::uint8> & left_mask,
                       DiskImageView<vw::uint8> & right_mask,
                       FilterT const& filter_func, vw::BBox2i & search_range,
                       stereo::CorrelatorType const& cost_mode,
                       bool draft_mode,
                       std::string & corr_debug_prefix,
                       bool use_pyramid=true ) {
    stereo::CorrelatorView<PixelGray<float>,
      vw::uint8,FilterT> corr_view( left_disk_image, right_disk_image,
                                    left_mask, right_mask, filter_func,
                                    use_pyramid );

    corr_view.set_search_range(search_range);
    corr_view.set_kernel_size(Vector2i(stereo_settings().h_kern,
                                       stereo_settings().v_kern));
    corr_view.set_cross_corr_threshold(stereo_settings().xcorr_threshold);
    corr_view.set_corr_score_threshold(stereo_settings().corrscore_rejection_threshold);
    corr_view.set_correlator_options(stereo_settings().cost_blur, cost_mode);

    if (draft_mode)
      corr_view.set_debug_mode(corr_debug_prefix);

    vw_out() << corr_view;
    vw_out() << "\t--> Building Disparity map." << std::endl;

    return corr_view;
  }

  void stereo_correlation( Options& opt ) {

    if (opt.entry_point == CORRELATION)
      vw_out() << "\nStarting at the CORRELATION stage.\n";
    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 1 --> CORRELATION \n";

    DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
      Rmask(opt.out_prefix + "-rMask.tif");

    std::string filename_L = opt.out_prefix+"-L.tif",
      filename_R = opt.out_prefix+"-R.tif";

    /*
      if (MEDIAN_FILTER==1){
      filename_L = out_prefix+"-median-L.tif";
      filename_R = out_prefix+"-median-R.tif";
      } else { *.
      filename_L = out_prefix+"-L.tif";
      filename_R = out_prefix+"-R.tif";
      }*/

    DiskImageView<PixelGray<float> > left_disk_image(filename_L),
      right_disk_image(filename_R);

#if defined(VW_HAS_BIGTIFF) && VW_HAS_BIGTIFF == 1
    // Determining if BigTiff is required
    if ( left_disk_image.cols()*left_disk_image.rows() > 10e6 )
      opt.gdal_options["BIGTIFF"] = "IF_SAFER";
#endif

    ImageViewRef<PixelMask<Vector2f> > disparity_map;
    stereo::CorrelatorType cost_mode = stereo::ABS_DIFF_CORRELATOR;
    if (stereo_settings().cost_mode == 1)
      cost_mode = stereo::SQR_DIFF_CORRELATOR;
    else if (stereo_settings().cost_mode == 2)
      cost_mode = stereo::NORM_XCORR_CORRELATOR;

    if (opt.optimized_correlator) {
      if (stereo_settings().pre_filter_mode == 3) {
        vw_out() << "\t--> Using LOG pre-processing filter with width: "
                 << stereo_settings().slogW << std::endl;
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW),
                             opt.search_range, cost_mode, false,
                             opt.corr_debug_prefix, false );
      } else if (stereo_settings().pre_filter_mode == 2) {
        vw_out() << "\t--> Using LOG pre-processing filter with width: "
                 << stereo_settings().slogW << std::endl;
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::LogStereoPreprocessingFilter(stereo_settings().slogW),
                             opt.search_range, cost_mode, false,
                             opt.corr_debug_prefix, false );
      } else if (stereo_settings().pre_filter_mode == 1) {
        vw_out() << "\t--> Using BLUR pre-processing filter with width: "
                 << stereo_settings().slogW << std::endl;
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::BlurStereoPreprocessingFilter(stereo_settings().slogW),
                             opt.search_range, cost_mode, false,
                             opt.corr_debug_prefix, false );
      } else {
        vw_out() << "\t--> Using NO pre-processing filter: " << std::endl;
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::NullStereoPreprocessingFilter(),
                             opt.search_range, cost_mode, false,
                             opt.corr_debug_prefix, false );
      }
    } else {
      if (stereo_settings().pre_filter_mode == 3) {
        vw_out() << "\t--> Using SLOG pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW),
                             opt.search_range, cost_mode, opt.draft_mode,
                             opt.corr_debug_prefix );
      } else if ( stereo_settings().pre_filter_mode == 2 ) {
        vw_out() << "\t--> Using LOG pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::LogStereoPreprocessingFilter(stereo_settings().slogW),
                             opt.search_range, cost_mode, opt.draft_mode,
                             opt.corr_debug_prefix );
      } else if ( stereo_settings().pre_filter_mode == 1 ) {
        vw_out() << "\t--> Using BLUR pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::BlurStereoPreprocessingFilter(stereo_settings().slogW),
                             opt.search_range, cost_mode, opt.draft_mode,
                             opt.corr_debug_prefix );
      } else {
        vw_out() << "\t--> Using NO pre-processing filter." << std::endl;
        disparity_map =
          correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                             stereo::NullStereoPreprocessingFilter(),
                             opt.search_range, cost_mode, opt.draft_mode,
                             opt.corr_debug_prefix );
      }
    }

    // Create a disk image resource and prepare to write a tiled
    DiskImageResourceGDAL disparity_map_rsrc(opt.out_prefix + "-D.tif",
                                             disparity_map.format(),
                                             opt.raster_tile_size,
                                             opt.gdal_options );
    block_write_image( disparity_map_rsrc, disparity_map );
  }

} //end namespace vw

#endif//__ASP_STEREO_CORRELATION_H__
