// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>

using namespace vw;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

void stereo_refinement( Options& opt ) {

  vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 2 --> REFINEMENT \n";

  try {
    std::string filename_L = opt.out_prefix+"-L.tif",
      filename_R  = opt.out_prefix+"-R.tif";
    /*
      if (MEDIAN_FILTER == 1){
      filename_L = out_prefix+"-median-L.tif";
      filename_R = out_prefix+"-median-R.tif";
      } else {
      filename_L = out_prefix+"-L.tif";
      filename_R = out_prefix+"-R.tif";
      }
    */
    typedef DiskImageView<PixelGray<float> > InnerView;
    InnerView left_disk_image(filename_L), right_disk_image(filename_R);
    DiskImageView<PixelMask<Vector2f> > disparity_disk_image(opt.out_prefix + "-D.tif");
    ImageViewRef<PixelMask<Vector2f> > disparity_map = disparity_disk_image;

    if (stereo_settings().subpixel_mode == 0) {
      // Do nothing
    } else if (stereo_settings().subpixel_mode == 1) {
      // Parabola
      vw_out() << "\t--> Using parabola subpixel mode.\n";
      if (stereo_settings().pre_filter_mode == 3) {
        vw_out() << "\t    SLOG preprocessing width: "
                 << stereo_settings().slogW << "\n";
        typedef stereo::SlogStereoPreprocessingFilter PreFilter;
        disparity_map =
          stereo::subpixel_refine( disparity_disk_image,
                                   left_disk_image, right_disk_image,
                                   stereo_settings().subpixel_h_kern,
                                   stereo_settings().subpixel_v_kern,
                                   stereo_settings().do_h_subpixel,
                                   stereo_settings().do_v_subpixel,
                                   stereo_settings().subpixel_mode,
                                   PreFilter(stereo_settings().slogW) );
      } else if (stereo_settings().pre_filter_mode == 2) {
        vw_out() << "\t    LOG preprocessing width: "
                 << stereo_settings().slogW << "\n";
        typedef stereo::LogStereoPreprocessingFilter PreFilter;
        disparity_map =
          stereo::subpixel_refine(disparity_disk_image,
                                  left_disk_image, right_disk_image,
                                  stereo_settings().subpixel_h_kern,
                                  stereo_settings().subpixel_v_kern,
                                  stereo_settings().do_h_subpixel,
                                  stereo_settings().do_v_subpixel,
                                  stereo_settings().subpixel_mode,
                                  PreFilter(stereo_settings().slogW) );
      } else if (stereo_settings().pre_filter_mode == 1) {
        vw_out() << "\t    BLUR preprocessing width: "
                 << stereo_settings().slogW << "\n";
        typedef stereo::BlurStereoPreprocessingFilter PreFilter;
        disparity_map =
          stereo::subpixel_refine(disparity_disk_image,
                                  left_disk_image, right_disk_image,
                                  stereo_settings().subpixel_h_kern,
                                  stereo_settings().subpixel_v_kern,
                                  stereo_settings().do_h_subpixel,
                                  stereo_settings().do_v_subpixel,
                                  stereo_settings().subpixel_mode,
                                  PreFilter(stereo_settings().slogW) );
      } else {
        vw_out() << "\t    NO preprocessing" << std::endl;
        typedef stereo::NullStereoPreprocessingFilter PreFilter;
        disparity_map =
          stereo::subpixel_refine(disparity_disk_image,
                                  left_disk_image, right_disk_image,
                                  stereo_settings().subpixel_h_kern,
                                  stereo_settings().subpixel_v_kern,
                                  stereo_settings().do_h_subpixel,
                                  stereo_settings().do_v_subpixel,
                                  stereo_settings().subpixel_mode,
                                  PreFilter() );
      }
    } else if (stereo_settings().subpixel_mode == 2) {
      // Bayes EM
      vw_out() << "\t--> Using affine adaptive subpixel mode "
               << stereo_settings().subpixel_mode << "\n";
      vw_out() << "\t--> Forcing use of LOG filter with "
               << stereo_settings().slogW << " sigma blur.\n";
      typedef stereo::LogStereoPreprocessingFilter PreFilter;
      disparity_map =
        stereo::subpixel_refine(disparity_disk_image,
                                left_disk_image, right_disk_image,
                                stereo_settings().subpixel_h_kern,
                                stereo_settings().subpixel_v_kern,
                                stereo_settings().do_h_subpixel,
                                stereo_settings().do_v_subpixel,
                                stereo_settings().subpixel_mode,
                                PreFilter(stereo_settings().slogW) );
    } else if (stereo_settings().subpixel_mode == 3) {
      // Affine and Bayes subpixel refinement always use the
      // LogPreprocessingFilter...
      vw_out() << "\t--> Using EM Subpixel mode "
               << stereo_settings().subpixel_mode << std::endl;
      vw_out() << "\t--> Mode 3 does internal preprocessing;"
               << " settings will be ignored. " << std::endl;

      typedef stereo::EMSubpixelCorrelatorView<float32> EMCorrelator;
      EMCorrelator em_correlator(channels_to_planes(left_disk_image),
                                 channels_to_planes(right_disk_image),
                                 disparity_disk_image, -1);
      em_correlator.set_em_iter_max(stereo_settings().subpixel_em_iter);
      em_correlator.set_inner_iter_max(stereo_settings().subpixel_affine_iter);
      em_correlator.set_kernel_size(Vector2i(stereo_settings().subpixel_h_kern,
                                             stereo_settings().subpixel_v_kern));
      em_correlator.set_pyramid_levels(stereo_settings().subpixel_pyramid_levels);

      DiskImageResourceOpenEXR em_disparity_map_rsrc(opt.out_prefix + "-F6.exr", em_correlator.format());

      block_write_image(em_disparity_map_rsrc, em_correlator,
                        TerminalProgressCallback("asp", "\t--> EM Refinement :"));

      DiskImageResource *em_disparity_map_rsrc_2 =
        DiskImageResourceOpenEXR::construct_open(opt.out_prefix + "-F6.exr");
      DiskImageView<PixelMask<Vector<float, 5> > > em_disparity_disk_image(em_disparity_map_rsrc_2);

      ImageViewRef<Vector<float, 3> > disparity_uncertainty =
        per_pixel_filter(em_disparity_disk_image,
                         EMCorrelator::ExtractUncertaintyFunctor());
      ImageViewRef<float> spectral_uncertainty =
        per_pixel_filter(disparity_uncertainty,
                         EMCorrelator::SpectralRadiusUncertaintyFunctor());
      write_image(opt.out_prefix+"-US.tif", spectral_uncertainty);
      write_image(opt.out_prefix+"-U.tif", disparity_uncertainty);

      disparity_map =
        per_pixel_filter(em_disparity_disk_image,
                         EMCorrelator::ExtractDisparityFunctor());
    } else {
      vw_out() << "\t--> Invalid Subpixel mode selection: " << stereo_settings().subpixel_mode << std::endl;
      vw_out() << "\t--> Doing nothing\n";
    }

    asp::block_write_gdal_image( opt.out_prefix + "-RD.tif",
                                 disparity_map, opt,
                                 TerminalProgressCallback("asp", "\t--> Refinement :") );

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at refinement stage -- could not read input files.\n" << e.what() << "\nExiting.\n\n" );
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
    stereo_refinement( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : REFINEMENT FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
