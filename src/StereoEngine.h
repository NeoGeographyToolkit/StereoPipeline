#ifndef __STEREO_ENGINE_H__
#define __STEREO_ENGINE_H__

#include <vw/Stereo/DisparityMap.h>
#include <vw/Stereo/OptimizedCorrelator.h>
#include <vw/Stereo/MultiresolutionCorrelator.h>
#include <vw/Stereo/BlockCorrelator.h>
#include <vw/Stereo/PyramidCorrelator.h>
#include <vw/Math/BBox.h>
#include <vw/Image/ImageView.h>

#include "SurfaceNURBS.h"

class MultiresCorrelatorNURBSHoleFiller : public vw::stereo::MultiresCorrelatorHoleFiller {
  int m_nurbs_iterations;
public:
  MultiresCorrelatorNURBSHoleFiller(int nurbs_iterations) : m_nurbs_iterations(nurbs_iterations) {}
  virtual ~MultiresCorrelatorNURBSHoleFiller() {} 
  virtual void operator() (vw::ImageView<vw::PixelDisparity<float> > &disparity_map) {
    vw::ImageView<vw::PixelDisparity<float> > output_disparity;
    output_disparity = MBASurfaceNURBS(disparity_map, m_nurbs_iterations);
    disparity_map = output_disparity;
  }
};

class StereoEngine {

public: 
  bool do_slog;
  bool do_log;
  bool do_cleanup;
  bool do_nurbs_hole_filling;
  bool do_horizontal_pyramid_search;
  bool do_vertical_pyramid_search;

  vw::BBox<int, 2> search_range;
  int kernel_width;
  int kernel_height;
  float slog_stddev;
  float cross_correlation_threshold;
  int block_size;
  bool use_multiresolution_correlator;

  int cleanup_passes;
  int cleanup_vertical_half_kernel;
  int cleanup_horizontal_half_kernel;
  int cleanup_min_matches;
  float cleanup_rm_threshold;

  int nurbs_iterations;

  /// The Stereo Engine constructor gives reasonable default values
  /// for most parameters.
  StereoEngine() {
    do_slog = true;
    do_log = false;
    do_cleanup = true;
    do_nurbs_hole_filling = true;
    do_horizontal_pyramid_search = false;
    do_vertical_pyramid_search = false;
    use_multiresolution_correlator = false; 

    kernel_width = 24;
    kernel_height = 24;
    slog_stddev = 1.5;
    cross_correlation_threshold = 2;
    block_size = 2048;
    cleanup_passes = 1;
    cleanup_horizontal_half_kernel = 5;
    cleanup_vertical_half_kernel = 5;
    cleanup_min_matches = 60;
    cleanup_rm_threshold = 0.8;
    nurbs_iterations = 10;
  }
  
  vw::ImageView<vw::PixelDisparity<float> > correlate (vw::ImageView<vw::PixelGray<float> > Limg, 
                                                       vw::ImageView<vw::PixelGray<float> > Rimg) {
    
    // determine the search window parameters 
    if(do_horizontal_pyramid_search || do_vertical_pyramid_search && !use_multiresolution_correlator) {
      std::cout << "\n ------------ Pyramid Search for Correlation Space --------- \n";
      printf("Starting pyramid search with initial search dimensions H: [ %d, %d ] V: [ %d, %d ]\n",
             search_range.min().x(), search_range.max().x(), search_range.min().y(), search_range.max().y());
      
      try {

        vw::stereo::PyramidCorrelator pyramid_correlator(search_range.min().x(), search_range.max().x(),
                                                         search_range.min().y(), search_range.max().y(),
                                                         kernel_width, kernel_height, 
                                                         true, cross_correlation_threshold,
                                                         slog_stddev);
        //        pyramid_correlator.enable_debug_mode("pyramid");
        search_range = pyramid_correlator(Limg, Rimg, do_horizontal_pyramid_search, do_vertical_pyramid_search);
        
      } catch (vw::stereo::CorrelatorErr &e) {
        std::cout << "Pyramid correlation failed:\n";
        e.what();
        std::cout << "Exiting.\n\n";
        exit(1);
      }
    }
    
    printf("------------------------- correlation ----------------------\n");    
    vw::ImageView<vw::PixelDisparity<float> > disparity_map;
    if (use_multiresolution_correlator) {
      vw::stereo::MultiresolutionCorrelator correlator(search_range.min().x(), search_range.max().x(), 
                                                       search_range.min().y(), search_range.max().y(),
                                                       kernel_width, kernel_height, 
                                                       true,         // verbose
                                                       cross_correlation_threshold,
                                                       slog_stddev,
                                                       true, true);  // subpixel both vert/horz
      correlator.set_min_dimension(512); // Hard coded to a reasonable value for now
      MultiresCorrelatorNURBSHoleFiller nurbs_hole_filler(nurbs_iterations);

      // This doesn't work as well as I had hoped.  Maybe more
      // debugging is in order? -mbroxton
      //       correlator.set_hole_filler(&nurbs_hole_filler);
      //       correlator.set_debug_mode("test/test");
      disparity_map = correlator(Limg, Rimg);
    } else {
      vw::stereo::BlockCorrelator correlator(search_range.min().x(), search_range.max().x(), 
                                             search_range.min().y(), search_range.max().y(),
                                             kernel_width, kernel_height, 
                                             true,         // verbose
                                             cross_correlation_threshold,
                                             block_size,   // Block correlator block size
                                             true, true);  // subpixel both vert/horz
      
      // perform the sign of laplacian of gaussian filter (SLOG) on the images 
      if(do_slog) {
        printf("Applying SLOG filter.\n");
        vw::ImageView<vw::PixelGray<uint8> > bit_Limg = vw::channel_cast<vw::uint8>(vw::threshold(vw::laplacian_filter(vw::gaussian_filter(Limg,slog_stddev)), 0.0));
        vw::ImageView<vw::PixelGray<uint8> > bit_Rimg = vw::channel_cast<vw::uint8>(vw::threshold(vw::laplacian_filter(vw::gaussian_filter(Rimg,slog_stddev)), 0.0));
        disparity_map = correlator(bit_Limg, bit_Rimg, true);
      } else if (do_log) {
        printf("Applying LOG filter.\n");
        Limg = vw::laplacian_filter(vw::gaussian_filter(Limg, slog_stddev));
        Rimg = vw::laplacian_filter(vw::gaussian_filter(Rimg, slog_stddev));
        disparity_map = correlator(Limg, Rimg, false);
      }
    
        
      printf("Cleaning up disparity map...\n");
      for(int nn=0; nn < do_cleanup; nn++) {
        vw::stereo::disparity::clean_up(disparity_map,
                                        cleanup_vertical_half_kernel,
                                        cleanup_horizontal_half_kernel,
                                        cleanup_min_matches,
                                        cleanup_rm_threshold,
                                        true);
      }
    }

    return disparity_map;
  }

  void filter(vw::ImageView<vw::PixelDisparity<float> > &disparity_map) {

    // Clean up the disparity map one more time (for good measure?)
    std::cout << "\nCleaning up disparity map prior to filtering processes.\n";
    
    for (int i = 0; i < 1; i++) {
      vw::stereo::disparity::clean_up(disparity_map,
                          cleanup_vertical_half_kernel,
                          cleanup_horizontal_half_kernel,
                          cleanup_min_matches,
                          cleanup_rm_threshold,
                          true);
    }
//     std::cout << "\tRemoving solitary pixels [20x20 window, 20% threshold]\n";
//     vw::stereo::disparity::remove_outliers(disparity_map, 20, 20, 20, 200, true);

    // These settings should move from being hard coded to being user configurable
    vw::stereo::disparity::sparse_disparity_filter(disparity_map, 20, 0.5);
    
    //     if(execute.fill_v_holes)
    //       disparity_map.fillHolesVert((double)dft.v_fill_treshold);
    //     if(execute.fill_h_holes) 
    //       disparity_map.fillHolesHorz();
    //     if(execute.extend_lr)
    //       disparity_map.extendLRDisp(dft.Lextend, dft.Rextend);
    //     if(execute.extend_tb)
    //       disparity_map.extendTBDisp(dft.Textend, dft.Bextend,
    // 				 dft.Toffset, dft.Boffset);
    
    //     if(execute.smooth_disp){
    //       smooth_disp(disparity_map.getHBuffer(),
    // 		  disparity_map.getHBuffer(),
    // 		  dft.smooth_disp_M, 
    // 		  dft.smooth_disp_N, &hd, &dft);
    //       smooth_disp(disparity_map.getVBuffer(),
    // 		  disparity_map.getVBuffer(),
    // 		  dft.smooth_disp_M, 
    // 		  dft.smooth_disp_N, &hd, &dft);
    //     }
  }  
  void interpolate(vw::ImageView<vw::PixelDisparity<float> > &disparity_map) {

    // Call out to NURBS hole filling code.     
    if(do_nurbs_hole_filling) {
      vw::ImageView<vw::PixelDisparity<float> > output_disparity;
      output_disparity = MBASurfaceNURBS(disparity_map, nurbs_iterations);
      disparity_map = output_disparity;
    }

  }

};


std::ostream& operator<<(std::ostream& os, StereoEngine const& s) {
  
  std::cout << "STEREO ENGINE SETTINGS\n";
  std::cout << "----------------------\n\n";
  std::cout << "SLOG\n";
  if (s.do_slog) {
    std::cout << "\tMode: SLOG\n";
    std::cout << "\tStdDev: " << s.slog_stddev << "\n";
  } else if (s.do_log) {
    std::cout << "\tMode: LOG\n";
    std::cout << "\tStdDev: " << s.slog_stddev << "\n";
  } else  
    std::cout << "\tMode: None\n";
  std::cout << "Correlation:\n";
  std::cout << "\th_corr_min: " << s.search_range.min().x() << "\n";
  std::cout << "\th_corr_max: " << s.search_range.max().x() << "\n";
  std::cout << "\tv_corr_min: " << s.search_range.min().y() << "\n";
  std::cout << "\tv_corr_max: " << s.search_range.max().y() << "\n";
  std::cout << "\th_kern: " << s.kernel_width << "\n";
  std::cout << "\tv_kern: " << s.kernel_height << "\n";
  std::cout << "\txcorr_treshold: " << s.cross_correlation_threshold << "\n";
  std::cout << "\n";
}

#endif	// __STEREO_ENGINE_H__
