#ifndef __ASP_STEREO_FILTERING_H__
#define __ASP_STEREO_FILTERING_H__

#include <asp/Tools/stereo.h>

namespace vw {

  void stereo_filtering( Options& opt ) {
    if (opt.entry_point == FILTERING)
      vw_out() << "\nStarting at the FILTERING stage.\n";
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
        DiskImageView<PixelMask<Vector2f> > disparity_disk_image(post_correlation_fname);
        ImageViewRef<PixelMask<Vector2f> > disparity_map = disparity_disk_image;

        vw_out() << "\t--> Cleaning up disparity map prior to filtering processes (" << stereo_settings().rm_cleanup_passes << " passes).\n";
        for (int i = 0; i < stereo_settings().rm_cleanup_passes; ++i)
          disparity_map =
            stereo::disparity_clean_up(disparity_map,
                                       stereo_settings().rm_h_half_kern,
                                       stereo_settings().rm_v_half_kern,
                                       stereo_settings().rm_threshold,
                                       stereo_settings().rm_min_matches/100.0);

        // Applying additional clipping from the edge. We make new
        // mask files to avoid a weird and tricky segfault due to
        // ownership issues.
        DiskImageView<vw::uint8> left_mask( opt.out_prefix+"-lMask.tif" );
        DiskImageView<vw::uint8> right_mask( opt.out_prefix+"-rMask.tif" );
        int mask_buffer = std::max( stereo_settings().subpixel_h_kern,
                                    stereo_settings().subpixel_v_kern );
        ImageViewRef<vw::uint8> Lmaskmore, Rmaskmore;
        Lmaskmore = apply_mask(edge_mask(left_mask,0,mask_buffer));
        Rmaskmore = apply_mask(edge_mask(right_mask,0,mask_buffer));
        DiskCacheImageView<vw::uint8>
          left_red_mask(Lmaskmore, "tif",
                        TerminalProgressCallback("asp","\t  Reduce LMask:"),
                        stereo_settings().cache_dir);
        DiskCacheImageView<vw::uint8>
          right_red_mask(Rmaskmore, "tif",
                         TerminalProgressCallback("asp","\t  Reduce RMask:"),
                         stereo_settings().cache_dir);

        disparity_map = stereo::disparity_mask( disparity_map,
                                                left_red_mask, right_red_mask );

        if ( stereo_settings().mask_flatfield ) {
          // This is only turned on for apollo. Blob detection doesn't
          // work to great when tracking a whole lot of spots. HiRISE
          // seems to keep breaking this so I've keep it turned off.
          //
          // The crash happens inside Boost Graph when dealing with
          // large number of blobs.
          DiskCacheImageView<PixelMask<Vector2f> >
            filtered_disp(disparity_map, "tif",
                          TerminalProgressCallback("asp","\t  Intermediate:"),
                          stereo_settings().cache_dir);

          vw_out() << "\t--> Rasterizing filtered disparity map to disk. \n";
          BlobIndexThreaded bindex( filtered_disp,
                                    stereo_settings().erode_max_size );
          vw_out() << "\t    * Eroding " << bindex.num_blobs() << " islands\n";
          ImageViewRef<PixelMask<Vector2f> > erode_disp_map;
          erode_disp_map = ErodeView<DiskCacheImageView<PixelMask<Vector2f> > >(filtered_disp, bindex );
          //erode_disp_map = filtered_disp;
          DiskImageResourceGDAL erode_map_rsrc(opt.out_prefix+"-FTemp.tif",
                                               erode_disp_map.format(),
                                               opt.raster_tile_size,
                                               opt.gdal_options );
          block_write_image( erode_map_rsrc, erode_disp_map,
                             TerminalProgressCallback("asp", "\t--> Eroding: "));
        } else {
          DiskImageResourceGDAL filt_rsrc( opt.out_prefix+"-FTemp.tif",
                                           disparity_map.format(),
                                           opt.raster_tile_size,
                                           opt.gdal_options );
          block_write_image( filt_rsrc, disparity_map,
                             TerminalProgressCallback("asp", "\t--> Filterd: "));
        }
      }

      DiskImageView<PixelMask<Vector2f> > filtered_disparity_map( opt.out_prefix+"-FTemp.tif" );

      { // Write Good Pixel Map
        vw_out() << "\t--> Creating \"Good Pixel\" image: "
                 << (opt.out_prefix + "-GoodPixelMap.tif") << "\n";
        {
          // Sub-sampling so that the user can actually view it.
          float sub_scale = 2048 / float( std::min( filtered_disparity_map.cols(),
                                                    filtered_disparity_map.rows() ) );
          if ( sub_scale > 1 ) sub_scale = 1;
          // Solving for the number of threads and the tile size to use for
          // subsampling while only using 500 MiB of memory. (The cache code
          // is a little slow on releasing so it will probably use 1.5GiB
          // memory during subsampling) Also tile size must be a power of 2
          // and greater than or equal to 64 px;
          int previous_num_threads = vw_settings().default_num_threads();
          int sub_threads = vw_settings().default_num_threads() + 1;
          int tile_power = 0;
          while ( tile_power < 6 && sub_threads > 1) {
            sub_threads--;
            tile_power = int( log10(500e6*sub_scale*sub_scale/(4.0*float(sub_threads)))/(2*log10(2)));
          }
          int sub_tile_size = int ( pow(2., tile_power) );
          if ( sub_tile_size > vw_settings().default_tile_size() )
            sub_tile_size = vw_settings().default_tile_size();

          ImageViewRef<PixelRGB<uint8> > good_pixel =
            resample(stereo::missing_pixel_image(filtered_disparity_map),
                     sub_scale);
          vw_settings().set_default_num_threads(sub_threads);
          DiskImageResourceGDAL good_pixel_rsrc( opt.out_prefix + "-GoodPixelMap.tif",
                                                 good_pixel.format(),
                                                 Vector2i(sub_tile_size,
                                                          sub_tile_size ),
                                                 opt.gdal_options );
          block_write_image( good_pixel_rsrc, good_pixel,
                             TerminalProgressCallback("asp", "\t    Writing: "));
          vw_settings().set_default_num_threads(previous_num_threads);
        }
      }

      // Fill Holes
      ImageViewRef<PixelMask<Vector2f> > hole_filled_disp_map;
      if(stereo_settings().fill_holes) {
        vw_out() << "\t--> Filling holes with Inpainting method.\n";
        BlobIndexThreaded bindex( invert_mask( filtered_disparity_map ),
                                  stereo_settings().fill_hole_max_size );
        vw_out() << "\t    * Identified " << bindex.num_blobs() << " holes\n";
        hole_filled_disp_map =
          InpaintView<DiskImageView<PixelMask<Vector2f> > >(filtered_disparity_map, bindex );
      } else {
        hole_filled_disp_map = filtered_disparity_map;
      }

      DiskImageResourceGDAL disparity_map_rsrc(opt.out_prefix + "-F.tif",
                                               hole_filled_disp_map.format(),
                                               opt.raster_tile_size,
                                               opt.gdal_options );
      block_write_image(disparity_map_rsrc, hole_filled_disp_map,
                        TerminalProgressCallback("asp", "\t--> Filtering: ") );

      // Delete temporary file
      std::string temp_file =  opt.out_prefix+"-FTemp.tif";
      unlink( temp_file.c_str() );
    } catch (IOErr &e) {
      vw_throw( ArgumentErr() << "\nUnable to start at filtering stage -- could not read input files.\n"
                << e.what() << "\nExiting.\n\n" );
    }
  }

} // ending namespace vw

#endif//__ASP_STEREO_FILTERING_H__
