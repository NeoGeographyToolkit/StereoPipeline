// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo_preprocessing.h

#ifndef __ASP_STEREO_PREPROCESSING_H__
#define __ASP_STEREO_PREPROCESSING_H__

#include <asp/Tools/stereo.h>

namespace vw {

  void stereo_preprocessing( Options& opt ) {

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 0 --> PREPROCESSING \n";

    std::string pre_preprocess_file1, pre_preprocess_file2;
    opt.session->pre_preprocessing_hook(opt.in_file1, opt.in_file2,
                                        pre_preprocess_file1,
                                        pre_preprocess_file2);

    DiskImageView<PixelGray<float> > left_image(pre_preprocess_file1),
      right_image(pre_preprocess_file2);

    try {
      DiskImageView<PixelGray<uint8> > testa(opt.out_prefix+"-lMask.tif");
      DiskImageView<PixelGray<uint8> > testb(opt.out_prefix+"-rMask.tif");
      vw_out() << "\t--> Using cached image masks.\n";
    } catch (vw::Exception const& e) {
      vw_out() << "\t--> Generating image masks... \n";

      ImageViewRef<vw::uint8> Lmask =
        pixel_cast<vw::uint8>(threshold(apply_mask(edge_mask(left_image, 0, 0)),0,0,255));
      ImageViewRef<vw::uint8> Rmask =
        pixel_cast<vw::uint8>(threshold(apply_mask(edge_mask(right_image, 0, 0)),0,0,255));

      DiskImageResourceGDAL l_mask_rsrc( opt.out_prefix+"-lMask.tif",
                                         Lmask.format(), opt.raster_tile_size,
                                         opt.gdal_options );
      DiskImageResourceGDAL r_mask_rsrc( opt.out_prefix+"-rMask.tif",
                                         Rmask.format(), opt.raster_tile_size,
                                         opt.gdal_options );
      block_write_image(l_mask_rsrc, Lmask,
                        TerminalProgressCallback("asp", "\t    Mask L: "));
      block_write_image(r_mask_rsrc, Rmask,
                        TerminalProgressCallback("asp", "\t    Mask R: "));
    }

    // Produce subsampled images, these will be used later for Auto
    // search range. They're also a handy debug tool.
    int smallest_edge = std::min(std::min(left_image.cols(), left_image.rows()),
                                 std::min(right_image.cols(), right_image.rows()) );
    float sub_scale = 2048.0 / float(smallest_edge);;
    if ( sub_scale > 1 ) sub_scale = 1;

    // Solving for the number of threads and the tile size to use for
    // subsampling while only using 500 MiB of memory. (The cache code
    // is a little slow on releasing so it will probably use 1.5GiB
    // memory during subsampling) Also tile size must be a power of 2
    // and greater than or equal to 64 px;
    int sub_threads = vw_settings().default_num_threads() + 1;
    int tile_power = 0;
    while ( tile_power < 6 && sub_threads > 1) {
      sub_threads--;
      tile_power = int( log10(500e6*sub_scale*sub_scale/(4.0*float(sub_threads)))/(2*log10(2)));
    }
    uint32 sub_tile_size = 1u << tile_power;
    if ( sub_tile_size > vw_settings().default_tile_size() )
      sub_tile_size = vw_settings().default_tile_size();

    std::string l_sub_file = opt.out_prefix+"-L_sub.tif";
    std::string r_sub_file = opt.out_prefix+"-R_sub.tif";
    try {
      DiskImageView<PixelGray<float> > testa(l_sub_file);
      DiskImageView<PixelGray<float> > testb(r_sub_file);
      vw_out() << "\t--> Using cached subsampled image.\n";
    } catch (vw::Exception const& e) {
      vw_out() << "\t--> Creating previews. Subsampling by " << sub_scale
               << " by using " << sub_tile_size << " tile size and "
               << sub_threads << " threads.\n";
      ImageViewRef<PixelGray<vw::float32> > Lsub = resample(left_image, sub_scale);
      ImageViewRef<PixelGray<vw::float32> > Rsub = resample(right_image, sub_scale);
      DiskImageResourceGDAL l_sub_rsrc( l_sub_file, Lsub.format(),
                                        Vector2i(sub_tile_size,
                                                 sub_tile_size),
                                        opt.gdal_options );
      DiskImageResourceGDAL r_sub_rsrc( r_sub_file, Rsub.format(),
                                        Vector2i(sub_tile_size,
                                                 sub_tile_size),
                                        opt.gdal_options );
      int previous_num_threads = vw_settings().default_num_threads();
      vw_settings().set_default_num_threads(sub_threads);
      block_write_image(l_sub_rsrc, Lsub,
                        TerminalProgressCallback("asp", "\t    Sub L: "));
      block_write_image(r_sub_rsrc, Rsub,
                        TerminalProgressCallback("asp", "\t    Sub R: "));
      vw_settings().set_default_num_threads(previous_num_threads);
    }
  }

} // end namespace vw

#endif//__ASP_STEREO_PREPROCESSING_H__
