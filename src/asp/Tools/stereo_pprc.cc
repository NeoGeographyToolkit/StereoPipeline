// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <asp/Core/ThreadedEdgeMask.h>

using namespace vw;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

void stereo_preprocessing( Options& opt ) {

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 0 --> PREPROCESSING \n";

  std::string pre_preprocess_file1, pre_preprocess_file2;
  opt.session->pre_preprocessing_hook(opt.in_file1, opt.in_file2,
                                      pre_preprocess_file1,
                                      pre_preprocess_file2);

  DiskImageView<PixelGray<float> > left_image(pre_preprocess_file1),
    right_image(pre_preprocess_file2);

  bool rebuild = false;
  try {
    vw_log().console_log().rule_set().add_rule(-1,"fileio");
    DiskImageView<PixelGray<uint8> > testa(opt.out_prefix+"-lMask.tif");
    DiskImageView<PixelGray<uint8> > testb(opt.out_prefix+"-rMask.tif");
    vw_settings().reload_config();
  } catch (vw::IOErr const& e) {
    vw_settings().reload_config();
    rebuild = true;
  } catch (vw::ArgumentErr const& e ) {
    // Throws on a corrupted file.
    vw_settings().reload_config();
    rebuild = true;
  }
  if (rebuild) {
    vw_out() << "\t--> Generating image masks... \n";

    asp::block_write_gdal_image( opt.out_prefix+"-lMask.tif",
                                 apply_mask(copy_mask(constant_view(uint8(255),left_image.cols(),
                                                                    left_image.rows() ),
                                                      asp::threaded_edge_mask(left_image,0,0,1024))),
                                 opt, TerminalProgressCallback("asp", "\t    Mask L: ") );
    asp::block_write_gdal_image( opt.out_prefix+"-rMask.tif",
                                 apply_mask(copy_mask(constant_view(uint8(255),right_image.cols(),
                                                                    right_image.rows() ),
                                                      asp::threaded_edge_mask(right_image,0,0,1024))),
                                 opt, TerminalProgressCallback("asp", "\t    Mask R: ") );
  }

  try {
    // This confusing try catch is to see if the subsampled images
    // actually have content.
    DiskImageView<PixelGray<float> > testa(opt.out_prefix+"-L_sub.tif");
    DiskImageView<PixelGray<float> > testb(opt.out_prefix+"-R_sub.tif");
    vw_out() << "\t--> Using cached subsampled image.\n";
  } catch (vw::Exception const& e) {
    // Produce subsampled images, these will be used later for Auto
    // search range. They're also a handy debug tool.
    float sub_scale =
      sqrt(1500.0 * 1500.0 / (float(left_image.cols()) * float(left_image.rows())));
    sub_scale +=
      sqrt(1500.0 * 1500.0 / (float(right_image.cols()) * float(right_image.rows())));
    sub_scale /= 2;
    if ( sub_scale > 1 ) sub_scale = 1;

    // Solving for the number of threads and the tile size to use for
    // subsampling while only using 500 MiB of memory. (The cache code
    // is a little slow on releasing so it will probably use 1.5GiB
    // memory during subsampling) Also tile size must be a power of 2
    // and greater than or equal to 64 px;
    uint32 sub_threads = vw_settings().default_num_threads() + 1;
    uint32 tile_power = 0;
    while ( tile_power < 6 && sub_threads > 1) {
      sub_threads--;
      tile_power = boost::numeric_cast<uint32>( log10(500e6*sub_scale*sub_scale/(4.0*float(sub_threads)))/(2*log10(2)));
    }
    uint32 sub_tile_size = 1u << tile_power;
    if ( sub_tile_size > vw_settings().default_tile_size() )
      sub_tile_size = vw_settings().default_tile_size();

    // Change writing parameters to ideal threads and tiles
    vw_out() << "\t--> Creating previews. Subsampling by " << sub_scale
             << " by using " << sub_tile_size << " tile size and "
             << sub_threads << " threads.\n";
    Vector2i previous_tile_size = opt.raster_tile_size;
    opt.raster_tile_size = Vector2i(sub_tile_size,sub_tile_size);
    uint32 previous_num_threads = vw_settings().default_num_threads();
    vw_settings().set_default_num_threads(sub_threads);
    asp::block_write_gdal_image( opt.out_prefix+"-L_sub.tif",
                                 resample(left_image, sub_scale), opt,
                                 TerminalProgressCallback("asp", "\t    Sub L: ") );
    asp::block_write_gdal_image( opt.out_prefix+"-R_sub.tif",
                                 resample(right_image, sub_scale), opt,
                                 TerminalProgressCallback("asp", "\t    Sub R: ") );
    opt.raster_tile_size = previous_tile_size;
    vw_settings().set_default_num_threads(previous_num_threads);
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
    vw_out() << "Using \"" << opt.stereo_default_filename << "\"\n";
    stereo_preprocessing( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : PREPROCESSING FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
