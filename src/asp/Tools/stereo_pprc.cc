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

/// \file stereo_pprc.cc
///
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Bathymetry.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspLog.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/ThreadedEdgeMask.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Tools/stereo.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Image/AntiAliasing.h>
#include <vw/Image/WindowAlgorithms.h>
#include <vw/Image/UtilityViews.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/Functors.h>

#include <xercesc/util/PlatformUtils.hpp>
#include <vw/Image/Manipulation.h>

using namespace vw;
using namespace asp;

// Check that the images in file1 and file2 have same size, and throw
// an exception if they don't.
void check_image_sizes(std::string const& file1, std::string const& file2) {
  DiskImageView<float> left_image (file1), right_image(file2);
  if (left_image.cols() != right_image.cols() ||
      left_image.rows() != right_image.rows())
    vw_throw( ArgumentErr() << "Expecting images: " << file1 << " and " << file2
              << " to have the same dimensions. Delete this run and start all over.\n");
}

// Create subsampled images and masks
void createSubsampledImages(ASPGlobalOptions const& opt,
                            std::vector<std::string> const& in_file_list,
                            bool crop_left, bool crop_right,
                            DiskImageView<PixelGray<float>> const& left_image,
                            DiskImageView<PixelGray<float>> const& right_image,
                            std::string const& left_mask_file,
                            std::string const& right_mask_file,
                            bool has_left_georef, bool has_right_georef,
                            cartography::GeoReference const& left_georef,
                            cartography::GeoReference const& right_georef,
                            float output_nodata) {

  bool has_nodata = true;

  std::string lsub  = opt.out_prefix+"-L_sub.tif";
  std::string rsub  = opt.out_prefix+"-R_sub.tif";
  std::string lmsub = opt.out_prefix+"-lMask_sub.tif";
  std::string rmsub = opt.out_prefix+"-rMask_sub.tif";

  bool inputs_changed = (!first_is_newer(lsub,  in_file_list) ||
                    !first_is_newer(rsub,  in_file_list) ||
                    !first_is_newer(lmsub, in_file_list) ||
                    !first_is_newer(rmsub, in_file_list));

  // We must always redo the subsampling if we are allowed to crop the images
  bool rebuild = crop_left || crop_right || inputs_changed;

  try {
    // First try to see if the subsampled images exist.
    if (!fs::exists(lsub)  || !fs::exists(rsub) ||
        !fs::exists(lmsub) || !fs::exists(rmsub)) {
      rebuild = true;
    } else {
      // See if the subsampled images are valid
      DiskImageView<PixelGray<float>> testl(lsub);
      DiskImageView<PixelGray<float>> testr(rsub);
      DiskImageView<uint8> testlm(lmsub);
      DiskImageView<uint8> testrm(rmsub);
      vw_out() << "\t--> Using cached subsampled images.\n";
    }
  } catch (vw::Exception const& e) {
    rebuild = true;
  }

  if (rebuild) {
    // Produce subsampled images, these will be used later for auto
    // search range detection.
    double s = 1500.0;
    float  sub_scale = sqrt(s * s / (float(left_image.cols ()) * float(left_image.rows ())))
                     + sqrt(s * s / (float(right_image.cols()) * float(right_image.rows())));
    sub_scale /= 2;
    if ( sub_scale > 0.6 ) // ???
      sub_scale = 0.6;

    // Solving for the number of threads and the tile size to use for
    // subsampling while only using 500 MiB of memory. (The cache code
    // is a little slow on releasing so it will probably use 1.5GiB
    // memory during subsampling) Also tile size must be a power of 2
    // and greater than or equal to 64 px.
    uint32 sub_threads = vw_settings().default_num_threads() + 1;
    uint32 tile_power  = 0;
    while (tile_power < 6 && sub_threads > 1) {
      sub_threads--;
      tile_power = boost::numeric_cast<uint32>
        (log10(500e6*sub_scale*sub_scale/(4.0*float(sub_threads)))/(2*log10(2)));
    }
    uint32 sub_tile_size = 1u << tile_power;
    if (sub_tile_size > vw_settings().default_tile_size())
      sub_tile_size = vw_settings().default_tile_size();
    Vector2 sub_tile_size_vec(sub_tile_size, sub_tile_size);
    
    vw_out() << "Creating subimages and submasks. Subsampling by " << sub_scale
             << " by using a tile of size " << sub_tile_size << " and "
             << sub_threads << " threads.\n";

    // Resample the images and the masks. We must use the masks when
    // resampling the images to interpolate correctly around invalid pixels.

    DiskImageView<uint8> left_mask(left_mask_file), right_mask(right_mask_file);
    // Below we use ImageView instead of ImageViewRef as the output
    // images are small.  Using an ImageViewRef would make the
    // subsampling operations happen twice, once for L_sub.tif and
    // second time for lMask_sub.tif.
    ImageView<PixelMask<PixelGray<float>>> left_sub_image, right_sub_image;
    if (sub_scale > 0.5) {
      // When we are near the pixel input to output ratio, standard
      // interpolation gives the best possible results.
      left_sub_image  = block_rasterize(
        resample(copy_mask(left_image, create_mask(left_mask)), sub_scale), 
                 sub_tile_size_vec, sub_threads);
      right_sub_image = block_rasterize(
        resample(copy_mask(right_image, create_mask(right_mask)), sub_scale), 
                 sub_tile_size_vec, sub_threads);
    } else {
      // When we heavily reduce the image size, super sampling seems
      // like the best approach. The method below should be equivalent.
      left_sub_image = block_rasterize(
          cache_tile_aware_render(resample_aa(copy_mask(left_image, create_mask(left_mask)),
                                              sub_scale), Vector2i(256,256) * sub_scale),
         sub_tile_size_vec, sub_threads);
      right_sub_image = block_rasterize(
        cache_tile_aware_render(resample_aa(copy_mask(right_image, create_mask(right_mask)),
                                             sub_scale), Vector2i(256,256) * sub_scale),
         sub_tile_size_vec, sub_threads);
    }

    // Enforce no predictor in compression, it works badly with sub-images
    vw::GdalWriteOptions opt_nopred = opt;
    opt_nopred.gdal_options["PREDICTOR"] = "1";
    opt_nopred.cog = false;  // Do not use COG with preprocessing intermediates

    vw::cartography::GeoReference left_sub_georef, right_sub_georef;
    if (has_left_georef) {
      // Account for scale.
      double left_scale = 0.5 * (double(left_sub_image.cols())/left_image.cols() + 
                                 double(left_sub_image.rows())/left_image.rows());
      left_sub_georef = resample(left_georef, left_scale);
    }
    if (has_right_georef) {
      // Account for scale.
      double right_scale = 0.5 * (double(right_sub_image.cols())/right_image.cols() + 
                                  double(right_sub_image.rows())/right_image.rows());
      right_sub_georef = resample(right_georef, right_scale);
    }

    vw::cartography::block_write_gdal_image(
      lsub, apply_mask(left_sub_image, output_nodata),
      has_left_georef, left_sub_georef,
      has_nodata, output_nodata,
      opt_nopred, TerminalProgressCallback("asp", "\t    Sub L: ") );
    vw::cartography::block_write_gdal_image(
      rsub, apply_mask(right_sub_image, output_nodata),
      has_right_georef, right_sub_georef,
      has_nodata, output_nodata,
      opt_nopred, TerminalProgressCallback("asp", "\t    Sub R: ") );
    vw::cartography::block_write_gdal_image(
      lmsub, channel_cast_rescale<uint8>(select_channel(left_sub_image, 1)),
      has_left_georef, left_sub_georef,
      has_nodata, output_nodata,
      opt_nopred, TerminalProgressCallback("asp", "\t    Sub L Mask: ") );
    vw::cartography::block_write_gdal_image(
      rmsub, channel_cast_rescale<uint8>(select_channel(right_sub_image, 1)),
      has_right_georef, right_sub_georef,
      has_nodata, output_nodata,
      opt_nopred, TerminalProgressCallback("asp", "\t    Sub R Mask: ") );
  } // End rebuild check
} // End function createSubsampledImages

// Create the image masks for the left and right images
void createImageMasks(ASPGlobalOptions & opt,
                      boost::shared_ptr<DiskImageResource> left_rsrc,
                      boost::shared_ptr<DiskImageResource> right_rsrc,
                      DiskImageView<PixelGray<float>> const& left_image,
                      DiskImageView<PixelGray<float>> const& right_image,
                      bool skip_img_norm,
                      bool has_left_georef, bool has_right_georef,
                      cartography::GeoReference const& left_georef,
                      cartography::GeoReference const& right_georef,
                      bool has_nodata, float output_nodata,
                      std::string const& left_mask_file,
                      std::string const& right_mask_file) {

  vw_out() << "\t--> Generating image masks.\n";

  vw::Stopwatch sw;
  sw.start();

  // Read the no-data values of L.tif and R.tif.
  float left_nodata_value  = std::numeric_limits<float>::quiet_NaN();
  float right_nodata_value = std::numeric_limits<float>::quiet_NaN();
  if (left_rsrc->has_nodata_read())
    left_nodata_value  = left_rsrc->nodata_read();
  if (right_rsrc->has_nodata_read())
    right_nodata_value = right_rsrc->nodata_read();

  // We need to treat the following special case: if the user
  // skipped image normalization, so we are still using the original
  // input images, and the user wants to use a custom no-data value,
  // this is the time to apply it.
  if (skip_img_norm && !std::isnan(stereo_settings().nodata_value)) {
    left_nodata_value  = stereo_settings().nodata_value;
    right_nodata_value = stereo_settings().nodata_value;
  }

  // TODO(oalexan1): Remove the ThreadedEdgeMask logic, as it is slow. But need
  // to ensure results are acceptable with both raw and mapprojected images.
  ImageViewRef<PixelMask<uint8>> left_mask
    = copy_mask(constant_view(uint8(255),
                              left_image.cols(), left_image.rows()),
                asp::threaded_edge_mask(left_image, 0, 0, 1024));
  ImageViewRef<PixelMask<uint8>> right_mask
    = copy_mask(constant_view(uint8(255),
                              right_image.cols(), right_image.rows()),
                asp::threaded_edge_mask(right_image, 0, 0, 1024));

  // Mask no-data pixels.
  left_mask = intersect_mask(left_mask,
                             create_mask_less_or_equal(left_image,
                                                       left_nodata_value));
  right_mask = intersect_mask(right_mask,
                              create_mask_less_or_equal(right_image,
                                                        right_nodata_value));

  // Mask out regions with low input pixel value standard deviations if the
  // user requested it.
  if (stereo_settings().nodata_stddev_kernel > 0) {
    Vector2i stddev_kernel(stereo_settings().nodata_stddev_kernel,
                           stereo_settings().nodata_stddev_kernel);

    // If the threshold value is negative write out a debug image
    // instead, allowing the user to tune the threshold.
    // TODO(oalexan1): See if to wipe this code.
    if (stereo_settings().nodata_stddev_thresh < 0) {
      vw::cartography::block_write_gdal_image
        (opt.out_prefix + "-L_stddev_filter_output.tif",
         vw::stddev_filter_view(left_image, stddev_kernel),
         has_left_georef, left_georef,
         false, output_nodata,
         opt, TerminalProgressCallback
         ("asp", "\t  StdDev filter raw output (left): "));
      vw::cartography::block_write_gdal_image
        (opt.out_prefix + "-R_stddev_filter_output.tif",
         vw::stddev_filter_view(right_image, stddev_kernel),
         has_right_georef, right_georef,
         false, output_nodata,
         opt, TerminalProgressCallback("asp", "\t  StdDev filter raw output (right): "));
    } else {
      left_mask  = intersect_mask(left_mask,
                                  create_mask_less_or_equal
                                  (vw::stddev_filter_view(left_image, stddev_kernel),
                                   stereo_settings().nodata_stddev_thresh));
      right_mask = intersect_mask(right_mask,
                                  create_mask_less_or_equal
                                  (vw::stddev_filter_view(right_image, stddev_kernel),
                                   stereo_settings().nodata_stddev_thresh));
    }
  }

  // Intersect the left mask with the warped version of the right
  // mask, and vice-versa to reduce noise, if the images
  // are map-projected.
  vw_out() << "Writing masks: " << left_mask_file << ' ' << right_mask_file << ".\n";
  if (has_left_georef && has_right_georef && opt.session->isMapProjected()) {

    // Write with big blocks. Small blocks results in slow writing and great
    // memory usage. The latter is likely because there's a memory leak
    // somewhere or the bookkeeping for small tiles takes too much memory.
    // This is a bugfix.
    Vector2 orig_tile_size = opt.raster_tile_size;
    opt.raster_tile_size = Vector2(1024, 1024);

    {
      // Right image mask transformed into left coordinates. Keep this in its
      // own scope to free up the memory of its bookkeeping as soon as
      // possible.
      ImageViewRef<PixelMask<uint8>> warped_right_mask
        = crop(vw::cartography::geo_transform
              (right_mask, right_georef, left_georef,
                ConstantEdgeExtension(), NearestPixelInterpolation()),
              bounding_box(left_mask));
      vw::cartography::block_write_gdal_image(left_mask_file,
                               apply_mask(intersect_mask(left_mask, warped_right_mask)),
                               has_left_georef, left_georef,
                               has_nodata, output_nodata,
                               opt, TerminalProgressCallback("asp", "\t    Mask L: "));
    }
    {
      // Left image mask transformed into right coordinates. Keep it in its
      // own scope as per above.
      ImageViewRef<PixelMask<uint8>> warped_left_mask
        = crop(vw::cartography::geo_transform
              (left_mask, left_georef, right_georef,
                ConstantEdgeExtension(), NearestPixelInterpolation()),
              bounding_box(right_mask));
      vw::cartography::block_write_gdal_image(right_mask_file,
                                  apply_mask(intersect_mask(right_mask, warped_left_mask)),
                                  has_right_georef, right_georef,
                                  has_nodata, output_nodata,
                                  opt, TerminalProgressCallback("asp", "\t    Mask R: "));
    }

    // Put the original tile size back
    opt.raster_tile_size = orig_tile_size;

  } else {
    // No DEM to map-project to.
    // TODO: Even so, the trick above with intersecting the masks will still work,
    // if the images are map-projected (such as with cam2map-ed cubes),
    // but this would require careful research.
    vw::cartography::block_write_gdal_image(left_mask_file, apply_mask(left_mask),
                                 has_left_georef, left_georef,
                                 has_nodata, output_nodata,
                                 opt, TerminalProgressCallback("asp", "\t Mask L: "));
    vw::cartography::block_write_gdal_image(right_mask_file, apply_mask(right_mask),
                                 has_right_georef, right_georef,
                                 has_nodata, output_nodata,
                                 opt, TerminalProgressCallback("asp", "\t Mask R: "));
  }

  sw.stop();
  vw_out(DebugMessage, "asp") << "Mask creation elapsed time: "
                              << sw.elapsed_seconds() << " s." << std::endl;
}

/// The main preprocessing function
void stereo_preprocessing(bool adjust_left_image_size, ASPGlobalOptions& opt) {

  // Normalize the images, or create symlinks if the user chose to skip
  // normalization. The preprocessing_hook handles both cases now.
  std::string left_image_file, right_image_file;
  bool skip_img_norm = asp::skip_image_normalization(opt);

  // Bathymetry will not work with skipping image normalization.
  if (skip_img_norm && asp::doBathy(asp::stereo_settings()))
    vw_throw(ArgumentErr()
             << "\nCannot do bathymetry when skipping image normalization.\n");

  // Perform image normalization (or create symlinks and compute stats if skipping)
  opt.session->preprocessing_hook(adjust_left_image_size,
                                  opt.in_file1, opt.in_file2,
                                  left_image_file, right_image_file);

  boost::shared_ptr<DiskImageResource>
    left_rsrc (vw::DiskImageResourcePtr(left_image_file)),
    right_rsrc(vw::DiskImageResourcePtr(right_image_file));

  // Load the normalized images
  DiskImageView<PixelGray<float>> left_image (left_rsrc), right_image(right_rsrc);

  // If we crop the images, we must always rebuild the masks
  // and subsample the images and masks.
  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  std::string left_mask_file  = opt.out_prefix+"-lMask.tif";
  std::string right_mask_file = opt.out_prefix+"-rMask.tif";
  
  // Also need to rebuild if the inputs changed after the mask files were produced.
  std::vector<std::string> in_file_list;
  in_file_list.push_back(opt.in_file1);
  in_file_list.push_back(opt.in_file2);
  in_file_list.push_back(opt.cam_file1);
  in_file_list.push_back(opt.cam_file2);
  bool inputs_changed = (!first_is_newer(left_mask_file,  in_file_list) ||
                         !first_is_newer(right_mask_file, in_file_list));

  bool rebuild = crop_left || crop_right || inputs_changed;
  try {
    // If files do not exist, create them. Also if they exist
    // but are invalid. The second check gives an ugly verbose
    // message, hence first check for existence with boost.
    if (!fs::exists(left_mask_file) || !fs::exists(right_mask_file)){
      rebuild = true;
    }else{
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelGray<uint8>> testa(left_mask_file );
      DiskImageView<PixelGray<uint8>> testb(right_mask_file);
      vw_settings().reload_config();
    }
  } catch (vw::IOErr const& e) {
    vw_settings().reload_config();
    rebuild = true;
  } catch (vw::ArgumentErr const& e ) {
    // Throws on a corrupted file.
    vw_settings().reload_config();
    rebuild = true;
  }

  cartography::GeoReference left_georef, right_georef;
  bool has_left_georef  = read_georeference(left_georef,  left_image_file);
  bool has_right_georef = read_georeference(right_georef, right_image_file);

  // The output no-data value must be < 0 as the images are scaled to around [0, 1].
  bool  has_nodata    = true;
  float output_nodata = -32768.0;

  // Create or reuse masks
  if (!rebuild) {
    vw_out() << "\t--> Using cached masks.\n";
    // This is to safeguard against the case when masks produced with alignment
    // or crop from a previous run are used with no alignment or no crop. 
    if (stereo_settings().alignment_method == "none") {
      check_image_sizes(opt.in_file1, left_mask_file);
      check_image_sizes(opt.in_file2, right_mask_file);
    }
  } else {
    createImageMasks(opt, left_rsrc, right_rsrc, left_image, right_image,
                     skip_img_norm, has_left_georef, has_right_georef,
                     left_georef, right_georef, has_nodata, output_nodata,
                     left_mask_file, right_mask_file);
  }

  // Create subsampled images and masks. Skip in distributed stereo mode.
  if (!stereo_settings().stereo_dist_mode)
    createSubsampledImages(opt, in_file_list, crop_left, crop_right,
                           left_image, right_image,
                           left_mask_file, right_mask_file,
                           has_left_georef, has_right_georef,
                           left_georef, right_georef,
                           output_nodata);

  // When alignment method is none or epipolar, no ip were created so far, so
  // produce them now.
  if (stereo_settings().alignment_method == "none" ||
      stereo_settings().alignment_method == "epipolar") {
    try {
        asp::compute_ip_LR(opt.out_prefix);
      } catch (const std::exception& e) {
        if (!asp::stereo_settings().search_range.empty() ||
            stereo_settings().seed_mode == 2) {
          // If the user provided a search range or disp from DEM, IP are not needed.
          vw_out() << "Could not compute interest points. The error was:\n";
          vw_out() << e.what();
          if (!asp::stereo_settings().search_range.empty())
            vw_out() << "Will continue, given that --corr-search was set.\n";
          if (stereo_settings().seed_mode == 2)
            vw_out() << "Will continue, given the option --seed-mode 2.\n";
      } else {
        vw::vw_throw(vw::ArgumentErr() << e.what() << "\n");
      }
    }
  }
    
} // End function stereo_preprocessing

int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
  
    vw_out() << "\n[ " << current_posix_time_string() << " ]: Stage 0 --> PREPROCESSING\n";

    stereo_register_sessions();

    bool verbose = false;
    std::vector<ASPGlobalOptions> opt_vec;
    std::string output_prefix;
    asp::parseStereoArgs(argc, argv, PreProcessingDescription(),
                         verbose, output_prefix, opt_vec);
    ASPGlobalOptions opt = opt_vec[0];

    vw_out() << "Image files:  " << opt.in_file1  << ", " << opt.in_file2  << "\n";
    if (opt.cam_file1 != "" || opt.cam_file2 != "") 
      vw_out() << "Camera files: " << opt.cam_file1 << ", " << opt.cam_file2 << "\n";
    if (!opt.input_dem.empty())
      vw_out() << "Input DEM: " << opt.input_dem << "\n";

    // We will not adjust the left image size if we do multiview stereo,
    // so we can keep one-to-one correspondence between the several
    // pairwise runs that are part of the multiview run for the time
    // when we need to combine all these runs to do simultaneous triangulation.
    bool adjust_left_image_size = (opt_vec.size() == 1 &&
                                   !stereo_settings().part_of_multiview_run);

    stereo_preprocessing(adjust_left_image_size, opt);
    if (!stereo_settings().stereo_dist_mode)
      asp::estimate_convergence_angle(opt);

    vw_out() << "\n[ " << current_posix_time_string() << " ]: PREPROCESSING FINISHED\n";

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
