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
#include <vw/Image/AntiAliasing.h>
#include <vw/Image/BlobIndex.h>
#include <vw/Image/InpaintView.h>
#include <vw/Image/WindowAlgorithms.h>
#include <vw/Image/UtilityViews.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/InterestPoint/Matcher.h>
#include <asp/Core/IpMatchingAlgs.h>        // Lightweight header
#include <asp/Sessions/CameraUtils.h>
#include <vw/Math/Functors.h>
#include <asp/Tools/stereo.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <xercesc/util/PlatformUtils.hpp>
#include <asp/Core/ThreadedEdgeMask.h>

using namespace vw;
using namespace asp;

// Invalidate pixels < threshold
struct MaskAboveThreshold: public ReturnFixedType<PixelMask<uint8>> {
  double m_threshold;
  MaskAboveThreshold(double threshold): m_threshold(threshold){}
  PixelMask<uint8> operator() (PixelGray<float> const& pix) const {
    if (pix >= m_threshold)
      return PixelMask<uint8>(255);
    else
      return PixelMask<uint8>();
  }
};
template <class ImageT>
UnaryPerPixelView<ImageT, MaskAboveThreshold>
inline mask_above_threshold(ImageViewBase<ImageT> const& image, double threshold) {
  return UnaryPerPixelView<ImageT, MaskAboveThreshold>(image.impl(),
                                                        MaskAboveThreshold(threshold));
}

struct BlobHolder {
  // This object will ensure that the current BlobIndexThreaded object
  // is not de-allocated while still being used to fill holes in a
  // given mask.
  boost::shared_ptr<vw::BlobIndexThreaded> m_blobPtr;

  // Member function which does the hole-filling.
  ImageViewRef<PixelMask<uint8>> 
  mask_and_fill_holes(ImageViewRef<PixelGray<float>> const& img, double threshold);
};

/// Create the mask of pixels above threshold. Fix any holes in it.
ImageViewRef<PixelMask<uint8>>
BlobHolder::mask_and_fill_holes(ImageViewRef<PixelGray<float>> const& img,
                                 double threshold) {

  ImageViewRef<PixelMask<uint8>> thresh_mask = mask_above_threshold(img, threshold);
  int max_area = 0; // fill arbitrarily big holes
  bool use_grassfire = false; // fill with default value
  PixelMask<uint8> default_inpaint_val = uint8(255);

  m_blobPtr = boost::shared_ptr<BlobIndexThreaded>
    (new BlobIndexThreaded( invert_mask( thresh_mask.impl() ), max_area ));
  return inpaint(thresh_mask.impl(), *m_blobPtr.get(), use_grassfire, default_inpaint_val);
}


/// Instead of writing L.tif and R.tif, just create sym links from
/// input left and right images. Creating symbolic links can be tricky.
void create_sym_links(std::string const& left_input_file,
                      std::string const& right_input_file,
                      std::string const& out_prefix,
                      std::string & left_output_file,
                      std::string & right_output_file) {

  vw_out(WarningMessage) << "Skipping image normalization.\n";

  left_output_file  = out_prefix+"-L.tif";
  right_output_file = out_prefix+"-R.tif";
  std::string cmd1, cmd2;
  fs::path out_prefix_path(out_prefix);
  fs::path left_rel_in, right_rel_in;
  if (out_prefix_path.has_parent_path()) {
    fs::path rundir = out_prefix_path.parent_path();
    cmd1 = "cd " + rundir.string() + "; ";
    cmd2 = "; cd " +  fs::canonical(".").string();
    left_rel_in  = make_file_relative_to_dir(fs::path(left_input_file),  rundir);
    right_rel_in = make_file_relative_to_dir(fs::path(right_input_file), rundir);
  }else{
    left_rel_in  = fs::path(left_input_file);
    right_rel_in = fs::path(right_input_file);
  }

  std::string left_rel_out  = fs::path(left_output_file ).filename().string();
  std::string right_rel_out = fs::path(right_output_file).filename().string();

  std::string cmd;
  if (!fs::exists(left_output_file)){
    cmd = cmd1 + "ln -s " + left_rel_in.string() + " " + left_rel_out + cmd2;
    vw_out() << cmd << std::endl;
    int ret = system(cmd.c_str());
    VW_ASSERT( ret == 0,
               ArgumentErr() << "Failed to execute: " << cmd << "\n" );
  }
  if (!fs::exists(right_output_file)){
    cmd = cmd1 + "ln -s " + right_rel_in.string() + " " + right_rel_out + cmd2;
    vw_out() << cmd << std::endl;
    int ret = system(cmd.c_str());
    VW_ASSERT( ret == 0,
               ArgumentErr() << "Failed to execute: " << cmd << "\n" );
  }
} // End function create_sym_links

// Check that the images in file1 and file2 have same size, and throw
// an exception if they don't.
void check_image_sizes(std::string const& file1, std::string const& file2) {
  DiskImageView<float> left_image (file1), right_image(file2);
  if (left_image.cols() != right_image.cols() ||
      left_image.rows() != right_image.rows())
    vw_throw( ArgumentErr() << "Expecting images: " << file1 << " and " << file2
              << " to have the same dimensions. Delete this run and start all over.\n");
}

/// The main preprocessing function
void stereo_preprocessing(bool adjust_left_image_size, ASPGlobalOptions& opt) {

  // Normalize the images, or create symlinks to the original
  // images if the user chose not to normalize.
  std::string left_image_file, right_image_file;
  bool skip_img_norm = asp::skip_image_normalization(opt);

  // Bathymetry will not work with skipping image normalization.
  // It could be made to work, but it is an obscure scenario not
  // worth testing.
  if (skip_img_norm && opt.session->do_bathymetry()) 
    vw_throw( ArgumentErr() 
             << "\nCannot do bathymetry when skipping image normalization.\n");

  // Need to also write the transformed bathy masks to disk, those
  // will be used in stereo_tri.
  if (skip_img_norm)
    create_sym_links(opt.in_file1, opt.in_file2, opt.out_prefix,
                     left_image_file, right_image_file);
  else // Perform image normalization
    opt.session->preprocessing_hook(adjust_left_image_size,
                                    opt.in_file1,    opt.in_file2,
                                    left_image_file, right_image_file);

  boost::shared_ptr<DiskImageResource>
    left_rsrc (vw::DiskImageResourcePtr(left_image_file)),
    right_rsrc(vw::DiskImageResourcePtr(right_image_file));

  // Load the normalized images.
  DiskImageView<PixelGray<float>> left_image (left_rsrc),
                                  right_image(right_rsrc);

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
  bool inputs_changed = (!is_latest_timestamp(left_mask_file,  in_file_list) ||
                         !is_latest_timestamp(right_mask_file, in_file_list));

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

  if (!rebuild) {
    vw_out() << "\t--> Using cached masks.\n";
    
    // This is to safeguard against the case when masks produced with alignment
    // or crop from a previous run are used with no alignment or no crop. 
    if (stereo_settings().alignment_method == "none") {
      check_image_sizes(opt.in_file1, left_mask_file);
      check_image_sizes(opt.in_file2, right_mask_file);
    }
  }else{

    vw_out() << "\t--> Generating image masks.\n";

    Stopwatch sw;
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
    
    // TODO(oalexan1): Remove the ThreadedEdgeMask logic, as it is slow. 
    // But the alternative outlined below is not doing the same thing when 
    // it comes to mapprojected images. 
#if 1
    ImageViewRef<PixelMask<uint8>> left_mask
      = copy_mask(constant_view(uint8(255),
                                left_image.cols(), left_image.rows()),
                  asp::threaded_edge_mask(left_image,0,0,1024));
    ImageViewRef<PixelMask<uint8>> right_mask
      = copy_mask(constant_view(uint8(255),
                                right_image.cols(), right_image.rows()),
                  asp::threaded_edge_mask(right_image,0,0,1024));

    // Mask no-data pixels.
    left_mask = intersect_mask(left_mask,
                               create_mask_less_or_equal(left_image,
                                                         left_nodata_value));
    right_mask = intersect_mask(right_mask,
                                create_mask_less_or_equal(right_image,
                                                          right_nodata_value));
#else
    // Mask no-data pixels
    ImageViewRef<PixelMask<uint8>> left_mask 
       = copy_mask(constant_view(uint8(255), left_image.cols(), left_image.rows()),
                                 create_mask_less_or_equal(left_image,
                                                         left_nodata_value));
    ImageViewRef<PixelMask<uint8>> right_mask 
        = copy_mask(constant_view(uint8(255), right_image.cols(), right_image.rows()),
                                  create_mask_less_or_equal(right_image,
                                                          right_nodata_value));
#endif
      
    // Invalidate pixels below (normalized) threshold. This is experimental.
    double left_threshold  = std::numeric_limits<double>::quiet_NaN();
    double right_threshold = std::numeric_limits<double>::quiet_NaN();
    double nodata_fraction = stereo_settings().nodata_pixel_percentage/100.0;
    if (skip_img_norm && !std::isnan(nodata_fraction))
      vw::vw_throw(vw::ArgumentErr() 
               << "Cannot skip image normalization while attempting "
               << "to apply a normalized threshold.\n");

    if (!std::isnan(nodata_fraction)) {
      // Declare a fixed proportion of low-value pixels to be no-data.
      math::CDFAccumulator<PixelGray<float>> left_cdf (1024, 1024),
                                               right_cdf(1024, 1024);
      for_each_pixel(left_image,  left_cdf );
      for_each_pixel(right_image, right_cdf);
      left_threshold  = left_cdf.quantile (nodata_fraction);
      right_threshold = right_cdf.quantile(nodata_fraction);
    }

    // The blob holders must not go out of scope while masks are being written.
    BlobHolder LB, RB;
    if ( !std::isnan(left_threshold) && !std::isnan(right_threshold)) {
      ImageViewRef<PixelMask<uint8>> left_thresh_mask
        = LB.mask_and_fill_holes(left_image,  left_threshold);
      ImageViewRef<PixelMask<uint8>> right_thresh_mask
        = RB.mask_and_fill_holes(right_image, right_threshold);
      left_mask  = intersect_mask(left_mask,  left_thresh_mask );
      right_mask = intersect_mask(right_mask, right_thresh_mask);
    }

    // Mask out regions with low input pixel value standard deviations if the
    // user requested it.
    if (stereo_settings().nodata_stddev_kernel > 0) {
      Vector2i stddev_kernel(stereo_settings().nodata_stddev_kernel, stereo_settings().nodata_stddev_kernel);

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
    if (has_left_georef && has_right_georef && !opt.input_dem.empty()) {
      
      // Write with big blocks. Small blocks results in slow writing and great
      // memory usage. The latter is likely because there's a memory leak
      // somewhere or the bookkeeping for small tiles takes too much memory.
      // This is a bugfix.
      Vector2 orig_tile_size = opt.raster_tile_size;
      opt.raster_tile_size = Vector2(1024, 1024);
      
      { 
        // Right image mask transformed into left coordinates
        // Keep this in its own scope to free up the memory of its bookkeeping
        // as soon as possible.
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
                                   opt, TerminalProgressCallback("asp", "\t Mask L: ") );
      vw::cartography::block_write_gdal_image( right_mask_file, apply_mask(right_mask),
                                   has_right_georef, right_georef,
                                   has_nodata, output_nodata,
                                   opt, TerminalProgressCallback("asp", "\t Mask R: ") );
    }

    sw.stop();
    vw_out(DebugMessage,"asp") << "Mask creation elapsed time: "
                               << sw.elapsed_seconds() << " s." << std::endl;
  } // End creating masks

  std::string lsub  = opt.out_prefix+"-L_sub.tif";
  std::string rsub  = opt.out_prefix+"-R_sub.tif";
  std::string lmsub = opt.out_prefix+"-lMask_sub.tif";
  std::string rmsub = opt.out_prefix+"-rMask_sub.tif";

  inputs_changed = (!is_latest_timestamp(lsub,  in_file_list ) ||
                    !is_latest_timestamp(rsub,  in_file_list)  ||
                    !is_latest_timestamp(lmsub, in_file_list ) ||
                    !is_latest_timestamp(rmsub, in_file_list )  );

  // We must always redo the subsampling if we are allowed to crop the images
  rebuild = crop_left || crop_right || inputs_changed;

  try {
    // First try to see if the subsampled images exist.
    if (!fs::exists(lsub)  || !fs::exists(rsub) ||
        !fs::exists(lmsub) || !fs::exists(rmsub)){
      rebuild = true;
    }else{
      // This confusing try catch is to see if the subsampled images actually have content.
      DiskImageView<PixelGray<float>> testl (lsub );
      DiskImageView<PixelGray<float>> testr (rsub );
      DiskImageView<uint8>             testlm(lmsub);
      DiskImageView<uint8>             testrm(rmsub);
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
    vw_out() << "\t--> Creating previews. Subsampling by " << sub_scale
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
      left_sub_image  = block_rasterize(resample(copy_mask(left_image,  create_mask(left_mask)),
                                                 sub_scale), 
                                        sub_tile_size_vec, sub_threads);
      right_sub_image = block_rasterize(resample(copy_mask(right_image, create_mask(right_mask)),
                                                 sub_scale), 
                                        sub_tile_size_vec, sub_threads);
    } else {
      // When we heavily reduce the image size, super sampling seems
      // like the best approach. The method below should be equivalent.
      left_sub_image
        = block_rasterize
        (cache_tile_aware_render(resample_aa(copy_mask(left_image,create_mask(left_mask)),
                                             sub_scale),
                                 Vector2i(256,256) * sub_scale),
         sub_tile_size_vec, sub_threads);
      right_sub_image
        = block_rasterize
        (cache_tile_aware_render(resample_aa(copy_mask(right_image,create_mask(right_mask)),
                                             sub_scale),
                                 Vector2i(256,256) * sub_scale),
         sub_tile_size_vec, sub_threads);
    }

    // Enforce no predictor in compression, it works badly with sub-images
    vw::GdalWriteOptions opt_nopred = opt;
    opt_nopred.gdal_options["PREDICTOR"] = "1";

    vw::cartography::GeoReference left_sub_georef, right_sub_georef;
    if (has_left_georef) {
      // Account for scale.
      double left_scale = 0.5*( double(left_sub_image.cols())/left_image.cols()
                              + double(left_sub_image.rows())/left_image.rows());
      left_sub_georef = resample(left_georef, left_scale);
    }
    if (has_right_georef) {
      // Account for scale.
      double right_scale = 0.5*( double(right_sub_image.cols())/right_image.cols()
                               + double(right_sub_image.rows())/right_image.rows());
      right_sub_georef = resample(right_georef, right_scale);
    }

    vw::cartography::block_write_gdal_image
      ( lsub, apply_mask(left_sub_image, output_nodata),
        has_left_georef, left_sub_georef,
        has_nodata, output_nodata,
        opt_nopred, TerminalProgressCallback("asp", "\t    Sub L: ") );
    vw::cartography::block_write_gdal_image
      ( rsub, apply_mask(right_sub_image, output_nodata),
        has_right_georef, right_sub_georef,
        has_nodata, output_nodata,
        opt_nopred, TerminalProgressCallback("asp", "\t    Sub R: ") );
    vw::cartography::block_write_gdal_image
      ( lmsub,
        channel_cast_rescale<uint8>(select_channel(left_sub_image, 1)),
        has_left_georef, left_sub_georef,
        has_nodata, output_nodata,
        opt_nopred, TerminalProgressCallback("asp", "\t    Sub L Mask: ") );
    vw::cartography::block_write_gdal_image
      ( rmsub,
        channel_cast_rescale<uint8>(select_channel(right_sub_image, 1)),
        has_right_georef, right_sub_georef,
        has_nodata, output_nodata,
        opt_nopred, TerminalProgressCallback("asp", "\t    Sub R Mask: ") );
  } // End try/catch to see if the subsampled images have content

  if (skip_img_norm && stereo_settings().subpixel_mode == 2){
    // If image normalization is not done, we still need to compute the image
    // stats, to do normalization on the fly in stereo_rfne.
    // This code is not in stereo_rfne, as that one is meant to be distributed
    // across multiple machines, so we want the stats to be computed just once,
    // hence they are done here.
    // TODO(oalexan1): Must integrate this in shared_preprocessing_hook,
    // as this repeats a lot of logic from there
    vw_out() << "Computing statistics for the un-normalized images.\n";
    float left_no_data_value, right_no_data_value;
    asp::get_nodata_values(left_rsrc, right_rsrc,
                           left_no_data_value, right_no_data_value); 
    if (!std::isnan(stereo_settings().nodata_value)){
      left_no_data_value  = stereo_settings().nodata_value;
      right_no_data_value = stereo_settings().nodata_value;
    }

    ImageViewRef<PixelMask<PixelGray<float>>> left_masked_image
      = create_mask_less_or_equal(left_image, left_no_data_value);
    ImageViewRef<PixelMask<PixelGray<float>>> right_masked_image
      = create_mask_less_or_equal(right_image, right_no_data_value); 

    Vector6f left_stats  = gather_stats(pixel_cast<PixelMask<float>>(left_masked_image), 
                                        "left",
                                        opt.out_prefix, left_image_file);
    Vector6f right_stats = gather_stats(pixel_cast<PixelMask<float>>(right_masked_image), 
                                        "right",
                                        opt.out_prefix, right_image_file);
    std::string   left_stats_file  = opt.out_prefix + "-lStats.tif";
    std::string   right_stats_file = opt.out_prefix + "-rStats.tif";

    vw_out() << "Writing: " << left_stats_file << ' ' << right_stats_file << std::endl;
    Vector<float32> left_stats2  = left_stats;  // cast
    Vector<float32> right_stats2 = right_stats; // cast
    write_vector(left_stats_file,  left_stats2 );
    write_vector(right_stats_file, right_stats2);
  }

  // When alignment method is none or epipolar, no ip were created so far, so
  // produce them now.
  if (stereo_settings().alignment_method == "none" ||
      stereo_settings().alignment_method == "epipolar")
    asp::compute_ip_LR(opt.out_prefix);
    
} // End function stereo_preprocessing

// Find the median angle in degrees at which rays emanating from
// matching points meet
void estimate_convergence_angle(ASPGlobalOptions const& opt) {

  if (stereo_settings().correlator_mode)
    return; // No camera can be assumed, hence no convergence angle.

  // When having matches between L and R, need to do things a bit differently.
  bool have_aligned_matches = (stereo_settings().alignment_method == "none" ||
                               stereo_settings().alignment_method == "epipolar");

  std::string match_filename;
  if (have_aligned_matches)
    match_filename = vw::ip::match_filename(opt.out_prefix, "L.tif", "R.tif");
  else 
    match_filename = opt.session->stereo_match_filename(opt.session->left_cropped_image(),
                                                        opt.session->right_cropped_image(),
                                                        opt.out_prefix);
  // The interest points must exist by now
  if (!fs::exists(match_filename))
    vw_throw(ArgumentErr() << "Missing IP matches file: " << match_filename);
  
  std::vector<ip::InterestPoint> left_ip, right_ip;
  ip::read_binary_match_file(match_filename, left_ip, right_ip);

  if (have_aligned_matches) {
    // Unalign the interest point matches
    std::vector<vw::ip::InterestPoint> unaligned_left_ip, unaligned_right_ip;
    asp::unalign_ip(opt.session->tx_left(), opt.session->tx_right(),
                    left_ip, right_ip, unaligned_left_ip, unaligned_right_ip);
    left_ip  = unaligned_left_ip;
    right_ip = unaligned_right_ip;
  }

  std::vector<double> sorted_angles;
  boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
  opt.session->camera_models(left_cam, right_cam);
  asp::convergence_angles(left_cam.get(), right_cam.get(), left_ip, right_ip, sorted_angles);

  if (sorted_angles.empty()) {
    vw_out() << "No convergence angles calculated.\n";
    return;
  }
  
  int len = sorted_angles.size();
  vw_out() << "Convergence angle percentiles (in degrees) based on interest point matches:\n";
  vw_out() << "\t"
           << "25% " << sorted_angles[0.25*len] << ", "
           << "50% " << sorted_angles[0.50*len] << ", "
           << "75% " << sorted_angles[0.75*len] << ".\n";
}

int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
  
    vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 0 --> PREPROCESSING \n";

    stereo_register_sessions();

    bool verbose = false;
    std::vector<ASPGlobalOptions> opt_vec;
    std::string output_prefix;
    asp::parse_multiview(argc, argv, PreProcessingDescription(),
                         verbose, output_prefix, opt_vec);
    ASPGlobalOptions opt = opt_vec[0];

    vw_out() << "Using image files:  " << opt.in_file1  << ", " << opt.in_file2  << std::endl;
    if (opt.cam_file1 != "" || opt.cam_file2 != "") 
      vw_out() << "Using camera files: " << opt.cam_file1 << ", " << opt.cam_file2 << std::endl;
    if (!opt.input_dem.empty())
      vw_out() << "Using input DEM: " << opt.input_dem << std::endl;

    // We will not adjust the left image size if we do multiview stereo,
    // so we can keep one-to-one correspondence between the several
    // pairwise runs that are part of the multiview run for the time
    // when we need to combine all these runs to do simultaneous triangulation.
    bool adjust_left_image_size = (opt_vec.size() == 1 &&
                                   !stereo_settings().part_of_multiview_run);

    stereo_preprocessing(adjust_left_image_size, opt);
    
    // TODO(oalexan1): Must test with crop win!
    estimate_convergence_angle(opt);
    
    vw_out() << "\n[ " << current_posix_time_string() << " ] : PREPROCESSING FINISHED \n";

     xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
