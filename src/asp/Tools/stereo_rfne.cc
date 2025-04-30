// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file stereo_rfne.cc
///

#include <asp/Tools/stereo.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Core/Macros.h>
#include <asp/Core/ImageNormalization.h>

#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/ParabolaSubpixelView.h>
#include <vw/Stereo/SubpixelView.h>
#include <vw/Stereo/EMSubpixelCorrelatorView.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageResourceOpenEXR.h>
#include <vw/Image/InpaintView.h>
#include <vw/FileIO/DiskImageUtils.h>

#include <xercesc/util/PlatformUtils.hpp>

using namespace vw;
using namespace vw::stereo;
using namespace asp;

template <class Image1T, class Image2T>
ImageViewRef<PixelMask<Vector2f>>
refine_disparity(Image1T const& left_image,
                 Image2T const& right_image,
                 ImageViewRef<PixelMask<Vector2f>> const& integer_disp,
                 ASPGlobalOptions const& opt, bool verbose) {

  ImageViewRef<PixelMask<Vector2f>> refined_disp = integer_disp;

  PrefilterModeType prefilter_mode =
    static_cast<vw::stereo::PrefilterModeType>(stereo_settings().pre_filter_mode);

  if ((stereo_settings().subpixel_mode == 0) ||
      (stereo_settings().subpixel_mode > 6)) {
    // Do nothing (includes SGM specific subpixel modes)
    if (verbose)
      vw_out() << "\t--> Skipping subpixel mode.\n";
  } else {
    if (verbose) {
      if (stereo_settings().pre_filter_mode == 2)
        vw_out() << "\t--> Using LOG pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
      else if (stereo_settings().pre_filter_mode == 1)
        vw_out() << "\t--> Using Subtracted Mean pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
      else
        vw_out() << "\t--> NO preprocessing.\n";
    }
  }

  if (stereo_settings().subpixel_mode == 1) {
    // Parabola
    if (verbose)
      vw_out() << "\t--> Using parabola subpixel mode.\n";

    refined_disp = parabola_subpixel(integer_disp,
                                     left_image, right_image,
                                     prefilter_mode, stereo_settings().slogW,
                                     stereo_settings().subpixel_kernel);

  } else if (stereo_settings().subpixel_mode == 2) {
    // Bayes EM
    if (verbose)
      vw_out() << "\t--> Using affine adaptive subpixel mode\n";

    refined_disp =
      bayes_em_subpixel(integer_disp,
                         left_image, right_image,
                         prefilter_mode, stereo_settings().slogW,
                         stereo_settings().subpixel_kernel,
                         stereo_settings().subpixel_max_levels);

  } else if (stereo_settings().subpixel_mode == 3) {
    // Fast affine
    if (verbose)
      vw_out() << "\t--> Using affine subpixel mode\n";
    refined_disp =
      affine_subpixel(integer_disp,
                      left_image, right_image,
                      prefilter_mode, stereo_settings().slogW,
                      stereo_settings().subpixel_kernel,
                      stereo_settings().subpixel_max_levels);

  } else if (stereo_settings().subpixel_mode == 4) {
    // Phase correlation
    if (verbose) {
      vw_out() << "\t--> Using Phase Correlation subpixel mode\n";
      vw_out() << "\t--> Forcing subpixel pyramid levels to zero\n";
    }
    // So far phase correlation has worked poorly with multiple levels.
    stereo_settings().subpixel_max_levels = 0;

    refined_disp =
      phase_subpixel(integer_disp,
                      left_image, right_image,
                      prefilter_mode, stereo_settings().slogW,
                      stereo_settings().subpixel_kernel,
                      stereo_settings().subpixel_max_levels,
                      stereo_settings().phase_subpixel_accuracy);

  } else if (stereo_settings().subpixel_mode == 5) {
    // Lucas-Kanade
    if (verbose)
      vw_out() << "\t--> Using Lucas-Kanade subpixel mode\n";

    refined_disp =
      lk_subpixel(integer_disp,
                   left_image, right_image,
                   prefilter_mode, stereo_settings().slogW,
                   stereo_settings().subpixel_kernel,
                   stereo_settings().subpixel_max_levels);

  } else if (stereo_settings().subpixel_mode == 6) {
    
    // This no longer works. Support for .exr files will be removed anyway.
    vw::vw_throw(NoImplErr() << "Subpixel mode 6 support has been removed.\n");
#if 0    
    // Affine and Bayes subpixel refinement always use the LogPreprocessingFilter.
    if (verbose) {
      vw_out() << "\t--> Using EM Subpixel mode "
               << stereo_settings().subpixel_mode << "\n";
      vw_out() << "\t--> Mode 3 does internal preprocessing;"
               << " settings will be ignored.\n";
    }

    typedef stereo::EMSubpixelCorrelatorView<float32> EMCorrelator;
    EMCorrelator em_correlator(channels_to_planes(left_image),
                               channels_to_planes(right_image),
                               pixel_cast<PixelMask<Vector2f>>(integer_disp), -1);
    em_correlator.set_em_iter_max   (stereo_settings().subpixel_em_iter);
    em_correlator.set_inner_iter_max(stereo_settings().subpixel_affine_iter);
    em_correlator.set_kernel_size   (stereo_settings().subpixel_kernel);
    em_correlator.set_pyramid_levels(stereo_settings().subpixel_pyramid_levels);

    DiskImageResourceOpenEXR em_disparity_map_rsrc(opt.out_prefix + "-F6.exr",
                                                   em_correlator.format());

    block_write_image(em_disparity_map_rsrc, em_correlator,
                      TerminalProgressCallback("asp", "\t--> EM Refinement :"));

    DiskImageResource *em_disparity_map_rsrc_2 =
      DiskImageResourceOpenEXR::construct_open(opt.out_prefix + "-F6.exr");
    DiskImageView<PixelMask<Vector<float, 5>>> em_disparity_disk_image(em_disparity_map_rsrc_2);

    ImageViewRef<Vector<float, 3>> disparity_uncertainty =
      per_pixel_filter(em_disparity_disk_image,
                       EMCorrelator::ExtractUncertaintyFunctor());
    ImageViewRef<float> spectral_uncertainty =
      per_pixel_filter(disparity_uncertainty,
                       EMCorrelator::SpectralRadiusUncertaintyFunctor());
    write_image(opt.out_prefix+"-US.tif", spectral_uncertainty);
    write_image(opt.out_prefix+"-U.tif", disparity_uncertainty);

    refined_disp =
      per_pixel_filter(em_disparity_disk_image,
                       EMCorrelator::ExtractDisparityFunctor());
#endif      
  } // End of subpixel mode selection

  if ((stereo_settings().subpixel_mode < 0) || (stereo_settings().subpixel_mode > 5)) {
    if (verbose) {
      vw_out() << "\t--> Invalid subpixel mode selection: "
               << stereo_settings().subpixel_mode << "\n";
      vw_out() << "\t--> Doing nothing\n";
    }
  }

  return refined_disp;
}

// Perform refinement in each tile. If using local homography,
// apply the local homography transform for the given tile
// to the right image before doing refinement in that tile.
template <class Image1T, class Image2T, class SeedDispT>
class PerTileRfne: public ImageViewBase<PerTileRfne<Image1T, Image2T, SeedDispT>> {
  Image1T              m_left_image;
  Image2T              m_right_image;
  SeedDispT            m_integer_disp;
  SeedDispT            m_sub_disp;
  ASPGlobalOptions const&       m_opt;
  Vector2              m_upscale_factor;

public:
  PerTileRfne(ImageViewBase<Image1T>    const& left_image,
               ImageViewBase<Image2T>   const& right_image,
               ImageViewBase<SeedDispT> const& integer_disp,
               ImageViewBase<SeedDispT> const& sub_disp,
               ASPGlobalOptions         const& opt):
    m_left_image(left_image.impl()), m_right_image(right_image.impl()),
    m_integer_disp(integer_disp.impl()), m_sub_disp(sub_disp.impl()),
    m_opt(opt) {

    m_upscale_factor = Vector2(double(m_left_image.impl().cols()) / m_sub_disp.cols(),
                               double(m_left_image.impl().rows()) / m_sub_disp.rows());
  }

  // Image View interface
  typedef PixelMask<Vector2f>                  pixel_type;
  typedef pixel_type                           result_type;
  typedef ProceduralPixelAccessor<PerTileRfne> pixel_accessor;

  inline int32 cols  () const { return m_left_image.cols(); }
  inline int32 rows  () const { return m_left_image.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double /*i*/, double /*j*/, int32 /*p*/ = 0) const {
    vw_throw(NoImplErr() << "PerTileRfne::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {
    ImageView<pixel_type> tile_disparity;
    bool verbose = false;
    tile_disparity = crop(refine_disparity(m_left_image, m_right_image,
                                           m_integer_disp, m_opt, verbose), bbox);

    prerasterize_type disparity = prerasterize_type(tile_disparity,
                                                    -bbox.min().x(), -bbox.min().y(),
                                                    cols(), rows());
    return disparity;
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class Image1T, class Image2T, class SeedDispT>
PerTileRfne<Image1T, Image2T, SeedDispT>
per_tile_rfne(ImageViewBase<Image1T>   const& left,
              ImageViewBase<Image2T>   const& right,
              ImageViewBase<SeedDispT> const& integer_disp,
              ImageViewBase<SeedDispT> const& sub_disp,
              ASPGlobalOptions         const& opt) {
  typedef PerTileRfne<Image1T, Image2T, SeedDispT> return_type;
  return return_type(left.impl(), right.impl(), integer_disp.impl(), sub_disp.impl(), opt);
}

// Enable conversion from images with float pixels to images with
// PixelGray<float> pixels.
class GrayCast: public ReturnFixedType<PixelGray<float>> {
public:
  PixelGray<float> operator()(float v) const {
    return PixelGray<float>(v);
  }
};

void stereo_refinement(ASPGlobalOptions const& opt) {

  ImageViewRef<float> left_image, right_image;
  ImageViewRef<PixelMask<Vector2f>> input_disp;
  ImageViewRef<PixelMask<Vector2f>> sub_disp;
  std::string left_image_file  = opt.out_prefix+"-L.tif";
  std::string right_image_file = opt.out_prefix+"-R.tif";
  std::string left_mask_file   = opt.out_prefix+"-lMask.tif";
  std::string right_mask_file  = opt.out_prefix+"-rMask.tif";

  int kernel_size = std::max(stereo_settings().subpixel_kernel[0],
                             stereo_settings().subpixel_kernel[1]);

  left_image  = DiskImageView<float>(left_image_file);
  right_image = DiskImageView<float>(right_image_file);

  // It is better to fill no-data pixels with an average from
  // neighbors than to use no-data values in processing. This is a
  // temporary band-aid solution.
  float left_nodata_val = -std::numeric_limits<float>::max();
  if (vw::read_nodata_val(left_image_file, left_nodata_val))
    vw_out() << "Left image nodata: " << left_nodata_val << "\n";
  float right_nodata_val = -std::numeric_limits<float>::max();
  if (vw::read_nodata_val(right_image_file, right_nodata_val))
    vw_out() << "Right image nodata: " << right_nodata_val << "\n";

  left_image = apply_mask(vw::fill_nodata_with_avg
                          (create_mask(left_image, left_nodata_val), kernel_size));
  right_image = apply_mask(vw::fill_nodata_with_avg
                           (create_mask(right_image, right_nodata_val), kernel_size));

  // Read the correct type of correlation file (float for SGM/MGM, otherwise integer)
  std::string disp_file  = opt.out_prefix + "-D.tif";
  std::string blend_file = opt.out_prefix + "-B.tif";

  if (stereo_settings().subpix_from_blend) { // Read the stereo_blend output file
    input_disp = DiskImageView<PixelMask<Vector2f>>(blend_file);
  } else {
    // Read the stereo_corr output file
    boost::shared_ptr<DiskImageResource> rsrc(DiskImageResourcePtr(disp_file));
    ChannelTypeEnum disp_data_type = rsrc->channel_type();
    if (disp_data_type == VW_CHANNEL_INT32)
      input_disp = pixel_cast<PixelMask<Vector2f>>
        (DiskImageView< PixelMask<Vector2i>>(disp_file));
    else // File on disk is float
      input_disp = DiskImageView<PixelMask<Vector2f>>(disp_file);
  }

  bool skip_img_norm = asp::skip_image_normalization(opt);
  if (skip_img_norm && 
      (stereo_settings().subpixel_mode == 2 ||
       stereo_settings().subpixel_mode == 3)) {
    // Images were not normalized in pre-processing. Must do so now
    // as bayes_em_subpixel assumes them to be normalized.
    ImageViewRef<uint8> left_mask,  right_mask;
    left_mask  = DiskImageView<uint8>(left_mask_file);
    right_mask = DiskImageView<uint8>(right_mask_file);

    ImageViewRef<PixelMask<float>> Limg
      = copy_mask(left_image, create_mask(left_mask));
    ImageViewRef<PixelMask<float>> Rimg
      = copy_mask(right_image, create_mask(right_mask));

    Vector<float32> left_stats, right_stats;
    std::string left_stats_file  = opt.out_prefix+"-lStats.tif";
    std::string right_stats_file = opt.out_prefix+"-rStats.tif";
    vw_out() << "Reading: " << left_stats_file << ' ' << right_stats_file << "\n";
    read_vector(left_stats,  left_stats_file);
    read_vector(right_stats, right_stats_file);

    bool use_percentile_stretch = false;
    bool do_not_exceed_min_max = (opt.session->name() == "isis" ||
                                  opt.session->name() == "isismapisis");
    asp::normalize_images(stereo_settings().force_use_entire_range,
                          stereo_settings().individually_normalize,
                          use_percentile_stretch,
                          do_not_exceed_min_max,
                          left_stats, right_stats, Limg, Rimg);

    // As above, fill no-data with average from neighbors
    left_image  = apply_mask(vw::fill_nodata_with_avg(Limg, kernel_size));
    right_image = apply_mask(vw::fill_nodata_with_avg(Rimg, kernel_size));
  }

  // The whole goal of this block it to go through the motions of
  // refining disparity solely for the purpose of printing
  // the relevant messages once, rather than per tile, as in the
  // processing below.
  bool verbose = true;
  ImageView<PixelGray<float>> left_dummy(1, 1), right_dummy(1, 1);
  ImageView<PixelMask<Vector2f>> dummy_disp(1, 1);
  refine_disparity(left_dummy, right_dummy, dummy_disp, opt, verbose);

  // The images must be explicitly converted to have PixelGray<float>
  // pixels.
  ImageViewRef<PixelGray<float>> left_gray
    = per_pixel_filter(left_image, GrayCast());
  ImageViewRef<PixelGray<float>> right_gray
    = per_pixel_filter(right_image, GrayCast());

  ImageViewRef<PixelMask<Vector2f>> refined_disp
     = crop(per_tile_rfne(left_gray, right_gray,
                          input_disp, sub_disp, opt),
            stereo_settings().trans_crop_win);

  cartography::GeoReference left_georef;
  bool   has_left_georef = read_georeference(left_georef,  opt.out_prefix + "-L.tif");
  bool   has_nodata      = false;
  double nodata          = -32768.0;

  std::string rd_file = opt.out_prefix + "-RD.tif";
  vw_out() << "Writing: " << rd_file << "\n";
  vw::cartography::block_write_gdal_image(rd_file, refined_disp,
                              has_left_georef, left_georef,
                              has_nodata, nodata, opt,
                              TerminalProgressCallback("asp", "\t--> Refinement :"));
}

int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();

    vw_out() << "\n[ " << current_posix_time_string()
             << " ]: Stage 3 --> REFINEMENT\n";

    stereo_register_sessions();

    bool verbose = false;
    std::vector<ASPGlobalOptions> opt_vec;
    std::string output_prefix;
    asp::parse_multiview(argc, argv, SubpixelDescription(),
                         verbose, output_prefix, opt_vec);
    ASPGlobalOptions opt = opt_vec[0];

    // Subpixel refinement uses smaller tiles.
    //---------------------------------------------------------
    int ts = ASPGlobalOptions::rfne_tile_size();
    opt.raster_tile_size = Vector2i(ts, ts);

    // Internal Processes
    //---------------------------------------------------------
    stereo_refinement(opt);

    vw_out() << "\n[ " << current_posix_time_string()
             << " ]: REFINEMENT FINISHED\n";

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
