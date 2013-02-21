// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file stereo_corr.cc
///

#include <asp/Tools/stereo.h>
#include <vw/InterestPoint.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>

using namespace vw;
using namespace asp;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

// This correlator takes a low resolution disparity image as an input
// so that it may narrow its search range for each tile that is
// processed.
template <class Image1T, class Image2T, class Mask1T, class Mask2T, class SeedDispT, class PProcT>
class SeededCorrelatorView : public ImageViewBase<SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT > > {
  Image1T m_left_image;
  Image2T m_right_image;
  Mask1T m_left_mask;
  Mask2T m_right_mask;
  SeedDispT m_sub_disparity;
  SeedDispT m_sub_disparity_spread;
  PProcT m_preproc_func;

  // Settings
  Vector2f m_upscale_factor;
  BBox2i m_seed_bbox;
  BBox2i m_left_image_crop_win;
  stereo::CostFunctionType m_cost_mode;

public:
  SeededCorrelatorView( ImageViewBase<Image1T> const& left_image,
                        ImageViewBase<Image2T> const& right_image,
                        ImageViewBase<Mask1T> const& left_mask,
                        ImageViewBase<Mask2T> const& right_mask,
                        ImageViewBase<SeedDispT> const& sub_disparity,
                        ImageViewBase<SeedDispT> const& sub_disparity_spread,
                        stereo::PreFilterBase<PProcT> const& filter,
                        BBox2i left_image_crop_win,
                        stereo::CostFunctionType cost_mode ) :
    m_left_image(left_image.impl()), m_right_image(right_image.impl()),
    m_left_mask(left_mask.impl()), m_right_mask(right_mask.impl()),
    m_sub_disparity( sub_disparity.impl() ),
    m_sub_disparity_spread( sub_disparity_spread.impl() ),
    m_preproc_func( filter.impl() ), m_left_image_crop_win(left_image_crop_win), m_cost_mode(cost_mode) {
    m_upscale_factor[0] = float(m_left_image.cols()) / float(m_sub_disparity.cols());
    m_upscale_factor[1] = float(m_left_image.rows()) / float(m_sub_disparity.rows());
    m_seed_bbox = bounding_box( m_sub_disparity );
  }

  // Image View interface
  typedef PixelMask<Vector2i> pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<SeededCorrelatorView> pixel_accessor;

  inline int32 cols() const { return m_left_image.cols(); }
  inline int32 rows() const { return m_left_image.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "SeededCorrelatorView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // We do stereo only in m_left_image_crop_win. Skip the current tile if
    // it does not intersect this region.
    BBox2i intersection = bbox; intersection.crop(m_left_image_crop_win);
    if (intersection.empty()){
      return prerasterize_type(ImageView<pixel_type>(bbox.width(),
                                                     bbox.height()),
                               -bbox.min().x(), -bbox.min().y(),
                               cols(), rows() );
    }

    CropView<ImageView<pixel_type> > disparity = prerasterize_helper(bbox);

    // Set to invalid the disparity outside m_left_image_crop_win.
    for (int col = bbox.min().x(); col < bbox.max().x(); col++){
      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
        if (!m_left_image_crop_win.contains(Vector2(col, row))){
          disparity(col, row) = pixel_type();
        }
      }
    }

    return disparity;
  }

  inline prerasterize_type prerasterize_helper(BBox2i const& bbox) const {

    // User strategies
    if ( stereo_settings().seed_mode == 1 || stereo_settings().seed_mode == 2 ) {
      BBox2i seed_bbox( elem_quot(bbox.min(), m_upscale_factor),
                        elem_quot(bbox.max(), m_upscale_factor) );
      seed_bbox.expand(1);
      seed_bbox.crop( m_seed_bbox );
      VW_OUT(DebugMessage, "stereo") << "Getting disparity range for : " << seed_bbox << "\n";

      BBox2f local_search_range =
        stereo::get_disparity_range( crop( m_sub_disparity, seed_bbox ) );


      if (stereo_settings().seed_mode == 2){
        // Expand the disparity range by the disparity spread computed
        // from input DEM.
        BBox2f spread =
          stereo::get_disparity_range( crop( m_sub_disparity_spread, seed_bbox ) );
        local_search_range.min() -= spread.max();
        local_search_range.max() += spread.max();
      }

      local_search_range = grow_bbox_to_int(local_search_range);
      // Expand local_search_range by 1. This is necessary since
      // m_sub_disparity is integer-valued, and perhaps the search
      // range was supposed to be a fraction of integer bigger.
      local_search_range.expand(1);
      // Scale the search range to full-resolution
      local_search_range.min() = floor(elem_prod(local_search_range.min(), m_upscale_factor));
      local_search_range.max() = ceil(elem_prod(local_search_range.max(), m_upscale_factor));

      VW_OUT(DebugMessage, "stereo") << "SeededCorrelatorView(" << bbox << ") search range "
                                     << local_search_range << " vs " << stereo_settings().search_range
                                     << "\n";

      typedef stereo::PyramidCorrelationView<Image1T, Image2T, Mask1T, Mask2T, PProcT> CorrView;
      CorrView corr_view( m_left_image, m_right_image,
                          m_left_mask, m_right_mask,
                          m_preproc_func, local_search_range,
                          stereo_settings().corr_kernel, m_cost_mode,
                          stereo_settings().xcorr_threshold,
                          stereo_settings().corr_max_levels );

      return corr_view.prerasterize( bbox );
    } else if ( stereo_settings().seed_mode != 0 ) {
      vw_throw( ArgumentErr() << "stereo_corr: Invalid value for seed-mode: " << stereo_settings().seed_mode << ".\n" );
    }

    VW_OUT(DebugMessage,"stereo") << "Searching with " << stereo_settings().search_range << "\n";
    typedef stereo::PyramidCorrelationView<Image1T, Image2T, Mask1T, Mask2T, PProcT> CorrView;
    CorrView corr_view( m_left_image, m_right_image,
                        m_left_mask, m_right_mask,
                        m_preproc_func, stereo_settings().search_range,
                        stereo_settings().corr_kernel, m_cost_mode,
                        stereo_settings().xcorr_threshold,
                        stereo_settings().corr_max_levels );

    return corr_view.prerasterize( bbox );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class Image1T, class Image2T, class Mask1T, class Mask2T, class SeedDispT, class PProcT>
SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT>
seeded_correlation( ImageViewBase<Image1T> const& left,
                    ImageViewBase<Image2T> const& right,
                    ImageViewBase<Mask1T> const& lmask,
                    ImageViewBase<Mask2T> const& rmask,
                    ImageViewBase<SeedDispT> const& sub_disparity,
                    ImageViewBase<SeedDispT> const& sub_disparity_spread,
                    stereo::PreFilterBase<PProcT> const& filter,
                    BBox2i left_image_crop_win,
                    stereo::CostFunctionType cost_type ) {
  typedef SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT> return_type;
  return return_type( left.impl(), right.impl(), lmask.impl(), rmask.impl(),
                      sub_disparity.impl(), sub_disparity_spread.impl(), filter.impl(), left_image_crop_win, cost_type );
}

void stereo_correlation( Options& opt ) {

  lowres_correlation(opt);

  if (stereo_settings().compute_low_res_disparity_only) return;

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 1 --> CORRELATION \n";

  // Provide the user with some feedback of what we are actually going
  // to use.
  vw_out()   << "\t--------------------------------------------------\n";
  vw_out()   << "\t   Kernel Size:    " << stereo_settings().corr_kernel << std::endl;
  if ( stereo_settings().seed_mode > 0 )
    vw_out() << "\t   Refined Search: "
             << stereo_settings().search_range << std::endl;
  else
    vw_out() << "\t   Search Range:   "
             << stereo_settings().search_range << std::endl;
  vw_out()   << "\t   Cost Mode:      " << stereo_settings().cost_mode << std::endl;
  vw_out(DebugMessage) << "\t   XCorr Thresh: " << stereo_settings().xcorr_threshold << std::endl;
  vw_out(DebugMessage) << "\t   Prefilter:    " << stereo_settings().pre_filter_mode << std::endl;
  vw_out(DebugMessage) << "\t   Prefilter Sz: " << stereo_settings().slogW << std::endl;
  vw_out() << "\t--------------------------------------------------\n";

  // Load up for the actual native resolution processing
  DiskImageView<PixelGray<float> > left_disk_image(opt.out_prefix+"-L.tif"),
    right_disk_image(opt.out_prefix+"-R.tif");
  ImageViewRef<PixelMask<Vector2i> > sub_disparity;
  if ( stereo_settings().seed_mode > 0 )
    sub_disparity =
      DiskImageView<PixelMask<Vector2i> >(opt.out_prefix+"-D_sub.tif");
  ImageViewRef<PixelMask<Vector2i> > sub_disparity_spread;
  if ( stereo_settings().seed_mode == 2 )
    sub_disparity_spread =
      DiskImageView<PixelMask<Vector2i> >(opt.out_prefix+"-D_sub_spread.tif");
  ImageViewRef<PixelMask<Vector2i> > fullres_disparity;

  DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
    Rmask(opt.out_prefix + "-rMask.tif");

  stereo::CostFunctionType cost_mode;
  if      (stereo_settings().cost_mode == 0) cost_mode = stereo::ABSOLUTE_DIFFERENCE;
  else if (stereo_settings().cost_mode == 1) cost_mode = stereo::SQUARED_DIFFERENCE;
  else if (stereo_settings().cost_mode == 2) cost_mode = stereo::CROSS_CORRELATION;
  else
    vw_throw( ArgumentErr() << "Unknown value " << stereo_settings().cost_mode << " for cost-mode.\n" );

  if ( stereo_settings().pre_filter_mode == 2 ) {
    vw_out() << "\t--> Using LOG pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    fullres_disparity =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity, sub_disparity_spread,
                          stereo::LaplacianOfGaussian(stereo_settings().slogW), opt.left_image_crop_win,
                          cost_mode );
  } else if ( stereo_settings().pre_filter_mode == 1 ) {
    vw_out() << "\t--> Using Subtracted Mean pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    fullres_disparity =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity, sub_disparity_spread,
                          stereo::SubtractedMean(stereo_settings().slogW), opt.left_image_crop_win,
                          cost_mode );
  } else {
    vw_out() << "\t--> Using NO pre-processing filter." << std::endl;
    fullres_disparity =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity, sub_disparity_spread,
                          stereo::NullOperation(), opt.left_image_crop_win,
                          cost_mode );
  }

  asp::block_write_gdal_image( opt.out_prefix + "-D.tif",
                               fullres_disparity, opt,
                               TerminalProgressCallback("asp", "\t--> Correlation :") );

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : CORRELATION FINISHED \n";

}

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      CorrelationDescription() );

    // Integer correlator requires 1024 px tiles
    //---------------------------------------------------------
    opt.raster_tile_size = Vector2i(1024,1024);

    // Internal Processes
    //---------------------------------------------------------
    stereo_correlation( opt );

  } ASP_STANDARD_CATCHES;

  return 0;
}
