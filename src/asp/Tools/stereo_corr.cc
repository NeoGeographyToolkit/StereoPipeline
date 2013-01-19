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
  SeedDispT m_source_disparity;
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
                        ImageViewBase<SeedDispT> const& source_disp,
                        stereo::PreFilterBase<PProcT> const& filter,
                        BBox2i left_image_crop_win,
                        stereo::CostFunctionType cost_mode ) :
    m_left_image(left_image.impl()), m_right_image(right_image.impl()),
    m_left_mask(left_mask.impl()), m_right_mask(right_mask.impl()),
    m_source_disparity( source_disp.impl() ),
    m_preproc_func( filter.impl() ), m_left_image_crop_win(left_image_crop_win), m_cost_mode(cost_mode) {
    m_upscale_factor[0] = float(m_left_image.cols()) / float(m_source_disparity.cols());
    m_upscale_factor[1] = float(m_left_image.rows()) / float(m_source_disparity.rows());
    m_seed_bbox = bounding_box( m_source_disparity );
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
    if ( stereo_settings().seed_mode == 1 ) {
      BBox2i seed_bbox( elem_quot(bbox.min(), m_upscale_factor),
                        elem_quot(bbox.max(), m_upscale_factor) );
      seed_bbox.expand(1);
      seed_bbox.crop( m_seed_bbox );
      VW_OUT(DebugMessage, "stereo") << "Getting disparity range for : " << seed_bbox << "\n";

      BBox2f local_search_range =
        stereo::get_disparity_range( crop( m_source_disparity, seed_bbox ) );

      local_search_range = grow_bbox_to_int(local_search_range);
      local_search_range.expand(1);
      local_search_range.min() = floor(elem_prod(local_search_range.min(), m_upscale_factor));
      local_search_range.max() = ceil(elem_prod(local_search_range.max(), m_upscale_factor));

      VW_OUT(DebugMessage, "stereo") << "SeededCorrelatorView(" << bbox << ") search range "
                                     << local_search_range << " vs " << stereo_settings().search_range
                                     << "\n";

      typedef stereo::PyramidCorrelationView<Image1T, Image2T, Mask1T, Mask2T, PProcT> CorrView;
      CorrView corr_view( m_left_image, m_right_image,
                          m_left_mask, m_right_mask,
                          m_preproc_func, local_search_range,
                          stereo_settings().kernel, m_cost_mode,
                          stereo_settings().xcorr_threshold,
                          stereo_settings().corr_max_levels );

      return corr_view.prerasterize( bbox );
    } else if ( stereo_settings().seed_mode > 1 ) {
      vw_throw( NoImplErr() << "Option " << stereo_settings().seed_mode
                << " is not an implemented choice.\n" );
    }

    VW_OUT(DebugMessage,"stereo") << "Searching with " << stereo_settings().search_range << "\n";
    typedef stereo::PyramidCorrelationView<Image1T, Image2T, Mask1T, Mask2T, PProcT> CorrView;
    CorrView corr_view( m_left_image, m_right_image,
                        m_left_mask, m_right_mask,
                        m_preproc_func, stereo_settings().search_range,
                        stereo_settings().kernel, m_cost_mode,
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
                    ImageViewBase<SeedDispT> const& sdisp,
                    stereo::PreFilterBase<PProcT> const& filter,
                    BBox2i left_image_crop_win,
                    stereo::CostFunctionType cost_type ) {
  typedef SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT> return_type;
  return return_type( left.impl(), right.impl(), lmask.impl(), rmask.impl(),
                      sdisp.impl(), filter.impl(), left_image_crop_win, cost_type );
}

void stereo_correlation( Options& opt ) {

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 1 --> CORRELATION \n";

  // Working out search range if need be
  if (stereo_settings().is_search_defined()) {
    vw_out() << "\t--> Using user defined search range.\n";
  } else {

    // Match file between the input files
    std::string match_filename
      = ip::match_filename(opt.out_prefix, opt.in_file1, opt.in_file2);

    std::cout << "Match file: " << match_filename << std::endl;

    if (!fs::exists(match_filename)) {
      // If there is not any match files for the input image. Let's
      // gather some IP quickly from the low resolution images. This
      // routine should only run for:
      //   Pinhole + Epipolar
      //   Pinhole + None
      //   DG + None
      // Everything else should gather IP's all the time.
      float sub_scale =
        sum(elem_quot( Vector2f(file_image_size( opt.out_prefix+"-L_sub.tif" )),
                       Vector2f(file_image_size( opt.out_prefix+"-L.tif" ) ) )) +
        sum(elem_quot( Vector2f(file_image_size( opt.out_prefix+"-R_sub.tif" )),
                       Vector2f(file_image_size( opt.out_prefix+"-R.tif" ) ) ));
      sub_scale /= 4.0f;

      stereo_settings().search_range =
        approximate_search_range( opt.out_prefix+"-L_sub.tif",
                                  opt.out_prefix+"-R_sub.tif",
                                  ip::match_filename( opt.out_prefix,
                                                      opt.out_prefix+"-L_sub.tif",
                                                      opt.out_prefix+"-R_sub.tif"),
                                  sub_scale );
    } else {
      // There exists a matchfile out there.
      std::vector<ip::InterestPoint> ip1, ip2;
      ip::read_binary_match_file( match_filename, ip1, ip2 );

      Matrix<double> align_matrix = math::identity_matrix<3>();
      if ( fs::exists(opt.out_prefix+"-align.exr") )
        read_matrix(align_matrix, opt.out_prefix + "-align.exr");

      BBox2 search_range;
      for ( size_t i = 0; i < ip1.size(); i++ ) {
        Vector3 r = align_matrix * Vector3(ip2[i].x,ip2[i].y,1);
        r /= r[2];
        search_range.grow( subvector(r,0,2) - Vector2(ip1[i].x,ip1[i].y) );
      }
      stereo_settings().search_range = grow_bbox_to_int( search_range );
    }
    vw_out() << "\t--> Detected search range: " << stereo_settings().search_range << "\n";
  }

  DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
    Rmask(opt.out_prefix + "-rMask.tif");

  // Performing disparity on sub images
  if ( stereo_settings().seed_mode > 0 ) {
    // Re use prior existing D_sub if it exists
    bool rebuild = false;

    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelMask<Vector2i> > test(opt.out_prefix+"-D_sub.tif");
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
      rebuild = true;
    } catch (vw::ArgumentErr const& e ) {
      // Throws on a corrupted file.
      vw_settings().reload_config();
      rebuild = true;
    }

    if ( rebuild )
      produce_lowres_disparity( Lmask.cols(), Lmask.rows(), opt );
  }

  // Provide the user with some feedback of what we are actually going
  // to use.
  vw_out() << "\t--------------------------------------------------\n";
  vw_out() << "\t   Kernel Size : " << stereo_settings().kernel << std::endl;
  if ( stereo_settings().seed_mode > 0 )
    vw_out() << "\t   Rfned Search: "
             << stereo_settings().search_range << std::endl;
  else
    vw_out() << "\t   Search Range: "
             << stereo_settings().search_range << std::endl;
  vw_out() << "\t   Cost Mode   : " << stereo_settings().cost_mode << std::endl;
  vw_out(DebugMessage) << "\t   XCorr Thres : " << stereo_settings().xcorr_threshold << std::endl;
  vw_out(DebugMessage) << "\t   Prefilter   : " << stereo_settings().pre_filter_mode << std::endl;
  vw_out(DebugMessage) << "\t   Prefilter Sz: " << stereo_settings().slogW << std::endl;
  vw_out() << "\t--------------------------------------------------\n";

  // Load up for the actual native resolution processing
  DiskImageView<PixelGray<float> > left_disk_image(opt.out_prefix+"-L.tif"),
    right_disk_image(opt.out_prefix+"-R.tif");
  ImageViewRef<PixelMask<Vector2i> > sub_disparity;
  if ( stereo_settings().seed_mode > 0 )
    sub_disparity =
      DiskImageView<PixelMask<Vector2i> >(opt.out_prefix+"-D_sub.tif");
  ImageViewRef<PixelMask<Vector2i> > disparity_map;

  stereo::CostFunctionType cost_mode = stereo::ABSOLUTE_DIFFERENCE;
  if (stereo_settings().cost_mode == 1)
    cost_mode = stereo::SQUARED_DIFFERENCE;
  else if (stereo_settings().cost_mode == 2)
    cost_mode = stereo::CROSS_CORRELATION;

  if ( stereo_settings().pre_filter_mode == 2 ) {
    vw_out() << "\t--> Using LOG pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::LaplacianOfGaussian(stereo_settings().slogW), opt.left_image_crop_win,
                          cost_mode );
  } else if ( stereo_settings().pre_filter_mode == 1 ) {
    vw_out() << "\t--> Using Subtracted Mean pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::SubtractedMean(stereo_settings().slogW), opt.left_image_crop_win,
                          cost_mode );
  } else {
    vw_out() << "\t--> Using NO pre-processing filter." << std::endl;
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::NullOperation(), opt.left_image_crop_win,
                          cost_mode );
  }

  asp::block_write_gdal_image( opt.out_prefix + "-D.tif",
                               disparity_map, opt,
                               TerminalProgressCallback("asp", "\t--> Correlation :") );
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

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : CORRELATION FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
