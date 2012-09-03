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
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <vw/InterestPoint.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/SubpixelView.h>

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
  stereo::CostFunctionType m_cost_mode;

public:
  SeededCorrelatorView( ImageViewBase<Image1T> const& left_image,
                        ImageViewBase<Image2T> const& right_image,
                        ImageViewBase<Mask1T> const& left_mask,
                        ImageViewBase<Mask2T> const& right_mask,
                        ImageViewBase<SeedDispT> const& source_disp,
                        stereo::PreFilterBase<PProcT> const& filter,
                        stereo::CostFunctionType cost_mode ) :
    m_left_image(left_image.impl()), m_right_image(right_image.impl()),
    m_left_mask(left_mask.impl()), m_right_mask(right_mask.impl()),
    m_source_disparity( source_disp.impl() ),
    m_preproc_func( filter.impl() ), m_cost_mode( cost_mode ) {
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
    // User strategies
    if ( stereo_settings().seed_option == 1 ) {
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
    } else if ( stereo_settings().seed_option > 1 ) {
      vw_throw( NoImplErr() << "Option " << stereo_settings().seed_option
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
                    stereo::CostFunctionType cost_type ) {
  typedef SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT> return_type;
  return return_type( left.impl(), right.impl(), lmask.impl(), rmask.impl(),
                      sdisp.impl(), filter.impl(), cost_type );
}

// approximate search range
//  Find interest points and grow them into a search range
BBox2i
approximate_search_range( std::string const& left_image,
                          std::string const& right_image,
                          float scale ) {

  typedef PixelGray<float32> PixelT;
  vw_out() << "\t--> Using interest points to determine search window.\n";
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  float i_scale = 1.0/scale;

  // String names
  std::string
    left_ip_file = fs::path( left_image ).replace_extension("vwip").string(),
    right_ip_file = fs::path( right_image ).replace_extension("vwip").string(),
    match_file =
    fs::path( left_image ).replace_extension("").string() + "__" +
    fs::path( right_image ).stem().string() + ".match";

  // Building / Loading Interest point data
  if ( fs::exists(match_file) ) {

    vw_out() << "\t    * Using cached match file.\n";
    ip::read_binary_match_file(match_file, matched_ip1, matched_ip2);

  } else {

    std::vector<ip::InterestPoint> ip1_copy, ip2_copy;

    if ( !fs::exists(left_ip_file) ||
         !fs::exists(right_ip_file) ) {

      // Worst case, no interest point operations have been performed before
      vw_out() << "\t    * Locating Interest Points\n";
      DiskImageView<PixelT> left_sub_image(left_image);
      DiskImageView<PixelT> right_sub_image(right_image);

      // Interest Point module detector code.
      float ipgain = 0.07;
      std::list<ip::InterestPoint> ip1, ip2;
      vw_out() << "\t    * Processing for Interest Points.\n";
      while ( ip1.size() < 1500 || ip2.size() < 1500 ) {
        ip1.clear(); ip2.clear();

        ip::OBALoGInterestOperator interest_operator( ipgain );
        ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator, 0 );

        ip1 = detect_interest_points( left_sub_image, detector );
        ip2 = detect_interest_points( right_sub_image, detector );

        ipgain *= 0.75;
        if ( ipgain < 1e-2 ) {
          vw_out() << "\t    * Unable to find desirable amount of Interest Points.\n";
          break;
        }
      }

      if ( ip1.size() < 8 || ip2.size() < 8 )
        vw_throw( InputErr() << "Unable to extract interest points from input images ["
                  << left_image << "," << right_image << "]! Unable to continue." );

      // Making sure we don't exceed 3000 points
      if ( ip1.size() > 3000 ) {
        ip1.sort(); ip1.resize(3000);
      }
      if ( ip2.size() > 3000 ) {
        ip2.sort(); ip2.resize(3000);
      }

      // Stripping out orientation. This allows for a better
      // possibility of interest point matches.
      //
      // This is no loss as images at this point are already aligned
      // since the dense correlator is not rotation-invariant.
      BOOST_FOREACH( ip::InterestPoint& ip, ip1 ) ip.orientation = 0;
      BOOST_FOREACH( ip::InterestPoint& ip, ip2 ) ip.orientation = 0;

      vw_out() << "\t    * Generating descriptors..." << std::flush;
      ip::SGradDescriptorGenerator descriptor;
      descriptor( left_sub_image, ip1 );
      descriptor( right_sub_image, ip2 );
      vw_out() << "done.\n";

      // Writing out the results
      vw_out() << "\t    * Caching interest points: "
               << left_ip_file << " & " << right_ip_file << std::endl;
      ip::write_binary_ip_file( left_ip_file, ip1 );
      ip::write_binary_ip_file( right_ip_file, ip2 );

    }

    vw_out() << "\t    * Using cached IPs.\n";
    ip1_copy = ip::read_binary_ip_file(left_ip_file);
    ip2_copy = ip::read_binary_ip_file(right_ip_file);

    vw_out() << "\t    * Matching interest points\n";
    ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.6);

    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
            false, TerminalProgressCallback( "asp", "\t    Matching: "));
    vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " putative matches.\n";

    vw_out() << "\t    * Rejecting outliers using RANSAC.\n";
    ip::remove_duplicates(matched_ip1, matched_ip2);
    std::vector<Vector3>
      ransac_ip1 = ip::iplist_to_vectorlist(matched_ip1),
      ransac_ip2 = ip::iplist_to_vectorlist(matched_ip2);
    std::vector<size_t> indices;

    try {
      // Figure out the inlier threshold .. it should be about 3% of
      // the edge lengths. This is a bit of a magic number, but I'm
      // pulling from experience that an inlier threshold of 30
      // worked best for 1024^2 AMC imagery.
      float inlier_threshold =
        0.0075 * ( sum( file_image_size( left_image ) ) +
                   sum( file_image_size( right_image ) ) );

      math::RandomSampleConsensus<math::HomographyFittingFunctor,math::InterestPointErrorMetric>
        ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), inlier_threshold );
      Matrix<double> trans = ransac( ransac_ip1, ransac_ip2 );
      vw_out(DebugMessage,"asp") << "\t    * Ransac Result: " << trans << std::endl;
      vw_out(DebugMessage,"asp") << "\t      inlier thresh: "
                                 << inlier_threshold << " px" << std::endl;
      indices = ransac.inlier_indices(trans, ransac_ip1, ransac_ip2 );
    } catch ( vw::math::RANSACErr const& e ) {
      vw_out() << "-------------------------------WARNING---------------------------------\n";
      vw_out() << "\t    RANSAC failed! Unable to auto detect search range.\n\n";
      vw_out() << "\t    Please proceed cautiously!\n";
      vw_out() << "-------------------------------WARNING---------------------------------\n";
      return BBox2i(-10,-10,20,20);
    }

    { // Keeping only inliers
      std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
      for ( size_t i = 0; i < indices.size(); i++ ) {
        inlier_ip1.push_back( matched_ip1[indices[i]] );
        inlier_ip2.push_back( matched_ip2[indices[i]] );
      }
      matched_ip1 = inlier_ip1;
      matched_ip2 = inlier_ip2;
    }

    vw_out() << "\t    * Caching matches: " << match_file << "\n";
    write_binary_match_file( match_file, matched_ip1, matched_ip2);
  }

  // Find search window based on interest point matches
  namespace ba = boost::accumulators;
  ba::accumulator_set<float, ba::stats<ba::tag::variance> > acc_x, acc_y;
  for (size_t i = 0; i < matched_ip1.size(); i++) {
    acc_x(i_scale * ( matched_ip2[i].x - matched_ip1[i].x ));
    acc_y(i_scale * ( matched_ip2[i].y - matched_ip1[i].y ));
  }
  Vector2f mean( ba::mean(acc_x), ba::mean(acc_y) );
  vw_out(DebugMessage,"asp") << "Mean search is : " << mean << std::endl;
  Vector2f stddev( sqrt(ba::variance(acc_x)), sqrt(ba::variance(acc_y)) );
  BBox2i search_range( mean - 2.5*stddev,
                       mean + 2.5*stddev );
  return search_range;
}

void produce_lowres_disparity( int32 cols, int32 rows, Options const& opt ) {
  DiskImageView<PixelGray<float> > left_sub( opt.out_prefix+"-L_sub.tif" ),
    right_sub( opt.out_prefix+"-R_sub.tif" );

  Vector2f down_sample_scale( float(left_sub.cols()) / float(cols),
                              float(left_sub.rows()) / float(rows) );

  DiskImageView<uint8> left_mask( opt.out_prefix+"-lMask_sub.tif" ),
    right_mask( opt.out_prefix+"-rMask_sub.tif" );

  BBox2i search_range( floor(elem_prod(down_sample_scale,stereo_settings().search_range.min())),
                       ceil(elem_prod(down_sample_scale,stereo_settings().search_range.max())) );

  {
    Vector2i expansion( search_range.width(),
                        search_range.height() );
    expansion *= stereo_settings().seed_percent_pad / 2.0f;
    // Expand by the user selected amount. Default is 25%.
    search_range.min() -= expansion;
    search_range.max() += expansion;
    VW_OUT(DebugMessage,"asp") << "D_sub search range: "
                               << search_range << " px\n";
    asp::block_write_gdal_image( opt.out_prefix + "-D_sub.tif",
                                 remove_outliers(
                                   stereo::pyramid_correlate( left_sub, right_sub,
                                                              left_mask, right_mask,
                                                              stereo::LaplacianOfGaussian(1.4),
                                                              search_range,
                                                              stereo_settings().kernel,
                                                              stereo::CROSS_CORRELATION, 2 ),
                                   1, 1, 2.0, 0.5), opt,
                                 TerminalProgressCallback("asp", "\t--> Low Resolution:") );
  }

  ImageView<PixelMask<Vector2i> > lowres_disparity;
  read_image( lowres_disparity, opt.out_prefix + "-D_sub.tif" );
  search_range =
    stereo::get_disparity_range( lowres_disparity );
  VW_OUT(DebugMessage,"asp") << "D_sub resolved search range: "
                             << search_range << " px\n";
  search_range.min() = floor(elem_quot(search_range.min(),down_sample_scale));
  search_range.max() = ceil(elem_quot(search_range.max(),down_sample_scale));
  stereo_settings().search_range = search_range;
}

void stereo_correlation( Options& opt ) {

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 1 --> CORRELATION \n";

  // Working out search range if need be
  if (stereo_settings().is_search_defined()) {
    vw_out() << "\t--> Using user defined search range.\n";
  } else {
    std::string match_filename =
      opt.out_prefix +
      fs::basename(opt.in_file1) + "__" +
      fs::basename(opt.in_file2) + ".match";
    if (!fs::exists(match_filename)) {
      // If there is not any match files for the input image. Let's
      // gather some IP quickly from the low resolution images. This
      // rountine should only run for:
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
                                  sub_scale );
    } else {
      // There exists a matchfile out there.
      std::vector<ip::InterestPoint> ip1, ip2;
      ip::read_binary_match_file( match_filename, ip1, ip2 );

      Matrix<double> align_matrix;
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
  if ( stereo_settings().seed_option > 0 )
    produce_lowres_disparity( Lmask.cols(), Lmask.rows(), opt );

  // Provide the user with some feedback of what we are actually going
  // to use.
  vw_out() << "\t--------------------------------------------------\n";
  vw_out() << "\t   Kernel Size : " << stereo_settings().kernel << std::endl;
  if ( stereo_settings().seed_option > 0 )
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
  if ( stereo_settings().seed_option > 0 )
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
                          stereo::LaplacianOfGaussian(stereo_settings().slogW),
                          cost_mode );
  } else if ( stereo_settings().pre_filter_mode == 1 ) {
    vw_out() << "\t--> Using Subtracted Mean pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::SubtractedMean(stereo_settings().slogW),
                          cost_mode );
  } else {
    vw_out() << "\t--> Using NO pre-processing filter." << std::endl;
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::NullOperation(), cost_mode );
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
