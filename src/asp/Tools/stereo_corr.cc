// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <vw/InterestPoint.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

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

  // Setings
  Vector2f m_upscale_factor;
  BBox2i m_seed_bbox;
  stereo::CorrelatorType const& m_cost_mode;

public:
  SeededCorrelatorView( ImageViewBase<Image1T> const& left_image,
                        ImageViewBase<Image2T> const& right_image,
                        ImageViewBase<Mask1T> const& left_mask,
                        ImageViewBase<Mask2T> const& right_mask,
                        ImageViewBase<SeedDispT> const& source_disp,
                        PProcT const& preproc_func,
                        stereo::CorrelatorType const& cost_mode ) :
    m_left_image(left_image.impl()), m_right_image(right_image.impl()),
    m_left_mask(left_mask.impl()), m_right_mask(right_mask.impl()),
    m_source_disparity( source_disp.impl() ),
    m_preproc_func( preproc_func ), m_cost_mode( cost_mode ) {
    m_upscale_factor[0] = float(m_left_image.cols()) / float(m_source_disparity.cols());
    m_upscale_factor[1] = float(m_left_image.rows()) / float(m_source_disparity.rows());
    m_seed_bbox = bounding_box( m_source_disparity );
  }

  // Image View interface
  typedef PixelMask<Vector2f> pixel_type;
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
    BBox2i seed_bbox( elem_quot(bbox.min(), m_upscale_factor),
                      elem_quot(bbox.max(), m_upscale_factor) );
    seed_bbox.crop( m_seed_bbox );
    vw_out(DebugMessage, "stereo") << "Getting disparity range for : " << seed_bbox << "\n";

    BBox2f local_search_range =
      stereo::get_disparity_range( crop( m_source_disparity, seed_bbox ) );
    local_search_range = grow_bbox_to_int(local_search_range);
    local_search_range.min() = floor(elem_prod(local_search_range.min(), m_upscale_factor));
    local_search_range.max() = ceil(elem_prod(local_search_range.max(), m_upscale_factor));
    vw_out(DebugMessage, "stereo") << "SeededCorrelatorView(" << bbox << ") search range "
                                   << local_search_range << " vs " << stereo_settings().search_range
                                   << "\n";

    typedef stereo::CorrelatorView<Image1T, Image2T, Mask1T, Mask2T, PProcT> CorrView;
    CorrView corr_view( m_left_image, m_right_image,
                        m_left_mask, m_right_mask,
                        m_preproc_func, true );

    corr_view.set_search_range( local_search_range );
    corr_view.set_kernel_size( stereo_settings().kernel );
    corr_view.set_cross_corr_threshold( stereo_settings().xcorr_threshold );
    corr_view.set_correlator_options(stereo_settings().cost_blur, m_cost_mode);

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
                    PProcT const& preproc_func,
                    stereo::CorrelatorType const& cost_mode ) {
  typedef SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT> return_type;
  return return_type( left.impl(), right.impl(), lmask.impl(), rmask.impl(),
                      sdisp.impl(), preproc_func, cost_mode );
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
  std::string left_ip_file =
    fs::path( left_image ).replace_extension("vwip").string();
  std::string right_ip_file =
    fs::path( right_image ).replace_extension("vwip").string();
  std::string match_file =
    fs::path( left_image ).replace_extension("").string() + "__" +
    fs::path( right_image ).stem() + ".match";

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

      // Stripping out orientation .. this allows for a better
      // possibility of interest point matches.
      //
      // This is no loss as images at this point are already aligned
      // since the dense correlator is not rotation invariant.
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
    std::vector<Vector3> ransac_ip1 = ip::iplist_to_vectorlist(matched_ip1);
    std::vector<Vector3> ransac_ip2 = ip::iplist_to_vectorlist(matched_ip2);
    std::vector<size_t> indices;

    try {
      // Figure out the inlier threshold .. it should be about 3% of
      // the edge lengths. This is a bit of a magic number, but I'm
      // pulling from experience that an inlier threshold of 30
      // worked best for 1024^2 AMC imagery.
      DiskImageView<PixelT> left_sub_image(left_image);
      DiskImageView<PixelT> right_sub_image(right_image);
      float inlier_threshold =
        0.0075 * left_sub_image.cols() +
        0.0075 * left_sub_image.rows() +
        0.0075 * right_sub_image.cols() +
        0.0075 * right_sub_image.rows();

      math::RandomSampleConsensus<math::HomographyFittingFunctor,math::InterestPointErrorMetric>
        ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), inlier_threshold );
      Matrix<double> trans = ransac( ransac_ip1, ransac_ip2 );
      vw_out(DebugMessage) << "\t    * Ransac Result: " << trans << std::endl;
      vw_out(DebugMessage) << "\t      inlier thresh: "
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
    Vector2f translation( i_scale*(Vector2f(matched_ip2[i].x, matched_ip2[i].y) -
                                   Vector2f(matched_ip1[i].x, matched_ip1[i].y)) );
    acc_x(translation.x());
    acc_y(translation.y());
  }
  Vector2f mean( ba::mean(acc_x), ba::mean(acc_y) );
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

  ImageViewRef<uint8> left_mask =
    select_channel(edge_mask(pixel_cast_rescale<uint8>(left_sub)),1);
  ImageViewRef<uint8> right_mask =
    select_channel(edge_mask(pixel_cast_rescale<uint8>(right_sub)),1);

  BBox2i search_range( floor(elem_prod(down_sample_scale,stereo_settings().search_range.min())),
                       ceil(elem_prod(down_sample_scale,stereo_settings().search_range.max())) );

  typedef stereo::LogStereoPreprocessingFilter PreFilterT;

  block_write_gdal_image( opt.out_prefix + "-D_sub.tif",
                          disparity_mask(stereo::correlate( left_sub, right_sub, left_mask,
                                                            right_mask, PreFilterT(1.4),
                                                            search_range,
                                                            stereo_settings().kernel,
                                                            stereo::NORM_XCORR_CORRELATOR),
                                         left_mask, right_mask), opt,
                          TerminalProgressCallback("asp", "\t--> Low Resolution:") );

  ImageView<PixelMask<Vector2f> > lowres_disparity;
  read_image( lowres_disparity, opt.out_prefix + "-D_sub.tif" );
  search_range =
    stereo::get_disparity_range( lowres_disparity );
  search_range.min() = floor(elem_quot(search_range.min(),down_sample_scale));
  search_range.max() = ceil(elem_quot(search_range.max(),down_sample_scale));
  stereo_settings().search_range = search_range;
  vw_out() << "\t--> Refined search range: " << search_range << "\n";
}

void stereo_correlation( Options& opt ) {

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 1 --> CORRELATION \n";

  // Working out search range if need be
  if (stereo_settings().is_search_defined()) {
    vw_out() << "\t--> Using user defined search range.\n";
  } else {
    std::string match_filename =
      fs::basename(opt.in_file1) + "__" +
      fs::basename(opt.in_file2) + ".match";
    if (!fs::exists(match_filename)) {
      // If there is not any match files for the input image. Let's
      // gather some IP quickly from the low resolution images. This
      // rountine should only run for:
      //   Pinhole + Epipolar
      //   Pinhole + None
      // Everything else should gather IP's all the time.
      std::string l_sub_file = opt.out_prefix+"-L_sub.tif";
      std::string r_sub_file = opt.out_prefix+"-R_sub.tif";
      float sub_scale = 0;
      {
        std::string l_org_file = opt.out_prefix+"-L.tif";
        std::string r_org_file = opt.out_prefix+"-R.tif";
        DiskImageView<uint8> l_sub(l_sub_file), r_sub(r_sub_file);
        DiskImageView<uint8> l_org(l_org_file), r_org(r_org_file);
        sub_scale += float(l_sub.cols())/float(l_org.cols());
        sub_scale += float(r_sub.cols())/float(r_org.cols());
        sub_scale += float(l_sub.rows())/float(l_org.rows());
        sub_scale += float(r_sub.rows())/float(r_org.rows());
        sub_scale /= 4;
      }

      stereo_settings().search_range =
        approximate_search_range( l_sub_file, r_sub_file, sub_scale );
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
  produce_lowres_disparity( Lmask.cols(), Lmask.rows(), opt );

  DiskImageView<PixelGray<float> > left_disk_image(opt.out_prefix+"-L.tif"),
    right_disk_image(opt.out_prefix+"-R.tif");
  DiskImageView<PixelMask<Vector2f> > sub_disparity(opt.out_prefix+"-D_sub.tif");
  ImageViewRef<PixelMask<Vector2f> > disparity_map;

  stereo::CorrelatorType cost_mode = stereo::ABS_DIFF_CORRELATOR;
  if (stereo_settings().cost_mode == 1)
    cost_mode = stereo::SQR_DIFF_CORRELATOR;
  else if (stereo_settings().cost_mode == 2)
    cost_mode = stereo::NORM_XCORR_CORRELATOR;

  if (stereo_settings().pre_filter_mode == 3) {
    vw_out() << "\t--> Using SLOG pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW),
                          cost_mode );
  } else if ( stereo_settings().pre_filter_mode == 2 ) {
    vw_out() << "\t--> Using LOG pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::LogStereoPreprocessingFilter(stereo_settings().slogW),
                          cost_mode );
  } else if ( stereo_settings().pre_filter_mode == 1 ) {
    vw_out() << "\t--> Using BLUR pre-processing filter with "
             << stereo_settings().slogW << " sigma blur.\n";
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::BlurStereoPreprocessingFilter(stereo_settings().slogW),
                          cost_mode );
  } else {
    vw_out() << "\t--> Using NO pre-processing filter." << std::endl;
    disparity_map =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask, sub_disparity,
                          stereo::NullStereoPreprocessingFilter(),
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
