// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo_correlation.h

#ifndef __ASP_STEREO_CORRELATION_H__
#define __ASP_STEREO_CORRELATION_H__

#include <asp/Tools/stereo.h>
#include <vw/InterestPoint.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace vw {

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
          vw_throw( InputErr() << "Unable to extract interest points from input images [" << left_image << "," << right_image << "]! Unable to continue." );

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
    vw_out() << "\t--> Detected search range: " << search_range << "\n";
    return search_range;
  }

  // Correlator View
  template <class FilterT>
    inline stereo::CorrelatorView<PixelGray<float>,vw::uint8,FilterT>
    correlator_helper( DiskImageView<PixelGray<float> > & left_disk_image,
                       DiskImageView<PixelGray<float> > & right_disk_image,
                       DiskImageView<vw::uint8> & left_mask,
                       DiskImageView<vw::uint8> & right_mask,
                       FilterT const& filter_func, vw::BBox2i & search_range,
                       stereo::CorrelatorType const& cost_mode,
                       bool draft_mode,
                       std::string & corr_debug_prefix,
                       bool use_pyramid=true ) {
    stereo::CorrelatorView<PixelGray<float>,
      vw::uint8,FilterT> corr_view( left_disk_image, right_disk_image,
                                    left_mask, right_mask, filter_func,
                                    use_pyramid );

    corr_view.set_search_range(search_range);
    corr_view.set_kernel_size(Vector2i(stereo_settings().h_kern,
                                       stereo_settings().v_kern));
    corr_view.set_cross_corr_threshold(stereo_settings().xcorr_threshold);
    corr_view.set_corr_score_threshold(stereo_settings().corrscore_rejection_threshold);
    corr_view.set_correlator_options(stereo_settings().cost_blur, cost_mode);

    if (draft_mode)
      corr_view.set_debug_mode(corr_debug_prefix);

    vw_out() << corr_view;
    vw_out() << "\t--> Building Disparity map." << std::endl;

    return corr_view;
  }

  void stereo_correlation( Options& opt ) {

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 1 --> CORRELATION \n";

    // Working out search range if need be
    if (stereo_settings().is_search_defined()) {
      vw_out() << "\t--> Using user defined search range: "
               << opt.search_range << "\n";
    } else {
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

      opt.search_range =
        approximate_search_range( l_sub_file, r_sub_file, sub_scale );
    }


    DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
      Rmask(opt.out_prefix + "-rMask.tif");

    std::string filename_L = opt.out_prefix+"-L.tif",
      filename_R = opt.out_prefix+"-R.tif";

    /*
      if (MEDIAN_FILTER==1){
      filename_L = out_prefix+"-median-L.tif";
      filename_R = out_prefix+"-median-R.tif";
      } else { *.
      filename_L = out_prefix+"-L.tif";
      filename_R = out_prefix+"-R.tif";
      }*/

    DiskImageView<PixelGray<float> > left_disk_image(filename_L),
      right_disk_image(filename_R);

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
        correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                           stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW),
                           opt.search_range, cost_mode, opt.draft_mode,
                           opt.corr_debug_prefix, !opt.optimized_correlator );
    } else if ( stereo_settings().pre_filter_mode == 2 ) {
      vw_out() << "\t--> Using LOG pre-processing filter with "
               << stereo_settings().slogW << " sigma blur.\n";
      disparity_map =
        correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                           stereo::LogStereoPreprocessingFilter(stereo_settings().slogW),
                           opt.search_range, cost_mode, opt.draft_mode,
                           opt.corr_debug_prefix, !opt.optimized_correlator );
    } else if ( stereo_settings().pre_filter_mode == 1 ) {
      vw_out() << "\t--> Using BLUR pre-processing filter with "
               << stereo_settings().slogW << " sigma blur.\n";
      disparity_map =
        correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                           stereo::BlurStereoPreprocessingFilter(stereo_settings().slogW),
                           opt.search_range, cost_mode, opt.draft_mode,
                           opt.corr_debug_prefix, !opt.optimized_correlator );
    } else {
      vw_out() << "\t--> Using NO pre-processing filter." << std::endl;
      disparity_map =
        correlator_helper( left_disk_image, right_disk_image, Lmask, Rmask,
                           stereo::NullStereoPreprocessingFilter(),
                           opt.search_range, cost_mode, opt.draft_mode,
                           opt.corr_debug_prefix, !opt.optimized_correlator );
    }

    asp::block_write_gdal_image( opt.out_prefix + "-D.tif",
                                 disparity_map, opt,
                                 TerminalProgressCallback("asp", "\t--> Correlation :") );
  }

} //end namespace vw

#endif//__ASP_STEREO_CORRELATION_H__
