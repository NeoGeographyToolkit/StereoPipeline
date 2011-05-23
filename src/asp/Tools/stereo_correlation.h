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

namespace vw {

  // approximate search range
  //  Find interest points and grow them into a search range
  BBox2i
  approximate_search_range( std::string left_image,
                            std::string right_image,
                            float scale ) {

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
        DiskImageView<PixelGray<float32> > left_sub_image(left_image);
        DiskImageView<PixelGray<float32> > right_sub_image(right_image);

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
        }

        // Making sure we don't exceed 3000 points
        ip1.sort();
        ip2.sort();
        if ( ip1.size() > 3000 )
          ip1.resize(3000);
        if ( ip2.size() > 3000 )
          ip2.resize(3000);

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
      ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.5);

      matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
              false, TerminalProgressCallback( "asp", "\t    Matching: "));
      vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " putative matches.\n";

      vw_out() << "\t    * Rejecting outliers using RANSAC.\n";
      remove_duplicates(matched_ip1, matched_ip2);
      std::vector<Vector3> ransac_ip1 = ip::iplist_to_vectorlist(matched_ip1);
      std::vector<Vector3> ransac_ip2 = ip::iplist_to_vectorlist(matched_ip2);
      std::vector<int> indices;

      try {
        Matrix<double> trans;
        math::RandomSampleConsensus<math::HomographyFittingFunctor,math::InterestPointErrorMetric>
          ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 25 );
        trans = ransac( ransac_ip1, ransac_ip2 );
        vw_out(DebugMessage) << "\t    * Ransac Result: " << trans << std::endl;
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
        for ( unsigned i = 0; i < indices.size(); i++ ) {
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
    BBox2i search_range;
    for (unsigned i = 0; i < matched_ip1.size(); i++) {
      Vector2i translation = ( i_scale*Vector2i(matched_ip2[i].x, matched_ip2[i].y) -
                               i_scale*Vector2i(matched_ip1[i].x, matched_ip1[i].y) );

      if ( i == 0 ) {
        search_range.min() = translation;
        search_range.max() = translation + Vector2i(1,1);
      } else
        search_range.grow( translation );
    }
    Vector2i offset = search_range.size()/4; // So we can grow by 50%
    search_range.min() -= offset;
    search_range.max() += offset;

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
