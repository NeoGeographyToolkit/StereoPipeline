
/// \file stereo_preprocessing.h

#ifndef __ASP_STEREO_PREPROCESSING_H__
#define __ASP_STEREO_PREPROCESSING_H__

#include <asp/Tools/stereo.h>

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
        DiskImageView<PixelGray<float32> > left_sub_disk_image(left_image);
        DiskImageView<PixelGray<float32> > right_sub_disk_image(right_image);
        ImageViewRef<PixelGray<float32> > left_sub_image = left_sub_disk_image;
        ImageViewRef<PixelGray<float32> > right_sub_image = right_sub_disk_image;

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
      } catch (...) {
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

  void stereo_preprocessing( Options& opt ) {

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 0 --> PREPROCESSING \n";

    std::string pre_preprocess_file1, pre_preprocess_file2;
    opt.session->pre_preprocessing_hook(opt.in_file1, opt.in_file2,
                                        pre_preprocess_file1,
                                        pre_preprocess_file2);

    DiskImageView<PixelGray<float> > left_rectified_image(pre_preprocess_file1),
      right_rectified_image(pre_preprocess_file2);
    ImageViewRef<PixelGray<float> > left_image = left_rectified_image,
      right_image = right_rectified_image;

#if defined(VW_HAS_BIGTIFF) && VW_HAS_BIGTIFF == 1
    // Determining if BigTiff is required
    if ( left_rectified_image.cols()*left_rectified_image.rows() > 10e6 )
      opt.gdal_options["BIGTIFF"] = "IF_SAFER";
#endif

    /*
      if (MEDIAN_FILTER == 1){
      vw_out() << "\t--> Median filtering.\n" << std::flush;
      left_image  = fast_median_filter(left_rectified_image, 7);
      right_image = fast_median_filter(right_rectified_image, 7);
      write_image(out_prefix+"-median-L.tif", left_image,
      TerminalProgressCallback( "asp", "\t--> Median L: "));
      write_image(out_prefix+"-median-R.tif", right_image,
      TerminalProgressCallback( "asp", "\t--> Median R: "));
      }
    */

    try {
      DiskImageView<PixelGray<uint8> > testa(opt.out_prefix+"-lMask.tif");
      DiskImageView<PixelGray<uint8> > testb(opt.out_prefix+"-rMask.tif");
      vw_out() << "\t--> Using cached image masks.\n";
    } catch (vw::Exception const& e) {
      vw_out() << "\t--> Generating image masks... \n";

      ImageViewRef<vw::uint8> Lmask =
        pixel_cast<vw::uint8>(threshold(apply_mask(edge_mask(left_image, 0, 0)),0,0,255));
      ImageViewRef<vw::uint8> Rmask =
        pixel_cast<vw::uint8>(threshold(apply_mask(edge_mask(right_image, 0, 0)),0,0,255));

      DiskImageResourceGDAL l_mask_rsrc( opt.out_prefix+"-lMask.tif",
                                         Lmask.format(), opt.raster_tile_size,
                                         opt.gdal_options );
      DiskImageResourceGDAL r_mask_rsrc( opt.out_prefix+"-rMask.tif",
                                         Rmask.format(), opt.raster_tile_size,
                                         opt.gdal_options );
      block_write_image(l_mask_rsrc, Lmask,
                        TerminalProgressCallback("asp", "\t    Mask L: "));
      block_write_image(r_mask_rsrc, Rmask,
                        TerminalProgressCallback("asp", "\t    Mask R: "));
    }

    // Produce subsampled images
    int smallest_edge = std::min(std::min(left_image.cols(), left_image.rows()),
                                 std::min(right_image.cols(), right_image.rows()) );
    float sub_scale = 2048.0 / float(smallest_edge);;
    if ( sub_scale > 1 ) sub_scale = 1;

    // Solving for the number of threads and the tile size to use for
    // subsampling while only using 500 MiB of memory. (The cache code
    // is a little slow on releasing so it will probably use 1.5GiB
    // memory during subsampling) Also tile size must be a power of 2
    // and greater than or equal to 64 px;
    int sub_threads = vw_settings().default_num_threads() + 1;
    int tile_power = 0;
    while ( tile_power < 6 && sub_threads > 1) {
      sub_threads--;
      tile_power = int( log10(500e6*sub_scale*sub_scale/(4.0*float(sub_threads)))/(2*log10(2)));
    }
    int sub_tile_size = int ( pow(2., tile_power) );
    if ( sub_tile_size > vw_settings().default_tile_size() )
      sub_tile_size = vw_settings().default_tile_size();

    std::string l_sub_file = opt.out_prefix+"-L_sub.tif";
    std::string r_sub_file = opt.out_prefix+"-R_sub.tif";

    try {
      DiskImageView<PixelGray<float> > testa(l_sub_file);
      DiskImageView<PixelGray<float> > testb(r_sub_file);
      vw_out() << "\t--> Using cached subsampled image.\n";
    } catch (vw::Exception const& e) {
      vw_out() << "\t--> Creating previews. Subsampling by " << sub_scale
               << " by using " << sub_tile_size << " tile size and "
               << sub_threads << " threads.\n";
      ImageViewRef<PixelGray<vw::float32> > Lsub = resample(left_image, sub_scale);
      ImageViewRef<PixelGray<vw::float32> > Rsub = resample(right_image, sub_scale);
      DiskImageResourceGDAL l_sub_rsrc( l_sub_file, Lsub.format(),
                                        Vector2i(sub_tile_size,
                                                 sub_tile_size),
                                        opt.gdal_options );
      DiskImageResourceGDAL r_sub_rsrc( r_sub_file, Rsub.format(),
                                        Vector2i(sub_tile_size,
                                                 sub_tile_size),
                                        opt.gdal_options );
      int previous_num_threads = vw_settings().default_num_threads();
      vw_settings().set_default_num_threads(sub_threads);
      block_write_image(l_sub_rsrc, Lsub,
                        TerminalProgressCallback("asp", "\t    Sub L: "));
      block_write_image(r_sub_rsrc, Rsub,
                        TerminalProgressCallback("asp", "\t    Sub R: "));
      vw_settings().set_default_num_threads(previous_num_threads);
    }

    // Auto Search Range
    if (stereo_settings().is_search_defined())
      vw_out() << "\t--> Using user defined search range: "
               << opt.search_range << "\n";
    else
      opt.search_range =
        approximate_search_range( l_sub_file, r_sub_file, sub_scale );

  }

} // end namespace vw

#endif//__ASP_STEREO_PREPROCESSING_H__
