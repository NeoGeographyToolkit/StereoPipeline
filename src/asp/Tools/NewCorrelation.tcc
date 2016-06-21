// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


namespace vw {
namespace stereo {

template <class Image1T, class Image2T, class Mask1T, class Mask2T>
void NewCorrelationView<Image1T, Image2T, Mask1T, Mask2T>::
prefilter_images(ImageView<typename Image1T::pixel_type> &left_image,
                 ImageView<typename Image2T::pixel_type> &right_image) const {

  // TODO: Would be nice not to be hard coding these constants!
  if (m_prefilter_mode == 2){  // LOG
      vw::stereo::LaplacianOfGaussian prefilter(m_prefilter_width);
      left_image  = prefilter.filter(left_image );
      right_image = prefilter.filter(right_image);
      return;
  }
  if (m_prefilter_mode == 1){  // Subtracted mean
      vw::stereo::SubtractedMean prefilter(m_prefilter_width);
      left_image  = prefilter.filter(left_image );
      right_image = prefilter.filter(right_image);
      return;
  }
  //Default: No prefilter
  vw::stereo::NullOperation prefilter;
  left_image  = prefilter.filter(left_image );
  right_image = prefilter.filter(right_image);
}


template <class Image1T, class Image2T, class Mask1T, class Mask2T>
bool NewCorrelationView<Image1T, Image2T, Mask1T, Mask2T>::
build_image_pyramids(vw::BBox2i const& bbox, int32 const max_pyramid_levels,
                     std::vector<ImageView<typename Image1T::pixel_type> > & left_pyramid,
                     std::vector<ImageView<typename Image2T::pixel_type> > & right_pyramid,
                     std::vector<ImageView<typename Mask1T::pixel_type > > & left_mask_pyramid,
                     std::vector<ImageView<typename Mask2T::pixel_type > > & right_mask_pyramid) const {

  Vector2i half_kernel = m_kernel_size/2;

  // Init the pyramids: Highest resolution image is stored at index zero.
  left_pyramid.resize      (max_pyramid_levels + 1);
  right_pyramid.resize     (max_pyramid_levels + 1);
  left_mask_pyramid.resize (max_pyramid_levels + 1);
  right_mask_pyramid.resize(max_pyramid_levels + 1);
  
  // TODO: The cropping could use a check and cleanup!
  
  int32 max_upscaling = 1 << max_pyramid_levels;
  BBox2i left_global_region, right_global_region;
  // Region in the left image is the input bbox expanded by the kernel
  left_global_region = bbox;
  left_global_region.expand(half_kernel * max_upscaling);
  // Region in the right image is the left region plus offsets
  right_global_region = left_global_region + m_search_region.min();
  right_global_region.max() += m_search_region.size() + Vector2i(max_upscaling,max_upscaling);
  
  // Extract the lowest resolution layer
  left_pyramid      [0] = crop(edge_extend(m_left_image                        ), left_global_region );
  right_pyramid     [0] = crop(edge_extend(m_right_image                       ), right_global_region);
  left_mask_pyramid [0] = crop(edge_extend(m_left_mask, ConstantEdgeExtension()), left_global_region );
  right_mask_pyramid[0] = crop(edge_extend(m_right_mask,ConstantEdgeExtension()), right_global_region);

#if VW_DEBUG_LEVEL > 0
  VW_OUT(DebugMessage,"stereo") << " > Left ROI: "    << left_global_region
                                << "\n > Right ROI: " << right_global_region << "\n";
#endif

  // Fill in the nodata of the left and right images with a mean
  // pixel value. This helps with the edge quality of a DEM.
  typename Image1T::pixel_type left_mean;
  typename Image2T::pixel_type right_mean;
  try {
    left_mean  = mean_pixel_value(subsample(copy_mask(left_pyramid [0], create_mask(left_mask_pyramid [0],0)),2));
    right_mean = mean_pixel_value(subsample(copy_mask(right_pyramid[0], create_mask(right_mask_pyramid[0],0)),2));
  } catch ( const ArgumentErr& err ) {
    // Mean pixel value will throw an argument error if there
    // are no valid pixels. If that happens, it means either the
    // left or the right image is full masked.
    return false;
  }
  // Now paste the mean value into the masked pixels
  left_pyramid [0] = apply_mask(copy_mask(left_pyramid [0],create_mask(left_mask_pyramid [0],0)), left_mean  );
  right_pyramid[0] = apply_mask(copy_mask(right_pyramid[0],create_mask(right_mask_pyramid[0],0)), right_mean );

  // Why are we doing this crop?
  // Don't actually need the whole over cropped disparity
  // mask. We only need the active region. I over cropped before
  // just to calculate the mean color value options.
  BBox2i right_mask = bbox + m_search_region.min();
  right_mask.max() += m_search_region.size();
  left_mask_pyramid [0] = crop(left_mask_pyramid [0], bbox       - left_global_region.min());
  right_mask_pyramid[0] = crop(right_mask_pyramid[0], right_mask - right_global_region.min());

  // Build a smoothing kernel to use before downsampling.
  // Szeliski's book recommended this simple kernel. This
  // operation is quickly becoming a time sink, we might
  // possibly want to write an integer optimized version.
  std::vector<typename DefaultKernelT<typename Image1T::pixel_type>::type > kernel(5);
  kernel[0] = kernel[4] = 1.0/16.0;
  kernel[1] = kernel[3] = 4.0/16.0;
  kernel[2] = 6.0/16.0;
  std::vector<uint8> mask_kern(max(m_kernel_size));
  std::fill(mask_kern.begin(), mask_kern.end(), 1 );

  // Smooth and downsample to build the pyramid (don't smooth the masks)
  for ( int32 i = 1; i <= max_pyramid_levels; ++i ) {
    left_pyramid      [i] = subsample(separable_convolution_filter(left_pyramid [i-1],kernel,kernel),2);
    right_pyramid     [i] = subsample(separable_convolution_filter(right_pyramid[i-1],kernel,kernel),2);
    left_mask_pyramid [i] = subsample_mask_by_two(left_mask_pyramid [i-1]);
    right_mask_pyramid[i] = subsample_mask_by_two(right_mask_pyramid[i-1]);
  }

  // Apply the prefilter to each pyramid level
  for ( int32 i = 0; i <= max_pyramid_levels; ++i )
    prefilter_images(left_pyramid[i], right_pyramid[i]);
    
  return true;
}





template <class Image1T, class Image2T, class Mask1T, class Mask2T>
typename NewCorrelationView<Image1T, Image2T, Mask1T, Mask2T>::prerasterize_type
NewCorrelationView<Image1T, Image2T, Mask1T, Mask2T>::
prerasterize(vw::BBox2i const& bbox) const {

    time_t start, end;
    if (m_corr_timeout){
      std::time (&start);
    }

#if VW_DEBUG_LEVEL > 0
    Stopwatch watch;
    watch.start();
#endif

    // 1.0) Determining the number of levels to process
    //      There's a maximum base on kernel size. There's also
    //      maximum defined by the search range. Here we determine
    //      the maximum based on kernel size and current bbox.
    // - max_pyramid_levels is the number of levels not including the original resolution level.
    int32 smallest_bbox      = math::min(bbox.size());
    int32 largest_kernel     = math::max(m_kernel_size);
    int32 max_pyramid_levels = std::floor(log(smallest_bbox)/log(2.0f) - log(largest_kernel)/log(2.0f));
    if ( m_max_level_by_search < max_pyramid_levels )
      max_pyramid_levels = m_max_level_by_search;
    if ( max_pyramid_levels < 1 )
      max_pyramid_levels = 0;
    Vector2i half_kernel = m_kernel_size/2;
    int32 max_upscaling = 1 << max_pyramid_levels;


    // 2.0) Build the pyramids
    //      - Highest resolution image is stored at index zero.
    std::vector<ImageView<typename Image1T::pixel_type> > left_pyramid;
    std::vector<ImageView<typename Image2T::pixel_type> > right_pyramid;
    std::vector<ImageView<typename Mask1T::pixel_type > > left_mask_pyramid;
    std::vector<ImageView<typename Mask2T::pixel_type > > right_mask_pyramid;

    if (!build_image_pyramids(bbox, max_pyramid_levels, left_pyramid, right_pyramid, 
                              left_mask_pyramid, right_mask_pyramid)){
#if VW_DEBUG_LEVEL > 0
      watch.stop();
      double elapsed = watch.elapsed_seconds();
      vw_out(DebugMessage,"stereo") << "Tile " << bbox << " has no data. Processed in " << elapsed << " s\n";
#endif
      return prerasterize_type(ImageView<pixel_type>(bbox.width(), bbox.height()),
                               -bbox.min().x(), -bbox.min().y(),
                               cols(), rows() );
    }
    


    // 3.0) Actually perform correlation now
    ImageView<pixel_type > disparity;
    std::vector<vw::stereo::SearchParam> zones; // Initial search region at lowest resolution level
    zones.push_back( SearchParam(bounding_box(left_mask_pyramid[max_pyramid_levels]),
                                 BBox2i(0,0,m_search_region.width ()/max_upscaling+1,
                                            m_search_region.height()/max_upscaling+1)) );

    // Perform correlation. Keep track of how much time elapsed
    // since we started and stop if we estimate that doing one more
    // image chunk will bring us over time.

    // To not slow us down with timing, we use some heuristics to
    // estimate how much time elapsed, as time to do an image chunk
    // is proportional with image area times search range area. This
    // is not completely accurate, so every now and then do actual
    // timing, no more often than once in measure_spacing seconds.
    double estim_elapsed   = 0.0;
    int    measure_spacing = 2; // seconds
    double prev_estim      = estim_elapsed;

    // Loop down through all of the pyramid levels, low res to high res.
    for ( int32 level = max_pyramid_levels; level >= 0; --level) {

      const bool on_last_level = (level == 0);

      int32 scaling = 1 << level;
      disparity.set_size( left_mask_pyramid[level] );
      Vector2i region_offset = max_upscaling*half_kernel/scaling;
      //vw_out() << "Processing level: " << level << " with size " << disparity.get_size() << std::endl;

      // 3.1) Process each zone with their refined search estimates
      // Do first the zones which take less time, as at some point
      // we may have to enforce the timeout.
      std::sort(zones.begin(), zones.end(), SearchParamLessThan()); // Sort the zones, smallest to largest.
      BOOST_FOREACH( SearchParam const& zone, zones ) {

        BBox2i left_region = zone.first + region_offset; // Kernel width offset
        left_region.expand(half_kernel);
        BBox2i right_region = left_region + zone.second.min();
        right_region.max() += zone.second.size();

        // Check timing estimate to see if we should go ahead with this zone or quit.
        double next_elapsed = m_seconds_per_op * search_volume(SearchParam(left_region, zone.second));
        if (m_corr_timeout > 0.0 && estim_elapsed + next_elapsed > m_corr_timeout){
          vw_out() << "Tile: " << bbox << " reached timeout: "
                   << m_corr_timeout << " s" << std::endl;
          break;
        }else
          estim_elapsed += next_elapsed;

        // See if it is time to actually accurately compute the time
        if (m_corr_timeout > 0.0 && estim_elapsed - prev_estim > measure_spacing){
          std::time (&end);
          double diff = std::difftime(end, start);
          estim_elapsed = diff;
          prev_estim = estim_elapsed;
        }

        // Compute left to right disparity vectors in this zone.
        crop(disparity, zone.first)
          = calc_disparity(m_cost_type,
                           crop(left_pyramid [level], left_region),
                           crop(right_pyramid[level], right_region),
                           left_region - left_region.min(),
                           zone.second.size(), m_kernel_size);

        // If at the last level and the user requested a left<->right consistency check,
        //   compute right to left disparity.
        if ( m_consistency_threshold >= 0 && level == 0 ) {

          // Check the time again before moving on with this
          double next_elapsed = m_seconds_per_op
            * search_volume(SearchParam(right_region, zone.second));
          if (m_corr_timeout > 0.0 && estim_elapsed + next_elapsed > m_corr_timeout){
            vw_out() << "Tile: " << bbox << " reached timeout: "
                     << m_corr_timeout << " s" << std::endl;
            break;
          }else{
            estim_elapsed += next_elapsed;
          }
          // Compute right to left disparity in this zone
          ImageView<pixel_type> rl_result
            = calc_disparity(m_cost_type,
                             crop(edge_extend(right_pyramid[level]), right_region),
                             crop(edge_extend(left_pyramid [level]),
                                  left_region - zone.second.size()),
                             right_region - right_region.min(),
                             zone.second.size(), m_kernel_size)
            - pixel_type(zone.second.size());

          // Find pixels where the disparity distance is greater than m_consistency_threshold
          stereo::cross_corr_consistency_check(crop(disparity,zone.first),
                                                rl_result,
                                               m_consistency_threshold, false);
        } // End of last level right to left disparity check

        // Fix the offsets to account for cropping.
        crop(disparity, zone.first) += pixel_type(zone.second.min());
      } // End of zone loop

      // 3.2a) Filter the disparity so we are not processing more than we need to.
      //       - Inner function filtering is only to catch "speckle" type noise of individual ouliers.
      //       - Outer function just merges the masks over the filtered disparity image.
      const int32 rm_half_kernel = 5;
      const float rm_min_matches_percent = 0.5;
      const float rm_threshold = 3.0;

      if ( !on_last_level ) {
        disparity = disparity_mask(disparity_cleanup_using_thresh
                                     (disparity,
                                      rm_half_kernel, rm_half_kernel,
                                      rm_threshold,
                                      rm_min_matches_percent),
                                     left_mask_pyramid[level],
                                     right_mask_pyramid[level]);
      } else {
        // We don't do a single hot pixel check on the final level as it leaves a border.
        disparity = disparity_mask(rm_outliers_using_thresh
                                     (disparity,
                                      rm_half_kernel, rm_half_kernel,
                                      rm_threshold,
                                      rm_min_matches_percent),
                                     left_mask_pyramid[level],
                                     right_mask_pyramid[level]);
      }

      // 3.2b) Refine search estimates but never let them go beyond
      // the search region defined by the user
      if ( !on_last_level ) {
        zones.clear();

        // What is this doing???
        subdivide_regions( disparity, bounding_box(disparity),
                           zones, m_kernel_size );

        if (0) { // DEBUG
          BBox2i scaled = bbox/2;
          std::ostringstream ostr;
          ostr << "disparity_" << scaled.min()[0] << "_"
               << scaled.min()[1] << "_" << scaled.max()[0] << "_"
               << scaled.max()[1] << "_" << level;
          write_image( ostr.str() + ".tif", pixel_cast<PixelMask<Vector2f> >(disparity) );
          std::ofstream f( (ostr.str() + "_zone.txt").c_str() );
          BOOST_FOREACH( SearchParam& zone, zones ) {
            f << zone.first << " " << zone.second << "\n";
          }
          write_image( ostr.str() + "left.tif",  normalize(left_pyramid [level]) );
          write_image( ostr.str() + "right.tif", normalize(right_pyramid[level]) );
          write_image( ostr.str() + "lmask.tif", left_mask_pyramid [level] );
          write_image( ostr.str() + "rmask.tif", right_mask_pyramid[level] );
          f.close();
        } // End DEBUG
        
        
        scaling >>= 1;
        // Scale search range defines the maximum search range that
        // is possible in the next step. This (at lower levels) will
        // actually be larger than the search range that the user
        // specified. We are able to do this because we are taking
        // advantage of the half kernel padding needed at the hight
        // level of the pyramid.
        BBox2i scale_search_region(0,0,
                                   right_pyramid[level-1].cols() - left_pyramid[level-1].cols(),
                                   right_pyramid[level-1].rows() - left_pyramid[level-1].rows() );
        BBox2i next_zone_size = bounding_box( left_mask_pyramid[level-1] );
        BOOST_FOREACH( SearchParam& zone, zones ) {
          zone.first *= 2;
          zone.first.crop( next_zone_size );
          zone.second *= 2;
          zone.second.expand(2); // This is practically required. Our
          // correlation will fail if the search has only one solution.
          zone.second.crop( scale_search_region );
        }
      } // End not the last level case
    } // End of the level loop

    VW_ASSERT( bbox.size() == bounding_box(disparity).size(),
               MathErr() << "PyramidCorrelation: Solved disparity doesn't match requested bbox size." );

#if VW_DEBUG_LEVEL > 0
    watch.stop();
    double elapsed = watch.elapsed_seconds();
    vw_out(DebugMessage,"stereo") << "Tile " << bbox << " processed in "
                                  << elapsed << " s\n";
    if (m_corr_timeout > 0.0){
      vw_out(DebugMessage,"stereo")
        << "Elapsed (actual/estimated/ratio): " << elapsed << ' '
        << estim_elapsed << ' ' << elapsed/estim_elapsed << std::endl;
    }
#endif

    // 5.0) Reposition our result back into the global
    // solution. Also we need to correct for the offset we applied
    // to the search region.
    return prerasterize_type(disparity + pixel_type(m_search_region.min()),
                             -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  } // End function prerasterize




} // namespace stereo
} // namespace vw


