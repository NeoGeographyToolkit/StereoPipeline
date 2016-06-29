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


  /// Lower level implementation function for calc_disparity.
  /// - The inputs must already be rasterized to safe sizes!
  /// - Since the inputs are rasterized, the input images must not be too big.
  template <template<class,bool> class CostFuncT, class PixelT>
  ImageView<PixelMask<Vector2i> >
  best_of_search_convolution2(ImageView<PixelT> const& left_raster,
                              ImageView<PixelT> const& right_raster,
                              BBox2i            const& left_region,
                              Vector2i          const& search_volume,
                              Vector2i          const& kernel_size,
                              bool debug ) {

    typedef ImageView<PixelT> ImageType;
    typedef typename CostFuncT<ImageType,
      boost::is_integral<typename PixelChannelType<PixelT>::type>::value>::accumulator_type AccumChannelT;
    typedef typename PixelChannelCast<PixelT,AccumChannelT>::type AccumT;
    typedef typename std::pair<AccumT,AccumT> QualT;


    // Build cost function which sometimes has side car data
    CostFuncT<ImageType,boost::is_integral<typename PixelChannelType<PixelT>::type>::value> 
          cost_function( left_raster, right_raster, kernel_size);

    // Result buffers
    Vector2i result_size = bounding_box(left_raster).size() - kernel_size + Vector2i(1,1);
    ImageView<PixelMask<Vector2i> > disparity_map(result_size[0], result_size[1]);
    std::fill( disparity_map.data(), disparity_map.data() + prod(result_size),
               PixelMask<Vector2i>(Vector2i()) );
    // First channel is best, second is worst.
    ImageView<QualT > quality_map( result_size[0], result_size[1] );
    
    // Storage buffers
    ImageView<AccumT> cost_metric      ( result_size[0], result_size[1] );
    ImageView<AccumT> cost_applied     ( left_raster.cols(), left_raster.rows() );
    ImageView<PixelT> right_raster_crop( left_raster.cols(), left_raster.rows() );

    // Loop across the disparity range we are searching over.
    Vector2i disparity(0,0);
    for ( ; disparity.y() != search_volume[1]; ++disparity.y() ) {
      for ( disparity.x() = 0; disparity.x() != search_volume[0]; ++disparity.x() ) {
      
        // Compute correlations quickly by shifting the right image by the
        //  current disparity, computing the pixel difference at each location,
        //  and using fast_box_sum/cost_function to get the final convolution
        //  value at each location in "cost_metric"
      
        // There's only one raster here. Fast box sum calls each pixel
        // individually by pixel accessor. It only calls each pixel
        // once so there's no reason to copy/rasterize the cost result before hand.
        //
        // The cost function should also not be applying an edge
        // extension as we've already over cropped the input.
        
        right_raster_crop = crop(right_raster, bounding_box(left_raster)+disparity);
        cost_applied      = cost_function( left_raster, right_raster_crop);
        cost_metric       = fast_box_sum<AccumChannelT>(cost_applied, kernel_size );
        cost_function.cost_modification( cost_metric, disparity );

        // Dump the scores for this offset to disk
        if (debug) {
          std::ostringstream ostr;
          ostr << "detail_" << disparity.y() << "_"
                 << disparity.x() << ".tif";
          write_image( ostr.str(),  cost_metric );
        }

        // Loop across the region we want to compute disparities for.
        // - The correlation score for each pixel is located in "cost_metric"
        // - We update the best and worst disparity for each pixel in "quality_map"

        // These conditionals might be served outside of the iteration
        // of dx and dy. It would make the code slightly longer but
        // would avoid a conditional inside a double loop.
        const AccumT* cost_ptr     = cost_metric.data();
        const AccumT* cost_ptr_end = cost_metric.data() + prod(result_size);
        QualT* quality_ptr         = quality_map.data();
        PixelMask<Vector2i>* disparity_ptr = disparity_map.data();
        if ( disparity != Vector2i(0,0) ) {
          // Normal comparison operations
          while ( cost_ptr != cost_ptr_end ) {
            if ( cost_function.quality_comparison( *cost_ptr, quality_ptr->first ) ) {
              // Better than best?
              quality_ptr->first = *cost_ptr;
              disparity_ptr->child() = disparity;
            } else if ( !cost_function.quality_comparison( *cost_ptr, quality_ptr->second ) ) {
              // Worse than worse
              quality_ptr->second = *cost_ptr;
            }
            ++cost_ptr;
            ++quality_ptr;
            ++disparity_ptr;
          }
        } else {
          // Initializing quality_map and disparity_map with first result
          while ( cost_ptr != cost_ptr_end ) {
            quality_ptr->first = quality_ptr->second = *cost_ptr;
            ++cost_ptr;
            ++quality_ptr;
          }
        }
      } // End x loop
    } // End y loop


    // Determine validity of result (detects rare invalid cases)
    size_t invalid_count = 0;
    const QualT* quality_ptr      = quality_map.data();
    const QualT* quality_ptr_end  = quality_map.data() + prod(result_size);
    PixelMask<Vector2i>* disp_ptr = disparity_map.data();
    while ( quality_ptr != quality_ptr_end ) {
      if ( quality_ptr->first == quality_ptr->second ) {
        invalidate( *disp_ptr );
        ++invalid_count;
      }
      ++quality_ptr;
      ++disp_ptr;
    }
    //std::cout << "Invalidated " << invalid_count << " pixels in best_of_search_convolution2\n";

    return disparity_map;
  } // End function best_of_search_convolution



  /// This actually RASTERIZES/COPY the input images. It then makes an
  /// allocation to store current costs.
  ///
  /// Users pass us the active region of the left image. Hopefully this
  /// allows them to consider if they need edge extension.
  ///
  /// The return size of this function will be:
  ///     return size = left_region_size - kernel_size + 1.
  ///
  /// This means the user must take in account the kernel size for
  /// deciding the region size.
  ///
  /// The size of the area we're going to access in the right is
  /// calculated as follows:
  ///     right_region = left_region + search_volume - 1.
  ///
  /// The pixel types on the input images need to be the same!
  template <class ImageT1, class ImageT2>
  ImageView<PixelMask<Vector2i> >
  calc_disparity2(CostFunctionType cost_type,
                 ImageViewBase<ImageT1> const& left_in,
                 ImageViewBase<ImageT2> const& right_in,
                 BBox2i                 const& left_region,
                 Vector2i               const& search_volume,
                 Vector2i               const& kernel_size,
                 bool debug=false){

    
    // Sanity check the input:
    VW_DEBUG_ASSERT( kernel_size[0] % 2 == 1 && kernel_size[1] % 2 == 1,
                     ArgumentErr() << "best_of_search_convolution: Kernel input not sized with odd values." );
    VW_DEBUG_ASSERT( kernel_size[0] <= left_region.width() &&
                     kernel_size[1] <= left_region.height(),
                     ArgumentErr() << "best_of_search_convolution: Kernel size too large of active region." );
    VW_DEBUG_ASSERT( search_volume[0] > 0 && search_volume[1] > 0,
                     ArgumentErr() << "best_of_search_convolution: Search volume must be greater than 0." );
    VW_DEBUG_ASSERT( left_region.min().x() >= 0 &&  left_region.min().y() >= 0 &&
                     left_region.max().x() <= left_in.impl().cols() &&
                     left_region.max().y() <= left_in.impl().rows(),
                     ArgumentErr() << "best_of_search_convolution: Region not inside left image." );

    // Rasterize input so that we can do a lot of processing on it.
    BBox2i right_region = left_region;
    right_region.max() += search_volume - Vector2i(1,1);
    ImageView<typename ImageT1::pixel_type> left ( crop(left_in.impl(),  left_region) );
    ImageView<typename ImageT2::pixel_type> right( crop(right_in.impl(), right_region) );
    
    // Call the lower level function with the appropriate cost function type
    switch ( cost_type ) {
    case CROSS_CORRELATION:
      return best_of_search_convolution2<NCCCost>(left, right, left_region, search_volume, kernel_size, debug);
    case SQUARED_DIFFERENCE:
      return best_of_search_convolution2<SquaredCost>(left, right, left_region, search_volume, kernel_size, debug);
    default: // case ABSOLUTE_DIFFERENCE:
      return best_of_search_convolution2<AbsoluteCost>(left, right, left_region, search_volume, kernel_size, debug);
    }
    
  } // End function calc_disparity

//---------------------------------------------------------

// An OpenCV based template matcher for debugging aid

/*
cv::Mat cvWrapImage(ImageView<PixelGray<float> > & vwImage) {
  
  // Figure out the image buffer parameters
  void*   raw_data_ptr = reinterpret_cast<void*>(vwImage.data());
  size_t  pixel_size   = sizeof(vw::uint8);
  size_t  step_size    = vwImage.cols() * pixel_size;

  // Create an OpenCV wrapper for the buffer image
  cv::Mat cv_image(vwImage.rows(), vwImage.cols(), CV_8UC1, raw_data_ptr, step_size);
  return cv_image;
}

template <class ImageT1, class ImageT2>
ImageView<PixelMask<Vector2i> >
cv_disparity(CostFunctionType cost_type,
               ImageViewBase<ImageT1> const& left,
               ImageViewBase<ImageT2> const& right,
               BBox2i                 const& left_region,
               Vector2i               const& search_volume,
               Vector2i               const& kernel_size){

  // Rasterize the two input images
  BBox2i right_region = left_region;
  right_region.max() += search_volume - Vector2i(1,1);
  ImageView<PixelGray<float> > left_raster ( crop(left.impl(),  left_region) );
  ImageView<PixelGray<float> > right_raster( crop(right.impl(), right_region) );

  //write_image( "ocv_left_raster.tif",  left_raster );
  //write_image( "ocv_right_raster.tif",  right_raster );

  // Wrap the image buffers with OpenCV
  cv::Mat left_cv  = cvWrapImage(left_raster);
  cv::Mat right_cv = cvWrapImage(right_raster);

  // Init the output image
  // - Half kernel smaller than left on each side
  Vector2i result_size = left_region.size() - kernel_size + Vector2i(1,1);
  ImageView<PixelMask<Vector2i> > disparity_map(result_size[0], result_size[1]);

  // For each pixel in the left image, find the offset to the equivalent
  //  pixel in the right image.

  int method = CV_TM_CCORR_NORMED;
  
  // Init buffer for search results
  cv::Mat values;
  values.create(search_volume[0], search_volume[1], CV_32FC1);

  // Loop through all pixels in the output image
  for (int r=0; r<disparity_map.rows(); ++r) {
    for (int c=0; c<disparity_map.cols(); ++c) {
    
      cv::Rect left_roi(c, r, kernel_size[0], kernel_size[1]);
      cv::Rect right_roi(c,r, search_volume[0]+kernel_size[0]-1, 
                              search_volume[1]+kernel_size[1]-1);
      cv::matchTemplate(right_cv(right_roi), left_cv(left_roi), values, method);
    
      // Grab the best match location
      double minVal, maxVal; 
      cv::Point minLoc, maxLoc, matchLoc;
      cv::minMaxLoc( values, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
      matchLoc = maxLoc; // Depends on the method
      
      printf("Disp loc %d, %d = %d, %d\n", r,c, matchLoc.x, matchLoc.y);
      
      disparity_map(r,c) = Vector2i(matchLoc.x, matchLoc.y);
    }
  }


  return disparity_map;
}

*/

//=======================================================================

  bool subdivide_regions2( ImageView<PixelMask<Vector2i> > const& disparity,
                          BBox2i const& current_bbox,
                          std::vector<SearchParam>& list,
                          Vector2i const& kernel_size,
                          int32 fail_count=0 ) {

    // Looking at the 2d disparity vectors inside current_bbox

    const int MIN_REGION_SIZE = 16;

    // 1.) Is this region too small? Must we stop?
    if ( prod(current_bbox.size()) <= 200 ||
         current_bbox.width() < MIN_REGION_SIZE || current_bbox.height() < MIN_REGION_SIZE ){
      BBox2i expanded = current_bbox;
      expanded.expand(1);
      expanded.crop( bounding_box( disparity ) );
      PixelAccumulator<EWMinMaxAccumulator<Vector2i> > accumulator;
      for_each_pixel( crop(disparity, expanded), accumulator );
      if ( !accumulator.is_valid() ) return true;

      list.push_back( SearchParam( current_bbox,
                                   BBox2i(accumulator.minimum(),
                                          accumulator.maximum() + Vector2i(1,1) ) ) );
      return true;
    }

    // 2) Divide the current_bbox into 4 quadrants, does it reduce total search?
    Vector2i split_pt = current_bbox.size()/2;
    BBox2i q1( current_bbox.min(), current_bbox.min()+split_pt );
    BBox2i q4( current_bbox.min()+split_pt, current_bbox.max() );
    BBox2i q2( current_bbox.min() + Vector2i(split_pt[0],0),
               Vector2i(current_bbox.max()[0],current_bbox.min()[1]+split_pt[1]) );
    BBox2i q3( current_bbox.min() + Vector2i(0,split_pt[1]),
               Vector2i(current_bbox.min()[0]+split_pt[0],current_bbox.max()[1]) );
    BBox2i q1_search, q2_search, q3_search, q4_search;

    // Inside each of the four quadrants, find the min and max disparity.
    // - Masked out pixels are ignored --> TODO: No split if too many masked in a region!
    // - Accumulate product of disparity search region + pixel area
    // - TODO: Should get some of this logic into class functions.
    int32 split_search = 0;
    { // Q1
      PixelAccumulator<EWMinMaxAccumulator<Vector2i> > accumulator;
      for_each_pixel( crop(disparity,q1), accumulator );
      if ( accumulator.is_valid() ) {
        q1_search = BBox2i(accumulator.minimum(),
                           accumulator.maximum()+Vector2i(1,1));
        split_search += q1_search.area() * prod(q1.size()+kernel_size);
      }
    }
    { // Q2
      PixelAccumulator<EWMinMaxAccumulator<Vector2i> > accumulator;
      for_each_pixel( crop(disparity,q2), accumulator );
      if ( accumulator.is_valid() ) {
        q2_search = BBox2i(accumulator.minimum(),
                           accumulator.maximum()+Vector2i(1,1));
        split_search += q2_search.area() * prod(q2.size()+kernel_size);
      }
    }
    { // Q3
      PixelAccumulator<EWMinMaxAccumulator<Vector2i> > accumulator;
      for_each_pixel( crop(disparity,q3), accumulator );
      if ( accumulator.is_valid() ) {
        q3_search = BBox2i(accumulator.minimum(),
                           accumulator.maximum()+Vector2i(1,1));
        split_search += q3_search.area() * prod(q3.size()+kernel_size);
      }
    }
    { // Q4
      PixelAccumulator<EWMinMaxAccumulator<Vector2i> > accumulator;
      for_each_pixel( crop(disparity,q4), accumulator );
      if ( accumulator.is_valid() ) {
        q4_search = BBox2i(accumulator.minimum(),
                           accumulator.maximum()+Vector2i(1,1));
        split_search += q4_search.area() * prod(q4.size()+kernel_size);
      }
    }
    // Now we have an estimate of the cost of processing these four
    // quadrants seperately

    // 3) Find current search v2
    //    - Get the min and max disparity search range that we just calculated
    //      for the four quadrants.  This is faster than recomputing the min/max.
    BBox2i current_search_region;
    if ( q1_search != BBox2i() )
      current_search_region = q1_search;
    if ( q2_search != BBox2i() && current_search_region == BBox2i() )
      current_search_region = q2_search;
    else
      current_search_region.grow(q2_search);
    if ( q3_search != BBox2i() && current_search_region == BBox2i() )
      current_search_region = q3_search;
    else
      current_search_region.grow(q3_search);
    if ( q4_search != BBox2i() && current_search_region == BBox2i() )
      current_search_region = q4_search;
    else
      current_search_region.grow(q4_search);
    int32 current_search = current_search_region.area() * prod(current_bbox.size()+kernel_size);

    const double IMPROVEMENT_RATIO = 0.8;
    
    //std::cout << "split search: " << split_search << ", current = " << current_search*IMPROVEMENT_RATIO << std::endl;

    if ( split_search > current_search*IMPROVEMENT_RATIO && fail_count == 0 ) {
      // Splitting up the disparity region did not reduce our workload.
      // This is our first failure, so see if we can still improve by
      //  subdividing the quadrants one more time.
      std::vector<SearchParam> failed;
      if (!subdivide_regions2( disparity, q1, list, kernel_size, fail_count + 1 ) )
        failed.push_back(SearchParam(q1,q1_search));
      if (!subdivide_regions2( disparity, q2, list, kernel_size, fail_count + 1 ) )
        failed.push_back(SearchParam(q2,q2_search));
      if (!subdivide_regions2( disparity, q3, list, kernel_size, fail_count + 1 ) )
        failed.push_back(SearchParam(q3,q3_search));
      if (!subdivide_regions2( disparity, q4, list, kernel_size, fail_count + 1 ) )
        failed.push_back(SearchParam(q4,q4_search));
      if ( failed.size() == 4 ) {
        // All failed, push back this region as a whole (what we started with)
        list.push_back( SearchParam( current_bbox,
                                     current_search_region ) );
        return true;
      } else if ( failed.size() == 3 ) {
        // 3 failed to split can I merge ?
        // - See the failed==2 case for description!
        std::vector<SearchParam>::const_iterator it1 = failed.begin(), it2 = failed.begin();
        ++it2;
        if ( ( it1->first.min().x() == it2->first.min().x() ||
               it1->first.min().y() == it2->first.min().y() ) &&
             it1->second == it2->second ) {
          BBox2i merge = it1->first;
          merge.grow(it2->first);
          list.push_back( SearchParam( merge, it1->second ) );
          list.push_back( *++it2 );
          return true;
        }
        ++it1; ++it2;
        if ( ( it1->first.min().x() == it2->first.min().x() ||
               it1->first.min().y() == it2->first.min().y() ) &&
             it1->second == it2->second ) {
          BBox2i merge = it1->first;
          merge.grow(it2->first);
          list.push_back( SearchParam( merge, it1->second ) );
          list.push_back( failed.front() );
          return true;
        }
        it1 = failed.begin();
        if ( ( it1->first.min().x() == it2->first.min().x() ||
               it1->first.min().y() == it2->first.min().y() ) &&
             it1->second == it2->second ) {
          BBox2i merge = it1->first;
          merge.grow(it2->first);
          list.push_back( SearchParam( merge, it1->second ) );
          list.push_back( *++it1 );
          return true;
        }
        // Push only the bombed regions, possibly a merge step could go here
        list.insert( list.end(), failed.begin(), failed.end() );
      } else if ( failed.size() == 2 ) {
        // 2 failed to split.
        // If the quadrants are adjacent and have the same disparity range,
        //  merge them into a single search region.
        // - TODO: How often does this actually work?
        if ( ( failed.front().first.min().x() == failed.back().first.min().x() ||
               failed.front().first.min().y() == failed.back().first.min().y() ) &&
             failed.front().second == failed.back().second ) {
          BBox2i merge = failed.front().first;
          merge.grow(failed.back().first);
          list.push_back( SearchParam( merge, failed.front().second ) );
          return true;
        }
        list.insert( list.end(), failed.begin(), failed.end() );
      } else if ( failed.size() == 1 ) {
        // Only 1 failed to split, use it in its entirety, allowing
        // us to take advantage of the other regions which split well.
        list.push_back( failed.front() );
      }
      return true;
    } else if ( split_search > current_search*IMPROVEMENT_RATIO && fail_count > 0 ) {
      // Second failure trying to split this region, give up!
      return false;
    } else {
      // Good split, Try to keep splitting each of the four quadrants further.
      subdivide_regions2( disparity, q1, list, kernel_size );
      subdivide_regions2( disparity, q2, list, kernel_size );
      subdivide_regions2( disparity, q3, list, kernel_size );
      subdivide_regions2( disparity, q4, list, kernel_size );
    }
    return true;
  }



//============================================================================





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

/// Mask out disparity pixels which have too much intersection error
void intersection_filter(ImageView<PixelMask<Vector2i> > &disparity, 
                        boost::shared_ptr<vw::stereo::StereoModel> stereo_model) {
  const double MAX_DISTANCE = 50; // TODO: Add to stereo settings!
                        
  // TODO: Move out of the class and use a per-pixel functor or something.
  double error_dist;
  for (int r=0; r<disparity.rows(); ++r) {
    for (int c=0; c<disparity.cols(); ++c) {
      // Grab this pixel and skip it if it is already masked
      PixelMask<Vector2i> disp_pixel = disparity(c,r);
      if (!is_valid(disp_pixel)) 
        continue;
      // Determine the global pixel coordinates of the left and right pixel
      Vector2i left_pos(c,r);
      Vector2i right_pos(c+disp_pixel[0], c+disp_pixel[1]);
      
      // Compute ray intersection distance, if the error is too high invalidate the pixel.
      stereo_model->operator()(left_pos, right_pos, error_dist);
      if (error_dist > MAX_DISTANCE)
        invalidate(disparity(c,r));
    }
  }
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
    
    // TODO: The ROI details are important, document them!
    
    // 3.0) Actually perform correlation now
    ImageView<pixel_type > disparity;
    std::vector<vw::stereo::SearchParam> zones; 
    // Start off the search at the lowest resolution pyramid level.  This zone covers
    // the entire image and uses the disparity range that was loaded into the class.
    BBox2i initial_disparity_range = BBox2i(0,0,m_search_region.width ()/max_upscaling+1,
                                            m_search_region.height()/max_upscaling+1);
    zones.push_back( SearchParam(bounding_box(left_mask_pyramid[max_pyramid_levels]),
                                 initial_disparity_range) );
    std::cout << "initial_disparity_range = " << initial_disparity_range << std::endl;

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
      vw_out() << "\nProcessing level: " << level << " with size " << disparity.get_size() << std::endl;
      std::cout << "region_offset = " << region_offset << std::endl;
      std::cout << "Number of zones = " << zones.size() << std::endl;

      // 3.1) Process each zone with their refined search estimates
      // - The zones are subregions of the image with similar disparities
      //   that we identified in previous iterations.
      // - Prioritize the zones which take less time so we don't miss
      //   a bunch of tiles because we spent all our time on a slow one.
      std::sort(zones.begin(), zones.end(), SearchParamLessThan()); // Sort the zones, smallest to largest.
      BOOST_FOREACH( SearchParam const& zone, zones ) {

        BBox2i left_region = zone.image_region() + region_offset; // Kernel width offset
        left_region.expand(half_kernel);
        BBox2i right_region = left_region + zone.disparity_range().min();
        right_region.max() += zone.disparity_range().size();

        // Check timing estimate to see if we should go ahead with this zone or quit.
        SearchParam params(left_region, zone.disparity_range());
        double next_elapsed = m_seconds_per_op * params.search_volume();
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
        
        //bool debug = false; // (level == max_pyramid_levels);
        crop(disparity, zone.image_region())
          = calc_disparity2(m_cost_type,
                           crop(left_pyramid [level], left_region),
                           crop(right_pyramid[level], right_region),
                           left_region - left_region.min(),
                           zone.disparity_range().size(), m_kernel_size/*, debug*/);



        // If at the last level and the user requested a left<->right consistency check,
        //   compute right to left disparity.
        if ( m_consistency_threshold >= 0 && level == 0 ) {

          // Check the time again before moving on with this
          SearchParam params2(right_region, zone.disparity_range());
          double next_elapsed = m_seconds_per_op * params2.search_volume();
          if (m_corr_timeout > 0.0 && estim_elapsed + next_elapsed > m_corr_timeout){
            vw_out() << "Tile: " << bbox << " reached timeout: "
                     << m_corr_timeout << " s" << std::endl;
            break;
          }else{
            estim_elapsed += next_elapsed;
          }
          // Compute right to left disparity in this zone
          ImageView<pixel_type> rl_result
            = calc_disparity2(m_cost_type,
                             crop(edge_extend(right_pyramid[level]), right_region),
                             crop(edge_extend(left_pyramid [level]),
                                  left_region - zone.disparity_range().size()),
                             right_region - right_region.min(),
                             zone.disparity_range().size(), m_kernel_size)
            - pixel_type(zone.disparity_range().size());

          // Find pixels where the disparity distance is greater than m_consistency_threshold
          stereo::cross_corr_consistency_check(crop(disparity,zone.image_region()),
                                                rl_result,
                                               m_consistency_threshold, false);
        } // End of last level right to left disparity check

        // Fix the offsets to account for cropping.
        crop(disparity, zone.image_region()) += pixel_type(zone.disparity_range().min());
      } // End of zone loop

      // TODO: How to get the pixel coordinates to absolute values!
      // TODO: Making this work with map projected images will require A LOT of work!
      // Test m_stereo_model!!
      //std::cout << "Applying intersection filter...\n";
      //intersection_filter(disparity, m_stereo_model, );
      //std::cout << "Finished applying intersection filter!\n";

      // Statistics based filter is likely to end up with tiling problems!
      //std::cout << "Applying stats filter...\n";
      //disparity_stats_filter(disparity);
      //std::cout << "Finished applying stats filter!\n";
      

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

  // TODO: The time seems to be VERY sensitive to this function parameters!
        // On the next resolution level, break up the image area into multiple
        // smaller zones with similar disparities.  This helps minimize
        // the total amount of searching done on the image.
        subdivide_regions( disparity, bounding_box(disparity),
                           zones, m_kernel_size );

        
        scaling >>= 1;
        // Scale search range defines the maximum search range that
        // is possible in the next step. This (at lower levels) will
        // actually be larger than the search range that the user
        // specified. We are able to do this because we are taking
        // advantage of the half kernel padding needed at the hight
        // level of the pyramid.
        const size_t next_level = level-1;
        BBox2i scale_search_region(0,0,
                                   right_pyramid[next_level].cols() - left_pyramid[next_level].cols(),
                                   right_pyramid[next_level].rows() - left_pyramid[next_level].rows() );
        std::cout << "scale_search_region = " << scale_search_region << std::endl;
        BBox2i next_zone_size = bounding_box( left_mask_pyramid[level-1] );
        BOOST_FOREACH( SearchParam& zone, zones ) {
          zone.image_region() *= 2;
          zone.image_region().crop( next_zone_size );
          zone.disparity_range() *= 2;
          zone.disparity_range().expand(2); // This is practically required. Our
          // correlation will fail if the search has only one solution.
          //zone.disparity_range().expand(6); // Don't let the range get too tight!
          zone.disparity_range().crop( scale_search_region );
          
        if (0) { // DEBUG
          BBox2i scaled = bbox/2;
          std::ostringstream ostr;
          ostr << "disparity_" << scaled.min()[0] << "_"
               << scaled.min()[1] << "_" << scaled.max()[0] << "_"
               << scaled.max()[1] << "_" << level;
          write_image( ostr.str() + ".tif", pixel_cast<PixelMask<Vector2f> >(disparity) );
          std::ofstream f( (ostr.str() + "_zone.txt").c_str() );
          BOOST_FOREACH( SearchParam& zone, zones ) {
            f << zone.image_region() << " " << zone.disparity_range() << "\n";
          }
          write_image( ostr.str() + "left.tif",  left_pyramid [level] );
          write_image( ostr.str() + "right.tif", right_pyramid[level] );
          write_image( ostr.str() + "lmask.tif", left_mask_pyramid [level] );
          write_image( ostr.str() + "rmask.tif", right_mask_pyramid[level] );
          f.close();
        } // End DEBUG
          
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


