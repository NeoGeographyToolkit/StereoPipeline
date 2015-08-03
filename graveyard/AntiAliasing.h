// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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


#ifndef __ASP_CORE_ANTIALIASING_H__
#define __ASP_CORE_ANTIALIASING_H__

#include <vw/Core.h>
#include <vw/Image.h>

#include <boost/foreach.hpp>

namespace asp {

  // Cache Tile Aware Raster View.
  //
  // Mostly just a fancy pass through that provides a fancy access
  // pattern that should match the input's tiles so we are not
  // accessing disk too often.
  template <class ImageT>
  class CacheTileAwareView : public vw::ImageViewBase< CacheTileAwareView<ImageT> > {
    ImageT m_child;
    vw::Vector2i m_tile_size;

  public:
    typedef typename ImageT::pixel_type pixel_type;
    typedef typename ImageT::result_type result_type;
    typedef typename ImageT::pixel_accessor pixel_accessor;

    CacheTileAwareView( ImageT const& image, vw::Vector2i tile_size ) : m_child( image ), m_tile_size( tile_size ) {}

    inline vw::int32 cols() const { return m_child.cols(); }
    inline vw::int32 rows() const { return m_child.rows(); }
    inline vw::int32 planes() const { return m_child.planes(); }

    inline pixel_accessor origin() const { return m_child.origin(); }

    inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 p=0 ) const {
      return m_child(i,j,p);
    }

    typedef vw::CropView<vw::ImageView<pixel_type> > prerasterize_type;
    inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const {
      using namespace vw;

      // Fancy rasterize in section of m_tile_size
      std::vector<BBox2i> subregions =
        image_blocks( bbox, m_tile_size.x(), m_tile_size.y() );
      ImageView<pixel_type> output( bbox.width(), bbox.height() );
      BOOST_FOREACH( BBox2i const& sub, subregions ) {
        crop( output, sub - bbox.min() ) = crop( m_child, sub );
      }
      return crop( output, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
    }
    template <class DestT> inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }
  };

  template <class ImageT>
  inline CacheTileAwareView<ImageT> cache_tile_aware_render( ImageT const& v, vw::Vector2i const& tile_size ) {
    return CacheTileAwareView<ImageT>( v, tile_size );
  }

  // Weighted Summation filter for antialiasing. This insures that we
  // are not summing with masked pixels.
  //
  // The correlators perform a different trick to avoid this
  // problem. They set invalid pixels to a mean pixel value when
  // performing the gaussian.
  //
  // This implementation uses a separable convolution to speed up
  // processing in the in the event of a great reduction.
  template <class ImageT>
  class WeightedSummationView : public vw::ImageViewBase<WeightedSummationView<ImageT> > {
    ImageT m_image;
    vw::int32 m_reduce_amt;

  public:
    typedef typename ImageT::pixel_type pixel_type;
    typedef pixel_type result_type;
    typedef vw::ProceduralPixelAccessor<WeightedSummationView<ImageT> > pixel_accessor;

    WeightedSummationView( ImageT const& image, vw::int32 reduce_amt ) : m_image(image), m_reduce_amt(reduce_amt) {}

    // Standard info
    inline vw::int32 cols() const { return m_image.cols(); }
    inline vw::int32 rows() const { return m_image.rows(); }
    inline vw::int32 planes() const { return m_image.planes(); }
    inline pixel_accessor origin() const { return pixel_accessor( *this ); }

    // Returns the pixel at the given position in the given plane.
    inline result_type operator()( vw::int32 x, vw::int32 y, vw::int32 p=0 ) const {
      vw_throw( vw::NoImplErr() << "WeightedSummationView doesn't support single pixel access.");
    }

    typedef vw::CropView<vw::ImageView<pixel_type> > prerasterize_type;
    inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const {
      using namespace vw;
      ImageView<pixel_type> dest( bbox.width(), bbox.height(), m_image.planes() );
      rasterize(dest, bbox);
      return prerasterize_type( dest, BBox2i( -bbox.min().x(), -bbox.min().y(),
                                              m_image.cols(), m_image.rows() ) );
    }

    template <class DestT>
    void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const {
      using namespace vw;

      typedef typename UnmaskedPixelType<pixel_type>::type unmasked_type;

      typedef typename SumType<unmasked_type, unmasked_type >::type sum_type;
      typedef typename ImageView<pixel_type>::pixel_accessor IPA;
      typedef typename ImageView<sum_type>::pixel_accessor WPA;
      typedef typename ImageView<int32>::pixel_accessor CPA;

      BBox2i child_bbox = bbox;
      child_bbox.max() += Vector2i( m_reduce_amt - 1,
                                    m_reduce_amt - 1 );
      ImageView<pixel_type> src_buf = edge_extend(m_image,child_bbox,ZeroEdgeExtension());

      ImageView<sum_type> work( bbox.width(), bbox.height() + m_reduce_amt - 1 );
      ImageView<int32> count( bbox.width(), bbox.height() + m_reduce_amt - 1 );

      {// Sum in the X direction
        IPA irow = src_buf.origin();
        WPA wrow = work.origin();
        CPA crow = count.origin();
        for ( int32 y = 0; y < work.rows(); y++ ) {
          // Start sum with first kernel
          IPA icol = irow;
          IPA icol_lookahead = icol;
          WPA wcol = wrow;
          CPA ccol = crow;

          sum_type sum;
          sum *= 0; // Needs this so it works for both scalar and Vector vals
          validate( sum );

          int32 count = 0;
          for ( int32 i = m_reduce_amt; i; i-- ) {
            if ( is_valid( *icol_lookahead ) ) {
              count++;
              sum += ( *icol_lookahead ).child();
            }
            icol_lookahead.next_col();
          }
          *wcol = sum;
          *ccol = count;
          wcol.next_col();
          ccol.next_col();

          // Fast sum the rest of the pixels using prior results
          for ( int32 x = 1; x < work.cols(); x++ ) {
            if ( is_valid( *icol ) ) {
              count--;
              sum -= ( *icol ).child();
            }
            if ( is_valid( *icol_lookahead ) ) {
              count++;
              sum += ( *icol_lookahead ).child();
            }
            *wcol = sum;
            *ccol = count;
            icol.next_col();
            icol_lookahead.next_col();
            wcol.next_col();
            ccol.next_col();
          }

          // Increment to the next row
          irow.next_row();
          wrow.next_row();
          crow.next_row();
        } // End of y loop for sum in X direction
      }

      ImageView<pixel_type> output( bbox.width(), bbox.height() );
      { // Sum in the Y direction
        WPA wcol = work.origin();
        CPA ccol = count.origin();
        IPA ocol = output.origin();
        for ( int32 x = 0; x < bbox.width(); x++ ) {
          // Start sequence by hand summing the first kernel
          WPA wrow = wcol;
          WPA wrow_lookahead = wcol;
          CPA crow = ccol;
          CPA crow_lookahead = ccol;
          IPA orow = ocol;

          sum_type sum;
          sum *= 0; // Needs this so it works for both scalar and Vector vals
          validate( sum );

          int32 count = 0;
          for ( int32 i = m_reduce_amt; i; i-- ) {
            sum += *wrow_lookahead;
            count += *crow_lookahead;
            wrow_lookahead.next_row();
            crow_lookahead.next_row();
          }
          *orow = sum / count;
          if ( count < 1 )
            invalidate( *orow );
          orow.next_row();

          // Fast sum the rest of the pixels in the column
          for ( int32 y = 1; y < bbox.height(); y++ ) {
            sum += *wrow_lookahead;
            sum -= *wrow;
            count += *crow_lookahead;
            count -= *crow;

            *orow = sum / count;
            if ( count < 1 )
              invalidate( *orow );
            wrow.next_row();
            wrow_lookahead.next_row();
            crow.next_row();
            crow_lookahead.next_row();
            orow.next_row();
          }

          // increment to next col
          wcol.next_col();
          ccol.next_col();
          ocol.next_col();
        }
      }
      dest = output;
    }
  };

  template <class ImageT>
  inline WeightedSummationView<ImageT> weighted_summation( vw::ImageViewBase<ImageT> const& v, vw::int32 reduction_amt ) {
    return WeightedSummationView<ImageT>( v.impl(), reduction_amt );
  }

  // Weighted Summation filter for antialiasing. This insures that we
  // are not summing with masked pixels. Sadly Gaussian filters will
  // always do this, thus the need for a custom type.
  //
  // The correlators perform a different trick to avoid this
  // problem. They set invalid pixels to a mean pixel value when
  // performing the gaussian.
  class WeightedAAFilter : public vw::UnaryReturnTemplateType<vw::PixelTypeFromPixelAccessor> {
    vw::int32 m_reduce_amt;
  public:
    WeightedAAFilter( vw::int32 reduce_amt ) : m_reduce_amt( reduce_amt ) {}

    vw::BBox2i work_area() const { return vw::BBox2i(0,0,m_reduce_amt,m_reduce_amt); }

    template <class PixelAccessorT>
    typename PixelAccessorT::pixel_type
    operator()( PixelAccessorT acc ) const {
      using namespace vw;

      typedef typename SumType<typename PixelAccessorT::pixel_type,
                               typename PixelAccessorT::pixel_type >::type sum_type;
      sum_type sum;
      sum *= 0; // Needs this so it works for both scalar and Vector vals
      validate( sum );

      int32 count = 0;
      for ( int32 r=m_reduce_amt; r; --r ) {
        PixelAccessorT col_acc = acc;
        for ( int32 c=m_reduce_amt; c; --c) {
          if ( is_valid( *col_acc ) ) {
            count++;
            sum += (*col_acc);
          }
          col_acc.next_col();
        }
        acc.next_row();
      }
      if ( !count )
        invalidate( sum );
      return sum / count;
    }
  };

  template <class ViewT>
  vw::TransformView<vw::InterpolationView<WeightedSummationView<ViewT>, vw::NearestPixelInterpolation>, vw::ResampleTransform>
  resample_aa( vw::ImageViewBase<ViewT> const& input, double factor ) {
    using namespace vw;
    typedef WeightedSummationView<ViewT> inner_type;
    typedef TransformView<InterpolationView<inner_type, NearestPixelInterpolation>, ResampleTransform> return_type;
    return return_type( InterpolationView<inner_type, NearestPixelInterpolation>( weighted_summation( input.impl(), int32(1.0/factor) ) ),
                        ResampleTransform( factor, factor ),
                        int32(.5+(input.impl().cols()*factor)),
                        int32(.5+(input.impl().rows()*factor)) );
  }

}

#endif//__ASP_CORE_ANTIALIASING_H__
