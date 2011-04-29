// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/Core/ThreadPool.h>
#include <vw/Image/MaskViews.h>
#include <boost/foreach.hpp>

#ifndef __ASP_CORE_THREADEDEDGEMASK_H__
#define __ASP_CORE_THREADEDEDGEMASK_H__

namespace asp {

  template <class ViewT>
  class ThreadedEdgeMaskView : public vw::ImageViewBase<ThreadedEdgeMaskView<ViewT> > {

    vw::ImageViewRef<typename ViewT::pixel_type> m_view;
    typedef boost::shared_array<vw::int32> SharedArray;
    SharedArray m_left, m_right, m_top, m_bottom;

    // Determines if a single pixel is valid.
    inline bool valid(vw::int32 i, vw::int32 j) const {
      if ( i > m_left[j] && i < m_right[j] && j > m_top[i] && j < m_bottom[i] )
        return true;
      else
        return false;
    }

    // Task that checks individual blocks for edges
    class EdgeMaskTask : public vw::Task, private boost::noncopyable {
      vw::ImageViewRef<typename ViewT::pixel_type> m_view;
      typename ViewT::pixel_type m_mask_value;
      vw::BBox2i m_bbox;   // Region of image we're working in
      typedef std::vector<vw::int32> Array;
      typedef boost::shared_array<vw::int32> SharedArray;
      SharedArray g_left, g_right, g_top, g_bottom;
      Array m_left, m_right, m_top, m_bottom;

      // This how much we increment after we test a pixel. Set to 1 if
      // you wish to test every pixel.
      const static vw::int32 STEP_SIZE=5;
    public:
      EdgeMaskTask(vw::ImageViewRef<typename ViewT::pixel_type> const& view,
                   typename ViewT::pixel_type mask_value,
                   vw::BBox2i bbox, SharedArray left, SharedArray right,
                   SharedArray top, SharedArray bottom ) :
        m_view(view), m_mask_value(mask_value), m_bbox(bbox), g_left(left), g_right(right), g_top(top), g_bottom(bottom), m_left( m_bbox.height() ), m_right( m_bbox.height() ), m_top( m_bbox.width() ), m_bottom( m_bbox.width() ) {

        std::fill( m_left.begin(), m_left.end(), -1 );
        std::fill( m_right.begin(), m_right.end(), -1 );
        std::fill( m_top.begin(), m_top.end(), -1 );
        std::fill( m_bottom.begin(), m_bottom.end(), -1 );
      }

      void operator()() {
        using namespace vw;

        // Rasterizing local tile
        ImageView<typename ViewT::pixel_type> copy =
          crop( m_view, m_bbox );

        { // Detecting Edges
          // Search left and right side
          for ( int32 j = 0; j < copy.rows(); ++j ) {
            int32 i = 0;
            while ( i < copy.cols() && copy(i,j) == m_mask_value )
              i += STEP_SIZE;
            if ( i > 0 ) i -= STEP_SIZE;
            while ( i < copy.cols() && copy(i,j) == m_mask_value )
              ++i;
            if ( i > 0 ) --i;
            // Early exit condition if entire row was nodata
            if ( i == copy.cols() -1 )
              continue; // We're keeping left and right as -1
            m_left[j] = i;

            i = copy.cols() - 1;
            while ( i >= 0 && copy(i,j) == m_mask_value )
              i -= STEP_SIZE;
            if ( i < copy.cols() - 1 )
              i += STEP_SIZE;
            while ( i >= 0 && copy(i,j) == m_mask_value )
              --i;
            if ( i < copy.cols() - 1 )
              ++i;
            m_right[j] = i;
          }

          for ( int32 i = 0; i < copy.cols(); ++i ) {
            int32 j = 0;
            while ( j < copy.rows() && copy(i,j) == m_mask_value )
              j += STEP_SIZE;
            if ( j > 0 )
              j -= STEP_SIZE;
            while ( j < copy.rows() && copy(i,j) == m_mask_value )
              ++j;
            if ( j > 0 )
              --j;
            // Early exit condition if entire column was nodata
            if ( j == copy.rows() - 1 )
              continue; // We're keeping top and bottom as -1
            m_top[i] = j;

            j = copy.rows()-1;
            while ( j >= 0 && copy(i,j) == m_mask_value )
              j -= STEP_SIZE;
            if ( j < copy.rows()-1 )
              j += STEP_SIZE;
            while ( j >= 0 && copy(i,j) == m_mask_value )
              --j;
            if ( j < copy.rows()-1 )
              ++j;
            m_bottom[i] = j;
          }
        }

        { // Merging result back into global perspective
          int32 l = 0;
          for ( int32 j = m_bbox.min()[1];
                j < m_bbox.max()[1]; j++ ) {
            if ( m_left[l] == -1 ) {
              l++; continue;
            }
            g_left[j] = std::min( m_left[l] + m_bbox.min()[0],
                                  g_left[j] );
            g_right[j] = std::max( m_right[l] + m_bbox.min()[0],
                                   g_right[j] );
            l++;
          }

          l = 0;
          for ( int32 i = m_bbox.min()[0];
                i < m_bbox.max()[0]; i++ ) {
            if ( m_top[l] == -1 ) {
              l++; continue;
            }
            g_top[i] = std::min( m_top[l] + m_bbox.min()[1],
                                 g_top[i] );
            g_bottom[i] = std::max( m_bottom[l] + m_bbox.min()[1],
                                    g_bottom[i] );
            l++;
          }
        }
      }
    };

  public:

    typedef typename ViewT::pixel_type orig_pixel_type;
    typedef typename boost::remove_cv<typename boost::remove_reference<orig_pixel_type>::type>::type unmasked_pixel_type;
    typedef vw::PixelMask<unmasked_pixel_type> pixel_type;
    typedef vw::PixelMask<unmasked_pixel_type> result_type;
    typedef vw::ProceduralPixelAccessor<ThreadedEdgeMaskView> pixel_accessor;

    ThreadedEdgeMaskView( ViewT const& view,
                          unmasked_pixel_type const& mask_value,
                          vw::int32 block_size,
                          const vw::ProgressCallback &progress_callback = vw::ProgressCallback::dummy_instance() ) :
      m_view(view), m_left( new vw::int32[view.rows()]), m_right( new vw::int32[view.rows()] ), m_top( new vw::int32[view.cols()] ), m_bottom( new vw::int32[view.cols()] ) {
      using namespace vw;

      std::fill( m_left.get(), m_left.get()+view.rows(), view.cols() );
      std::fill( m_right.get(), m_right.get()+view.rows(), 0 );
      std::fill( m_top.get(), m_top.get()+view.cols(), view.rows() );
      std::fill( m_bottom.get(), m_bottom.get()+view.cols(), 0 );

      // Calculating edges in parallel
      FifoWorkQueue queue( vw_settings().default_num_threads() );

      std::vector<BBox2i> bboxes =
        image_blocks( m_view, block_size, block_size );

      BOOST_FOREACH( BBox2i const& box, bboxes ) {
        boost::shared_ptr<EdgeMaskTask> task(new EdgeMaskTask(m_view, mask_value, box, m_left, m_right, m_top, m_bottom ) );
        queue.add_task(task);
      }
      queue.join_all();
    }

    // Specialized deep copy constructor
    ThreadedEdgeMaskView( ThreadedEdgeMaskView const& other,
                          vw::BBox2i const& box ) :
      m_view( constant_view(typename ViewT::pixel_type(0),
                            box.width(), box.height() ) ) {
      // Note to future authors: Actually copying other's m_view
      // causes locking issues on write.

      using namespace vw;
      SharedArray left( new int32[box.height()] ); m_left = left;
      SharedArray right( new int32[box.height()] ); m_right = right;
      SharedArray top( new int32[box.width()] ); m_top = top;
      SharedArray bottom( new int32[box.width()] ); m_bottom = bottom;

      // Copy only sections that we need
      std::copy( other.m_left.get()+box.min()[1],
                 other.m_left.get()+box.max()[1], m_left.get() );
      std::copy( other.m_right.get()+box.min()[1],
                 other.m_right.get()+box.max()[1], m_right.get() );
      std::copy( other.m_top.get()+box.min()[0],
                 other.m_top.get()+box.max()[0], m_top.get() );
      std::copy( other.m_bottom.get()+box.min()[0],
                 other.m_bottom.get()+box.max()[0], m_bottom.get() );

      // Modify to new coordinate system
      std::for_each( m_left.get(), m_left.get()+box.height(),
                     ArgValInPlaceSumFunctor<int32>( -box.min()[0] ) );
      std::for_each( m_right.get(), m_right.get()+box.height(),
                     ArgValInPlaceSumFunctor<int32>( -box.min()[0] ) );
      std::for_each( m_top.get(), m_top.get()+box.width(),
                     ArgValInPlaceSumFunctor<int32>( -box.min()[1] ) );
      std::for_each( m_bottom.get(), m_bottom.get()+box.width(),
                     ArgValInPlaceSumFunctor<int32>( -box.min()[1] ) );
    }

    inline vw::int32 cols() const { return m_view.cols(); }
    inline vw::int32 rows() const { return m_view.rows(); }
    inline vw::int32 planes() const { return m_view.planes(); }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 p=0 ) const {
      if ( this->valid(i,j) )
        return pixel_type(m_view(i,j,p));
      else
        return pixel_type();
    }

    typedef vw::CropView<ThreadedEdgeMaskView<ViewT > > prerasterize_type;
    inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const {
      // We are deep copying a small section of ThreadedEdgeMaskView
      // and then uncropping back to original coordinates.
      return vw::crop(ThreadedEdgeMaskView<ViewT>( *this, bbox ),
                      -bbox.min()[0], -bbox.min()[1],
                      this->cols(), this->rows() );
    }

    template <class DestT> inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }
  };

  template <class ViewT>
  ThreadedEdgeMaskView<ViewT> threaded_edge_mask( vw::ImageViewBase<ViewT> const& v,
                                                  typename ViewT::pixel_type value,
                                                  vw::int32 block_size,
                                                  const vw::ProgressCallback &progress_callback = vw::ProgressCallback::dummy_instance() ) {
    return ThreadedEdgeMaskView<ViewT>( v.impl(), value, block_size, progress_callback );
  }
}

namespace vw {
  template <class ViewT>
  struct IsMultiplyAccessible<asp::ThreadedEdgeMaskView<ViewT> > : public true_type {};
}

#endif//__ASP_CORE_THREADEDEDGEMASK_H__
