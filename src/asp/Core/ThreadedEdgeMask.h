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


#include <vw/Core/System.h>
#include <vw/Core/ThreadPool.h>
#include <vw/Image/MaskViews.h>
#include <boost/foreach.hpp>

#ifndef __ASP_CORE_THREADEDEDGEMASK_H__
#define __ASP_CORE_THREADEDEDGEMASK_H__

namespace asp {

  /// Quick way to check which pixels are inside the valid input mask??
  template <class ViewT>
  class ThreadedEdgeMaskView : public vw::ImageViewBase<ThreadedEdgeMaskView<ViewT> > {

    ViewT m_view;

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
      ViewT m_view;
      typename ViewT::pixel_type m_mask_value;
      vw::BBox2i m_bbox;   // Region of image we're working in
      typedef std::vector<vw::int32> Array;
      typedef boost::shared_array<vw::int32> SharedArray;
      SharedArray g_left, g_right, g_top, g_bottom;
      Array       m_left, m_right, m_top, m_bottom;
      // This how much we increment after we test a pixel. Set to 1 if
      // you wish to test every pixel.
      const vw::int32 STEP_SIZE;
    public:
      EdgeMaskTask(ViewT const& view,
                   typename ViewT::pixel_type mask_value,
                   vw::int32 search_step,
                   vw::BBox2i bbox, SharedArray left, SharedArray right,
                   SharedArray top, SharedArray bottom ) :
        m_view(view), m_mask_value(mask_value), m_bbox(bbox), 
        g_left(left), g_right(right), g_top(top), g_bottom(bottom), 
        m_left( m_bbox.height() ), m_right( m_bbox.height() ), 
        m_top( m_bbox.width() ), m_bottom( m_bbox.width() ), STEP_SIZE(search_step) {

        std::fill( m_left.begin  (), m_left.end  (), -1 );
        std::fill( m_right.begin (), m_right.end (), -1 );
        std::fill( m_top.begin   (), m_top.end   (), -1 );
        std::fill( m_bottom.begin(), m_bottom.end(), -1 );
      }

      void operator()() {
        using namespace vw;

        // Rasterizing local tile
        ImageView<typename ViewT::pixel_type> copy( crop(m_view, m_bbox ) );

        { // Detecting Edges
          // Search left and right side
          for ( int32 j = 0; j < copy.rows(); ++j ) { // Loop up through the rows in the local tile
            int32 i = 0;
            while ( i < copy.cols() && copy(i,j) == m_mask_value ) // Move from left to right in row by STEP_SIZE
              i += STEP_SIZE;                                      //    until we hit an invalid pixel
            if ( i > 0 ) i -= STEP_SIZE;                           // Walk back one step if we are not at col 0
            while ( i < copy.cols() && copy(i,j) == m_mask_value ) // Now do the same thing but in steps of 1
              ++i;
            if ( i > 0 ) --i;
            // Early exit condition if entire row was nodata
            if ( i == copy.cols() -1 )
              continue; // We're keeping left and right as -1
            m_left[j] = i;                                         // Now we have the left-most valid column in the row

            i = copy.cols() - 1;                                   // Do the same thing going right to left
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
                                                                   // Now find the first valid rows from the bottom
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

            j = copy.rows()-1;                                     // And the first valid rows from the top
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
          for ( int32 j = m_bbox.min()[1];                      // Loop through rows
                j < m_bbox.max()[1]; j++ ) {
            if ( m_left[l] == -1 ) {                            // Skip rows with no pixels
              l++; continue;
            }
            g_left[j] = std::min( m_left[l] + m_bbox.min()[0],  // Add in the bounding box column
                                  g_left[j] );
            g_right[j] = std::max( m_right[l] + m_bbox.min()[0],
                                   g_right[j] );
            l++;
          }

          l = 0;
          for ( int32 i = m_bbox.min()[0];                       // Loop through columns
                i < m_bbox.max()[0]; i++ ) {
            if ( m_top[l] == -1 ) {                              // Skip columns with no pixels
              l++; continue;
            }
            g_top[i] = std::min( m_top[l] + m_bbox.min()[1],     // Add in the bounding box row
                                 g_top[i] );
            g_bottom[i] = std::max( m_bottom[l] + m_bbox.min()[1],
                                    g_bottom[i] );
            l++;
          }
        }
      }
    };

    // Specialized deep copy constructor (private)
    template <class OViewT>
    ThreadedEdgeMaskView( ViewT const& view,
                          ThreadedEdgeMaskView<OViewT> const& other,
                          vw::BBox2i const& box ) :
      m_view( view ), m_left(new vw::int32[box.height()]), m_right(new vw::int32[box.height()]),
      m_top(new vw::int32[box.width()]), m_bottom(new vw::int32[box.width()]) {
      using namespace vw;

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

  public:

    typedef typename ViewT::pixel_type orig_pixel_type;
    typedef typename boost::remove_cv<typename boost::remove_reference<orig_pixel_type>::type>::type unmasked_pixel_type;
    typedef vw::PixelMask<unmasked_pixel_type> pixel_type;
    typedef vw::PixelMask<unmasked_pixel_type> result_type;
    typedef vw::ProceduralPixelAccessor<ThreadedEdgeMaskView> pixel_accessor;

    ThreadedEdgeMaskView( ViewT const& view,
                          unmasked_pixel_type const& mask_value,
                          vw::int32 mask_buffer = 0,
                          vw::int32 block_size = vw::vw_settings().default_tile_size()) :
      m_view(view), m_left( new vw::int32[view.rows()]), m_right( new vw::int32[view.rows()] ),
      m_top( new vw::int32[view.cols()] ), m_bottom( new vw::int32[view.cols()] ) {
      using namespace vw;

      std::fill( m_left.get(),   m_left.get  ()+view.rows(), view.cols() );
      std::fill( m_right.get(),  m_right.get ()+view.rows(), 0           );
      std::fill( m_top.get(),    m_top.get   ()+view.cols(), view.rows() );
      std::fill( m_bottom.get(), m_bottom.get()+view.cols(), 0           );

      // Calculating edges in parallel
      FifoWorkQueue queue( vw_settings().default_num_threads() );

      std::vector<BBox2i> bboxes = subdivide_bbox( m_view, block_size, block_size );

      // Figure out an ideal search step size. Smaller means we're
      // more likely to catch small features. Bigger step size means
      // will move a lot faster.
      int32 search_step = norm_2(Vector2i(m_view.cols(),m_view.rows())) / 500;
      if (search_step < 1 )
        search_step = 1;
      if ( search_step > 10 ) 
        search_step = 10;
      VW_OUT(DebugMessage, "threadededgemask") << "Setting search step to " << search_step << std::endl;

      // Find the outermost valid pixel coming in from each line/direction.
      BOOST_FOREACH( BBox2i const& box, bboxes ) {
        VW_OUT(DebugMessage, "threadededgemask") << "Created EdgeMaskTask for " << box << std::endl;
        boost::shared_ptr<EdgeMaskTask> task(new EdgeMaskTask(m_view, mask_value, search_step, box, 
                                                              m_left, m_right, m_top, m_bottom ) );
        queue.add_task(task);
      }
      queue.join_all(); // Wait for all tasks to complete

      // Erode the valid area by mask_buffer size on each side.
      std::for_each( m_left.get(), m_left.get()+view.rows(),
                     vw::ArgValInPlaceSumFunctor<vw::int32>( mask_buffer ) );
      std::for_each( m_right.get(), m_right.get()+view.rows(),
                     vw::ArgValInPlaceDifferenceFunctor<vw::int32>( mask_buffer ) );
      std::for_each( m_top.get(), m_top.get()+view.cols(),
                     vw::ArgValInPlaceSumFunctor<vw::int32>( mask_buffer ) );
      std::for_each( m_bottom.get(), m_bottom.get()+view.cols(),
                     vw::ArgValInPlaceDifferenceFunctor<vw::int32>( mask_buffer ) );
    }

    inline vw::int32 cols  () const { return m_view.cols  (); }
    inline vw::int32 rows  () const { return m_view.rows  (); }
    inline vw::int32 planes() const { return m_view.planes(); }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 p=0 ) const {
      if ( this->valid(i,j) )
        return pixel_type(m_view(i,j,p));
      else
        return pixel_type();
    }

    vw::BBox2i active_area() const {
      return vw::BBox2i( vw::Vector2i(*std::min_element(&m_left  [0], &m_left  [rows()])+1,
                                      *std::min_element(&m_top   [0], &m_top   [cols()])+1),
                         vw::Vector2i(*std::max_element(&m_right [0], &m_right [rows()]),
                                      *std::max_element(&m_bottom[0], &m_bottom[cols()])) );
    }

    typedef vw::CropView<ThreadedEdgeMaskView<vw::CropView<typename ViewT::prerasterize_type> > > prerasterize_type;
    inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const {
      // We are deep copying a small section of ThreadedEdgeMaskView
      // and then uncropping back to original coordinates.
      typedef ThreadedEdgeMaskView<vw::CropView<typename ViewT::prerasterize_type> > inner_type;
      return vw::crop(inner_type( vw::crop(m_view.prerasterize(bbox),bbox),
                                  *this, bbox ),
                      -bbox.min()[0], -bbox.min()[1],
                      this->cols(), this->rows() );
    }

    template <class DestT> inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }

    // Friend other types of threaded edge mask
    template <class OViewT>
    friend class ThreadedEdgeMaskView;
  };

  template <class ViewT>
  ThreadedEdgeMaskView<ViewT> threaded_edge_mask( vw::ImageViewBase<ViewT> const& v,
                                                  typename ViewT::pixel_type value,
                                                  vw::int32 mask_buffer = 0,
                                                  vw::int32 block_size = vw::vw_settings().default_tile_size()) {
    return ThreadedEdgeMaskView<ViewT>( v.impl(), value, mask_buffer, block_size );
  }
}

namespace vw {
  template <class ViewT>
  struct IsMultiplyAccessible<asp::ThreadedEdgeMaskView<ViewT> > : public true_type {};
}

#endif//__ASP_CORE_THREADEDEDGEMASK_H__
