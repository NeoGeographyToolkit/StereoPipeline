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


/// \file BlobIndexThreaded.h
///

#ifndef __SPARSE_IMAGE_VIEW_H__
#define __SPARSE_IMAGE_VIEW_H__

// Standard
#include <vector>

// VW
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/PixelMask.h>

namespace asp {
  // Sparse ImageView
  //////////////////////////////////

  // This holds an image as strips of connected pixels in the row
  // direction. The goal of this is to hold patching pixels in less
  // memory than it would take to store the entire image.

  // SparseImageView is considerably different than other
  // views. SparseImageViews are for reading mostly. They accept only one
  // input via the absorb(..) function. Absorb takes in ImageViews with a
  // start offset vector. Only pixel passing the is_valid() will be
  // stored. So, be sure to use pixel mask to mark the spots not to be
  // recorded

  // For the time being we do not support overwriting previous data. It
  // seems non trivial at this hour.

  template <class ImageT>
  class SparseCompositeView : public vw::ImageViewBase< SparseCompositeView<ImageT> > {
    // The key in our map is the index marking the end of the
    // vector. This somewhat confusing method is to allow us better use of
    // the container's search method.
    typedef std::map<vw::int32,std::vector<typename ImageT::pixel_type > > map_type;
    std::vector<map_type> m_data;
    bool m_allow_overlap;
    ImageT m_under_image;

  public:
    typedef typename ImageT::pixel_type pixel_type;
    typedef pixel_type result_type;
    typedef vw::ProceduralPixelAccessor<SparseCompositeView<ImageT> > pixel_accessor;

    // Standard stuff
    SparseCompositeView( vw::ImageViewBase<ImageT> const& under_image,
                         bool allow_overlap = false ) :
      m_data(under_image.impl().rows()), m_allow_overlap(allow_overlap),
      m_under_image(under_image.impl()) {}

    inline vw::int32 cols() const { return m_under_image.cols(); }
    inline vw::int32 rows() const { return m_under_image.rows(); }
    inline vw::int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this,0,0); }

    inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 p=0 ) const {

      typename map_type::const_iterator it;
      it = m_data[j].upper_bound(i);
      if ( it != m_data[j].end() ) {
        vw::int32 s_idx = it->first - it->second.size();
        if ( i < s_idx )
          return m_under_image( i, j, p );
        else
          return it->second[i-s_idx];
      }
      return m_under_image( i, j, p );
    }

    typedef SparseCompositeView<ImageT> prerasterize_type;
    inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const { return *this; }
    template <class DestT>
    inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }

    // Difficult insertation
    template <class InputT>
    void absorb( vw::Vector2i starting_index,
                 vw::ImageViewBase<InputT> const& image_base ) {
      using namespace vw;
      InputT image = image_base.impl();
      VW_DEBUG_ASSERT( starting_index[0] >= 0 && starting_index[1] >= 0,
                       NoImplErr() << "SparseCompositeView doesn't support insertation behind image origin.\n" );

      // Building temporary data structure
      std::vector<std::list<int32> > t_row_end(image.rows());
      std::vector<std::list<std::vector<pixel_type> > > t_row_data(image.rows());

      for ( int32 r = 0; r < image.rows(); ++r ) {
        std::vector<pixel_type> temp;
        bool building_segment = false;
        for ( int32 c = 0; c < image.cols(); ++c ) {
          if ( is_valid( image(c,r) ) )
            if ( !building_segment ) {
              building_segment = true;
              temp.push_back( image(c,r) );
            } else
              temp.push_back( image(c,r) );
          else if ( building_segment ) {
            t_row_end[r].push_back(c);
            building_segment = false;
            t_row_data[r].push_back( temp );
            temp.clear();
          }
        }
        if ( building_segment ) {
          t_row_end[r].push_back(image.cols());
          t_row_data[r].push_back(temp);
          temp.clear();
        }
      }

      // Fix x offset
      for ( uint32 i = 0; i < t_row_end.size(); ++i )
        for ( std::list<int32>::iterator iter = t_row_end[i].begin();
              iter != t_row_end[i].end(); ++iter )
          *iter += starting_index[0];

      // Inserting into global data set
      if ( int32(m_data.size()) < starting_index[1]+image.rows() )
        m_data.resize( starting_index[1]+image.rows() );
      for ( int32 t_i=0, m_i=starting_index[1];
            t_i < image.rows(); ++t_i, ++m_i ) {

        // Insert a strip at a time
        std::list<int32>::const_iterator t_e_iter = t_row_end[t_i].begin();
        typename std::list<std::vector<pixel_type> >::const_iterator t_d_iter = t_row_data[t_i].begin();
        while ( t_e_iter != t_row_end[t_i].end() ) {
          typename map_type::iterator it = m_data[m_i].lower_bound( *t_e_iter );
          if ( it == m_data[m_i].end() ) { // Doesn't appear anything is this far right
            m_data[m_i][*t_e_iter] = *t_d_iter;
          } else if ( it == m_data[m_i].begin() ) {
            // Nothing in front of this point.
            int32 m_start_idx = it->first - it->second.size();
            if ( *t_e_iter > m_start_idx )
              vw_throw( NoImplErr() << "SparseCompositeView at this time doesn't allow insert over existing data.\n");
            else if ( *t_e_iter == m_start_idx ) // Append to front
              it->second.insert( it->second.begin(),
                                 t_d_iter->begin(),
                                 t_d_iter->end() );
            else // Whole new key
              m_data[m_i][*t_e_iter] = *t_d_iter;
          } else {
            // Somewhere in the middle
            typename map_type::const_iterator prev_it = it;
            prev_it--;
            int32 t_start_idx = *t_e_iter - t_d_iter->size();
            int32 m_start_idx = it->first - it->second.size();
            if ( (t_start_idx < prev_it->first) ||
                 (*t_e_iter > m_start_idx) )
              vw_throw( NoImplErr() << "SparseCompositeView at this time doesn't allow insert over existing data.\n");
            else if ( *t_e_iter == m_start_idx ) // Append to front
              it->second.insert( it->second.begin(),
                                 t_d_iter->begin(),
                                 t_d_iter->end() );
            else // Whole new key
              m_data[m_i][*t_e_iter] = *t_d_iter;
          }

          ++t_e_iter;
          ++t_d_iter;
        }
      }
    }

    // Debug structure
    void print_structure() const {
      using namespace vw;
      vw_out() << "SparseCompositeView Structure:\n";
      for ( uint32 i = 0; i < m_data->size(); ++i ) {
        vw_out() << i << " | ";
        for ( typename map_type::const_iterator it = (*m_data)[i].begin();
              it != (*m_data)[i].end(); ++it ) {
          int32 start = it->first - it->second.size();
          vw_out() << "(" << start << "->" << it->first << ")";
        }
        vw_out() << "\n";
      }
    }

  };

} // end namespace asp

#endif//__SPARSE_IMAGE_VIEW_H__
