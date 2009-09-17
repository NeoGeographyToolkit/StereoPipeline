// __BEGIN_LICENSE__
//
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
//
// Copyright 2008 Carnegie Mellon University. All rights reserved.
//
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
//
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file BlobIndexThreaded.h
///

#ifndef __SPARSE_IMAGE_VIEW_H__
#define __SPARSE_IMAGE_VIEW_H__

// Standard
#include <vector>

// VW
#include <vw/Math/Vector.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/PixelMask.h>

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

template <class PixelT>
class SparseView : public vw::ImageViewBase< SparseView<PixelT> > {

 private:
  // The key in our map is the index marking the end of the
  // vector. This somewhat confusing method is to allow us better use of
  // the container's search method.
  typedef std::map<vw::int32,std::vector<typename vw::UnmaskedPixelType<PixelT>::type > > map_type;
  boost::shared_ptr<std::vector<map_type> > m_data;
  bool m_allow_overlap;

  // Group leaves together if possible
  void refactor( void ) {}

 public:
  typedef typename vw::UnmaskedPixelType<PixelT>::type pixel_type;
  typedef typename vw::UnmaskedPixelType<PixelT>::type result_type;
  typedef vw::ProceduralPixelAccessor<SparseView<PixelT> > pixel_accessor;

  // Number of filled points in SparseView
  vw::uint32 size( void ) const {
    return 0; // FIX THIS!
  }

  // Standard stuff
  SparseView( bool allow_overlap = false ) :
  m_data(new std::vector<map_type>() ), m_allow_overlap(allow_overlap) {}

  inline vw::int32 cols() const {
    vw::int32 max_col = 0;
    typename map_type::const_reverse_iterator rit;
    for ( vw::uint32 i = 0; i < m_data->size(); i++ )
      if ( !(*m_data)[i].empty() ) {
        rit = (*m_data)[i].rbegin();
        if ( rit->first > max_col )
          max_col = rit->first;
      }
    return max_col;
  }
  inline vw::int32 rows() const { return m_data->size(); }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this,0,0); }

  inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 p=0 ) const {
    typename map_type::const_iterator it;
    it = (*m_data)[j].upper_bound(i);
    if ( it != (*m_data)[j].end() ) {
      vw::int32 s_idx = it->first - it->second.size();
      if ( i < s_idx )
        return result_type(1);
      else
        return it->second[i-s_idx];
    } else
      return result_type(1);
  }

  typedef SparseView<PixelT> prerasterize_type;
  inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const { return *this; }
  template <class DestT>
  inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }

  // Non standard stuff
  bool contains( vw::int32 i, vw::int32 j, PixelT & pixel_ref ) const {
    typename map_type::const_iterator it;
    if ( j >= vw::int32(m_data->size()) )
      return false;
    if ( (*m_data)[j].empty() )
      return false;
    it = (*m_data)[j].upper_bound(i);
    if ( it == (*m_data)[j].end() )
      return false;
    else {
      vw::int32 s_idx = it->first - it->second.size();
      if ( i < s_idx )
        return false;
      else {
        pixel_ref = it->second[i-s_idx];
        return true;
      }
    }
  }

  // Difficult insertation
  template <class InputT>
  void absorb( vw::Vector2i starting_index,
               vw::ImageViewBase<InputT> const& image ) {
    if ( starting_index[0] < 0 ||
         starting_index[1] < 0 )
      vw_throw( vw::NoImplErr() << "SparseView doesn't support insertation behind image origin.\n");
    // Building temporary data structure
    std::vector<std::list<vw::int32> > t_row_end(image.impl().rows());
    std::vector<std::list<std::vector<pixel_type> > > t_row_data(image.impl().rows());

    for ( vw::int32 r = 0; r < image.impl().rows(); r++ ) {
      std::vector<pixel_type> temp;
      bool building_segment = false;
      for ( vw::int32 c = 0; c < image.impl().cols(); c++ ) {
        if ( is_valid( image.impl()(c,r) ) )
          if ( !building_segment ) {
            building_segment = true;
            temp.push_back( image.impl()(c,r).child() );
          } else
            temp.push_back( image.impl()(c,r).child() );
        else if ( building_segment ) {
          t_row_end[r].push_back(c);
          building_segment = false;
          t_row_data[r].push_back( temp );
          temp.clear();
        }
      }
      if ( building_segment ) {
        t_row_end[r].push_back(image.impl().cols());
        t_row_data[r].push_back(temp);
        temp.clear();
      }
    }

    // Fix x offset
    for ( vw::uint32 i = 0; i < t_row_end.size(); i++ )
      for ( std::list<vw::int32>::iterator iter = t_row_end[i].begin();
            iter != t_row_end[i].end(); iter++ )
        *iter += starting_index[0];

    // Inserting into global data set
    if ( vw::int32(m_data->size()) < starting_index[1]+image.impl().rows() ) {
      vw::int32 length = starting_index[1]+image.impl().rows();
      std::cout << "Resizing to: " << length << std::endl;
      m_data->resize( length );
    }
    for ( vw::int32 t_i=0, m_i=starting_index[1];
          t_i < image.impl().rows(); t_i++, m_i++ ) {

      // Insert a strip at a time
      std::list<vw::int32>::const_iterator t_e_iter = t_row_end[t_i].begin();
      typename std::list<std::vector<pixel_type> >::const_iterator t_d_iter = t_row_data[t_i].begin();
      while ( t_e_iter != t_row_end[t_i].end() ) {
        typename map_type::iterator it = (*m_data)[m_i].lower_bound( *t_e_iter );
        if ( it == (*m_data)[m_i].end() ) { // Doesn't appear anything is this far right
          (*m_data)[m_i][*t_e_iter] = *t_d_iter;
        } else if ( it == (*m_data)[m_i].begin() ) {
          // Nothing in front of this point.
          vw::int32 m_start_idx = it->first - it->second.size();
          if ( *t_e_iter > m_start_idx )
            vw_throw( vw::NoImplErr() << "SparseView at this time doesn't allow insert over existing data.\n");
          else if ( *t_e_iter == m_start_idx ) // Append to front
            it->second.insert( it->second.begin(),
                               t_d_iter->begin(),
                               t_d_iter->end() );
          else // Whole new key
            (*m_data)[m_i][*t_e_iter] = *t_d_iter;
        } else {
          // Somewhere in the middle
          typename map_type::const_iterator prev_it = it;
          prev_it--;
          vw::int32 t_start_idx = *t_e_iter - t_d_iter->size();
          vw::int32 m_start_idx = it->first - it->second.size();
          if ( (t_start_idx < prev_it->first) ||
               (*t_e_iter > m_start_idx) )
            vw_throw( vw::NoImplErr() << "SparseView at this time doesn't allow insert over existing data.\n");
          else if ( *t_e_iter == m_start_idx ) // Append to front
            it->second.insert( it->second.begin(),
                               t_d_iter->begin(),
                               t_d_iter->end() );
          else // Whole new key
            (*m_data)[m_i][*t_e_iter] = *t_d_iter;
        }

        t_e_iter++;
        t_d_iter++;
      }
    }
  }

  // Debug structure
  void print_structure( void ) const {
    std::cout << "SparseView Structure:\n";
    for ( vw::uint32 i = 0; i < m_data->size(); i++ ) {
      std::cout << i << " | ";
      for ( typename map_type::const_iterator it = (*m_data)[i].begin();
            it != (*m_data)[i].end(); it++ ) {
        vw::int32 start = it->first - it->second.size();
        std::cout << "(" << start << "->" << it->first << ")";
      }
      std::cout << "\n";
    }

  }

};

#endif//__SPARSE_IMAGE_VIEW_H__
