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

#ifndef __BLOB_INDEX_THREADED_H__
#define __BLOB_INDEX_THREADED_H__

// VW
#include <vw/Core/Log.h>
#include <vw/Core/Thread.h>
#include <vw/Core/ThreadPool.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Image/AlgorithmFunctions.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/PixelAccessors.h>

// Standard
#include <vector>
#include <deque>
#include <list>
#include <ostream>

// Boost
#include <boost/noncopyable.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graph_selectors.hpp>

// BlobIndex (Multi) Threaded
///////////////////////////////////////

// This code is a custom write up of the blob_index function from VW. It has several
// different features.
// --> Multithread (as expected)
// --> Lower Memory Impact
//     via a new internal compressed format
// --> Allows for limiting on size.

// TODO: Move to vision workbench and replace the old blob function.

namespace blob {

  // Blob Compressed
  ////////////////////////////////////
  // A nice way to describe a blob,
  // but reducing our memory foot print
  class BlobCompressed {
    // This describes a blob as lines of rows
    // to reduce the memory foot print
    vw::Vector2i m_min;
    std::vector< std::list<vw::int32> > m_row_start; // assumed to be ordered
    std::vector< std::list<vw::int32> > m_row_end;

    void shift_x ( vw::int32 const& value );
    void refactor();

  public:
    BlobCompressed( vw::Vector2i const& top_left,
                    std::vector<std::list<vw::int32> > const& row_start,
                    std::vector<std::list<vw::int32> > const& row_end );
    BlobCompressed();

    // Standard Access point
    vw::Vector2i const& min() const;
    vw::Vector2i      & min();

    std::list<vw::int32> const& start( vw::uint32 const& index ) const;
    std::list<vw::int32> const& end  ( vw::uint32 const& index ) const;

    vw::int32 num_rows() const;
    vw::int32 size    () const; // Please use sparingly

    vw::BBox2i bounding_box() const;
    bool intersects( vw::BBox2i const& input ) const;

    // Specific conditionals used by BlobIndexThreaded
    bool is_on_right ( BlobCompressed const& right  ) const;
    bool is_on_bottom( BlobCompressed const& bottom ) const;

    // Append a row (since these guys are built a row at a time )
    void add_row( vw::Vector2i const& start, int const& width );
    // Use to expand this blob into a non overlapped area
    void absorb( BlobCompressed const& victim );
    // Dump listing of every pixel used
    void decompress( std::list<vw::Vector2i>& output ) const;
    // Print internal data
    void print() const;
  };

  // Blob Index Custom
  ////////////////////////////////////
  /// A different version of Blob index that uses the compressed format and the new options
  class BlobIndexCustom {
    std::vector<BlobCompressed> m_c_blob;
    uint m_blob_count;

  public:
    // Constructor performs processing
    template <class SourceT>
    BlobIndexCustom( vw::ImageViewBase<SourceT> const& src,
                     vw::ImageView<vw::uint32>& dst,
                     uint /*max_area*/=0 ) {

      if ( src.impl().planes() > 1 )
        vw_throw( vw::NoImplErr()
                  << "Blob index currently only works with 2D images." );
      dst.set_size( src.impl().cols(),
                    src.impl().rows() );
      fill(dst,0);

      // Initialize Graph used to pair blobs
      typedef boost::adjacency_list<boost::vecS,boost::vecS,boost::undirectedS> Graph;
      Graph connections;
      m_blob_count=1;

      { // Initial Pass
        typename SourceT::pixel_accessor s_acc = src.impl().origin();
        typename SourceT::pixel_accessor p_s_acc = src.impl().origin(); // previous
        typename vw::ImageView<vw::uint32>::pixel_accessor d_acc = dst.origin();
        typename vw::ImageView<vw::uint32>::pixel_accessor p_d_acc = dst.origin(); // previous

        // Top Corner
        if ( is_valid(*s_acc) ) {
          *d_acc = m_blob_count;
          m_blob_count++;
        }

        // Top Row
        s_acc.next_col();
        d_acc.next_col();
        for ( vw::int32 i = 1; i < dst.cols(); i++ ) {
          if ( is_valid(*s_acc) ) {
            if ( is_valid(*p_s_acc) ) {
              *d_acc = *p_d_acc;
            } else {
              *d_acc = m_blob_count;
              m_blob_count++;
            }
          }
          s_acc.next_col();
          d_acc.next_col();
          p_s_acc.next_col();
          p_d_acc.next_col();
        }
      }

      { // Everything else (9 connected)
        typename SourceT::pixel_accessor s_acc_row = src.impl().origin();
        typename vw::ImageView<vw::uint32>::pixel_accessor d_acc_row = dst.origin();
        s_acc_row.advance(0,1);
        d_acc_row.advance(0,1);

        for (int j = dst.rows()-1; j; --j ) { // Not for indexing
          typename SourceT::pixel_accessor s_acc = s_acc_row;
          typename SourceT::pixel_accessor p_s_acc = s_acc_row;
          typename vw::ImageView<vw::uint32>::pixel_accessor d_acc = d_acc_row;
          typename vw::ImageView<vw::uint32>::pixel_accessor p_d_acc = d_acc_row;

          // Process
          for ( int i = dst.cols(); i; --i ) {
            if ( is_valid(*s_acc) ) {
              if ( i != dst.cols() ) {
                // Left
                p_s_acc.advance(-1,0);
                p_d_acc.advance(-1,0);
                if ( is_valid(*p_s_acc) ) {
                  if ( (*d_acc != 0) && (*d_acc != *p_d_acc) )
                    boost::add_edge(*p_d_acc,*d_acc,connections);
                  else
                    *d_acc = *p_d_acc;
                }
                // Upper Left
                p_s_acc.advance(0,-1);
                p_d_acc.advance(0,-1);
                if ( is_valid(*p_s_acc) ) {
                  if ( (*d_acc != 0) && (*d_acc != *p_d_acc) )
                    boost::add_edge(*p_d_acc,*d_acc,connections);
                  else
                    *d_acc = *p_d_acc;
                }
              } else {
                p_s_acc.advance(-1,-1);
                p_d_acc.advance(-1,-1);
              }
              // Upper
              p_s_acc.advance(1,0);
              p_d_acc.advance(1,0);
              if ( is_valid(*p_s_acc) ) {
                if ( (*d_acc != 0) && (*d_acc != *p_d_acc) )
                  boost::add_edge(*p_d_acc,*d_acc,connections);
                else
                  *d_acc = *p_d_acc;
              }
              // Upper Right
              p_s_acc.advance(1,0);
              p_d_acc.advance(1,0);
              if ( i != 1 ) {
                if ( is_valid(*p_s_acc) ) {
                  if ( (*d_acc != 0) && (*d_acc != *p_d_acc) )
                    boost::add_edge(*p_d_acc,*d_acc,connections);
                  else
                    *d_acc = *p_d_acc;
                }
              }
              // Setting if not
              p_s_acc.advance(-1,1);
              p_d_acc.advance(-1,1);
              if ( *d_acc == 0 ) {
                *d_acc = m_blob_count;
                m_blob_count++;
              }
            }
            s_acc.next_col();
            p_s_acc.next_col();
            d_acc.next_col();
            p_d_acc.next_col();
          } // end row process

          s_acc_row.next_row();
          d_acc_row.next_row();
        }
      }

      // Making sure connections has vertices for all indexes made
      add_edge(m_blob_count-1,m_blob_count-1,connections);
      std::vector<vw::uint32> component(boost::num_vertices(connections));
      m_blob_count = boost::connected_components(connections, &component[0])-1;
      m_c_blob.resize(m_blob_count);

      { // Update index map to optimal numbering
        vw::ImageView<vw::uint32>::pixel_accessor p_d_acc = dst.origin(); // previous

        for ( vw::int32 r = 0; r < dst.rows(); r++ ) {
          vw::ImageView<vw::uint32>::pixel_accessor d_acc = p_d_acc;
          bool building_segment=false;
          vw::int32 start_c = 0;
          vw::uint32 index = 0;
          for ( vw::int32 c = 0; c < dst.cols(); c++ ) {
            if ( (*d_acc) != 0 ) {
              if ( building_segment && (index != component[*d_acc]) ) {
                vw_throw(vw::LogicErr() << "Insert seems wrong.\n");
                // I believe the way it's processed this shouldn't
                // happen
              } else if (!building_segment) {
                building_segment = true;
                index = component[*d_acc];
                start_c = c;
              }
            } else if ( building_segment ) {
              // Looks like we're finishing up here
              building_segment = false;
              m_c_blob[index-1].add_row( vw::Vector2i(start_c,r),
                                         c-start_c );
            }
            d_acc.next_col();
          }
          if ( building_segment ) {
            m_c_blob[index-1].add_row( vw::Vector2i(start_c,r),
                                       dst.cols()-start_c );
          }
          p_d_acc.next_row();
        }
      }

    }

    // Access points to intersting information
    vw::uint32 num_blobs() const;

    // Access to blobs
    BlobCompressed const& blob( vw::uint32 const& index ) const;
  };

  // Blob Index Task
  /////////////////////////////////////
  /// A task wrapper to allow threading
  template <class SourceT>
  class BlobIndexTask : public vw::Task, private boost::noncopyable {

    vw::ImageViewBase<SourceT> const& m_view;
    vw::BBox2i const& m_bbox;
    vw::Mutex&        m_append_mutex;

    std::deque<BlobCompressed> &m_c_blob; // reference to global
    std::deque<vw::BBox2i>     &m_blob_bbox;
    int m_id;
    int m_max_area;
  public:
    BlobIndexTask( vw::ImageViewBase<SourceT> const& view,
                   vw::BBox2i const& bbox, vw::Mutex &mutex,
                   std::deque<BlobCompressed> & blobs,
                   std::deque<vw::BBox2i> & blob_boxes,
                   int const& id, int const& max_area ) :
    m_view(view), m_bbox(bbox), m_append_mutex(mutex),
      m_c_blob(blobs), m_blob_bbox(blob_boxes), m_id(id), m_max_area(max_area) {}

    void operator()() {
      vw::Stopwatch sw;
      sw.start();
      vw::ImageView<vw::uint32> index_image(m_bbox.width(),
                                            m_bbox.height() );

      // Render so threads don't wait on each other
      vw::ImageView<typename SourceT::pixel_type> cropped_copy = crop(m_view,m_bbox);
      // Decided only to do trimming in the global perspective. This
      // avoids weird edge effects.
      BlobIndexCustom bindex( cropped_copy, index_image);

      // Build local bboxes
      std::vector<vw::BBox2i> local_bboxes( bindex.num_blobs() );
      for ( vw::uint32 i = 0; i < bindex.num_blobs(); i++ ) {
        local_bboxes[i] = bindex.blob(i).bounding_box() + m_bbox.min();
      }

      { // Append results (single thread)
        vw::Mutex::Lock lock(m_append_mutex);
        for ( uint i = 0; i < local_bboxes.size(); i++ ) {
          m_c_blob.push_back( bindex.blob(i) );
          m_c_blob.back().min() += m_bbox.min(); // Fix offset
          m_blob_bbox.push_back( local_bboxes[i] );
        }
      }

      sw.stop();
      vw_out(vw::VerboseDebugMessage,"inpaint") << "Task " << m_id << ": finished, " << sw.elapsed_seconds() << "s\n";
    }
  };
} // end namespace blob

// Blob Index Threaded
///////////////////////////////////
/// Performs Blob Index using all threads and a minimal amount of memory
class BlobIndexThreaded {

  std::deque<vw::BBox2i>           m_blob_bbox;
  std::deque<blob::BlobCompressed> m_c_blob;

  vw::Mutex m_insert_mutex;
  int m_max_area;
  int m_tile_size;

  // Tasks might section a blob in half.
  // This will match them
  void consolidate( vw::Vector2i const& image_size,
                    vw::Vector2i const& proc_block_size );

 public:
  // Constructor does most of the processing work
  template <class SourceT>
  BlobIndexThreaded( vw::ImageViewBase<SourceT> const& src,
                     vw::int32 const& max_area = 0,
                     vw::int32 const& tile_size
                     = vw::vw_settings().default_tile_size(),
                     vw::int32 const& num_threads
                     = vw::vw_settings().default_num_threads()
                     )
    : m_max_area(max_area), m_tile_size(tile_size) {
    
    std::vector<vw::BBox2i> bboxes =
      image_blocks( src.impl(), m_tile_size, m_tile_size );
    // User needs to remember to give a pixel mask'd input
    typedef blob::BlobIndexTask<SourceT> task_type;
    if (bboxes.size() > 1){
      vw::Stopwatch sw;
      sw.start();
      vw::FifoWorkQueue queue(num_threads);
      
      for ( size_t i = 0; i < bboxes.size(); ++i ) {
        boost::shared_ptr<task_type> task(new task_type(src, bboxes[i],
                                                        m_insert_mutex,
                                                        m_c_blob, m_blob_bbox,
                                                        i, m_max_area ));
        queue.add_task(task);
      }
      queue.join_all();
      
      sw.stop();
      vw_out(vw::DebugMessage,"inpaint") << "Blob detection took " << sw.elapsed_seconds() << "s\n";
      consolidate( vw::Vector2i( src.impl().cols(), src.impl().rows() ),
                   vw::Vector2i( m_tile_size, m_tile_size ) );
    }else if (bboxes.size() == 1){
      // This is a special case when we want to fill in holes
      // in a single small image.
      boost::shared_ptr<task_type> task(new task_type(src, bboxes[0],
                                                      m_insert_mutex,
                                                      m_c_blob, m_blob_bbox,
                                                      0, m_max_area ));
      (*task.get())();
    }
    

    // Cull blobs that are too big.
    if ( m_max_area > 0 ) {
      for ( std::deque<blob::BlobCompressed>::iterator iter = m_c_blob.begin();
            iter != m_c_blob.end(); iter++ ) {
        if ( iter->size() > m_max_area ) {
          iter = m_c_blob.erase( iter );
          iter--;
        }
      }
    }
  }

  /// Wipe blobs bigger than this size.
  void wipe_big_blobs(int max_size){

    // Keep the bounding boxes up-to-date.
    m_blob_bbox.clear();
    
    for ( std::deque<blob::BlobCompressed>::iterator iter = m_c_blob.begin();
          iter != m_c_blob.end(); iter++ ) {

      vw::BBox2i blob_bbox = iter->bounding_box();
      
      if ( blob_bbox.width() > max_size ||
           blob_bbox.height() > max_size) {
        iter = m_c_blob.erase( iter );
        iter--;
      }else{
        m_blob_bbox.push_back(blob_bbox);
      }
      
    }
    
  }
  
  // Access for the users
  vw::uint32 num_blobs() const;
  /// ?
  void blob( vw::uint32 const& index,
             std::list<vw::Vector2i>& output ) const;
  /// ?
  blob::BlobCompressed const& compressed_blob( vw::uint32 const& index ) const;

  typedef std::deque<blob::BlobCompressed>::iterator             blob_iterator;
  typedef std::deque<blob::BlobCompressed>::const_iterator const_blob_iterator;
        blob_iterator begin();
  const_blob_iterator begin() const;
        blob_iterator end();
  const_blob_iterator end() const;

  vw::BBox2i const& blob_bbox( vw::uint32 const& index ) const;
  typedef std::deque<vw::BBox2i>::iterator             bbox_iterator;
  typedef std::deque<vw::BBox2i>::const_iterator const_bbox_iterator;
        bbox_iterator bbox_begin();
  const_bbox_iterator bbox_begin() const;
        bbox_iterator bbox_end();
  const_bbox_iterator bbox_end() const;
};

#endif//__BLOB_INDEX_THREADED_H__
