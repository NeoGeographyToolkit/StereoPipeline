// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file BlobIndexThreaded.cc
///

#include <asp/Core/BlobIndexThreaded.h>

using namespace vw;
using namespace blob;

// BLOB COMPRESSED
///////////////////////////////////////////

// shift_x(..)
//----------------------------
void BlobCompressed::shift_x( int32 const& value ) {
  for ( uint32 i = 0; i < m_row_end.size(); i++ )
    for ( std::list<int32>::iterator iter_start = m_row_start[i].begin(),
            iter_end = m_row_end[i].begin(); iter_start != m_row_start[i].end();
          iter_start++, iter_end++ ) {
      *iter_start -= value;
      *iter_end -= value;
    }
  m_min[0] += value;
}

// refactor()
//----------------------------
void BlobCompressed::refactor( void ) {
  for ( uint32 i = 0; i < m_row_end.size(); i++ ) {
    std::list<int32>::iterator iter_n_start = m_row_start[i].begin(),
      iter_end = m_row_end[i].begin();
    iter_n_start++;
    while ( iter_n_start != m_row_start[i].end() ) {
      if ( *iter_n_start == *iter_end ) {
        iter_n_start = m_row_start[i].erase( iter_n_start );
        iter_end = m_row_end[i].erase( iter_end );
      } else {
        iter_n_start++;
        iter_end++;
      }
    }
  }
}

// size()
//----------------------------
int32 BlobCompressed::size() const {
  int32 sum = 0;
  for ( uint32 r = 0; r < m_row_end.size(); r++ )
    for ( std::list<int32>::const_iterator iter_start = m_row_start[r].begin(),
            iter_end = m_row_end[r].begin(); iter_start != m_row_start[r].end();
          iter_start++, iter_end++ )
      sum += *iter_end - *iter_start;
  return sum;
}

// bounding_box()
//----------------------------
BBox2i BlobCompressed::bounding_box() const {
  BBox2i bbox;
  bbox.min() = m_min;
  int32 max_col = 0;
  for ( uint32 i = 0; i < m_row_end.size(); i++ )
    for ( std::list<int32>::const_iterator iter = m_row_end[i].begin();
          iter != m_row_end[i].end(); iter++ )
      if ( *iter > max_col )
        max_col = *iter;
  bbox.max() = Vector2i(m_min.x()+max_col,m_min.y()+m_row_start.size());
  return bbox;
}

// is_on_right(..) (8 connected)
//---------------------------
bool BlobCompressed::is_on_right( BlobCompressed const& right ) const {
  int32 y_offset = m_min.y()-right.min().y()-1;
  // Starting r_i on the index above
  for( int32 i = 0, r_i = y_offset;
       (i < int32(m_row_end.size()))&&(r_i < right.num_rows());
       i++, r_i++ ) {
    if ( r_i >= 0 && r_i < int32(right.num_rows()) )
      if ( m_row_end[i].back() + m_min.x() ==
           right.start(r_i).front()+right.min().x() )
        return true;
    if ( r_i+1 >= 0 && r_i+1 < int32(right.num_rows()) )
      if ( m_row_end[i].back() + m_min.x() ==
           right.start(r_i+1).front()+right.min().x() )
        return true;
    if ( r_i+2 >= 0 && r_i+2 < int32(right.num_rows()) )
      if ( m_row_end[i].back() + m_min.x() ==
           right.start(r_i+2).front()+right.min().x() )
        return true;
  }
  return false;
}

// is_on_bottom(..) (8 connected)
//---------------------------
bool BlobCompressed::is_on_bottom( BlobCompressed const& bottom ) const {
  if ( bottom.min().y() != m_min.y()+int32(m_row_start.size()) )
    return false;
  // Are the rows connected ?
  for ( std::list<int32>::const_iterator top_start = m_row_start.back().begin(),
          top_end = m_row_end.back().begin(); top_start != m_row_start.back().end();
        top_start++, top_end++ )
    for ( std::list<int32>::const_iterator bot_start = bottom.start(0).begin(),
            bot_end = bottom.end(0).begin(); bot_start != bottom.start(0).end();
          bot_start++, bot_end++ ) {
      if ( (*top_end+m_min.x() >= *bot_start + bottom.min().x()) &&
           (*top_start+m_min.x() <= *bot_end+bottom.min().x()) )
        return true;
    }
  return false;
}

// add_row(..)
//----------------------------
void BlobCompressed::add_row( Vector2i const& start,
                              int const& width ) {
  if ( m_min[0] == -1 ) {
    // First insertion
    m_min = start;
    m_row_start.resize(1);
    m_row_end.resize(1);
    m_row_start[0].push_back(0);
    m_row_end[0].push_back(width);
  } else { // If not first
    if ( !( (start.y() == m_min.y()+int32(m_row_start.size()) ) ||
            (start.y() == m_min.y()+int32(m_row_start.size())-1) ) )
      vw_throw(vw::NoImplErr() << "Add_row expects rows to be added in order.\n" );
    if ( start.y() == m_min.y()+int32(m_row_start.size()) ) {
       m_row_start.resize(m_row_start.size()+1);
       m_row_end.resize(m_row_start.size());
    }

    if ( (start.x() < m_row_start.back().back()+m_min.x()) &&
         !m_row_start.back().empty() ) {
      vw_out() << "start: " << start << " w: " << width << std::endl;
      vw_out() << "front() = " <<  m_row_start.back().back() << std::endl;
      vw_out() << "min.x() << " << m_min.x() << std::endl;
      vw_throw(vw::NoImplErr() << "It appears a segment is trying to be inserted out of order.\n" );
    }

    m_row_start.back().push_back(start.x()-m_min.x());
    m_row_end.back().push_back(start.x()-m_min.x()+width);
    if ( m_min.x() > start.x() ) {
      int32 offset = start.x()-m_min.x();
      this->shift_x(offset); // I guess this is really only need at the end
    }

  }
}

// absorb(..)
//-----------------------------
void BlobCompressed::absorb( BlobCompressed const& victim ) {

  // First check to see if I'm empty
  if ( m_row_start.size() == 0 ) {
    m_min = victim.min();
    for ( int i = 0; i < victim.num_rows(); i++ ) {
      m_row_start.push_back( victim.start(i) );
      m_row_end.push_back( victim.end(i) );
    }
    return;
  }

  // Normal operations
  int y_offset =- m_min.y() + victim.min().y();
  for ( int i = 0; i < victim.num_rows(); i++ ) {
    // Index i is in terms of the victim
    // m_index is in terms of this
    int m_index = y_offset+i;

    // Checking if can insert
    if ( m_index >= 0 && m_index < int32(m_row_start.size()) ) {

      // Inserting victim's connected rows in singletons at a time
      // become sometimes they connect like zippers.
      for ( std::list<int32>::const_iterator v_singleton_start = victim.start(i).begin(),
              v_singleton_end = victim.end(i).begin(); v_singleton_start != victim.start(i).end();
            v_singleton_start++, v_singleton_end++ ) {

        std::list<int32>::iterator start_insertion_point = m_row_start[m_index].begin(),
          end_insertion_point = m_row_end[m_index].begin(),
          start_lookahead = m_row_start[m_index].begin();
        bool found_good_spot = false;
        if ( m_row_start[m_index].empty()  )
          found_good_spot = true;
        else if ( *v_singleton_end+victim.min().x()-m_min.x() <=
                  m_row_start[m_index].front() )
          found_good_spot = true;
        else {
          start_lookahead++;
        }
        while ( start_lookahead != m_row_start[m_index].end() &&
                !found_good_spot ) {
          if ( (*v_singleton_start+victim.min().x()-m_min.x() >=
                *end_insertion_point ) &&
               (*v_singleton_end+victim.min().x()-m_min.x() <=
                *start_lookahead ) ) {
            found_good_spot = true;
          }
          start_insertion_point++;
          end_insertion_point++;
          start_lookahead++;
        }
        // If insert on end
        if ( (start_lookahead == m_row_start[m_index].end()) &&
             !found_good_spot ) {
          if ( *v_singleton_start+victim.min().x()-m_min.x() >=
               *end_insertion_point ) {
            found_good_spot = true;
            start_insertion_point++;
            end_insertion_point++;
          }
        }
        // Checking for error and debug
        if ( !found_good_spot ) {
          vw_out() << "index =  " << m_index << std::endl;
          vw_out() << "Have: min[" << m_min << "] ";
          for ( std::list<int32>::const_iterator m_i_str = m_row_start[m_index].begin(),
                  m_i_end = m_row_end[m_index].begin();
                m_i_str != m_row_start[m_index].end(); m_i_str++, m_i_end++ )
            vw_out() << "(" << *m_i_str << "-" << *m_i_end << ")";
          vw_out() << "\nTrying to insert singleton: (" << *v_singleton_start << "-"
                    << *v_singleton_end << ")\n";

          vw_throw( vw::NoImplErr() << "BlobCompressed: Seems to be inserting an overlapping blob compressed object.\n" );
        }
        // Inserting ( but apply offset fix first)
        // Is there a simpler way of inserting, without making an intermediate
        { // Start
          std::list<int32> temp;
          temp.push_back( *v_singleton_start+victim.min().x()-m_min.x() );
          m_row_start[m_index].insert(start_insertion_point,
                                      temp.begin(),temp.end() );
        }
        { // End
          std::list<int32> temp;
          temp.push_back( *v_singleton_end+victim.min().x()-m_min.x() );
          m_row_end[m_index].insert(end_insertion_point,
                                    temp.begin(), temp.end() );
        }
      }
    }
  }
  // Now handle the rows that didn't previously exist
  // > Insert any on top?
  if ( victim.min().y() < m_min.y() ) {
    std::vector<std::list<int32> > temp_start;
    std::vector<std::list<int32> > temp_end;
    for ( int v_i = 0; v_i < m_min.y()-victim.min().y(); v_i++ ) {
      if ( v_i < victim.num_rows() ) {
        temp_start.push_back(victim.start(v_i));
        temp_end.push_back(victim.end(v_i));
        for ( std::list<int32>::iterator i_start = temp_start.back().begin(),
                i_stop = temp_end.back().begin();
              i_start != temp_start.back().end(); i_start++, i_stop++ ) {
          *i_start += victim.min().x()-m_min.x();
          *i_stop += victim.min().x()-m_min.x();
        }
      } else {
        // Blank filling. This happens when building a blob from many
        // that happen out of connection order. Think a U rotated 90
        // to the left.
        temp_start.push_back(std::list<int32>());
        temp_end.push_back(std::list<int32>());
      }
    }
    m_row_start.insert( m_row_start.begin(),
                        temp_start.begin(),
                        temp_start.end() );
    m_row_end.insert( m_row_end.begin(),
                      temp_end.begin(),
                      temp_end.end() );
    m_min.y()=victim.min().y();
  }
  // > Insert any on bottom?
  if ( victim.num_rows()+victim.min().y() >
       m_min.y() + int32(m_row_end.size()) ) {
    std::vector<std::list<int32> > temp_start;
    std::vector<std::list<int32> > temp_end;
    for ( int v_i = m_row_end.size()+m_min.y()-victim.min().y();
          v_i < victim.num_rows(); v_i++ ) {
      if ( v_i < 0 ) {
        // Blank filling
        temp_start.push_back(std::list<int32>());
        temp_end.push_back(std::list<int32>());
      } else {
        temp_start.push_back( victim.start(v_i) );
        temp_end.push_back( victim.end(v_i) );
        for ( std::list<int32>::iterator i_start = temp_start.back().begin(),
                i_stop = temp_end.back().begin();
              i_start != temp_start.back().end(); i_start++, i_stop++ ) {
          *i_start += victim.min().x()-m_min.x();
          *i_stop += victim.min().x()-m_min.x();
        }
      }
    }
    m_row_start.insert( m_row_start.end(),
                        temp_start.begin(),
                        temp_start.end() );
    m_row_end.insert( m_row_end.end(),
                      temp_end.begin(),
                      temp_end.end() );
  }
  // Recalculate min.x()
  int32 lowest_value = 0;
  for ( uint32 i = 0; i < m_row_start.size(); i++ ) {
    if ( m_row_start[i].front() < lowest_value )
      lowest_value = m_row_start[i].front();
  }
  this->shift_x( lowest_value );
  this->refactor();
}

// decompress(..)
//-------------------------
void BlobCompressed::decompress( std::list<Vector2i>& output ) const {
  output.clear();
  for (int32 r = 0; r < int32(m_row_end.size()); r++ )
    for ( std::list<int32>::const_iterator iter_start = m_row_start[r].begin(),
            iter_end = m_row_end[r].begin(); iter_start != m_row_start[r].end();
          iter_start++, iter_end++ )
      for ( int c = *iter_start; c < *iter_end; c++ )
        output.push_back( Vector2i(c,r)+m_min );
}

// BLOB INDEX CUSTOM
////////////////////////////////////////////
// ( unfortunately they're templated )

// BLOB INDEX TASK
////////////////////////////////////////////
// ( unfortunately they're templated )

// BLOB INDEX THREADED
////////////////////////////////////////////

// consolidate( .. )
//----------------------------
void BlobIndexThreaded::consolidate( Vector2i const& image_size,
                                     Vector2i const& proc_block_size ) {
  // Working out divisors
  uint32 x_divisions = uint32(ceil( float(image_size[0])/float(proc_block_size[0]) ));
  uint32 y_divisions = uint32(ceil( float(image_size[1])/float(proc_block_size[1]) ));
  std::vector<uint32> x_div, y_div;
  for ( uint32 i = 1; i < x_divisions; i++ )
    x_div.push_back( proc_block_size[0]*i );
  for ( uint32 i = 1; i < y_divisions; i++ )
    y_div.push_back( proc_block_size[1]*i );

  // Set up graph
  typedef boost::adjacency_list<boost::vecS,boost::vecS,boost::undirectedS> Graph;
  Graph connections;
  add_edge(m_blob_bbox.size()-1,m_blob_bbox.size()-1,connections);

  // Check for bbox on the division line
  // --> x div
  for ( uint32 i = 0; i < x_div.size(); i++ ) {
    std::vector<uint32> left_side;
    std::vector<uint32> right_side;
    for ( uint32 b_i = 0; b_i < m_blob_bbox.size(); b_i++ )
      if ( m_blob_bbox[b_i].max().x() == int32(x_div[i]) )
        left_side.push_back( b_i );
      else if ( m_blob_bbox[b_i].min().x() == int32(x_div[i]) )
        right_side.push_back( b_i );

    // Cool, work which boxes are side by side
    for ( uint32 l_i = 0; l_i < left_side.size(); l_i++ )
      for ( uint32 r_i = 0; r_i < right_side.size(); r_i++ ) {
        uint32 l = left_side[l_i];
        uint32 r = right_side[r_i];
        if ( ( m_blob_bbox[l].max().y()+1 >= m_blob_bbox[r].min().y() ) &&
             ( m_blob_bbox[r].max().y()+1 >= m_blob_bbox[l].min().y() ) )
          if ( m_c_blob[l].is_on_right(m_c_blob[r]) )
            add_edge(l,r,connections);
      }
  }
  // --> y div
  for ( uint32 i = 0; i < y_div.size(); i++ ) {
    std::vector<uint32> up_side;
    std::vector<uint32> down_side;
    for ( uint32 b_i = 0; b_i < m_blob_bbox.size(); b_i++ )
      if ( m_blob_bbox[b_i].max().y() == int32(y_div[i]) )
        up_side.push_back( b_i );
      else if ( m_blob_bbox[b_i].min().y() == int32(y_div[i]) )
        down_side.push_back( b_i );

    // Work out which line up
    for ( uint32 u_i = 0; u_i < up_side.size(); u_i++ )
      for ( uint32 d_i = 0; d_i < down_side.size(); d_i++ ) {
        uint32 u = up_side[u_i];
        uint32 d = down_side[d_i];
        if ( ( m_blob_bbox[u].max().x()+1 >= m_blob_bbox[d].min().x() ) &&
             ( m_blob_bbox[d].max().x()+1 >= m_blob_bbox[u].min().x() ) )
          if ( m_c_blob[u].is_on_bottom(m_c_blob[d]) )
            add_edge(u,d,connections);
      }
  }

  { // Reorder on graph connections
    std::vector<uint32> component(boost::num_vertices(connections));
    int final_num = boost::connected_components(connections,&component[0]);

    std::vector<BlobCompressed> blob_temp(final_num);
    for ( uint32 i = 0; i < component.size(); i++ ) {
      uint32 dest = component[i];
      blob_temp[dest].absorb( m_c_blob[i] );
    }
    m_c_blob = blob_temp;
  }
  if ( m_max_area > 0 ) {
    std::vector<BlobCompressed> blob_temp;
    for ( uint32 i = 0; i < m_c_blob.size(); i++ )
      if ( m_c_blob[i].size() < m_max_area )
        blob_temp.push_back( m_c_blob[i] );
    m_c_blob = blob_temp;
  }
  // Rebuild bounding boxes
  m_blob_bbox.clear();
  for ( uint32 i = 0; i < m_c_blob.size(); i++ )
    m_blob_bbox.push_back( m_c_blob[i].bounding_box() );
}
